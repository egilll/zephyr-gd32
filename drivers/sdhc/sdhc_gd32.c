/*
 * Copyright (c) 2026 The Zephyr Project Contributors
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT gd_gd32_sdhc

/*
 * Notes:
 *   - This driver differs from GD32's sample driver in that it doesn't rely on
 *     the DMA "full transfer finish" flag as the marker for data transfers being complete.
 *     That flag will often fail to ever be set especially when the memory bus is under a
 *     significant load. We instead rely on the SDIO peripheral's DTEND/DTBLKEND to signal
 *     complet
 */

#include <zephyr/kernel.h>
#include <zephyr/devicetree.h>
#include <zephyr/device.h>
#include <zephyr/drivers/sdhc.h>
#include <zephyr/sd/sd_spec.h>
#include <zephyr/sys/util.h>
#include <zephyr/cache.h>
#include <zephyr/drivers/clock_control.h>
#include <zephyr/drivers/clock_control/gd32.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/pinctrl.h>
#include <zephyr/drivers/dma.h>
#include <zephyr/drivers/reset.h>
#include <zephyr/mem_mgmt/mem_attr.h>
#include <zephyr/logging/log.h>
#include <zephyr/irq.h>
#include <zephyr/sys/atomic.h>

/* GD32 HAL (provided by the SoC HAL module) */
#include <gd32_sdio.h>
#include <gd32_dma.h>

LOG_MODULE_REGISTER(sdhc_gd32, CONFIG_SDHC_LOG_LEVEL);

#if ((CONFIG_SDHC_BUFFER_ALIGNMENT % 4) == 0)
#define GD32_SDHC_DMA_BURST_LENGTH 4U
#else
#define GD32_SDHC_DMA_BURST_LENGTH 1U
#endif

/* SDIOCLK input to the SDIO peripheral on GD32F4xx is 48MHz. */
#define GD32_SDIOCLK_HZ       48000000U
#define GD32_SDIO_DATATIMEOUT 0xFFFFFFFFU

/* Optional extra pinctrl states to match bus speed */
#define PINCTRL_STATE_SLOW PINCTRL_STATE_PRIV_START
#define PINCTRL_STATE_FAST (PINCTRL_STATE_PRIV_START + 1U)

#define GD32_SDIO_INT_CLEAR_ALL                                                                    \
	(SDIO_INT_FLAG_CCRCERR | SDIO_INT_FLAG_DTCRCERR | SDIO_INT_FLAG_CMDTMOUT |                 \
	 SDIO_INT_FLAG_DTTMOUT | SDIO_INT_FLAG_TXURE | SDIO_INT_FLAG_RXORE |                       \
	 SDIO_INT_FLAG_CMDRECV | SDIO_INT_FLAG_CMDSEND | SDIO_INT_FLAG_DTEND |                     \
	 SDIO_INT_FLAG_STBITE | SDIO_INT_FLAG_DTBLKEND | SDIO_INT_FLAG_SDIOINT |                   \
	 SDIO_INT_FLAG_ATAEND)

#define GD32_SDIO_INT_CLEAR_XFER                                                                   \
	(GD32_SDIO_INT_CLEAR_ALL & ~(SDIO_INT_FLAG_SDIOINT | SDIO_INT_FLAG_ATAEND))

#define GD32_SDIO_INT_CMD_MASK                                                                     \
	(SDIO_INT_CMDRECV | SDIO_INT_CMDSEND | SDIO_INT_CCRCERR | SDIO_INT_CMDTMOUT |              \
	 SDIO_INT_STBITE)

#define GD32_SDIO_INT_DATA_MASK                                                                    \
	(SDIO_INT_DTEND | SDIO_INT_DTCRCERR | SDIO_INT_DTTMOUT | SDIO_INT_TXURE | SDIO_INT_RXORE | \
	 SDIO_INT_STBITE | SDIO_INT_DTBLKEND)

enum gd32_sdhc_xfer_err {
	GD32_SDHC_ERR_NONE = 0U,
	GD32_SDHC_ERR_CMD_CRC = BIT(0),
	GD32_SDHC_ERR_CMD_TIMEOUT = BIT(1),
	GD32_SDHC_ERR_DATA_CRC = BIT(2),
	GD32_SDHC_ERR_DATA_TIMEOUT = BIT(3),
	GD32_SDHC_ERR_TX_UNDERRUN = BIT(4),
	GD32_SDHC_ERR_RX_OVERRUN = BIT(5),
	GD32_SDHC_ERR_START_BIT = BIT(6),
	GD32_SDHC_ERR_CARD_REMOVED = BIT(7),
};

struct gd32_sdhc_dma_chan {
	const struct device *dev;
	uint32_t reg;
	uint32_t channel;
#if DT_HAS_COMPAT_STATUS_OKAY(gd_gd32_dma_v1)
	uint32_t slot;
	uint8_t fifo_threshold;
#endif
};

struct gd32_sdhc_config {
	uint16_t clkid;
	const struct pinctrl_dev_config *pcfg;
	void (*irq_config_func)(const struct device *dev);

	struct gpio_dt_spec cd_gpio;
	struct gd32_sdhc_dma_chan dma;
	uint32_t fifo_addr;
	struct reset_dt_spec reset;

	uint8_t bus_width;
	uint32_t power_delay_ms;
	uint32_t min_bus_freq;
	uint32_t max_bus_freq;
	uint32_t max_current_330;
	uint32_t max_current_300;
	uint32_t max_current_180;
};

struct gd32_sdhc_data {
	struct sdhc_host_props props;
	struct sdhc_io host_io;

	struct k_mutex lock;
	struct k_sem cmd_sem;
	struct k_sem data_sem;
	struct k_sem dma_sem;

	volatile uint32_t cmd_err;
	volatile uint32_t data_err;
	volatile uint32_t data_events;
	volatile int dma_status;

	bool card_high_capacity;

	sdhc_interrupt_cb_t card_cb;
	void *card_cb_user_data;
	int enabled_sources;

	struct gpio_callback cd_cb;
	const struct device *dev;
	bool cd_present;
	atomic_t card_removed;

	uint32_t cmd13_count;
	uint32_t last_cmd13_r1;
	uint32_t last_r1_err_flags;
};

static void gd32_sdhc_abort_io(const struct device *dev);

static inline bool gd32_sdhc_is_card_removed(const struct gd32_sdhc_data *data)
{
	return atomic_get(&data->card_removed) != 0;
}

static void gd32_sdhc_signal_card_removed(struct gd32_sdhc_data *data)
{
	atomic_set(&data->card_removed, 1);
	k_sem_give(&data->cmd_sem);
	k_sem_give(&data->data_sem);
	k_sem_give(&data->dma_sem);
}

static void gd32_sdhc_signal_card_inserted(struct gd32_sdhc_data *data)
{
	atomic_set(&data->card_removed, 0);
}

static int gd32_sdhc_cd_irq_arm(const struct device *dev)
{
	struct gd32_sdhc_data *data = dev->data;
	const struct gd32_sdhc_config *cfg = dev->config;

	if (cfg->cd_gpio.port == NULL) {
		return -ENOTSUP;
	}

	int v = gpio_pin_get_dt(&cfg->cd_gpio);
	if (v < 0) {
		return v;
	}

	data->cd_present = (v > 0);
	atomic_set(&data->card_removed, data->cd_present ? 0 : 1);
	return gpio_pin_interrupt_configure_dt(&cfg->cd_gpio,
					       data->cd_present ? GPIO_INT_EDGE_TO_INACTIVE
								: GPIO_INT_EDGE_TO_ACTIVE);
}

static void gd32_cd_gpio_cb(const struct device *port, struct gpio_callback *cb,
			    gpio_port_pins_t pins)
{
	ARG_UNUSED(port);
	ARG_UNUSED(pins);

	struct gd32_sdhc_data *data = CONTAINER_OF(cb, struct gd32_sdhc_data, cd_cb);
	const struct device *dev = data->dev;
	const struct gd32_sdhc_config *cfg = dev->config;

	if ((data->enabled_sources & (SDHC_INT_INSERTED | SDHC_INT_REMOVED)) == 0) {
		return;
	}

	if (data->card_cb == NULL) {
		return;
	}

	int v = gpio_pin_get_dt(&cfg->cd_gpio);
	if (v < 0) {
		return;
	}

	bool present = (v > 0);
	if (present == data->cd_present) {
		return;
	}
	data->cd_present = present;

	if (!present) {
		gd32_sdhc_signal_card_removed(data);
	} else {
		gd32_sdhc_signal_card_inserted(data);
	}

	data->card_cb(dev, present ? SDHC_INT_INSERTED : SDHC_INT_REMOVED, data->card_cb_user_data);
	(void)gpio_pin_interrupt_configure_dt(&cfg->cd_gpio,
					      present ? GPIO_INT_EDGE_TO_INACTIVE : GPIO_INT_EDGE_TO_ACTIVE);
}

static void gd32_sdhc_dump_hw_state(const char *tag)
{
	LOG_ERR("%s: STAT=0x%08x INTEN=0x%08x CMDCTL=0x%08x DATACTL=0x%08x DATATO=0x%08x "
		"DATALEN=0x%08x DATACNT=0x%08x",
		tag, SDIO_STAT, SDIO_INTEN, SDIO_CMDCTL, SDIO_DATACTL, SDIO_DATATO, SDIO_DATALEN,
		SDIO_DATACNT);
	LOG_ERR("%s: RSPCMDIDX=0x%08x RESP0=0x%08x RESP1=0x%08x RESP2=0x%08x RESP3=0x%08x", tag,
		SDIO_RSPCMDIDX, SDIO_RESP0, SDIO_RESP1, SDIO_RESP2, SDIO_RESP3);
}

static void gd32_sdhc_log_cmd_error(uint32_t cmd_err)
{
	if ((cmd_err & GD32_SDHC_ERR_CMD_TIMEOUT) != 0U) {
		LOG_ERR("CMD timeout");
	}
	if ((cmd_err & GD32_SDHC_ERR_CMD_CRC) != 0U) {
		LOG_ERR("CMD CRC error");
	}
	if ((cmd_err & GD32_SDHC_ERR_START_BIT) != 0U) {
		LOG_ERR("CMD start-bit error");
	}
}

static void gd32_sdhc_log_data_error(uint32_t data_err, int dma_status)
{
	if (dma_status < 0) {
		LOG_ERR("DMA error: %d", dma_status);
	}
	if ((data_err & GD32_SDHC_ERR_DATA_TIMEOUT) != 0U) {
		LOG_ERR("DATA timeout");
	}
	if ((data_err & GD32_SDHC_ERR_DATA_CRC) != 0U) {
		LOG_ERR("DATA CRC error");
	}
	if ((data_err & GD32_SDHC_ERR_TX_UNDERRUN) != 0U) {
		LOG_ERR("TX underrun");
	}
	if ((data_err & GD32_SDHC_ERR_RX_OVERRUN) != 0U) {
		LOG_ERR("RX overrun");
	}
	if ((data_err & GD32_SDHC_ERR_START_BIT) != 0U) {
		LOG_ERR("DATA start-bit error");
	}
}

static const char *gd32_sdhc_primary_error_str(uint32_t cmd_err, uint32_t data_err, int dma_status)
{
	if ((cmd_err & GD32_SDHC_ERR_CARD_REMOVED) != 0U || (data_err & GD32_SDHC_ERR_CARD_REMOVED) != 0U) {
		return "Card removed";
	}
	if (dma_status < 0) {
		return "DMA error";
	}
	if ((data_err & GD32_SDHC_ERR_DATA_CRC) != 0U) {
		return "DATA CRC error";
	}
	if ((data_err & GD32_SDHC_ERR_DATA_TIMEOUT) != 0U) {
		return "DATA timeout";
	}
	if ((data_err & GD32_SDHC_ERR_TX_UNDERRUN) != 0U) {
		return "TX underrun";
	}
	if ((data_err & GD32_SDHC_ERR_RX_OVERRUN) != 0U) {
		return "RX overrun";
	}
	if ((data_err & GD32_SDHC_ERR_START_BIT) != 0U) {
		return "DATA start-bit error";
	}
	if ((cmd_err & GD32_SDHC_ERR_CMD_TIMEOUT) != 0U) {
		return "CMD timeout";
	}
	if ((cmd_err & GD32_SDHC_ERR_CMD_CRC) != 0U) {
		return "CMD CRC error";
	}
	if ((cmd_err & GD32_SDHC_ERR_START_BIT) != 0U) {
		return "CMD start-bit error";
	}
	return "I/O error";
}

static uint32_t gd32_sdhc_retry_delay_ms(uint32_t cmd_err, uint32_t data_err, int dma_status)
{
	if ((cmd_err & GD32_SDHC_ERR_CARD_REMOVED) != 0U || (data_err & GD32_SDHC_ERR_CARD_REMOVED) != 0U) {
		return 0U;
	}
	if (dma_status < 0) {
		return 10U;
	}

	if ((data_err & (GD32_SDHC_ERR_DATA_CRC | GD32_SDHC_ERR_START_BIT)) != 0U) {
		return 50U;
	}

	if (data_err != 0U) {
		return 20U;
	}

	if ((cmd_err & GD32_SDHC_ERR_CMD_TIMEOUT) != 0U) {
		return 5U;
	}

	if ((cmd_err & (GD32_SDHC_ERR_CMD_CRC | GD32_SDHC_ERR_START_BIT)) != 0U) {
		return 2U;
	}

	return 5U;
}

static void gd32_sdhc_log_r1_error_bits(uint32_t r1)
{
	uint32_t err = r1 & SD_R1_ERR_FLAGS;

	if (err == 0U) {
		return;
	}

	LOG_ERR("R1 error flags: 0x%08x (state=%u ready=%u)", err, SD_R1_CURRENT_STATE(r1),
		(r1 & SD_R1_RDY_DATA) != 0U);

	if ((err & SD_R1_OUT_OF_RANGE) != 0U) {
		LOG_ERR("R1: OUT_OF_RANGE");
	}
	if ((err & SD_R1_ADDR_ERR) != 0U) {
		LOG_ERR("R1: ADDR_ERR");
	}
	if ((err & SD_R1_BLOCK_LEN_ERR) != 0U) {
		LOG_ERR("R1: BLOCK_LEN_ERR");
	}
	if ((err & SD_R1_ERASE_SEQ_ERR) != 0U) {
		LOG_ERR("R1: ERASE_SEQ_ERR");
	}
	if ((err & SD_R1_ERASE_PARAM) != 0U) {
		LOG_ERR("R1: ERASE_PARAM");
	}
	if ((err & SD_R1_WP_VIOLATION) != 0U) {
		LOG_ERR("R1: WP_VIOLATION");
	}
	if ((err & SD_R1_CARD_LOCKED) != 0U) {
		LOG_ERR("R1: CARD_LOCKED");
	}
	if ((err & SD_R1_UNLOCK_FAIL) != 0U) {
		LOG_ERR("R1: UNLOCK_FAIL");
	}
	if ((err & SD_R1_CRC_ERR) != 0U) {
		LOG_ERR("R1: CRC_ERR");
	}
	if ((err & SD_R1_ILLEGAL_CMD) != 0U) {
		LOG_ERR("R1: ILLEGAL_CMD");
	}
	if ((err & SD_R1_ECC_FAIL) != 0U) {
		LOG_ERR("R1: ECC_FAIL");
	}
	if ((err & SD_R1_CC_ERR) != 0U) {
		LOG_ERR("R1: CC_ERR");
	}
	if ((err & SD_R1_ERR) != 0U) {
		LOG_ERR("R1: ERR");
	}
	if ((err & SD_R1_CSD_OVERWRITE) != 0U) {
		LOG_ERR("R1: CSD_OVERWRITE");
	}
	if ((err & SD_R1_ERASE_SKIP) != 0U) {
		LOG_ERR("R1: ERASE_SKIP");
	}
	if ((err & SD_R1_AUTH_ERR) != 0U) {
		LOG_ERR("R1: AUTH_ERR");
	}
}

static void gd32_sdhc_maybe_log_response(struct gd32_sdhc_data *data,
					 const struct sdhc_command *cmd)
{
	uint32_t native = cmd->response_type & SDHC_NATIVE_RESPONSE_MASK;

	if (native == SD_RSP_TYPE_NONE) {
		return;
	}

	if (native == SD_RSP_TYPE_R2) {
		LOG_DBG("CMD%u resp=%08x %08x %08x %08x", cmd->opcode, cmd->response[0],
			cmd->response[1], cmd->response[2], cmd->response[3]);
		return;
	}

	uint32_t r0 = cmd->response[0];

	if (cmd->opcode == SD_SEND_STATUS) {
		data->cmd13_count++;

		uint32_t err = r0 & SD_R1_ERR_FLAGS;
		bool log_err = (err != 0U) && (err != data->last_r1_err_flags);
		bool log_dbg = (r0 != data->last_cmd13_r1) || ((data->cmd13_count & 0x3FU) == 0U);

		if (log_err) {
			LOG_ERR("CMD13 status=0x%08x (state=%u ready=%u)", r0,
				SD_R1_CURRENT_STATE(r0), (r0 & SD_R1_RDY_DATA) != 0U);
			gd32_sdhc_log_r1_error_bits(r0);
			data->last_r1_err_flags = err;
		}

		if (log_dbg) {
			LOG_DBG("CMD13 status=0x%08x (state=%u ready=%u err=0x%08x)", r0,
				SD_R1_CURRENT_STATE(r0), (r0 & SD_R1_RDY_DATA) != 0U, err);
			data->last_cmd13_r1 = r0;
		}
		return;
	}

	LOG_DBG("CMD%u resp0=0x%08x", cmd->opcode, r0);

	if ((native == SD_RSP_TYPE_R1) || (native == SD_RSP_TYPE_R1b)) {
		uint32_t err = r0 & SD_R1_ERR_FLAGS;
		if ((err != 0U) && (err != data->last_r1_err_flags)) {
			LOG_ERR("CMD%u R1 error: 0x%08x (state=%u ready=%u)", cmd->opcode, r0,
				SD_R1_CURRENT_STATE(r0), (r0 & SD_R1_RDY_DATA) != 0U);
			gd32_sdhc_log_r1_error_bits(r0);
			data->last_r1_err_flags = err;
		}
	}
}

static inline uint32_t gd32_sdio_datablocksize_get(uint16_t bytes)
{
	uint8_t exp_val = 0U;

	while (bytes > 1U) {
		bytes >>= 1U;
		exp_val++;
	}

	return DATACTL_BLKSZ(exp_val);
}

static inline k_timeout_t gd32_ms_to_timeout(int timeout_ms)
{
	return (timeout_ms == SDHC_TIMEOUT_FOREVER) ? K_FOREVER : K_MSEC(timeout_ms);
}

static inline void gd32_sdio_sync(void)
{
	(void)SDIO_STAT;
}

static uint32_t gd32_sdhc_timeout_to_cycles(const struct gd32_sdhc_data *data, int timeout_ms)
{
	if (timeout_ms == SDHC_TIMEOUT_FOREVER) {
		return GD32_SDIO_DATATIMEOUT;
	}

	uint32_t clock_hz = data->host_io.clock;
	if (clock_hz == 0U) {
		clock_hz = SDMMC_CLOCK_400KHZ;
	}

	uint64_t cycles = ((uint64_t)timeout_ms * (uint64_t)clock_hz) / 1000U;
	if (cycles == 0U) {
		cycles = 1U;
	}

	return (cycles > UINT32_MAX) ? UINT32_MAX : (uint32_t)cycles;
}

static void gd32_sdhc_isr(const struct device *dev)
{
	struct gd32_sdhc_data *data = dev->data;
	uint32_t pending = SDIO_STAT & SDIO_INTEN;

	if (pending == 0U) {
		return;
	}

	/* SDIO card interrupt (function interrupt) */
	if ((pending & SDIO_INT_FLAG_SDIOINT) != 0U) {
		sdio_interrupt_flag_clear(SDIO_INT_FLAG_SDIOINT);
		if ((data->enabled_sources & SDHC_INT_SDIO) != 0 && data->card_cb != NULL) {
			data->card_cb(dev, SDHC_INT_SDIO, data->card_cb_user_data);
		}
		pending &= ~SDIO_INT_FLAG_SDIOINT;
	}

	/* Command completion / error */
	if ((pending & (SDIO_INT_FLAG_CCRCERR | SDIO_INT_FLAG_CMDTMOUT | SDIO_INT_FLAG_STBITE)) !=
	    0U) {
		if ((pending & SDIO_INT_FLAG_CCRCERR) != 0U) {
			data->cmd_err |= GD32_SDHC_ERR_CMD_CRC;
		}
		if ((pending & SDIO_INT_FLAG_CMDTMOUT) != 0U) {
			data->cmd_err |= GD32_SDHC_ERR_CMD_TIMEOUT;
		}
		if ((pending & SDIO_INT_FLAG_STBITE) != 0U) {
			data->cmd_err |= GD32_SDHC_ERR_START_BIT;
		}
		k_sem_give(&data->cmd_sem);
	}

	if ((pending & (SDIO_INT_FLAG_CMDRECV | SDIO_INT_FLAG_CMDSEND)) != 0U) {
		k_sem_give(&data->cmd_sem);
	}

	/* Data completion / error */
	if ((pending & (SDIO_INT_FLAG_DTCRCERR | SDIO_INT_FLAG_DTTMOUT | SDIO_INT_FLAG_TXURE |
			SDIO_INT_FLAG_RXORE | SDIO_INT_FLAG_STBITE)) != 0U) {
		if ((pending & SDIO_INT_FLAG_DTCRCERR) != 0U) {
			data->data_err |= GD32_SDHC_ERR_DATA_CRC;
		}
		if ((pending & SDIO_INT_FLAG_DTTMOUT) != 0U) {
			data->data_err |= GD32_SDHC_ERR_DATA_TIMEOUT;
		}
		if ((pending & SDIO_INT_FLAG_TXURE) != 0U) {
			data->data_err |= GD32_SDHC_ERR_TX_UNDERRUN;
		}
		if ((pending & SDIO_INT_FLAG_RXORE) != 0U) {
			data->data_err |= GD32_SDHC_ERR_RX_OVERRUN;
		}
		if ((pending & SDIO_INT_FLAG_STBITE) != 0U) {
			data->data_err |= GD32_SDHC_ERR_START_BIT;
		}
		k_sem_give(&data->data_sem);
	}

	if ((pending & SDIO_INT_FLAG_DTEND) != 0U) {
		data->data_events |= SDIO_INT_FLAG_DTEND;
		k_sem_give(&data->data_sem);
	}

	if ((pending & SDIO_INT_FLAG_DTBLKEND) != 0U) {
		data->data_events |= SDIO_INT_FLAG_DTBLKEND;
		k_sem_give(&data->data_sem);
	}

	/* Clear all latched interrupt flags we have enabled. */
	sdio_interrupt_flag_clear(pending & GD32_SDIO_INT_CLEAR_ALL);
}

static void gd32_sdhc_dma_cb(const struct device *dma_dev, void *user_data, uint32_t channel,
			     int status)
{
	ARG_UNUSED(dma_dev);
	ARG_UNUSED(channel);

	const struct device *sdhc = user_data;
	struct gd32_sdhc_data *data = sdhc->data;

	data->dma_status = status;
	k_sem_give(&data->dma_sem);
}

static uint32_t gd32_sdhc_calc_f_max(uint32_t f_max_dt)
{
	return MIN(f_max_dt, GD32_SDIOCLK_HZ);
}

static int gd32_sdhc_reset(const struct device *dev)
{
	struct gd32_sdhc_data *data = dev->data;
	const struct gd32_sdhc_config *cfg = dev->config;

	LOG_DBG("Reset");

	k_mutex_lock(&data->lock, K_FOREVER);

	sdio_interrupt_disable(0xFFFFFFFFU);
	sdio_dma_disable();
	sdio_dsm_disable();
	sdio_csm_disable();
	sdio_interrupt_flag_clear(GD32_SDIO_INT_CLEAR_ALL);
	gd32_sdio_sync();

	if (cfg->dma.dev != NULL) {
		(void)dma_stop(cfg->dma.dev, cfg->dma.channel);
	}

	data->card_high_capacity = false;

	k_mutex_unlock(&data->lock);
	return 0;
}

static int gd32_sdhc_set_clock(const struct device *dev, uint32_t clock_hz)
{
	const struct gd32_sdhc_config *cfg = dev->config;
	struct gd32_sdhc_data *data = dev->data;

	if (clock_hz == 0U) {
		LOG_DBG("Clock off");
		sdio_clock_disable();
		sdio_hardware_clock_disable();
		gd32_sdio_sync();
		data->host_io.clock = 0U;
		return 0;
	}

	if ((clock_hz < cfg->min_bus_freq) ||
	    (clock_hz > gd32_sdhc_calc_f_max(cfg->max_bus_freq))) {
		LOG_ERR("Clock out of range: %uHz (min=%u max=%u)", clock_hz, cfg->min_bus_freq,
			gd32_sdhc_calc_f_max(cfg->max_bus_freq));
		return -EINVAL;
	}

	LOG_DBG("Clock set: %uHz", clock_hz);

	sdio_clock_disable();
	gd32_sdio_sync();

	if (clock_hz >= GD32_SDIOCLK_HZ) {
		sdio_clock_config(SDIO_SDIOCLKEDGE_RISING, SDIO_CLOCKBYPASS_ENABLE,
				  SDIO_CLOCKPWRSAVE_DISABLE, 0);
		gd32_sdio_sync();
	} else {
		uint32_t denom = DIV_ROUND_UP(GD32_SDIOCLK_HZ, clock_hz);
		denom = CLAMP(denom, 2U, 513U);
		uint16_t div = (uint16_t)(denom - 2U);

		sdio_clock_config(SDIO_SDIOCLKEDGE_RISING, SDIO_CLOCKBYPASS_DISABLE,
				  SDIO_CLOCKPWRSAVE_DISABLE, div);
		gd32_sdio_sync();
	}

	sdio_hardware_clock_disable();
	gd32_sdio_sync();
	sdio_clock_enable();
	gd32_sdio_sync();

	data->host_io.clock = clock_hz;

	if (clock_hz <= SDMMC_CLOCK_400KHZ) {
		(void)pinctrl_apply_state(cfg->pcfg, PINCTRL_STATE_SLOW);
	} else if (clock_hz > 25000000U) {
		(void)pinctrl_apply_state(cfg->pcfg, PINCTRL_STATE_FAST);
	} else {
		(void)pinctrl_apply_state(cfg->pcfg, PINCTRL_STATE_DEFAULT);
	}

	return 0;
}

static int gd32_sdhc_set_io(const struct device *dev, struct sdhc_io *ios)
{
	struct gd32_sdhc_data *data = dev->data;
	const struct gd32_sdhc_config *cfg = dev->config;
	int ret = 0;

	LOG_DBG("Set IO: clk=%u width=%u power=%u timing=%u v=%u", ios->clock, ios->bus_width,
		ios->power_mode, ios->timing, ios->signal_voltage);

	k_mutex_lock(&data->lock, K_FOREVER);

	if ((ios->signal_voltage != 0) && (ios->signal_voltage != SD_VOL_3_3_V)) {
		LOG_WRN("Unsupported signal voltage: %u", ios->signal_voltage);
		ret = -ENOTSUP;
		goto out;
	}

	if ((ios->timing != 0) && (ios->timing != SDHC_TIMING_LEGACY) &&
	    (ios->timing != SDHC_TIMING_HS)) {
		LOG_WRN("Unsupported timing: %u", ios->timing);
		ret = -ENOTSUP;
		goto out;
	}

	if (ios->power_mode == SDHC_POWER_OFF) {
		LOG_DBG("Power off");
		sdio_power_state_set(SDIO_POWER_OFF);
		gd32_sdio_sync();
		data->host_io.power_mode = SDHC_POWER_OFF;
	} else if (ios->power_mode == SDHC_POWER_ON) {
		LOG_DBG("Power on");
		sdio_power_state_set(SDIO_POWER_ON);
		gd32_sdio_sync();
		data->host_io.power_mode = SDHC_POWER_ON;
	}

	if ((ios->bus_width != 0U) && (data->host_io.bus_width != ios->bus_width)) {
		if ((ios->bus_width == SDHC_BUS_WIDTH8BIT) && (cfg->bus_width < 8U)) {
			LOG_WRN("8-bit bus requested but board supports %u-bit", cfg->bus_width);
			ret = -ENOTSUP;
			goto out;
		}
		if ((ios->bus_width == SDHC_BUS_WIDTH4BIT) && (cfg->bus_width < 4U)) {
			LOG_WRN("4-bit bus requested but board supports %u-bit", cfg->bus_width);
			ret = -ENOTSUP;
			goto out;
		}

		LOG_DBG("Bus width set: %u", ios->bus_width);
		switch (ios->bus_width) {
		case SDHC_BUS_WIDTH1BIT:
			sdio_bus_mode_set(SDIO_BUSMODE_1BIT);
			break;
		case SDHC_BUS_WIDTH4BIT:
			sdio_bus_mode_set(SDIO_BUSMODE_4BIT);
			break;
		case SDHC_BUS_WIDTH8BIT:
			sdio_bus_mode_set(SDIO_BUSMODE_8BIT);
			break;
		default:
			ret = -ENOTSUP;
			goto out;
		}

		gd32_sdio_sync();
		data->host_io.bus_width = ios->bus_width;
	}

	if (data->host_io.clock != ios->clock) {
		ret = gd32_sdhc_set_clock(dev, ios->clock);
	}

out:
	k_mutex_unlock(&data->lock);
	return ret;
}

static int gd32_sdhc_get_card_present(const struct device *dev)
{
	const struct gd32_sdhc_config *cfg = dev->config;
	struct gd32_sdhc_data *data = dev->data;

	if (cfg->cd_gpio.port == NULL) {
		LOG_DBG("No CD GPIO, assuming present");
		gd32_sdhc_signal_card_inserted(data);
		return 1;
	}

	int v = gpio_pin_get_dt(&cfg->cd_gpio);
	if (v < 0) {
		LOG_WRN("CD gpio_pin_get_dt failed (%d), assuming present", v);
		gd32_sdhc_signal_card_inserted(data);
		return 1;
	}
	bool present = (v > 0);

	LOG_DBG("CD raw=%d present=%d flags=0x%x", v, present ? 1 : 0, cfg->cd_gpio.dt_flags);
	if (present) {
		gd32_sdhc_signal_card_inserted(data);
	} else {
		gd32_sdhc_signal_card_removed(data);
	}
	return present ? 1 : 0;
}

static int gd32_sdhc_card_busy(const struct device *dev)
{
	ARG_UNUSED(dev);

	if ((sdio_flag_get(SDIO_FLAG_CMDRUN) != RESET) ||
	    (sdio_flag_get(SDIO_FLAG_TXRUN) != RESET) ||
	    (sdio_flag_get(SDIO_FLAG_RXRUN) != RESET)) {
		return 1;
	}

	return 0;
}

static int gd32_sdhc_get_host_props(const struct device *dev, struct sdhc_host_props *props)
{
	struct gd32_sdhc_data *data = dev->data;

	memcpy(props, &data->props, sizeof(*props));
	return 0;
}

static int gd32_sdhc_enable_interrupt(const struct device *dev, sdhc_interrupt_cb_t callback,
				      int sources, void *user_data)
{
	struct gd32_sdhc_data *data = dev->data;
	const struct gd32_sdhc_config *cfg = dev->config;

	if ((sources & ~(SDHC_INT_SDIO | SDHC_INT_INSERTED | SDHC_INT_REMOVED)) != 0) {
		LOG_WRN("Unsupported interrupt sources: 0x%x", sources);
		return -ENOTSUP;
	}

	if (callback == NULL) {
		return -EINVAL;
	}

	data->card_cb = callback;
	data->card_cb_user_data = user_data;
	data->enabled_sources = sources;

	LOG_DBG("Enable interrupts: 0x%x", sources);

	if ((sources & SDHC_INT_SDIO) != 0) {
		sdio_interrupt_enable(SDIO_INT_SDIOINT);
	}

	if ((cfg->cd_gpio.port != NULL) &&
	    ((sources & (SDHC_INT_INSERTED | SDHC_INT_REMOVED)) != 0)) {
		int rc = gd32_sdhc_cd_irq_arm(dev);
		if (rc < 0) {
			LOG_ERR("failed to arm CD interrupt: %d", rc);
			return rc;
		}
	}

	return 0;
}

static int gd32_sdhc_disable_interrupt(const struct device *dev, int sources)
{
	struct gd32_sdhc_data *data = dev->data;
	const struct gd32_sdhc_config *cfg = dev->config;

	LOG_DBG("Disable interrupts: 0x%x", sources);

	if ((sources & SDHC_INT_SDIO) != 0) {
		sdio_interrupt_disable(SDIO_INT_SDIOINT);
	}

	data->enabled_sources &= ~sources;

	if ((cfg->cd_gpio.port != NULL) &&
	    ((data->enabled_sources & (SDHC_INT_INSERTED | SDHC_INT_REMOVED)) == 0)) {
		(void)gpio_pin_interrupt_configure_dt(&cfg->cd_gpio, GPIO_INT_DISABLE);
	}

	return 0;
}

static void gd32_sdhc_read_response(struct sdhc_command *cmd)
{
	uint32_t native = cmd->response_type & SDHC_NATIVE_RESPONSE_MASK;

	if (native == SD_RSP_TYPE_R2) {
		cmd->response[0] = sdio_response_get(SDIO_RESPONSE3);
		cmd->response[1] = sdio_response_get(SDIO_RESPONSE2);
		cmd->response[2] = sdio_response_get(SDIO_RESPONSE1);
		cmd->response[3] = sdio_response_get(SDIO_RESPONSE0);
	} else if (native != SD_RSP_TYPE_NONE) {
		cmd->response[0] = sdio_response_get(SDIO_RESPONSE0);
	}
}

static bool gd32_sdhc_resp_has_crc(uint32_t response_type)
{
	switch (response_type & SDHC_NATIVE_RESPONSE_MASK) {
	case SD_RSP_TYPE_R3:
	case SD_RSP_TYPE_R4:
		return false;
	default:
		return (response_type & SDHC_NATIVE_RESPONSE_MASK) != SD_RSP_TYPE_NONE;
	}
}

static bool gd32_sdhc_resp_has_cmdidx(uint32_t response_type)
{
	switch (response_type & SDHC_NATIVE_RESPONSE_MASK) {
	case SD_RSP_TYPE_R1:
	case SD_RSP_TYPE_R1b:
	case SD_RSP_TYPE_R5:
	case SD_RSP_TYPE_R5b:
	case SD_RSP_TYPE_R6:
	case SD_RSP_TYPE_R7:
		return true;
	default:
		return false;
	}
}

static uint32_t gd32_sdhc_resp_sel(uint32_t response_type)
{
	switch (response_type & SDHC_NATIVE_RESPONSE_MASK) {
	case SD_RSP_TYPE_NONE:
		return SDIO_RESPONSETYPE_NO;
	case SD_RSP_TYPE_R2:
		return SDIO_RESPONSETYPE_LONG;
	default:
		return SDIO_RESPONSETYPE_SHORT;
	}
}

static bool gd32_sdhc_cmd_needs_cmdsend(uint32_t response_type)
{
	return (response_type & SDHC_NATIVE_RESPONSE_MASK) == SD_RSP_TYPE_NONE;
}

static int gd32_sdhc_send_cmd(const struct device *dev, struct sdhc_command *cmd)
{
	struct gd32_sdhc_data *data = dev->data;
	uint32_t resp = gd32_sdhc_resp_sel(cmd->response_type);
	bool wait_cmdsend = gd32_sdhc_cmd_needs_cmdsend(cmd->response_type);
	bool resp_has_crc = gd32_sdhc_resp_has_crc(cmd->response_type);

	if (gd32_sdhc_is_card_removed(data)) {
		data->cmd_err = GD32_SDHC_ERR_CARD_REMOVED;
		return -ENODEV;
	}

	LOG_DBG("CMD%u arg=0x%08x resp=%u timeout=%d", cmd->opcode, cmd->arg,
		(unsigned int)(cmd->response_type & SDHC_NATIVE_RESPONSE_MASK), cmd->timeout_ms);

	data->cmd_err = 0U;
	k_sem_reset(&data->cmd_sem);

	/* Clear any previous latched flags before starting a new command. */
	sdio_interrupt_flag_clear(GD32_SDIO_INT_CLEAR_XFER);
	gd32_sdio_sync();

	if (wait_cmdsend) {
		sdio_interrupt_enable(SDIO_INT_CMDSEND | SDIO_INT_CMDTMOUT | SDIO_INT_STBITE);
	} else {
		/*
		 * The GD32 SDIO peripheral may assert CCRCERR even for response
		 * types without a CRC (e.g. R3). Use it as a completion event,
		 * then ignore it when resp_has_crc is false.
		 */
		uint32_t en =
			SDIO_INT_CMDRECV | SDIO_INT_CCRCERR | SDIO_INT_CMDTMOUT | SDIO_INT_STBITE;

		sdio_interrupt_enable(en);
	}

	sdio_command_response_config(cmd->opcode, cmd->arg, resp);
	gd32_sdio_sync();
	sdio_wait_type_set(SDIO_WAITTYPE_NO);
	gd32_sdio_sync();
	sdio_csm_enable();
	gd32_sdio_sync();

	int ret = k_sem_take(&data->cmd_sem, gd32_ms_to_timeout(cmd->timeout_ms));

	sdio_interrupt_disable(GD32_SDIO_INT_CMD_MASK);

	if (gd32_sdhc_is_card_removed(data)) {
		data->cmd_err |= GD32_SDHC_ERR_CARD_REMOVED;
		gd32_sdhc_abort_io(dev);
		return -ENODEV;
	}

	if (ret != 0) {
		LOG_ERR("CMD%u timed out", cmd->opcode);
		gd32_sdhc_dump_hw_state("CMD timeout");
		return -ETIMEDOUT;
	}

	if (data->cmd_err != 0U) {
		if (!resp_has_crc && ((data->cmd_err & GD32_SDHC_ERR_CMD_CRC) != 0U)) {
			data->cmd_err &= ~GD32_SDHC_ERR_CMD_CRC;
		}

		if (data->cmd_err == 0U) {
			gd32_sdhc_read_response(cmd);
			gd32_sdhc_maybe_log_response(data, cmd);
			if (cmd->opcode == SD_APP_SEND_OP_COND) {
				data->card_high_capacity =
					(cmd->response[0] & SD_OCR_CARD_CAP_FLAG) != 0U;
			} else if (cmd->opcode == MMC_SEND_OP_COND) {
				data->card_high_capacity =
					(cmd->response[0] & MMC_OCR_SECTOR_MODE) != 0U;
			}
			return 0;
		}

		LOG_ERR("CMD%u failed (err=0x%x)", cmd->opcode, data->cmd_err);
		gd32_sdhc_log_cmd_error(data->cmd_err);
		gd32_sdhc_dump_hw_state("CMD failed");
		if ((data->cmd_err & GD32_SDHC_ERR_CMD_TIMEOUT) != 0U) {
			return -ETIMEDOUT;
		}
		return -EIO;
	}

	if (gd32_sdhc_resp_has_cmdidx(cmd->response_type) &&
	    (sdio_command_index_get() != cmd->opcode)) {
		LOG_ERR("CMD%u bad cmd index in response (%u)", cmd->opcode,
			sdio_command_index_get());
		gd32_sdhc_dump_hw_state("CMD bad index");
		return -EIO;
	}

	gd32_sdhc_read_response(cmd);
	gd32_sdhc_maybe_log_response(data, cmd);
	if (cmd->opcode == SD_APP_SEND_OP_COND) {
		data->card_high_capacity = (cmd->response[0] & SD_OCR_CARD_CAP_FLAG) != 0U;
	} else if (cmd->opcode == MMC_SEND_OP_COND) {
		data->card_high_capacity = (cmd->response[0] & MMC_OCR_SECTOR_MODE) != 0U;
	}
	return 0;
}

static bool gd32_sdhc_is_write(const struct sdhc_command *cmd)
{
	switch (cmd->opcode) {
	case SD_WRITE_SINGLE_BLOCK:
	case SD_WRITE_MULTIPLE_BLOCK:
		return true;
	case SDIO_RW_EXTENDED:
		return (cmd->arg & BIT(SDIO_CMD_ARG_RW_SHIFT)) != 0U;
	default:
		return false;
	}
}

static bool gd32_sdhc_is_multiblock_rw(const struct sdhc_command *cmd)
{
	return (cmd->opcode == SD_READ_MULTIPLE_BLOCK) || (cmd->opcode == SD_WRITE_MULTIPLE_BLOCK);
}

static uint32_t gd32_sdhc_data_mode(const struct sdhc_command *cmd)
{
	if (cmd->opcode != SDIO_RW_EXTENDED) {
		return SDIO_TRANSMODE_BLOCK;
	}

	/* CMD53 block mode is indicated by BLK bit in argument. */
	bool is_block_mode = (cmd->arg & BIT(SDIO_EXTEND_CMD_ARG_BLK_SHIFT)) != 0U;
	return is_block_mode ? SDIO_TRANSMODE_BLOCK : SDIO_TRANSMODE_STREAM;
}

static bool gd32_sdhc_buf_is_dma_capable(const void *buf, size_t len)
{
#if IS_ENABLED(CONFIG_MEM_ATTR)
	if (IS_ENABLED(CONFIG_MMU) || (len == 0U)) {
		return true;
	}

	uintptr_t addr = (uintptr_t)buf;
	if ((addr + len) < addr) {
		return false;
	}

	const struct mem_attr_region_t *regions;
	size_t num_regions = mem_attr_get_regions(&regions);

	for (size_t i = 0; i < num_regions; i++) {
		const struct mem_attr_region_t *region = &regions[i];
		uintptr_t region_end = region->dt_addr + region->dt_size;

		if ((region->dt_attr & DT_MEM_DMA) != DT_MEM_DMA) {
			continue;
		}

		if ((addr >= region->dt_addr) && (addr < region_end) &&
		    ((addr + len) <= region_end)) {
			return true;
		}
	}

	return false;
#else
	ARG_UNUSED(buf);
	ARG_UNUSED(len);
	return true;
#endif
}

static bool gd32_sdhc_data_buf_is_compatible(const struct device *dev, const void *buf, size_t len)
{
	ARG_UNUSED(dev);
	/*
	 * SDIO FIFO access is word-based. This driver configures DMA for 32-bit
	 * transfers (as in the GD32 firmware library `sdcard.c` sample), so the
	 * buffer and length must be word-aligned.
	 */
	return IS_ALIGNED(buf, sizeof(uint32_t)) && ((len % sizeof(uint32_t)) == 0U) &&
	       gd32_sdhc_buf_is_dma_capable(buf, len);
}

static int gd32_sdhc_start_dma(const struct device *dev, bool write, void *dma_buf, size_t dma_len)
{
	const struct gd32_sdhc_config *cfg = dev->config;
	struct gd32_sdhc_data *data = dev->data;

	if ((cfg->dma.dev == NULL) || !device_is_ready(cfg->dma.dev)) {
		return -ENODEV;
	}

	struct dma_block_config blk = {0};
	struct dma_config dma_cfg = {0};

	dma_cfg.channel_direction = write ? MEMORY_TO_PERIPHERAL : PERIPHERAL_TO_MEMORY;
	dma_cfg.source_data_size = 4U;
	dma_cfg.dest_data_size = 4U;
	dma_cfg.source_burst_length = GD32_SDHC_DMA_BURST_LENGTH;
	dma_cfg.dest_burst_length = GD32_SDHC_DMA_BURST_LENGTH;
	dma_cfg.channel_priority = 3U;
	dma_cfg.block_count = 1U;
	dma_cfg.head_block = &blk;
	dma_cfg.dma_callback = gd32_sdhc_dma_cb;
	dma_cfg.user_data = (void *)dev;

#if DT_HAS_COMPAT_STATUS_OKAY(gd_gd32_dma_v1)
	dma_cfg.dma_slot = cfg->dma.slot;
#endif

	if (write) {
		blk.source_address = (uint32_t)dma_buf;
		blk.dest_address = cfg->fifo_addr;
		blk.source_addr_adj = DMA_ADDR_ADJ_INCREMENT;
		blk.dest_addr_adj = DMA_ADDR_ADJ_NO_CHANGE;
	} else {
		blk.source_address = cfg->fifo_addr;
		blk.dest_address = (uint32_t)dma_buf;
		blk.source_addr_adj = DMA_ADDR_ADJ_NO_CHANGE;
		blk.dest_addr_adj = DMA_ADDR_ADJ_INCREMENT;
	}

	blk.block_size = (uint32_t)dma_len;
	/* Match the GD32 firmware library SDIO sample: `DMA_FIFO_4_WORD`. */
	blk.fifo_mode_control = 3U;

	data->dma_status = 0;
	k_sem_reset(&data->dma_sem);

	int ret = dma_config(cfg->dma.dev, cfg->dma.channel, &dma_cfg);
	if (ret != 0) {
		return ret;
	}

#if defined(DMA_FLOW_CONTROLLER_PERI)
	/*
	 * Match the GD32 firmware library SDIO sample: use peripheral flow control.
	 */
	dma_flow_controller_config(cfg->dma.reg, (dma_channel_enum)cfg->dma.channel,
				   DMA_FLOW_CONTROLLER_PERI);
#endif

	ret = dma_start(cfg->dma.dev, cfg->dma.channel);
	if (ret != 0) {
		return ret;
	}

	return 0;
}

static void gd32_sdhc_stop_dma(const struct device *dev)
{
	const struct gd32_sdhc_config *cfg = dev->config;

	if ((cfg->dma.dev != NULL) && device_is_ready(cfg->dma.dev)) {
		(void)dma_stop(cfg->dma.dev, cfg->dma.channel);
	}
}

static void gd32_sdhc_drain_rx_fifo(void)
{
	while (sdio_flag_get(SDIO_FLAG_RXDTVAL) != RESET) {
		(void)sdio_data_read();
	}
}

static void gd32_sdhc_abort_io(const struct device *dev)
{
	struct gd32_sdhc_data *data = dev->data;

	sdio_interrupt_disable(GD32_SDIO_INT_CMD_MASK | GD32_SDIO_INT_DATA_MASK);
	sdio_dsm_disable();
	sdio_dma_disable();
	sdio_csm_disable();

	gd32_sdhc_stop_dma(dev);
	(void)k_sem_take(&data->dma_sem, K_NO_WAIT);

	gd32_sdhc_drain_rx_fifo();

	sdio_interrupt_flag_clear(GD32_SDIO_INT_CLEAR_XFER);
	gd32_sdio_sync();
}

static void gd32_sdhc_wait_idle_ms(uint32_t timeout_ms)
{
	int64_t deadline = k_uptime_get() + (int64_t)timeout_ms;

	while ((sdio_flag_get(SDIO_FLAG_CMDRUN) != RESET) ||
	       (sdio_flag_get(SDIO_FLAG_TXRUN) != RESET) ||
	       (sdio_flag_get(SDIO_FLAG_RXRUN) != RESET)) {
		if (timeout_ms != 0U && (k_uptime_get() >= deadline)) {
			break;
		}
		k_sleep(K_MSEC(1));
	}
}

static int gd32_sdhc_prepare_data_path(const struct device *dev, const struct sdhc_command *cmd,
				       const struct sdhc_data *xfer, bool write)
{
	size_t bytes = (size_t)xfer->block_size * (size_t)xfer->blocks;

	if ((bytes == 0U) || (xfer->data == NULL)) {
		return -EINVAL;
	}

	if (!is_power_of_two(xfer->block_size) || (xfer->block_size > 16384U)) {
		return -EINVAL;
	}

	uint32_t blksz = gd32_sdio_datablocksize_get(xfer->block_size);
	uint32_t timeout = gd32_sdhc_timeout_to_cycles(dev->data, xfer->timeout_ms);

	sdio_interrupt_flag_clear(GD32_SDIO_INT_CLEAR_XFER);
	gd32_sdio_sync();

	sdio_data_config(timeout, bytes, blksz);
	gd32_sdio_sync();
	sdio_data_transfer_config(gd32_sdhc_data_mode(cmd),
				  write ? SDIO_TRANSDIRECTION_TOCARD : SDIO_TRANSDIRECTION_TOSDIO);
	gd32_sdio_sync();

	return 0;
}

static int gd32_sdhc_wait_data_done(const struct device *dev, const struct sdhc_command *cmd,
				    const struct sdhc_data *xfer, bool write)
{
	struct gd32_sdhc_data *data = dev->data;
	const struct gd32_sdhc_config *cfg = dev->config;
	bool need_dtblkend = (gd32_sdhc_data_mode(cmd) == SDIO_TRANSMODE_BLOCK);

	if (gd32_sdhc_is_card_removed(data)) {
		data->data_err = GD32_SDHC_ERR_CARD_REMOVED;
		return -ENODEV;
	}

	int64_t deadline_ms = INT64_MAX;
	if (xfer->timeout_ms != SDHC_TIMEOUT_FOREVER) {
		deadline_ms = k_uptime_get() + (int64_t)xfer->timeout_ms;
	}

	for (;;) {
		k_timeout_t to = K_FOREVER;
		if (xfer->timeout_ms != SDHC_TIMEOUT_FOREVER) {
			int64_t remaining = deadline_ms - k_uptime_get();
			if (remaining <= 0) {
				LOG_ERR("DATA timeout waiting for SDIO (CMD%u write=%u "
					"data_err=0x%x "
					"events=0x%x STAT=0x%08x INTEN=0x%08x DATACNT=0x%08x)",
					cmd->opcode, write ? 1U : 0U, data->data_err,
					data->data_events, SDIO_STAT, SDIO_INTEN, SDIO_DATACNT);
				gd32_sdhc_dump_hw_state("DATA SDIO timeout");
				return -ETIMEDOUT;
			}
			to = K_MSEC((uint32_t)remaining);
		}

		int ret = k_sem_take(&data->data_sem, to);
		if (ret != 0) {
			LOG_ERR("DATA timeout waiting for SDIO (CMD%u write=%u data_err=0x%x "
				"events=0x%x STAT=0x%08x INTEN=0x%08x DATACNT=0x%08x)",
				cmd->opcode, write ? 1U : 0U, data->data_err, data->data_events,
				SDIO_STAT, SDIO_INTEN, SDIO_DATACNT);
			gd32_sdhc_dump_hw_state("DATA SDIO timeout");
			return -ETIMEDOUT;
		}

		if (gd32_sdhc_is_card_removed(data)) {
			data->data_err |= GD32_SDHC_ERR_CARD_REMOVED;
			gd32_sdhc_abort_io(dev);
			return -ENODEV;
		}

		if (write && ((data->data_err & GD32_SDHC_ERR_DATA_TIMEOUT) != 0U) &&
		    ((data->data_events & SDIO_INT_FLAG_DTEND) != 0U) && (SDIO_DATACNT == 0U)) {
			data->data_err &= ~GD32_SDHC_ERR_DATA_TIMEOUT;
		}

		if (data->data_err != 0U) {
			LOG_ERR("DATA error (CMD%u write=%u data_err=0x%x events=0x%x STAT=0x%08x "
				"INTEN=0x%08x DATACNT=0x%08x)",
				cmd->opcode, write ? 1U : 0U, data->data_err, data->data_events,
				SDIO_STAT, SDIO_INTEN, SDIO_DATACNT);
			sdio_interrupt_disable(GD32_SDIO_INT_DATA_MASK);
			sdio_dsm_disable();
			sdio_dma_disable();
			gd32_sdio_sync();

			gd32_sdhc_stop_dma(dev);
			(void)k_sem_take(&data->dma_sem, K_NO_WAIT);

			gd32_sdhc_log_data_error(data->data_err, data->dma_status);
			gd32_sdhc_dump_hw_state("DATA failed");
			if ((data->data_err & GD32_SDHC_ERR_DATA_TIMEOUT) != 0U) {
				return -ETIMEDOUT;
			}
			return -EIO;
		}

		if (((data->data_events & SDIO_INT_FLAG_DTEND) != 0U) &&
		    (!need_dtblkend || ((data->data_events & SDIO_INT_FLAG_DTBLKEND) != 0U))) {
			break;
		}
	}

	/*
	 * The GD32 firmware library SDIO sample polls on DMA "full transfer finish".
	 * In practice this can be noisy for SDIO transfers, so this driver uses the
	 * SDIO DTEND/DTBLKEND events (+ DATACNT==0) as the completion condition.
	 */
	for (;;) {
		if (gd32_sdhc_is_card_removed(data)) {
			data->data_err |= GD32_SDHC_ERR_CARD_REMOVED;
			gd32_sdhc_abort_io(dev);
			return -ENODEV;
		}

		int sem_ret = k_sem_take(&data->dma_sem, K_NO_WAIT);
		if (sem_ret == 0) {
			break;
		}

		if (xfer->timeout_ms != SDHC_TIMEOUT_FOREVER) {
			int64_t remaining = deadline_ms - k_uptime_get();
			if (remaining <= 0) {
				LOG_ERR("DATA timeout waiting for DMA (CMD%u write=%u "
					"data_err=0x%x "
					"events=0x%x STAT=0x%08x INTEN=0x%08x DATACNT=0x%08x)",
					cmd->opcode, write ? 1U : 0U, data->data_err,
					data->data_events, SDIO_STAT, SDIO_INTEN, SDIO_DATACNT);
				gd32_sdhc_dump_hw_state("DATA DMA timeout");
				return -ETIMEDOUT;
			}
		}

		if ((cfg->dma.dev != NULL) && device_is_ready(cfg->dma.dev)) {
			struct dma_status stat;
			if (dma_get_status(cfg->dma.dev, cfg->dma.channel, &stat) == 0) {
				if (!stat.busy && (stat.pending_length == 0U)) {
					break;
				}
			}
		}

		/* Yield to allow DMA ISR to run (when it does fire). */
		k_sleep(K_MSEC(1));
	}

	/* DMA error signalling can be noisy for SDIO even when SDIO completes cleanly. */
	if (data->dma_status < 0) {
		LOG_DBG("Ignoring DMA status error (CMD%u write=%u dma=%d events=0x%x "
			"STAT=0x%08x DATACNT=0x%08x)",
			cmd->opcode, write ? 1U : 0U, data->dma_status, data->data_events,
			SDIO_STAT, SDIO_DATACNT);
		data->dma_status = 0;
	}

	if (data->data_err != 0U) {
		LOG_ERR("DATA error after DMA (CMD%u write=%u data_err=0x%x events=0x%x "
			"STAT=0x%08x "
			"INTEN=0x%08x DATACNT=0x%08x)",
			cmd->opcode, write ? 1U : 0U, data->data_err, data->data_events, SDIO_STAT,
			SDIO_INTEN, SDIO_DATACNT);
		gd32_sdhc_log_data_error(data->data_err, data->dma_status);
		gd32_sdhc_dump_hw_state("DATA failed");
		if ((data->data_err & GD32_SDHC_ERR_DATA_TIMEOUT) != 0U) {
			return -ETIMEDOUT;
		}
		return -EIO;
	}

	return 0;
}

static bool gd32_sdhc_is_transient_data_failure(uint32_t data_err, int dma_status)
{
	if ((data_err & GD32_SDHC_ERR_CARD_REMOVED) != 0U) {
		return false;
	}

	if (dma_status < 0) {
		return true;
	}

	return (data_err &
		(GD32_SDHC_ERR_DATA_CRC | GD32_SDHC_ERR_START_BIT | GD32_SDHC_ERR_TX_UNDERRUN |
		 GD32_SDHC_ERR_RX_OVERRUN | GD32_SDHC_ERR_DATA_TIMEOUT)) != 0U;
}

static int gd32_sdhc_transfer_data_chunk(const struct device *dev, struct sdhc_command *cmd,
					 struct sdhc_data *xfer)
{
	struct gd32_sdhc_data *data = dev->data;
	bool write = gd32_sdhc_is_write(cmd);

	size_t bytes = (size_t)xfer->block_size * (size_t)xfer->blocks;
	void *dma_buf = xfer->data;
	size_t dma_len = bytes;

	data->data_err = 0U;
	data->data_events = 0U;
	data->dma_status = 0;
	k_sem_reset(&data->data_sem);

	if (!gd32_sdhc_data_buf_is_compatible(dev, dma_buf, dma_len)) {
		return -ENOTSUP;
	}

	if (write) {
		sys_cache_data_flush_range(dma_buf, dma_len);
	} else {
		sys_cache_data_invd_range(dma_buf, dma_len);
	}

	int ret = gd32_sdhc_prepare_data_path(dev, cmd, xfer, write);
	if (ret != 0) {
		return ret;
	}

	if (!write) {
		/*
		 * Read: arm DMA+DSM before issuing the command. This differs from the GD32
		 * firmware library SDIO sample, which enables DMA after the command.
		 */
		sdio_interrupt_enable(GD32_SDIO_INT_DATA_MASK);

		ret = gd32_sdhc_start_dma(dev, write, dma_buf, dma_len);
		if (ret != 0) {
			gd32_sdhc_abort_io(dev);
			return ret;
		}

		sdio_dma_enable();
		gd32_sdio_sync();
		sdio_dsm_enable();
		gd32_sdio_sync();

		ret = gd32_sdhc_send_cmd(dev, cmd);
		if (ret != 0) {
			gd32_sdhc_abort_io(dev);
			return ret;
		}
	} else {
		/*
		 * Write: issue the command first, then enable DMA+DSM so the host is
		 * ready when the card starts requesting data.
		 */
		ret = gd32_sdhc_send_cmd(dev, cmd);
		if (ret != 0) {
			gd32_sdhc_abort_io(dev);
			return ret;
		}

		sdio_interrupt_enable(GD32_SDIO_INT_DATA_MASK);

		ret = gd32_sdhc_start_dma(dev, write, dma_buf, dma_len);
		if (ret != 0) {
			gd32_sdhc_abort_io(dev);
			return ret;
		}

		sdio_dma_enable();
		gd32_sdio_sync();
		sdio_dsm_enable();
		gd32_sdio_sync();
	}

	ret = gd32_sdhc_wait_data_done(dev, cmd, xfer, write);

	gd32_sdhc_stop_dma(dev);
	sdio_dsm_disable();
	sdio_dma_disable();
	sdio_interrupt_disable(GD32_SDIO_INT_DATA_MASK);
	gd32_sdhc_drain_rx_fifo();
	sdio_interrupt_flag_clear(GD32_SDIO_INT_CLEAR_XFER);
	gd32_sdio_sync();

	if (!write && (ret == 0)) {
		sys_cache_data_invd_range(dma_buf, dma_len);
	}

	if (ret == 0) {
		xfer->bytes_xfered = bytes;
	}

	if (gd32_sdhc_is_multiblock_rw(cmd) && (xfer->blocks > 1U)) {
		struct sdhc_command stop = {
			.opcode = SD_STOP_TRANSMISSION,
			.arg = 0U,
			.response_type = SD_RSP_TYPE_R1b,
			.timeout_ms = cmd->timeout_ms,
			.retries = cmd->retries,
		};
		(void)gd32_sdhc_send_cmd(dev, &stop);
	}

	return ret;
}

static int gd32_sdhc_transfer_data_chunk_with_retries(const struct device *dev,
						      struct sdhc_command *cmd,
						      struct sdhc_data *xfer)
{
	struct gd32_sdhc_data *data = dev->data;
	int base_attempts = (int)cmd->retries + 1;
	int max_attempts = base_attempts + ((cmd->retries == 0U) ? 1 : 0);
	int last_ret = 0;
	uint32_t last_cmd_err = 0U;
	uint32_t last_data_err = 0U;
	int last_dma_status = 0;
	int ret = 0;
	int attempts_used = 0;

	for (int attempt = 1;; ++attempt) {
		attempts_used = attempt;
		if (gd32_sdhc_is_card_removed(data)) {
			ret = -ENODEV;
			break;
		}
		gd32_sdhc_abort_io(dev);
		gd32_sdhc_wait_idle_ms(20U);

		ret = gd32_sdhc_transfer_data_chunk(dev, cmd, xfer);

		last_ret = ret;
		last_cmd_err = data->cmd_err;
		last_data_err = data->data_err;
		last_dma_status = data->dma_status;

		if (ret == 0) {
			break;
		}
		if (ret == -ENODEV) {
			break;
		}

		bool transient =
			gd32_sdhc_is_transient_data_failure(last_data_err, last_dma_status);
		bool retry_allowed =
			(attempt < base_attempts) ||
			((cmd->retries == 0U) && (attempt == base_attempts) && transient);
		if (!retry_allowed) {
			break;
		}

		uint32_t delay_ms =
			gd32_sdhc_retry_delay_ms(last_cmd_err, last_data_err, last_dma_status);
		LOG_INF("Retrying DATA CMD%u (%s) in %ums (%d/%d) (ret=%d cmd_err=0x%x "
			"data_err=0x%x dma=%d)",
			cmd->opcode,
			gd32_sdhc_primary_error_str(last_cmd_err, last_data_err, last_dma_status),
			delay_ms, attempt + 1, max_attempts, last_ret, last_cmd_err, last_data_err,
			last_dma_status);

		gd32_sdhc_abort_io(dev);
		gd32_sdhc_wait_idle_ms(20U);
		k_sleep(K_MSEC(delay_ms));
	}

	if (ret != 0) {
		if (ret == -ENODEV) {
			return ret;
		}
		LOG_ERR("DATA CMD%u failed after %d attempts (ret=%d cmd_err=0x%x data_err=0x%x "
			"dma=%d)",
			cmd->opcode, attempts_used, last_ret, last_cmd_err, last_data_err,
			last_dma_status);
		gd32_sdhc_log_data_error(last_data_err, last_dma_status);
	}

	return ret;
}

static int gd32_sdhc_cmd_with_data(const struct device *dev, struct sdhc_command *cmd,
				   struct sdhc_data *xfer)
{
	return gd32_sdhc_transfer_data_chunk_with_retries(dev, cmd, xfer);
}

static int gd32_sdhc_request(const struct device *dev, struct sdhc_command *cmd,
			     struct sdhc_data *xfer)
{
	struct gd32_sdhc_data *data = dev->data;

	__ASSERT_NO_MSG(cmd != NULL);

	k_mutex_lock(&data->lock, K_FOREVER);

	if (gd32_sdhc_is_card_removed(data)) {
		k_mutex_unlock(&data->lock);
		return -ENODEV;
	}

	int ret = 0;

	if (xfer != NULL) {
		ret = gd32_sdhc_cmd_with_data(dev, cmd, xfer);
	} else {
		int attempts = (int)cmd->retries + 1;
		int last_ret = 0;
		uint32_t last_cmd_err = 0U;

		for (int attempt = 1; attempt <= attempts; ++attempt) {
			gd32_sdhc_abort_io(dev);
			gd32_sdhc_wait_idle_ms(20U);

			ret = gd32_sdhc_send_cmd(dev, cmd);
			last_ret = ret;
			last_cmd_err = data->cmd_err;

			if (ret == 0) {
				break;
			}
			if (ret == -ENODEV) {
				break;
			}

			if (attempt < attempts) {
				uint32_t delay_ms = gd32_sdhc_retry_delay_ms(last_cmd_err, 0U, 0);
				LOG_INF("Retrying CMD%u (%s) in %ums (%d/%d) (ret=%d cmd_err=0x%x)",
					cmd->opcode,
					gd32_sdhc_primary_error_str(last_cmd_err, 0U, 0), delay_ms,
					attempt + 1, attempts, last_ret, last_cmd_err);

				gd32_sdhc_abort_io(dev);
				gd32_sdhc_wait_idle_ms(20U);
				k_sleep(K_MSEC(delay_ms));
			}
		}

		if (ret != 0) {
			if (ret == -ENODEV) {
				k_mutex_unlock(&data->lock);
				return ret;
			}
			LOG_ERR("CMD%u failed after %d attempts (ret=%d cmd_err=0x%x)", cmd->opcode,
				attempts, last_ret, last_cmd_err);
			gd32_sdhc_log_cmd_error(last_cmd_err);
		}
	}

	k_mutex_unlock(&data->lock);
	return ret;
}

static void gd32_sdhc_init_host_props(const struct device *dev)
{
	struct gd32_sdhc_data *data = dev->data;
	const struct gd32_sdhc_config *cfg = dev->config;

	memset(&data->props, 0, sizeof(data->props));

	data->props.f_min = cfg->min_bus_freq;
	data->props.f_max = gd32_sdhc_calc_f_max(cfg->max_bus_freq);
	data->props.power_delay = cfg->power_delay_ms;

	data->props.host_caps.bus_4_bit_support = (cfg->bus_width >= 4U);
	data->props.host_caps.bus_8_bit_support = (cfg->bus_width >= 8U);
	data->props.host_caps.high_spd_support = (data->props.f_max >= SD_CLOCK_25MHZ);
	data->props.host_caps.sdma_support = 1;
	data->props.host_caps.sdio_async_interrupt_support = 1;
	data->props.host_caps.vol_330_support = 1;

	data->props.max_current_330 = cfg->max_current_330;
	data->props.max_current_300 = cfg->max_current_300;
	data->props.max_current_180 = cfg->max_current_180;
	data->props.is_spi = false;
}

static int gd32_sdhc_init(const struct device *dev)
{
	struct gd32_sdhc_data *data = dev->data;
	const struct gd32_sdhc_config *cfg = dev->config;
	int ret;

	data->dev = dev;

	LOG_INF("Init");

	ret = pinctrl_apply_state(cfg->pcfg, PINCTRL_STATE_DEFAULT);
	if (ret < 0) {
		LOG_ERR("pinctrl_apply_state failed (%d)", ret);
		return ret;
	}

	ret = clock_control_on(GD32_CLOCK_CONTROLLER, (clock_control_subsys_t)&cfg->clkid);
	if (ret != 0) {
		LOG_ERR("clock_control_on failed (%d)", ret);
		return ret;
	}

	if (cfg->reset.dev != NULL) {
		(void)reset_line_toggle_dt(&cfg->reset);
	}

	if (cfg->cd_gpio.port != NULL) {
		if (!gpio_is_ready_dt(&cfg->cd_gpio)) {
			LOG_WRN("CD GPIO not ready");
		} else {
			ret = gpio_pin_configure_dt(&cfg->cd_gpio, GPIO_INPUT);
			if (ret < 0) {
				LOG_ERR("CD gpio_pin_configure_dt failed (%d)", ret);
				return ret;
			}
			data->cd_present = (gpio_pin_get_dt(&cfg->cd_gpio) > 0);
			gpio_init_callback(&data->cd_cb, gd32_cd_gpio_cb, BIT(cfg->cd_gpio.pin));
			ret = gpio_add_callback(cfg->cd_gpio.port, &data->cd_cb);
			if (ret < 0) {
				LOG_ERR("CD gpio_add_callback failed (%d)", ret);
				return ret;
			}
		}
	}

	k_mutex_init(&data->lock);
	k_sem_init(&data->cmd_sem, 0, 1);
	k_sem_init(&data->data_sem, 0, 1);
	k_sem_init(&data->dma_sem, 0, 1);

	data->cmd13_count = 0U;
	data->last_cmd13_r1 = 0U;
	data->last_r1_err_flags = 0U;
	data->card_high_capacity = false;

	/* Ensure the peripheral is in a known state. */
	sdio_deinit();
	sdio_interrupt_disable(GD32_SDIO_INT_CMD_MASK | GD32_SDIO_INT_DATA_MASK | SDIO_INT_SDIOINT);
	sdio_interrupt_flag_clear(GD32_SDIO_INT_CLEAR_ALL);
	gd32_sdio_sync();

	sdio_bus_mode_set(SDIO_BUSMODE_1BIT);
	gd32_sdio_sync();
	sdio_power_state_set(SDIO_POWER_OFF);
	gd32_sdio_sync();

	data->host_io.bus_width = SDHC_BUS_WIDTH1BIT;
	data->host_io.power_mode = SDHC_POWER_OFF;
	data->host_io.signal_voltage = SD_VOL_3_3_V;
	data->host_io.timing = SDHC_TIMING_LEGACY;

	ret = gd32_sdhc_set_clock(dev, 0U);
	if (ret != 0) {
		LOG_ERR("Failed to gate clock (%d)", ret);
		return ret;
	}

	LOG_INF("Card present: %d", gd32_sdhc_get_card_present(dev));

	/* Hook IRQ */
	cfg->irq_config_func(dev);

	gd32_sdhc_init_host_props(dev);
	LOG_INF("Ready (f_min=%u f_max=%u bus_width=%u)", data->props.f_min, data->props.f_max,
		cfg->bus_width);

	return 0;
}

static DEVICE_API(sdhc, gd32_sdhc_api) = {
	.reset = gd32_sdhc_reset,
	.request = gd32_sdhc_request,
	.data_buf_is_compatible = gd32_sdhc_data_buf_is_compatible,
	.set_io = gd32_sdhc_set_io,
	.get_card_present = gd32_sdhc_get_card_present,
	.execute_tuning = NULL,
	.card_busy = gd32_sdhc_card_busy,
	.get_host_props = gd32_sdhc_get_host_props,
	.enable_interrupt = gd32_sdhc_enable_interrupt,
	.disable_interrupt = gd32_sdhc_disable_interrupt,
};

#define GD32_SDHC_IRQ_CONNECT(inst)                                                                \
	static void gd32_sdhc_irq_config_##inst(const struct device *dev)                          \
	{                                                                                          \
		IRQ_CONNECT(DT_INST_IRQN(inst), DT_INST_IRQ(inst, priority), gd32_sdhc_isr,        \
			    DEVICE_DT_INST_GET(inst), 0);                                          \
		irq_enable(DT_INST_IRQN(inst));                                                    \
	}

#define GD32_SDHC_DMA_INIT(inst)                                                                   \
	.dma = {.dev = DEVICE_DT_GET(DT_INST_DMAS_CTLR(inst)),                                     \
		.reg = DT_REG_ADDR(DT_INST_DMAS_CTLR(inst)),                                       \
		.channel = DT_INST_DMAS_CELL_BY_IDX(inst, 0, channel),                             \
		COND_CODE_1(DT_HAS_COMPAT_STATUS_OKAY(gd_gd32_dma_v1),                                  \
			    (.slot = DT_INST_DMAS_CELL_BY_IDX(inst, 0, slot),                      \
			     .fifo_threshold = DT_INST_DMAS_CELL_BY_IDX(inst, 0, fifo_threshold),), \
			    ()) }

#define GD32_SDHC_FIFO_ADDR(inst) (DT_INST_REG_ADDR(inst) + 0x80U)

#define GD32_SDHC_INIT(inst)                                                                       \
	PINCTRL_DT_INST_DEFINE(inst);                                                              \
	GD32_SDHC_IRQ_CONNECT(inst)                                                                \
	static const struct gd32_sdhc_config gd32_sdhc_config_##inst = {                           \
		.clkid = DT_INST_CLOCKS_CELL(inst, id),                                            \
		.pcfg = PINCTRL_DT_INST_DEV_CONFIG_GET(inst),                                      \
		.irq_config_func = gd32_sdhc_irq_config_##inst,                                    \
		.cd_gpio = GPIO_DT_SPEC_INST_GET_OR(inst, cd_gpios, {0}),                          \
		GD32_SDHC_DMA_INIT(inst),                                                          \
		.fifo_addr = GD32_SDHC_FIFO_ADDR(inst),                                            \
		IF_ENABLED(DT_INST_NODE_HAS_PROP(inst, resets), (.reset = RESET_DT_SPEC_INST_GET(inst),)) .bus_width =                       \
				 DT_INST_PROP(inst, bus_width),                                    \
			 .power_delay_ms = DT_INST_PROP(inst, power_delay_ms),                     \
			 .min_bus_freq = DT_INST_PROP(inst, min_bus_freq),                         \
			 .max_bus_freq = DT_INST_PROP(inst, max_bus_freq),                         \
			 .max_current_330 = DT_INST_PROP_OR(inst, max_current_330, 0),             \
			 .max_current_300 = DT_INST_PROP_OR(inst, max_current_300, 0),             \
			 .max_current_180 = DT_INST_PROP_OR(inst, max_current_180, 0),             \
	};                                                                                         \
	static struct gd32_sdhc_data gd32_sdhc_data_##inst;                                        \
	DEVICE_DT_INST_DEFINE(inst, &gd32_sdhc_init, NULL, &gd32_sdhc_data_##inst,                 \
			      &gd32_sdhc_config_##inst, POST_KERNEL, CONFIG_SDHC_INIT_PRIORITY,    \
			      &gd32_sdhc_api);

DT_INST_FOREACH_STATUS_OKAY(GD32_SDHC_INIT)
