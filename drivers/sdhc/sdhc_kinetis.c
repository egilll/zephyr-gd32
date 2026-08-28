/*
 * Copyright (c) 2026 Ylhyra ehf.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT nxp_kinetis_sdhc

#include <errno.h>
#include <limits.h>
#include <stdint.h>
#include <string.h>

#include <zephyr/cache.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/clock_control.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/pinctrl.h>
#include <zephyr/drivers/sdhc.h>
#include <zephyr/dt-bindings/clock/kinetis_sim.h>
#include <zephyr/irq.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/atomic.h>
#include <zephyr/sys/math_extras.h>
#include <zephyr/sys/util.h>

#include <soc.h>
#include <fsl_clock.h>
#include <fsl_sdhc.h>

LOG_MODULE_REGISTER(sdhc_kinetis, CONFIG_SDHC_LOG_LEVEL);

#define KINETIS_SDHC_RESET_TIMEOUT         1000000U
#define KINETIS_SDHC_DEFAULT_TIMEOUT_MS    5000U
#define KINETIS_SDHC_CARD_READY_TIMEOUT_MS 1000U
#define KINETIS_SDHC_RECOVERY_POLL_US      125U
#define KINETIS_SDHC_COMPLETION_POLL_MS    2U
#define KINETIS_SDHC_CLOCK_STABLE_TIMEOUT_US 10000U
#define KINETIS_SDHC_DATA_TIMEOUT_COUNTER  0xEU
#define KINETIS_SDHC_DMA_THRESHOLD_BYTES   512U
#define KINETIS_SDHC_READ_WATERMARK_LEVEL  128U
#define KINETIS_SDHC_WRITE_WATERMARK_LEVEL 128U
#define KINETIS_SDHC_ADMA_ALIGNMENT        8U
#define KINETIS_SDHC_DMA_ALIGNMENT         4U
#define KINETIS_SDHC_DMA_BUF_ALIGNMENT MAX(CONFIG_SDHC_BUFFER_ALIGNMENT, KINETIS_SDHC_DMA_ALIGNMENT)
#define KINETIS_SDHC_TRANSFER_IRQ_FLAGS                                                            \
	((uint32_t)kSDHC_CommandFlag | (uint32_t)kSDHC_DataFlag | (uint32_t)kSDHC_DataDMAFlag)
#define KINETIS_SDHC_BUSY_IRQ_FLAGS                                                                \
	((uint32_t)kSDHC_DataCompleteFlag | (uint32_t)kSDHC_DataErrorFlag)

BUILD_ASSERT((CONFIG_SDHC_BUFFER_ALIGNMENT % KINETIS_SDHC_DMA_ALIGNMENT) == 0U);
BUILD_ASSERT((CONFIG_SDHC_KINETIS_ADMA_TABLE_WORDS % 2U) == 0U);

enum sdhc_kinetis_completion {
	KINETIS_SDHC_REQ_ARMED = 0,
	KINETIS_SDHC_REQ_CMD_DONE = 1,
	KINETIS_SDHC_REQ_DATA_DONE = 2,
};

struct sdhc_kinetis_hw_snapshot {
	uint32_t irqstat;
	uint32_t prsstat;
	uint32_t ac12err;
	uint32_t admaes;
};

struct sdhc_kinetis_req {
	struct sdhc_command *z_cmd;
	struct sdhc_data *z_data;
	uint8_t *buf;
	size_t len;
	atomic_t completion;
	uint32_t transferred_words;
	bool use_dma;
	bool is_write;
	bool is_multiblock;
	bool use_auto_cmd12;
	bool started;
	struct sdhc_kinetis_hw_snapshot snapshot;
	bool first_error_valid;
};

struct sdhc_kinetis_config {
	SDHC_Type *base;
	const struct pinctrl_dev_config *pincfg;
	struct gpio_dt_spec cd_gpio;
	struct gpio_dt_spec pwr_gpio;
	const struct device *clock_dev;
	clock_control_subsys_t clock_subsys;
	uint8_t bus_width;
	uint32_t power_delay_ms;
	uint32_t min_bus_freq;
	uint32_t max_bus_freq;
	void (*irq_config_func)(const struct device *dev);
};

struct sdhc_kinetis_data {
	struct sdhc_io host_io;
	struct k_mutex lock;
	struct k_sem transfer_sem;
	sdhc_interrupt_cb_t card_cb;
	void *card_cb_user_data;
	int enabled_sources;
	struct gpio_callback cd_cb;
	const struct device *dev;
	atomic_t cd_present;
	struct sdhc_kinetis_req request;
	atomic_ptr_t active_req;
#if defined(CONFIG_SDHC_KINETIS_TEST)
	atomic_t test_mask_terminal;
	atomic_t test_force_dma_error;
	atomic_t test_force_pio;
	atomic_t test_force_tc_dtoe;
	atomic_t test_pio_fallbacks;
#endif
	uint32_t adma_table[CONFIG_SDHC_KINETIS_ADMA_TABLE_WORDS]
		__aligned(KINETIS_SDHC_ADMA_ALIGNMENT);
};

BUILD_ASSERT(sizeof(struct sdhc_kinetis_data) <= 256U,
	     "Kinetis SDHC per-instance state exceeded its RAM budget");

#if defined(CONFIG_SDHC_KINETIS_TEST)
void sdhc_kinetis_test_mask_next_terminal_interrupt(const struct device *dev)
{
	struct sdhc_kinetis_data *data = dev->data;

	atomic_set(&data->test_mask_terminal, 1);
}

void sdhc_kinetis_test_force_next_dma_error(const struct device *dev)
{
	struct sdhc_kinetis_data *data = dev->data;

	atomic_set(&data->test_force_dma_error, 1);
}

void sdhc_kinetis_test_force_next_pio(const struct device *dev)
{
	struct sdhc_kinetis_data *data = dev->data;

	atomic_set(&data->test_force_pio, 1);
}

void sdhc_kinetis_test_force_next_tc_dtoe(const struct device *dev)
{
	struct sdhc_kinetis_data *data = dev->data;

	atomic_set(&data->test_force_tc_dtoe, 1);
}

uint32_t sdhc_kinetis_test_pio_fallback_count(const struct device *dev)
{
	struct sdhc_kinetis_data *data = dev->data;

	return (uint32_t)atomic_get(&data->test_pio_fallbacks);
}
#endif

static int sdhc_kinetis_sync_card_interrupts(const struct device *dev);
static void sdhc_kinetis_arm_host_cd_interrupts(const struct device *dev, bool present);
static int sdhc_kinetis_reset_controller_only(const struct device *dev);
static int sdhc_kinetis_restore_host_state(const struct device *dev);
static int sdhc_kinetis_status_to_errno(status_t status,
					const struct sdhc_kinetis_hw_snapshot *snapshot);

static uint32_t sdhc_kinetis_initial_dma_mode(const struct sdhc_kinetis_config *cfg)
{
	return (cfg->base->HTCAPBLT & SDHC_HTCAPBLT_ADMAS_MASK) != 0U ? (uint32_t)kSDHC_DmaModeAdma2
								      : (uint32_t)kSDHC_DmaModeNo;
}

static void sdhc_kinetis_init_host_cfg(const struct sdhc_kinetis_config *cfg,
				       sdhc_config_t *host_cfg)
{
	memset(host_cfg, 0, sizeof(*host_cfg));
	host_cfg->cardDetectDat3 = false;
	host_cfg->endianMode = kSDHC_EndianModeLittle;
	host_cfg->dmaMode = sdhc_kinetis_initial_dma_mode(cfg);
	host_cfg->readWatermarkLevel = KINETIS_SDHC_READ_WATERMARK_LEVEL;
	host_cfg->writeWatermarkLevel = KINETIS_SDHC_WRITE_WATERMARK_LEVEL;
}

static void sdhc_kinetis_take_snapshot(SDHC_Type *base, struct sdhc_kinetis_hw_snapshot *snapshot)
{
	snapshot->irqstat = SDHC_GetInterruptStatusFlags(base);
	snapshot->prsstat = SDHC_GetPresentStatusFlags(base);
	snapshot->ac12err = base->AC12ERR;
	snapshot->admaes = base->ADMAES;
}

static void sdhc_kinetis_req_record_snapshot(struct sdhc_kinetis_req *req, SDHC_Type *base)
{
	struct sdhc_kinetis_hw_snapshot snapshot;

	sdhc_kinetis_take_snapshot(base, &snapshot);
	if (((snapshot.irqstat & (uint32_t)kSDHC_ErrorFlag) != 0U) &&
	    !req->first_error_valid) {
		req->snapshot = snapshot;
		req->first_error_valid = true;
	} else if (!req->first_error_valid) {
		req->snapshot = snapshot;
	}
}

static const struct sdhc_kinetis_hw_snapshot *
sdhc_kinetis_primary_snapshot(const struct sdhc_kinetis_req *req)
{
	return &req->snapshot;
}

static bool sdhc_kinetis_complete_request(struct sdhc_kinetis_req *req, int completion)
{
	__ASSERT(completion != KINETIS_SDHC_REQ_ARMED, "completion must be terminal");

	return atomic_cas(&req->completion, KINETIS_SDHC_REQ_ARMED, completion);
}

static void sdhc_kinetis_disable_transfer_irqs(SDHC_Type *base)
{
	SDHC_DisableInterruptSignal(base, KINETIS_SDHC_TRANSFER_IRQ_FLAGS);
}

static bool sdhc_kinetis_cmd_is_write(const struct sdhc_command *cmd)
{
	switch (cmd->opcode) {
	case SD_WRITE_SINGLE_BLOCK:
	case SD_WRITE_MULTIPLE_BLOCK:
		return true;
	case SDIO_RW_EXTENDED:
		return (cmd->arg & BIT(31)) != 0U;
	default:
		return false;
	}
}

static bool sdhc_kinetis_cmd_is_read(const struct sdhc_command *cmd)
{
	switch (cmd->opcode) {
	case SD_READ_SINGLE_BLOCK:
	case SD_READ_MULTIPLE_BLOCK:
	case SD_SWITCH:
	case SD_SEND_TUNING_BLOCK:
	case MMC_SEND_TUNING_BLOCK:
	case MMC_SEND_EXT_CSD:
	case SD_APP_SEND_SCR:
	case SD_APP_SEND_NUM_WRITTEN_BLK:
	case MMC_CHECK_BUS_TEST:
		return true;
	case SDIO_RW_EXTENDED:
		return (cmd->arg & BIT(31)) == 0U;
	default:
		return false;
	}
}

static bool sdhc_kinetis_cmd_allowed_while_dat_busy(const struct sdhc_command *cmd)
{
	switch (cmd->opcode) {
	case SD_GO_IDLE_STATE:
	case SD_STOP_TRANSMISSION:
	case SD_SEND_STATUS:
	case SDIO_RW_DIRECT:
		return true;
	default:
		return false;
	}
}

static bool sdhc_kinetis_cmd_has_busy_response(const struct sdhc_kinetis_req *req)
{
	uint32_t response_type = req->z_cmd->response_type & SDHC_NATIVE_RESPONSE_MASK;

	return (response_type == SD_RSP_TYPE_R1b) || (response_type == SD_RSP_TYPE_R5b);
}

static bool sdhc_kinetis_is_multiblock_rw(const struct sdhc_command *cmd,
					  const struct sdhc_data *data)
{
	if ((data == NULL) || (data->blocks <= 1U)) {
		return false;
	}

	return (cmd->opcode == SD_READ_MULTIPLE_BLOCK) || (cmd->opcode == SD_WRITE_MULTIPLE_BLOCK);
}

static bool sdhc_kinetis_use_auto_cmd12(const struct sdhc_command *cmd,
					const struct sdhc_data *data)
{
	return sdhc_kinetis_is_multiblock_rw(cmd, data);
}

static bool sdhc_kinetis_dma_buf_usable(const void *buf, size_t len)
{
	return IS_ALIGNED(buf, KINETIS_SDHC_DMA_BUF_ALIGNMENT) &&
	       ((len % KINETIS_SDHC_DMA_ALIGNMENT) == 0U);
}

static uint32_t sdhc_kinetis_adma_capacity_bytes(void)
{
	uint32_t entries = (CONFIG_SDHC_KINETIS_ADMA_TABLE_WORDS * sizeof(uint32_t)) / 8U;

	return (entries - 1U) * SDHC_ADMA2_DESCRIPTOR_MAX_LENGTH_PER_ENTRY;
}

static bool sdhc_kinetis_request_prefers_dma(const struct sdhc_kinetis_config *cfg,
					     const struct sdhc_command *cmd,
					     const struct sdhc_data *data)
{
	size_t bytes;
	bool write;
	bool read;

	if ((data == NULL) || (sdhc_kinetis_initial_dma_mode(cfg) == (uint32_t)kSDHC_DmaModeNo)) {
		return false;
	}

	write = sdhc_kinetis_cmd_is_write(cmd);
	read = sdhc_kinetis_cmd_is_read(cmd);
	if (!read && !write) {
		return false;
	}

	bytes = (size_t)data->blocks * (size_t)data->block_size;
	if (bytes < KINETIS_SDHC_DMA_THRESHOLD_BYTES) {
		return false;
	}

	if (bytes > sdhc_kinetis_adma_capacity_bytes()) {
		return false;
	}

	return sdhc_kinetis_dma_buf_usable(data->data, bytes);
}

static int sdhc_kinetis_cache_result(int ret)
{
	return (ret == -ENOTSUP) ? 0 : ret;
}

static void sdhc_kinetis_set_dma_mode(SDHC_Type *base, sdhc_dma_mode_t mode)
{
	base->PROCTL = (base->PROCTL & ~SDHC_PROCTL_DMAS_MASK) | SDHC_PROCTL_DMAS(mode);
}

static int sdhc_kinetis_set_power(const struct device *dev, enum sdhc_power power_mode)
{
	const struct sdhc_kinetis_config *cfg = dev->config;
	struct sdhc_kinetis_data *data = dev->data;
	int ret;

	if ((power_mode == 0U) || (power_mode == data->host_io.power_mode)) {
		return 0;
	}

	switch (power_mode) {
	case SDHC_POWER_OFF:
		SDHC_EnableSdClock(cfg->base, false);
		if (cfg->pwr_gpio.port != NULL) {
			ret = gpio_pin_set_dt(&cfg->pwr_gpio, 0);
			if (ret != 0) {
				return ret;
			}
		}
		break;
	case SDHC_POWER_ON:
		if (cfg->pwr_gpio.port != NULL) {
			ret = gpio_pin_set_dt(&cfg->pwr_gpio, 1);
			if (ret != 0) {
				return ret;
			}

			if (cfg->power_delay_ms != 0U) {
				k_msleep(cfg->power_delay_ms);
			}
		}
		break;
	default:
		return -ENOTSUP;
	}

	data->host_io.power_mode = power_mode;

	return 0;
}

static int sdhc_kinetis_wait_present_state(const struct device *dev, uint32_t mask, bool set,
					   uint32_t timeout_ms)
{
	const struct sdhc_kinetis_config *cfg = dev->config;
	int64_t deadline = (timeout_ms == SDHC_TIMEOUT_FOREVER)
				   ? INT64_MAX
				   : (k_uptime_get() + (int64_t)timeout_ms);

	while (true) {
		uint32_t prsstat = SDHC_GetPresentStatusFlags(cfg->base);
		bool matched = ((prsstat & mask) == mask);

		if (matched == set) {
			return 0;
		}

		if ((timeout_ms != SDHC_TIMEOUT_FOREVER) && (k_uptime_get() >= deadline)) {
			return -ETIMEDOUT;
		}

		k_sleep(K_USEC(KINETIS_SDHC_RECOVERY_POLL_US));
	}
}

static uint32_t sdhc_kinetis_src_clock_hz(void)
{
	uint32_t sel = (SIM->SOPT2 & SIM_SOPT2_SDHCSRC_MASK) >> SIM_SOPT2_SDHCSRC_SHIFT;

	switch (sel) {
	case 0U:
		return CLOCK_GetFreq(kCLOCK_CoreSysClk);
	case 1U:
		return CLOCK_GetFreq(kCLOCK_PllFllSelClk);
	case 2U:
		return CLOCK_GetFreq(kCLOCK_Osc0ErClk);
	case 3U:
	default:
		return 0U;
	}
}

static int sdhc_kinetis_set_clock(SDHC_Type *base, uint32_t src_clock_hz,
				   uint32_t requested_hz, uint32_t *actual_hz)
{
	uint32_t best_hz = 0U;
	uint32_t best_divisor = 0U;
	uint32_t best_prescaler = 0U;
	uint32_t sysctl;

	if ((src_clock_hz == 0U) || (requested_hz == 0U) || (requested_hz > src_clock_hz)) {
		return -EINVAL;
	}

	for (uint32_t prescaler = 1U; prescaler <= 256U; prescaler <<= 1U) {
		for (uint32_t divisor = 1U; divisor <= 16U; divisor++) {
			uint32_t candidate_hz = src_clock_hz / prescaler / divisor;

			if ((candidate_hz <= requested_hz) && (candidate_hz > best_hz)) {
				best_hz = candidate_hz;
				best_divisor = divisor;
				best_prescaler = prescaler;
			}
		}
	}

	if (best_hz == 0U) {
		return -ENOTSUP;
	}

	base->SYSCTL &= ~SDHC_SYSCTL_SDCLKEN_MASK;
	sysctl = base->SYSCTL;
	sysctl &= ~(SDHC_SYSCTL_DVS_MASK | SDHC_SYSCTL_SDCLKFS_MASK | SDHC_SYSCTL_DTOCV_MASK);
	sysctl |= SDHC_SYSCTL_DVS(best_divisor - 1U) |
		  SDHC_SYSCTL_SDCLKFS(best_prescaler == 1U ? 0U : best_prescaler / 2U) |
		  SDHC_SYSCTL_DTOCV(KINETIS_SDHC_DATA_TIMEOUT_COUNTER);
	base->SYSCTL = sysctl;

	for (uint32_t elapsed_us = 0U; elapsed_us < KINETIS_SDHC_CLOCK_STABLE_TIMEOUT_US;
	     elapsed_us++) {
		if ((base->PRSSTAT & SDHC_PRSSTAT_SDSTB_MASK) != 0U) {
			base->SYSCTL |= SDHC_SYSCTL_SDCLKEN_MASK;
			*actual_hz = best_hz;
			return 0;
		}
		k_busy_wait(1U);
	}

	return -ETIMEDOUT;
}

static uint32_t sdhc_kinetis_max_blk_len(size_t max_block_len)
{
	switch (max_block_len) {
	case 512U:
	default:
		return 0U;
	case 1024U:
		return 1U;
	case 2048U:
		return 2U;
	case 4096U:
		return 3U;
	}
}

static int sdhc_kinetis_validate_io(const struct sdhc_io *ios)
{
	if ((ios->timing != 0) && (ios->timing != SDHC_TIMING_LEGACY) &&
	    (ios->timing != SDHC_TIMING_HS)) {
		return -ENOTSUP;
	}

	if ((ios->bus_mode != 0) && (ios->bus_mode != SDHC_BUSMODE_PUSHPULL)) {
		return -ENOTSUP;
	}

	if ((ios->signal_voltage != 0) && (ios->signal_voltage != SD_VOL_3_3_V)) {
		return -ENOTSUP;
	}

	if ((ios->power_mode != 0U) && (ios->power_mode != SDHC_POWER_OFF) &&
	    (ios->power_mode != SDHC_POWER_ON)) {
		return -ENOTSUP;
	}

	return 0;
}

static int sdhc_kinetis_apply_runtime_io(const struct device *dev, const struct sdhc_io *ios)
{
	const struct sdhc_kinetis_config *cfg = dev->config;
	uint32_t actual_clock_hz;
	uint32_t src_clk_hz;
	uint8_t bus_width;

	src_clk_hz = sdhc_kinetis_src_clock_hz();
	if (src_clk_hz == 0U) {
		LOG_ERR("SDHC source clock is unavailable");
		return -ENOTSUP;
	}

	if (ios->clock == 0U) {
		SDHC_EnableSdClock(cfg->base, false);
	} else if ((ios->clock >= cfg->min_bus_freq) && (ios->clock <= cfg->max_bus_freq)) {
		if (ios->clock > src_clk_hz) {
			LOG_ERR("SDHC clock request %u exceeds source %u", ios->clock, src_clk_hz);
			return -ENOTSUP;
		}
		if (sdhc_kinetis_set_clock(cfg->base, src_clk_hz, ios->clock,
					    &actual_clock_hz) != 0) {
			LOG_ERR("SDHC clock did not stabilize for request %u from source %u",
				ios->clock, src_clk_hz);
			return -ETIMEDOUT;
		}
		if (actual_clock_hz > ios->clock) {
			LOG_ERR("SDHC clock request %u produced invalid rate %u from source %u",
				ios->clock, actual_clock_hz, src_clk_hz);
			return -ENOTSUP;
		}
	} else {
		LOG_ERR("SDHC clock request %u is outside %u..%u", ios->clock,
			cfg->min_bus_freq, cfg->max_bus_freq);
		return -ENOTSUP;
	}

	bus_width = (ios->bus_width == 0U) ? SDHC_BUS_WIDTH1BIT : ios->bus_width;

	switch (bus_width) {
	case SDHC_BUS_WIDTH1BIT:
		SDHC_SetDataBusWidth(cfg->base, kSDHC_DataBusWidth1Bit);
		break;
	case SDHC_BUS_WIDTH4BIT:
		if (cfg->bus_width < 4U) {
			LOG_ERR("4-bit bus requested on %u-bit host", cfg->bus_width);
			return -ENOTSUP;
		}
		SDHC_SetDataBusWidth(cfg->base, kSDHC_DataBusWidth4Bit);
		break;
	case SDHC_BUS_WIDTH8BIT:
		if (cfg->bus_width < 8U) {
			LOG_ERR("8-bit bus requested on %u-bit host", cfg->bus_width);
			return -ENOTSUP;
		}
		SDHC_SetDataBusWidth(cfg->base, kSDHC_DataBusWidth8Bit);
		break;
	default:
		LOG_ERR("unsupported SDHC bus width %u", bus_width);
		return -ENOTSUP;
	}

	return 0;
}

static int sdhc_kinetis_apply_io(const struct device *dev, const struct sdhc_io *ios)
{
	const struct sdhc_kinetis_config *cfg = dev->config;
	struct sdhc_kinetis_data *data = dev->data;
	bool powering_on = (ios->power_mode == SDHC_POWER_ON) &&
			   (data->host_io.power_mode != SDHC_POWER_ON);
	bool powering_off = (ios->power_mode == SDHC_POWER_OFF) &&
			    (data->host_io.power_mode != SDHC_POWER_OFF);
	int ret;

	LOG_DBG("set_io clk=%u width=%u power=%u timing=%u volt=%u", ios->clock, ios->bus_width,
		ios->power_mode, ios->timing, ios->signal_voltage);

	ret = sdhc_kinetis_validate_io(ios);
	if (ret != 0) {
		LOG_ERR("invalid SDHC I/O request clk=%u width=%u power=%u timing=%u voltage=%u",
			ios->clock, ios->bus_width, ios->power_mode, ios->timing,
			ios->signal_voltage);
		return ret;
	}

	if (powering_off) {
		atomic_ptr_clear(&data->active_req);
		sdhc_kinetis_disable_transfer_irqs(cfg->base);
		SDHC_ClearInterruptStatusFlags(cfg->base, kSDHC_AllInterruptFlags);
		ret = sdhc_kinetis_set_power(dev, SDHC_POWER_OFF);
		if (ret != 0) {
			LOG_ERR("failed to power off SDHC: %d", ret);
			return ret;
		}

		data->host_io = *ios;
		data->host_io.clock = 0U;

		ret = sdhc_kinetis_sync_card_interrupts(dev);
		if (ret != 0) {
			LOG_ERR("failed to synchronize SDHC card interrupts after power off: %d", ret);
		}
		return ret;
	}

	if (powering_on) {
		ret = sdhc_kinetis_set_power(dev, SDHC_POWER_ON);
		if (ret != 0) {
			LOG_ERR("failed to power on SDHC: %d", ret);
			return ret;
		}

		ret = sdhc_kinetis_reset_controller_only(dev);
		if (ret != 0) {
			LOG_ERR("failed to reset SDHC after power on: %d", ret);
			return ret;
		}
	}

	ret = sdhc_kinetis_apply_runtime_io(dev, ios);
	if (ret != 0) {
		return ret;
	}

	data->host_io = *ios;

	return 0;
}

static void sdhc_kinetis_disable_host_cd_interrupts(const struct device *dev)
{
	const struct sdhc_kinetis_config *cfg = dev->config;

	SDHC_DisableInterruptSignal(cfg->base, (uint32_t)kSDHC_CardDetectFlag);
	SDHC_DisableInterruptStatus(cfg->base, (uint32_t)kSDHC_CardDetectFlag);
	SDHC_ClearInterruptStatusFlags(cfg->base, (uint32_t)kSDHC_CardDetectFlag);
}

static void sdhc_kinetis_arm_host_cd_interrupts(const struct device *dev, bool present)
{
	const struct sdhc_kinetis_config *cfg = dev->config;

	sdhc_kinetis_disable_host_cd_interrupts(dev);

	if (!present) {
		SDHC_EnableInterruptStatus(cfg->base, (uint32_t)kSDHC_CardInsertionFlag);
		SDHC_EnableInterruptSignal(cfg->base, (uint32_t)kSDHC_CardInsertionFlag);
	}

	if (present) {
		SDHC_EnableInterruptStatus(cfg->base, (uint32_t)kSDHC_CardRemovalFlag);
		SDHC_EnableInterruptSignal(cfg->base, (uint32_t)kSDHC_CardRemovalFlag);
	}
}

static int sdhc_kinetis_sync_card_interrupts(const struct device *dev)
{
	const struct sdhc_kinetis_config *cfg = dev->config;
	struct sdhc_kinetis_data *data = dev->data;

	SDHC_DisableInterruptSignal(cfg->base, (uint32_t)kSDHC_CardInterruptFlag);
	SDHC_DisableInterruptStatus(cfg->base, (uint32_t)kSDHC_CardInterruptFlag);
	SDHC_ClearInterruptStatusFlags(cfg->base, (uint32_t)kSDHC_CardInterruptFlag);

	if (((data->enabled_sources & SDHC_INT_SDIO) != 0) && (data->card_cb != NULL)) {
		SDHC_EnableInterruptStatus(cfg->base, (uint32_t)kSDHC_CardInterruptFlag);
		SDHC_EnableInterruptSignal(cfg->base, (uint32_t)kSDHC_CardInterruptFlag);
	}

	if (cfg->cd_gpio.port != NULL) {
		return gpio_pin_interrupt_configure_dt(&cfg->cd_gpio, GPIO_INT_EDGE_BOTH);
	}

	atomic_set(&data->cd_present,
		   (SDHC_GetPresentStatusFlags(cfg->base) & (uint32_t)kSDHC_CardInsertedFlag) !=
			   0U);
	sdhc_kinetis_arm_host_cd_interrupts(dev, atomic_get(&data->cd_present) != 0);

	return 0;
}

static int sdhc_kinetis_init_controller(const struct device *dev)
{
	const struct sdhc_kinetis_config *cfg = dev->config;
	sdhc_config_t host_cfg;
	uint32_t proctl;
	uint32_t wml;

	sdhc_kinetis_init_host_cfg(cfg, &host_cfg);
	if (!SDHC_Reset(cfg->base, (uint32_t)kSDHC_ResetAll, KINETIS_SDHC_RESET_TIMEOUT)) {
		return -ETIMEDOUT;
	}

	proctl = cfg->base->PROCTL;
	proctl &= ~(SDHC_PROCTL_D3CD_MASK | SDHC_PROCTL_EMODE_MASK | SDHC_PROCTL_DMAS_MASK);
	proctl |= SDHC_PROCTL_EMODE(host_cfg.endianMode) | SDHC_PROCTL_DMAS(host_cfg.dmaMode);

	wml = cfg->base->WML;
	wml &= ~(SDHC_WML_RDWML_MASK | SDHC_WML_WRWML_MASK);
	wml |= SDHC_WML_RDWML(host_cfg.readWatermarkLevel) |
	       SDHC_WML_WRWML(host_cfg.writeWatermarkLevel);

	cfg->base->WML = wml;
	cfg->base->PROCTL = proctl;
	cfg->base->SYSCTL |=
		SDHC_SYSCTL_PEREN_MASK | SDHC_SYSCTL_HCKEN_MASK | SDHC_SYSCTL_IPGEN_MASK;
	SDHC_DisableInterruptSignal(cfg->base, kSDHC_AllInterruptFlags);
	SDHC_EnableInterruptStatus(cfg->base, kSDHC_AllInterruptFlags);
	SDHC_ClearInterruptStatusFlags(cfg->base, kSDHC_AllInterruptFlags);

	return 0;
}

static int sdhc_kinetis_reset_controller_only(const struct device *dev)
{
	const struct sdhc_kinetis_config *cfg = dev->config;
	struct sdhc_kinetis_data *data = dev->data;
	int ret;

	atomic_ptr_clear(&data->active_req);
	sdhc_kinetis_disable_transfer_irqs(cfg->base);
	ret = sdhc_kinetis_init_controller(dev);

	if (ret != 0) {
		return ret;
	}

	return sdhc_kinetis_sync_card_interrupts(dev);
}

static int sdhc_kinetis_restore_host_state(const struct device *dev)
{
	struct sdhc_kinetis_data *data = dev->data;
	int ret;

	ret = sdhc_kinetis_reset_controller_only(dev);
	if (ret != 0) {
		return ret;
	}

	if (data->host_io.power_mode == SDHC_POWER_ON) {
		ret = sdhc_kinetis_apply_runtime_io(dev, &data->host_io);
		if (ret != 0) {
			return ret;
		}
	}

	return sdhc_kinetis_sync_card_interrupts(dev);
}

static int sdhc_kinetis_reset_host(const struct device *dev)
{
	return sdhc_kinetis_restore_host_state(dev);
}

static uint32_t sdhc_kinetis_request_timeout_ms(const struct sdhc_command *cmd,
						const struct sdhc_data *data)
{
	uint32_t timeout_ms = cmd->timeout_ms;

	if ((data != NULL) && (data->timeout_ms == SDHC_TIMEOUT_FOREVER)) {
		return SDHC_TIMEOUT_FOREVER;
	}

	if (cmd->timeout_ms == SDHC_TIMEOUT_FOREVER) {
		return SDHC_TIMEOUT_FOREVER;
	}

	if ((data != NULL) && (data->timeout_ms != 0U)) {
		timeout_ms = data->timeout_ms;
	}

	return timeout_ms != 0U ? timeout_ms : KINETIS_SDHC_DEFAULT_TIMEOUT_MS;
}

static uint32_t sdhc_kinetis_finite_request_timeout_ms(const struct sdhc_command *cmd,
						       const struct sdhc_data *data)
{
	uint32_t timeout_ms = sdhc_kinetis_request_timeout_ms(cmd, data);

	return timeout_ms == SDHC_TIMEOUT_FOREVER ? KINETIS_SDHC_DEFAULT_TIMEOUT_MS : timeout_ms;
}

static int64_t sdhc_kinetis_deadline_after(uint64_t budget_ms)
{
	return k_uptime_get() + (int64_t)MIN(budget_ms, (uint64_t)INT64_MAX / 2U);
}

static int64_t sdhc_kinetis_operation_deadline(uint32_t timeout_ms, uint32_t retries)
{
	uint64_t attempts = (uint64_t)retries + 1U;
	uint64_t per_attempt_ms = (uint64_t)timeout_ms + KINETIS_SDHC_CARD_READY_TIMEOUT_MS;
	uint64_t budget_ms;

	if (u64_mul_overflow(per_attempt_ms, attempts, &budget_ms)) {
		budget_ms = UINT64_MAX;
	}

	return sdhc_kinetis_deadline_after(budget_ms);
}

static uint32_t sdhc_kinetis_timeout_before_deadline(uint32_t requested_ms,
						      int64_t deadline)
{
	int64_t remaining_ms = deadline - k_uptime_get();

	if (remaining_ms <= 0) {
		return 0U;
	}
	if (remaining_ms > UINT32_MAX) {
		remaining_ms = UINT32_MAX;
	}

	return MIN((uint32_t)remaining_ms, requested_ms);
}

static int sdhc_kinetis_xfer_error(const struct sdhc_kinetis_hw_snapshot *snapshot)
{
	if ((snapshot->irqstat &
	     ((uint32_t)kSDHC_CommandTimeoutFlag | (uint32_t)kSDHC_DataTimeoutFlag)) != 0U) {
		return -ETIMEDOUT;
	}

	if (((snapshot->irqstat & (uint32_t)kSDHC_AutoCommand12ErrorFlag) != 0U) &&
	    ((snapshot->ac12err & (uint32_t)kSDHC_AutoCommand12TimeoutFlag) != 0U)) {
		return -ETIMEDOUT;
	}

	return -EIO;
}

static int sdhc_kinetis_status_to_errno(status_t status,
					const struct sdhc_kinetis_hw_snapshot *snapshot)
{
	switch ((uint32_t)status) {
	case (uint32_t)kStatus_Success:
	case (uint32_t)kStatus_SDHC_TransferCommandComplete:
	case (uint32_t)kStatus_SDHC_TransferDataComplete:
		return 0;
	case (uint32_t)kStatus_SDHC_BusyTransferring:
		return -EBUSY;
	case (uint32_t)kStatus_InvalidArgument:
	case (uint32_t)kStatus_SDHC_DMADataBufferAddrNotAlign:
	case (uint32_t)kStatus_OutOfRange:
		return -EINVAL;
	case (uint32_t)kStatus_SDHC_PrepareAdmaDescriptorFailed:
		return -EIO;
	case (uint32_t)kStatus_SDHC_SendCommandFailed:
	case (uint32_t)kStatus_SDHC_TransferDataFailed:
	default:
		return sdhc_kinetis_xfer_error(snapshot);
	}
}

static void sdhc_kinetis_fail_request(const struct device *dev, struct sdhc_kinetis_req *req,
				      int ret)
{
	const struct sdhc_kinetis_config *cfg = dev->config;
	struct sdhc_kinetis_data *data = dev->data;

	if (!sdhc_kinetis_complete_request(req, ret)) {
		return;
	}

	sdhc_kinetis_disable_transfer_irqs(cfg->base);
	k_sem_give(&data->transfer_sem);
}

static void sdhc_kinetis_receive_response(SDHC_Type *base, struct sdhc_command *command)
{
	uint32_t response0 = SDHC_GetCommandResponse(base, 0U);
	uint32_t response1 = SDHC_GetCommandResponse(base, 1U);
	uint32_t response2 = SDHC_GetCommandResponse(base, 2U);
	uint32_t response_type = command->response_type & SDHC_NATIVE_RESPONSE_MASK;

	if (response_type != SD_RSP_TYPE_NONE) {
		command->response[0] = response0;
		if (response_type == SD_RSP_TYPE_R2) {
			command->response[0] <<= 8U;
			command->response[1] =
				(response1 << 8U) | ((response0 & 0xFF000000U) >> 24U);
			command->response[2] =
				(response2 << 8U) | ((response1 & 0xFF000000U) >> 24U);
			command->response[3] =
				(SDHC_GetCommandResponse(base, 3U) << 8U) |
				((response2 & 0xFF000000U) >> 24U);
		}
	}
}

static uint32_t sdhc_kinetis_service_pio(SDHC_Type *base, struct sdhc_kinetis_req *req,
					 uint32_t irqstat)
{
	uint32_t ready_flag = req->is_write ? (uint32_t)kSDHC_BufferWriteReadyFlag
					    : (uint32_t)kSDHC_BufferReadReadyFlag;
	uint32_t watermark = req->is_write
				     ? (base->WML & SDHC_WML_WRWML_MASK) >> SDHC_WML_WRWML_SHIFT
				     : (base->WML & SDHC_WML_RDWML_MASK) >> SDHC_WML_RDWML_SHIFT;
	uint32_t total_words = DIV_ROUND_UP(req->len, sizeof(uint32_t));
	uint32_t words;

	if ((irqstat & ready_flag) == 0U) {
		return 0U;
	}
	if (req->transferred_words >= total_words) {
		return ready_flag;
	}
	words = MIN(watermark, total_words - req->transferred_words);

	for (uint32_t i = 0U; i < words; i++) {
		size_t offset = (size_t)req->transferred_words * sizeof(uint32_t);
		size_t bytes = MIN(sizeof(uint32_t), req->len - offset);
		uint32_t word = 0U;

		if (req->is_write) {
			memcpy(&word, req->buf + offset, bytes);
			SDHC_WriteData(base, word);
		} else {
			word = SDHC_ReadData(base);
			memcpy(req->buf + offset, &word, bytes);
		}
		req->transferred_words++;
	}

	return ready_flag;
}

static void sdhc_kinetis_harvest_terminal(const struct device *dev,
					  struct sdhc_kinetis_req *req)
{
	const struct sdhc_kinetis_config *cfg = dev->config;
	struct sdhc_kinetis_data *data = dev->data;
	uint32_t irqstat = SDHC_GetInterruptStatusFlags(cfg->base);
	uint32_t data_errors =
		irqstat & ((uint32_t)kSDHC_DataErrorFlag | (uint32_t)kSDHC_DmaErrorFlag);
	uint32_t cmd_errors = irqstat & (uint32_t)kSDHC_CommandErrorFlag;
	bool has_data_phase = (req->z_data != NULL) || sdhc_kinetis_cmd_has_busy_response(req);
	bool transfer_complete;
	bool transfer_won;
	uint32_t clear_flags = 0U;

#if defined(CONFIG_SDHC_KINETIS_TEST)
	if ((req->z_data != NULL) &&
	    ((irqstat & (uint32_t)kSDHC_DataCompleteFlag) != 0U) &&
	    atomic_cas(&data->test_force_tc_dtoe, 1, 0)) {
		SDHC_SetForceEvent(cfg->base, (uint32_t)kSDHC_ForceEventDataTimeout);
		irqstat = SDHC_GetInterruptStatusFlags(cfg->base);
		data_errors = irqstat &
			      ((uint32_t)kSDHC_DataErrorFlag | (uint32_t)kSDHC_DmaErrorFlag);
		cmd_errors = irqstat & (uint32_t)kSDHC_CommandErrorFlag;
	}
#endif
	transfer_complete = has_data_phase &&
			    ((irqstat & (uint32_t)kSDHC_DataCompleteFlag) != 0U);
	transfer_won = transfer_complete && (cmd_errors == 0U) &&
		       ((data_errors == 0U) ||
			(data_errors == (uint32_t)kSDHC_DataTimeoutFlag));

	if (transfer_won) {
		sdhc_kinetis_receive_response(cfg->base, req->z_cmd);
		if (sdhc_kinetis_complete_request(
			    req, req->z_data == NULL ? KINETIS_SDHC_REQ_CMD_DONE
						     : KINETIS_SDHC_REQ_DATA_DONE)) {
			sdhc_kinetis_disable_transfer_irqs(cfg->base);
			k_sem_give(&data->transfer_sem);
		}
		clear_flags = (uint32_t)kSDHC_CommandFlag | (uint32_t)kSDHC_DataCompleteFlag |
			      data_errors;
	} else if ((data_errors != 0U) || (cmd_errors != 0U)) {
		sdhc_kinetis_req_record_snapshot(req, cfg->base);
		sdhc_kinetis_fail_request(
			dev, req, sdhc_kinetis_xfer_error(sdhc_kinetis_primary_snapshot(req)));
		clear_flags = data_errors | cmd_errors |
			      (irqstat & (uint32_t)kSDHC_DataCompleteFlag);
	} else if ((irqstat & (uint32_t)kSDHC_CommandCompleteFlag) != 0U) {
		sdhc_kinetis_receive_response(cfg->base, req->z_cmd);
		if (!has_data_phase &&
		    sdhc_kinetis_complete_request(req, KINETIS_SDHC_REQ_CMD_DONE)) {
			sdhc_kinetis_disable_transfer_irqs(cfg->base);
			k_sem_give(&data->transfer_sem);
		}
		clear_flags = (uint32_t)kSDHC_CommandFlag;
	}

	if (clear_flags != 0U) {
		SDHC_ClearInterruptStatusFlags(cfg->base, clear_flags);
	}
}

static void sdhc_kinetis_service_irqs(const struct device *dev)
{
	const struct sdhc_kinetis_config *cfg = dev->config;
	struct sdhc_kinetis_data *data = dev->data;
	struct sdhc_kinetis_req *req = atomic_ptr_get(&data->active_req);
	uint32_t irqstat;
	uint32_t clear_flags;

	if ((req == NULL) ||
	    (atomic_get(&req->completion) != KINETIS_SDHC_REQ_ARMED)) {
		return;
	}

	irqstat = SDHC_GetInterruptStatusFlags(cfg->base);
	if (!req->use_dma && (req->z_data != NULL) &&
	    ((irqstat & ((uint32_t)kSDHC_DataErrorFlag | (uint32_t)kSDHC_DmaErrorFlag)) ==
	     0U)) {
		clear_flags = sdhc_kinetis_service_pio(cfg->base, req, irqstat);
		if (clear_flags != 0U) {
			SDHC_ClearInterruptStatusFlags(cfg->base, clear_flags);
		}
	}

	sdhc_kinetis_harvest_terminal(dev, req);
}

static int sdhc_kinetis_prepare_data_cache(const struct sdhc_kinetis_req *req)
{
	if (!req->use_dma || (req->buf == NULL) || (req->len == 0U)) {
		return 0;
	}

	if (req->is_write) {
		return sdhc_kinetis_cache_result(sys_cache_data_flush_range(req->buf, req->len));
	}

	return sdhc_kinetis_cache_result(sys_cache_data_flush_and_invd_range(req->buf, req->len));
}

static int sdhc_kinetis_complete_data_cache(const struct sdhc_kinetis_req *req, bool success)
{
	if (!req->use_dma || !success || req->is_write || (req->buf == NULL) || (req->len == 0U)) {
		return 0;
	}

	return sdhc_kinetis_cache_result(sys_cache_data_invd_range(req->buf, req->len));
}

static int sdhc_kinetis_prepare_adma_table(const struct device *dev, struct sdhc_kinetis_req *req)
{
	const struct sdhc_kinetis_config *cfg = dev->config;
	struct sdhc_kinetis_data *data = dev->data;
	status_t status;

	status = SDHC_SetAdmaTableConfig(cfg->base, kSDHC_DmaModeAdma2, data->adma_table,
					 ARRAY_SIZE(data->adma_table), (const uint32_t *)req->buf,
					 req->len);
	if (status != kStatus_Success) {
		sdhc_kinetis_take_snapshot(cfg->base, &req->snapshot);
		return sdhc_kinetis_status_to_errno(status, &req->snapshot);
	}

	return sdhc_kinetis_cache_result(
		sys_cache_data_flush_range(data->adma_table, sizeof(data->adma_table)));
}

static bool sdhc_kinetis_req_terminal(const struct sdhc_kinetis_req *req)
{
	return atomic_get(&req->completion) != KINETIS_SDHC_REQ_ARMED;
}

static int sdhc_kinetis_prepare_request(struct sdhc_kinetis_req *req, struct sdhc_command *cmd,
					struct sdhc_data *data, bool use_dma)
{
	bool read;
	bool write;

	memset(req, 0, sizeof(*req));

	req->z_cmd = cmd;
	req->z_data = data;
	if ((cmd->response_type & SDHC_NATIVE_RESPONSE_MASK) > SD_RSP_TYPE_R7) {
		return -ENOTSUP;
	}

	if (data == NULL) {
		return 0;
	}

	read = sdhc_kinetis_cmd_is_read(cmd);
	write = sdhc_kinetis_cmd_is_write(cmd);
	if (!read && !write) {
		return -ENOTSUP;
	}

	req->len = (size_t)data->blocks * (size_t)data->block_size;
	if ((data->data == NULL) || (req->len == 0U) || (data->blocks == 0U) ||
	    (data->blocks > SDHC_MAX_BLOCK_COUNT) || (data->block_size == 0U) ||
	    (data->block_size > (SDHC_BLKATTR_BLKSIZE_MASK >> SDHC_BLKATTR_BLKSIZE_SHIFT))) {
		return -EINVAL;
	}

	req->buf = data->data;
	req->use_dma = use_dma;
	req->is_write = write;
	req->is_multiblock = sdhc_kinetis_is_multiblock_rw(cmd, data);
	req->use_auto_cmd12 = sdhc_kinetis_use_auto_cmd12(cmd, data);

	return 0;
}

static uint32_t sdhc_kinetis_response_flags(uint32_t response_type)
{
	switch (response_type & SDHC_NATIVE_RESPONSE_MASK) {
	case SD_RSP_TYPE_NONE:
		return 0U;
	case SD_RSP_TYPE_R2:
		return (uint32_t)kSDHC_ResponseLength136Flag |
		       (uint32_t)kSDHC_EnableCrcCheckFlag;
	case SD_RSP_TYPE_R3:
	case SD_RSP_TYPE_R4:
		return (uint32_t)kSDHC_ResponseLength48Flag;
	case SD_RSP_TYPE_R1b:
	case SD_RSP_TYPE_R5b:
		return (uint32_t)kSDHC_ResponseLength48BusyFlag |
		       (uint32_t)kSDHC_EnableCrcCheckFlag |
		       (uint32_t)kSDHC_EnableIndexCheckFlag;
	case SD_RSP_TYPE_R1:
	case SD_RSP_TYPE_R5:
	case SD_RSP_TYPE_R6:
	case SD_RSP_TYPE_R7:
	default:
		return (uint32_t)kSDHC_ResponseLength48Flag |
		       (uint32_t)kSDHC_EnableCrcCheckFlag |
		       (uint32_t)kSDHC_EnableIndexCheckFlag;
	}
}

static uint32_t sdhc_kinetis_transfer_flags(const struct sdhc_kinetis_req *req)
{
	uint32_t flags = sdhc_kinetis_response_flags(req->z_cmd->response_type);

	if (req->z_cmd->opcode == SD_STOP_TRANSMISSION) {
		flags |= (uint32_t)kSDHC_CommandTypeAbortFlag;
	}
	if (req->z_data == NULL) {
		return flags;
	}

	flags |= (uint32_t)kSDHC_DataPresentFlag;
	if (req->use_dma) {
		flags |= (uint32_t)kSDHC_EnableDmaFlag;
	}
	if (!req->is_write) {
		flags |= (uint32_t)kSDHC_DataReadFlag;
	}
	if (req->z_data->blocks > 1U) {
		flags |= (uint32_t)kSDHC_MultipleBlockFlag |
			 (uint32_t)kSDHC_EnableBlockCountFlag;
		if (req->use_auto_cmd12) {
			flags |= (uint32_t)kSDHC_EnableAutoCommand12Flag;
		}
	}

	return flags;
}

static int sdhc_kinetis_start_request(const struct device *dev, struct sdhc_kinetis_req *req)
{
	const struct sdhc_kinetis_config *cfg = dev->config;
	struct sdhc_kinetis_data *data = dev->data;
	sdhc_dma_mode_t dma_mode = req->use_dma ? kSDHC_DmaModeAdma2 : kSDHC_DmaModeNo;
	uint32_t prsstat = SDHC_GetPresentStatusFlags(cfg->base);
	unsigned int key;
	uint32_t irq_flags = (uint32_t)kSDHC_CommandFlag;
	uint32_t transfer_flags;
#if defined(CONFIG_SDHC_KINETIS_TEST)
	bool mask_terminal;
#endif
	int ret;

	if ((prsstat & (uint32_t)kSDHC_CommandInhibitFlag) != 0U) {
		sdhc_kinetis_take_snapshot(cfg->base, &req->snapshot);
		return -EBUSY;
	}

	if (((prsstat & (uint32_t)kSDHC_DataInhibitFlag) != 0U) &&
	    !sdhc_kinetis_cmd_allowed_while_dat_busy(req->z_cmd)) {
		sdhc_kinetis_take_snapshot(cfg->base, &req->snapshot);
		return -EBUSY;
	}

	SDHC_ClearInterruptStatusFlags(cfg->base, kSDHC_AllInterruptFlags);

	ret = sdhc_kinetis_prepare_data_cache(req);
	if (ret != 0) {
		return ret;
	}

	if (req->use_dma) {
		ret = sdhc_kinetis_prepare_adma_table(dev, req);
		if (ret != 0) {
			return ret;
		}
	}

	atomic_set(&req->completion, KINETIS_SDHC_REQ_ARMED);

	k_sem_reset(&data->transfer_sem);
	atomic_ptr_set(&data->active_req, req);

	sdhc_kinetis_set_dma_mode(cfg->base, dma_mode);
	if (req->z_data != NULL) {
		irq_flags |= req->use_dma ? (uint32_t)kSDHC_DataDMAFlag
					  : (uint32_t)kSDHC_DataFlag;
	} else if (sdhc_kinetis_cmd_has_busy_response(req)) {
		irq_flags |= KINETIS_SDHC_BUSY_IRQ_FLAGS;
	}
	SDHC_EnableInterruptSignal(cfg->base, irq_flags);

	cfg->base->BLKATTR = req->z_data == NULL
				 ? 0U
				 : SDHC_BLKATTR_BLKSIZE(req->z_data->block_size) |
					   SDHC_BLKATTR_BLKCNT(req->z_data->blocks);
	cfg->base->CMDARG = req->z_cmd->arg;
	transfer_flags = sdhc_kinetis_transfer_flags(req);

#if defined(CONFIG_SDHC_KINETIS_TEST)
	mask_terminal = (req->z_data != NULL) &&
			atomic_cas(&data->test_mask_terminal, 1, 0);
#endif
	key = irq_lock();
	req->started = true;
	cfg->base->XFERTYP = SDHC_XFERTYP_CMDINX(req->z_cmd->opcode) | transfer_flags;
#if defined(CONFIG_SDHC_KINETIS_TEST)
	if (req->use_dma && atomic_cas(&data->test_force_dma_error, 1, 0)) {
		SDHC_SetForceEvent(cfg->base, (uint32_t)kSDHC_ForceEventDmaError);
	}
	if (mask_terminal) {
		SDHC_DisableInterruptSignal(cfg->base, (uint32_t)kSDHC_DataCompleteFlag);
	}
#endif
	irq_unlock(key);

	return 0;
}

static int sdhc_kinetis_wait_for_request(const struct device *dev, struct sdhc_kinetis_req *req,
					 int64_t deadline)
{
	const struct sdhc_kinetis_config *cfg = dev->config;
	int completion;

	while (!sdhc_kinetis_req_terminal(req)) {
		int64_t remaining_ms = deadline - k_uptime_get();
		uint32_t wait_ms;
		unsigned int key;

		if (remaining_ms <= 0) {
			sdhc_kinetis_req_record_snapshot(req, cfg->base);
			return -ETIMEDOUT;
		}

		wait_ms = (uint32_t)MIN(remaining_ms, (int64_t)KINETIS_SDHC_COMPLETION_POLL_MS);
		(void)k_sem_take(&((struct sdhc_kinetis_data *)dev->data)->transfer_sem,
				 K_MSEC(wait_ms));

		key = irq_lock();
		sdhc_kinetis_service_irqs(dev);
		irq_unlock(key);
	}

	completion = (int)atomic_get(&req->completion);

	return completion > 0 ? 0 : completion;
}

static int sdhc_kinetis_execute_request_until(const struct device *dev,
					      struct sdhc_kinetis_req *req, int64_t deadline)
{
	const struct sdhc_kinetis_config *cfg = dev->config;
	struct sdhc_kinetis_data *data = dev->data;
	int ret;

	if (k_uptime_get() >= deadline) {
		return -ETIMEDOUT;
	}

	ret = sdhc_kinetis_start_request(dev, req);
	if (ret != 0) {
		(void)sdhc_kinetis_complete_data_cache(req, false);
		return ret;
	}

	ret = sdhc_kinetis_wait_for_request(dev, req, deadline);
	atomic_ptr_cas(&data->active_req, req, NULL);
	sdhc_kinetis_disable_transfer_irqs(cfg->base);
	if (ret != 0) {
		(void)sdhc_kinetis_complete_data_cache(req, false);
		return ret;
	}

	ret = sdhc_kinetis_complete_data_cache(req, true);
	if (ret != 0) {
		return ret;
	}

	if (req->z_data != NULL) {
		req->z_data->bytes_xfered = req->z_data->blocks * req->z_data->block_size;
	}

	return 0;
}

static int sdhc_kinetis_execute_request(const struct device *dev, struct sdhc_kinetis_req *req)
{
	uint32_t timeout_ms = sdhc_kinetis_finite_request_timeout_ms(req->z_cmd, req->z_data);

	return sdhc_kinetis_execute_request_until(dev, req,
						  sdhc_kinetis_deadline_after(timeout_ms));
}

static int sdhc_kinetis_issue_stop(const struct device *dev, uint32_t timeout_ms, int64_t deadline)
{
	struct sdhc_kinetis_data *data = dev->data;
	struct sdhc_command z_cmd = {
		.opcode = SD_STOP_TRANSMISSION,
		.arg = 0U,
		.response_type = SD_RSP_TYPE_R1b,
		.timeout_ms = timeout_ms,
	};
	int ret;

	ret = sdhc_kinetis_prepare_request(&data->request, &z_cmd, NULL, false);
	if (ret != 0) {
		return ret;
	}

	ret = sdhc_kinetis_execute_request_until(dev, &data->request, deadline);
	data->request.z_cmd = NULL;

	return ret;
}

static int sdhc_kinetis_issue_go_idle_cmd(const struct device *dev)
{
	struct sdhc_kinetis_data *data = dev->data;
	struct sdhc_command z_cmd = {
		.opcode = SD_GO_IDLE_STATE,
		.arg = 0U,
		.response_type = SD_RSP_TYPE_NONE,
		.timeout_ms = KINETIS_SDHC_DEFAULT_TIMEOUT_MS,
	};
	int ret;

	ret = sdhc_kinetis_prepare_request(&data->request, &z_cmd, NULL, false);
	if (ret != 0) {
		return ret;
	}

	ret = sdhc_kinetis_execute_request(dev, &data->request);
	data->request.z_cmd = NULL;

	return ret;
}

static int sdhc_kinetis_wait_card_ready(const struct device *dev, uint32_t timeout_ms)
{
	return sdhc_kinetis_wait_present_state(dev, (uint32_t)kSDHC_Data0LineLevelFlag, true,
					       timeout_ms);
}

static int sdhc_kinetis_reset_lines(const struct device *dev, bool reset_cmd, bool reset_data)
{
	const struct sdhc_kinetis_config *cfg = dev->config;

	if (reset_cmd &&
	    !SDHC_Reset(cfg->base, (uint32_t)kSDHC_ResetCommand, KINETIS_SDHC_RESET_TIMEOUT)) {
		return sdhc_kinetis_restore_host_state(dev);
	}

	if (reset_data &&
	    !SDHC_Reset(cfg->base, (uint32_t)kSDHC_ResetData, KINETIS_SDHC_RESET_TIMEOUT)) {
		return sdhc_kinetis_restore_host_state(dev);
	}

	return 0;
}

static int sdhc_kinetis_drain_single_block(const struct device *dev, size_t block_size,
					    int64_t deadline)
{
	struct sdhc_kinetis_data *data = dev->data;
	uint32_t clock_hz = data->host_io.clock;
	uint64_t wire_bits = ((uint64_t)block_size * CHAR_BIT) + 128U;
	uint32_t drain_ms;

	if ((block_size == 0U) || (clock_hz == 0U)) {
		return 0;
	}

	/*
	 * A data-line reset makes the host idle immediately, but it cannot cancel
	 * bits the card is already sending.  DAT0 is not a reliable read-transfer
	 * completion indication because it naturally toggles within the block.  Use
	 * one-bit bus timing as a conservative bound and keep one scheduler tick of
	 * margin before accepting another data command.
	 */
	drain_ms = MAX(1U, (uint32_t)DIV_ROUND_UP(wire_bits * MSEC_PER_SEC, clock_hz) + 1U);
	if (sdhc_kinetis_timeout_before_deadline(drain_ms, deadline) < drain_ms) {
		return -ETIMEDOUT;
	}

	k_msleep(drain_ms);
	return 0;
}

static int sdhc_kinetis_recover_request(const struct device *dev,
					const struct sdhc_kinetis_req *req, int64_t deadline)
{
	const struct sdhc_kinetis_config *cfg = dev->config;
	struct sdhc_kinetis_data *data = dev->data;
	const struct sdhc_kinetis_hw_snapshot *snapshot = sdhc_kinetis_primary_snapshot(req);
	uint32_t snapshot_irqstat = snapshot->irqstat;
	uint32_t snapshot_prsstat = snapshot->prsstat;
	uint32_t live_prsstat;
	uint32_t live_irqstat;
	bool cmd_allowed_while_dat_busy = sdhc_kinetis_cmd_allowed_while_dat_busy(req->z_cmd);
	bool request_had_data = req->z_data != NULL;
	bool request_was_multiblock = req->is_multiblock;
	bool card_removed = atomic_get(&req->completion) == -ENODEV;
	size_t request_block_size = request_had_data ? req->z_data->block_size : 0U;
	bool need_cmd_reset;
	bool need_data_reset;
	bool need_stop;
	bool data_path_needs_recovery;
	bool dma_error = (snapshot_irqstat & (uint32_t)kSDHC_DmaErrorFlag) != 0U;
	uint32_t timeout_ms = sdhc_kinetis_finite_request_timeout_ms(req->z_cmd, req->z_data);
	int ret;

	sdhc_kinetis_disable_transfer_irqs(cfg->base);
	atomic_ptr_clear(&data->active_req);
	SDHC_ClearInterruptStatusFlags(cfg->base, KINETIS_SDHC_TRANSFER_IRQ_FLAGS);
	if (card_removed) {
		return sdhc_kinetis_reset_lines(dev, true, true);
	}

	live_prsstat = SDHC_GetPresentStatusFlags(cfg->base);

	/*
	 * The reference manual requires CMD12 when aborting a multi-block transfer.
	 * A single-block command terminates at the block boundary by itself; injecting
	 * CMD12 into that recovery path can instead leave some cards out of phase with
	 * the host.  The data-line reset below is still mandatory for every DMA error.
	 */
	data_path_needs_recovery =
		((snapshot_irqstat &
		  ((uint32_t)kSDHC_DataErrorFlag | (uint32_t)kSDHC_DmaErrorFlag |
		   (uint32_t)kSDHC_AutoCommand12ErrorFlag)) != 0U) ||
		((snapshot_prsstat &
		  ((uint32_t)kSDHC_DataInhibitFlag | (uint32_t)kSDHC_DataLineActiveFlag)) != 0U) ||
		((live_prsstat &
		  ((uint32_t)kSDHC_DataInhibitFlag | (uint32_t)kSDHC_DataLineActiveFlag)) != 0U);
	need_stop = request_was_multiblock && data_path_needs_recovery &&
		    (request_had_data || !cmd_allowed_while_dat_busy);

	if (need_stop) {
		uint32_t stop_timeout_ms =
			sdhc_kinetis_timeout_before_deadline(timeout_ms, deadline);

		ret = stop_timeout_ms == 0U
			      ? -ETIMEDOUT
			      : sdhc_kinetis_issue_stop(dev, stop_timeout_ms, deadline);
		if ((ret != 0) && (ret != -ETIMEDOUT) && (ret != -EIO) && (ret != -EBUSY)) {
			return sdhc_kinetis_restore_host_state(dev);
		}
	}

	live_prsstat = SDHC_GetPresentStatusFlags(cfg->base);
	live_irqstat = SDHC_GetInterruptStatusFlags(cfg->base);

	need_cmd_reset = ((live_irqstat & (uint32_t)kSDHC_CommandErrorFlag) != 0U) ||
			 ((live_prsstat & (uint32_t)kSDHC_CommandInhibitFlag) != 0U);
	need_data_reset =
		(((live_irqstat &
		   ((uint32_t)kSDHC_DataErrorFlag | (uint32_t)kSDHC_DmaErrorFlag)) != 0U) ||
		 ((snapshot_irqstat &
		   ((uint32_t)kSDHC_DataErrorFlag | (uint32_t)kSDHC_DmaErrorFlag |
		    (uint32_t)kSDHC_AutoCommand12ErrorFlag)) != 0U) ||
		 ((live_prsstat &
		   ((uint32_t)kSDHC_DataInhibitFlag | (uint32_t)kSDHC_DataLineActiveFlag)) != 0U)) &&
		(request_had_data || !cmd_allowed_while_dat_busy);

	/* A full reset is the only reliable way to leave the ADMA engine after DMAE. */
	ret = dma_error ? sdhc_kinetis_restore_host_state(dev)
			: sdhc_kinetis_reset_lines(dev, need_cmd_reset, need_data_reset);
	if (ret != 0) {
		return ret;
	}
	if (need_data_reset && request_had_data && !request_was_multiblock) {
		ret = sdhc_kinetis_drain_single_block(dev, request_block_size, deadline);
		if (ret != 0) {
			return ret;
		}
	}

	if (need_stop || need_data_reset) {
		uint32_t ready_timeout_ms = sdhc_kinetis_timeout_before_deadline(
			KINETIS_SDHC_CARD_READY_TIMEOUT_MS, deadline);

		ret = ready_timeout_ms == 0U ? -ETIMEDOUT
					     : sdhc_kinetis_wait_card_ready(dev, ready_timeout_ms);
		if (ret != 0) {
			return ret;
		}
	}

	SDHC_ClearInterruptStatusFlags(cfg->base, kSDHC_AllInterruptFlags);

	return 0;
}

static bool sdhc_kinetis_should_retry(int ret)
{
	return (ret == -ETIMEDOUT) || (ret == -EIO) || (ret == -EBUSY);
}

struct sdhc_kinetis_bit_name {
	uint32_t mask;
	const char *name;
};

static const struct sdhc_kinetis_bit_name sdhc_kinetis_irq_error_names[] = {
	{(uint32_t)kSDHC_CommandTimeoutFlag, "CMD timeout"},
	{(uint32_t)kSDHC_CommandCrcErrorFlag, "CMD CRC error"},
	{(uint32_t)kSDHC_CommandEndBitErrorFlag, "CMD end-bit error"},
	{(uint32_t)kSDHC_CommandIndexErrorFlag, "CMD index error"},
	{(uint32_t)kSDHC_DataTimeoutFlag, "DATA timeout"},
	{(uint32_t)kSDHC_DataCrcErrorFlag, "DATA CRC error"},
	{(uint32_t)kSDHC_DataEndBitErrorFlag, "DATA end-bit error"},
	{(uint32_t)kSDHC_AutoCommand12ErrorFlag, "Auto CMD12 error"},
	{(uint32_t)kSDHC_DmaErrorFlag, "DMA error"},
};

static const struct sdhc_kinetis_bit_name sdhc_kinetis_ac12_error_names[] = {
	{(uint32_t)kSDHC_AutoCommand12NotExecutedFlag, "not executed"},
	{(uint32_t)kSDHC_AutoCommand12TimeoutFlag, "timeout"},
	{(uint32_t)kSDHC_AutoCommand12EndBitErrorFlag, "end-bit error"},
	{(uint32_t)kSDHC_AutoCommand12CrcErrorFlag, "CRC error"},
	{(uint32_t)kSDHC_AutoCommand12IndexErrorFlag, "index error"},
	{(uint32_t)kSDHC_AutoCommand12NotIssuedFlag, "not issued"},
};

static void sdhc_kinetis_log_bits(const char *kind, uint32_t value,
				   const struct sdhc_kinetis_bit_name *names, size_t count)
{
	for (size_t i = 0U; i < count; i++) {
		if ((value & names[i].mask) != 0U) {
			LOG_DBG("  %s: %s", kind, names[i].name);
		}
	}
}

static void sdhc_kinetis_log_irq_errors(uint32_t irqstat)
{
	sdhc_kinetis_log_bits("IRQ", irqstat, sdhc_kinetis_irq_error_names,
			       ARRAY_SIZE(sdhc_kinetis_irq_error_names));
}

static void sdhc_kinetis_log_ac12_error(uint32_t ac12err)
{
	sdhc_kinetis_log_bits("AC12", ac12err, sdhc_kinetis_ac12_error_names,
			       ARRAY_SIZE(sdhc_kinetis_ac12_error_names));
}

static void sdhc_kinetis_log_adma_error(uint32_t admaes)
{
	uint32_t state;

	if (admaes == 0U) {
		return;
	}

	state = (admaes & SDHC_ADMAES_ADMAES_MASK) >> SDHC_ADMAES_ADMAES_SHIFT;

	static const char *const state_names[] = {
		[kSDHC_AdmaErrorStateStopDma] = "stop DMA",
		[kSDHC_AdmaErrorStateFetchDescriptor] = "fetch descriptor",
		[kSDHC_AdmaErrorStateChangeAddress] = "change address",
		[kSDHC_AdmaErrorStateTransferData] = "transfer data",
	};
	static const struct sdhc_kinetis_bit_name error_names[] = {
		{(uint32_t)kSDHC_AdmaLenghMismatchFlag, "length mismatch"},
		{(uint32_t)kSDHC_AdmaDescriptorErrorFlag, "descriptor error"},
	};

	LOG_DBG("  ADMA state: %s", (state < ARRAY_SIZE(state_names)) &&
					(state_names[state] != NULL)
				    ? state_names[state]
				    : "unknown");
	sdhc_kinetis_log_bits("ADMA", admaes, error_names, ARRAY_SIZE(error_names));
}

static bool sdhc_kinetis_should_fallback_to_pio(const struct sdhc_kinetis_req *req, int ret)
{
	const struct sdhc_kinetis_hw_snapshot *snapshot;

	if (!req->use_dma || (req->z_data == NULL)) {
		return false;
	}

	if (ret == -EINVAL) {
		return true;
	}

	snapshot = sdhc_kinetis_primary_snapshot(req);

	return ((snapshot->irqstat & (uint32_t)kSDHC_DmaErrorFlag) != 0U) ||
	       (snapshot->admaes != 0U);
}

static void sdhc_kinetis_log_failure(const struct sdhc_command *cmd,
				     const struct sdhc_kinetis_req *req, int ret)
{
	const struct sdhc_kinetis_hw_snapshot *snapshot = sdhc_kinetis_primary_snapshot(req);

	LOG_ERR("cmd%u failed ret=%d mode=%s irqstat=0x%08x prsstat=0x%08x ac12err=0x%08x "
		"admaes=0x%08x completion=%d",
		cmd->opcode, ret, req->use_dma ? "dma" : "pio", snapshot->irqstat,
		snapshot->prsstat, snapshot->ac12err, snapshot->admaes,
		(int)atomic_get(&req->completion));
	sdhc_kinetis_log_irq_errors(snapshot->irqstat);
	sdhc_kinetis_log_ac12_error(snapshot->ac12err);
	sdhc_kinetis_log_adma_error(snapshot->admaes);
}

static int sdhc_kinetis_go_idle(const struct device *dev)
{
	const struct sdhc_kinetis_config *cfg = dev->config;
	struct sdhc_kinetis_data *data = dev->data;
	uint32_t prsstat;
	uint32_t src_clk_hz;
	uint32_t init_clock_hz;
	bool restore_clock_off = false;
	bool ok;
	int ret;

	if (data->host_io.power_mode != SDHC_POWER_ON) {
		return -EIO;
	}

	ret = sdhc_kinetis_restore_host_state(dev);
	if (ret != 0) {
		return ret;
	}

	if ((cfg->base->SYSCTL & SDHC_SYSCTL_SDCLKEN_MASK) == 0U) {
		src_clk_hz = sdhc_kinetis_src_clock_hz();
		if (src_clk_hz == 0U) {
			return -ENOTSUP;
		}

		init_clock_hz = data->host_io.clock;
		if (init_clock_hz == 0U) {
			init_clock_hz = cfg->min_bus_freq;
		}

		if ((init_clock_hz < cfg->min_bus_freq) || (init_clock_hz > cfg->max_bus_freq) ||
		    (init_clock_hz > src_clk_hz) ||
		    (sdhc_kinetis_set_clock(cfg->base, src_clk_hz, init_clock_hz,
					     &init_clock_hz) != 0)) {
			return -ENOTSUP;
		}
		restore_clock_off = (data->host_io.clock == 0U);
	}

	ret = sdhc_kinetis_wait_present_state(dev, (uint32_t)kSDHC_CommandInhibitFlag, false,
					      KINETIS_SDHC_DEFAULT_TIMEOUT_MS);
	if (ret != 0) {
		goto out;
	}

	prsstat = SDHC_GetPresentStatusFlags(cfg->base);
	if ((prsstat & ((uint32_t)kSDHC_DataInhibitFlag | (uint32_t)kSDHC_DataLineActiveFlag)) !=
	    0U) {
		ret = sdhc_kinetis_issue_stop(
			dev, KINETIS_SDHC_DEFAULT_TIMEOUT_MS,
			sdhc_kinetis_deadline_after(KINETIS_SDHC_DEFAULT_TIMEOUT_MS));
		if ((ret != 0) && (ret != -ETIMEDOUT) && (ret != -EIO) && (ret != -EBUSY)) {
			ret = sdhc_kinetis_restore_host_state(dev);
			goto out;
		}

		prsstat = SDHC_GetPresentStatusFlags(cfg->base);
		if ((prsstat &
		     ((uint32_t)kSDHC_DataInhibitFlag | (uint32_t)kSDHC_DataLineActiveFlag)) !=
		    0U) {
			ret = sdhc_kinetis_reset_lines(dev, false, true);
			if (ret != 0) {
				goto out;
			}
		}
	}

	ret = sdhc_kinetis_wait_present_state(
		dev, (uint32_t)kSDHC_CommandInhibitFlag | (uint32_t)kSDHC_DataInhibitFlag, false,
		KINETIS_SDHC_DEFAULT_TIMEOUT_MS);
	if (ret != 0) {
		goto out;
	}

	ok = SDHC_SetCardActive(cfg->base, KINETIS_SDHC_DEFAULT_TIMEOUT_MS);
	if (!ok) {
		ret = -EIO;
		goto out;
	}

	ret = sdhc_kinetis_issue_go_idle_cmd(dev);

out:
	if (restore_clock_off) {
		SDHC_EnableSdClock(cfg->base, false);
	}

	return ret;
}

static void sdhc_kinetis_cd_gpio_cb(const struct device *port, struct gpio_callback *cb,
			       gpio_port_pins_t pins)
{
	ARG_UNUSED(port);
	ARG_UNUSED(pins);

	struct sdhc_kinetis_data *data = CONTAINER_OF(cb, struct sdhc_kinetis_data, cd_cb);
	const struct device *dev = data->dev;
	const struct sdhc_kinetis_config *cfg = dev->config;
	int value;
	bool present;
	struct sdhc_kinetis_req *req;

	value = gpio_pin_get_dt(&cfg->cd_gpio);
	if (value < 0) {
		return;
	}

	present = value > 0;
	if (present == (atomic_get(&data->cd_present) != 0)) {
		return;
	}

	atomic_set(&data->cd_present, present);
	if (!present) {
		req = atomic_ptr_get(&data->active_req);
		if (req != NULL) {
			sdhc_kinetis_fail_request(dev, req, -ENODEV);
		}
	}

	if ((data->card_cb != NULL) &&
	    ((data->enabled_sources & (present ? SDHC_INT_INSERTED : SDHC_INT_REMOVED)) != 0)) {
		data->card_cb(dev, present ? SDHC_INT_INSERTED : SDHC_INT_REMOVED,
			      data->card_cb_user_data);
	}

}

static int sdhc_kinetis_reset(const struct device *dev)
{
	struct sdhc_kinetis_data *data = dev->data;
	int ret;

	k_mutex_lock(&data->lock, K_FOREVER);
	ret = sdhc_kinetis_reset_host(dev);
	k_mutex_unlock(&data->lock);

	return ret;
}

static int sdhc_kinetis_get_host_props(const struct device *dev, struct sdhc_host_props *props)
{
	const struct sdhc_kinetis_config *cfg = dev->config;
	sdhc_capability_t cap = {0};
	uint32_t src_clk_hz = sdhc_kinetis_src_clock_hz();

	if (src_clk_hz < cfg->min_bus_freq) {
		return -ENOTSUP;
	}

	memset(props, 0, sizeof(*props));
	props->f_min = cfg->min_bus_freq;
	props->f_max = MIN(cfg->max_bus_freq, src_clk_hz);
	props->power_delay = cfg->power_delay_ms;
	props->bus_4_bit_support = (cfg->bus_width >= 4U);
	props->host_caps.vol_330_support = true;
	props->host_caps.bus_8_bit_support = (cfg->bus_width >= 8U);

	SDHC_GetCapability(cfg->base, &cap);
	props->host_caps.high_spd_support =
		(cap.flags & (uint32_t)kSDHC_SupportHighSpeedFlag) != 0U;
	props->host_caps.suspend_res_support =
		(cap.flags & (uint32_t)kSDHC_SupportSuspendResumeFlag) != 0U;
	props->host_caps.sdma_support = (cap.flags & (uint32_t)kSDHC_SupportDmaFlag) != 0U;
	props->host_caps.adma_2_support = (cap.flags & (uint32_t)kSDHC_SupportAdmaFlag) != 0U;
	props->host_caps.max_blk_len = sdhc_kinetis_max_blk_len(cap.maxBlockLength);
#if defined(FSL_FEATURE_SDHC_HAS_V300_SUPPORT) && FSL_FEATURE_SDHC_HAS_V300_SUPPORT
	props->host_caps.vol_300_support =
		(cap.flags & (uint32_t)kSDHC_SupportV300Flag) != 0U;
#endif
#if defined(FSL_FEATURE_SDHC_HAS_V180_SUPPORT) && FSL_FEATURE_SDHC_HAS_V180_SUPPORT
	props->host_caps.vol_180_support =
		(cap.flags & (uint32_t)kSDHC_SupportV180Flag) != 0U;
#endif

	return 0;
}

static int sdhc_kinetis_set_io(const struct device *dev, struct sdhc_io *ios)
{
	struct sdhc_kinetis_data *data = dev->data;
	int ret;

	k_mutex_lock(&data->lock, K_FOREVER);
	ret = sdhc_kinetis_apply_io(dev, ios);
	k_mutex_unlock(&data->lock);

	return ret;
}

static int sdhc_kinetis_get_card_present(const struct device *dev)
{
	const struct sdhc_kinetis_config *cfg = dev->config;
	struct sdhc_kinetis_data *data = dev->data;

	if (cfg->cd_gpio.port != NULL) {
		int value = gpio_pin_get_dt(&cfg->cd_gpio);

		if (value < 0) {
			return value;
		}

		atomic_set(&data->cd_present, value > 0);
		return atomic_get(&data->cd_present) != 0;
	}

	atomic_set(&data->cd_present,
		   (SDHC_GetPresentStatusFlags(cfg->base) & (uint32_t)kSDHC_CardInsertedFlag) !=
			   0U);
	return atomic_get(&data->cd_present) != 0;
}

static int sdhc_kinetis_card_busy(const struct device *dev)
{
	const struct sdhc_kinetis_config *cfg = dev->config;
	uint32_t status = SDHC_GetPresentStatusFlags(cfg->base);

	return (status & (uint32_t)kSDHC_Data0LineLevelFlag) == 0U;
}

static int sdhc_kinetis_execute_tuning(const struct device *dev)
{
	ARG_UNUSED(dev);
	return -ENOTSUP;
}

static int sdhc_kinetis_request(const struct device *dev, struct sdhc_command *cmd,
				struct sdhc_data *data)
{
	const struct sdhc_kinetis_config *cfg = dev->config;
	struct sdhc_kinetis_data *dev_data = dev->data;
	bool prefer_dma = sdhc_kinetis_request_prefers_dma(cfg, cmd, data);
	bool force_pio = false;
	uint32_t retries_left = cmd->retries;
	uint32_t timeout_ms;
	int64_t operation_deadline;
	int ret;

	if (cmd->retries == UINT32_MAX) {
		return -EINVAL;
	}
#if defined(CONFIG_SDHC_KINETIS_TEST)
	if (prefer_dma && atomic_cas(&dev_data->test_force_pio, 1, 0)) {
		prefer_dma = false;
	}
#endif

	timeout_ms = sdhc_kinetis_finite_request_timeout_ms(cmd, data);
	operation_deadline = sdhc_kinetis_operation_deadline(timeout_ms, cmd->retries);

	k_mutex_lock(&dev_data->lock, K_FOREVER);
	LOG_DBG("cmd%u arg=0x%08x resp=0x%x data=%p", cmd->opcode, cmd->arg, cmd->response_type,
		data);

	if (cmd->opcode == SD_GO_IDLE_STATE) {
		ret = sdhc_kinetis_go_idle(dev);
		k_mutex_unlock(&dev_data->lock);
		return ret;
	}

	if (data != NULL) {
		data->bytes_xfered = 0U;
	}

	for (;;) {
		struct sdhc_kinetis_req *req = &dev_data->request;
		int64_t attempt_deadline;
		bool fallback_to_pio;
		bool prepared = false;

		if (k_uptime_get() >= operation_deadline) {
			ret = -ETIMEDOUT;
			break;
		}
		attempt_deadline = MIN(operation_deadline, sdhc_kinetis_deadline_after(timeout_ms));

		ret = sdhc_kinetis_prepare_request(req, cmd, data, prefer_dma && !force_pio);
		if (ret == 0) {
			prepared = true;
			ret = sdhc_kinetis_execute_request_until(dev, req, attempt_deadline);
		}

		if (ret == 0) {
			break;
		}
		if (prepared && !req->started && (ret == -EBUSY)) {
			uint32_t inhibit = (uint32_t)kSDHC_CommandInhibitFlag;
			uint32_t remaining_ms = sdhc_kinetis_timeout_before_deadline(
				UINT32_MAX, operation_deadline);

			if (!sdhc_kinetis_cmd_allowed_while_dat_busy(cmd)) {
				inhibit |= (uint32_t)kSDHC_DataInhibitFlag;
			}
			ret = remaining_ms == 0U
				      ? -ETIMEDOUT
				      : sdhc_kinetis_wait_present_state(dev, inhibit, false,
									remaining_ms);
			if (ret == 0) {
				continue;
			}
		}

		sdhc_kinetis_log_failure(cmd, req, ret);
		fallback_to_pio = prepared && sdhc_kinetis_should_fallback_to_pio(req, ret);

		if (prepared && req->started) {
			int recover_ret =
				sdhc_kinetis_recover_request(dev, req, operation_deadline);

			if (recover_ret != 0) {
				ret = recover_ret;
				fallback_to_pio = false;
			}
		}

		if (fallback_to_pio && !force_pio &&
		    (retries_left != 0U)) {
			retries_left--;
			force_pio = true;
#if defined(CONFIG_SDHC_KINETIS_TEST)
			atomic_inc(&dev_data->test_pio_fallbacks);
#endif
			LOG_DBG("retrying cmd%u in pio mode retries_left=%u", cmd->opcode,
				retries_left);
			continue;
		}

		if (!sdhc_kinetis_should_retry(ret) || (retries_left == 0U)) {
			break;
		}

		retries_left--;
		LOG_DBG("retrying cmd%u ret=%d retries_left=%u mode=%s", cmd->opcode, ret,
			retries_left, force_pio ? "pio" : "dma");
	}

	k_mutex_unlock(&dev_data->lock);
	return ret;
}

static int sdhc_kinetis_enable_interrupt(const struct device *dev, sdhc_interrupt_cb_t callback,
					 int sources, void *user_data)
{
	struct sdhc_kinetis_data *data = dev->data;
	const struct sdhc_kinetis_config *cfg = dev->config;
	int ret;

	if ((sources & ~(SDHC_INT_SDIO | SDHC_INT_INSERTED | SDHC_INT_REMOVED)) != 0) {
		return -ENOTSUP;
	}

	if (callback == NULL) {
		return -EINVAL;
	}

	k_mutex_lock(&data->lock, K_FOREVER);
	data->card_cb = callback;
	data->card_cb_user_data = user_data;
	data->enabled_sources |= sources;

	if (cfg->cd_gpio.port != NULL) {
		int value = gpio_pin_get_dt(&cfg->cd_gpio);

		if (value < 0) {
			k_mutex_unlock(&data->lock);
			return value;
		}

		atomic_set(&data->cd_present, value > 0);
	}

	ret = sdhc_kinetis_sync_card_interrupts(dev);
	k_mutex_unlock(&data->lock);

	return ret;
}

static int sdhc_kinetis_disable_interrupt(const struct device *dev, int sources)
{
	struct sdhc_kinetis_data *data = dev->data;
	int ret;

	k_mutex_lock(&data->lock, K_FOREVER);
	data->enabled_sources &= ~sources;
	if (data->enabled_sources == 0) {
		data->card_cb = NULL;
		data->card_cb_user_data = NULL;
	}

	ret = sdhc_kinetis_sync_card_interrupts(dev);
	k_mutex_unlock(&data->lock);

	return ret;
}

static void sdhc_kinetis_handle_sdio_interrupt(const struct device *dev)
{
	const struct sdhc_kinetis_config *cfg = dev->config;
	struct sdhc_kinetis_data *data = dev->data;

	if (data->card_cb != NULL) {
		data->card_cb(dev, SDHC_INT_SDIO, data->card_cb_user_data);
	}

	SDHC_DisableInterruptStatus(cfg->base, (uint32_t)kSDHC_CardInterruptFlag);
	SDHC_DisableInterruptSignal(cfg->base, (uint32_t)kSDHC_CardInterruptFlag);
}

static void sdhc_kinetis_card_inserted(const struct device *dev)
{
	struct sdhc_kinetis_data *data = dev->data;

	atomic_set(&data->cd_present, true);
	sdhc_kinetis_arm_host_cd_interrupts(dev, true);

	if (((data->enabled_sources & SDHC_INT_INSERTED) != 0) && (data->card_cb != NULL)) {
		data->card_cb(dev, SDHC_INT_INSERTED, data->card_cb_user_data);
	}
}

static void sdhc_kinetis_card_removed(const struct device *dev)
{
	struct sdhc_kinetis_data *data = dev->data;
	struct sdhc_kinetis_req *req;

	atomic_set(&data->cd_present, false);
	req = atomic_ptr_get(&data->active_req);
	if (req != NULL) {
		sdhc_kinetis_fail_request(dev, req, -ENODEV);
	}
	sdhc_kinetis_arm_host_cd_interrupts(dev, false);

	if (((data->enabled_sources & SDHC_INT_REMOVED) != 0) && (data->card_cb != NULL)) {
		data->card_cb(dev, SDHC_INT_REMOVED, data->card_cb_user_data);
	}
}

static void sdhc_kinetis_isr(const struct device *dev)
{
	const struct sdhc_kinetis_config *cfg = dev->config;
	uint32_t irqstat = SDHC_GetEnabledInterruptStatusFlags(cfg->base);
	uint32_t clear_flags = 0U;

	if (irqstat == 0U) {
		return;
	}
	if ((irqstat & (uint32_t)kSDHC_CardInsertionFlag) != 0U) {
		sdhc_kinetis_card_inserted(dev);
		clear_flags |= (uint32_t)kSDHC_CardInsertionFlag;
	}
	if ((irqstat & (uint32_t)kSDHC_CardRemovalFlag) != 0U) {
		sdhc_kinetis_card_removed(dev);
		clear_flags |= (uint32_t)kSDHC_CardRemovalFlag;
	}
	if ((irqstat & (uint32_t)kSDHC_CardInterruptFlag) != 0U) {
		sdhc_kinetis_handle_sdio_interrupt(dev);
		clear_flags |= (uint32_t)kSDHC_CardInterruptFlag;
	}
	if (clear_flags != 0U) {
		SDHC_ClearInterruptStatusFlags(cfg->base, clear_flags);
	}

	sdhc_kinetis_service_irqs(dev);
}

static int sdhc_kinetis_init(const struct device *dev)
{
	const struct sdhc_kinetis_config *cfg = dev->config;
	struct sdhc_kinetis_data *data = dev->data;
	struct sdhc_host_props props;
	int ret;

	if (!device_is_ready(cfg->clock_dev)) {
		return -ENODEV;
	}

	ret = pinctrl_apply_state(cfg->pincfg, PINCTRL_STATE_DEFAULT);
	if (ret != 0) {
		return ret;
	}

	ret = clock_control_on(cfg->clock_dev, cfg->clock_subsys);
	if (ret != 0) {
		return ret;
	}

	k_mutex_init(&data->lock);
	k_sem_init(&data->transfer_sem, 0, 1);
	data->dev = dev;
	data->host_io.power_mode = SDHC_POWER_OFF;
	data->host_io.bus_width = SDHC_BUS_WIDTH1BIT;
	atomic_ptr_clear(&data->active_req);

	ret = sdhc_kinetis_init_controller(dev);
	if (ret != 0) {
		return ret;
	}

	ret = sdhc_kinetis_get_host_props(dev, &props);
	if (ret != 0) {
		LOG_ERR("SDHC source clock is below the minimum bus frequency");
		return ret;
	}

	if (cfg->cd_gpio.port != NULL) {
		if (!gpio_is_ready_dt(&cfg->cd_gpio)) {
			LOG_ERR("card-detect gpio device not ready");
			return -ENODEV;
		}

		ret = gpio_pin_configure_dt(&cfg->cd_gpio, GPIO_INPUT);
		if (ret != 0) {
			LOG_ERR("CD gpio_pin_configure_dt failed (%d)", ret);
			return ret;
		}

		gpio_init_callback(&data->cd_cb, sdhc_kinetis_cd_gpio_cb, BIT(cfg->cd_gpio.pin));
		ret = gpio_add_callback(cfg->cd_gpio.port, &data->cd_cb);
		if (ret != 0) {
			LOG_ERR("CD gpio_add_callback failed (%d)", ret);
			return ret;
		}

		ret = gpio_pin_get_dt(&cfg->cd_gpio);
		if (ret < 0) {
			LOG_ERR("CD gpio_pin_get_dt failed (%d)", ret);
			return ret;
		}

		atomic_set(&data->cd_present, ret > 0);
		ret = gpio_pin_interrupt_configure_dt(&cfg->cd_gpio, GPIO_INT_EDGE_BOTH);
		if (ret != 0) {
			LOG_ERR("CD gpio interrupt configure failed (%d)", ret);
			return ret;
		}
	}

	if (cfg->pwr_gpio.port != NULL) {
		if (!gpio_is_ready_dt(&cfg->pwr_gpio)) {
			LOG_ERR("power gpio device not ready");
			return -ENODEV;
		}

		ret = gpio_pin_configure_dt(&cfg->pwr_gpio, GPIO_OUTPUT_INACTIVE);
		if (ret != 0) {
			LOG_ERR("power gpio_pin_configure_dt failed (%d)", ret);
			return ret;
		}
	}

	cfg->irq_config_func(dev);

	return sdhc_kinetis_sync_card_interrupts(dev);
}

static DEVICE_API(sdhc, sdhc_kinetis_api) = {
	.reset = sdhc_kinetis_reset,
	.request = sdhc_kinetis_request,
	.set_io = sdhc_kinetis_set_io,
	.get_card_present = sdhc_kinetis_get_card_present,
	.execute_tuning = sdhc_kinetis_execute_tuning,
	.card_busy = sdhc_kinetis_card_busy,
	.get_host_props = sdhc_kinetis_get_host_props,
	.enable_interrupt = sdhc_kinetis_enable_interrupt,
	.disable_interrupt = sdhc_kinetis_disable_interrupt,
};

#define KINETIS_SDHC_IRQ_CONFIG(inst)                                                              \
	static void sdhc_kinetis_irq_config_##inst(const struct device *dev)                       \
	{                                                                                          \
		ARG_UNUSED(dev);                                                                   \
		NVIC_ClearPendingIRQ( \
			DT_INST_IRQN(inst)); \
		IRQ_CONNECT(DT_INST_IRQN(inst), DT_INST_IRQ(inst, priority), sdhc_kinetis_isr,     \
			    DEVICE_DT_INST_GET(inst), 0);                                          \
		irq_enable(DT_INST_IRQN(inst));                                                    \
	}

#define KINETIS_SDHC_INIT(inst)                                                                    \
	PINCTRL_DT_INST_DEFINE(inst);                                                              \
	KINETIS_SDHC_IRQ_CONFIG(inst);                                                             \
	static const struct sdhc_kinetis_config sdhc_kinetis_cfg_##inst = {                        \
		.base = (SDHC_Type *)DT_INST_REG_ADDR(inst),                                       \
		.pincfg = PINCTRL_DT_INST_DEV_CONFIG_GET(inst),                                    \
		.cd_gpio = GPIO_DT_SPEC_INST_GET_OR(inst, cd_gpios, {0}),                          \
		.pwr_gpio = GPIO_DT_SPEC_INST_GET_OR(inst, pwr_gpios, {0}),                        \
		.clock_dev = DEVICE_DT_GET(DT_INST_CLOCKS_CTLR(inst)),                             \
		.clock_subsys = (clock_control_subsys_t)(uintptr_t)KINETIS_SIM_CLOCK_ID(           \
			DT_INST_CLOCKS_CELL(inst, name), DT_INST_CLOCKS_CELL(inst, offset),        \
			DT_INST_CLOCKS_CELL(inst, bits)),                                          \
		.bus_width = DT_INST_PROP(inst, bus_width),                                        \
		.power_delay_ms = DT_INST_PROP(inst, power_delay_ms),                              \
		.min_bus_freq = DT_INST_PROP(inst, min_bus_freq),                                  \
		.max_bus_freq = DT_INST_PROP(inst, max_bus_freq),                                  \
		.irq_config_func = sdhc_kinetis_irq_config_##inst,                                 \
	};                                                                                         \
	static struct sdhc_kinetis_data sdhc_kinetis_data_##inst;                                  \
	DEVICE_DT_INST_DEFINE(inst, sdhc_kinetis_init, NULL, &sdhc_kinetis_data_##inst,            \
			      &sdhc_kinetis_cfg_##inst, POST_KERNEL, CONFIG_SDHC_INIT_PRIORITY,    \
			      &sdhc_kinetis_api);

DT_INST_FOREACH_STATUS_OKAY(KINETIS_SDHC_INIT)
