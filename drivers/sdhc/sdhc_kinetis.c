/*
 * Copyright (c) 2026 The Zephyr Project Contributors
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT nxp_kinetis_sdhc

#include <errno.h>
#include <stdint.h>
#include <string.h>

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
#include <zephyr/sys/util.h>

#include <soc.h>
#include <fsl_clock.h>
#include <fsl_sdhc.h>

LOG_MODULE_REGISTER(sdhc_kinetis, CONFIG_SDHC_LOG_LEVEL);

BUILD_ASSERT((CONFIG_SDHC_KINETIS_ADMA_TABLE_SIZE % sizeof(uint32_t)) == 0U);

enum kinetis_sdhc_transfer_status {
	TRANSFER_CMD_COMPLETE = BIT(0),
	TRANSFER_CMD_FAILED = BIT(1),
	TRANSFER_DATA_COMPLETE = BIT(2),
	TRANSFER_DATA_FAILED = BIT(3),
};

#define TRANSFER_CMD_FLAGS  (TRANSFER_CMD_COMPLETE | TRANSFER_CMD_FAILED)
#define TRANSFER_DATA_FLAGS (TRANSFER_DATA_COMPLETE | TRANSFER_DATA_FAILED)

#define KINETIS_SDHC_RESET_TIMEOUT       1000000U
#define KINETIS_SDHC_DEFAULT_TIMEOUT_MS  5000U
#define KINETIS_SDHC_WATERMARK_LEVEL     128U
#define KINETIS_SDHC_TRANSFER_IRQ_FLAGS  ((uint32_t)kSDHC_CommandFlag | \
					  (uint32_t)kSDHC_DataFlag | \
					  (uint32_t)kSDHC_DataDMAFlag)
#define KINETIS_SDHC_CARD_IRQ_FLAGS      ((uint32_t)kSDHC_CardDetectFlag | \
					  (uint32_t)kSDHC_CardInterruptFlag)

struct kinetis_sdhc_config {
	SDHC_Type *base;
	const struct pinctrl_dev_config *pincfg;
	struct gpio_dt_spec cd_gpio;
	const struct device *clock_dev;
	clock_control_subsys_t clock_subsys;
	uint8_t bus_width;
	uint32_t power_delay_ms;
	uint32_t min_bus_freq;
	uint32_t max_bus_freq;
	void (*irq_config_func)(const struct device *dev);
};

struct kinetis_sdhc_data {
	struct sdhc_host_props props;
	struct sdhc_io host_io;
	struct k_mutex lock;
	struct k_sem transfer_sem;
	volatile uint32_t transfer_status;
	sdhc_handle_t transfer_handle;
	sdhc_interrupt_cb_t card_cb;
	void *card_cb_user_data;
	int enabled_sources;
	struct gpio_callback cd_cb;
	const struct device *dev;
	bool cd_present;
	uint32_t adma_table[CONFIG_SDHC_KINETIS_ADMA_TABLE_SIZE / sizeof(uint32_t)]
		__aligned(sizeof(uint32_t));
};

static void kinetis_sdhc_init_host_cfg(sdhc_config_t *host_cfg)
{
	memset(host_cfg, 0, sizeof(*host_cfg));
	host_cfg->cardDetectDat3 = false;
	host_cfg->endianMode = kSDHC_EndianModeLittle;
	host_cfg->dmaMode = kSDHC_DmaModeAdma2;
	host_cfg->readWatermarkLevel = KINETIS_SDHC_WATERMARK_LEVEL;
	host_cfg->writeWatermarkLevel = KINETIS_SDHC_WATERMARK_LEVEL;
}

static uint32_t kinetis_sdhc_src_clock_hz(void)
{
	/*
	 * The SIM clock-control driver handles the gate, but the SDHC source
	 * clock mux still has to be read directly from SIM_SOPT2[SDHCSRC].
	 */
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

static uint32_t kinetis_sdhc_max_blk_len(size_t max_block_len)
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

static k_timeout_t kinetis_sdhc_timeout(int timeout_ms)
{
	return (timeout_ms == SDHC_TIMEOUT_FOREVER) ? K_FOREVER : K_MSEC(MAX(timeout_ms, 0));
}

static bool kinetis_sdhc_cmd_is_write(const struct sdhc_command *cmd)
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

static bool kinetis_sdhc_cmd_is_read(const struct sdhc_command *cmd)
{
	switch (cmd->opcode) {
	case SD_READ_SINGLE_BLOCK:
	case SD_READ_MULTIPLE_BLOCK:
	case SD_SEND_TUNING_BLOCK:
	case MMC_SEND_TUNING_BLOCK:
	case MMC_SEND_EXT_CSD:
	case SD_APP_SEND_SCR:
		return true;
	case SDIO_RW_EXTENDED:
		return (cmd->arg & BIT(31)) == 0U;
	default:
		return false;
	}
}

static void kinetis_sdhc_clear_transfer_state(struct kinetis_sdhc_data *data)
{
	data->transfer_status = 0U;
	k_sem_reset(&data->transfer_sem);
	data->transfer_handle.command = NULL;
	data->transfer_handle.data = NULL;
	data->transfer_handle.transferredWords = 0U;
}

static int kinetis_sdhc_set_io_locked(const struct device *dev, const struct sdhc_io *ios)
{
	const struct kinetis_sdhc_config *cfg = dev->config;
	struct kinetis_sdhc_data *data = dev->data;
	uint32_t src_clk_hz;
	uint8_t bus_width;

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

	src_clk_hz = kinetis_sdhc_src_clock_hz();
	if (src_clk_hz == 0U) {
		return -ENOTSUP;
	}

	if (ios->clock == 0U) {
		SDHC_EnableSdClock(cfg->base, false);
	} else if ((ios->clock >= cfg->min_bus_freq) && (ios->clock <= cfg->max_bus_freq)) {
		if (SDHC_SetSdClock(cfg->base, src_clk_hz, ios->clock) == 0U) {
			return -ENOTSUP;
		}
		SDHC_EnableSdClock(cfg->base, true);
	} else {
		return -ENOTSUP;
	}

	bus_width = (ios->bus_width == 0U) ? SDHC_BUS_WIDTH1BIT : ios->bus_width;

	switch (bus_width) {
	case SDHC_BUS_WIDTH1BIT:
		SDHC_SetDataBusWidth(cfg->base, kSDHC_DataBusWidth1Bit);
		break;
	case SDHC_BUS_WIDTH4BIT:
		if (cfg->bus_width < 4U) {
			return -ENOTSUP;
		}
		SDHC_SetDataBusWidth(cfg->base, kSDHC_DataBusWidth4Bit);
		break;
	case SDHC_BUS_WIDTH8BIT:
		if (cfg->bus_width < 8U) {
			return -ENOTSUP;
		}
		SDHC_SetDataBusWidth(cfg->base, kSDHC_DataBusWidth8Bit);
		break;
	default:
		return -ENOTSUP;
	}

	if (ios->power_mode == SDHC_POWER_OFF) {
		SDHC_EnableSdClock(cfg->base, false);
	} else if ((ios->power_mode != 0) && (ios->power_mode != SDHC_POWER_ON)) {
		return -ENOTSUP;
	}

	data->host_io = *ios;
	return 0;
}

static int kinetis_sdhc_sync_card_interrupts(const struct device *dev)
{
	const struct kinetis_sdhc_config *cfg = dev->config;
	struct kinetis_sdhc_data *data = dev->data;

	SDHC_DisableInterruptSignal(cfg->base, KINETIS_SDHC_CARD_IRQ_FLAGS);
	SDHC_DisableInterruptStatus(cfg->base, KINETIS_SDHC_CARD_IRQ_FLAGS);
	SDHC_ClearInterruptStatusFlags(cfg->base, KINETIS_SDHC_CARD_IRQ_FLAGS);

	if ((data->enabled_sources & SDHC_INT_SDIO) != 0) {
		SDHC_EnableInterruptStatus(cfg->base, (uint32_t)kSDHC_CardInterruptFlag);
		SDHC_EnableInterruptSignal(cfg->base, (uint32_t)kSDHC_CardInterruptFlag);
	}

	if (cfg->cd_gpio.port != NULL) {
		if ((data->enabled_sources & (SDHC_INT_INSERTED | SDHC_INT_REMOVED)) == 0 ||
		    data->card_cb == NULL) {
			return gpio_pin_interrupt_configure_dt(&cfg->cd_gpio, GPIO_INT_DISABLE);
		}

		return gpio_pin_interrupt_configure_dt(
			&cfg->cd_gpio,
			data->cd_present ? GPIO_INT_EDGE_TO_INACTIVE : GPIO_INT_EDGE_TO_ACTIVE);
	}

	if ((data->enabled_sources & (SDHC_INT_INSERTED | SDHC_INT_REMOVED)) == 0 ||
	    data->card_cb == NULL) {
		return 0;
	}

	data->cd_present =
		(SDHC_GetPresentStatusFlags(cfg->base) & (uint32_t)kSDHC_CardInsertedFlag) != 0U;

	if (!data->cd_present && ((data->enabled_sources & SDHC_INT_INSERTED) != 0)) {
		SDHC_EnableInterruptStatus(cfg->base, (uint32_t)kSDHC_CardInsertionFlag);
		SDHC_EnableInterruptSignal(cfg->base, (uint32_t)kSDHC_CardInsertionFlag);
	}

	if (data->cd_present && ((data->enabled_sources & SDHC_INT_REMOVED) != 0)) {
		SDHC_EnableInterruptStatus(cfg->base, (uint32_t)kSDHC_CardRemovalFlag);
		SDHC_EnableInterruptSignal(cfg->base, (uint32_t)kSDHC_CardRemovalFlag);
	}

	return 0;
}

static int kinetis_sdhc_reset_locked(const struct device *dev)
{
	const struct kinetis_sdhc_config *cfg = dev->config;
	sdhc_config_t host_cfg;
	int ret;

	kinetis_sdhc_init_host_cfg(&host_cfg);
	SDHC_Init(cfg->base, &host_cfg);

	ret = kinetis_sdhc_set_io_locked(dev, &((struct kinetis_sdhc_data *)dev->data)->host_io);
	if (ret != 0) {
		return ret;
	}

	kinetis_sdhc_clear_transfer_state(dev->data);

	return kinetis_sdhc_sync_card_interrupts(dev);
}

static int kinetis_sdhc_recover_locked(const struct device *dev)
{
	const struct kinetis_sdhc_config *cfg = dev->config;
	uint32_t irqstat = SDHC_GetInterruptStatusFlags(cfg->base);
	uint32_t prsstat = SDHC_GetPresentStatusFlags(cfg->base);
	bool need_data_reset = false;
	bool need_cmd_reset = false;

	SDHC_DisableInterruptSignal(cfg->base, KINETIS_SDHC_TRANSFER_IRQ_FLAGS);
	SDHC_ClearInterruptStatusFlags(cfg->base, KINETIS_SDHC_TRANSFER_IRQ_FLAGS);

	if ((irqstat & (uint32_t)kSDHC_CommandErrorFlag) != 0U ||
	    (prsstat & (uint32_t)kSDHC_CommandInhibitFlag) != 0U) {
		need_cmd_reset = true;
	}

	if ((irqstat & ((uint32_t)kSDHC_DataErrorFlag | (uint32_t)kSDHC_DmaErrorFlag)) != 0U ||
	    (prsstat & ((uint32_t)kSDHC_DataInhibitFlag | (uint32_t)kSDHC_DataLineActiveFlag)) != 0U) {
		need_data_reset = true;
	}

	if (need_cmd_reset &&
	    !SDHC_Reset(cfg->base, (uint32_t)kSDHC_ResetCommand, KINETIS_SDHC_RESET_TIMEOUT)) {
		return kinetis_sdhc_reset_locked(dev);
	}

	if (need_data_reset &&
	    !SDHC_Reset(cfg->base, (uint32_t)kSDHC_ResetData, KINETIS_SDHC_RESET_TIMEOUT)) {
		return kinetis_sdhc_reset_locked(dev);
	}

	kinetis_sdhc_clear_transfer_state(dev->data);
	return 0;
}

static int kinetis_sdhc_xfer_error(const struct device *dev)
{
	const struct kinetis_sdhc_config *cfg = dev->config;
	uint32_t irqstat = SDHC_GetInterruptStatusFlags(cfg->base);

	if ((irqstat & ((uint32_t)kSDHC_CommandTimeoutFlag | (uint32_t)kSDHC_DataTimeoutFlag)) != 0U) {
		return -ETIMEDOUT;
	}

	return -EIO;
}

static int kinetis_sdhc_wait_status(struct kinetis_sdhc_data *data, uint32_t mask, int timeout_ms)
{
	while ((data->transfer_status & mask) == 0U) {
		if (k_sem_take(&data->transfer_sem, kinetis_sdhc_timeout(timeout_ms)) != 0) {
			return -ETIMEDOUT;
		}
	}

	return 0;
}

static int kinetis_sdhc_go_idle_locked(const struct device *dev)
{
	const struct kinetis_sdhc_config *cfg = dev->config;
	struct kinetis_sdhc_data *data = dev->data;
	uint32_t src_clk_hz;
	uint32_t init_clock_hz;
	bool restore_clock_off = false;
	bool ok;

	if ((SDHC_GetPresentStatusFlags(cfg->base) &
	     ((uint32_t)kSDHC_CommandInhibitFlag | (uint32_t)kSDHC_DataInhibitFlag |
	      (uint32_t)kSDHC_DataLineActiveFlag)) != 0U) {
		return -EBUSY;
	}

	if ((cfg->base->SYSCTL & SDHC_SYSCTL_SDCLKEN_MASK) == 0U) {
		src_clk_hz = kinetis_sdhc_src_clock_hz();
		if (src_clk_hz == 0U) {
			return -ENOTSUP;
		}

		init_clock_hz = data->host_io.clock;
		if (init_clock_hz == 0U) {
			init_clock_hz = cfg->min_bus_freq;
		}

		if ((init_clock_hz < cfg->min_bus_freq) || (init_clock_hz > cfg->max_bus_freq) ||
		    (SDHC_SetSdClock(cfg->base, src_clk_hz, init_clock_hz) == 0U)) {
			return -ENOTSUP;
		}

		SDHC_EnableSdClock(cfg->base, true);
		restore_clock_off = (data->host_io.clock == 0U);
	}

	ok = SDHC_SetCardActive(cfg->base, KINETIS_SDHC_DEFAULT_TIMEOUT_MS);
	if (restore_clock_off) {
		SDHC_EnableSdClock(cfg->base, false);
	}

	return ok ? 0 : -EIO;
}

static void kinetis_sdhc_transfer_complete(SDHC_Type *base, sdhc_handle_t *handle, status_t status,
					   void *user_data)
{
	ARG_UNUSED(base);
	ARG_UNUSED(handle);

	const struct device *dev = user_data;
	struct kinetis_sdhc_data *data = dev->data;

	switch ((uint32_t)status) {
	case (uint32_t)kStatus_SDHC_TransferCommandComplete:
		data->transfer_status |= TRANSFER_CMD_COMPLETE;
		break;
	case (uint32_t)kStatus_SDHC_TransferDataComplete:
		data->transfer_status |= TRANSFER_DATA_COMPLETE;
		break;
	case (uint32_t)kStatus_SDHC_SendCommandFailed:
		data->transfer_status |= TRANSFER_CMD_FAILED;
		break;
	case (uint32_t)kStatus_SDHC_TransferDataFailed:
		data->transfer_status |= TRANSFER_DATA_FAILED;
		break;
	default:
		data->transfer_status |= TRANSFER_CMD_FAILED;
		break;
	}

	k_sem_give(&data->transfer_sem);
}

static void kinetis_sdhc_sdio_interrupt(SDHC_Type *base, void *user_data)
{
	ARG_UNUSED(base);

	const struct device *dev = user_data;
	struct kinetis_sdhc_data *data = dev->data;

	if (((data->enabled_sources & SDHC_INT_SDIO) != 0) && (data->card_cb != NULL)) {
		data->card_cb(dev, SDHC_INT_SDIO, data->card_cb_user_data);
	}
}

static void kinetis_sdhc_card_inserted(SDHC_Type *base, void *user_data)
{
	const struct device *dev = user_data;
	struct kinetis_sdhc_data *data = dev->data;

	data->cd_present = true;
	SDHC_DisableInterruptSignal(base, (uint32_t)kSDHC_CardInsertionFlag);
	SDHC_DisableInterruptStatus(base, (uint32_t)kSDHC_CardInsertionFlag);
	if ((data->enabled_sources & SDHC_INT_REMOVED) != 0) {
		SDHC_ClearInterruptStatusFlags(base, (uint32_t)kSDHC_CardRemovalFlag);
		SDHC_EnableInterruptStatus(base, (uint32_t)kSDHC_CardRemovalFlag);
		SDHC_EnableInterruptSignal(base, (uint32_t)kSDHC_CardRemovalFlag);
	}

	if (((data->enabled_sources & SDHC_INT_INSERTED) != 0) && (data->card_cb != NULL)) {
		data->card_cb(dev, SDHC_INT_INSERTED, data->card_cb_user_data);
	}
}

static void kinetis_sdhc_card_removed(SDHC_Type *base, void *user_data)
{
	const struct device *dev = user_data;
	struct kinetis_sdhc_data *data = dev->data;

	data->cd_present = false;
	SDHC_DisableInterruptSignal(base, (uint32_t)kSDHC_CardRemovalFlag);
	SDHC_DisableInterruptStatus(base, (uint32_t)kSDHC_CardRemovalFlag);
	if ((data->enabled_sources & SDHC_INT_INSERTED) != 0) {
		SDHC_ClearInterruptStatusFlags(base, (uint32_t)kSDHC_CardInsertionFlag);
		SDHC_EnableInterruptStatus(base, (uint32_t)kSDHC_CardInsertionFlag);
		SDHC_EnableInterruptSignal(base, (uint32_t)kSDHC_CardInsertionFlag);
	}

	if (((data->enabled_sources & SDHC_INT_REMOVED) != 0) && (data->card_cb != NULL)) {
		data->card_cb(dev, SDHC_INT_REMOVED, data->card_cb_user_data);
	}
}

static void kinetis_cd_gpio_cb(const struct device *port, struct gpio_callback *cb,
			       gpio_port_pins_t pins)
{
	ARG_UNUSED(port);
	ARG_UNUSED(pins);

	struct kinetis_sdhc_data *data = CONTAINER_OF(cb, struct kinetis_sdhc_data, cd_cb);
	const struct device *dev = data->dev;
	const struct kinetis_sdhc_config *cfg = dev->config;
	int v;
	bool present;

	if (((data->enabled_sources & (SDHC_INT_INSERTED | SDHC_INT_REMOVED)) == 0) ||
	    (data->card_cb == NULL)) {
		return;
	}

	v = gpio_pin_get_dt(&cfg->cd_gpio);
	if (v < 0) {
		return;
	}

	present = (v > 0);
	if (present == data->cd_present) {
		return;
	}

	data->cd_present = present;
	data->card_cb(dev, present ? SDHC_INT_INSERTED : SDHC_INT_REMOVED, data->card_cb_user_data);

	(void)gpio_pin_interrupt_configure_dt(&cfg->cd_gpio,
					      present ? GPIO_INT_EDGE_TO_INACTIVE
						      : GPIO_INT_EDGE_TO_ACTIVE);
}

static int kinetis_sdhc_reset(const struct device *dev)
{
	struct kinetis_sdhc_data *data = dev->data;
	int ret;

	k_mutex_lock(&data->lock, K_FOREVER);
	ret = kinetis_sdhc_reset_locked(dev);
	k_mutex_unlock(&data->lock);

	return ret;
}

static int kinetis_sdhc_get_host_props(const struct device *dev, struct sdhc_host_props *props)
{
	memcpy(props, &((struct kinetis_sdhc_data *)dev->data)->props, sizeof(*props));
	return 0;
}

static int kinetis_sdhc_set_io(const struct device *dev, struct sdhc_io *ios)
{
	struct kinetis_sdhc_data *data = dev->data;
	int ret;

	k_mutex_lock(&data->lock, K_FOREVER);
	ret = kinetis_sdhc_set_io_locked(dev, ios);
	k_mutex_unlock(&data->lock);

	return ret;
}

static int kinetis_sdhc_get_card_present(const struct device *dev)
{
	const struct kinetis_sdhc_config *cfg = dev->config;
	struct kinetis_sdhc_data *data = dev->data;

	if (cfg->cd_gpio.port != NULL) {
		int v = gpio_pin_get_dt(&cfg->cd_gpio);

		if (v < 0) {
			return v;
		}

		data->cd_present = (v > 0);
		return data->cd_present;
	}

	data->cd_present =
		(SDHC_GetPresentStatusFlags(cfg->base) & (uint32_t)kSDHC_CardInsertedFlag) != 0U;

	return data->cd_present;
}

static int kinetis_sdhc_card_busy(const struct device *dev)
{
	const struct kinetis_sdhc_config *cfg = dev->config;
	uint32_t s = SDHC_GetPresentStatusFlags(cfg->base);

	return (s & (uint32_t)kSDHC_DataLineActiveFlag) != 0U;
}

static int kinetis_sdhc_execute_tuning(const struct device *dev)
{
	ARG_UNUSED(dev);
	return -ENOTSUP;
}

static int kinetis_sdhc_request(const struct device *dev, struct sdhc_command *cmd,
				struct sdhc_data *data)
{
	const struct kinetis_sdhc_config *cfg = dev->config;
	struct kinetis_sdhc_data *dev_data = dev->data;
	sdhc_command_t sdhc_cmd = {0};
	sdhc_data_t sdhc_data = {0};
	sdhc_transfer_t transfer = {0};
	status_t st;
	int ret;

	k_mutex_lock(&dev_data->lock, K_FOREVER);
	kinetis_sdhc_clear_transfer_state(dev_data);

	if (cmd->opcode == SD_GO_IDLE_STATE) {
		ret = kinetis_sdhc_go_idle_locked(dev);
		k_mutex_unlock(&dev_data->lock);
		return ret;
	}

	sdhc_cmd.index = cmd->opcode;
	sdhc_cmd.argument = cmd->arg;
	sdhc_cmd.type = kCARD_CommandTypeNormal;
	sdhc_cmd.responseType = (cmd->response_type & SDHC_NATIVE_RESPONSE_MASK);
	sdhc_cmd.responseErrorFlags = 0U;
	transfer.command = &sdhc_cmd;

	if (data != NULL) {
		bool write = kinetis_sdhc_cmd_is_write(cmd);
		bool read = kinetis_sdhc_cmd_is_read(cmd);

		if (!write && !read) {
			k_mutex_unlock(&dev_data->lock);
			return -ENOTSUP;
		}

		sdhc_data.enableAutoCommand12 = (data->blocks > 1U);
		sdhc_data.enableIgnoreError = false;
		sdhc_data.blockSize = data->block_size;
		sdhc_data.blockCount = data->blocks;

		if (read) {
			sdhc_data.rxData = (uint32_t *)data->data;
		} else {
			sdhc_data.txData = (const uint32_t *)data->data;
		}

		transfer.data = &sdhc_data;
	}

	st = SDHC_TransferNonBlocking(cfg->base, &dev_data->transfer_handle, dev_data->adma_table,
				      ARRAY_SIZE(dev_data->adma_table), &transfer);
	if (st != kStatus_Success) {
		ret = (st == kStatus_SDHC_BusyTransferring) ? -EBUSY : -EIO;
		(void)kinetis_sdhc_recover_locked(dev);
		k_mutex_unlock(&dev_data->lock);
		return ret;
	}

	ret = kinetis_sdhc_wait_status(dev_data, TRANSFER_CMD_FLAGS, cmd->timeout_ms);
	if (ret != 0) {
		(void)kinetis_sdhc_recover_locked(dev);
		k_mutex_unlock(&dev_data->lock);
		return ret;
	}

	if ((dev_data->transfer_status & TRANSFER_CMD_FAILED) != 0U) {
		ret = kinetis_sdhc_xfer_error(dev);
		(void)kinetis_sdhc_recover_locked(dev);
		k_mutex_unlock(&dev_data->lock);
		return ret;
	}

	memcpy(cmd->response, sdhc_cmd.response, sizeof(cmd->response));

	if (data != NULL) {
		ret = kinetis_sdhc_wait_status(dev_data, TRANSFER_DATA_FLAGS, data->timeout_ms);
		if (ret != 0) {
			(void)kinetis_sdhc_recover_locked(dev);
			k_mutex_unlock(&dev_data->lock);
			return ret;
		}

		if ((dev_data->transfer_status & TRANSFER_DATA_FAILED) != 0U) {
			ret = kinetis_sdhc_xfer_error(dev);
			(void)kinetis_sdhc_recover_locked(dev);
			k_mutex_unlock(&dev_data->lock);
			return ret;
		}

		data->bytes_xfered = data->blocks * data->block_size;
	}

	k_mutex_unlock(&dev_data->lock);
	return 0;
}

static int kinetis_sdhc_enable_interrupt(const struct device *dev, sdhc_interrupt_cb_t callback,
					 int sources, void *user_data)
{
	struct kinetis_sdhc_data *data = dev->data;
	const struct kinetis_sdhc_config *cfg = dev->config;
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
		int v = gpio_pin_get_dt(&cfg->cd_gpio);

		if (v < 0) {
			k_mutex_unlock(&data->lock);
			return v;
		}

		data->cd_present = (v > 0);
	}

	ret = kinetis_sdhc_sync_card_interrupts(dev);
	k_mutex_unlock(&data->lock);

	return ret;
}

static int kinetis_sdhc_disable_interrupt(const struct device *dev, int sources)
{
	struct kinetis_sdhc_data *data = dev->data;
	int ret;

	k_mutex_lock(&data->lock, K_FOREVER);
	data->enabled_sources &= ~sources;
	if (data->enabled_sources == 0) {
		data->card_cb = NULL;
		data->card_cb_user_data = NULL;
	}

	ret = kinetis_sdhc_sync_card_interrupts(dev);
	k_mutex_unlock(&data->lock);

	return ret;
}

static void kinetis_sdhc_isr(const struct device *dev)
{
	const struct kinetis_sdhc_config *cfg = dev->config;
	struct kinetis_sdhc_data *data = dev->data;

	SDHC_TransferHandleIRQ(cfg->base, &data->transfer_handle);
}

static int kinetis_sdhc_init(const struct device *dev)
{
	const struct kinetis_sdhc_config *cfg = dev->config;
	struct kinetis_sdhc_data *data = dev->data;
	sdhc_transfer_callback_t cb = {0};
	sdhc_capability_t cap = {0};
	sdhc_config_t host_cfg;
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

	kinetis_sdhc_init_host_cfg(&host_cfg);
	SDHC_Init(cfg->base, &host_cfg);

	cb.TransferComplete = kinetis_sdhc_transfer_complete;
	cb.SdioInterrupt = kinetis_sdhc_sdio_interrupt;
	cb.CardInserted = kinetis_sdhc_card_inserted;
	cb.CardRemoved = kinetis_sdhc_card_removed;
	SDHC_TransferCreateHandle(cfg->base, &data->transfer_handle, &cb, (void *)dev);

	k_mutex_init(&data->lock);
	k_sem_init(&data->transfer_sem, 0, 1);
	data->dev = dev;
	memset(&data->host_io, 0, sizeof(data->host_io));

	memset(&data->props, 0, sizeof(data->props));
	data->props.f_min = cfg->min_bus_freq;
	data->props.f_max = cfg->max_bus_freq;
	data->props.power_delay = cfg->power_delay_ms;
	data->props.bus_4_bit_support = (cfg->bus_width >= 4U);
	data->props.host_caps.vol_330_support = true;
	data->props.host_caps.bus_8_bit_support = (cfg->bus_width >= 8U);

	SDHC_GetCapability(cfg->base, &cap);
	data->props.host_caps.high_spd_support =
		(cap.flags & (uint32_t)kSDHC_SupportHighSpeedFlag) != 0U;
	data->props.host_caps.suspend_res_support =
		(cap.flags & (uint32_t)kSDHC_SupportSuspendResumeFlag) != 0U;
	data->props.host_caps.sdma_support = (cap.flags & (uint32_t)kSDHC_SupportDmaFlag) != 0U;
	data->props.host_caps.adma_2_support = (cap.flags & (uint32_t)kSDHC_SupportAdmaFlag) != 0U;
	data->props.host_caps.max_blk_len = kinetis_sdhc_max_blk_len(cap.maxBlockLength);
#if defined(FSL_FEATURE_SDHC_HAS_V300_SUPPORT) && FSL_FEATURE_SDHC_HAS_V300_SUPPORT
	data->props.host_caps.vol_300_support = (cap.flags & (uint32_t)kSDHC_SupportV300Flag) != 0U;
#endif
#if defined(FSL_FEATURE_SDHC_HAS_V180_SUPPORT) && FSL_FEATURE_SDHC_HAS_V180_SUPPORT
	data->props.host_caps.vol_180_support = (cap.flags & (uint32_t)kSDHC_SupportV180Flag) != 0U;
#endif

	if (cfg->cd_gpio.port != NULL) {
		ret = gpio_pin_configure_dt(&cfg->cd_gpio, GPIO_INPUT);
		if (ret != 0) {
			LOG_ERR("CD gpio_pin_configure_dt failed (%d)", ret);
			return ret;
		}

		gpio_init_callback(&data->cd_cb, kinetis_cd_gpio_cb, BIT(cfg->cd_gpio.pin));
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

		data->cd_present = (ret > 0);
		ret = gpio_pin_interrupt_configure_dt(&cfg->cd_gpio, GPIO_INT_DISABLE);
		if (ret != 0) {
			LOG_ERR("CD gpio disable failed (%d)", ret);
			return ret;
		}
	}

	cfg->irq_config_func(dev);

	return 0;
}

static DEVICE_API(sdhc, kinetis_sdhc_api) = {
	.reset = kinetis_sdhc_reset,
	.request = kinetis_sdhc_request,
	.set_io = kinetis_sdhc_set_io,
	.get_card_present = kinetis_sdhc_get_card_present,
	.execute_tuning = kinetis_sdhc_execute_tuning,
	.card_busy = kinetis_sdhc_card_busy,
	.get_host_props = kinetis_sdhc_get_host_props,
	.enable_interrupt = kinetis_sdhc_enable_interrupt,
	.disable_interrupt = kinetis_sdhc_disable_interrupt,
};

#define KINETIS_SDHC_IRQ_CONFIG(inst)                                                           \
	static void kinetis_sdhc_irq_config_##inst(const struct device *dev)                     \
	{                                                                                         \
		ARG_UNUSED(dev);                                                                  \
		IRQ_CONNECT(DT_INST_IRQN(inst), DT_INST_IRQ(inst, priority), kinetis_sdhc_isr,     \
			    DEVICE_DT_INST_GET(inst), 0);                                        \
		irq_enable(DT_INST_IRQN(inst));                                                   \
	}

#define KINETIS_SDHC_INIT(inst)                                                                  \
	PINCTRL_DT_INST_DEFINE(inst);                                                            \
	KINETIS_SDHC_IRQ_CONFIG(inst);                                                           \
	static const struct kinetis_sdhc_config kinetis_sdhc_cfg_##inst = {                       \
		.base = (SDHC_Type *)DT_INST_REG_ADDR(inst),                                      \
		.pincfg = PINCTRL_DT_INST_DEV_CONFIG_GET(inst),                                  \
		.cd_gpio = GPIO_DT_SPEC_INST_GET_OR(inst, cd_gpios, {0}),                       \
		.clock_dev = DEVICE_DT_GET(DT_INST_CLOCKS_CTLR(inst)),                          \
		.clock_subsys =                                                                  \
			(clock_control_subsys_t)(uintptr_t)                                      \
				KINETIS_SIM_CLOCK_ID(DT_INST_CLOCKS_CELL(inst, name),           \
						     DT_INST_CLOCKS_CELL(inst, offset),         \
						     DT_INST_CLOCKS_CELL(inst, bits)),          \
		.bus_width = DT_INST_PROP(inst, bus_width),                                      \
		.power_delay_ms = DT_INST_PROP(inst, power_delay_ms),                            \
		.min_bus_freq = DT_INST_PROP(inst, min_bus_freq),                                \
		.max_bus_freq = DT_INST_PROP(inst, max_bus_freq),                                \
		.irq_config_func = kinetis_sdhc_irq_config_##inst,                               \
	};                                                                                        \
	static struct kinetis_sdhc_data kinetis_sdhc_data_##inst;                                 \
	DEVICE_DT_INST_DEFINE(inst, kinetis_sdhc_init, NULL, &kinetis_sdhc_data_##inst,           \
			      &kinetis_sdhc_cfg_##inst, POST_KERNEL, CONFIG_SDHC_INIT_PRIORITY, \
			      &kinetis_sdhc_api);

DT_INST_FOREACH_STATUS_OKAY(KINETIS_SDHC_INIT)
