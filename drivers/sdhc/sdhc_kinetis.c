/*
 * Copyright (c) 2026 Ylhyra ehf.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT nxp_kinetis_sdhc

#include <errno.h>
#include <stdint.h>
#include <string.h>

#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/pinctrl.h>
#include <zephyr/drivers/sdhc.h>
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

#define KINETIS_SDHC_RESET_TIMEOUT (1000000U)
#define KINETIS_SDHC_DEFAULT_TIMEOUT_MS 5000U

struct kinetis_sdhc_config {
	SDHC_Type *base;
	const struct pinctrl_dev_config *pincfg;

	uint32_t clock_gate;

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

	uint32_t adma_table[CONFIG_SDHC_KINETIS_ADMA_TABLE_SIZE / sizeof(uint32_t)]
		__aligned(sizeof(uint32_t));
};

static uint32_t kinetis_sdhc_src_clock_hz(void)
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

	if ((data->enabled_sources & SDHC_INT_SDIO) == 0) {
		return;
	}

	if (data->card_cb) {
		data->card_cb(dev, SDHC_INT_SDIO, data->card_cb_user_data);
	}
}

static void kinetis_sdhc_card_inserted(SDHC_Type *base, void *user_data)
{
	ARG_UNUSED(base);

	const struct device *dev = user_data;
	struct kinetis_sdhc_data *data = dev->data;

	if ((data->enabled_sources & SDHC_INT_INSERTED) == 0) {
		return;
	}

	if (data->card_cb) {
		data->card_cb(dev, SDHC_INT_INSERTED, data->card_cb_user_data);
	}
}

static void kinetis_sdhc_card_removed(SDHC_Type *base, void *user_data)
{
	ARG_UNUSED(base);

	const struct device *dev = user_data;
	struct kinetis_sdhc_data *data = dev->data;

	if ((data->enabled_sources & SDHC_INT_REMOVED) == 0) {
		return;
	}

	if (data->card_cb) {
		data->card_cb(dev, SDHC_INT_REMOVED, data->card_cb_user_data);
	}
}

static int kinetis_sdhc_reset(const struct device *dev)
{
	const struct kinetis_sdhc_config *cfg = dev->config;

	SDHC_DisableInterruptSignal(cfg->base, (uint32_t)kSDHC_AllInterruptFlags);
	SDHC_DisableInterruptStatus(cfg->base, (uint32_t)kSDHC_AllInterruptFlags);
	SDHC_ClearInterruptStatusFlags(cfg->base, (uint32_t)kSDHC_AllInterruptFlags);

	if (!SDHC_Reset(cfg->base, (uint32_t)kSDHC_ResetAll, KINETIS_SDHC_RESET_TIMEOUT)) {
		return -ETIMEDOUT;
	}

	return 0;
}

static int kinetis_sdhc_get_host_props(const struct device *dev, struct sdhc_host_props *props)
{
	struct kinetis_sdhc_data *data = dev->data;

	*props = data->props;
	return 0;
}

static int kinetis_sdhc_set_io(const struct device *dev, struct sdhc_io *ios)
{
	const struct kinetis_sdhc_config *cfg = dev->config;
	struct kinetis_sdhc_data *data = dev->data;
	uint32_t src_clk_hz;

	if ((ios->timing != 0) && (ios->timing != SDHC_TIMING_LEGACY) && (ios->timing != SDHC_TIMING_HS)) {
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

	if (ios->clock == 0) {
		SDHC_EnableSdClock(cfg->base, false);
	} else if ((ios->clock >= cfg->min_bus_freq) && (ios->clock <= cfg->max_bus_freq)) {
		(void)SDHC_SetSdClock(cfg->base, src_clk_hz, ios->clock);
		SDHC_EnableSdClock(cfg->base, true);
	} else {
		return -ENOTSUP;
	}

	switch (ios->bus_width) {
	case 0:
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
	} else if (ios->power_mode != 0 && ios->power_mode != SDHC_POWER_ON) {
		return -ENOTSUP;
	}

	data->host_io = *ios;
	return 0;
}

static int kinetis_sdhc_get_card_present(const struct device *dev)
{
	const struct kinetis_sdhc_config *cfg = dev->config;
	uint32_t s = SDHC_GetPresentStatusFlags(cfg->base);

	return (s & (uint32_t)kSDHC_CardInsertedFlag) != 0U;
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

static int kinetis_sdhc_wait_status(struct kinetis_sdhc_data *data, uint32_t mask,
				    int timeout_ms)
{
	int64_t deadline = (timeout_ms == SDHC_TIMEOUT_FOREVER) ? INT64_MAX : k_uptime_get() + timeout_ms;

	while ((data->transfer_status & mask) == 0U) {
		int take_ms;

		if (timeout_ms == SDHC_TIMEOUT_FOREVER) {
			take_ms = KINETIS_SDHC_DEFAULT_TIMEOUT_MS;
		} else {
			int64_t remaining = deadline - k_uptime_get();
			if (remaining <= 0) {
				return -ETIMEDOUT;
			}
			take_ms = (remaining > KINETIS_SDHC_DEFAULT_TIMEOUT_MS) ?
					  KINETIS_SDHC_DEFAULT_TIMEOUT_MS :
					  (int)remaining;
		}

		int ret = k_sem_take(&data->transfer_sem, K_MSEC(take_ms));
		if (ret != 0) {
			return -ETIMEDOUT;
		}
	}

	return 0;
}

static int kinetis_sdhc_request(const struct device *dev, struct sdhc_command *cmd,
				struct sdhc_data *data)
{
	const struct kinetis_sdhc_config *cfg = dev->config;
	struct kinetis_sdhc_data *dev_data = dev->data;
	status_t st;

	k_mutex_lock(&dev_data->lock, K_FOREVER);

	dev_data->transfer_status = 0U;

	if (cmd->opcode == SD_GO_IDLE_STATE) {
		bool ok = SDHC_SetCardActive(cfg->base, KINETIS_SDHC_DEFAULT_TIMEOUT_MS);
		k_mutex_unlock(&dev_data->lock);
		return ok ? 0 : -EIO;
	}

	sdhc_command_t sdhc_cmd = {0};
	sdhc_data_t sdhc_data = {0};
	sdhc_transfer_t transfer = {0};

	sdhc_cmd.index = cmd->opcode;
	sdhc_cmd.argument = cmd->arg;
	sdhc_cmd.type = kCARD_CommandTypeNormal;
	sdhc_cmd.responseType = (cmd->response_type & SDHC_NATIVE_RESPONSE_MASK);
	sdhc_cmd.responseErrorFlags = 0U;
	transfer.command = &sdhc_cmd;

	if (data) {
		bool write = kinetis_sdhc_cmd_is_write(cmd);
		bool read = kinetis_sdhc_cmd_is_read(cmd);

		if (!write && !read) {
			k_mutex_unlock(&dev_data->lock);
			return -ENOTSUP;
		}

		sdhc_data.enableAutoCommand12 = false;
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
		k_mutex_unlock(&dev_data->lock);
		return -EIO;
	}

	int ret = kinetis_sdhc_wait_status(dev_data, TRANSFER_CMD_FLAGS, cmd->timeout_ms);
	if (ret != 0) {
		k_mutex_unlock(&dev_data->lock);
		return ret;
	}
	if ((dev_data->transfer_status & TRANSFER_CMD_FAILED) != 0U) {
		k_mutex_unlock(&dev_data->lock);
		return -EIO;
	}

	memcpy(cmd->response, sdhc_cmd.response, sizeof(cmd->response));

	if (data) {
		ret = kinetis_sdhc_wait_status(dev_data, TRANSFER_DATA_FLAGS, data->timeout_ms);
		if (ret != 0) {
			k_mutex_unlock(&dev_data->lock);
			return ret;
		}

		if ((dev_data->transfer_status & TRANSFER_DATA_FAILED) != 0U) {
			k_mutex_unlock(&dev_data->lock);
			return -EIO;
		}

		data->bytes_xfered = data->blocks * data->block_size;
	}

	k_mutex_unlock(&dev_data->lock);
	return 0;
}

static int kinetis_sdhc_enable_interrupt(const struct device *dev, sdhc_interrupt_cb_t callback,
					int sources, void *user_data)
{
	const struct kinetis_sdhc_config *cfg = dev->config;
	struct kinetis_sdhc_data *data = dev->data;

	data->card_cb = callback;
	data->card_cb_user_data = user_data;
	data->enabled_sources = sources;

	uint32_t mask = 0U;
	if ((sources & SDHC_INT_SDIO) != 0) {
		mask |= (uint32_t)kSDHC_CardInterruptFlag;
	}
	if ((sources & (SDHC_INT_INSERTED | SDHC_INT_REMOVED)) != 0) {
		mask |= (uint32_t)kSDHC_CardDetectFlag;
	}

	SDHC_EnableInterruptStatus(cfg->base, mask);
	SDHC_EnableInterruptSignal(cfg->base, mask);

	return 0;
}

static int kinetis_sdhc_disable_interrupt(const struct device *dev, int sources)
{
	const struct kinetis_sdhc_config *cfg = dev->config;
	struct kinetis_sdhc_data *data = dev->data;

	data->enabled_sources &= ~sources;

	uint32_t mask = 0U;
	if ((sources & SDHC_INT_SDIO) != 0) {
		mask |= (uint32_t)kSDHC_CardInterruptFlag;
	}
	if ((sources & (SDHC_INT_INSERTED | SDHC_INT_REMOVED)) != 0) {
		mask |= (uint32_t)kSDHC_CardDetectFlag;
	}

	SDHC_DisableInterruptSignal(cfg->base, mask);
	SDHC_DisableInterruptStatus(cfg->base, mask);
	SDHC_ClearInterruptStatusFlags(cfg->base, mask);

	return 0;
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
	int ret;

	ret = pinctrl_apply_state(cfg->pincfg, PINCTRL_STATE_DEFAULT);
	if (ret != 0) {
		return ret;
	}

	CLOCK_EnableClock((clock_ip_name_t)cfg->clock_gate);

	sdhc_config_t host_cfg = {0};
	host_cfg.cardDetectDat3 = false;
	host_cfg.endianMode = kSDHC_EndianModeLittle;
	host_cfg.dmaMode = kSDHC_DmaModeAdma2;
	host_cfg.readWatermarkLevel = 128U;
	host_cfg.writeWatermarkLevel = 128U;
	SDHC_Init(cfg->base, &host_cfg);

	sdhc_transfer_callback_t cb = {0};
	cb.TransferComplete = kinetis_sdhc_transfer_complete;
	cb.SdioInterrupt = kinetis_sdhc_sdio_interrupt;
	cb.CardInserted = kinetis_sdhc_card_inserted;
	cb.CardRemoved = kinetis_sdhc_card_removed;
	SDHC_TransferCreateHandle(cfg->base, &data->transfer_handle, &cb, (void *)dev);

	k_mutex_init(&data->lock);
	k_sem_init(&data->transfer_sem, 0, 1);

	memset(&data->props, 0, sizeof(data->props));
	data->props.f_min = cfg->min_bus_freq;
	data->props.f_max = cfg->max_bus_freq;
	data->props.power_delay = cfg->power_delay_ms;
	data->props.host_caps.vol_330_support = true;
	data->props.host_caps.bus_4_bit_support = (cfg->bus_width >= 4U);
	data->props.host_caps.bus_8_bit_support = (cfg->bus_width >= 8U);

	sdhc_capability_t cap = {0};
	SDHC_GetCapability(cfg->base, &cap);
	data->props.host_caps.high_spd_support = (cap.flags & (uint32_t)kSDHC_SupportHighSpeedFlag) != 0U;
	data->props.host_caps.suspend_res_support =
		(cap.flags & (uint32_t)kSDHC_SupportSuspendResumeFlag) != 0U;
	data->props.host_caps.sdma_support = (cap.flags & (uint32_t)kSDHC_SupportDmaFlag) != 0U;
	data->props.host_caps.adma_2_support = (cap.flags & (uint32_t)kSDHC_SupportAdmaFlag) != 0U;

	ret = kinetis_sdhc_reset(dev);
	if (ret != 0) {
		return ret;
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
		.clock_gate = CLK_GATE_DEFINE(DT_INST_CLOCKS_CELL(inst, offset),                 \
					      DT_INST_CLOCKS_CELL(inst, bits)),                      \
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
