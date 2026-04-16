/*
 * Copyright (c) 2026 The Zephyr Project Contributors
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT nxp_kinetis_sdhc

#include <errno.h>
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
#include <zephyr/sys/util.h>

#include <soc.h>
#include <fsl_clock.h>
#include <fsl_sdhc.h>

LOG_MODULE_REGISTER(sdhc_kinetis, CONFIG_SDHC_LOG_LEVEL);

#define KINETIS_SDHC_RESET_TIMEOUT         1000000U
#define KINETIS_SDHC_DEFAULT_TIMEOUT_MS    5000U
#define KINETIS_SDHC_CARD_READY_TIMEOUT_MS 1000U
#define KINETIS_SDHC_RECOVERY_POLL_US      125U
#define KINETIS_SDHC_DMA_THRESHOLD_BYTES   512U
#define KINETIS_SDHC_READ_WATERMARK_LEVEL  128U
#define KINETIS_SDHC_WRITE_WATERMARK_LEVEL 16U
#define KINETIS_SDHC_ADMA_TABLE_WORDS      32U
#define KINETIS_SDHC_ADMA_ALIGNMENT        8U
#define KINETIS_SDHC_DMA_ALIGNMENT         4U
#define KINETIS_SDHC_DMA_BUF_ALIGNMENT MAX(CONFIG_SDHC_BUFFER_ALIGNMENT, KINETIS_SDHC_DMA_ALIGNMENT)
#define KINETIS_SDHC_TRANSFER_IRQ_FLAGS                                                            \
	((uint32_t)kSDHC_CommandFlag | (uint32_t)kSDHC_DataFlag | (uint32_t)kSDHC_DataDMAFlag)

enum kinetis_sdhc_req_event {
	KINETIS_SDHC_REQ_CMD_DONE = BIT(0),
	KINETIS_SDHC_REQ_DATA_DONE = BIT(1),
	KINETIS_SDHC_REQ_FAILED = BIT(2),
};

struct kinetis_sdhc_hw_snapshot {
	uint32_t irqstat;
	uint32_t prsstat;
	uint32_t ac12err;
	uint32_t admaes;
	uint32_t adsaddr;
	uint32_t dsaddr;
	uint32_t blkattr;
	uint32_t xfertyp;
	uint32_t cmdarg;
};

struct kinetis_sdhc_req {
	struct sdhc_command *z_cmd;
	struct sdhc_data *z_data;
	sdhc_command_t hal_cmd;
	sdhc_data_t hal_data;
	uint8_t *buf;
	size_t len;
	volatile uint32_t events;
	volatile int result;
	uint32_t transferred_words;
	bool use_dma;
	bool is_write;
	bool is_multiblock;
	bool use_auto_cmd12;
	struct kinetis_sdhc_hw_snapshot first_snapshot;
	struct kinetis_sdhc_hw_snapshot last_snapshot;
	bool first_snapshot_valid;
};

struct kinetis_sdhc_config {
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
	uint32_t irq_num;
	void (*irq_config_func)(const struct device *dev);
};

struct kinetis_sdhc_data {
	struct sdhc_host_props props;
	struct sdhc_io host_io;
	struct k_mutex lock;
	struct k_sem transfer_sem;
	sdhc_interrupt_cb_t card_cb;
	void *card_cb_user_data;
	int enabled_sources;
	struct gpio_callback cd_cb;
	const struct device *dev;
	bool cd_present;
	struct kinetis_sdhc_req *active_req;
	uint32_t adma_table[KINETIS_SDHC_ADMA_TABLE_WORDS] __aligned(KINETIS_SDHC_ADMA_ALIGNMENT);
};

static int kinetis_sdhc_sync_card_interrupts(const struct device *dev);
static void kinetis_sdhc_arm_host_cd_interrupts(const struct device *dev, bool present);
static void kinetis_sdhc_power_on_reset(const struct device *dev);

static uint32_t kinetis_sdhc_initial_dma_mode(const struct kinetis_sdhc_config *cfg)
{
	return (cfg->base->HTCAPBLT & SDHC_HTCAPBLT_ADMAS_MASK) != 0U ? (uint32_t)kSDHC_DmaModeAdma2
								      : (uint32_t)kSDHC_DmaModeNo;
}

static void kinetis_sdhc_init_host_cfg(const struct kinetis_sdhc_config *cfg,
				       sdhc_config_t *host_cfg)
{
	memset(host_cfg, 0, sizeof(*host_cfg));
	host_cfg->cardDetectDat3 = false;
	host_cfg->endianMode = kSDHC_EndianModeLittle;
	host_cfg->dmaMode = kinetis_sdhc_initial_dma_mode(cfg);
	host_cfg->readWatermarkLevel = KINETIS_SDHC_READ_WATERMARK_LEVEL;
	host_cfg->writeWatermarkLevel = KINETIS_SDHC_WRITE_WATERMARK_LEVEL;
}

static void kinetis_sdhc_take_snapshot(SDHC_Type *base, struct kinetis_sdhc_hw_snapshot *snapshot)
{
	snapshot->irqstat = SDHC_GetInterruptStatusFlags(base);
	snapshot->prsstat = SDHC_GetPresentStatusFlags(base);
	snapshot->ac12err = base->AC12ERR;
	snapshot->admaes = base->ADMAES;
	snapshot->adsaddr = base->ADSADDR;
	snapshot->dsaddr = base->DSADDR;
	snapshot->blkattr = base->BLKATTR;
	snapshot->xfertyp = base->XFERTYP;
	snapshot->cmdarg = base->CMDARG;
}

static void kinetis_sdhc_req_record_snapshot(struct kinetis_sdhc_req *req, SDHC_Type *base)
{
	kinetis_sdhc_take_snapshot(base, &req->last_snapshot);

	if (((req->last_snapshot.irqstat & (uint32_t)kSDHC_ErrorFlag) != 0U) &&
	    !req->first_snapshot_valid) {
		req->first_snapshot = req->last_snapshot;
		req->first_snapshot_valid = true;
	}
}

static const struct kinetis_sdhc_hw_snapshot *
kinetis_sdhc_primary_snapshot(const struct kinetis_sdhc_req *req)
{
	return req->first_snapshot_valid ? &req->first_snapshot : &req->last_snapshot;
}

static void kinetis_sdhc_disable_transfer_irqs(SDHC_Type *base)
{
	SDHC_DisableInterruptSignal(base, KINETIS_SDHC_TRANSFER_IRQ_FLAGS);
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
	case SD_SWITCH:
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

static bool kinetis_sdhc_is_multiblock_rw(const struct sdhc_command *cmd,
					  const struct sdhc_data *data)
{
	if ((data == NULL) || (data->blocks <= 1U)) {
		return false;
	}

	return (cmd->opcode == SD_READ_MULTIPLE_BLOCK) || (cmd->opcode == SD_WRITE_MULTIPLE_BLOCK);
}

static bool kinetis_sdhc_use_auto_cmd12(const struct sdhc_command *cmd,
					const struct sdhc_data *data)
{
	return kinetis_sdhc_is_multiblock_rw(cmd, data);
}

static uint32_t kinetis_sdhc_response_error_flags(uint32_t response_type)
{
	switch (response_type & SDHC_NATIVE_RESPONSE_MASK) {
	case SD_RSP_TYPE_R1:
	case SD_RSP_TYPE_R1b:
		return SD_R1_ERR_FLAGS;
	default:
		return 0U;
	}
}

static bool kinetis_sdhc_dma_buf_usable(const void *buf, size_t len)
{
	return IS_ALIGNED(buf, KINETIS_SDHC_DMA_BUF_ALIGNMENT) &&
	       ((len % KINETIS_SDHC_DMA_ALIGNMENT) == 0U);
}

static uint32_t kinetis_sdhc_adma_capacity_bytes(void)
{
	uint32_t entries = (KINETIS_SDHC_ADMA_TABLE_WORDS * sizeof(uint32_t)) / 8U;

	return entries * SDHC_ADMA2_DESCRIPTOR_MAX_LENGTH_PER_ENTRY;
}

static bool kinetis_sdhc_request_prefers_dma(const struct kinetis_sdhc_config *cfg,
					     const struct sdhc_command *cmd,
					     const struct sdhc_data *data)
{
	size_t bytes;
	bool write;
	bool read;

	if ((data == NULL) || (kinetis_sdhc_initial_dma_mode(cfg) == (uint32_t)kSDHC_DmaModeNo)) {
		return false;
	}

	write = kinetis_sdhc_cmd_is_write(cmd);
	read = kinetis_sdhc_cmd_is_read(cmd);
	if (!read && !write) {
		return false;
	}

	bytes = (size_t)data->blocks * (size_t)data->block_size;
	if (bytes < KINETIS_SDHC_DMA_THRESHOLD_BYTES) {
		return false;
	}

	if (bytes > kinetis_sdhc_adma_capacity_bytes()) {
		return false;
	}

	return kinetis_sdhc_dma_buf_usable(data->data, bytes);
}

static int kinetis_sdhc_cache_result(int ret)
{
	return (ret == -ENOTSUP) ? 0 : ret;
}

static void kinetis_sdhc_set_dma_mode(SDHC_Type *base, sdhc_dma_mode_t mode)
{
	base->PROCTL = (base->PROCTL & ~SDHC_PROCTL_DMAS_MASK) | SDHC_PROCTL_DMAS(mode);
}

static int kinetis_sdhc_set_power(const struct device *dev, enum sdhc_power power_mode)
{
	const struct kinetis_sdhc_config *cfg = dev->config;
	struct kinetis_sdhc_data *data = dev->data;
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

static int kinetis_sdhc_apply_io(const struct device *dev, const struct sdhc_io *ios)
{
	const struct kinetis_sdhc_config *cfg = dev->config;
	struct kinetis_sdhc_data *data = dev->data;
	uint32_t src_clk_hz;
	uint8_t bus_width;
	bool power_was_off = (data->host_io.power_mode != SDHC_POWER_ON);
	int ret;

	LOG_DBG("set_io clk=%u width=%u power=%u timing=%u volt=%u", ios->clock, ios->bus_width,
		ios->power_mode, ios->timing, ios->signal_voltage);

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

	if (ios->power_mode == SDHC_POWER_OFF) {
		ret = kinetis_sdhc_set_power(dev, SDHC_POWER_OFF);
		if (ret != 0) {
			return ret;
		}
	} else if ((ios->power_mode != 0U) && (ios->power_mode != SDHC_POWER_ON)) {
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

	if (ios->power_mode == SDHC_POWER_ON) {
		ret = kinetis_sdhc_set_power(dev, SDHC_POWER_ON);
		if (ret != 0) {
			return ret;
		}

		if (power_was_off) {
			kinetis_sdhc_power_on_reset(dev);
		}
	}

	data->host_io = *ios;

	return 0;
}

static void kinetis_sdhc_disable_host_cd_interrupts(const struct device *dev)
{
	const struct kinetis_sdhc_config *cfg = dev->config;

	SDHC_DisableInterruptSignal(cfg->base, (uint32_t)kSDHC_CardDetectFlag);
	SDHC_DisableInterruptStatus(cfg->base, (uint32_t)kSDHC_CardDetectFlag);
	SDHC_ClearInterruptStatusFlags(cfg->base, (uint32_t)kSDHC_CardDetectFlag);
}

static void kinetis_sdhc_arm_host_cd_interrupts(const struct device *dev, bool present)
{
	const struct kinetis_sdhc_config *cfg = dev->config;
	struct kinetis_sdhc_data *data = dev->data;

	kinetis_sdhc_disable_host_cd_interrupts(dev);

	if (((data->enabled_sources & (SDHC_INT_INSERTED | SDHC_INT_REMOVED)) == 0) ||
	    (data->card_cb == NULL)) {
		return;
	}

	if (!present && ((data->enabled_sources & SDHC_INT_INSERTED) != 0)) {
		SDHC_EnableInterruptStatus(cfg->base, (uint32_t)kSDHC_CardInsertionFlag);
		SDHC_EnableInterruptSignal(cfg->base, (uint32_t)kSDHC_CardInsertionFlag);
	}

	if (present && ((data->enabled_sources & SDHC_INT_REMOVED) != 0)) {
		SDHC_EnableInterruptStatus(cfg->base, (uint32_t)kSDHC_CardRemovalFlag);
		SDHC_EnableInterruptSignal(cfg->base, (uint32_t)kSDHC_CardRemovalFlag);
	}
}

static int kinetis_sdhc_sync_card_interrupts(const struct device *dev)
{
	const struct kinetis_sdhc_config *cfg = dev->config;
	struct kinetis_sdhc_data *data = dev->data;

	SDHC_DisableInterruptSignal(cfg->base, (uint32_t)kSDHC_CardInterruptFlag);
	SDHC_DisableInterruptStatus(cfg->base, (uint32_t)kSDHC_CardInterruptFlag);
	SDHC_ClearInterruptStatusFlags(cfg->base, (uint32_t)kSDHC_CardInterruptFlag);

	if (((data->enabled_sources & SDHC_INT_SDIO) != 0) && (data->card_cb != NULL)) {
		SDHC_EnableInterruptStatus(cfg->base, (uint32_t)kSDHC_CardInterruptFlag);
		SDHC_EnableInterruptSignal(cfg->base, (uint32_t)kSDHC_CardInterruptFlag);
	}

	if (cfg->cd_gpio.port != NULL) {
		if (((data->enabled_sources & (SDHC_INT_INSERTED | SDHC_INT_REMOVED)) == 0) ||
		    (data->card_cb == NULL)) {
			return gpio_pin_interrupt_configure_dt(&cfg->cd_gpio, GPIO_INT_DISABLE);
		}

		return gpio_pin_interrupt_configure_dt(&cfg->cd_gpio,
						       data->cd_present ? GPIO_INT_EDGE_TO_INACTIVE
									: GPIO_INT_EDGE_TO_ACTIVE);
	}

	data->cd_present =
		(SDHC_GetPresentStatusFlags(cfg->base) & (uint32_t)kSDHC_CardInsertedFlag) != 0U;
	kinetis_sdhc_arm_host_cd_interrupts(dev, data->cd_present);

	return 0;
}

static void kinetis_sdhc_init_controller(const struct device *dev)
{
	const struct kinetis_sdhc_config *cfg = dev->config;
	sdhc_config_t host_cfg;

	kinetis_sdhc_init_host_cfg(cfg, &host_cfg);
	SDHC_Init(cfg->base, &host_cfg);
	SDHC_DisableInterruptSignal(cfg->base, kSDHC_AllInterruptFlags);
	SDHC_EnableInterruptStatus(cfg->base, kSDHC_AllInterruptFlags);
	SDHC_ClearInterruptStatusFlags(cfg->base, kSDHC_AllInterruptFlags);
}

static void kinetis_sdhc_power_on_reset(const struct device *dev)
{
	struct kinetis_sdhc_data *data = dev->data;

	data->active_req = NULL;
	kinetis_sdhc_init_controller(dev);
	(void)kinetis_sdhc_sync_card_interrupts(dev);
}

static int kinetis_sdhc_reset_host(const struct device *dev)
{
	struct kinetis_sdhc_data *data = dev->data;
	int ret;

	data->active_req = NULL;
	kinetis_sdhc_init_controller(dev);

	ret = kinetis_sdhc_apply_io(dev, &data->host_io);
	if (ret != 0) {
		return ret;
	}

	return kinetis_sdhc_sync_card_interrupts(dev);
}

static uint32_t kinetis_sdhc_request_timeout_ms(const struct sdhc_command *cmd,
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

static k_timeout_t kinetis_sdhc_request_timeout(const struct kinetis_sdhc_req *req)
{
	uint32_t timeout_ms = kinetis_sdhc_request_timeout_ms(req->z_cmd, req->z_data);

	return timeout_ms == SDHC_TIMEOUT_FOREVER ? K_FOREVER : K_MSEC(timeout_ms);
}

static int kinetis_sdhc_xfer_error(const struct kinetis_sdhc_hw_snapshot *snapshot)
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

static int kinetis_sdhc_status_to_errno(status_t status,
					const struct kinetis_sdhc_hw_snapshot *snapshot)
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
		return kinetis_sdhc_xfer_error(snapshot);
	}
}

static int kinetis_sdhc_prepare_data_cache(const struct kinetis_sdhc_req *req)
{
	if (!req->use_dma || (req->buf == NULL) || (req->len == 0U)) {
		return 0;
	}

	if (req->is_write) {
		return kinetis_sdhc_cache_result(sys_cache_data_flush_range(req->buf, req->len));
	}

	return kinetis_sdhc_cache_result(sys_cache_data_flush_and_invd_range(req->buf, req->len));
}

static int kinetis_sdhc_complete_data_cache(const struct kinetis_sdhc_req *req, bool success)
{
	if (!req->use_dma || !success || req->is_write || (req->buf == NULL) || (req->len == 0U)) {
		return 0;
	}

	return kinetis_sdhc_cache_result(sys_cache_data_invd_range(req->buf, req->len));
}

static int kinetis_sdhc_prepare_adma_table(const struct device *dev, struct kinetis_sdhc_req *req)
{
	const struct kinetis_sdhc_config *cfg = dev->config;
	struct kinetis_sdhc_data *data = dev->data;
	const uint32_t *xfer_buf =
		(req->hal_data.rxData != NULL) ? req->hal_data.rxData : req->hal_data.txData;
	status_t status;

	status = SDHC_SetAdmaTableConfig(cfg->base, kSDHC_DmaModeAdma2, data->adma_table,
					 ARRAY_SIZE(data->adma_table), xfer_buf, req->len);
	if (status != kStatus_Success) {
		kinetis_sdhc_take_snapshot(cfg->base, &req->last_snapshot);
		return kinetis_sdhc_status_to_errno(status, &req->last_snapshot);
	}

	return kinetis_sdhc_cache_result(
		sys_cache_data_flush_range(data->adma_table, sizeof(data->adma_table)));
}

static status_t kinetis_sdhc_receive_response(SDHC_Type *base, sdhc_command_t *command)
{
	uint32_t response0 = base->CMDRSP[0];
	uint32_t response1 = base->CMDRSP[1];
	uint32_t response2 = base->CMDRSP[2];

	if (command->responseType != kCARD_ResponseTypeNone) {
		command->response[0U] = response0;

		if (command->responseType == kCARD_ResponseTypeR2) {
			command->response[0U] <<= 8U;
			command->response[1U] =
				(response1 << 8U) | ((response0 & 0xFF000000U) >> 24U);
			command->response[2U] =
				(response2 << 8U) | ((response1 & 0xFF000000U) >> 24U);
			command->response[3U] =
				(base->CMDRSP[3] << 8U) | ((response2 & 0xFF000000U) >> 24U);
		}
	}

	if ((command->responseErrorFlags != 0U) &&
	    ((command->responseType == kCARD_ResponseTypeR1) ||
	     (command->responseType == kCARD_ResponseTypeR1b) ||
	     (command->responseType == kCARD_ResponseTypeR6) ||
	     (command->responseType == kCARD_ResponseTypeR5))) {
		if ((command->responseErrorFlags & command->response[0U]) != 0U) {
			return kStatus_SDHC_SendCommandFailed;
		}
	}

	return kStatus_Success;
}

static uint32_t kinetis_sdhc_total_words(size_t bytes)
{
	return DIV_ROUND_UP(bytes, sizeof(uint32_t));
}

static uint32_t kinetis_sdhc_data_words_available(SDHC_Type *base, size_t bytes,
						  uint32_t transferred_words, bool read)
{
	uint32_t watermark = read ? ((base->WML & SDHC_WML_RDWML_MASK) >> SDHC_WML_RDWML_SHIFT)
				  : ((base->WML & SDHC_WML_WRWML_MASK) >> SDHC_WML_WRWML_SHIFT);
	uint32_t total_words = kinetis_sdhc_total_words(bytes);

	if (watermark >= total_words) {
		return total_words - transferred_words;
	}

	if ((total_words - transferred_words) >= watermark) {
		return watermark;
	}

	return total_words - transferred_words;
}

static void kinetis_sdhc_pio_transfer_data(SDHC_Type *base, struct kinetis_sdhc_req *req)
{
	uint32_t words;
	uint32_t i;

	words = kinetis_sdhc_data_words_available(base, req->len, req->transferred_words,
						  !req->is_write);

	for (i = 0U; i < words; i++) {
		size_t offset = (size_t)req->transferred_words * sizeof(uint32_t);
		size_t remaining = req->len - MIN(req->len, offset);
		size_t chunk = MIN(remaining, sizeof(uint32_t));

		if (req->is_write) {
			uint32_t word = 0U;

			if (chunk > 0U) {
				memcpy(&word, req->buf + offset, chunk);
			}
			SDHC_WriteData(base, word);
		} else {
			uint32_t word = SDHC_ReadData(base);

			if (chunk > 0U) {
				memcpy(req->buf + offset, &word, chunk);
			}
		}

		req->transferred_words++;
	}
}

static void kinetis_sdhc_set_transfer_config(SDHC_Type *base, const struct kinetis_sdhc_req *req)
{
	uint32_t flags = 0U;
	sdhc_transfer_config_t transfer_cfg = {0};

	switch (req->hal_cmd.responseType) {
	case kCARD_ResponseTypeR1:
	case kCARD_ResponseTypeR5:
	case kCARD_ResponseTypeR6:
	case kCARD_ResponseTypeR7:
		flags |= (uint32_t)kSDHC_ResponseLength48Flag | (uint32_t)kSDHC_EnableCrcCheckFlag |
			 (uint32_t)kSDHC_EnableIndexCheckFlag;
		break;
	case kCARD_ResponseTypeR1b:
	case kCARD_ResponseTypeR5b:
		flags |= (uint32_t)kSDHC_ResponseLength48BusyFlag |
			 (uint32_t)kSDHC_EnableCrcCheckFlag | (uint32_t)kSDHC_EnableIndexCheckFlag;
		break;
	case kCARD_ResponseTypeR2:
		flags |= (uint32_t)kSDHC_ResponseLength136Flag | (uint32_t)kSDHC_EnableCrcCheckFlag;
		break;
	case kCARD_ResponseTypeR3:
	case kCARD_ResponseTypeR4:
		flags |= (uint32_t)kSDHC_ResponseLength48Flag;
		break;
	default:
		break;
	}

	if (req->hal_cmd.type == kCARD_CommandTypeAbort) {
		flags |= (uint32_t)kSDHC_CommandTypeAbortFlag;
	}

	if (req->z_data != NULL) {
		flags |= (uint32_t)kSDHC_DataPresentFlag;

		if (req->use_dma) {
			flags |= (uint32_t)kSDHC_EnableDmaFlag;
		}

		if (!req->is_write) {
			flags |= (uint32_t)kSDHC_DataReadFlag;
		}

		if (req->hal_data.blockCount > 1U) {
			flags |= (uint32_t)kSDHC_MultipleBlockFlag |
				 (uint32_t)kSDHC_EnableBlockCountFlag;
			if (req->hal_data.enableAutoCommand12) {
				flags |= (uint32_t)kSDHC_EnableAutoCommand12Flag;
			}
		}

		transfer_cfg.dataBlockSize = req->hal_data.blockSize;
		transfer_cfg.dataBlockCount = req->hal_data.blockCount;
	}

	transfer_cfg.commandArgument = req->hal_cmd.argument;
	transfer_cfg.commandIndex = req->hal_cmd.index;
	transfer_cfg.flags = flags;

	SDHC_SetTransferConfig(base, &transfer_cfg);
}

static bool kinetis_sdhc_req_terminal(const struct kinetis_sdhc_req *req)
{
	if ((req->events & KINETIS_SDHC_REQ_FAILED) != 0U) {
		return true;
	}

	if (req->z_data == NULL) {
		return (req->events & KINETIS_SDHC_REQ_CMD_DONE) != 0U;
	}

	return (req->events & KINETIS_SDHC_REQ_DATA_DONE) != 0U;
}

static void kinetis_sdhc_req_finish(const struct device *dev, struct kinetis_sdhc_req *req, int ret,
				    uint32_t event_bits)
{
	const struct kinetis_sdhc_config *cfg = dev->config;

	if (kinetis_sdhc_req_terminal(req)) {
		return;
	}

	if (ret != 0) {
		kinetis_sdhc_req_record_snapshot(req, cfg->base);
		event_bits |= KINETIS_SDHC_REQ_FAILED;
	}

	req->result = ret;
	req->events |= event_bits;
	kinetis_sdhc_disable_transfer_irqs(cfg->base);
	k_sem_give(&((struct kinetis_sdhc_data *)dev->data)->transfer_sem);
}

static int kinetis_sdhc_prepare_request(struct kinetis_sdhc_req *req, struct sdhc_command *cmd,
					struct sdhc_data *data, bool use_dma)
{
	bool read;
	bool write;

	memset(req, 0, sizeof(*req));

	req->z_cmd = cmd;
	req->z_data = data;
	req->hal_cmd.index = cmd->opcode;
	req->hal_cmd.argument = cmd->arg;
	req->hal_cmd.type = (cmd->opcode == SD_STOP_TRANSMISSION) ? kCARD_CommandTypeAbort
								  : kCARD_CommandTypeNormal;
	req->hal_cmd.responseType = (cmd->response_type & SDHC_NATIVE_RESPONSE_MASK);
	req->hal_cmd.responseErrorFlags = kinetis_sdhc_response_error_flags(cmd->response_type);

	if (data == NULL) {
		return 0;
	}

	read = kinetis_sdhc_cmd_is_read(cmd);
	write = kinetis_sdhc_cmd_is_write(cmd);
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
	req->is_multiblock = kinetis_sdhc_is_multiblock_rw(cmd, data);
	req->use_auto_cmd12 = kinetis_sdhc_use_auto_cmd12(cmd, data);

	req->hal_data.enableAutoCommand12 = req->use_auto_cmd12;
	req->hal_data.enableIgnoreError = false;
	req->hal_data.blockSize = data->block_size;
	req->hal_data.blockCount = data->blocks;

	if (read) {
		req->hal_data.rxData = (uint32_t *)data->data;
	} else {
		req->hal_data.txData = (const uint32_t *)data->data;
	}

	return 0;
}

static int kinetis_sdhc_start_request(const struct device *dev, struct kinetis_sdhc_req *req)
{
	const struct kinetis_sdhc_config *cfg = dev->config;
	struct kinetis_sdhc_data *data = dev->data;
	sdhc_dma_mode_t dma_mode = req->use_dma ? kSDHC_DmaModeAdma2 : kSDHC_DmaModeNo;
	uint32_t prsstat = SDHC_GetPresentStatusFlags(cfg->base);
	uint32_t irq_mask = (uint32_t)kSDHC_CommandFlag;
	int ret;

	if ((prsstat & (uint32_t)kSDHC_CommandInhibitFlag) != 0U) {
		kinetis_sdhc_take_snapshot(cfg->base, &req->last_snapshot);
		return -EBUSY;
	}

	if ((req->z_data != NULL) && ((prsstat & (uint32_t)kSDHC_DataInhibitFlag) != 0U)) {
		kinetis_sdhc_take_snapshot(cfg->base, &req->last_snapshot);
		return -EBUSY;
	}

	ret = kinetis_sdhc_prepare_data_cache(req);
	if (ret != 0) {
		return ret;
	}

	if (req->use_dma) {
		ret = kinetis_sdhc_prepare_adma_table(dev, req);
		if (ret != 0) {
			return ret;
		}
	}

	if (req->z_data != NULL) {
		irq_mask |= req->use_dma ? (uint32_t)kSDHC_DataDMAFlag : (uint32_t)kSDHC_DataFlag;
	}

	req->events = 0U;
	req->result = 0;
	req->transferred_words = 0U;

	k_sem_reset(&data->transfer_sem);
	data->active_req = req;

	kinetis_sdhc_set_dma_mode(cfg->base, dma_mode);
	SDHC_ClearInterruptStatusFlags(cfg->base, KINETIS_SDHC_TRANSFER_IRQ_FLAGS);
	SDHC_EnableInterruptSignal(cfg->base, irq_mask);
	kinetis_sdhc_set_transfer_config(cfg->base, req);

	return 0;
}

static int kinetis_sdhc_wait_for_request(const struct device *dev, struct kinetis_sdhc_req *req)
{
	const struct kinetis_sdhc_config *cfg = dev->config;

	while (!kinetis_sdhc_req_terminal(req)) {
		if (k_sem_take(&((struct kinetis_sdhc_data *)dev->data)->transfer_sem,
			       kinetis_sdhc_request_timeout(req)) != 0) {
			kinetis_sdhc_take_snapshot(cfg->base, &req->last_snapshot);
			return -ETIMEDOUT;
		}
	}

	return req->result;
}

static int kinetis_sdhc_execute_request(const struct device *dev, struct kinetis_sdhc_req *req)
{
	const struct kinetis_sdhc_config *cfg = dev->config;
	struct kinetis_sdhc_data *data = dev->data;
	int ret;

	ret = kinetis_sdhc_start_request(dev, req);
	if (ret != 0) {
		(void)kinetis_sdhc_complete_data_cache(req, false);
		return ret;
	}

	ret = kinetis_sdhc_wait_for_request(dev, req);
	kinetis_sdhc_disable_transfer_irqs(cfg->base);
	data->active_req = NULL;

	if (ret != 0) {
		(void)kinetis_sdhc_complete_data_cache(req, false);
		return ret;
	}

	ret = kinetis_sdhc_complete_data_cache(req, true);
	if (ret != 0) {
		return ret;
	}

	memcpy(req->z_cmd->response, req->hal_cmd.response, sizeof(req->z_cmd->response));

	if (req->z_data != NULL) {
		req->z_data->bytes_xfered = req->z_data->blocks * req->z_data->block_size;
	}

	return 0;
}

static int kinetis_sdhc_issue_stop(const struct device *dev, uint32_t timeout_ms)
{
	struct sdhc_command z_cmd = {
		.opcode = SD_STOP_TRANSMISSION,
		.arg = 0U,
		.response_type = SD_RSP_TYPE_R1b,
		.timeout_ms = timeout_ms,
	};
	struct kinetis_sdhc_req req;
	int ret;

	ret = kinetis_sdhc_prepare_request(&req, &z_cmd, NULL, false);
	if (ret != 0) {
		return ret;
	}

	return kinetis_sdhc_execute_request(dev, &req);
}

static int kinetis_sdhc_wait_card_ready(const struct device *dev, uint32_t timeout_ms)
{
	const struct kinetis_sdhc_config *cfg = dev->config;
	int64_t deadline = (timeout_ms == SDHC_TIMEOUT_FOREVER)
				   ? INT64_MAX
				   : (k_uptime_get() + (int64_t)timeout_ms);

	while ((SDHC_GetPresentStatusFlags(cfg->base) & (uint32_t)kSDHC_Data0LineLevelFlag) == 0U) {
		if ((timeout_ms != SDHC_TIMEOUT_FOREVER) && (k_uptime_get() >= deadline)) {
			return -ETIMEDOUT;
		}

		k_sleep(K_USEC(KINETIS_SDHC_RECOVERY_POLL_US));
	}

	return 0;
}

static int kinetis_sdhc_recover_request(const struct device *dev,
					const struct kinetis_sdhc_req *req)
{
	const struct kinetis_sdhc_config *cfg = dev->config;
	struct kinetis_sdhc_data *data = dev->data;
	const struct kinetis_sdhc_hw_snapshot *snapshot = kinetis_sdhc_primary_snapshot(req);
	uint32_t live_prsstat;
	uint32_t live_irqstat;
	bool need_cmd_reset;
	bool need_data_reset;
	bool need_stop;
	int ret = 0;

	kinetis_sdhc_disable_transfer_irqs(cfg->base);
	data->active_req = NULL;

	need_stop = (req->z_data != NULL) &&
		    (req->is_multiblock ||
		     ((snapshot->irqstat &
		       ((uint32_t)kSDHC_DataErrorFlag | (uint32_t)kSDHC_DmaErrorFlag |
			(uint32_t)kSDHC_AutoCommand12ErrorFlag)) != 0U) ||
		     ((snapshot->prsstat & ((uint32_t)kSDHC_DataInhibitFlag |
					    (uint32_t)kSDHC_DataLineActiveFlag)) != 0U));

	if (need_stop) {
		ret = kinetis_sdhc_issue_stop(
			dev, kinetis_sdhc_request_timeout_ms(req->z_cmd, req->z_data));
	}

	if ((ret == 0) && (req->z_data != NULL) && req->is_write) {
		ret = kinetis_sdhc_wait_card_ready(dev, KINETIS_SDHC_CARD_READY_TIMEOUT_MS);
	}

	live_prsstat = SDHC_GetPresentStatusFlags(cfg->base);
	live_irqstat = SDHC_GetInterruptStatusFlags(cfg->base);

	need_cmd_reset = ((live_irqstat & (uint32_t)kSDHC_CommandErrorFlag) != 0U) ||
			 ((live_prsstat & (uint32_t)kSDHC_CommandInhibitFlag) != 0U);
	need_data_reset =
		((live_irqstat & ((uint32_t)kSDHC_DataErrorFlag | (uint32_t)kSDHC_DmaErrorFlag)) !=
		 0U) ||
		((live_prsstat &
		  ((uint32_t)kSDHC_DataInhibitFlag | (uint32_t)kSDHC_DataLineActiveFlag)) != 0U);

	if (need_cmd_reset &&
	    !SDHC_Reset(cfg->base, (uint32_t)kSDHC_ResetCommand, KINETIS_SDHC_RESET_TIMEOUT)) {
		return kinetis_sdhc_reset_host(dev);
	}

	if (need_data_reset &&
	    !SDHC_Reset(cfg->base, (uint32_t)kSDHC_ResetData, KINETIS_SDHC_RESET_TIMEOUT)) {
		return kinetis_sdhc_reset_host(dev);
	}

	SDHC_ClearInterruptStatusFlags(cfg->base, KINETIS_SDHC_TRANSFER_IRQ_FLAGS);

	return ret;
}

static bool kinetis_sdhc_should_retry(int ret)
{
	return (ret == -ETIMEDOUT) || (ret == -EIO) || (ret == -EBUSY);
}

static void kinetis_sdhc_log_irq_errors(uint32_t irqstat)
{
	if ((irqstat & (uint32_t)kSDHC_CommandTimeoutFlag) != 0U) {
		LOG_INF("  CMD timeout");
	}
	if ((irqstat & (uint32_t)kSDHC_CommandCrcErrorFlag) != 0U) {
		LOG_INF("  CMD CRC error");
	}
	if ((irqstat & (uint32_t)kSDHC_CommandEndBitErrorFlag) != 0U) {
		LOG_INF("  CMD end-bit error");
	}
	if ((irqstat & (uint32_t)kSDHC_CommandIndexErrorFlag) != 0U) {
		LOG_INF("  CMD index error");
	}
	if ((irqstat & (uint32_t)kSDHC_DataTimeoutFlag) != 0U) {
		LOG_INF("  DATA timeout");
	}
	if ((irqstat & (uint32_t)kSDHC_DataCrcErrorFlag) != 0U) {
		LOG_INF("  DATA CRC error");
	}
	if ((irqstat & (uint32_t)kSDHC_DataEndBitErrorFlag) != 0U) {
		LOG_INF("  DATA end-bit error");
	}
	if ((irqstat & (uint32_t)kSDHC_AutoCommand12ErrorFlag) != 0U) {
		LOG_INF("  Auto CMD12 error");
	}
	if ((irqstat & (uint32_t)kSDHC_DmaErrorFlag) != 0U) {
		LOG_INF("  DMA error");
	}
}

static void kinetis_sdhc_log_ac12_error(uint32_t ac12err)
{
	if (ac12err == 0U) {
		return;
	}

	if ((ac12err & (uint32_t)kSDHC_AutoCommand12NotExecutedFlag) != 0U) {
		LOG_INF("  AC12: not executed");
	}
	if ((ac12err & (uint32_t)kSDHC_AutoCommand12TimeoutFlag) != 0U) {
		LOG_INF("  AC12: timeout");
	}
	if ((ac12err & (uint32_t)kSDHC_AutoCommand12EndBitErrorFlag) != 0U) {
		LOG_INF("  AC12: end-bit error");
	}
	if ((ac12err & (uint32_t)kSDHC_AutoCommand12CrcErrorFlag) != 0U) {
		LOG_INF("  AC12: CRC error");
	}
	if ((ac12err & (uint32_t)kSDHC_AutoCommand12IndexErrorFlag) != 0U) {
		LOG_INF("  AC12: index error");
	}
	if ((ac12err & (uint32_t)kSDHC_AutoCommand12NotIssuedFlag) != 0U) {
		LOG_INF("  AC12: not issued");
	}
}

static void kinetis_sdhc_log_adma_error(uint32_t admaes)
{
	uint32_t state;

	if (admaes == 0U) {
		return;
	}

	state = (admaes & SDHC_ADMAES_ADMAES_MASK) >> SDHC_ADMAES_ADMAES_SHIFT;

	switch (state) {
	case kSDHC_AdmaErrorStateStopDma:
		LOG_INF("  ADMA state: stop DMA");
		break;
	case kSDHC_AdmaErrorStateFetchDescriptor:
		LOG_INF("  ADMA state: fetch descriptor");
		break;
	case kSDHC_AdmaErrorStateChangeAddress:
		LOG_INF("  ADMA state: change address");
		break;
	case kSDHC_AdmaErrorStateTransferData:
		LOG_INF("  ADMA state: transfer data");
		break;
	default:
		LOG_INF("  ADMA state: unknown");
		break;
	}

	if ((admaes & (uint32_t)kSDHC_AdmaLenghMismatchFlag) != 0U) {
		LOG_INF("  ADMA: length mismatch");
	}
	if ((admaes & (uint32_t)kSDHC_AdmaDescriptorErrorFlag) != 0U) {
		LOG_INF("  ADMA: descriptor error");
	}
}

static bool kinetis_sdhc_should_fallback_to_pio(const struct kinetis_sdhc_req *req, int ret)
{
	const struct kinetis_sdhc_hw_snapshot *snapshot;

	if (!req->use_dma || (req->z_data == NULL)) {
		return false;
	}

	if (ret == -EINVAL) {
		return true;
	}

	snapshot = kinetis_sdhc_primary_snapshot(req);

	return (ret == -ETIMEDOUT) ||
	       ((snapshot->irqstat & ((uint32_t)kSDHC_DmaErrorFlag | (uint32_t)kSDHC_DataErrorFlag |
				      (uint32_t)kSDHC_AutoCommand12ErrorFlag)) != 0U);
}

static void kinetis_sdhc_log_failure(const struct sdhc_command *cmd,
				     const struct kinetis_sdhc_req *req, int ret)
{
	const struct kinetis_sdhc_hw_snapshot *snapshot = kinetis_sdhc_primary_snapshot(req);

	LOG_INF("cmd%u failed ret=%d mode=%s irqstat=0x%08x prsstat=0x%08x ac12err=0x%08x "
		"admaes=0x%08x adsaddr=0x%08x dsaddr=0x%08x events=0x%02x",
		cmd->opcode, ret, req->use_dma ? "dma" : "pio", snapshot->irqstat,
		snapshot->prsstat, snapshot->ac12err, snapshot->admaes, snapshot->adsaddr,
		snapshot->dsaddr, req->events);
	kinetis_sdhc_log_irq_errors(snapshot->irqstat);
	kinetis_sdhc_log_ac12_error(snapshot->ac12err);
	kinetis_sdhc_log_adma_error(snapshot->admaes);
}

static int kinetis_sdhc_go_idle(const struct device *dev)
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

static void kinetis_cd_gpio_cb(const struct device *port, struct gpio_callback *cb,
			       gpio_port_pins_t pins)
{
	ARG_UNUSED(port);
	ARG_UNUSED(pins);

	struct kinetis_sdhc_data *data = CONTAINER_OF(cb, struct kinetis_sdhc_data, cd_cb);
	const struct device *dev = data->dev;
	const struct kinetis_sdhc_config *cfg = dev->config;
	int value;
	bool present;

	if (((data->enabled_sources & (SDHC_INT_INSERTED | SDHC_INT_REMOVED)) == 0) ||
	    (data->card_cb == NULL)) {
		return;
	}

	value = gpio_pin_get_dt(&cfg->cd_gpio);
	if (value < 0) {
		return;
	}

	present = value > 0;
	if (present == data->cd_present) {
		return;
	}

	data->cd_present = present;
	data->card_cb(dev, present ? SDHC_INT_INSERTED : SDHC_INT_REMOVED, data->card_cb_user_data);

	(void)gpio_pin_interrupt_configure_dt(&cfg->cd_gpio, present ? GPIO_INT_EDGE_TO_INACTIVE
								     : GPIO_INT_EDGE_TO_ACTIVE);
}

static int kinetis_sdhc_reset(const struct device *dev)
{
	struct kinetis_sdhc_data *data = dev->data;
	int ret;

	k_mutex_lock(&data->lock, K_FOREVER);
	ret = kinetis_sdhc_reset_host(dev);
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
	ret = kinetis_sdhc_apply_io(dev, ios);
	k_mutex_unlock(&data->lock);

	return ret;
}

static int kinetis_sdhc_get_card_present(const struct device *dev)
{
	const struct kinetis_sdhc_config *cfg = dev->config;
	struct kinetis_sdhc_data *data = dev->data;

	if (cfg->cd_gpio.port != NULL) {
		int value = gpio_pin_get_dt(&cfg->cd_gpio);

		if (value < 0) {
			return value;
		}

		data->cd_present = value > 0;
		return data->cd_present;
	}

	data->cd_present =
		(SDHC_GetPresentStatusFlags(cfg->base) & (uint32_t)kSDHC_CardInsertedFlag) != 0U;
	return data->cd_present;
}

static int kinetis_sdhc_card_busy(const struct device *dev)
{
	const struct kinetis_sdhc_config *cfg = dev->config;
	uint32_t status = SDHC_GetPresentStatusFlags(cfg->base);

	return (status & (uint32_t)kSDHC_Data0LineLevelFlag) == 0U;
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
	bool prefer_dma = kinetis_sdhc_request_prefers_dma(cfg, cmd, data);
	bool force_pio = false;
	uint32_t retries_left = cmd->retries;
	int ret;

	k_mutex_lock(&dev_data->lock, K_FOREVER);
	LOG_DBG("cmd%u arg=0x%08x resp=0x%x data=%p", cmd->opcode, cmd->arg, cmd->response_type,
		data);

	if (cmd->opcode == SD_GO_IDLE_STATE) {
		ret = kinetis_sdhc_go_idle(dev);
		if (ret != 0) {
			k_mutex_unlock(&dev_data->lock);
			return ret;
		}
	}

	if (data != NULL) {
		data->bytes_xfered = 0U;
	}

	for (;;) {
		struct kinetis_sdhc_req req;
		bool prepared = false;

		ret = kinetis_sdhc_prepare_request(&req, cmd, data, prefer_dma && !force_pio);
		if (ret == 0) {
			prepared = true;
			ret = kinetis_sdhc_execute_request(dev, &req);
		}

		if (ret == 0) {
			break;
		}

		kinetis_sdhc_log_failure(cmd, &req, ret);

		if (prepared) {
			int recover_ret = kinetis_sdhc_recover_request(dev, &req);

			if (recover_ret != 0) {
				ret = recover_ret;
			}
		}

		if (kinetis_sdhc_should_fallback_to_pio(&req, ret) && !force_pio) {
			force_pio = true;
			LOG_DBG("retrying cmd%u in pio mode", cmd->opcode);
			continue;
		}

		if (!kinetis_sdhc_should_retry(ret) || (retries_left == 0U)) {
			break;
		}

		retries_left--;
		LOG_DBG("retrying cmd%u ret=%d retries_left=%u mode=%s", cmd->opcode, ret,
			retries_left, force_pio ? "pio" : "dma");
	}

	k_mutex_unlock(&dev_data->lock);
	return ret;
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
		int value = gpio_pin_get_dt(&cfg->cd_gpio);

		if (value < 0) {
			k_mutex_unlock(&data->lock);
			return value;
		}

		data->cd_present = value > 0;
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

static void kinetis_sdhc_handle_card_events(const struct device *dev, uint32_t flags)
{
	struct kinetis_sdhc_data *data = dev->data;
	const struct kinetis_sdhc_config *cfg = dev->config;

	if ((flags & (uint32_t)kSDHC_CardInsertionFlag) != 0U) {
		if (cfg->cd_gpio.port == NULL) {
			data->cd_present = true;
			kinetis_sdhc_arm_host_cd_interrupts(dev, true);

			if (((data->enabled_sources & SDHC_INT_INSERTED) != 0) &&
			    (data->card_cb != NULL)) {
				data->card_cb(dev, SDHC_INT_INSERTED, data->card_cb_user_data);
			}
		}
	}

	if ((flags & (uint32_t)kSDHC_CardRemovalFlag) != 0U) {
		if (cfg->cd_gpio.port == NULL) {
			data->cd_present = false;
			kinetis_sdhc_arm_host_cd_interrupts(dev, false);

			if (((data->enabled_sources & SDHC_INT_REMOVED) != 0) &&
			    (data->card_cb != NULL)) {
				data->card_cb(dev, SDHC_INT_REMOVED, data->card_cb_user_data);
			}
		}
	}
}

static void kinetis_sdhc_handle_sdio_interrupt(const struct device *dev)
{
	const struct kinetis_sdhc_config *cfg = dev->config;
	struct kinetis_sdhc_data *data = dev->data;

	if (data->card_cb != NULL) {
		data->card_cb(dev, SDHC_INT_SDIO, data->card_cb_user_data);
	}

	SDHC_DisableInterruptStatus(cfg->base, (uint32_t)kSDHC_CardInterruptFlag);
	SDHC_DisableInterruptSignal(cfg->base, (uint32_t)kSDHC_CardInterruptFlag);
}

static void kinetis_sdhc_handle_command_irq(const struct device *dev, uint32_t flags,
					    struct kinetis_sdhc_req *req)
{
	const struct kinetis_sdhc_config *cfg = dev->config;
	status_t status;

	if (((flags & (uint32_t)kSDHC_CommandFlag) == 0U) || kinetis_sdhc_req_terminal(req)) {
		return;
	}

	if ((flags & (uint32_t)kSDHC_CommandErrorFlag) != 0U) {
		kinetis_sdhc_req_record_snapshot(req, cfg->base);
		kinetis_sdhc_req_finish(
			dev, req, kinetis_sdhc_xfer_error(kinetis_sdhc_primary_snapshot(req)), 0U);
		return;
	}

	status = kinetis_sdhc_receive_response(cfg->base, &req->hal_cmd);
	SDHC_DisableInterruptSignal(cfg->base, (uint32_t)kSDHC_CommandFlag);

	if (status != kStatus_Success) {
		kinetis_sdhc_req_record_snapshot(req, cfg->base);
		kinetis_sdhc_req_finish(
			dev, req,
			kinetis_sdhc_status_to_errno(status, kinetis_sdhc_primary_snapshot(req)),
			0U);
		return;
	}

	if (req->z_data == NULL) {
		kinetis_sdhc_req_finish(dev, req, 0, KINETIS_SDHC_REQ_CMD_DONE);
	}
}

static void kinetis_sdhc_handle_data_irq(const struct device *dev, uint32_t flags,
					 struct kinetis_sdhc_req *req)
{
	const struct kinetis_sdhc_config *cfg = dev->config;

	if (((flags & ((uint32_t)kSDHC_DataFlag | (uint32_t)kSDHC_DataDMAFlag)) == 0U) ||
	    kinetis_sdhc_req_terminal(req)) {
		return;
	}

	if ((flags & ((uint32_t)kSDHC_DataErrorFlag | (uint32_t)kSDHC_DmaErrorFlag)) != 0U) {
		kinetis_sdhc_req_record_snapshot(req, cfg->base);
		kinetis_sdhc_req_finish(
			dev, req, kinetis_sdhc_xfer_error(kinetis_sdhc_primary_snapshot(req)), 0U);
		return;
	}

	if (!req->use_dma && ((flags & ((uint32_t)kSDHC_BufferWriteReadyFlag |
					(uint32_t)kSDHC_BufferReadReadyFlag)) != 0U)) {
		kinetis_sdhc_pio_transfer_data(cfg->base, req);
	}

	if ((flags & (uint32_t)kSDHC_DataCompleteFlag) != 0U) {
		kinetis_sdhc_req_finish(dev, req, 0, KINETIS_SDHC_REQ_DATA_DONE);
	}
}

static void kinetis_sdhc_isr(const struct device *dev)
{
	const struct kinetis_sdhc_config *cfg = dev->config;
	struct kinetis_sdhc_data *data = dev->data;
	struct kinetis_sdhc_req *req = data->active_req;
	uint32_t flags = SDHC_GetEnabledInterruptStatusFlags(cfg->base);

	if (flags == 0U) {
		return;
	}

	if ((flags & (uint32_t)kSDHC_CardDetectFlag) != 0U) {
		kinetis_sdhc_handle_card_events(dev, flags);
	}

	if ((flags & (uint32_t)kSDHC_CardInterruptFlag) != 0U) {
		kinetis_sdhc_handle_sdio_interrupt(dev);
	}

	if (req != NULL) {
		kinetis_sdhc_handle_command_irq(dev, flags, req);
		kinetis_sdhc_handle_data_irq(dev, flags, req);
	}

	SDHC_ClearInterruptStatusFlags(cfg->base, flags);
}

static int kinetis_sdhc_init(const struct device *dev)
{
	const struct kinetis_sdhc_config *cfg = dev->config;
	struct kinetis_sdhc_data *data = dev->data;
	sdhc_capability_t cap = {0};
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
	data->active_req = NULL;

	kinetis_sdhc_init_controller(dev);

	memset(&data->props, 0, sizeof(data->props));
	data->props.f_min = cfg->min_bus_freq;
	data->props.f_max = cfg->max_bus_freq;
	data->props.power_delay = cfg->power_delay_ms;
	data->props.bus_4_bit_support = (cfg->bus_width >= 4U);
	data->props.host_caps.vol_330_support = true;
	data->props.host_caps.bus_8_bit_support = (cfg->bus_width >= 8U);
	data->props.no_card_power_control = (cfg->pwr_gpio.port == NULL);

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

	NVIC_ClearPendingIRQ(cfg->irq_num);
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

#define KINETIS_SDHC_IRQ_CONFIG(inst)                                                              \
	static void kinetis_sdhc_irq_config_##inst(const struct device *dev)                       \
	{                                                                                          \
		ARG_UNUSED(dev);                                                                   \
		IRQ_CONNECT(DT_INST_IRQN(inst), DT_INST_IRQ(inst, priority), kinetis_sdhc_isr,     \
			    DEVICE_DT_INST_GET(inst), 0);                                          \
		irq_enable(DT_INST_IRQN(inst));                                                    \
	}

#define KINETIS_SDHC_INIT(inst)                                                                    \
	PINCTRL_DT_INST_DEFINE(inst);                                                              \
	KINETIS_SDHC_IRQ_CONFIG(inst);                                                             \
	static const struct kinetis_sdhc_config kinetis_sdhc_cfg_##inst = {                        \
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
		.irq_num = DT_INST_IRQN(inst),                                                     \
		.irq_config_func = kinetis_sdhc_irq_config_##inst,                                 \
	};                                                                                         \
	static struct kinetis_sdhc_data kinetis_sdhc_data_##inst;                                  \
	DEVICE_DT_INST_DEFINE(inst, kinetis_sdhc_init, NULL, &kinetis_sdhc_data_##inst,            \
			      &kinetis_sdhc_cfg_##inst, POST_KERNEL, CONFIG_SDHC_INIT_PRIORITY,    \
			      &kinetis_sdhc_api);

DT_INST_FOREACH_STATUS_OKAY(KINETIS_SDHC_INIT)
