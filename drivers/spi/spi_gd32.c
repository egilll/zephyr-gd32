/*
 * Copyright (c) 2021 BrainCo Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT gd_gd32_spi

#include <errno.h>
#include <zephyr/kernel.h>
#include <zephyr/drivers/clock_control.h>
#include <zephyr/drivers/clock_control/gd32.h>
#include <zephyr/drivers/pinctrl.h>
#include <zephyr/drivers/reset.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/drivers/spi/rtio.h>
#include <zephyr/sys/util.h>
#include <zephyr/sys/atomic.h>
#ifdef CONFIG_SPI_GD32_DMA
#include <zephyr/drivers/dma.h>
#include <zephyr/drivers/dma/dma_gd32.h>
#endif

#include <gd32_spi.h>

#include <zephyr/logging/log.h>
#include <zephyr/irq.h>
LOG_MODULE_REGISTER(spi_gd32, CONFIG_SPI_LOG_LEVEL);

#include "spi_context.h"

/* SPI error status mask. */
#define SPI_GD32_ERR_MASK                                                                          \
	(SPI_STAT_RXORERR | SPI_STAT_CONFERR | SPI_STAT_CRCERR | SPI_STAT_FERR | SPI_STAT_TXURERR)

#define GD32_SPI_PSC_MAX 0x7U

enum spi_gd32_xfer_state {
	SPI_GD32_XFER_IDLE = 0,
	SPI_GD32_XFER_ACTIVE,
	SPI_GD32_XFER_FINISHING,
	SPI_GD32_XFER_ABORTED,
};

#ifdef CONFIG_SPI_GD32_DMA

enum spi_gd32_dma_direction {
	RX = 0,
	TX,
	NUM_OF_DIRECTION
};

struct spi_gd32_dma_config {
	const struct device *dev;
	uint32_t channel;
	uint32_t config;
	uint32_t slot;
	uint32_t fifo_threshold;
};

struct spi_gd32_dma_data {
	struct dma_config config;
	struct dma_block_config block;
	uint32_t count;
};

#endif

struct spi_gd32_config {
	uint32_t reg;
	uint16_t clkid;
	struct reset_dt_spec reset;
	const struct pinctrl_dev_config *pcfg;
#ifdef CONFIG_SPI_GD32_DMA
	const struct spi_gd32_dma_config dma[NUM_OF_DIRECTION];
#endif
#ifdef CONFIG_SPI_GD32_INTERRUPT
	void (*irq_configure)(const struct device *dev);
#endif
};

struct spi_gd32_data {
	struct spi_context ctx;
#ifdef CONFIG_SPI_GD32_DMA
	struct spi_gd32_dma_data dma[NUM_OF_DIRECTION];
#endif
	const struct device *dev;
	struct k_work_delayable finish_work;
	struct k_work_delayable timeout_work;
	atomic_t state;
	bool busy;
	bool use_dma;
	bool hw_rx_expected;
	uint8_t dfs;
	size_t pending_rx;
	int completion_status;
	uint8_t finish_attempts;
	uint32_t effective_hz;
	uint32_t last_timeout_ms;
#ifdef CONFIG_SPI_GD32_DMA
	atomic_t dma_done_mask;
	size_t dma_chunk_frames;
#endif
};

#ifdef CONFIG_SPI_GD32_DMA

static uint32_t dummy_tx;
static uint32_t dummy_rx;

static bool spi_gd32_dma_enabled(const struct device *dev)
{
	const struct spi_gd32_config *cfg = dev->config;

	if (cfg->dma[TX].dev && cfg->dma[RX].dev) {
		return true;
	}

	return false;
}

static size_t spi_gd32_dma_enabled_num(const struct device *dev)
{
	return spi_gd32_dma_enabled(dev) ? 2 : 0;
}

#endif

static bool spi_gd32_transfer_ongoing(struct spi_gd32_data *data)
{
	return spi_context_tx_on(&data->ctx) || spi_context_rx_on(&data->ctx);
}

static int spi_gd32_get_dfs(const struct spi_config *config, uint8_t *dfs)
{
	uint32_t bits = SPI_WORD_SIZE_GET(config->operation);

	if (bits == 8U) {
		*dfs = 1U;
		return 0;
	}

	if (bits == 16U) {
		*dfs = 2U;
		return 0;
	}

	LOG_ERR("Unsupported word size: %u", bits);
	return -ENOTSUP;
}

static void spi_gd32_flush_rx(uint32_t reg)
{
	/* RX buffer is 1-deep; cap reads to avoid any pathological infinite loop. */
	for (int i = 0; i < 32 && ((SPI_STAT(reg) & SPI_STAT_RBNE) != 0U); i++) {
		/*
		 * The RXORERR clear sequence is SPI_DATA read followed by SPI_STAT read.
		 * Reading SPI_STAT after draining ensures we don't leave RXORERR latched.
		 */
		(void)SPI_DATA(reg);
		(void)SPI_STAT(reg);
	}
}

static void spi_gd32_clear_errors(uint32_t reg)
{
	uint32_t stat = SPI_STAT(reg);

	/*
	 * Clear methods (see GD32 SPI/I2S reference):
	 * - RXORERR: read SPI_DATA then read SPI_STAT
	 * - CONFERR: read/write SPI_STAT then write SPI_CTL0
	 * - CRCERR: write 0 to CRCERR bit
	 * - FERR: write 0 to FERR bit
	 * - TXURERR: read SPI_STAT
	 */
	if ((stat & SPI_STAT_RXORERR) != 0U) {
		(void)SPI_DATA(reg);
		(void)SPI_STAT(reg);
	}

	if ((stat & (SPI_STAT_CRCERR | SPI_STAT_FERR)) != 0U) {
		SPI_STAT(reg) &= ~(SPI_STAT_CRCERR | SPI_STAT_FERR);
	}

	if ((stat & SPI_STAT_CONFERR) != 0U) {
		uint32_t ctl0 = SPI_CTL0(reg);

		(void)SPI_STAT(reg);
		SPI_CTL0(reg) = ctl0;
	}

	if ((stat & SPI_STAT_TXURERR) != 0U) {
		(void)SPI_STAT(reg);
	}
}

static void spi_gd32_disable_irqs_and_dma(uint32_t reg)
{
	SPI_CTL1(reg) &= ~(SPI_CTL1_RBNEIE | SPI_CTL1_TBEIE | SPI_CTL1_ERRIE | SPI_CTL1_DMATEN |
			   SPI_CTL1_DMAREN);
}

static int spi_gd32_configure(const struct device *dev, const struct spi_config *config)
{
	struct spi_gd32_data *data = dev->data;
	const struct spi_gd32_config *cfg = dev->config;
	uint32_t bus_freq;
	uint32_t ctl0;
	uint32_t ctl1;
	uint32_t bits;
	uint32_t target_hz;
	uint8_t psc_idx;
	uint32_t actual_hz;
	int ret;

	if (spi_context_configured(&data->ctx, config)) {
		return 0;
	}

	if (SPI_OP_MODE_GET(config->operation) == SPI_OP_MODE_SLAVE) {
		LOG_ERR("Slave mode not supported");
		return -ENOTSUP;
	}

	if ((config->operation & SPI_MODE_LOOP) != 0U) {
		LOG_ERR("Loopback not supported");
		return -ENOTSUP;
	}

	if (IS_ENABLED(CONFIG_SPI_EXTENDED_MODES) &&
	    ((config->operation & SPI_LINES_MASK) != SPI_LINES_SINGLE)) {
		LOG_ERR("Only single SPI line mode supported");
		return -ENOTSUP;
	}

	if ((config->operation & SPI_CS_ACTIVE_HIGH) != 0U && !spi_cs_is_gpio(config)) {
		LOG_ERR("Active-high CS only supported with GPIO CS");
		return -ENOTSUP;
	}

	if ((config->operation & SPI_HOLD_ON_CS) != 0U && !spi_cs_is_gpio(config)) {
		LOG_ERR("SPI_HOLD_ON_CS only supported with GPIO CS");
		return -ENOTSUP;
	}

	if ((config->operation & SPI_FRAME_FORMAT_TI) != 0U && spi_cs_is_gpio(config)) {
		LOG_ERR("TI frame format requires hardware NSS");
		return -ENOTSUP;
	}

	bits = SPI_WORD_SIZE_GET(config->operation);
	if ((bits != 8U) && (bits != 16U)) {
		LOG_ERR("Unsupported word size: %u", bits);
		return -ENOTSUP;
	}

	if (((config->operation & SPI_FRAME_FORMAT_TI) != 0U) &&
	    ((config->operation & SPI_TRANSFER_LSB) != 0U)) {
		LOG_ERR("TI mode only supports MSB-first");
		return -ENOTSUP;
	}

	spi_gd32_disable_irqs_and_dma(cfg->reg);
	SPI_CTL0(cfg->reg) &= ~SPI_CTL0_SPIEN;
	/* Ensure SPI mode, not I2S. */
	SPI_I2SCTL(cfg->reg) &= ~SPI_I2SCTL_I2SSEL;

	ret = clock_control_get_rate(GD32_CLOCK_CONTROLLER, (clock_control_subsys_t)&cfg->clkid,
				     &bus_freq);
	if (ret < 0) {
		return ret;
	}

	target_hz = config->frequency;
	if (target_hz == 0U) {
		target_hz = UINT32_MAX;
	}

	ctl0 = SPI_CTL0(cfg->reg);
	ctl0 &= ~(SPI_CTL0_CKPH | SPI_CTL0_CKPL | SPI_CTL0_MSTMOD | SPI_CTL0_PSC | SPI_CTL0_LF |
		  SPI_CTL0_SWNSS | SPI_CTL0_SWNSSEN | SPI_CTL0_RO | SPI_CTL0_FF16 | SPI_CTL0_CRCNT |
		  SPI_CTL0_CRCEN | SPI_CTL0_BDOEN | SPI_CTL0_BDEN);
	ctl0 |= (SPI_CTL0_MSTMOD | SPI_CTL0_SWNSS);

	if (bits == 16U) {
		ctl0 |= SPI_CTL0_FF16;
	}

	if ((config->operation & SPI_TRANSFER_LSB) != 0U) {
		ctl0 |= SPI_CTL0_LF;
	}

	if ((config->operation & SPI_MODE_CPOL) != 0U) {
		ctl0 |= SPI_CTL0_CKPL;
	}

	if ((config->operation & SPI_MODE_CPHA) != 0U) {
		ctl0 |= SPI_CTL0_CKPH;
	}

	if (spi_cs_is_gpio(config)) {
		ctl0 |= SPI_CTL0_SWNSSEN;
	}

	/* Prescaler produces bus_freq / 2^(PSC+1). */
	ctl0 &= ~SPI_CTL0_PSC;
	psc_idx = GD32_SPI_PSC_MAX;
	actual_hz = bus_freq / (2U << psc_idx);
	for (uint8_t i = 0U; i <= GD32_SPI_PSC_MAX; i++) {
		uint32_t hz = bus_freq / (2U << i);

		if (hz <= target_hz) {
			psc_idx = i;
			actual_hz = hz;
			break;
		}
	}
	ctl0 |= CTL0_PSC(psc_idx);

	SPI_CTL0(cfg->reg) = ctl0;

	ctl1 = SPI_CTL1(cfg->reg);
	ctl1 &= ~(SPI_CTL1_TMOD | SPI_CTL1_NSSDRV);

	if (!spi_cs_is_gpio(config)) {
		ctl1 |= SPI_CTL1_NSSDRV;
	}

	if ((config->operation & SPI_FRAME_FORMAT_TI) != 0U) {
		ctl1 |= SPI_CTL1_TMOD;
	}

	SPI_CTL1(cfg->reg) = ctl1;

	data->ctx.config = config;
	data->effective_hz = actual_hz;

	return 0;
}

static void spi_gd32_prepare_transfer(const struct device *dev)
{
	const struct spi_gd32_config *cfg = dev->config;

	spi_gd32_clear_errors(cfg->reg);
	spi_gd32_flush_rx(cfg->reg);
}

static bool spi_gd32_hw_rx_expected(uint32_t reg)
{
	uint32_t ctl0 = SPI_CTL0(reg);

	/* Receive-only mode always expects RX. */
	if ((ctl0 & SPI_CTL0_RO) != 0U) {
		return true;
	}

	/*
	 * Bidirectional mode (BDEN=1): BDOEN selects output (transmit-only) vs input (receive).
	 * In transmit-only bidirectional mode, the peripheral does not provide meaningful RX data.
	 */
	if ((ctl0 & SPI_CTL0_BDEN) != 0U) {
		return (ctl0 & SPI_CTL0_BDOEN) == 0U;
	}

	/* 2-line full-duplex always shifts RX while transmitting. */
	return true;
}

#ifdef CONFIG_SPI_GD32_DMA
static void spi_gd32_dma_callback(const struct device *dma_dev, void *arg, uint32_t channel,
				  int status);

static int spi_gd32_dma_setup(const struct device *dev, uint32_t dir, size_t frames)
{
	const struct spi_gd32_config *cfg = dev->config;
	struct spi_gd32_data *data = dev->data;
	struct dma_config *dma_cfg = &data->dma[dir].config;
	struct dma_block_config *block_cfg = &data->dma[dir].block;
	const struct spi_gd32_dma_config *dma = &cfg->dma[dir];
	int ret;

	if (frames == 0U) {
		return 0;
	}

	memset(dma_cfg, 0, sizeof(struct dma_config));
	memset(block_cfg, 0, sizeof(struct dma_block_config));

	dma_cfg->source_burst_length = 1;
	dma_cfg->dest_burst_length = 1;
	dma_cfg->user_data = (void *)dev;
	dma_cfg->dma_callback = spi_gd32_dma_callback;
	dma_cfg->block_count = 1U;
	dma_cfg->head_block = block_cfg;
	dma_cfg->dma_slot = cfg->dma[dir].slot;
	dma_cfg->channel_priority = GD32_DMA_CONFIG_PRIORITY(cfg->dma[dir].config);
	dma_cfg->channel_direction = dir == TX ? MEMORY_TO_PERIPHERAL : PERIPHERAL_TO_MEMORY;

	dma_cfg->source_data_size = data->dfs;
	dma_cfg->dest_data_size = data->dfs;

	block_cfg->block_size = frames * (size_t)data->dfs;

	if (dir == TX) {
		block_cfg->dest_address = (uint32_t)&SPI_DATA(cfg->reg);
		block_cfg->dest_addr_adj = DMA_ADDR_ADJ_NO_CHANGE;
		if (spi_context_tx_buf_on(&data->ctx)) {
			block_cfg->source_address = (uint32_t)data->ctx.tx_buf;
			block_cfg->source_addr_adj = DMA_ADDR_ADJ_INCREMENT;
		} else {
			block_cfg->source_address = (uint32_t)&dummy_tx;
			block_cfg->source_addr_adj = DMA_ADDR_ADJ_NO_CHANGE;
		}
	}

	if (dir == RX) {
		block_cfg->source_address = (uint32_t)&SPI_DATA(cfg->reg);
		block_cfg->source_addr_adj = DMA_ADDR_ADJ_NO_CHANGE;

		if (spi_context_rx_buf_on(&data->ctx)) {
			block_cfg->dest_address = (uint32_t)data->ctx.rx_buf;
			block_cfg->dest_addr_adj = DMA_ADDR_ADJ_INCREMENT;
		} else {
			block_cfg->dest_address = (uint32_t)&dummy_rx;
			block_cfg->dest_addr_adj = DMA_ADDR_ADJ_NO_CHANGE;
		}
	}

	ret = dma_config(dma->dev, dma->channel, dma_cfg);
	if (ret < 0) {
		LOG_ERR("dma_config %p failed %d\n", dma->dev, ret);
		return ret;
	}

	ret = dma_start(dma->dev, dma->channel);
	if (ret < 0) {
		LOG_ERR("dma_start %p failed %d\n", dma->dev, ret);
		return ret;
	}

	return 0;
}

static int spi_gd32_start_dma_transceive(const struct device *dev)
{
	const struct spi_gd32_config *cfg = dev->config;
	struct spi_gd32_data *data = dev->data;
	size_t frames = spi_context_max_continuous_chunk(&data->ctx);
	int ret;

	data->dma_chunk_frames = frames;
	atomic_set(&data->dma_done_mask, 0);

	for (size_t i = 0; i < spi_gd32_dma_enabled_num(dev); i++) {
		dma_stop(cfg->dma[i].dev, cfg->dma[i].channel);
	}

	/* Start RX first to avoid overruns. */
	ret = spi_gd32_dma_setup(dev, RX, frames);
	if (ret < 0) {
		goto on_error;
	}

	ret = spi_gd32_dma_setup(dev, TX, frames);
	if (ret < 0) {
		goto on_error;
	}

	SPI_CTL1(cfg->reg) |= (SPI_CTL1_DMATEN | SPI_CTL1_DMAREN);
	return 0;

on_error:
	for (size_t i = 0; i < spi_gd32_dma_enabled_num(dev); i++) {
		dma_stop(cfg->dma[i].dev, cfg->dma[i].channel);
	}
	return ret;
}
#endif

static k_timeout_t spi_gd32_finish_delay(const struct spi_config *config)
{
	uint32_t bits = SPI_WORD_SIZE_GET(config->operation);
	uint32_t hz = config->frequency;
	uint64_t us;

	if ((bits == 0U) || (hz == 0U)) {
		return K_MSEC(1);
	}

	us = DIV_ROUND_UP((uint64_t)bits * 1000000ULL, (uint64_t)hz);
	us = CLAMP(us, 1ULL, 1000000ULL);

	return K_USEC((uint32_t)us);
}

static void spi_gd32_isr(const struct device *dev);

static void spi_gd32_prime_irq_xfer(const struct device *dev)
{
	const struct spi_gd32_config *cfg = dev->config;
	struct spi_gd32_data *data = dev->data;
	struct spi_context *ctx = &data->ctx;
	uint32_t stat;
	uint16_t tx = 0U;

	if (data->use_dma || atomic_get(&data->state) != SPI_GD32_XFER_ACTIVE) {
		return;
	}

	if (!spi_gd32_transfer_ongoing(data)) {
		return;
	}

	stat = SPI_STAT(cfg->reg);
	if ((stat & SPI_STAT_TBE) == 0U) {
		return;
	}

	if (spi_context_tx_buf_on(ctx)) {
		if (data->dfs == 1U) {
			tx = UNALIGNED_GET((uint8_t *)ctx->tx_buf);
		} else {
			tx = UNALIGNED_GET((uint16_t *)ctx->tx_buf);
		}
	}

	SPI_DATA(cfg->reg) = tx;
	spi_context_update_tx(ctx, data->dfs, 1);
	if (data->hw_rx_expected) {
		data->pending_rx++;
	}
}

static uint32_t spi_gd32_xfer_timeout_ms(const struct spi_context *ctx, uint32_t effective_hz)
{
	size_t tx_len = spi_context_total_tx_len((struct spi_context *)ctx);
	size_t rx_len = spi_context_total_rx_len((struct spi_context *)ctx);
	uint32_t hz = effective_hz != 0U ? effective_hz : ctx->config->frequency;
	uint64_t bits_ms;
	uint32_t timeout_ms;

	if (hz == 0U) {
		return 1000U;
	}

	bits_ms = (uint64_t)MAX(tx_len, rx_len) * 8ULL * 1000ULL;
	timeout_ms = (uint32_t)DIV_ROUND_UP(bits_ms, (uint64_t)hz);
	timeout_ms += CONFIG_SPI_COMPLETION_TIMEOUT_TOLERANCE;

	if (timeout_ms == 0U) {
		timeout_ms = 1U;
	}

	return timeout_ms;
}

static void spi_gd32_request_finish(const struct device *dev, int status);

static void spi_gd32_log_errors(const struct device *dev, const char *where, uint32_t stat)
{
	if ((stat & SPI_GD32_ERR_MASK) == 0U) {
		return;
	}

	LOG_INF("%s: %s: SPI error stat=0x%08x (rxor=%u conf=%u crc=%u ferr=%u txur=%u)", dev->name,
		where, stat, (stat & SPI_STAT_RXORERR) != 0U, (stat & SPI_STAT_CONFERR) != 0U,
		(stat & SPI_STAT_CRCERR) != 0U, (stat & SPI_STAT_FERR) != 0U,
		(stat & SPI_STAT_TXURERR) != 0U);
}

static void spi_gd32_timeout_work_handler(struct k_work *work)
{
	struct spi_gd32_data *data = CONTAINER_OF(work, struct spi_gd32_data, timeout_work.work);
	const struct spi_gd32_config *cfg = data->dev->config;

	if (atomic_get(&data->state) != SPI_GD32_XFER_ACTIVE) {
		return;
	}

	LOG_WRN("%s: xfer timeout after %u ms (eff_hz=%u, req_hz=%u, tx=%zu, rx=%zu, stat=0x%08x)",
		data->dev->name, data->last_timeout_ms, data->effective_hz,
		data->ctx.config != NULL ? data->ctx.config->frequency : 0U,
		spi_context_total_tx_len(&data->ctx), spi_context_total_rx_len(&data->ctx),
		SPI_STAT(cfg->reg));
	spi_gd32_request_finish(data->dev, -ETIMEDOUT);
}

static void spi_gd32_request_finish(const struct device *dev, int status)
{
	const struct spi_gd32_config *cfg = dev->config;
	struct spi_gd32_data *data = dev->data;
	uint32_t stat = SPI_STAT(cfg->reg);

	if (!atomic_cas(&data->state, SPI_GD32_XFER_ACTIVE, SPI_GD32_XFER_FINISHING)) {
		return;
	}

	if (status != 0) {
		spi_gd32_log_errors(dev, "finish request", stat);
		LOG_DBG("%s: finish request status=%d (use_dma=%u, pending_rx=%u, tx_on=%u, "
			"rx_on=%u, stat=0x%08x)",
			dev->name, status, data->use_dma, data->pending_rx,
			spi_context_tx_on(&data->ctx), spi_context_rx_on(&data->ctx), stat);
	}

	data->completion_status = status;
	data->finish_attempts = 0U;

	spi_gd32_disable_irqs_and_dma(cfg->reg);

	(void)k_work_schedule(&data->finish_work, K_NO_WAIT);
}

static void spi_gd32_finish_work_handler(struct k_work *work)
{
	struct spi_gd32_data *data = CONTAINER_OF(work, struct spi_gd32_data, finish_work.work);
	const struct device *dev = data->dev;
	const struct spi_gd32_config *cfg = dev->config;
	bool force_cs_inactive = (data->completion_status != 0);

	if (atomic_get(&data->state) != SPI_GD32_XFER_FINISHING) {
		return;
	}

	spi_gd32_prepare_transfer(dev);

	if ((SPI_STAT(cfg->reg) & SPI_STAT_TRANS) != 0U && data->finish_attempts++ < 10U) {
		(void)k_work_schedule(&data->finish_work, spi_gd32_finish_delay(data->ctx.config));
		return;
	}

	if ((SPI_STAT(cfg->reg) & SPI_STAT_TRANS) != 0U) {
		LOG_INF("%s: SPI transfer stuck (TRANS=1); resetting peripheral", dev->name);
		SPI_CTL0(cfg->reg) &= ~SPI_CTL0_SPIEN;
		(void)reset_line_toggle_dt(&cfg->reset);
		spi_gd32_prepare_transfer(dev);
	}

#ifdef CONFIG_SPI_GD32_DMA
	for (size_t i = 0; i < spi_gd32_dma_enabled_num(dev); i++) {
		dma_stop(cfg->dma[i].dev, cfg->dma[i].channel);
	}
#endif

	if (force_cs_inactive) {
#ifndef DT_SPI_CTX_HAS_NO_CS_GPIOS
		_spi_context_cs_control(&data->ctx, false, true);
#endif
	} else {
		spi_context_cs_control(&data->ctx, false);
	}
	SPI_CTL0(cfg->reg) &= ~SPI_CTL0_SPIEN;
	spi_gd32_prepare_transfer(dev);

	data->busy = false;
	data->use_dma = false;
	data->pending_rx = 0U;
	atomic_set(&data->state, SPI_GD32_XFER_IDLE);

	spi_context_complete(&data->ctx, dev, data->completion_status);
}

static void spi_gd32_abort(const struct device *dev)
{
	const struct spi_gd32_config *cfg = dev->config;
	struct spi_gd32_data *data = dev->data;
	uint32_t stat = SPI_STAT(cfg->reg);

	spi_gd32_log_errors(dev, "abort", stat);
	LOG_DBG("%s: abort (use_dma=%u, pending_rx=%u, tx_on=%u, rx_on=%u, stat=0x%08x)", dev->name,
		data->use_dma, data->pending_rx, spi_context_tx_on(&data->ctx),
		spi_context_rx_on(&data->ctx), stat);

	atomic_set(&data->state, SPI_GD32_XFER_ABORTED);
	(void)k_work_cancel_delayable(&data->finish_work);
	(void)k_work_cancel_delayable(&data->timeout_work);

	spi_gd32_disable_irqs_and_dma(cfg->reg);

#ifdef CONFIG_SPI_GD32_DMA
	for (size_t i = 0; i < spi_gd32_dma_enabled_num(dev); i++) {
		dma_stop(cfg->dma[i].dev, cfg->dma[i].channel);
	}
#endif

#ifndef DT_SPI_CTX_HAS_NO_CS_GPIOS
	_spi_context_cs_control(&data->ctx, false, true);
#endif
	SPI_CTL0(cfg->reg) &= ~SPI_CTL0_SPIEN;
	spi_gd32_prepare_transfer(dev);

	data->busy = false;
	data->use_dma = false;
	data->pending_rx = 0U;
	atomic_set(&data->state, SPI_GD32_XFER_IDLE);
}

static int spi_gd32_apply_duplex_mode(const struct device *dev)
{
	struct spi_gd32_data *data = dev->data;
	const struct spi_gd32_config *cfg = dev->config;
	struct spi_context *ctx = &data->ctx;
	uint32_t ctl0 = SPI_CTL0(cfg->reg);

	ctl0 &= ~(SPI_CTL0_BDEN | SPI_CTL0_BDOEN);

	if ((ctx->config->operation & SPI_HALF_DUPLEX) != 0U) {
		bool has_tx = spi_context_total_tx_len(ctx) > 0U;
		bool has_rx = spi_context_total_rx_len(ctx) > 0U;

		if (has_tx && has_rx) {
			LOG_ERR("Half-duplex transceive requires TX-only or RX-only");
			return -ENOTSUP;
		}

		ctl0 |= SPI_CTL0_BDEN;
		if (has_tx) {
			ctl0 |= SPI_CTL0_BDOEN;
		}
	}

	SPI_CTL0(cfg->reg) = ctl0;
	return 0;
}

static int spi_gd32_transceive_impl(const struct device *dev, const struct spi_config *config,
				    const struct spi_buf_set *tx_bufs,
				    const struct spi_buf_set *rx_bufs, spi_callback_t cb,
				    void *userdata)
{
	struct spi_gd32_data *data = dev->data;
	const struct spi_gd32_config *cfg = dev->config;
	bool asynchronous = (cb != NULL);
	uint8_t dfs;
	int ret;

	spi_context_lock(&data->ctx, asynchronous, cb, userdata, config);

	ret = spi_gd32_configure(dev, config);
	if (ret < 0) {
		goto out;
	}

	ret = spi_gd32_get_dfs(config, &dfs);
	if (ret < 0) {
		goto out;
	}

	data->dfs = dfs;
	spi_gd32_prepare_transfer(dev);
	spi_context_buffers_setup(&data->ctx, tx_bufs, rx_bufs, dfs);

	ret = spi_gd32_apply_duplex_mode(dev);
	if (ret < 0) {
		goto out;
	}

	data->pending_rx = 0U;
	data->completion_status = 0;
	data->finish_attempts = 0U;
	data->busy = true;
#ifdef CONFIG_SPI_GD32_DMA
	data->use_dma = spi_gd32_dma_enabled(dev);
#else
	data->use_dma = false;
#endif
	if (!spi_gd32_transfer_ongoing(data)) {
		data->use_dma = false;
	}

	data->hw_rx_expected = spi_gd32_hw_rx_expected(cfg->reg);
	if (!data->hw_rx_expected) {
		/* DMA relies on RX DMA requests; disable DMA for TX-only bidirectional mode. */
		data->use_dma = false;
	}

	atomic_set(&data->state, SPI_GD32_XFER_ACTIVE);

	spi_context_cs_control(&data->ctx, true);
	SPI_CTL0(cfg->reg) |= SPI_CTL0_SPIEN;
	spi_gd32_prepare_transfer(dev);

	if (data->use_dma) {
#ifdef CONFIG_SPI_GD32_DMA
		ret = spi_gd32_start_dma_transceive(dev);
		if (ret < 0) {
			spi_gd32_abort(dev);
			goto out;
		}
#endif
		SPI_CTL1(cfg->reg) |= SPI_CTL1_ERRIE;
	} else {
		unsigned int key = irq_lock();

		SPI_CTL1(cfg->reg) |= (SPI_CTL1_RBNEIE | SPI_CTL1_TBEIE | SPI_CTL1_ERRIE);
		/*
		 * Some parts only generate the TBE interrupt on a 0->1 transition; prime the first
		 * frame so the peripheral can generate a subsequent TBE edge.
		 */
		spi_gd32_prime_irq_xfer(dev);
		irq_unlock(key);
	}

	data->last_timeout_ms = spi_gd32_xfer_timeout_ms(&data->ctx, data->effective_hz);
	(void)k_work_schedule(&data->timeout_work, K_MSEC(data->last_timeout_ms));

	if (!spi_gd32_transfer_ongoing(data)) {
		spi_gd32_request_finish(dev, 0);
	}

	ret = spi_context_wait_for_completion(&data->ctx);

	if (ret == -ETIMEDOUT) {
		spi_gd32_abort(dev);
	}

out:
	spi_context_release(&data->ctx, ret);

	return ret;
}

static int spi_gd32_transceive(const struct device *dev, const struct spi_config *config,
			       const struct spi_buf_set *tx_bufs, const struct spi_buf_set *rx_bufs)
{
	return spi_gd32_transceive_impl(dev, config, tx_bufs, rx_bufs, NULL, NULL);
}

#ifdef CONFIG_SPI_ASYNC
static int spi_gd32_transceive_async(const struct device *dev, const struct spi_config *config,
				     const struct spi_buf_set *tx_bufs,
				     const struct spi_buf_set *rx_bufs, spi_callback_t cb,
				     void *userdata)
{
	return spi_gd32_transceive_impl(dev, config, tx_bufs, rx_bufs, cb, userdata);
}
#endif

#ifdef CONFIG_SPI_GD32_INTERRUPT

static void spi_gd32_update_tbe_irq(const struct device *dev)
{
	const struct spi_gd32_config *cfg = dev->config;
	struct spi_gd32_data *data = dev->data;
	uint32_t ctl1 = SPI_CTL1(cfg->reg);
	uint32_t new_ctl1 = ctl1;

	if (!data->use_dma && atomic_get(&data->state) == SPI_GD32_XFER_ACTIVE &&
	    data->pending_rx == 0U && spi_gd32_transfer_ongoing(data)) {
		new_ctl1 |= SPI_CTL1_TBEIE;
	} else {
		new_ctl1 &= ~SPI_CTL1_TBEIE;
	}

	if (new_ctl1 != ctl1) {
		SPI_CTL1(cfg->reg) = new_ctl1;
	}
}

static void spi_gd32_isr(const struct device *dev)
{
	const struct spi_gd32_config *cfg = dev->config;
	struct spi_gd32_data *data = dev->data;
	struct spi_context *ctx = &data->ctx;
	uint32_t stat;
	uint32_t errs;

	if (atomic_get(&data->state) != SPI_GD32_XFER_ACTIVE) {
		spi_gd32_disable_irqs_and_dma(cfg->reg);
		return;
	}

	stat = SPI_STAT(cfg->reg);
	errs = stat & SPI_GD32_ERR_MASK;
	if (errs != 0U) {
		bool rxor_only = (errs == SPI_STAT_RXORERR);
		bool ignore_rxor = rxor_only && !data->use_dma &&
				   (!data->hw_rx_expected ||
				    (spi_context_total_rx_len(ctx) == 0U));

		if (!ignore_rxor) {
			spi_gd32_log_errors(dev, "isr", stat);
			spi_gd32_request_finish(dev, -EIO);
			return;
		}

		spi_gd32_clear_errors(cfg->reg);
		spi_gd32_flush_rx(cfg->reg);
		data->pending_rx = 0U;
	}

	/* DMA path uses DMA IRQs for data movement. */
	if (data->use_dma) {
		return;
	}

	for (int i = 0; i < 4; i++) {
		bool progress = false;
		uint32_t stat_iter = SPI_STAT(cfg->reg);
		uint32_t errs_iter = stat_iter & SPI_GD32_ERR_MASK;

		if (errs_iter != 0U) {
			bool rxor_only = (errs_iter == SPI_STAT_RXORERR);
			bool ignore_rxor = rxor_only &&
					   (!data->hw_rx_expected ||
					    (spi_context_total_rx_len(ctx) == 0U));

			if (!ignore_rxor) {
				spi_gd32_log_errors(dev, "isr", stat_iter);
				spi_gd32_clear_errors(cfg->reg);
				spi_gd32_request_finish(dev, -EIO);
				return;
			}

			spi_gd32_clear_errors(cfg->reg);
			spi_gd32_flush_rx(cfg->reg);
			data->pending_rx = 0U;
			progress = true;
		}

		if ((stat_iter & SPI_STAT_RBNE) != 0U) {
			uint16_t rx = (uint16_t)SPI_DATA(cfg->reg);

			if (data->hw_rx_expected && data->pending_rx != 0U) {
				data->pending_rx--;
			}

			if (spi_context_rx_buf_on(ctx)) {
				if (data->dfs == 1U) {
					UNALIGNED_PUT((uint8_t)rx, (uint8_t *)ctx->rx_buf);
				} else {
					UNALIGNED_PUT(rx, (uint16_t *)ctx->rx_buf);
				}
			}

			spi_context_update_rx(ctx, data->dfs, 1);
			progress = true;
		}

		if ((stat_iter & SPI_STAT_TBE) != 0U &&
		    (!data->hw_rx_expected || data->pending_rx == 0U) &&
		    spi_gd32_transfer_ongoing(data)) {
			uint16_t tx = 0U;

			if (spi_context_tx_buf_on(ctx)) {
				if (data->dfs == 1U) {
					tx = UNALIGNED_GET((uint8_t *)ctx->tx_buf);
				} else {
					tx = UNALIGNED_GET((uint16_t *)ctx->tx_buf);
				}
			}

			SPI_DATA(cfg->reg) = tx;
			spi_context_update_tx(ctx, data->dfs, 1);
			if (data->hw_rx_expected) {
				data->pending_rx++;
			}
			progress = true;
		}

		if (!spi_gd32_transfer_ongoing(data) && data->pending_rx == 0U) {
			spi_gd32_request_finish(dev, 0);
			return;
		}

		if (!progress) {
			break;
		}
	}

	spi_gd32_update_tbe_irq(dev);
}

#endif /* SPI_GD32_INTERRUPT */

#ifdef CONFIG_SPI_GD32_DMA

static int spi_gd32_dma_dir_from_channel(const struct device *dev, const struct device *dma_dev,
					 uint32_t channel)
{
	const struct spi_gd32_config *cfg = dev->config;

	for (size_t i = 0; i < ARRAY_SIZE(cfg->dma); i++) {
		if (dma_dev == cfg->dma[i].dev && channel == cfg->dma[i].channel) {
			return (int)i;
		}
	}

	return -EINVAL;
}

static void spi_gd32_dma_callback(const struct device *dma_dev, void *arg, uint32_t channel,
				  int status)
{
	const struct device *dev = (const struct device *)arg;
	const struct spi_gd32_config *cfg = dev->config;
	struct spi_gd32_data *data = dev->data;
	struct spi_context *ctx = &data->ctx;
	int dir;
	atomic_val_t mask;

	if (atomic_get(&data->state) != SPI_GD32_XFER_ACTIVE) {
		return;
	}

	dir = spi_gd32_dma_dir_from_channel(dev, dma_dev, channel);
	if (dir < 0) {
		LOG_ERR("Unexpected DMA callback dev=%p ch=%u", dma_dev, channel);
		spi_gd32_request_finish(dev, -EIO);
		return;
	}

	if (status < 0) {
		LOG_ERR("DMA error dev=%p ch=%u status=%d", dma_dev, channel, status);
		spi_gd32_request_finish(dev, status);
		return;
	}

	mask = atomic_or(&data->dma_done_mask, BIT(dir));
	mask |= BIT(dir);

	if ((mask & (BIT(RX) | BIT(TX))) != (BIT(RX) | BIT(TX))) {
		return;
	}

	/* Current chunk finished. */
	atomic_set(&data->dma_done_mask, 0);

	SPI_CTL1(cfg->reg) &= ~(SPI_CTL1_DMATEN | SPI_CTL1_DMAREN);
	for (size_t i = 0; i < spi_gd32_dma_enabled_num(dev); i++) {
		dma_stop(cfg->dma[i].dev, cfg->dma[i].channel);
	}

	spi_context_update_tx(ctx, data->dfs, data->dma_chunk_frames);
	spi_context_update_rx(ctx, data->dfs, data->dma_chunk_frames);

	if (!spi_gd32_transfer_ongoing(data)) {
		spi_gd32_request_finish(dev, 0);
		return;
	}

	if (spi_gd32_start_dma_transceive(dev) < 0) {
		spi_gd32_request_finish(dev, -EIO);
	}
}

#endif /* DMA */

static int spi_gd32_release(const struct device *dev, const struct spi_config *config)
{
	struct spi_gd32_data *data = dev->data;

	if (!spi_context_configured(&data->ctx, config)) {
		return -EINVAL;
	}

#ifdef CONFIG_MULTITHREADING
	if (data->ctx.owner != config) {
		return -EALREADY;
	}
#endif

	if (data->busy) {
		return -EBUSY;
	}

	spi_context_unlock_unconditionally(&data->ctx);
	spi_gd32_abort(dev);

	return 0;
}

static DEVICE_API(spi, spi_gd32_driver_api) = {.transceive = spi_gd32_transceive,
#ifdef CONFIG_SPI_ASYNC
					       .transceive_async = spi_gd32_transceive_async,
#endif
#ifdef CONFIG_SPI_RTIO
					       .iodev_submit = spi_rtio_iodev_default_submit,
#endif
					       .release = spi_gd32_release};

int spi_gd32_init(const struct device *dev)
{
	struct spi_gd32_data *data = dev->data;
	const struct spi_gd32_config *cfg = dev->config;
	int ret;
#ifdef CONFIG_SPI_GD32_DMA
	uint32_t ch_filter;
#endif

	(void)clock_control_on(GD32_CLOCK_CONTROLLER, (clock_control_subsys_t)&cfg->clkid);

	(void)reset_line_toggle_dt(&cfg->reset);

	ret = pinctrl_apply_state(cfg->pcfg, PINCTRL_STATE_DEFAULT);
	if (ret) {
		LOG_ERR("Failed to apply pinctrl state");
		return ret;
	}

#ifdef CONFIG_SPI_GD32_DMA
	if ((cfg->dma[RX].dev && !cfg->dma[TX].dev) || (cfg->dma[TX].dev && !cfg->dma[RX].dev)) {
		LOG_ERR("DMA must be enabled for both TX and RX channels");
		return -ENODEV;
	}

	for (size_t i = 0; i < spi_gd32_dma_enabled_num(dev); i++) {
		if (!device_is_ready(cfg->dma[i].dev)) {
			LOG_ERR("DMA %s not ready", cfg->dma[i].dev->name);
			return -ENODEV;
		}

		ch_filter = BIT(cfg->dma[i].channel);
		ret = dma_request_channel(cfg->dma[i].dev, &ch_filter);
		if (ret < 0) {
			LOG_ERR("dma_request_channel failed %d", ret);
			return ret;
		}
	}
#endif

	ret = spi_context_cs_configure_all(&data->ctx);
	if (ret < 0) {
		return ret;
	}

	data->dev = dev;
	k_work_init_delayable(&data->finish_work, spi_gd32_finish_work_handler);
	k_work_init_delayable(&data->timeout_work, spi_gd32_timeout_work_handler);
	atomic_set(&data->state, SPI_GD32_XFER_IDLE);
	data->busy = false;

#ifdef CONFIG_SPI_GD32_INTERRUPT
	cfg->irq_configure(dev);
#endif

	spi_context_unlock_unconditionally(&data->ctx);

	return 0;
}

#define DMA_INITIALIZER(idx, dir)                                                                  \
	{                                                                                          \
		.dev = DEVICE_DT_GET(DT_INST_DMAS_CTLR_BY_NAME(idx, dir)),                         \
		.channel = DT_INST_DMAS_CELL_BY_NAME(idx, dir, channel),                           \
		.slot = COND_CODE_1(                                           \
			DT_HAS_COMPAT_STATUS_OKAY(gd_gd32_dma_v1),             \
			(DT_INST_DMAS_CELL_BY_NAME(idx, dir, slot)), (0)),                            \
			 .config = DT_INST_DMAS_CELL_BY_NAME(idx, dir, config),                    \
			 .fifo_threshold = COND_CODE_1(                                 \
			DT_HAS_COMPAT_STATUS_OKAY(gd_gd32_dma_v1),             \
			(DT_INST_DMAS_CELL_BY_NAME(idx, dir, fifo_threshold)), \
			(0)),         \
			 }

#define DMAS_DECL(idx)                                                                             \
	{                                                                                          \
		COND_CODE_1(DT_INST_DMAS_HAS_NAME(idx, rx),                    \
			    (DMA_INITIALIZER(idx, rx)), ({0})),                                          \
			 COND_CODE_1(DT_INST_DMAS_HAS_NAME(idx, tx),                    \
			    (DMA_INITIALIZER(idx, tx)), ({0})),       \
			 }

#define GD32_IRQ_CONFIGURE(idx)                                                                    \
	static void spi_gd32_irq_configure_##idx(const struct device *dev)                         \
	{                                                                                          \
		ARG_UNUSED(dev);                                                                   \
		IRQ_CONNECT(DT_INST_IRQN(idx), DT_INST_IRQ(idx, priority), spi_gd32_isr,           \
			    DEVICE_DT_INST_GET(idx), 0);                                           \
		irq_enable(DT_INST_IRQN(idx));                                                     \
	}

#define GD32_SPI_INIT(idx)                                                                         \
	PINCTRL_DT_INST_DEFINE(idx);                                                               \
	IF_ENABLED(CONFIG_SPI_GD32_INTERRUPT, (GD32_IRQ_CONFIGURE(idx)));                           \
	static struct spi_gd32_data spi_gd32_data_##idx = {                                        \
		SPI_CONTEXT_INIT_LOCK(spi_gd32_data_##idx, ctx),                                   \
		SPI_CONTEXT_INIT_SYNC(spi_gd32_data_##idx, ctx),                                   \
		SPI_CONTEXT_CS_GPIOS_INITIALIZE(DT_DRV_INST(idx), ctx)};                           \
	static struct spi_gd32_config spi_gd32_config_##idx = {                                    \
		.reg = DT_INST_REG_ADDR(idx),                                                      \
		.clkid = DT_INST_CLOCKS_CELL(idx, id),                                             \
		.reset = RESET_DT_SPEC_INST_GET(idx),                                              \
		.pcfg = PINCTRL_DT_INST_DEV_CONFIG_GET(idx),                                       \
		IF_ENABLED(CONFIG_SPI_GD32_DMA, (.dma = DMAS_DECL(idx),))                                                                         \
				    IF_ENABLED(CONFIG_SPI_GD32_INTERRUPT,			       \
			   (.irq_configure = spi_gd32_irq_configure_## idx)) };   \
	SPI_DEVICE_DT_INST_DEFINE(idx, spi_gd32_init, NULL, &spi_gd32_data_##idx,                  \
				  &spi_gd32_config_##idx, POST_KERNEL, CONFIG_SPI_INIT_PRIORITY,   \
				  &spi_gd32_driver_api);

DT_INST_FOREACH_STATUS_OKAY(GD32_SPI_INIT)
