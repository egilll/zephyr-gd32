/*
 * Copyright (c) 2021 BrainCo Inc.
 * Copyright (c) 2026 Ylhyra ehf.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT gd_gd32_spi

#include <errno.h>

#include <zephyr/drivers/clock_control.h>
#include <zephyr/drivers/clock_control/gd32.h>
#include <zephyr/drivers/pinctrl.h>
#include <zephyr/drivers/reset.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/drivers/spi/rtio.h>
#include <zephyr/cache.h>
#include <zephyr/irq.h>
#include <zephyr/kernel.h>
#include <zephyr/linker/linker-defs.h>
#include <zephyr/logging/log.h>
#include <zephyr/mem_mgmt/mem_attr.h>
#include <zephyr/sys/util.h>
#ifdef CONFIG_SPI_GD32_DMA
#include <zephyr/drivers/dma.h>
#include <zephyr/drivers/dma/dma_gd32.h>
#endif

#include <gd32_spi.h>

LOG_MODULE_REGISTER(spi_gd32, CONFIG_SPI_LOG_LEVEL);

#include "spi_context.h"

#define SPI_GD32_ERR_MASK             (SPI_STAT_TXURERR | SPI_STAT_RXORERR | SPI_STAT_CONFERR | SPI_STAT_CRCERR | SPI_STAT_FERR)
#define SPI_GD32_PSC_MAX              0x7U
#define SPI_GD32_RX_CAPACITY_FRAMES   2U
#define SPI_GD32_INIT_MASK            0x00003040U

enum spi_gd32_xfer_mode {
	SPI_GD32_XFER_POLLING,
	SPI_GD32_XFER_INTERRUPT,
	SPI_GD32_XFER_DMA,
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
};

#if defined(CONFIG_NOCACHE_MEMORY)
#define SPI_GD32_DMA_DUMMY_ATTR __nocache
#else
#define SPI_GD32_DMA_DUMMY_ATTR
#endif

static uint32_t dummy_tx SPI_GD32_DMA_DUMMY_ATTR;
static uint32_t dummy_rx SPI_GD32_DMA_DUMMY_ATTR;

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
	uint8_t dfs;
	size_t frames_left;
	size_t rx_pending;
	bool transfer_tx;
	bool transfer_rx;
	enum spi_gd32_xfer_mode xfer_mode;
	bool transfer_active;
#ifdef CONFIG_SPI_GD32_DMA
	struct spi_gd32_dma_data dma[NUM_OF_DIRECTION];
#endif
	bool reset_needed;
};

#ifdef CONFIG_SPI_GD32_DMA
static void spi_gd32_dma_callback(const struct device *dma_dev, void *arg, uint32_t channel,
				  int status);
static int spi_gd32_dma_prepare_chunk(const struct device *dev);
static int spi_gd32_dma_start_chunk(const struct device *dev);
static bool spi_gd32_dma_bufs_usable(const struct spi_buf_set *bufs, uint8_t dfs);
static int spi_gd32_dma_setup(const struct device *dev, enum spi_gd32_dma_direction dir,
			      size_t chunk_len);
static void spi_gd32_enable_rx_dma_request(const struct spi_gd32_config *cfg);
static void spi_gd32_enable_tx_dma_request(const struct spi_gd32_config *cfg);
#endif
#ifdef CONFIG_SPI_GD32_INTERRUPT
static int spi_gd32_start_interrupt_transfer(const struct device *dev);
static void spi_gd32_isr(const struct device *dev);
#endif
static int spi_gd32_polling_transfer(const struct device *dev);
static bool spi_gd32_can_write_frame(const struct spi_gd32_data *data);
static bool spi_gd32_is_slave(const struct spi_gd32_data *data);
static bool spi_gd32_is_rx_only(const struct spi_gd32_data *data);
static bool spi_gd32_shift_slave(const struct device *dev, uint32_t *stat);

static inline uint32_t spi_gd32_stat(const struct spi_gd32_config *cfg)
{
	return SPI_STAT(cfg->reg);
}

#ifdef CONFIG_SPI_GD32_DMA
static uint32_t spi_gd32_get_errors(const struct spi_gd32_config *cfg)
{
	return spi_gd32_stat(cfg) & SPI_GD32_ERR_MASK;
}
#endif

static void spi_gd32_log_errors(const struct device *dev, uint32_t errors)
{
	if (errors != 0U) {
		LOG_ERR("%s error status 0x%08x", dev->name, errors);
	}
}

static void spi_gd32_clear_errors(const struct spi_gd32_config *cfg, uint32_t stat)
{
	if ((stat & SPI_STAT_TXURERR) != 0U) {
		(void)SPI_STAT(cfg->reg);
	}

	if ((stat & SPI_STAT_RXORERR) != 0U) {
		(void)SPI_DATA(cfg->reg);
		(void)SPI_STAT(cfg->reg);
	}

	if ((stat & SPI_STAT_CONFERR) != 0U) {
		uint32_t ctl0 = SPI_CTL0(cfg->reg);

		(void)SPI_STAT(cfg->reg);
		SPI_CTL0(cfg->reg) = ctl0;
	}

	if ((stat & SPI_STAT_CRCERR) != 0U) {
		SPI_STAT(cfg->reg) &= ~SPI_STAT_CRCERR;
	}

	if ((stat & SPI_STAT_FERR) != 0U) {
		SPI_STAT(cfg->reg) &= ~SPI_STAT_FERR;
	}
}

static uint16_t spi_gd32_discard_frame(const struct spi_gd32_config *cfg)
{
	return (uint16_t)SPI_DATA(cfg->reg);
}

static void spi_gd32_flush_stale_status(const struct spi_gd32_config *cfg)
{
	uint32_t stat = spi_gd32_stat(cfg);

	if ((stat & SPI_STAT_RBNE) != 0U) {
		(void)spi_gd32_discard_frame(cfg);
		stat = spi_gd32_stat(cfg);
	}

	spi_gd32_clear_errors(cfg, stat);
}

static void spi_gd32_disable_interrupts(const struct spi_gd32_config *cfg)
{
	SPI_CTL1(cfg->reg) &= ~(SPI_CTL1_RBNEIE | SPI_CTL1_TBEIE | SPI_CTL1_ERRIE);
}

#ifdef CONFIG_SPI_GD32_INTERRUPT
static void spi_gd32_update_interrupt_mask(const struct spi_gd32_config *cfg,
					   const struct spi_gd32_data *data)
{
	uint32_t ctl1 = SPI_CTL1(cfg->reg);

	ctl1 &= ~(SPI_CTL1_ERRIE | SPI_CTL1_RBNEIE | SPI_CTL1_TBEIE);
	ctl1 |= SPI_CTL1_ERRIE;

	if (spi_gd32_is_slave(data)) {
		if (data->transfer_rx || data->transfer_tx) {
			ctl1 |= SPI_CTL1_RBNEIE;
		}

		if (!spi_gd32_is_rx_only(data) && spi_gd32_can_write_frame(data)) {
			ctl1 |= SPI_CTL1_TBEIE;
		}
	} else {
		if ((data->rx_pending != 0U) || data->transfer_rx) {
			ctl1 |= SPI_CTL1_RBNEIE;
		}

		if (spi_gd32_can_write_frame(data)) {
			ctl1 |= SPI_CTL1_TBEIE;
		}
	}

	SPI_CTL1(cfg->reg) = ctl1;
}
#endif

static bool spi_gd32_is_slave_operation(uint16_t operation)
{
	return SPI_OP_MODE_GET(operation) == SPI_OP_MODE_SLAVE;
}

static bool spi_gd32_is_slave_config(const struct spi_config *config)
{
	return (config != NULL) && spi_gd32_is_slave_operation(config->operation);
}

static bool spi_gd32_is_slave(const struct spi_gd32_data *data)
{
	return spi_gd32_is_slave_config(data->ctx.config);
}

static bool spi_gd32_is_rx_only(const struct spi_gd32_data *data)
{
	return !data->transfer_tx && data->transfer_rx;
}

static bool spi_gd32_is_tx_only(const struct spi_gd32_data *data)
{
	return data->transfer_tx && !data->transfer_rx;
}

static bool spi_gd32_can_write_frame(const struct spi_gd32_data *data)
{
	size_t capacity = SPI_GD32_RX_CAPACITY_FRAMES;

	if ((data->xfer_mode == SPI_GD32_XFER_POLLING) && !spi_gd32_is_slave(data) &&
	    spi_gd32_is_tx_only(data)) {
		capacity = 1U;
	}

	return (data->frames_left != 0U) && (data->rx_pending < capacity);
}
static size_t spi_gd32_remaining_frames(struct spi_context *ctx, uint8_t dfs)
{
	return DIV_ROUND_UP(MAX(spi_context_total_tx_len(ctx), spi_context_total_rx_len(ctx)), dfs);
}

static bool spi_gd32_keep_enabled(const struct spi_gd32_data *data)
{
	return !spi_gd32_is_slave(data) && (data->ctx.config != NULL) &&
	       ((data->ctx.config->operation & SPI_HOLD_ON_CS) != 0U);
}

#ifdef CONFIG_SPI_GD32_DMA
static bool spi_gd32_dma_enabled(const struct device *dev)
{
	const struct spi_gd32_config *cfg = dev->config;

	return (cfg->dma[TX].dev != NULL) && (cfg->dma[RX].dev != NULL);
}

static void spi_gd32_disable_dma_requests(const struct spi_gd32_config *cfg)
{
	SPI_CTL1(cfg->reg) &= ~(SPI_CTL1_DMATEN | SPI_CTL1_DMAREN);
}

static void spi_gd32_stop_dma(const struct device *dev)
{
	const struct spi_gd32_config *cfg = dev->config;

	if (!spi_gd32_dma_enabled(dev)) {
		return;
	}

	for (size_t i = 0; i < NUM_OF_DIRECTION; i++) {
		(void)dma_stop(cfg->dma[i].dev, cfg->dma[i].channel);
	}
}

static bool spi_gd32_dma_addr_ok(uintptr_t addr, size_t len)
{
#if IS_ENABLED(CONFIG_MEM_ATTR)
	int ret;

	if (IS_ENABLED(CONFIG_MMU) || (len == 0U)) {
		return true;
	}

	if ((addr + len) < addr) {
		return false;
	}

	ret = mem_attr_check_buf((void *)addr, len, DT_MEM_DMA);
	if (ret == 0 || ret == -ENOSYS) {
		return true;
	}

	return ret == -ENOBUFS;
#else
	ARG_UNUSED(addr);
	ARG_UNUSED(len);
#endif

	return true;
}

static bool spi_gd32_dma_bufs_usable(const struct spi_buf_set *bufs, uint8_t dfs)
{
	if (bufs == NULL) {
		return true;
	}

	for (size_t i = 0; i < bufs->count; i++) {
		const struct spi_buf *buf = &bufs->buffers[i];

		if ((buf->buf == NULL) || (buf->len == 0U)) {
			continue;
		}

		if ((dfs == 2U) && !IS_ALIGNED((uintptr_t)buf->buf, 2U)) {
			return false;
		}

		if (!spi_gd32_dma_addr_ok((uintptr_t)buf->buf, buf->len)) {
			return false;
		}
	}

	return true;
}

static bool spi_gd32_dma_possible(const struct device *dev, const struct spi_buf_set *tx_bufs,
				  const struct spi_buf_set *rx_bufs)
{
	struct spi_gd32_data *data = dev->data;

	if (spi_gd32_is_slave(data) || !spi_gd32_dma_enabled(dev)) {
		return false;
	}

	return spi_gd32_dma_bufs_usable(tx_bufs, data->dfs) &&
	       spi_gd32_dma_bufs_usable(rx_bufs, data->dfs);
}
#else
static void spi_gd32_disable_dma_requests(const struct spi_gd32_config *cfg)
{
	ARG_UNUSED(cfg);
}
#endif

static uint32_t spi_gd32_frame_timeout_us(const struct spi_gd32_data *data, size_t frames)
{
	if (spi_gd32_is_slave(data)) {
		return MAX((uint32_t)CONFIG_SPI_COMPLETION_TIMEOUT_TOLERANCE * USEC_PER_MSEC, 1U);
	}

	uint32_t bits = SPI_WORD_SIZE_GET(data->ctx.config->operation);
	uint64_t timeout_us;

	timeout_us = (uint64_t)MAX(frames, 1U) * bits * USEC_PER_SEC;
	timeout_us = DIV_ROUND_UP(timeout_us, data->ctx.config->frequency);
	timeout_us += (uint64_t)CONFIG_SPI_COMPLETION_TIMEOUT_TOLERANCE * USEC_PER_MSEC;

	return (uint32_t)MIN(timeout_us, (uint64_t)UINT32_MAX);
}

#if defined(CONFIG_SPI_GD32_INTERRUPT) || defined(CONFIG_SPI_GD32_DMA)
static k_timeout_t spi_gd32_completion_timeout(const struct spi_gd32_data *data)
{
	return K_USEC(spi_gd32_frame_timeout_us(data, MAX(data->frames_left, 2U)));
}
#endif

#ifdef CONFIG_SPI_SLAVE
struct spi_gd32_slave_progress {
	size_t frames_left;
	size_t rx_pending;
	int recv_frames;
};

static struct spi_gd32_slave_progress spi_gd32_get_slave_progress(struct spi_gd32_data *data)
{
	struct spi_gd32_slave_progress progress;
	unsigned int key = irq_lock();

	progress.frames_left = data->frames_left;
	progress.rx_pending = data->rx_pending;
	progress.recv_frames = data->ctx.recv_frames;

	irq_unlock(key);

	return progress;
}

static bool spi_gd32_slave_progressed(const struct spi_gd32_slave_progress *before,
				      const struct spi_gd32_slave_progress *after)
{
	return (before->frames_left != after->frames_left) ||
	       (before->rx_pending != after->rx_pending) ||
	       (before->recv_frames != after->recv_frames);
}
#endif

static uint32_t spi_gd32_spin_timeout(const struct spi_gd32_data *data, size_t frames)
{
	uint64_t timeout = (uint64_t)spi_gd32_frame_timeout_us(data, frames) * 64U;

	return (uint32_t)MAX(MIN(timeout, (uint64_t)UINT32_MAX), 1000U);
}

static int spi_gd32_wait_for_status(const struct device *dev, uint32_t flags, uint32_t *stat_out)
{
	struct spi_gd32_data *data = dev->data;
	const struct spi_gd32_config *cfg = dev->config;
	uint32_t timeout = spi_gd32_spin_timeout(data, 2U);
	uint32_t stat;

	while (timeout-- != 0U) {
		stat = spi_gd32_stat(cfg);
		if (((stat & SPI_GD32_ERR_MASK) != 0U) || ((stat & flags) != 0U)) {
			goto out;
		}
	}

	LOG_ERR("%s timed out waiting for SPI flags 0x%08x", dev->name, flags);
	return -ETIMEDOUT;

out:
	if ((stat & SPI_GD32_ERR_MASK) != 0U) {
		spi_gd32_log_errors(dev, stat & SPI_GD32_ERR_MASK);
		spi_gd32_clear_errors(cfg, stat);
		return -EIO;
	}

	if (stat_out != NULL) {
		*stat_out = stat;
	}

	return 0;
}

static int spi_gd32_wait_until_idle(const struct device *dev)
{
	struct spi_gd32_data *data = dev->data;
	const struct spi_gd32_config *cfg = dev->config;
	uint32_t timeout = spi_gd32_spin_timeout(data, 2U);
	uint32_t stat;

	while (timeout-- != 0U) {
		stat = spi_gd32_stat(cfg);
		if (((stat & SPI_GD32_ERR_MASK) != 0U) ||
		    (((stat & SPI_STAT_TBE) != 0U) && ((stat & SPI_STAT_TRANS) == 0U))) {
			goto out;
		}
	}

	LOG_ERR("%s timed out waiting for SPI idle", dev->name);
	return -ETIMEDOUT;

out:
	if ((stat & SPI_GD32_ERR_MASK) != 0U) {
		spi_gd32_log_errors(dev, stat & SPI_GD32_ERR_MASK);
		spi_gd32_clear_errors(cfg, stat);
		return -EIO;
	}

	return 0;
}

#if defined(CONFIG_SPI_GD32_INTERRUPT) || defined(CONFIG_SPI_GD32_DMA)
static bool spi_gd32_mark_transfer_inactive(struct spi_gd32_data *data)
{
	unsigned int key = irq_lock();
	bool active = data->transfer_active;

	if (active) {
		data->transfer_active = false;
	}

	irq_unlock(key);

	return active;
}
#endif

#if defined(CONFIG_SPI_GD32_INTERRUPT) || defined(CONFIG_SPI_GD32_DMA)
static int spi_gd32_wait_for_completion(const struct device *dev)
{
	struct spi_gd32_data *data = dev->data;
	struct spi_context *ctx = &data->ctx;
#ifdef CONFIG_SPI_SLAVE
	struct spi_gd32_slave_progress progress = spi_gd32_get_slave_progress(data);
#endif

	if (!spi_gd32_is_slave(data)) {
		return spi_context_wait_for_completion(ctx);
	}

	while (k_sem_take(&ctx->sync, spi_gd32_completion_timeout(data)) != 0) {
#ifdef CONFIG_SPI_SLAVE
		struct spi_gd32_slave_progress current = spi_gd32_get_slave_progress(data);

		if (spi_gd32_slave_progressed(&progress, &current)) {
			progress = current;
			continue;
		}
#endif
		LOG_ERR("%s timed out waiting for slave transfer completion", dev->name);
		return -ETIMEDOUT;
	}

#ifdef CONFIG_SPI_SLAVE
	if (ctx->sync_status == 0) {
		return ctx->recv_frames;
	}
#endif

	return ctx->sync_status;
}
#endif

static void spi_gd32_write_frame(const struct device *dev)
{
	struct spi_gd32_data *data = dev->data;
	const struct spi_gd32_config *cfg = dev->config;
	struct spi_context *ctx = &data->ctx;
	uint16_t frame = 0U;

	if (data->dfs == 1U) {
		if (spi_context_tx_buf_on(ctx)) {
			frame = UNALIGNED_GET((uint8_t *)ctx->tx_buf);
		}
	} else if (spi_context_tx_buf_on(ctx)) {
		frame = UNALIGNED_GET((uint16_t *)ctx->tx_buf);
	}

	SPI_DATA(cfg->reg) = frame;

	if (spi_context_tx_on(ctx)) {
		spi_context_update_tx(ctx, data->dfs, 1U);
	}

	data->frames_left--;
	data->rx_pending++;
}

static void spi_gd32_read_frame(const struct device *dev)
{
	struct spi_gd32_data *data = dev->data;
	const struct spi_gd32_config *cfg = dev->config;
	struct spi_context *ctx = &data->ctx;
	uint16_t frame = SPI_DATA(cfg->reg);

	if (data->dfs == 1U) {
		if (spi_context_rx_buf_on(ctx)) {
			UNALIGNED_PUT((uint8_t)frame, (uint8_t *)ctx->rx_buf);
		}
	} else if (spi_context_rx_buf_on(ctx)) {
		UNALIGNED_PUT(frame, (uint16_t *)ctx->rx_buf);
	}

	if (spi_context_rx_on(ctx)) {
		spi_context_update_rx(ctx, data->dfs, 1U);
	}

	if (data->rx_pending != 0U) {
		data->rx_pending--;
	}
}

static void spi_gd32_drop_frame(const struct device *dev)
{
	struct spi_gd32_data *data = dev->data;
	const struct spi_gd32_config *cfg = dev->config;

	(void)SPI_DATA(cfg->reg);

	if (data->rx_pending != 0U) {
		data->rx_pending--;
	}
}

static void spi_gd32_handle_slave_rx_frame(const struct device *dev)
{
	struct spi_gd32_data *data = dev->data;
	struct spi_context *ctx = &data->ctx;

	if (spi_gd32_is_rx_only(data)) {
		spi_gd32_read_frame(dev);
		data->frames_left--;
	} else if (data->rx_pending != 0U) {
		if (data->transfer_rx && spi_context_rx_on(ctx)) {
			spi_gd32_read_frame(dev);
		} else {
			spi_gd32_drop_frame(dev);
		}
	} else {
		spi_gd32_drop_frame(dev);
	}
}

static bool spi_gd32_shift_slave(const struct device *dev, uint32_t *stat)
{
	struct spi_gd32_data *data = dev->data;
	const struct spi_gd32_config *cfg = dev->config;
	bool progressed = false;

	if (!spi_gd32_is_rx_only(data)) {
		while (((*stat) & SPI_STAT_TBE) != 0U && spi_gd32_can_write_frame(data)) {
			spi_gd32_write_frame(dev);
			progressed = true;
			*stat = spi_gd32_stat(cfg);
		}
	}

	while (((*stat) & SPI_STAT_RBNE) != 0U) {
		spi_gd32_handle_slave_rx_frame(dev);
		progressed = true;
		*stat = spi_gd32_stat(cfg);
	}

	if (!spi_gd32_is_rx_only(data)) {
		while (((*stat) & SPI_STAT_TBE) != 0U && spi_gd32_can_write_frame(data)) {
			spi_gd32_write_frame(dev);
			progressed = true;
			*stat = spi_gd32_stat(cfg);
		}
	}

	return progressed;
}

#ifdef CONFIG_SPI_GD32_INTERRUPT

static int spi_gd32_handle_slave_overrun(const struct device *dev, uint32_t stat)
{
	struct spi_gd32_data *data = dev->data;
	const struct spi_gd32_config *cfg = dev->config;
	bool handled_data = false;

	if (!spi_gd32_is_slave(data) || ((stat & SPI_GD32_ERR_MASK) != SPI_STAT_RXORERR)) {
		return -EIO;
	}

	if ((stat & SPI_STAT_RBNE) != 0U) {
		spi_gd32_handle_slave_rx_frame(dev);
		handled_data = true;
		stat = spi_gd32_stat(cfg);
	}

	if ((stat & SPI_STAT_RBNE) != 0U) {
		spi_gd32_handle_slave_rx_frame(dev);
		handled_data = true;
	}

	if (handled_data) {
		(void)SPI_STAT(cfg->reg);
	} else {
		spi_gd32_clear_errors(cfg, stat);
		return -EIO;
	}

	if ((data->frames_left == 0U) && (data->rx_pending == 0U)) {
		return 1;
	}

	return 0;
}
#endif

static void spi_gd32_prepare_transfer(const struct device *dev)
{
	struct spi_gd32_data *data = dev->data;

	data->dfs = SPI_WORD_SIZE_GET(data->ctx.config->operation) > 8U ? 2U : 1U;
	data->transfer_tx = spi_context_total_tx_len(&data->ctx) != 0U;
	data->transfer_rx = spi_context_total_rx_len(&data->ctx) != 0U;
	data->frames_left = spi_gd32_remaining_frames(&data->ctx, data->dfs);
	data->rx_pending = 0U;
	data->xfer_mode = SPI_GD32_XFER_POLLING;
	data->transfer_active = false;
	data->reset_needed = false;
}

static int spi_gd32_validate_buffers(const struct spi_buf_set *bufs, uint8_t dfs)
{
	if (bufs == NULL) {
		return 0;
	}

	for (size_t i = 0; i < bufs->count; i++) {
		if ((bufs->buffers[i].len % dfs) != 0U) {
			return -EINVAL;
		}
	}

	return 0;
}

static void spi_gd32_reset_peripheral(const struct device *dev)
{
	const struct spi_gd32_config *cfg = dev->config;
	int ret;

	ret = reset_line_toggle_dt(&cfg->reset);
	if (ret != 0) {
		LOG_ERR("%s failed to reset SPI peripheral (%d)", dev->name, ret);
	}
}

static void spi_gd32_stop_transfer_engine(const struct device *dev)
{
	const struct spi_gd32_config *cfg = dev->config;

	spi_gd32_disable_interrupts(cfg);
	spi_gd32_disable_dma_requests(cfg);
#ifdef CONFIG_SPI_GD32_DMA
	struct spi_gd32_data *data = dev->data;

	if (data->xfer_mode == SPI_GD32_XFER_DMA) {
		spi_gd32_stop_dma(dev);
	}
#endif
}

static void spi_gd32_latch_fatal(struct spi_gd32_data *data, int status)
{
	if (status < 0) {
		data->reset_needed = true;
	}
}

static int spi_gd32_cleanup_transfer(const struct device *dev, int status)
{
	struct spi_gd32_data *data = dev->data;
	const struct spi_gd32_config *cfg = dev->config;

	if ((status == 0) && !data->reset_needed && !spi_gd32_is_slave(data)) {
		status = spi_gd32_wait_until_idle(dev);
	}

	spi_gd32_stop_transfer_engine(dev);

	if (status < 0) {
		spi_gd32_latch_fatal(data, status);
	}

	if (data->reset_needed) {
		SPI_CTL0(cfg->reg) &= ~SPI_CTL0_SPIEN;
		if (!spi_gd32_is_slave(data)) {
			spi_context_cs_control(&data->ctx, false);
		}
		spi_gd32_clear_errors(cfg, spi_gd32_stat(cfg));
		spi_gd32_reset_peripheral(dev);
		data->reset_needed = false;
#ifdef CONFIG_SPI_ASYNC
		if (!data->ctx.asynchronous) {
			data->ctx.config = NULL;
		}
#else
		data->ctx.config = NULL;
#endif
		return status;
	}

	if (!spi_gd32_is_slave(data)) {
		spi_context_cs_control(&data->ctx, false);
	}
	if (!spi_gd32_keep_enabled(data)) {
		SPI_CTL0(cfg->reg) &= ~SPI_CTL0_SPIEN;
	}
	spi_gd32_flush_stale_status(cfg);
	return 0;
}

#ifdef CONFIG_SPI_ASYNC
static void spi_gd32_complete_async(const struct device *dev, int status)
{
	struct spi_gd32_data *data = dev->data;

	status = spi_gd32_cleanup_transfer(dev, status);
	spi_context_complete(&data->ctx, dev, status);
	if (status < 0) {
		data->ctx.config = NULL;
	}
}
#endif

#if defined(CONFIG_SPI_GD32_INTERRUPT) || defined(CONFIG_SPI_GD32_DMA)
static void spi_gd32_signal_completion(const struct device *dev, int status)
{
	struct spi_gd32_data *data = dev->data;

	if (!spi_gd32_mark_transfer_inactive(data)) {
		return;
	}

#ifdef CONFIG_SPI_ASYNC
	if (data->ctx.asynchronous) {
		spi_gd32_complete_async(dev, status);
		return;
	}
#endif

	spi_gd32_latch_fatal(data, status);
	spi_gd32_stop_transfer_engine(dev);
	spi_context_complete(&data->ctx, dev, status);
}
#endif

static int spi_gd32_configure(const struct device *dev, const struct spi_config *config)
{
	struct spi_gd32_data *data = dev->data;
	const struct spi_gd32_config *cfg = dev->config;
	bool slave = spi_gd32_is_slave_config(config);
	bool ti_mode = (config->operation & SPI_FRAME_FORMAT_TI) != 0U;
	uint32_t bus_freq;
	uint32_t prescaler = 0U;

	if (spi_context_configured(&data->ctx, config)) {
		return 0;
	}

	if (slave && !IS_ENABLED(CONFIG_SPI_SLAVE)) {
		LOG_ERR("Slave mode support is disabled");
		return -ENOTSUP;
	}

	if ((config->operation & SPI_HALF_DUPLEX) != 0U) {
		LOG_ERR("Half-duplex mode not supported");
		return -ENOTSUP;
	}

	if ((config->operation & SPI_MODE_LOOP) != 0U) {
		LOG_ERR("Loopback mode not supported");
		return -ENOTSUP;
	}

	if (IS_ENABLED(CONFIG_SPI_EXTENDED_MODES) &&
	    ((config->operation & SPI_LINES_MASK) != SPI_LINES_SINGLE)) {
		LOG_ERR("Only single-line SPI is supported");
		return -ENOTSUP;
	}

	if (SPI_WORD_SIZE_GET(config->operation) != 8U &&
	    SPI_WORD_SIZE_GET(config->operation) != 16U) {
		LOG_ERR("Only 8-bit and 16-bit words are supported");
		return -ENOTSUP;
	}

	if (ti_mode && ((config->operation & SPI_TRANSFER_LSB) != 0U)) {
		LOG_ERR("TI frame format requires MSB-first transfers");
		return -ENOTSUP;
	}

	if (!spi_cs_is_gpio(config) && ((config->operation & SPI_CS_ACTIVE_HIGH) != 0U)) {
		LOG_ERR("Active-high native CS not supported");
		return -ENOTSUP;
	}

	if (slave && spi_cs_is_gpio(config)) {
		LOG_ERR("GPIO CS is not supported in slave mode");
		return -ENOTSUP;
	}

	if (ti_mode && spi_cs_is_gpio(config)) {
		LOG_ERR("TI frame format requires native NSS");
		return -ENOTSUP;
	}

	if (!slave && (config->frequency == 0U)) {
		return -EINVAL;
	}

	if (!slave || (config->frequency != 0U)) {
		if (clock_control_get_rate(GD32_CLOCK_CONTROLLER,
					   (clock_control_subsys_t)&cfg->clkid, &bus_freq) != 0) {
			LOG_ERR("Failed to get SPI clock rate");
			return -EIO;
		}

		if (!slave) {
			prescaler = SPI_GD32_PSC_MAX;
		}

		if (config->frequency != 0U) {
			for (uint32_t i = 0U; i <= SPI_GD32_PSC_MAX; i++) {
				if ((bus_freq >> (i + 1U)) <= config->frequency) {
					prescaler = i;
					break;
				}
			}
		}
	}

	SPI_CTL0(cfg->reg) &= ~SPI_CTL0_SPIEN;
	spi_gd32_disable_interrupts(cfg);
	spi_gd32_disable_dma_requests(cfg);
	spi_gd32_flush_stale_status(cfg);
	spi_gd32_reset_peripheral(dev);

	/* Match the vendor HAL spi_init() field programming after a reset. */
	uint32_t ctl0 = SPI_CTL0(cfg->reg) & SPI_GD32_INIT_MASK;
	uint32_t ctl1 = SPI_CTL1(cfg->reg) &
		~(SPI_CTL1_DMATEN | SPI_CTL1_DMAREN | SPI_CTL1_NSSDRV |
		  SPI_CTL1_TMOD | SPI_CTL1_ERRIE | SPI_CTL1_RBNEIE | SPI_CTL1_TBEIE);

	ctl0 |= slave ? SPI_SLAVE : SPI_MASTER;
	ctl0 |= SPI_TRANSMODE_FULLDUPLEX;
	ctl0 |= (SPI_WORD_SIZE_GET(config->operation) == 16U) ?
		SPI_FRAMESIZE_16BIT : SPI_FRAMESIZE_8BIT;
	ctl0 |= slave ? SPI_NSS_HARD :
		(spi_cs_is_gpio(config) ? SPI_NSS_SOFT : SPI_NSS_HARD);
	ctl0 |= (config->operation & SPI_TRANSFER_LSB) != 0U ?
		SPI_ENDIAN_LSB : SPI_ENDIAN_MSB;

	if ((config->operation & SPI_MODE_CPOL) != 0U) {
		ctl0 |= (config->operation & SPI_MODE_CPHA) != 0U ?
			SPI_CK_PL_HIGH_PH_2EDGE : SPI_CK_PL_HIGH_PH_1EDGE;
	} else {
		ctl0 |= (config->operation & SPI_MODE_CPHA) != 0U ?
			SPI_CK_PL_LOW_PH_2EDGE : SPI_CK_PL_LOW_PH_1EDGE;
	}

	ctl0 |= CTL0_PSC(prescaler);

	SPI_CTL0(cfg->reg) = ctl0;
	SPI_I2SCTL(cfg->reg) &= ~SPI_I2SCTL_I2SSEL;

	if (ti_mode) {
		ctl1 |= SPI_CTL1_TMOD;
	}

	if (!slave && !spi_cs_is_gpio(config)) {
		ctl1 |= SPI_CTL1_NSSDRV;
	}

	SPI_CTL1(cfg->reg) = ctl1;

	data->ctx.config = config;

	return 0;
}

static int spi_gd32_polling_transfer(const struct device *dev)
{
	struct spi_gd32_data *data = dev->data;
	const struct spi_gd32_config *cfg = dev->config;
	int ret;

	if (spi_gd32_is_slave(data) && spi_gd32_is_rx_only(data)) {
		while (data->frames_left != 0U) {
			ret = spi_gd32_wait_for_status(dev, SPI_STAT_RBNE, NULL);
			if (ret != 0) {
				return ret;
			}

			uint32_t stat = spi_gd32_stat(cfg);

			(void)spi_gd32_shift_slave(dev, &stat);
		}

		return 0;
	}

	if (spi_gd32_is_slave(data)) {
		while ((data->rx_pending != 0U) || (data->frames_left != 0U)) {
			uint32_t stat = spi_gd32_stat(cfg);
			bool progressed = spi_gd32_shift_slave(dev, &stat);

			if ((data->frames_left == 0U) && (data->rx_pending == 0U)) {
				return 0;
			}

			if (progressed) {
				continue;
			}

			ret = spi_gd32_wait_for_status(dev, SPI_STAT_TBE | SPI_STAT_RBNE, NULL);
			if (ret != 0) {
				return ret;
			}
		}

		return 0;
	}

	while (data->frames_left != 0U) {
		ret = spi_gd32_wait_for_status(dev, SPI_STAT_TBE, NULL);
		if (ret != 0) {
			return ret;
		}

		spi_gd32_write_frame(dev);

		ret = spi_gd32_wait_for_status(dev, SPI_STAT_RBNE, NULL);
		if (ret != 0) {
			return ret;
		}

		spi_gd32_read_frame(dev);
	}

	return 0;
}

static enum spi_gd32_xfer_mode spi_gd32_select_xfer_mode(const struct device *dev,
							 const struct spi_buf_set *tx_bufs,
							 const struct spi_buf_set *rx_bufs)
{
#ifdef CONFIG_SPI_GD32_DMA
	if (spi_gd32_dma_possible(dev, tx_bufs, rx_bufs)) {
		return SPI_GD32_XFER_DMA;
	}
#endif

#ifdef CONFIG_SPI_GD32_INTERRUPT
	return SPI_GD32_XFER_INTERRUPT;
#endif

	return SPI_GD32_XFER_POLLING;
}

#ifdef CONFIG_SPI_GD32_DMA
static void spi_gd32_enable_rx_dma_request(const struct spi_gd32_config *cfg)
{
	SPI_CTL1(cfg->reg) |= SPI_CTL1_DMAREN;
}

static void spi_gd32_enable_tx_dma_request(const struct spi_gd32_config *cfg)
{
	SPI_CTL1(cfg->reg) |= SPI_CTL1_DMATEN;
}

static int spi_gd32_dma_setup(const struct device *dev, enum spi_gd32_dma_direction dir,
			      size_t chunk_len)
{
	const struct spi_gd32_config *cfg = dev->config;
	struct spi_gd32_data *data = dev->data;
	struct dma_config *dma_cfg = &data->dma[dir].config;
	struct dma_block_config *block_cfg = &data->dma[dir].block;
	const struct spi_gd32_dma_config *dma = &cfg->dma[dir];
	struct spi_context *ctx = &data->ctx;

	if (chunk_len == 0U) {
		return 0;
	}

	memset(dma_cfg, 0, sizeof(*dma_cfg));
	memset(block_cfg, 0, sizeof(*block_cfg));

	dma_cfg->source_burst_length = 1U;
	dma_cfg->dest_burst_length = 1U;
	dma_cfg->user_data = (void *)dev;
	dma_cfg->dma_callback = spi_gd32_dma_callback;
	dma_cfg->block_count = 1U;
	dma_cfg->head_block = block_cfg;
	dma_cfg->dma_slot = dma->slot;
	dma_cfg->channel_priority = GD32_DMA_CONFIG_PRIORITY(dma->config);
	dma_cfg->channel_direction = (dir == TX) ? MEMORY_TO_PERIPHERAL : PERIPHERAL_TO_MEMORY;
	dma_cfg->source_data_size = data->dfs;
	dma_cfg->dest_data_size = data->dfs;

	block_cfg->block_size = chunk_len * data->dfs;
	block_cfg->fifo_mode_control = dma->fifo_threshold;

	if (dir == TX) {
		block_cfg->dest_address = (uint32_t)&SPI_DATA(cfg->reg);
		block_cfg->dest_addr_adj = DMA_ADDR_ADJ_NO_CHANGE;
		if (spi_context_tx_buf_on(ctx)) {
			block_cfg->source_address = (uint32_t)ctx->tx_buf;
			block_cfg->source_addr_adj = DMA_ADDR_ADJ_INCREMENT;
		} else {
			block_cfg->source_address = (uint32_t)&dummy_tx;
			block_cfg->source_addr_adj = DMA_ADDR_ADJ_NO_CHANGE;
		}
	} else {
		block_cfg->source_address = (uint32_t)&SPI_DATA(cfg->reg);
		block_cfg->source_addr_adj = DMA_ADDR_ADJ_NO_CHANGE;
		if (spi_context_rx_buf_on(ctx)) {
			block_cfg->dest_address = (uint32_t)ctx->rx_buf;
			block_cfg->dest_addr_adj = DMA_ADDR_ADJ_INCREMENT;
		} else {
			block_cfg->dest_address = (uint32_t)&dummy_rx;
			block_cfg->dest_addr_adj = DMA_ADDR_ADJ_NO_CHANGE;
		}
	}

	if (dma_config(dma->dev, dma->channel, dma_cfg) != 0) {
		LOG_ERR("%s failed to configure %s DMA channel", dev->name,
			(dir == TX) ? "TX" : "RX");
		return -EIO;
	}

	return 0;
}

static int spi_gd32_dma_start(const struct device *dev, enum spi_gd32_dma_direction dir)
{
	const struct spi_gd32_config *cfg = dev->config;
	const struct spi_gd32_dma_config *dma = &cfg->dma[dir];

	if (dma_start(dma->dev, dma->channel) != 0) {
		LOG_ERR("%s failed to start %s DMA channel", dev->name, (dir == TX) ? "TX" : "RX");
		return -EIO;
	}

	return 0;
}

static int spi_gd32_dma_prepare_chunk(const struct device *dev)
{
	struct spi_gd32_data *data = dev->data;
	size_t chunk_len = spi_context_max_continuous_chunk(&data->ctx);
	int ret;

	if (chunk_len == 0U) {
		return -EIO;
	}

#if defined(CONFIG_CACHE_MANAGEMENT) && defined(CONFIG_DCACHE) && !defined(CONFIG_NOCACHE_MEMORY)
	struct spi_context *ctx = &data->ctx;

	if (!spi_context_tx_buf_on(ctx)) {
		dummy_tx = 0U;
		(void)sys_cache_data_flush_range((void *)&dummy_tx, sizeof(dummy_tx));
	}

	if (!spi_context_rx_buf_on(ctx)) {
		(void)sys_cache_data_invd_range((void *)&dummy_rx, sizeof(dummy_rx));
	}
#endif

	ret = spi_gd32_dma_setup(dev, RX, chunk_len);
	if (ret != 0) {
		return ret;
	}

	ret = spi_gd32_dma_setup(dev, TX, chunk_len);
	if (ret != 0) {
		return ret;
	}

	return 0;
}

static int spi_gd32_dma_start_chunk(const struct device *dev)
{
	const struct spi_gd32_config *cfg = dev->config;
	int ret;

	spi_gd32_disable_dma_requests(cfg);

	ret = spi_gd32_dma_start(dev, RX);
	if (ret != 0) {
		return ret;
	}

	ret = spi_gd32_dma_start(dev, TX);
	if (ret != 0) {
		(void)dma_stop(cfg->dma[RX].dev, cfg->dma[RX].channel);
		return ret;
	}

	spi_gd32_enable_rx_dma_request(cfg);
	spi_gd32_enable_tx_dma_request(cfg);

	return 0;
}

static void spi_gd32_dma_callback(const struct device *dma_dev, void *arg, uint32_t channel,
				  int status)
{
	const struct device *dev = arg;
	const struct spi_gd32_config *cfg = dev->config;
	struct spi_gd32_data *data = dev->data;
	size_t chunk_len;
	uint32_t errors;
	int ret;

	if (!data->transfer_active) {
		return;
	}

	if (status < 0) {
		LOG_ERR("%s DMA error on channel %u: %d", dev->name, channel, status);
		spi_gd32_signal_completion(dev, status);
		return;
	}

	if (status == DMA_STATUS_BLOCK) {
		return;
	}

	errors = spi_gd32_get_errors(cfg);
	if (errors != 0U) {
		spi_gd32_log_errors(dev, errors);
		spi_gd32_clear_errors(cfg, errors);
		spi_gd32_signal_completion(dev, -EIO);
		return;
	}

	if ((dma_dev != cfg->dma[RX].dev) || (channel != cfg->dma[RX].channel)) {
		return;
	}

	spi_gd32_disable_dma_requests(cfg);
	chunk_len = spi_context_max_continuous_chunk(&data->ctx);

	if (spi_context_tx_on(&data->ctx)) {
		spi_context_update_tx(&data->ctx, data->dfs, chunk_len);
	}

	if (spi_context_rx_on(&data->ctx)) {
		spi_context_update_rx(&data->ctx, data->dfs, chunk_len);
	}

	if (spi_context_max_continuous_chunk(&data->ctx) == 0U) {
		spi_gd32_signal_completion(dev, 0);
		return;
	}

	ret = spi_gd32_dma_prepare_chunk(dev);
	if (ret != 0) {
		spi_gd32_signal_completion(dev, ret);
		return;
	}

	ret = spi_gd32_dma_start_chunk(dev);
	if (ret != 0) {
		spi_gd32_signal_completion(dev, ret);
		return;
	}
}
#endif

#ifdef CONFIG_SPI_GD32_INTERRUPT
static int spi_gd32_start_interrupt_transfer(const struct device *dev)
{
	const struct spi_gd32_config *cfg = dev->config;
	struct spi_gd32_data *data = dev->data;

	spi_gd32_update_interrupt_mask(cfg, data);

	return 0;
}

static void spi_gd32_isr(const struct device *dev)
{
	const struct spi_gd32_config *cfg = dev->config;
	struct spi_gd32_data *data = dev->data;
	uint32_t stat;
	uint32_t errors;

	if (!data->transfer_active) {
		return;
	}

	while (true) {
		bool progressed = false;

		stat = spi_gd32_stat(cfg);
		errors = stat & SPI_GD32_ERR_MASK;
		if (errors != 0U) {
			int overrun = spi_gd32_handle_slave_overrun(dev, stat);

			if (overrun > 0) {
				spi_gd32_signal_completion(dev, 0);
				return;
			}
			if (overrun == 0) {
				progressed = true;
				spi_gd32_update_interrupt_mask(cfg, data);
				continue;
			}

			spi_gd32_log_errors(dev, errors);
			spi_gd32_clear_errors(cfg, stat);
			spi_gd32_signal_completion(dev, -EIO);
			return;
		}

		if (spi_gd32_is_slave(data) && spi_gd32_is_rx_only(data)) {
			progressed = spi_gd32_shift_slave(dev, &stat);

			if (data->frames_left == 0U) {
				spi_gd32_signal_completion(dev, 0);
			}

			return;
		}

		if (spi_gd32_is_slave(data) && data->transfer_tx && data->transfer_rx) {
			progressed = spi_gd32_shift_slave(dev, &stat);

			if ((data->frames_left == 0U) && (data->rx_pending == 0U)) {
				spi_gd32_signal_completion(dev, 0);
				return;
			}

			spi_gd32_update_interrupt_mask(cfg, data);

			if (!progressed) {
				return;
			}

			continue;
		}

		while (((stat & SPI_STAT_RBNE) != 0U) && (data->rx_pending != 0U)) {
			spi_gd32_read_frame(dev);
			progressed = true;
			stat = spi_gd32_stat(cfg);
		}

		if (((stat & SPI_STAT_RBNE) != 0U) && (data->rx_pending == 0U)) {
			spi_gd32_drop_frame(dev);
			progressed = true;
			stat = spi_gd32_stat(cfg);
		}

		if (((stat & SPI_STAT_TBE) != 0U) && spi_gd32_can_write_frame(data)) {
			spi_gd32_write_frame(dev);
			progressed = true;
		}

		if ((data->frames_left == 0U) && (data->rx_pending == 0U)) {
			spi_gd32_signal_completion(dev, 0);
			return;
		}

		spi_gd32_update_interrupt_mask(cfg, data);

		if (!progressed) {
			return;
		}
	}
}
#endif

static int spi_gd32_start_transfer(const struct device *dev)
{
	struct spi_gd32_data *data = dev->data;
	const struct spi_gd32_config *cfg = dev->config;
	bool slave = spi_gd32_is_slave(data);
#ifdef CONFIG_SPI_GD32_DMA
	bool dma = data->xfer_mode == SPI_GD32_XFER_DMA;
	int ret;
#endif

	spi_gd32_stop_transfer_engine(dev);
	spi_gd32_flush_stale_status(cfg);
	data->transfer_active = true;

#ifdef CONFIG_SPI_GD32_DMA
	if (dma) {
		ret = spi_gd32_dma_prepare_chunk(dev);
		if (ret != 0) {
			data->transfer_active = false;
			return ret;
		}
	}
#endif

	SPI_CTL0(cfg->reg) |= SPI_CTL0_SPIEN;

	if (!slave) {
		spi_context_cs_control(&data->ctx, true);
	}

#ifdef CONFIG_SPI_GD32_DMA
	if (dma) {
		ret = spi_gd32_dma_start_chunk(dev);
		if (ret != 0) {
			data->transfer_active = false;
			return ret;
		}

		return 0;
	}
#endif

	if (slave && !spi_gd32_is_rx_only(data)) {
		/* In slave mode the first frame must be queued after SPIEN is set and before the
		 * master starts clocking data.
		 */
		while (((spi_gd32_stat(cfg) & SPI_STAT_TBE) != 0U) &&
		       spi_gd32_can_write_frame(data)) {
			spi_gd32_write_frame(dev);
		}
	}

#ifdef CONFIG_SPI_GD32_INTERRUPT
	if (data->xfer_mode == SPI_GD32_XFER_INTERRUPT) {
		return spi_gd32_start_interrupt_transfer(dev);
	}
#endif

	return 0;
}

static int spi_gd32_transceive_impl(const struct device *dev, const struct spi_config *config,
				    const struct spi_buf_set *tx_bufs,
				    const struct spi_buf_set *rx_bufs, spi_callback_t cb,
				    void *userdata)
{
	struct spi_gd32_data *data = dev->data;
	int ret;

	if ((tx_bufs == NULL) && (rx_bufs == NULL)) {
		return 0;
	}

	spi_context_lock(&data->ctx, (cb != NULL), cb, userdata, config);

	ret = spi_gd32_configure(dev, config);
	if (ret != 0) {
		goto out_release;
	}

	ret = spi_gd32_validate_buffers(tx_bufs,
					SPI_WORD_SIZE_GET(config->operation) > 8U ? 2U : 1U);
	if (ret != 0) {
		goto out_release;
	}

	ret = spi_gd32_validate_buffers(rx_bufs,
					SPI_WORD_SIZE_GET(config->operation) > 8U ? 2U : 1U);
	if (ret != 0) {
		goto out_release;
	}

	spi_context_buffers_setup(&data->ctx, tx_bufs, rx_bufs,
				  SPI_WORD_SIZE_GET(config->operation) > 8U ? 2U : 1U);
	spi_gd32_prepare_transfer(dev);

	if (data->frames_left == 0U) {
		ret = 0;
		goto out_release;
	}

	data->xfer_mode = spi_gd32_select_xfer_mode(dev, tx_bufs, rx_bufs);

#ifdef CONFIG_SPI_ASYNC
	if (data->ctx.asynchronous && (data->xfer_mode == SPI_GD32_XFER_POLLING)) {
		ret = -ENOTSUP;
		goto out_release;
	}
#endif

	ret = spi_gd32_start_transfer(dev);
	if (ret != 0) {
		ret = spi_gd32_cleanup_transfer(dev, ret);
		goto out_release;
	}

	if (data->xfer_mode == SPI_GD32_XFER_POLLING) {
		int cleanup_ret;

		ret = spi_gd32_polling_transfer(dev);
		data->transfer_active = false;
		cleanup_ret = spi_gd32_cleanup_transfer(dev, ret);
		if (cleanup_ret != 0) {
			ret = cleanup_ret;
		}
#ifdef CONFIG_SPI_SLAVE
		else if ((ret == 0) && spi_gd32_is_slave(data)) {
			ret = data->ctx.recv_frames;
		}
#endif
		goto out_release;
	}

#ifdef CONFIG_SPI_ASYNC
	if (data->ctx.asynchronous) {
		return 0;
	}
#endif

#if defined(CONFIG_SPI_GD32_INTERRUPT) || defined(CONFIG_SPI_GD32_DMA)
	int cleanup_ret;

	ret = spi_gd32_wait_for_completion(dev);

	if ((ret == -ETIMEDOUT) && spi_gd32_mark_transfer_inactive(data)) {
		spi_gd32_latch_fatal(data, ret);
		spi_gd32_stop_transfer_engine(dev);
	}

	cleanup_ret = spi_gd32_cleanup_transfer(dev, ret);
	if (cleanup_ret != 0) {
		ret = cleanup_ret;
	}
#endif

out_release:
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

static int spi_gd32_release(const struct device *dev, const struct spi_config *config)
{
	struct spi_gd32_data *data = dev->data;
	const struct spi_gd32_config *cfg = dev->config;

	ARG_UNUSED(config);

	spi_gd32_stop_transfer_engine(dev);
	if (!spi_gd32_is_slave(data)) {
		spi_context_cs_control(&data->ctx, false);
	}
	SPI_CTL0(cfg->reg) &= ~SPI_CTL0_SPIEN;
	spi_gd32_flush_stale_status(cfg);
	spi_context_unlock_unconditionally(&data->ctx);

	return 0;
}

static DEVICE_API(spi, spi_gd32_driver_api) = {
	.transceive = spi_gd32_transceive,
#ifdef CONFIG_SPI_ASYNC
	.transceive_async = spi_gd32_transceive_async,
#endif
#ifdef CONFIG_SPI_RTIO
	.iodev_submit = spi_rtio_iodev_default_submit,
#endif
	.release = spi_gd32_release,
};

static int spi_gd32_init(const struct device *dev)
{
	struct spi_gd32_data *data = dev->data;
	const struct spi_gd32_config *cfg = dev->config;
	int ret;
#ifdef CONFIG_SPI_GD32_DMA
	uint32_t ch_filter;
#endif

	ret = clock_control_on(GD32_CLOCK_CONTROLLER, (clock_control_subsys_t)&cfg->clkid);
	if (ret != 0) {
		LOG_ERR("Failed to enable SPI clock");
		return ret;
	}

	ret = reset_line_toggle_dt(&cfg->reset);
	if (ret != 0) {
		LOG_ERR("Failed to reset SPI peripheral");
		return ret;
	}

	ret = pinctrl_apply_state(cfg->pcfg, PINCTRL_STATE_DEFAULT);
	if (ret != 0) {
		LOG_ERR("Failed to apply pinctrl state");
		return ret;
	}

#ifdef CONFIG_SPI_GD32_DMA
	if ((cfg->dma[RX].dev == NULL) != (cfg->dma[TX].dev == NULL)) {
		LOG_ERR("DMA must be provided for both RX and TX");
		return -ENODEV;
	}

	if (spi_gd32_dma_enabled(dev)) {
		for (size_t i = 0; i < NUM_OF_DIRECTION; i++) {
			if (!device_is_ready(cfg->dma[i].dev)) {
				LOG_ERR("DMA controller %s not ready", cfg->dma[i].dev->name);
				return -ENODEV;
			}

			ch_filter = BIT(cfg->dma[i].channel);
			ret = dma_request_channel(cfg->dma[i].dev, &ch_filter);
			if (ret < 0) {
				LOG_ERR("Failed to request DMA channel %u", cfg->dma[i].channel);
				return ret;
			}
		}
	}
#endif

	ret = spi_context_cs_configure_all(&data->ctx);
	if (ret != 0) {
		return ret;
	}

#ifdef CONFIG_SPI_GD32_INTERRUPT
	cfg->irq_configure(dev);
#endif

	spi_gd32_disable_interrupts(cfg);
	spi_gd32_disable_dma_requests(cfg);
	SPI_CTL0(cfg->reg) &= ~SPI_CTL0_SPIEN;
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
	static const struct spi_gd32_config spi_gd32_config_##idx = {                              \
		.reg = DT_INST_REG_ADDR(idx),                                                      \
		.clkid = DT_INST_CLOCKS_CELL(idx, id),                                             \
		.reset = RESET_DT_SPEC_INST_GET(idx),                                              \
		.pcfg = PINCTRL_DT_INST_DEV_CONFIG_GET(idx),                                       \
		IF_ENABLED(CONFIG_SPI_GD32_DMA, (.dma = DMAS_DECL(idx),))                                                                         \
				    IF_ENABLED(CONFIG_SPI_GD32_INTERRUPT,			       \
			   (.irq_configure = spi_gd32_irq_configure_## idx,)) };  \
	SPI_DEVICE_DT_INST_DEFINE(idx, spi_gd32_init, NULL, &spi_gd32_data_##idx,                  \
				  &spi_gd32_config_##idx, POST_KERNEL, CONFIG_SPI_INIT_PRIORITY,   \
				  &spi_gd32_driver_api);

DT_INST_FOREACH_STATUS_OKAY(GD32_SPI_INIT)
