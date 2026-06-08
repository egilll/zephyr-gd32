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

/* TXURERR is I2S-only on GD32 SPI; we never program I2S mode so leave it out
 * to avoid spuriously aborting transfers.
 */
#define SPI_GD32_ERR_MASK              (SPI_STAT_RXORERR | SPI_STAT_CONFERR | \
					SPI_STAT_CRCERR | SPI_STAT_FERR)
#define SPI_GD32_PSC_MAX               0x7U
/* The peripheral is single-buffered; at most one frame may sit in the TX
 * register while a second is shifting, otherwise the next frame to land in
 * RX overruns.
 */
#define SPI_GD32_INFLIGHT_MAX          2U
#define SPI_GD32_INIT_MASK             0x00003040U
#define SPI_GD32_FLUSH_MAX_ITERATIONS  32U

enum spi_gd32_xfer_mode {
	SPI_GD32_XFER_POLLING,
	SPI_GD32_XFER_INTERRUPT,
	SPI_GD32_XFER_DMA,
};

#ifdef CONFIG_SPI_GD32_DMA
enum spi_gd32_dma_direction {
	RX = 0,
	TX,
	NUM_OF_DIRECTION,
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
#endif

#if defined(CONFIG_SPI_ASYNC) || defined(CONFIG_SPI_RTIO)
struct spi_gd32_async_state {
	struct k_work completion_work;
	struct k_work_delayable timeout_work;
	int completion_status;
	bool completion_suppressed;
	bool progress_seen;
};
#endif

struct spi_gd32_stats {
	uint32_t timeouts;
	uint32_t errors;
	uint32_t resets;
	uint32_t dma_inferred;
	uint32_t dma_timeout_recoveries;
	uint32_t last_detail;
	int last_status;
};

struct spi_gd32_config {
	uint32_t reg;
	uint16_t clkid;
	struct reset_dt_spec reset;
	const struct pinctrl_dev_config *pcfg;
#ifdef CONFIG_SPI_GD32_DMA
	const struct spi_gd32_dma_config dma[NUM_OF_DIRECTION];
	uint32_t *dummy_tx;
	uint32_t *dummy_rx;
#endif
#ifdef CONFIG_SPI_GD32_INTERRUPT
	void (*irq_configure)(const struct device *dev);
#endif
};

struct spi_gd32_data {
	const struct device *dev;
	struct spi_context ctx;
	uint8_t dfs;
	size_t frames_left;
	size_t rx_pending;
	bool transfer_tx;
	bool transfer_rx;
	bool transfer_active;
	enum spi_gd32_xfer_mode xfer_mode;
#ifdef CONFIG_SPI_GD32_DMA
	struct spi_gd32_dma_data dma[NUM_OF_DIRECTION];
	bool dma_done[NUM_OF_DIRECTION];
	size_t dma_chunk_len;
#endif
	struct spi_gd32_stats stats;
#if defined(CONFIG_SPI_ASYNC) || defined(CONFIG_SPI_RTIO)
	struct spi_gd32_async_state async;
#endif
#ifdef CONFIG_SPI_RTIO
	struct spi_rtio *rtio_ctx;
	struct k_work rtio_work;
	bool rtio_active;
#endif
};

#ifdef CONFIG_SPI_RTIO
static void spi_gd32_iodev_start(const struct device *dev);
static void spi_gd32_iodev_complete(const struct device *dev, int status);
#endif
#ifdef CONFIG_SPI_ASYNC
static void spi_gd32_complete_async(const struct device *dev, int status);
#endif
#ifdef CONFIG_SPI_GD32_DMA
static int spi_gd32_dma_prepare_chunk(const struct device *dev);
static int spi_gd32_dma_start_chunk(const struct device *dev);
static int spi_gd32_dma_recover_completion(const struct device *dev, bool timeout_path);
#endif

static inline uint32_t spi_gd32_stat(const struct spi_gd32_config *cfg)
{
	return SPI_STAT(cfg->reg);
}

static inline uint8_t spi_gd32_dfs_from_op(uint16_t operation)
{
	return SPI_WORD_SIZE_GET(operation) > 8U ? 2U : 1U;
}

static inline bool spi_gd32_is_slave_op(uint16_t operation)
{
	return SPI_OP_MODE_GET(operation) == SPI_OP_MODE_SLAVE;
}

static inline bool spi_gd32_is_slave(const struct spi_gd32_data *data)
{
	return data->ctx.config != NULL && spi_gd32_is_slave_op(data->ctx.config->operation);
}

static inline bool spi_gd32_is_rx_only(const struct spi_gd32_data *data)
{
	return data->transfer_rx && !data->transfer_tx;
}

static inline bool spi_gd32_transfer_done(const struct spi_gd32_data *data)
{
	if (spi_gd32_is_slave(data) && spi_gd32_is_rx_only(data)) {
		return data->frames_left == 0U;
	}
	return data->frames_left == 0U && data->rx_pending == 0U;
}

static inline bool spi_gd32_can_write_frame(const struct spi_gd32_data *data)
{
	return data->frames_left != 0U && data->rx_pending < SPI_GD32_INFLIGHT_MAX;
}

static inline bool spi_gd32_keep_enabled(const struct spi_gd32_data *data)
{
	if (data->ctx.config == NULL || spi_gd32_is_slave(data)) {
		return false;
	}
	return (data->ctx.config->operation & SPI_HOLD_ON_CS) != 0U;
}

static void spi_gd32_note_fault(const struct device *dev, int status, uint32_t detail,
				const char *reason)
{
	const struct spi_gd32_config *cfg = dev->config;
	struct spi_gd32_data *data = dev->data;
	uint32_t stat = spi_gd32_stat(cfg);
	uint32_t ctl1 = SPI_CTL1(cfg->reg);
	static const char *const mode_str[] = {
		[SPI_GD32_XFER_POLLING] = "poll",
		[SPI_GD32_XFER_INTERRUPT] = "irq",
		[SPI_GD32_XFER_DMA] = "dma",
	};

	if (status == -ETIMEDOUT) {
		data->stats.timeouts++;
	} else {
		data->stats.errors++;
	}
	data->stats.last_status = status;
	data->stats.last_detail = detail;

	LOG_WRN_RATELIMIT_RATE(
		5000,
		"%s %s: status=%d detail=0x%08x mode=%s stat=0x%04x ctl1=0x%04x "
		"frames_left=%u rx_pending=%u timeouts=%u errors=%u resets=%u "
		"dma_inferred=%u dma_timeout_recoveries=%u",
		dev->name, reason, status, detail, mode_str[data->xfer_mode], stat, ctl1,
		(unsigned int)data->frames_left, (unsigned int)data->rx_pending,
		data->stats.timeouts, data->stats.errors, data->stats.resets,
		data->stats.dma_inferred, data->stats.dma_timeout_recoveries);
}

static void spi_gd32_clear_errors(const struct spi_gd32_config *cfg, uint32_t stat)
{
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
		SPI_STAT(cfg->reg) = stat & ~SPI_STAT_CRCERR;
	}
	if ((stat & SPI_STAT_FERR) != 0U) {
		SPI_STAT(cfg->reg) = stat & ~SPI_STAT_FERR;
	}
}

static void spi_gd32_flush_stale_status(const struct spi_gd32_config *cfg)
{
	for (unsigned int i = 0U; i < SPI_GD32_FLUSH_MAX_ITERATIONS; i++) {
		uint32_t stat = spi_gd32_stat(cfg);

		while ((stat & SPI_STAT_RBNE) != 0U) {
			(void)SPI_DATA(cfg->reg);
			stat = spi_gd32_stat(cfg);
		}
		if ((stat & SPI_GD32_ERR_MASK) != 0U) {
			spi_gd32_clear_errors(cfg, stat);
		}
		stat = spi_gd32_stat(cfg);
		if ((stat & (SPI_STAT_RBNE | SPI_GD32_ERR_MASK)) == 0U) {
			return;
		}
	}

	LOG_WRN_RATELIMIT_RATE(5000, "spi_gd32: flush_stale_status did not converge");
}

static void spi_gd32_disable_interrupts(const struct spi_gd32_config *cfg)
{
	SPI_CTL1(cfg->reg) &= ~(SPI_CTL1_RBNEIE | SPI_CTL1_TBEIE | SPI_CTL1_ERRIE);
}

#ifdef CONFIG_SPI_GD32_DMA
static void spi_gd32_disable_dma_requests(const struct spi_gd32_config *cfg)
{
	SPI_CTL1(cfg->reg) &= ~(SPI_CTL1_DMATEN | SPI_CTL1_DMAREN);
}
#else
static inline void spi_gd32_disable_dma_requests(const struct spi_gd32_config *cfg)
{
	ARG_UNUSED(cfg);
}
#endif

static void spi_gd32_stop_transfer_engine(const struct device *dev)
{
	const struct spi_gd32_config *cfg = dev->config;

	spi_gd32_disable_interrupts(cfg);
	spi_gd32_disable_dma_requests(cfg);

#ifdef CONFIG_SPI_GD32_DMA
	struct spi_gd32_data *data = dev->data;

	if (data->xfer_mode == SPI_GD32_XFER_DMA) {
		for (size_t i = 0; i < NUM_OF_DIRECTION; i++) {
			if (cfg->dma[i].dev != NULL) {
				(void)dma_stop(cfg->dma[i].dev, cfg->dma[i].channel);
			}
		}
	}
#endif
}

static void spi_gd32_reset_peripheral(const struct device *dev)
{
	const struct spi_gd32_config *cfg = dev->config;
	int ret = reset_line_toggle_dt(&cfg->reset);

	if (ret != 0) {
		LOG_ERR("%s failed to reset SPI peripheral (%d)", dev->name, ret);
	}
}

static void spi_gd32_force_recover(const struct device *dev, bool reset_peripheral)
{
	const struct spi_gd32_config *cfg = dev->config;
	struct spi_gd32_data *data = dev->data;

	spi_gd32_stop_transfer_engine(dev);
	if (!spi_gd32_is_slave(data)) {
		spi_context_cs_control(&data->ctx, false);
	}
	SPI_CTL0(cfg->reg) &= ~SPI_CTL0_SPIEN;
	spi_gd32_flush_stale_status(cfg);

	if (reset_peripheral) {
		spi_gd32_reset_peripheral(dev);
		data->stats.resets++;
		spi_gd32_flush_stale_status(cfg);
	}
}

#ifdef CONFIG_SPI_GD32_INTERRUPT
static void spi_gd32_update_interrupt_mask(const struct spi_gd32_config *cfg,
					    const struct spi_gd32_data *data)
{
	uint32_t ctl1 = SPI_CTL1(cfg->reg);
	bool slave_rx_only = spi_gd32_is_slave(data) && spi_gd32_is_rx_only(data);

	ctl1 &= ~(SPI_CTL1_ERRIE | SPI_CTL1_RBNEIE | SPI_CTL1_TBEIE);
	ctl1 |= SPI_CTL1_ERRIE;

	if (data->rx_pending != 0U || data->transfer_rx) {
		ctl1 |= SPI_CTL1_RBNEIE;
	}
	/* Slave RX-only never writes — the master clocks data in. Master
	 * RX-only still has to write zeros to generate the clock, so TBEIE
	 * must stay enabled there.
	 */
	if (spi_gd32_can_write_frame(data) && !slave_rx_only) {
		ctl1 |= SPI_CTL1_TBEIE;
	}

	SPI_CTL1(cfg->reg) = ctl1;
}
#endif

static uint32_t spi_gd32_frame_timeout_us(const struct spi_gd32_data *data, size_t frames)
{
	if (spi_gd32_is_slave(data)) {
		return MAX((uint32_t)CONFIG_SPI_COMPLETION_TIMEOUT_TOLERANCE * USEC_PER_MSEC, 1U);
	}

	uint32_t bits = SPI_WORD_SIZE_GET(data->ctx.config->operation);
	uint64_t us = (uint64_t)MAX(frames, 1U) * bits * USEC_PER_SEC;

	us = DIV_ROUND_UP(us, data->ctx.config->frequency);
	us += (uint64_t)CONFIG_SPI_COMPLETION_TIMEOUT_TOLERANCE * USEC_PER_MSEC;

	return (uint32_t)MIN(us, (uint64_t)UINT32_MAX);
}

#if defined(CONFIG_SPI_GD32_INTERRUPT) || defined(CONFIG_SPI_GD32_DMA) || \
	defined(CONFIG_SPI_ASYNC) || defined(CONFIG_SPI_RTIO)
static k_timeout_t spi_gd32_completion_timeout(const struct spi_gd32_data *data)
{
	return K_USEC(spi_gd32_frame_timeout_us(data, MAX(data->frames_left, 2U)));
}
#endif

static uint32_t spi_gd32_spin_iters(const struct spi_gd32_data *data)
{
	uint64_t budget = (uint64_t)spi_gd32_frame_timeout_us(data, 2U) * 64U;

	return (uint32_t)CLAMP(budget, 1000ULL, (uint64_t)UINT32_MAX);
}

static int spi_gd32_wait_for_status(const struct device *dev, uint32_t flags, uint32_t *stat_out)
{
	const struct spi_gd32_config *cfg = dev->config;
	struct spi_gd32_data *data = dev->data;
	uint32_t iters = spi_gd32_spin_iters(data);
	uint32_t stat;

	while (iters-- != 0U) {
		stat = spi_gd32_stat(cfg);
		if ((stat & SPI_GD32_ERR_MASK) != 0U) {
			spi_gd32_note_fault(dev, -EIO, stat & SPI_GD32_ERR_MASK,
					    "error while polling status");
			spi_gd32_clear_errors(cfg, stat);
			return -EIO;
		}
		if ((stat & flags) != 0U) {
			if (stat_out != NULL) {
				*stat_out = stat;
			}
			return 0;
		}
	}

	spi_gd32_note_fault(dev, -ETIMEDOUT, flags, "timeout polling status");
	return -ETIMEDOUT;
}

static int spi_gd32_wait_until_idle(const struct device *dev)
{
	const struct spi_gd32_config *cfg = dev->config;
	struct spi_gd32_data *data = dev->data;
	uint32_t iters = spi_gd32_spin_iters(data);

	while (iters-- != 0U) {
		uint32_t stat = spi_gd32_stat(cfg);

		if ((stat & SPI_GD32_ERR_MASK) != 0U) {
			spi_gd32_note_fault(dev, -EIO, stat & SPI_GD32_ERR_MASK,
					    "error while waiting for idle");
			spi_gd32_clear_errors(cfg, stat);
			return -EIO;
		}
		if ((stat & SPI_STAT_TBE) != 0U && (stat & SPI_STAT_TRANS) == 0U) {
			return 0;
		}
	}

	spi_gd32_note_fault(dev, -ETIMEDOUT, spi_gd32_stat(cfg), "timeout waiting for idle");
	return -ETIMEDOUT;
}

#if defined(CONFIG_SPI_GD32_INTERRUPT) || defined(CONFIG_SPI_GD32_DMA) || \
	defined(CONFIG_SPI_ASYNC) || defined(CONFIG_SPI_RTIO)
static bool spi_gd32_mark_transfer_inactive(struct spi_gd32_data *data)
{
	unsigned int key = irq_lock();
	bool was_active = data->transfer_active;

	data->transfer_active = false;
	irq_unlock(key);

	return was_active;
}
#endif

#if defined(CONFIG_SPI_ASYNC) || defined(CONFIG_SPI_RTIO)
static bool spi_gd32_has_deferred_completion(const struct spi_gd32_data *data)
{
#ifdef CONFIG_SPI_RTIO
	if (data->rtio_active) {
		return true;
	}
#endif
#ifdef CONFIG_SPI_ASYNC
	return data->ctx.asynchronous;
#else
	return false;
#endif
}

static void spi_gd32_disarm_timeout(struct spi_gd32_data *data)
{
	if (spi_gd32_has_deferred_completion(data)) {
		(void)k_work_cancel_delayable(&data->async.timeout_work);
	}
}

static void spi_gd32_refresh_timeout(const struct device *dev)
{
	struct spi_gd32_data *data = dev->data;

	if (!data->transfer_active || !spi_gd32_has_deferred_completion(data)) {
		return;
	}
	(void)k_work_reschedule(&data->async.timeout_work, spi_gd32_completion_timeout(data));
}

static void spi_gd32_arm_timeout_on_start(const struct device *dev)
{
	struct spi_gd32_data *data = dev->data;

	if (spi_gd32_is_slave(data)) {
		return;
	}
	spi_gd32_refresh_timeout(dev);
}

static void spi_gd32_note_progress(const struct device *dev)
{
	struct spi_gd32_data *data = dev->data;

	data->async.progress_seen = true;
	spi_gd32_refresh_timeout(dev);
}

static void spi_gd32_queue_completion_work(const struct device *dev, int status)
{
	struct spi_gd32_data *data = dev->data;

	data->async.completion_status = status;
	(void)k_work_submit(&data->async.completion_work);
}

static void spi_gd32_completion_work_handler(struct k_work *work)
{
	struct spi_gd32_data *data = CONTAINER_OF(work, struct spi_gd32_data, async.completion_work);
	const struct device *dev = data->dev;
	int status = data->async.completion_status;

	if (data->async.completion_suppressed || data->ctx.config == NULL) {
		return;
	}

#ifdef CONFIG_SPI_RTIO
	if (data->rtio_active) {
		spi_gd32_iodev_complete(dev, status);
		return;
	}
#endif
#ifdef CONFIG_SPI_ASYNC
	if (data->ctx.asynchronous) {
		spi_gd32_complete_async(dev, status);
	}
#endif
}

static void spi_gd32_timeout_work_handler(struct k_work *work)
{
	struct k_work_delayable *dwork = k_work_delayable_from_work(work);
	struct spi_gd32_data *data = CONTAINER_OF(dwork, struct spi_gd32_data, async.timeout_work);
	const struct device *dev = data->dev;
	const struct spi_gd32_config *cfg = dev->config;

	if (!data->transfer_active) {
		return;
	}

	if (spi_gd32_is_slave(data) && !data->async.progress_seen) {
		spi_gd32_refresh_timeout(dev);
		return;
	}
#ifdef CONFIG_SPI_GD32_DMA
	if (spi_gd32_dma_recover_completion(dev, true) > 0) {
		return;
	}
#endif

	if (!spi_gd32_mark_transfer_inactive(data)) {
		return;
	}
	spi_gd32_stop_transfer_engine(dev);

	spi_gd32_note_fault(dev, -ETIMEDOUT, spi_gd32_stat(cfg),
			    "timeout waiting for async transfer progress");
	spi_gd32_queue_completion_work(dev, -ETIMEDOUT);
}
#endif

#if defined(CONFIG_SPI_GD32_INTERRUPT) || defined(CONFIG_SPI_GD32_DMA)
/* Central completion path for IRQ/DMA contexts. Stops the engine *before*
 * deferring user-visible completion so we cannot IRQ-storm in the gap
 * before the work queue runs cleanup.
 */
static void spi_gd32_signal_completion(const struct device *dev, int status)
{
	struct spi_gd32_data *data = dev->data;

	if (!spi_gd32_mark_transfer_inactive(data)) {
		return;
	}

	spi_gd32_stop_transfer_engine(dev);

#if defined(CONFIG_SPI_ASYNC) || defined(CONFIG_SPI_RTIO)
	spi_gd32_disarm_timeout(data);
	if (spi_gd32_has_deferred_completion(data)) {
		spi_gd32_queue_completion_work(dev, status);
		return;
	}
#endif

	spi_context_complete(&data->ctx, dev, status);
}
#endif

static void spi_gd32_write_frame(const struct device *dev)
{
	struct spi_gd32_data *data = dev->data;
	const struct spi_gd32_config *cfg = dev->config;
	struct spi_context *ctx = &data->ctx;
	uint16_t frame = 0U;

	if (spi_context_tx_buf_on(ctx)) {
		frame = (data->dfs == 1U) ? UNALIGNED_GET((uint8_t *)ctx->tx_buf)
					  : UNALIGNED_GET((uint16_t *)ctx->tx_buf);
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
	uint16_t frame = (uint16_t)SPI_DATA(cfg->reg);

	if (spi_context_rx_buf_on(ctx)) {
		if (data->dfs == 1U) {
			UNALIGNED_PUT((uint8_t)frame, (uint8_t *)ctx->rx_buf);
		} else {
			UNALIGNED_PUT(frame, (uint16_t *)ctx->rx_buf);
		}
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
		return;
	}
	if (data->rx_pending != 0U && data->transfer_rx && spi_context_rx_on(ctx)) {
		spi_gd32_read_frame(dev);
		return;
	}
	spi_gd32_drop_frame(dev);
}

static bool spi_gd32_shift_slave(const struct device *dev, uint32_t *stat)
{
	struct spi_gd32_data *data = dev->data;
	const struct spi_gd32_config *cfg = dev->config;
	bool progressed = false;
	uint32_t outer_iters = spi_gd32_spin_iters(data);

	while (outer_iters-- != 0U) {
		bool worked = false;

		if (!spi_gd32_is_rx_only(data)) {
			uint32_t tx_iters = spi_gd32_spin_iters(data);
			while ((*stat & SPI_STAT_TBE) != 0U && spi_gd32_can_write_frame(data)) {
				if (tx_iters-- == 0U) {
					return progressed;
				}
				spi_gd32_write_frame(dev);
				worked = true;
				*stat = spi_gd32_stat(cfg);
			}
		}
		uint32_t rx_iters = spi_gd32_spin_iters(data);
		while ((*stat & SPI_STAT_RBNE) != 0U) {
			if (rx_iters-- == 0U) {
				return progressed || worked;
			}
			spi_gd32_handle_slave_rx_frame(dev);
			worked = true;
			*stat = spi_gd32_stat(cfg);
		}

		progressed = progressed || worked;
		if (!worked) {
			return progressed;
		}
	}

	return progressed;
}

static bool spi_gd32_shift_master(const struct device *dev, uint32_t *stat)
{
	struct spi_gd32_data *data = dev->data;
	const struct spi_gd32_config *cfg = dev->config;
	bool progressed = false;

	while ((*stat & SPI_STAT_RBNE) != 0U) {
		if (data->rx_pending != 0U) {
			spi_gd32_read_frame(dev);
		} else {
			spi_gd32_drop_frame(dev);
		}
		progressed = true;
		*stat = spi_gd32_stat(cfg);
	}
	while ((*stat & SPI_STAT_TBE) != 0U && spi_gd32_can_write_frame(data)) {
		spi_gd32_write_frame(dev);
		progressed = true;
		*stat = spi_gd32_stat(cfg);
	}
	return progressed;
}

static bool spi_gd32_shift(const struct device *dev, uint32_t *stat)
{
	struct spi_gd32_data *data = dev->data;

	return spi_gd32_is_slave(data) ? spi_gd32_shift_slave(dev, stat)
				       : spi_gd32_shift_master(dev, stat);
}

/* Drain whatever frames are sitting in the RX register before clearing the
 * RXORERR flag, so an in-flight overrun does not cost us live data. Returns
 * true if recovery succeeded, false to abort the transfer.
 */
static bool spi_gd32_recover_slave_overrun(const struct device *dev, uint32_t stat)
{
	struct spi_gd32_data *data = dev->data;
	const struct spi_gd32_config *cfg = dev->config;
	uint32_t errs = stat & SPI_GD32_ERR_MASK;
	bool drained = false;

	if (!spi_gd32_is_slave(data)) {
		return false;
	}
	if ((errs & ~(SPI_STAT_RXORERR | SPI_STAT_FERR)) != 0U) {
		return false;
	}
	if ((errs & SPI_STAT_RXORERR) == 0U) {
		return false;
	}

	uint32_t drain_iters = spi_gd32_spin_iters(data);

	while ((spi_gd32_stat(cfg) & SPI_STAT_RBNE) != 0U) {
		if (drain_iters-- == 0U) {
			spi_gd32_note_fault(dev, -EIO,
					    spi_gd32_stat(cfg),
					    "slave overrun drain exceeded budget");
			break;
		}
		spi_gd32_handle_slave_rx_frame(dev);
		drained = true;
	}

	(void)SPI_STAT(cfg->reg);

	if ((errs & SPI_STAT_FERR) != 0U) {
		SPI_STAT(cfg->reg) = spi_gd32_stat(cfg) & ~SPI_STAT_FERR;
	}

	return drained;
}

static int spi_gd32_polling_transfer(const struct device *dev)
{
	const struct spi_gd32_config *cfg = dev->config;
	struct spi_gd32_data *data = dev->data;

	/* Hard wall-clock deadline so this loop cannot pin the CPU even if
	 * the controller wedges in a way the inner timeouts miss. Budget is
	 * (bit-time * frames) + tolerance, with a floor so very small
	 * transfers still get a reasonable bail-out window.
	 */
	const uint32_t total_frames = MAX(data->frames_left + data->rx_pending, 2U);
	uint64_t deadline_us = spi_gd32_frame_timeout_us(data, total_frames);

	if (deadline_us < 2000ULL) {
		deadline_us = 2000ULL;
	}
	const int64_t deadline_ticks = k_uptime_ticks() +
				       (int64_t)k_us_to_ticks_ceil64(deadline_us);

	uint32_t last_frames_left = data->frames_left;
	uint32_t last_rx_pending = data->rx_pending;
	uint32_t stalled_iters = 0U;
	uint32_t loop_iters = 0U;

	while (!spi_gd32_transfer_done(data)) {
		if (k_uptime_ticks() >= deadline_ticks) {
			spi_gd32_note_fault(dev, -ETIMEDOUT, spi_gd32_stat(cfg),
					    "polling transfer wall-clock deadline exceeded");
			spi_gd32_force_recover(dev, true);
			return -ETIMEDOUT;
		}

		uint32_t stat = spi_gd32_stat(cfg);

		if ((stat & SPI_GD32_ERR_MASK) != 0U) {
			if (spi_gd32_recover_slave_overrun(dev, stat)) {
				continue;
			}
			spi_gd32_note_fault(dev, -EIO, stat & SPI_GD32_ERR_MASK,
					    "polling saw SPI error status");
			spi_gd32_clear_errors(cfg, stat);
			spi_gd32_force_recover(dev, false);
			return -EIO;
		}

		if (spi_gd32_shift(dev, &stat)) {
			last_frames_left = data->frames_left;
			last_rx_pending = data->rx_pending;
			stalled_iters = 0U;
			continue;
		}

		uint32_t wait_for = spi_gd32_is_rx_only(data)
					    ? SPI_STAT_RBNE
					    : (SPI_STAT_TBE | SPI_STAT_RBNE);
		int ret = spi_gd32_wait_for_status(dev, wait_for, NULL);

		if (ret != 0) {
			spi_gd32_force_recover(dev, ret == -EIO);
			return ret;
		}

		/* wait_for_status returned 0 (flag observed) but the next
		 * shift made no progress: count it and bail out before we
		 * can busy-spin a higher-priority thread to death.
		 */
		if (data->frames_left == last_frames_left &&
		    data->rx_pending == last_rx_pending) {
			if (++stalled_iters >= 16U) {
				spi_gd32_note_fault(dev, -EIO,
						    spi_gd32_stat(cfg),
						    "polling transfer stalled without progress");
				spi_gd32_force_recover(dev, true);
				return -EIO;
			}
		} else {
			last_frames_left = data->frames_left;
			last_rx_pending = data->rx_pending;
			stalled_iters = 0U;
		}

		/* Cooperatively yield every so often so a high-priority
		 * caller polling SPI cannot starve other threads (most
		 * importantly the task watchdog feeders) on this CPU.
		 */
		if ((++loop_iters & 0x3FU) == 0U) {
			k_yield();
		}
	}

	return 0;
}

#if defined(CONFIG_SPI_GD32_INTERRUPT) || defined(CONFIG_SPI_GD32_DMA)
static int spi_gd32_wait_for_completion(const struct device *dev)
{
	struct spi_gd32_data *data = dev->data;
	struct spi_context *ctx = &data->ctx;

	if (!spi_gd32_is_slave(data)) {
		return spi_context_wait_for_completion(ctx);
	}

	size_t last_frames = data->frames_left;
	size_t last_pending = data->rx_pending;

	while (k_sem_take(&ctx->sync, spi_gd32_completion_timeout(data)) != 0) {
		unsigned int key = irq_lock();
		size_t frames = data->frames_left;
		size_t pending = data->rx_pending;

		irq_unlock(key);

		if (frames != last_frames || pending != last_pending) {
			last_frames = frames;
			last_pending = pending;
			continue;
		}
		spi_gd32_note_fault(dev, -ETIMEDOUT, 0U,
				    "timed out waiting for slave completion");
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

#ifdef CONFIG_SPI_GD32_DMA
static bool spi_gd32_dma_enabled(const struct device *dev)
{
	const struct spi_gd32_config *cfg = dev->config;

	return cfg->dma[TX].dev != NULL && cfg->dma[RX].dev != NULL;
}

static bool spi_gd32_dma_addr_ok(uintptr_t addr, size_t len)
{
#if IS_ENABLED(CONFIG_MEM_ATTR)
	if (IS_ENABLED(CONFIG_MMU) || len == 0U) {
		return true;
	}
	if (addr + len < addr) {
		return false;
	}

	int ret = mem_attr_check_buf((void *)addr, len, DT_MEM_DMA);

	return ret == 0 || ret == -ENOSYS || ret == -ENOBUFS;
#else
	ARG_UNUSED(addr);
	ARG_UNUSED(len);
	return true;
#endif
}

static bool spi_gd32_dma_bufs_usable(const struct spi_buf_set *bufs, uint8_t dfs)
{
	if (bufs == NULL) {
		return true;
	}
	for (size_t i = 0; i < bufs->count; i++) {
		const struct spi_buf *buf = &bufs->buffers[i];

		if (buf->buf == NULL || buf->len == 0U) {
			continue;
		}
		if (dfs == 2U && !IS_ALIGNED((uintptr_t)buf->buf, 2U)) {
			return false;
		}
		if (!spi_gd32_dma_addr_ok((uintptr_t)buf->buf, buf->len)) {
			return false;
		}
	}
	return true;
}

static bool spi_gd32_dma_possible(const struct device *dev, const struct spi_buf_set *tx_bufs,
				  const struct spi_buf_set *rx_bufs, uint8_t dfs)
{
	struct spi_gd32_data *data = dev->data;

	if (spi_gd32_is_slave(data) || !spi_gd32_dma_enabled(dev)) {
		return false;
	}
	return spi_gd32_dma_bufs_usable(tx_bufs, dfs) && spi_gd32_dma_bufs_usable(rx_bufs, dfs);
}

static void spi_gd32_dma_callback(const struct device *dma_dev, void *arg, uint32_t channel,
				  int status);

/**
 * DMA terminal callbacks are treated as wakeups, not the only completion
 * proof. On GD32 this SPI workload can reach NDT==0 on one channel without the
 * matching callback, so the SPI driver always re-checks hardware state before
 * advancing the transfer.
 */
static bool spi_gd32_dma_channel_complete(const struct spi_gd32_dma_config *dma, bool callback_done)
{
	struct dma_status status;

	if (callback_done) {
		return true;
	}
	if (dma_get_status(dma->dev, dma->channel, &status) != 0) {
		return false;
	}

	return status.pending_length == 0U;
}

/**
 * Fold callback observations and live DMA state into a single "both channels
 * are done" decision. Missing callbacks are counted silently here; user-visible
 * logging is reserved for the timeout path.
 */
static bool spi_gd32_dma_reconcile_completion(const struct device *dev)
{
	struct spi_gd32_data *data = dev->data;
	const struct spi_gd32_config *cfg = dev->config;
	bool done[NUM_OF_DIRECTION];
	bool inferred[NUM_OF_DIRECTION] = {false};
	unsigned int key;

	key = irq_lock();
	if (!data->transfer_active) {
		irq_unlock(key);
		return false;
	}
	done[RX] = data->dma_done[RX];
	done[TX] = data->dma_done[TX];
	irq_unlock(key);

	for (size_t dir = 0U; dir < NUM_OF_DIRECTION; dir++) {
		if (done[dir]) {
			continue;
		}

		done[dir] = spi_gd32_dma_channel_complete(&cfg->dma[dir], false);
		inferred[dir] = done[dir];
	}

	if (!done[RX] || !done[TX]) {
		return false;
	}

	key = irq_lock();
	if (!data->transfer_active) {
		irq_unlock(key);
		return false;
	}
	for (size_t dir = 0U; dir < NUM_OF_DIRECTION; dir++) {
		if (!data->dma_done[dir] && done[dir]) {
			data->dma_done[dir] = true;
		} else {
			inferred[dir] = false;
		}
	}
	irq_unlock(key);

	if (inferred[RX] || inferred[TX]) {
		data->stats.dma_inferred++;
	}

	return true;
}

/**
 * Once both DMA channels have consumed the current chunk, either queue the next
 * chunk immediately or complete the SPI transfer. The final bus-idle wait still
 * happens in the common cleanup path before CS is released.
 */
static int spi_gd32_dma_complete_chunk(const struct device *dev)
{
	const struct spi_gd32_config *cfg = dev->config;
	struct spi_gd32_data *data = dev->data;
	uint32_t errs = spi_gd32_stat(cfg) & SPI_GD32_ERR_MASK;
	int ret;

	if (errs != 0U) {
		spi_gd32_note_fault(dev, -EIO, errs, "DMA completion saw SPI error");
		spi_gd32_clear_errors(cfg, spi_gd32_stat(cfg));
		spi_gd32_signal_completion(dev, -EIO);
		return -EIO;
	}

	spi_gd32_disable_dma_requests(cfg);

	if (spi_context_tx_on(&data->ctx)) {
		spi_context_update_tx(&data->ctx, data->dfs, data->dma_chunk_len);
	}
	if (spi_context_rx_on(&data->ctx)) {
		spi_context_update_rx(&data->ctx, data->dfs, data->dma_chunk_len);
	}

	if (spi_context_max_continuous_chunk(&data->ctx) == 0U) {
		spi_gd32_signal_completion(dev, 0);
		return 0;
	}

	ret = spi_gd32_dma_prepare_chunk(dev);
	if (ret == 0) {
		ret = spi_gd32_dma_start_chunk(dev);
	}
	if (ret != 0) {
		spi_gd32_signal_completion(dev, ret);
	}

	return ret;
}

static int spi_gd32_dma_recover_completion(const struct device *dev, bool timeout_path)
{
	struct spi_gd32_data *data = dev->data;
	int ret;

	if (data->xfer_mode != SPI_GD32_XFER_DMA || spi_gd32_is_slave(data)) {
		return 0;
	}
	if (!spi_gd32_dma_reconcile_completion(dev)) {
		return 0;
	}

	if (timeout_path) {
		data->stats.dma_timeout_recoveries++;
		LOG_WRN_RATELIMIT_RATE(
			5000,
			"%s recovered DMA completion after timeout "
			"dma_inferred=%u dma_timeout_recoveries=%u",
			dev->name, data->stats.dma_inferred,
			data->stats.dma_timeout_recoveries);
	}

	ret = spi_gd32_dma_complete_chunk(dev);

#if defined(CONFIG_SPI_ASYNC) || defined(CONFIG_SPI_RTIO)
	if ((ret == 0) && data->transfer_active) {
		spi_gd32_note_progress(dev);
	}
#endif

	return ret == 0 ? 1 : ret;
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

	*dma_cfg = (struct dma_config){
		.source_burst_length = 1U,
		.dest_burst_length = 1U,
		.user_data = (void *)dev,
		.dma_callback = spi_gd32_dma_callback,
		.block_count = 1U,
		.head_block = block_cfg,
		.dma_slot = dma->slot,
		.channel_priority = GD32_DMA_CONFIG_PRIORITY(dma->config),
		.channel_direction = (dir == TX) ? MEMORY_TO_PERIPHERAL : PERIPHERAL_TO_MEMORY,
		.source_data_size = data->dfs,
		.dest_data_size = data->dfs,
	};

	*block_cfg = (struct dma_block_config){
		.block_size = chunk_len * data->dfs,
		.fifo_mode_control = dma->fifo_threshold,
	};

	if (dir == TX) {
		block_cfg->dest_address = (uint32_t)&SPI_DATA(cfg->reg);
		block_cfg->dest_addr_adj = DMA_ADDR_ADJ_NO_CHANGE;
		if (spi_context_tx_buf_on(ctx)) {
			block_cfg->source_address = (uint32_t)ctx->tx_buf;
			block_cfg->source_addr_adj = DMA_ADDR_ADJ_INCREMENT;
		} else {
			block_cfg->source_address = (uint32_t)cfg->dummy_tx;
			block_cfg->source_addr_adj = DMA_ADDR_ADJ_NO_CHANGE;
		}
	} else {
		block_cfg->source_address = (uint32_t)&SPI_DATA(cfg->reg);
		block_cfg->source_addr_adj = DMA_ADDR_ADJ_NO_CHANGE;
		if (spi_context_rx_buf_on(ctx)) {
			block_cfg->dest_address = (uint32_t)ctx->rx_buf;
			block_cfg->dest_addr_adj = DMA_ADDR_ADJ_INCREMENT;
		} else {
			block_cfg->dest_address = (uint32_t)cfg->dummy_rx;
			block_cfg->dest_addr_adj = DMA_ADDR_ADJ_NO_CHANGE;
		}
	}

	if (dma_config(dma->dev, dma->channel, dma_cfg) != 0) {
		LOG_ERR("%s failed to configure %s DMA channel", dev->name,
			dir == TX ? "TX" : "RX");
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
	const struct spi_gd32_config *cfg = dev->config;
	struct spi_context *ctx = &data->ctx;

	if (!spi_context_tx_buf_on(ctx)) {
		*cfg->dummy_tx = 0U;
		(void)sys_cache_data_flush_range(cfg->dummy_tx, sizeof(*cfg->dummy_tx));
	}
	if (!spi_context_rx_buf_on(ctx)) {
		(void)sys_cache_data_invd_range(cfg->dummy_rx, sizeof(*cfg->dummy_rx));
	}
#endif

	ret = spi_gd32_dma_setup(dev, RX, chunk_len);
	if (ret != 0) {
		return ret;
	}
	return spi_gd32_dma_setup(dev, TX, chunk_len);
}

static int spi_gd32_dma_start_chunk(const struct device *dev)
{
	const struct spi_gd32_config *cfg = dev->config;
	struct spi_gd32_data *data = dev->data;
	size_t chunk_len = spi_context_max_continuous_chunk(&data->ctx);

	if (chunk_len == 0U) {
		return -EIO;
	}

	spi_gd32_disable_dma_requests(cfg);
	data->dma_chunk_len = chunk_len;
	data->dma_done[RX] = false;
	data->dma_done[TX] = false;

	if (dma_start(cfg->dma[RX].dev, cfg->dma[RX].channel) != 0) {
		LOG_ERR("%s failed to start RX DMA channel", dev->name);
		return -EIO;
	}
	if (dma_start(cfg->dma[TX].dev, cfg->dma[TX].channel) != 0) {
		LOG_ERR("%s failed to start TX DMA channel", dev->name);
		(void)dma_stop(cfg->dma[RX].dev, cfg->dma[RX].channel);
		return -EIO;
	}

	SPI_CTL1(cfg->reg) |= SPI_CTL1_ERRIE;
	SPI_CTL1(cfg->reg) |= SPI_CTL1_DMAREN | SPI_CTL1_DMATEN;

	return 0;
}

static void spi_gd32_dma_callback(const struct device *dma_dev, void *arg, uint32_t channel,
				  int status)
{
	const struct device *dev = arg;
	const struct spi_gd32_config *cfg = dev->config;
	struct spi_gd32_data *data = dev->data;
	enum spi_gd32_dma_direction dir;
	bool both_done;

	if (status < 0) {
		spi_gd32_note_fault(dev, status, channel, "DMA transfer failed");
		spi_gd32_signal_completion(dev, status);
		return;
	}
	if (status == DMA_STATUS_BLOCK) {
#if defined(CONFIG_SPI_ASYNC) || defined(CONFIG_SPI_RTIO)
		spi_gd32_note_progress(dev);
#endif
		return;
	}

	if (dma_dev == cfg->dma[RX].dev && channel == cfg->dma[RX].channel) {
		dir = RX;
	} else if (dma_dev == cfg->dma[TX].dev && channel == cfg->dma[TX].channel) {
		dir = TX;
	} else {
		spi_gd32_note_fault(dev, -EIO, channel, "DMA callback from unexpected channel");
		spi_gd32_signal_completion(dev, -EIO);
		return;
	}

	unsigned int key = irq_lock();

	if (!data->transfer_active) {
		irq_unlock(key);
		return;
	}
	data->dma_done[dir] = true;
	both_done = data->dma_done[RX] && data->dma_done[TX];
	irq_unlock(key);

#if defined(CONFIG_SPI_ASYNC) || defined(CONFIG_SPI_RTIO)
	spi_gd32_note_progress(dev);
#endif
	if (!both_done) {
		both_done = spi_gd32_dma_reconcile_completion(dev);
	}
	if (both_done) {
		(void)spi_gd32_dma_complete_chunk(dev);
	}
}
#endif

#ifdef CONFIG_SPI_GD32_INTERRUPT
static void spi_gd32_isr(const struct device *dev)
{
	const struct spi_gd32_config *cfg = dev->config;
	struct spi_gd32_data *data = dev->data;

	if (!data->transfer_active) {
		/* Belt-and-suspenders: a pending IRQ after completion must
		 * not be able to storm. The flags are level-driven, so we
		 * must silence the IE bits on this path too.
		 */
		spi_gd32_disable_interrupts(cfg);
		return;
	}

	while (true) {
		uint32_t stat = spi_gd32_stat(cfg);
		uint32_t errs = stat & SPI_GD32_ERR_MASK;
		bool progressed;

		if (errs != 0U) {
			if (spi_gd32_recover_slave_overrun(dev, stat)) {
				spi_gd32_update_interrupt_mask(cfg, data);
				continue;
			}
			spi_gd32_note_fault(dev, -EIO, errs, "ISR saw SPI error status");
			spi_gd32_clear_errors(cfg, stat);
			spi_gd32_signal_completion(dev, -EIO);
			return;
		}

		progressed = spi_gd32_shift(dev, &stat);

		if (spi_gd32_transfer_done(data)) {
			spi_gd32_signal_completion(dev, 0);
			return;
		}

#if defined(CONFIG_SPI_ASYNC) || defined(CONFIG_SPI_RTIO)
		if (progressed) {
			spi_gd32_note_progress(dev);
		}
#endif
		spi_gd32_update_interrupt_mask(cfg, data);

		if (!progressed) {
			return;
		}
	}
}
#endif

static int spi_gd32_configure(const struct device *dev, const struct spi_config *config)
{
	struct spi_gd32_data *data = dev->data;
	const struct spi_gd32_config *cfg = dev->config;
	bool slave = spi_gd32_is_slave_op(config->operation);
	bool ti = (config->operation & SPI_FRAME_FORMAT_TI) != 0U;
	uint32_t prescaler = 0U;
	uint32_t bus_freq;
	int ret;

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
	    (config->operation & SPI_LINES_MASK) != SPI_LINES_SINGLE) {
		LOG_ERR("Only single-line SPI is supported");
		return -ENOTSUP;
	}
	if (SPI_WORD_SIZE_GET(config->operation) != 8U &&
	    SPI_WORD_SIZE_GET(config->operation) != 16U) {
		LOG_ERR("Only 8-bit and 16-bit words are supported");
		return -ENOTSUP;
	}
	if (ti && (config->operation & SPI_TRANSFER_LSB) != 0U) {
		LOG_ERR("TI frame format requires MSB-first transfers");
		return -ENOTSUP;
	}
	if (!spi_cs_is_gpio(config) && (config->operation & SPI_CS_ACTIVE_HIGH) != 0U) {
		LOG_ERR("Active-high native CS not supported");
		return -ENOTSUP;
	}
	if (slave && spi_cs_is_gpio(config)) {
		LOG_ERR("GPIO CS is not supported in slave mode");
		return -ENOTSUP;
	}
	if (ti && spi_cs_is_gpio(config)) {
		LOG_ERR("TI frame format requires native NSS");
		return -ENOTSUP;
	}
	if (!slave && config->frequency == 0U) {
		return -EINVAL;
	}

	if (!slave || config->frequency != 0U) {
		ret = clock_control_get_rate(GD32_CLOCK_CONTROLLER,
					     (clock_control_subsys_t)&cfg->clkid, &bus_freq);
		if (ret != 0) {
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

	uint32_t ctl0 = SPI_CTL0(cfg->reg) & SPI_GD32_INIT_MASK;
	uint32_t ctl1 = SPI_CTL1(cfg->reg) &
		~(SPI_CTL1_DMATEN | SPI_CTL1_DMAREN | SPI_CTL1_NSSDRV |
		  SPI_CTL1_TMOD | SPI_CTL1_ERRIE | SPI_CTL1_RBNEIE | SPI_CTL1_TBEIE);

	ctl0 |= slave ? SPI_SLAVE : SPI_MASTER;
	ctl0 |= SPI_TRANSMODE_FULLDUPLEX;
	ctl0 |= (SPI_WORD_SIZE_GET(config->operation) == 16U) ? SPI_FRAMESIZE_16BIT
							      : SPI_FRAMESIZE_8BIT;
	ctl0 |= slave ? SPI_NSS_HARD
		      : (spi_cs_is_gpio(config) ? SPI_NSS_SOFT : SPI_NSS_HARD);
	ctl0 |= (config->operation & SPI_TRANSFER_LSB) != 0U ? SPI_ENDIAN_LSB : SPI_ENDIAN_MSB;

	if ((config->operation & SPI_MODE_CPOL) != 0U) {
		ctl0 |= (config->operation & SPI_MODE_CPHA) != 0U ? SPI_CK_PL_HIGH_PH_2EDGE
								  : SPI_CK_PL_HIGH_PH_1EDGE;
	} else {
		ctl0 |= (config->operation & SPI_MODE_CPHA) != 0U ? SPI_CK_PL_LOW_PH_2EDGE
								  : SPI_CK_PL_LOW_PH_1EDGE;
	}

	ctl0 |= CTL0_PSC(prescaler);

	SPI_CTL0(cfg->reg) = ctl0;
	SPI_I2SCTL(cfg->reg) &= ~SPI_I2SCTL_I2SSEL;

	if (ti) {
		ctl1 |= SPI_CTL1_TMOD;
	}
	if (!slave && !spi_cs_is_gpio(config)) {
		ctl1 |= SPI_CTL1_NSSDRV;
	}

	SPI_CTL1(cfg->reg) = ctl1;

	data->ctx.config = config;
	data->dfs = spi_gd32_dfs_from_op(config->operation);

	return 0;
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

static void spi_gd32_prepare_transfer(const struct device *dev)
{
	struct spi_gd32_data *data = dev->data;
	struct spi_context *ctx = &data->ctx;

	data->transfer_tx = spi_context_total_tx_len(ctx) != 0U;
	data->transfer_rx = spi_context_total_rx_len(ctx) != 0U;
	data->frames_left = MAX(spi_context_total_tx_len(ctx), spi_context_total_rx_len(ctx)) /
			    data->dfs;
	data->rx_pending = 0U;
	data->xfer_mode = SPI_GD32_XFER_POLLING;
	data->transfer_active = false;
#ifdef CONFIG_SPI_GD32_DMA
	data->dma_done[RX] = false;
	data->dma_done[TX] = false;
	data->dma_chunk_len = 0U;
#endif
#if defined(CONFIG_SPI_ASYNC) || defined(CONFIG_SPI_RTIO)
	spi_gd32_disarm_timeout(data);
	data->async.completion_status = 0;
	data->async.completion_suppressed = false;
	data->async.progress_seen = false;
#endif
}

static enum spi_gd32_xfer_mode spi_gd32_select_xfer_mode(const struct device *dev,
							 const struct spi_buf_set *tx_bufs,
							 const struct spi_buf_set *rx_bufs,
							 uint8_t dfs)
{
#ifdef CONFIG_SPI_GD32_DMA
	if (spi_gd32_dma_possible(dev, tx_bufs, rx_bufs, dfs)) {
		return SPI_GD32_XFER_DMA;
	}
#else
	ARG_UNUSED(tx_bufs);
	ARG_UNUSED(rx_bufs);
	ARG_UNUSED(dfs);
#endif
	ARG_UNUSED(dev);
#ifdef CONFIG_SPI_GD32_INTERRUPT
	return SPI_GD32_XFER_INTERRUPT;
#else
	return SPI_GD32_XFER_POLLING;
#endif
}

static int spi_gd32_start_transfer(const struct device *dev)
{
	const struct spi_gd32_config *cfg = dev->config;
	struct spi_gd32_data *data = dev->data;
	bool slave = spi_gd32_is_slave(data);

	spi_gd32_stop_transfer_engine(dev);
	spi_gd32_flush_stale_status(cfg);
	data->transfer_active = true;

#ifdef CONFIG_SPI_GD32_DMA
	if (data->xfer_mode == SPI_GD32_XFER_DMA) {
		int ret = spi_gd32_dma_prepare_chunk(dev);

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
	if (data->xfer_mode == SPI_GD32_XFER_DMA) {
		int ret = spi_gd32_dma_start_chunk(dev);

		if (ret != 0) {
			data->transfer_active = false;
			return ret;
		}
		return 0;
	}
#endif

	/* In slave mode the first frame must be queued after SPIEN is set and
	 * before the master starts clocking, otherwise the first response
	 * frame is shifted out as zeros.
	 */
	if (slave && !spi_gd32_is_rx_only(data)) {
		uint32_t stat = spi_gd32_stat(cfg);

		while ((stat & SPI_STAT_TBE) != 0U && spi_gd32_can_write_frame(data)) {
			spi_gd32_write_frame(dev);
			stat = spi_gd32_stat(cfg);
		}
	}

#ifdef CONFIG_SPI_GD32_INTERRUPT
	if (data->xfer_mode == SPI_GD32_XFER_INTERRUPT) {
		spi_gd32_update_interrupt_mask(cfg, data);
	}
#endif

	return 0;
}

static int spi_gd32_cleanup_transfer(const struct device *dev, int status)
{
	struct spi_gd32_data *data = dev->data;
	const struct spi_gd32_config *cfg = dev->config;

#if defined(CONFIG_SPI_ASYNC) || defined(CONFIG_SPI_RTIO)
	spi_gd32_disarm_timeout(data);
#endif

	if (status == 0 && !spi_gd32_is_slave(data)) {
		status = spi_gd32_wait_until_idle(dev);
	}

	if (status < 0) {
		spi_gd32_force_recover(dev, true);
#ifdef CONFIG_SPI_ASYNC
		if (!data->ctx.asynchronous) {
			data->ctx.config = NULL;
		}
#else
		data->ctx.config = NULL;
#endif
		return status;
	}

	spi_gd32_stop_transfer_engine(dev);
	if (!spi_gd32_is_slave(data)) {
		spi_context_cs_control(&data->ctx, false);
	}
	if (!spi_gd32_keep_enabled(data)) {
		SPI_CTL0(cfg->reg) &= ~SPI_CTL0_SPIEN;
	}
	spi_gd32_flush_stale_status(cfg);

	return 0;
}

#ifdef CONFIG_SPI_RTIO
static int spi_gd32_cleanup_transfer_step(const struct device *dev)
{
	struct spi_gd32_data *data = dev->data;
	const struct spi_gd32_config *cfg = dev->config;

	spi_gd32_stop_transfer_engine(dev);

	if (!spi_gd32_is_slave(data)) {
		int ret = spi_gd32_wait_until_idle(dev);

		if (ret != 0) {
			spi_gd32_force_recover(dev, true);
			return ret;
		}
	}
	spi_gd32_flush_stale_status(cfg);
	return 0;
}

static int spi_gd32_iodev_setup_buffers(const struct device *dev)
{
	struct spi_gd32_data *data = dev->data;
	struct spi_rtio *rtio_ctx = data->rtio_ctx;
	struct rtio_sqe *sqe = &rtio_ctx->txn_curr->sqe;
	struct spi_buf tx_buf = {0};
	struct spi_buf rx_buf = {0};
	struct spi_buf_set tx_bufs = {.buffers = &tx_buf, .count = 1U};
	struct spi_buf_set rx_bufs = {.buffers = &rx_buf, .count = 1U};
	const struct spi_buf_set *tx_set = NULL;
	const struct spi_buf_set *rx_set = NULL;
	int ret;

	switch (sqe->op) {
	case RTIO_OP_RX:
		rx_buf.buf = sqe->rx.buf;
		rx_buf.len = sqe->rx.buf_len;
		rx_set = &rx_bufs;
		break;
	case RTIO_OP_TX:
		tx_buf.buf = (void *)sqe->tx.buf;
		tx_buf.len = sqe->tx.buf_len;
		tx_set = &tx_bufs;
		break;
	case RTIO_OP_TINY_TX:
		tx_buf.buf = (void *)sqe->tiny_tx.buf;
		tx_buf.len = sqe->tiny_tx.buf_len;
		tx_set = &tx_bufs;
		break;
	case RTIO_OP_TXRX:
		tx_buf.buf = (void *)sqe->txrx.tx_buf;
		tx_buf.len = sqe->txrx.buf_len;
		rx_buf.buf = sqe->txrx.rx_buf;
		rx_buf.len = sqe->txrx.buf_len;
		tx_set = &tx_bufs;
		rx_set = &rx_bufs;
		break;
	default:
		return -EINVAL;
	}

	ret = spi_gd32_validate_buffers(tx_set, data->dfs);
	if (ret == 0) {
		ret = spi_gd32_validate_buffers(rx_set, data->dfs);
	}
	if (ret != 0) {
		return ret;
	}

	spi_context_buffers_setup(&data->ctx, tx_set, rx_set, data->dfs);
	spi_gd32_prepare_transfer(dev);
	data->xfer_mode = spi_gd32_select_xfer_mode(dev, tx_set, rx_set, data->dfs);

	return 0;
}

static int spi_gd32_iodev_prepare_start(const struct device *dev)
{
	struct spi_gd32_data *data = dev->data;
	struct spi_rtio *rtio_ctx = data->rtio_ctx;
	struct rtio_iodev_sqe *txn_curr = rtio_ctx->txn_curr;
	struct spi_dt_spec *spi_dt_spec;

	if (txn_curr == NULL) {
		return -EINVAL;
	}
	spi_dt_spec = txn_curr->sqe.iodev->data;

	if (!data->rtio_active) {
		int ret;

		spi_context_lock(&data->ctx, false, NULL, NULL, &spi_dt_spec->config);
		data->rtio_active = true;
		ret = spi_gd32_configure(dev, &spi_dt_spec->config);
		if (ret != 0) {
			return ret;
		}
	} else if (txn_curr->sqe.iodev != rtio_ctx->txn_head->sqe.iodev) {
		return -EINVAL;
	}

	return spi_gd32_iodev_setup_buffers(dev);
}

static void spi_gd32_iodev_start(const struct device *dev)
{
	struct spi_gd32_data *data = dev->data;
	struct spi_rtio *rtio_ctx = data->rtio_ctx;
	bool new_transaction = !data->rtio_active;
	int ret = spi_gd32_iodev_prepare_start(dev);

	if (ret != 0) {
		goto out_error;
	}
	if (data->frames_left == 0U) {
		spi_gd32_iodev_complete(dev, 0);
		return;
	}

	ret = spi_gd32_start_transfer(dev);
	if (ret != 0) {
		goto out_error;
	}

#if defined(CONFIG_SPI_ASYNC) || defined(CONFIG_SPI_RTIO)
	spi_gd32_arm_timeout_on_start(dev);
#endif
	if (data->xfer_mode == SPI_GD32_XFER_POLLING) {
		int xret = spi_gd32_polling_transfer(dev);

		data->transfer_active = false;
		spi_gd32_iodev_complete(dev, xret);
	}
	return;

out_error:
	if (data->rtio_active) {
		ret = spi_gd32_cleanup_transfer(dev, ret);
		data->rtio_active = false;
		if (ret < 0) {
			data->ctx.config = NULL;
		}
		spi_context_release(&data->ctx, ret);
	} else if (new_transaction) {
		if (ret < 0) {
			data->ctx.config = NULL;
		}
		spi_context_release(&data->ctx, ret);
	}

	if (spi_rtio_complete(rtio_ctx, ret)) {
		(void)k_work_submit(&data->rtio_work);
	}
}

static void spi_gd32_iodev_complete(const struct device *dev, int status)
{
	struct spi_gd32_data *data = dev->data;
	struct spi_rtio *rtio_ctx = data->rtio_ctx;

	if (status == 0 && (rtio_ctx->txn_curr->sqe.flags & RTIO_SQE_TRANSACTION) != 0U) {
		status = spi_gd32_cleanup_transfer_step(dev);
		if (status == 0) {
			rtio_ctx->txn_curr = rtio_txn_next(rtio_ctx->txn_curr);
			spi_gd32_iodev_start(dev);
			return;
		}
	}

	status = spi_gd32_cleanup_transfer(dev, status);
	data->rtio_active = false;
	if (status < 0) {
		data->ctx.config = NULL;
	}
	spi_context_release(&data->ctx, status);

	if (spi_rtio_complete(rtio_ctx, status)) {
		(void)k_work_submit(&data->rtio_work);
	}
}

static void spi_gd32_iodev_submit(const struct device *dev, struct rtio_iodev_sqe *iodev_sqe)
{
	struct spi_gd32_data *data = dev->data;

	if (spi_rtio_submit(data->rtio_ctx, iodev_sqe)) {
		(void)k_work_submit(&data->rtio_work);
	}
}

static void spi_gd32_rtio_work_handler(struct k_work *work)
{
	struct spi_gd32_data *data = CONTAINER_OF(work, struct spi_gd32_data, rtio_work);

	if (!data->rtio_active && data->rtio_ctx->txn_curr != NULL) {
		spi_gd32_iodev_start(data->dev);
	}
}
#endif

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

static int spi_gd32_transceive_impl(const struct device *dev, const struct spi_config *config,
				    const struct spi_buf_set *tx_bufs,
				    const struct spi_buf_set *rx_bufs, bool asynchronous,
				    spi_callback_t cb, void *userdata)
{
	struct spi_gd32_data *data = dev->data;
	int ret;

	if (tx_bufs == NULL && rx_bufs == NULL) {
		return 0;
	}

	spi_context_lock(&data->ctx, asynchronous, cb, userdata, config);

	ret = spi_gd32_configure(dev, config);
	if (ret != 0) {
		goto out_release;
	}

	ret = spi_gd32_validate_buffers(tx_bufs, data->dfs);
	if (ret == 0) {
		ret = spi_gd32_validate_buffers(rx_bufs, data->dfs);
	}
	if (ret != 0) {
		goto out_release;
	}

	spi_context_buffers_setup(&data->ctx, tx_bufs, rx_bufs, data->dfs);
	spi_gd32_prepare_transfer(dev);

	if (data->frames_left == 0U) {
		ret = 0;
		goto out_release;
	}

	data->xfer_mode = spi_gd32_select_xfer_mode(dev, tx_bufs, rx_bufs, data->dfs);

#ifdef CONFIG_SPI_ASYNC
	if (asynchronous && data->xfer_mode == SPI_GD32_XFER_POLLING) {
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
		int xfer_ret = spi_gd32_polling_transfer(dev);

		data->transfer_active = false;
		ret = spi_gd32_cleanup_transfer(dev, xfer_ret);
#ifdef CONFIG_SPI_SLAVE
		if (ret == 0 && spi_gd32_is_slave(data)) {
			ret = data->ctx.recv_frames;
		}
#endif
		goto out_release;
	}

#ifdef CONFIG_SPI_ASYNC
	if (data->ctx.asynchronous) {
		spi_gd32_arm_timeout_on_start(dev);
		return 0;
	}
#endif

#if defined(CONFIG_SPI_GD32_INTERRUPT) || defined(CONFIG_SPI_GD32_DMA)
	{
		int wait_ret;

#if defined(CONFIG_SPI_ASYNC) || defined(CONFIG_SPI_RTIO)
		spi_gd32_arm_timeout_on_start(dev);
#endif
		wait_ret = spi_gd32_wait_for_completion(dev);
#ifdef CONFIG_SPI_GD32_DMA
		if (wait_ret == -ETIMEDOUT && data->xfer_mode == SPI_GD32_XFER_DMA) {
			int recover_ret = spi_gd32_dma_recover_completion(dev, true);

			if (recover_ret > 0) {
				wait_ret = spi_gd32_wait_for_completion(dev);
			} else if (recover_ret < 0) {
				wait_ret = recover_ret;
			}
		}
#endif

		if (wait_ret == -ETIMEDOUT && spi_gd32_mark_transfer_inactive(data)) {
			spi_gd32_note_fault(dev, wait_ret, 0U,
					    "timeout waiting for transfer completion");
			spi_gd32_stop_transfer_engine(dev);
		}
		ret = spi_gd32_cleanup_transfer(dev, wait_ret);
	}
#endif

out_release:
	spi_context_release(&data->ctx, ret);
	return ret;
}

static int spi_gd32_transceive(const struct device *dev, const struct spi_config *config,
			       const struct spi_buf_set *tx_bufs, const struct spi_buf_set *rx_bufs)
{
	return spi_gd32_transceive_impl(dev, config, tx_bufs, rx_bufs, false, NULL, NULL);
}

#ifdef CONFIG_SPI_ASYNC
static int spi_gd32_transceive_async(const struct device *dev, const struct spi_config *config,
				     const struct spi_buf_set *tx_bufs,
				     const struct spi_buf_set *rx_bufs, spi_callback_t cb,
				     void *userdata)
{
	return spi_gd32_transceive_impl(dev, config, tx_bufs, rx_bufs, true, cb, userdata);
}
#endif

static int spi_gd32_release(const struct device *dev, const struct spi_config *config)
{
	struct spi_gd32_data *data = dev->data;
	bool was_active = false;

	ARG_UNUSED(config);

#if defined(CONFIG_SPI_ASYNC) || defined(CONFIG_SPI_RTIO)
	data->async.completion_suppressed = true;
	spi_gd32_disarm_timeout(data);
#endif

	if (data->transfer_active) {
#if defined(CONFIG_SPI_GD32_INTERRUPT) || defined(CONFIG_SPI_GD32_DMA)
		was_active = spi_gd32_mark_transfer_inactive(data);
#else
		was_active = true;
		data->transfer_active = false;
#endif
		if (was_active) {
			spi_gd32_note_fault(dev, -ECANCELED, 0U,
					    "transfer aborted by release");
		}
	}

	if (was_active) {
		spi_gd32_force_recover(dev, true);
		data->ctx.config = NULL;
	}

	spi_context_unlock_unconditionally(&data->ctx);
	return 0;
}

static DEVICE_API(spi, spi_gd32_driver_api) = {
	.transceive = spi_gd32_transceive,
#ifdef CONFIG_SPI_ASYNC
	.transceive_async = spi_gd32_transceive_async,
#endif
#ifdef CONFIG_SPI_RTIO
	.iodev_submit = spi_gd32_iodev_submit,
#endif
	.release = spi_gd32_release,
};

static int spi_gd32_init(const struct device *dev)
{
	struct spi_gd32_data *data = dev->data;
	const struct spi_gd32_config *cfg = dev->config;
	int ret;

	ret = clock_control_on(GD32_CLOCK_CONTROLLER, (clock_control_subsys_t)&cfg->clkid);
	if (ret != 0) {
		LOG_ERR("Failed to enable SPI clock");
		return ret;
	}

	data->dev = dev;

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
			uint32_t ch_filter;

			if (!device_is_ready(cfg->dma[i].dev)) {
				LOG_ERR("DMA controller %s not ready", cfg->dma[i].dev->name);
				return -ENODEV;
			}
			ch_filter = BIT(cfg->dma[i].channel);
			ret = dma_request_channel(cfg->dma[i].dev, &ch_filter);
			if (ret < 0) {
				LOG_ERR("Failed to request DMA channel %u",
					cfg->dma[i].channel);
				return ret;
			}
		}
	}
#endif

	ret = spi_context_cs_configure_all(&data->ctx);
	if (ret != 0) {
		return ret;
	}

#ifdef CONFIG_SPI_RTIO
	k_work_init(&data->rtio_work, spi_gd32_rtio_work_handler);
	spi_rtio_init(data->rtio_ctx, dev);
#endif

#if defined(CONFIG_SPI_ASYNC) || defined(CONFIG_SPI_RTIO)
	k_work_init(&data->async.completion_work, spi_gd32_completion_work_handler);
	k_work_init_delayable(&data->async.timeout_work, spi_gd32_timeout_work_handler);
#endif

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
		.slot = COND_CODE_1(DT_HAS_COMPAT_STATUS_OKAY(gd_gd32_dma_v1),                     \
				    (DT_INST_DMAS_CELL_BY_NAME(idx, dir, slot)), (0)),             \
		.config = DT_INST_DMAS_CELL_BY_NAME(idx, dir, config),                             \
		.fifo_threshold = COND_CODE_1(DT_HAS_COMPAT_STATUS_OKAY(gd_gd32_dma_v1),           \
					      (GD32_DMA_DT_FIFO_MODE(                            \
						      DT_INST_DMAS_CELL_BY_NAME(                  \
							      idx, dir, fifo_threshold))),      \
					      (0)),                                                \
		}

#define DMAS_DECL(idx)                                                                             \
	{                                                                                          \
		COND_CODE_1(DT_INST_DMAS_HAS_NAME(idx, rx),                                        \
			    (DMA_INITIALIZER(idx, rx)), ({0})),                                    \
		COND_CODE_1(DT_INST_DMAS_HAS_NAME(idx, tx),                                        \
			    (DMA_INITIALIZER(idx, tx)), ({0})),                                    \
	}

#define GD32_IRQ_CONFIGURE(idx)                                                                    \
	static void spi_gd32_irq_configure_##idx(const struct device *dev)                         \
	{                                                                                          \
		ARG_UNUSED(dev);                                                                   \
		IRQ_CONNECT(DT_INST_IRQN(idx), DT_INST_IRQ(idx, priority), spi_gd32_isr,           \
			    DEVICE_DT_INST_GET(idx), 0);                                           \
		irq_enable(DT_INST_IRQN(idx));                                                     \
	}

#ifdef CONFIG_SPI_GD32_DMA
#define GD32_SPI_DMA_DUMMY_DEFINE(idx)                                                             \
	static uint32_t spi_gd32_dummy_tx_##idx SPI_GD32_DMA_DUMMY_ATTR;                           \
	static uint32_t spi_gd32_dummy_rx_##idx SPI_GD32_DMA_DUMMY_ATTR;
#define GD32_SPI_DMA_CFG(idx)                                                                      \
	.dma = DMAS_DECL(idx),                                                                     \
	.dummy_tx = &spi_gd32_dummy_tx_##idx,                                                      \
	.dummy_rx = &spi_gd32_dummy_rx_##idx,
#else
#define GD32_SPI_DMA_DUMMY_DEFINE(idx)
#define GD32_SPI_DMA_CFG(idx)
#endif

#define GD32_SPI_INIT(idx)                                                                         \
	PINCTRL_DT_INST_DEFINE(idx);                                                               \
	IF_ENABLED(CONFIG_SPI_GD32_INTERRUPT, (GD32_IRQ_CONFIGURE(idx)))                            \
	IF_ENABLED(CONFIG_SPI_RTIO,                                                                \
		   (SPI_RTIO_DEFINE(spi_gd32_rtio_##idx,                                           \
				    CONFIG_SPI_GD32_RTIO_SQ_SIZE,                                  \
				    CONFIG_SPI_GD32_RTIO_CQ_SIZE);))                               \
	GD32_SPI_DMA_DUMMY_DEFINE(idx)                                                             \
	static struct spi_gd32_data spi_gd32_data_##idx = {                                        \
		SPI_CONTEXT_INIT_LOCK(spi_gd32_data_##idx, ctx),                                   \
		SPI_CONTEXT_INIT_SYNC(spi_gd32_data_##idx, ctx),                                   \
		SPI_CONTEXT_CS_GPIOS_INITIALIZE(DT_DRV_INST(idx), ctx)                             \
		IF_ENABLED(CONFIG_SPI_RTIO, (.rtio_ctx = &spi_gd32_rtio_##idx,))                   \
	};                                                                                         \
	static const struct spi_gd32_config spi_gd32_config_##idx = {                              \
		.reg = DT_INST_REG_ADDR(idx),                                                      \
		.clkid = DT_INST_CLOCKS_CELL(idx, id),                                             \
		.reset = RESET_DT_SPEC_INST_GET(idx),                                              \
		.pcfg = PINCTRL_DT_INST_DEV_CONFIG_GET(idx),                                       \
		GD32_SPI_DMA_CFG(idx)                                                              \
		IF_ENABLED(CONFIG_SPI_GD32_INTERRUPT,                                              \
			   (.irq_configure = spi_gd32_irq_configure_##idx,))                       \
	};                                                                                         \
	SPI_DEVICE_DT_INST_DEFINE(idx, spi_gd32_init, NULL, &spi_gd32_data_##idx,                  \
				  &spi_gd32_config_##idx, POST_KERNEL, CONFIG_SPI_INIT_PRIORITY,   \
				  &spi_gd32_driver_api);

DT_INST_FOREACH_STATUS_OKAY(GD32_SPI_INIT)
