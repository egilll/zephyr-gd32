/*
 * Copyright (c) 2026 Ylhyra ehf.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT gd_gd32_i2s

#include <errno.h>
#include <string.h>

#include <zephyr/cache.h>
#include <zephyr/device.h>
#include <zephyr/drivers/clock_control.h>
#include <zephyr/drivers/clock_control/gd32.h>
#include <zephyr/drivers/dma.h>
#include <zephyr/drivers/dma/dma_gd32.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/i2s.h>
#include <zephyr/drivers/pinctrl.h>
#include <zephyr/drivers/reset.h>
#include <zephyr/irq.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/clock.h>
#include <zephyr/sys/util.h>

#include <gd32_spi.h>

#include "i2s_gd32_banks.h"

LOG_MODULE_REGISTER(i2s_gd32, CONFIG_I2S_LOG_LEVEL);

/*
 * Each direction runs a two-buffer ("switch-buffer") DMA channel over the application's own
 * blocks: the DMA moves to the other buffer at the end of every block, and the completion points
 * the buffer it left at the next block. A transmit block is released once it has been sent; a
 * receive buffer gets a fresh block and its filled block is queued for i2s_read().
 *
 * Both buffers share one transfer count, so every block has the configured length: a shorter
 * transmit block is followed by silence. Receiving needs a third slab block to swap in.
 *
 * A stream underruns or overruns when the DMA reaches a buffer with nothing to send or nowhere to
 * receive; a transmit block written after the completion found none is in time if it is there
 * before the DMA. A stream also fails if a completion was serviced too late to know which buffer
 * the DMA is in. Either way it enters the error state until I2S_TRIGGER_PREPARE.
 *
 * The DMA cannot be told to stop after a buffer, only halted from its completion: STOP ends after
 * the blocks the DMA holds, the one it is in and the one it has been given next, and in the
 * interrupt latency after the last block of a stream the start of that block is sent again.
 */

#define GD32_I2SPSC_DEFAULT_VALUE 0x00000002U
#define GD32_I2S_STOP_TIMEOUT_MS  10U
#define GD32_I2S_RX_MIN_BLOCKS    3U

struct queue_item {
	void *mem_block;
	size_t size;
};

struct i2s_gd32_dma {
	const struct device *dev;
	uint32_t channel;
	uint32_t slot;
	uint32_t config;
	uint32_t fifo_threshold;
};

struct i2s_gd32_config {
	uint32_t reg;
	uint16_t clkid;
	struct reset_dt_spec reset;
	const struct pinctrl_dev_config *pcfg;
	bool mck_enabled;
	void (*irq_configure)(const struct device *dev);
	struct i2s_gd32_dma dma_rx;
	struct i2s_gd32_dma dma_tx;
	struct gpio_dt_spec ws_gpio;
};

struct stream {
	const struct device *dev;
	enum i2s_dir dir;
	enum i2s_state state;
	struct k_msgq *msgq;
	struct i2s_config cfg;
	const struct device *dma_dev;
	uint32_t dma_channel;
	uint32_t dma_slot;
	uint32_t dma_priority;
	struct gd32_i2s_banks banks;
	/* A finished block the DMA may still read, released once it has stopped */
	void *retired;
	uint64_t actual_frame_rate_num;
	uint64_t actual_frame_rate_den;
	uint64_t frame_count;
	uint64_t timestamp_cycles;
	/* Completions further apart than a block and a half mean one was missed */
	uint32_t block_cycles;
	uint32_t last_completion;
	bool completed;
	/* The last block is done; the stop work stops the stream. Work left over from an earlier
	 * stop finds the stream not finishing, or finishing as it would itself have left it.
	 */
	bool finishing;
	struct k_work_delayable stop_work;
	int64_t stop_deadline;
	bool master;
	bool drain;
};

struct i2s_gd32_data {
	struct stream rx;
	struct stream tx;
	/* Guards the streams' state and blocks against the interrupt handlers */
	struct k_spinlock lock;
	/* Serializes configuration, triggers and stop work */
	struct k_mutex api_lock;
	bool duplex;
	bool clock_claimed;
};

static inline uint32_t gd32_i2s_add_reg(uint32_t reg)
{
	if (reg == SPI1) {
		return I2S1_ADD;
	}

	if (reg == SPI2) {
		return I2S2_ADD;
	}

	return 0U;
}

static inline uint32_t gd32_i2s_rx_reg(const struct device *dev)
{
	const struct i2s_gd32_config *cfg = dev->config;
	const struct i2s_gd32_data *data = dev->data;
	uint32_t add = gd32_i2s_add_reg(cfg->reg);

	return data->duplex && add ? add : cfg->reg;
}

static inline uint32_t gd32_i2s_stream_reg(const struct stream *stream)
{
	const struct i2s_gd32_config *cfg = stream->dev->config;

	return stream->dir == I2S_DIR_RX ? gd32_i2s_rx_reg(stream->dev) : cfg->reg;
}

static inline uint32_t gd32_i2s_stream_dma_request(const struct stream *stream)
{
	return stream->dir == I2S_DIR_RX ? SPI_DMA_RECEIVE : SPI_DMA_TRANSMIT;
}

static inline const char *gd32_i2s_dir_name(const struct stream *stream)
{
	return stream->dir == I2S_DIR_RX ? "rx" : "tx";
}

static int queue_get(struct k_msgq *q, void **mem_block, size_t *size, int32_t timeout)
{
	struct queue_item item;
	int ret = k_msgq_get(q, &item, SYS_TIMEOUT_MS(timeout));

	if (ret == 0) {
		*mem_block = item.mem_block;
		*size = item.size;
	}

	return ret;
}

static int queue_put(struct k_msgq *q, void *mem_block, size_t size, int32_t timeout)
{
	struct queue_item item = {
		.mem_block = mem_block,
		.size = size,
	};

	return k_msgq_put(q, &item, SYS_TIMEOUT_MS(timeout));
}

static void stream_queue_drop(struct stream *stream)
{
	size_t size;
	void *mem_block;

	while (queue_get(stream->msgq, &mem_block, &size, 0) == 0) {
		/* A NULL block only wakes a reader after an error */
		if (mem_block != NULL) {
			k_mem_slab_free(stream->cfg.mem_slab, mem_block);
		}
	}
}

static void gd32_i2s_frameformat_apply(uint32_t reg, uint32_t frameformat)
{
	SPI_I2SCTL(reg) &= (uint32_t)(~(SPI_I2SCTL_DTLEN | SPI_I2SCTL_CHLEN));
	SPI_I2SCTL(reg) |= frameformat;
}

static uint32_t gd32_i2s_frame_factor(uint32_t frameformat, uint32_t mckout)
{
	if (mckout == I2S_MCKOUT_ENABLE) {
		return 256U;
	}

	if (frameformat == I2S_FRAMEFORMAT_DT16B_CH16B) {
		return 32U;
	}

	return 64U;
}

static uint32_t gd32_i2s_word_bytes(const struct i2s_config *cfg)
{
	return cfg->word_size > 16U ? sizeof(uint32_t) : sizeof(uint16_t);
}

static uint32_t gd32_i2s_frame_bytes(const struct i2s_config *cfg)
{
	return 2U * gd32_i2s_word_bytes(cfg);
}

static uint32_t gd32_i2s_block_frames(const struct stream *stream)
{
	return stream->cfg.block_size / gd32_i2s_frame_bytes(&stream->cfg);
}

/*
 * Empties the receive buffer and clears its errors. A word received just before the last stop
 * stays in SPI_DATA; the DMA would read it first, and every frame of the new stream would then
 * pair one frame's right channel with the next frame's left.
 */
static void gd32_i2s_rx_flush(uint32_t reg)
{
	volatile uint32_t clear_read;

	while ((SPI_STAT(reg) & SPI_STAT_RBNE) != 0U) {
		clear_read = SPI_DATA(reg);
	}
	clear_read = SPI_STAT(reg);
	(void)clear_read;
	SPI_STAT(reg) &= ~SPI_STAT_FERR;
}

static void gd32_i2s_tx_errors_clear(uint32_t reg)
{
	volatile uint32_t clear_read;

	if ((SPI_STAT(reg) & SPI_STAT_TXURERR) != 0U) {
		clear_read = SPI_STAT(reg);
		(void)clear_read;
	}
	SPI_STAT(reg) &= ~SPI_STAT_FERR;
}

static int gd32_i2s_master_clock_apply(const struct device *dev, uint32_t target_sample_rate_hz,
				       uint32_t frameformat, uint32_t *prescaler,
				       uint64_t *rate_num, uint64_t *rate_den)
{
	const struct i2s_gd32_config *cfg = dev->config;
	struct i2s_gd32_data *data = dev->data;
	struct gd32_i2s_clock_config clk_cfg = {
		.type = GD32_CLOCK_CONFIG_TYPE_I2S,
		.action = GD32_I2S_CLOCK_ACQUIRE,
		.source = GD32_I2S_CLOCK_SRC_PLLI2S,
		.target_rate_hz = target_sample_rate_hz,
		.max_error_ppm = CONFIG_I2S_GD32_MAX_CLOCK_ERROR_PPM,
	};
	uint32_t mckout = cfg->mck_enabled ? I2S_MCKOUT_ENABLE : I2S_MCKOUT_DISABLE;
	int ret;

	clk_cfg.frame_factor = gd32_i2s_frame_factor(frameformat, mckout);

	ret = clock_control_configure(GD32_CLOCK_CONTROLLER, (clock_control_subsys_t)&cfg->clkid,
				      &clk_cfg);
	if (ret < 0) {
		return ret;
	}

	LOG_DBG("%s master clock psc=%u%s sample=%uHz target=%uHz", dev->name, clk_cfg.divider,
		clk_cfg.odd ? "+odd" : "", clk_cfg.actual_rate_hz, target_sample_rate_hz);
	*prescaler = clk_cfg.divider | (clk_cfg.odd ? BIT(8) : 0U) | mckout;
	*rate_num = clk_cfg.rate_num;
	*rate_den = clk_cfg.rate_den;
	data->clock_claimed = true;

	return 0;
}

static void gd32_i2s_clock_release(const struct device *dev)
{
	const struct i2s_gd32_config *cfg = dev->config;
	struct i2s_gd32_data *data = dev->data;
	struct gd32_i2s_clock_config clk_cfg = {
		.type = GD32_CLOCK_CONFIG_TYPE_I2S,
		.action = GD32_I2S_CLOCK_RELEASE,
	};

	if (data->clock_claimed) {
		(void)clock_control_configure(GD32_CLOCK_CONTROLLER,
					      (clock_control_subsys_t)&cfg->clkid, &clk_cfg);
		data->clock_claimed = false;
	}
}

/* Stops the stream's peripheral side at once: no more DMA requests and no more frames. */
static void gd32_i2s_halt(struct stream *stream)
{
	const uint32_t reg = gd32_i2s_stream_reg(stream);

	SPI_CTL1(reg) &= ~SPI_CTL1_ERRIE;
	spi_dma_disable(reg, gd32_i2s_stream_dma_request(stream));
	i2s_disable(reg);
}

static bool gd32_i2s_active(const struct stream *stream)
{
	return stream->state == I2S_STATE_RUNNING || stream->state == I2S_STATE_STOPPING;
}

/*
 * Puts the stream into the error state; in full duplex both directions share the peripheral, so
 * both stop. A reader waiting for a block, or a writer waiting for room, is woken. Called with the
 * lock held.
 */
static void gd32_i2s_fail_locked(struct stream *stream)
{
	struct i2s_gd32_data *data = stream->dev->data;
	struct stream *const streams[] = {
		data->duplex ? &data->rx : stream,
		data->duplex ? &data->tx : NULL,
	};

	ARRAY_FOR_EACH(streams, i) {
		struct stream *failed = streams[i];

		if (failed == NULL || !gd32_i2s_active(failed)) {
			continue;
		}

		gd32_i2s_halt(failed);
		failed->state = I2S_STATE_ERROR;
		if (failed->dir == I2S_DIR_RX) {
			(void)queue_put(failed->msgq, NULL, 0U, 0);
		} else {
			/* Nothing queued will be sent; a writer waiting for room gets it. */
			stream_queue_drop(failed);
		}
	}
}

/* Hands the stream to the stop work once its last block is done. Called with the lock held. */
static void gd32_i2s_finish_locked(struct stream *stream)
{
	stream->finishing = true;
	stream->stop_deadline = k_uptime_get() + GD32_I2S_STOP_TIMEOUT_MS;
	k_work_reschedule(&stream->stop_work, K_NO_WAIT);
}

static int gd32_i2s_reload(struct stream *stream, void *memory)
{
	const uint32_t data_reg = (uint32_t)&SPI_DATA(gd32_i2s_stream_reg(stream));

	if (stream->dir == I2S_DIR_RX) {
		return dma_reload(stream->dma_dev, stream->dma_channel, data_reg, (uint32_t)memory,
				  stream->cfg.block_size);
	}

	return dma_reload(stream->dma_dev, stream->dma_channel, (uint32_t)memory, data_reg,
			  stream->cfg.block_size);
}

/*
 * Gives the next transmit bank the oldest queued block if its completion found none. With a
 * completion pending, the DMA may already be past that bank: the block stays queued, for the
 * completion to take. Lock held.
 */
static int gd32_i2s_tx_feed_locked(struct stream *stream)
{
	struct queue_item item;
	int ret;

	if (stream->state != I2S_STATE_RUNNING || gd32_i2s_banks_next_held(&stream->banks) ||
	    k_msgq_peek(stream->msgq, &item) != 0) {
		return 0;
	}

	ret = gd32_i2s_reload(stream, gd32_i2s_banks_fill_next(&stream->banks, item.mem_block));
	if (ret != 0) {
		(void)gd32_i2s_banks_fill_next(&stream->banks, NULL);
		return ret == -EBUSY ? 0 : ret;
	}

	(void)k_msgq_get(stream->msgq, &item, K_NO_WAIT);
	return 0;
}

/*
 * The streams' clock starts now, so their first blocks complete a block from now; in full duplex
 * one clock drives both. Without this, as for a target whose master may not be clocking yet, the
 * first completion is not timed. Lock held.
 */
static void gd32_i2s_clock_starts_locked(struct stream *stream)
{
	struct i2s_gd32_data *data = stream->dev->data;
	const uint32_t now = k_cycle_get_32();

	if (data->duplex) {
		data->rx.completed = true;
		data->rx.last_completion = now;
		data->tx.completed = true;
		data->tx.last_completion = now;
	} else {
		stream->completed = true;
		stream->last_completion = now;
	}
}

/* Whether this completion came within a block and a half of the last one, or of the start. */
static bool gd32_i2s_on_time(struct stream *stream)
{
	const uint32_t now = k_cycle_get_32();
	const bool late = stream->completed &&
			  now - stream->last_completion >
				  stream->block_cycles + stream->block_cycles / 2U;

	stream->completed = true;
	stream->last_completion = now;
	return !late;
}

static const char *gd32_i2s_tx_complete_locked(struct stream *stream)
{
	void *done = gd32_i2s_banks_complete(&stream->banks);
	void *next = NULL;
	size_t size;

	stream->frame_count += gd32_i2s_block_frames(stream);
	stream->timestamp_cycles = k_cycle_get_64();

	if (!gd32_i2s_banks_active_held(&stream->banks)) {
		/* The DMA moved to a bank with nothing left to send; it reads `done` until halted. */
		stream->retired = done;
		if (stream->state == I2S_STATE_STOPPING) {
			const uint32_t reg = gd32_i2s_stream_reg(stream);

			/* A target with nothing to send underruns on the next frame, by design. */
			SPI_CTL1(reg) &= ~SPI_CTL1_ERRIE;
			spi_dma_disable(reg, SPI_DMA_TRANSMIT);
			gd32_i2s_finish_locked(stream);
			return NULL;
		}
		gd32_i2s_fail_locked(stream);
		return "underrun";
	}

	if (stream->state == I2S_STATE_RUNNING || stream->drain) {
		(void)queue_get(stream->msgq, &next, &size, 0);
	}

	if (gd32_i2s_reload(stream, gd32_i2s_banks_fill_next(&stream->banks, next)) < 0) {
		stream->retired = done;
		gd32_i2s_fail_locked(stream);
		return "DMA reload";
	}

	/* The bank that sent `done` points elsewhere now, and the DMA is in the other one. */
	k_mem_slab_free(stream->cfg.mem_slab, done);
	return NULL;
}

static const char *gd32_i2s_rx_complete_locked(struct stream *stream)
{
	void *done = gd32_i2s_banks_complete(&stream->banks);
	void *fresh;

	stream->frame_count += gd32_i2s_block_frames(stream);
	stream->timestamp_cycles = k_cycle_get_64();
	sys_cache_data_invd_range(done, stream->cfg.block_size);

	if (stream->state == I2S_STATE_STOPPING) {
		/*
		 * The DMA stops in the bank it has started; that partial block is discarded. The bank
		 * `done` came from is pointed at it too, so the DMA cannot come back to `done`.
		 */
		if (gd32_i2s_reload(stream, gd32_i2s_banks_fill_next(&stream->banks, NULL)) < 0) {
			stream->retired = done;
			gd32_i2s_fail_locked(stream);
			return "DMA reload";
		}
		gd32_i2s_halt(stream);
		gd32_i2s_finish_locked(stream);
		if (queue_put(stream->msgq, done, stream->cfg.block_size, 0) != 0) {
			k_mem_slab_free(stream->cfg.mem_slab, done);
			stream->state = I2S_STATE_ERROR;
			return "overrun";
		}
		return NULL;
	}

	if (k_mem_slab_alloc(stream->cfg.mem_slab, &fresh, K_NO_WAIT) != 0) {
		/* The bank keeps `done` until the DMA has stopped. */
		(void)gd32_i2s_banks_fill_next(&stream->banks, done);
		gd32_i2s_fail_locked(stream);
		return "overrun";
	}

	sys_cache_data_invd_range(fresh, stream->cfg.block_size);
	if (gd32_i2s_reload(stream, gd32_i2s_banks_fill_next(&stream->banks, fresh)) < 0) {
		stream->retired = done;
		gd32_i2s_fail_locked(stream);
		return "DMA reload";
	}

	if (queue_put(stream->msgq, done, stream->cfg.block_size, 0) != 0) {
		k_mem_slab_free(stream->cfg.mem_slab, done);
		gd32_i2s_fail_locked(stream);
		return "overrun";
	}

	return NULL;
}

static void gd32_i2s_dma_callback(const struct device *dma_dev, void *arg, uint32_t channel,
				  int status)
{
	struct stream *stream = arg;
	struct i2s_gd32_data *data = stream->dev->data;
	const char *failure = NULL;
	k_spinlock_key_t key;

	ARG_UNUSED(dma_dev);
	ARG_UNUSED(channel);

	/* Only whole blocks complete a bank; a half transfer is never one. */
	if (status > 0) {
		return;
	}

	key = k_spin_lock(&data->lock);
	if (!gd32_i2s_active(stream) || stream->finishing) {
		/* Pending while the stream stopped: it belongs to the stopped stream. */
	} else if (status < 0) {
		gd32_i2s_fail_locked(stream);
		failure = "DMA error";
	} else if (!gd32_i2s_on_time(stream)) {
		gd32_i2s_fail_locked(stream);
		failure = "late completion";
	} else if (stream->dir == I2S_DIR_RX) {
		failure = gd32_i2s_rx_complete_locked(stream);
	} else {
		failure = gd32_i2s_tx_complete_locked(stream);
	}
	k_spin_unlock(&data->lock, key);

	if (failure != NULL) {
		LOG_ERR("%s %s %s", stream->dev->name, gd32_i2s_dir_name(stream), failure);
	}
}

/*
 * Stops the stream's peripheral and DMA and puts it into `state`, then releases every block the
 * banks held. If the DMA does not stop, the blocks stay with the stream. Called with the API lock
 * held.
 */
static int gd32_i2s_teardown(struct stream *stream, enum i2s_state state)
{
	struct i2s_gd32_data *data = stream->dev->data;
	void *blocks[3];
	size_t count = 0U;
	k_spinlock_key_t key;
	int ret;

	key = k_spin_lock(&data->lock);
	stream->state = state;
	stream->finishing = false;
	gd32_i2s_halt(stream);
	k_spin_unlock(&data->lock, key);

	(void)k_work_cancel_delayable(&stream->stop_work);
	ret = dma_stop(stream->dma_dev, stream->dma_channel);

	key = k_spin_lock(&data->lock);
	if (ret == 0) {
		count = gd32_i2s_banks_take_all(&stream->banks, blocks);
		if (stream->retired != NULL) {
			blocks[count++] = stream->retired;
			stream->retired = NULL;
		}
	} else {
		stream->state = I2S_STATE_ERROR;
	}
	k_spin_unlock(&data->lock, key);

	for (size_t i = 0U; i < count; i++) {
		k_mem_slab_free(stream->cfg.mem_slab, blocks[i]);
	}

	return ret;
}

static void gd32_i2s_stop_work_handler(struct k_work *work)
{
	struct k_work_delayable *dwork = k_work_delayable_from_work(work);
	struct stream *stream = CONTAINER_OF(dwork, struct stream, stop_work);
	struct i2s_gd32_data *data = stream->dev->data;
	const uint32_t reg = gd32_i2s_stream_reg(stream);
	k_spinlock_key_t key;
	bool current;
	bool shifting;

	k_mutex_lock(&data->api_lock, K_FOREVER);
	key = k_spin_lock(&data->lock);
	current = stream->state == I2S_STATE_STOPPING && stream->finishing;
	k_spin_unlock(&data->lock, key);

	shifting = stream->dir == I2S_DIR_TX &&
		   ((SPI_STAT(reg) & SPI_STAT_TBE) == 0U || (SPI_STAT(reg) & SPI_STAT_TRANS) != 0U);

	if (current && shifting && k_uptime_get() < stream->stop_deadline) {
		/* The last frames are still shifting out. */
		k_work_reschedule(&stream->stop_work, K_USEC(20));
	} else if (current && shifting) {
		/* The clock stopped with frames still to send: they are lost, and like any failure
		 * the stream keeps its blocks until I2S_TRIGGER_PREPARE.
		 */
		key = k_spin_lock(&data->lock);
		stream->finishing = false;
		gd32_i2s_fail_locked(stream);
		k_spin_unlock(&data->lock, key);
		LOG_ERR("%s tx stopped before it drained", stream->dev->name);
	} else if (current) {
		(void)gd32_i2s_teardown(stream, I2S_STATE_READY);
	}

	k_mutex_unlock(&data->api_lock);
}

static int gd32_std_from_config(const struct i2s_config *cfg, uint32_t *std)
{
	switch (cfg->format & I2S_FMT_DATA_FORMAT_MASK) {
	case I2S_FMT_DATA_FORMAT_I2S:
		*std = I2S_STD_PHILLIPS;
		return 0;
	case I2S_FMT_DATA_FORMAT_LEFT_JUSTIFIED:
		*std = I2S_STD_MSB;
		return 0;
	case I2S_FMT_DATA_FORMAT_RIGHT_JUSTIFIED:
		*std = I2S_STD_LSB;
		return 0;
	case I2S_FMT_DATA_FORMAT_PCM_SHORT:
		*std = I2S_STD_PCMSHORT;
		return 0;
	case I2S_FMT_DATA_FORMAT_PCM_LONG:
		*std = I2S_STD_PCMLONG;
		return 0;
	default:
		return -ENOTSUP;
	}
}

static int gd32_frameformat_from_config(const struct i2s_config *cfg, uint32_t *fmt)
{
	if (cfg->word_size == 16U) {
		*fmt = I2S_FRAMEFORMAT_DT16B_CH16B;
		return 0;
	}
	if (cfg->word_size == 24U) {
		*fmt = I2S_FRAMEFORMAT_DT24B_CH32B;
		return 0;
	}
	if (cfg->word_size == 32U) {
		*fmt = I2S_FRAMEFORMAT_DT32B_CH32B;
		return 0;
	}

	return -ENOTSUP;
}

static int gd32_i2s_validate_config(enum i2s_dir dir, const struct i2s_config *cfg)
{
	uint32_t data_format = cfg->format & I2S_FMT_DATA_FORMAT_MASK;

	if ((cfg->format & I2S_FMT_DATA_ORDER_LSB) != 0U) {
		return -ENOTSUP;
	}

	if ((cfg->format & I2S_FMT_FRAME_CLK_INV) != 0U) {
		return -ENOTSUP;
	}

	if ((cfg->options & (I2S_OPT_PINGPONG | I2S_OPT_LOOPBACK | I2S_OPT_BIT_CLK_GATED)) != 0U) {
		return -ENOTSUP;
	}

	if (((cfg->options & I2S_OPT_FRAME_CLK_TARGET) != 0U) !=
	    ((cfg->options & I2S_OPT_BIT_CLK_TARGET) != 0U)) {
		return -EINVAL;
	}

	if (cfg->mem_slab == NULL) {
		return -EINVAL;
	}
	if (cfg->word_size != 16U && cfg->word_size != 24U && cfg->word_size != 32U) {
		return -ENOTSUP;
	}
	if (data_format != I2S_FMT_DATA_FORMAT_I2S && cfg->channels != 2U) {
		return -ENOTSUP;
	}
	if (cfg->frame_clk_freq < 8000U || cfg->frame_clk_freq > 192000U) {
		return -EINVAL;
	}

	if (cfg->block_size == 0U || (cfg->block_size % gd32_i2s_frame_bytes(cfg)) != 0U ||
	    cfg->block_size > cfg->mem_slab->info.block_size) {
		return -EINVAL;
	}

	if (dir == I2S_DIR_RX && cfg->mem_slab->info.num_blocks < GD32_I2S_RX_MIN_BLOCKS) {
		return -EINVAL;
	}

	if (cfg->timeout < SYS_FOREVER_MS) {
		return -EINVAL;
	}

	return 0;
}

static bool gd32_i2s_configs_compatible(const struct i2s_config *first,
					const struct i2s_config *second)
{
	return first->word_size == second->word_size && first->channels == second->channels &&
	       first->format == second->format && first->options == second->options &&
	       first->frame_clk_freq == second->frame_clk_freq &&
	       first->block_size == second->block_size;
}

/*
 * A target has to be enabled before its master starts a frame (GD32F4xx user manual, I2S slave
 * transmission and reception sequences). Enabled part way through a frame it raises a frame
 * error. With ws-gpios the target arms in step with a master that is already clocking: Philips
 * frames start as WS falls, so it arms while WS is high; justified frames start as WS rises, so
 * it arms while WS is low. Waiting for the other level first catches the start of a half frame,
 * which leaves the whole half for the enable. PCM's WS is a pulse too short to poll for, so PCM
 * targets enable at once.
 */
static bool gd32_i2s_arms_on_ws(const struct stream *stream)
{
	const struct i2s_gd32_config *cfg = stream->dev->config;
	const uint32_t format = stream->cfg.format & I2S_FMT_DATA_FORMAT_MASK;

	return !stream->master && cfg->ws_gpio.port != NULL &&
	       format != I2S_FMT_DATA_FORMAT_PCM_SHORT && format != I2S_FMT_DATA_FORMAT_PCM_LONG;
}

static bool gd32_i2s_wait_ws(const struct gpio_dt_spec *ws, int level, uint32_t deadline)
{
	gpio_port_value_t value;

	while ((int32_t)(deadline - k_cycle_get_32()) > 0) {
		if (gpio_port_get_raw(ws->port, &value) == 0 &&
		    (int)((value >> ws->pin) & 1U) == level) {
			return true;
		}
	}
	return false;
}

/* Enables `first` and `second` (or 0) in the half frame before the next frame starts. Without
 * WS edges within two frames the master is not clocking yet, the case the manual describes, so it
 * enables them at once. With the master clocking, interrupts stay locked for at most a frame, and
 * the first completion is timed from the enable.
 */
static void gd32_i2s_enable_on_ws(struct stream *stream, uint32_t first, uint32_t second)
{
	const struct i2s_gd32_config *cfg = stream->dev->config;
	const uint32_t format = stream->cfg.format & I2S_FMT_DATA_FORMAT_MASK;
	const int arm_level = format == I2S_FMT_DATA_FORMAT_I2S ? 1 : 0;
	const uint32_t frames = MAX(stream->cfg.frame_clk_freq, 1U);
	const uint32_t window = 2U * (sys_clock_hw_cycles_per_sec() / frames);
	struct i2s_gd32_data *data = stream->dev->data;
	k_spinlock_key_t key = k_spin_lock(&data->lock);
	const uint32_t deadline = k_cycle_get_32() + window;
	const bool clocking = gd32_i2s_wait_ws(&cfg->ws_gpio, !arm_level, deadline) &&
			      gd32_i2s_wait_ws(&cfg->ws_gpio, arm_level, deadline);

	if (clocking) {
		gd32_i2s_clock_starts_locked(stream);
	}
	i2s_enable(first);
	if (second != 0U) {
		i2s_enable(second);
	}
	k_spin_unlock(&data->lock, key);
}

/* The blocks a stream starts with: queued transmit blocks, or receive blocks from the slab. */
static int gd32_i2s_first_blocks(struct stream *stream, void *blocks[2])
{
	struct queue_item item;

	if (stream->dir == I2S_DIR_TX) {
		if (k_msgq_peek_at(stream->msgq, &item, 0U) != 0) {
			return -EIO;
		}
		blocks[0] = item.mem_block;
		blocks[1] = k_msgq_peek_at(stream->msgq, &item, 1U) == 0 ? item.mem_block : NULL;
		return 0;
	}

	if (k_mem_slab_alloc(stream->cfg.mem_slab, &blocks[0], K_NO_WAIT) != 0) {
		return -ENOMEM;
	}
	if (k_mem_slab_alloc(stream->cfg.mem_slab, &blocks[1], K_NO_WAIT) != 0) {
		k_mem_slab_free(stream->cfg.mem_slab, blocks[0]);
		return -ENOMEM;
	}
	sys_cache_data_invd_range(blocks[0], stream->cfg.block_size);
	sys_cache_data_invd_range(blocks[1], stream->cfg.block_size);
	return 0;
}

/*
 * Points the two DMA banks at the stream's first blocks, starts the DMA and enables the
 * peripheral's requests. The peripheral itself is enabled here, or on the frame clock by the
 * caller. Called with the API lock held, from READY.
 */
static int gd32_i2s_stream_start(struct stream *stream)
{
	const struct i2s_gd32_config *cfg = stream->dev->config;
	struct i2s_gd32_data *data = stream->dev->data;
	const struct i2s_gd32_dma *dma = stream->dir == I2S_DIR_RX ? &cfg->dma_rx : &cfg->dma_tx;
	const uint32_t reg = gd32_i2s_stream_reg(stream);
	struct dma_block_config blocks[2] = {0};
	struct dma_config dma_cfg = {
		.dma_slot = stream->dma_slot,
		.channel_direction =
			stream->dir == I2S_DIR_RX ? PERIPHERAL_TO_MEMORY : MEMORY_TO_PERIPHERAL,
		.channel_priority = stream->dma_priority,
		.source_data_size = 2U,
		.dest_data_size = 2U,
		.source_burst_length = 2U,
		.dest_burst_length = 2U,
		.cyclic = 1U,
		.block_count = 2U,
		.head_block = &blocks[0],
		.user_data = stream,
		.dma_callback = gd32_i2s_dma_callback,
	};
	struct queue_item item;
	k_spinlock_key_t key;
	void *first[2];
	int ret;

	ret = gd32_i2s_first_blocks(stream, first);
	if (ret < 0) {
		return ret;
	}
	gd32_i2s_banks_init(&stream->banks, first[0], first[1]);

	for (uint8_t bank = 0U; bank < 2U; bank++) {
		const uint32_t memory = (uint32_t)gd32_i2s_banks_memory(&stream->banks, bank);

		blocks[bank].block_size = stream->cfg.block_size;
		blocks[bank].fifo_mode_control = dma->fifo_threshold;
		if (stream->dir == I2S_DIR_RX) {
			blocks[bank].source_address = (uint32_t)&SPI_DATA(reg);
			blocks[bank].source_addr_adj = DMA_ADDR_ADJ_NO_CHANGE;
			blocks[bank].dest_address = memory;
			blocks[bank].dest_addr_adj = DMA_ADDR_ADJ_INCREMENT;
		} else {
			blocks[bank].source_address = memory;
			blocks[bank].source_addr_adj = DMA_ADDR_ADJ_INCREMENT;
			blocks[bank].dest_address = (uint32_t)&SPI_DATA(reg);
			blocks[bank].dest_addr_adj = DMA_ADDR_ADJ_NO_CHANGE;
		}
	}
	blocks[0].next_block = &blocks[1];

	ret = dma_config(stream->dma_dev, stream->dma_channel, &dma_cfg);
	if (ret == 0) {
		ret = dma_start(stream->dma_dev, stream->dma_channel);
	}
	if (ret < 0) {
		LOG_ERR("%s %s DMA start failed: %d", stream->dev->name,
			gd32_i2s_dir_name(stream), ret);
		if (stream->dir == I2S_DIR_RX) {
			k_mem_slab_free(stream->cfg.mem_slab, first[0]);
			k_mem_slab_free(stream->cfg.mem_slab, first[1]);
		}
		gd32_i2s_banks_init(&stream->banks, NULL, NULL);
		return ret;
	}

	/* The transmit blocks were only looked at; the banks hold them now. */
	if (stream->dir == I2S_DIR_TX) {
		(void)k_msgq_get(stream->msgq, &item, K_NO_WAIT);
		if (first[1] != NULL) {
			(void)k_msgq_get(stream->msgq, &item, K_NO_WAIT);
		}
	}

	key = k_spin_lock(&data->lock);
	stream->retired = NULL;
	stream->drain = false;
	stream->finishing = false;
	stream->completed = false;
	stream->block_cycles =
		(uint32_t)((uint64_t)gd32_i2s_block_frames(stream) * sys_clock_hw_cycles_per_sec() /
			   stream->cfg.frame_clk_freq);
	stream->state = I2S_STATE_RUNNING;
	k_spin_unlock(&data->lock, key);

	if (stream->dir == I2S_DIR_RX) {
		gd32_i2s_rx_flush(reg);
	} else {
		gd32_i2s_tx_errors_clear(reg);
	}
	SPI_CTL1(reg) |= SPI_CTL1_ERRIE;
	spi_dma_enable(reg, gd32_i2s_stream_dma_request(stream));
	if (!gd32_i2s_arms_on_ws(stream)) {
		/* A controller's clock starts with it; in full duplex, with the transmitter. */
		key = k_spin_lock(&data->lock);
		if (stream->master && !(data->duplex && stream->dir == I2S_DIR_RX)) {
			gd32_i2s_clock_starts_locked(stream);
		}
		i2s_enable(reg);
		k_spin_unlock(&data->lock, key);
	}

	return 0;
}

static void i2s_gd32_isr(const struct device *dev)
{
	const struct i2s_gd32_config *cfg = dev->config;
	struct i2s_gd32_data *data = dev->data;
	const uint32_t rx_reg = gd32_i2s_rx_reg(dev);
	const uint32_t tx_stat = SPI_STAT(cfg->reg);
	const uint32_t rx_stat = SPI_STAT(rx_reg);
	const uint32_t tx_err_mask = SPI_STAT_TXURERR | SPI_STAT_CONFERR | SPI_STAT_FERR;
	const uint32_t rx_err_mask = SPI_STAT_RXORERR | SPI_STAT_CONFERR | SPI_STAT_FERR;
	volatile uint32_t clear_read = 0U;
	k_spinlock_key_t key;

	if ((tx_stat & tx_err_mask) == 0U && (rx_stat & rx_err_mask) == 0U) {
		return;
	}

	if ((rx_stat & SPI_STAT_RXORERR) != 0U) {
		clear_read = SPI_DATA(rx_reg);
		clear_read = SPI_STAT(rx_reg);
	}
	if ((tx_stat & SPI_STAT_TXURERR) != 0U) {
		clear_read = SPI_STAT(cfg->reg);
	}
	if ((rx_stat & SPI_STAT_FERR) != 0U) {
		SPI_STAT(rx_reg) &= ~SPI_STAT_FERR;
	}
	if ((tx_stat & SPI_STAT_FERR) != 0U) {
		SPI_STAT(cfg->reg) &= ~SPI_STAT_FERR;
	}
	(void)clear_read;

	key = k_spin_lock(&data->lock);
	if ((rx_stat & rx_err_mask) != 0U) {
		gd32_i2s_fail_locked(&data->rx);
	}
	if ((tx_stat & tx_err_mask) != 0U) {
		gd32_i2s_fail_locked(&data->tx);
	}
	k_spin_unlock(&data->lock, key);

	LOG_ERR("%s i2s error tx=0x%08x rx=0x%08x", dev->name, tx_stat, rx_stat);
}

/*
 * Configures one direction. In full duplex the receive direction is the I2S_ADD extension of
 * the transmitting peripheral, so the pair is only unconfigured together (`paired`). Called with
 * the API lock held.
 */
static int gd32_i2s_configure_one(const struct device *dev, enum i2s_dir dir,
				  const struct i2s_config *i2s_cfg, bool paired)
{
	const struct i2s_gd32_config *cfg = dev->config;
	struct i2s_gd32_data *data = dev->data;
	struct stream *stream = dir == I2S_DIR_RX ? &data->rx : &data->tx;
	struct stream *other = dir == I2S_DIR_RX ? &data->tx : &data->rx;
	const uint32_t add_reg = gd32_i2s_add_reg(cfg->reg);
	const bool has_add = add_reg != 0U;
	uint32_t prescaler = GD32_I2SPSC_DEFAULT_VALUE;
	uint32_t ckpl = I2S_CKPL_LOW;
	bool forming_duplex;
	bool use_extension;
	uint32_t frameformat;
	uint32_t mode;
	uint32_t std;
	bool master;
	int ret;

	if (stream->state != I2S_STATE_NOT_READY && stream->state != I2S_STATE_READY) {
		return -EINVAL;
	}
	if (gd32_i2s_active(other)) {
		return -EBUSY;
	}

	if (i2s_cfg->frame_clk_freq == 0U) {
		if (data->duplex && !paired) {
			return -EBUSY;
		}
		stream_queue_drop(stream);
		memset(&stream->cfg, 0, sizeof(stream->cfg));
		stream->state = I2S_STATE_NOT_READY;
		if (other->state == I2S_STATE_NOT_READY) {
			data->duplex = false;
			gd32_i2s_clock_release(dev);
		}
		return 0;
	}

	ret = gd32_i2s_validate_config(dir, i2s_cfg);
	if (ret < 0) {
		return ret;
	}

	forming_duplex = other->state != I2S_STATE_NOT_READY;
	if (forming_duplex && (!has_add || !gd32_i2s_configs_compatible(i2s_cfg, &other->cfg))) {
		LOG_ERR("%s duplex stream configurations differ", dev->name);
		return -EINVAL;
	}

	ret = gd32_std_from_config(i2s_cfg, &std);
	if (ret == 0) {
		ret = gd32_frameformat_from_config(i2s_cfg, &frameformat);
	}
	if (ret < 0) {
		return ret;
	}

	if ((i2s_cfg->format & I2S_FMT_CLK_FORMAT_MASK) & I2S_FMT_BIT_CLK_INV) {
		ckpl = I2S_CKPL_HIGH;
	}

	master = (i2s_cfg->options & I2S_OPT_FRAME_CLK_TARGET) == 0U;
	use_extension = has_add && dir == I2S_DIR_RX && forming_duplex;
	if (dir == I2S_DIR_TX) {
		mode = master ? I2S_MODE_MASTERTX : I2S_MODE_SLAVETX;
	} else {
		mode = master ? I2S_MODE_MASTERRX : I2S_MODE_SLAVERX;
	}

	/* Blocks queued under the old configuration go back to the slab they came from. */
	stream_queue_drop(stream);

	if (use_extension) {
		stream->actual_frame_rate_num = data->tx.actual_frame_rate_num;
		stream->actual_frame_rate_den = data->tx.actual_frame_rate_den;
	} else if (master) {
		ret = gd32_i2s_master_clock_apply(dev, i2s_cfg->frame_clk_freq, frameformat,
						  &prescaler, &stream->actual_frame_rate_num,
						  &stream->actual_frame_rate_den);
		if (ret < 0) {
			LOG_ERR("%s failed to configure master clock: %d", dev->name, ret);
			return ret;
		}
	} else {
		if (!forming_duplex) {
			gd32_i2s_clock_release(dev);
		}
		stream->actual_frame_rate_num = i2s_cfg->frame_clk_freq;
		stream->actual_frame_rate_den = 1U;
	}

	if (!use_extension) {
		spi_i2s_deinit(cfg->reg);
		i2s_init(cfg->reg, mode, std, ckpl);
		SPI_I2SPSC(cfg->reg) = prescaler;
		gd32_i2s_frameformat_apply(cfg->reg, frameformat);
	}

	if (has_add && dir == I2S_DIR_TX) {
		i2s_full_duplex_mode_config(add_reg, mode, std, ckpl, frameformat);
	}

	data->duplex = forming_duplex;
	stream->cfg = *i2s_cfg;
	stream->master = master;
	stream->state = I2S_STATE_READY;
	stream->frame_count = 0U;
	stream->timestamp_cycles = 0U;

	LOG_DBG("%s configured %s mode=%s rate=%uHz block=%u", dev->name,
		dir == I2S_DIR_TX ? "tx" : "rx", master ? "master" : "slave",
		i2s_cfg->frame_clk_freq, i2s_cfg->block_size);

	return 0;
}

/* Configures both directions as one full-duplex pair, or leaves both as they were. */
static int gd32_i2s_configure_both(const struct device *dev, const struct i2s_config *i2s_cfg)
{
	const struct i2s_gd32_config *cfg = dev->config;
	struct i2s_gd32_data *data = dev->data;
	const struct i2s_config disabled = {0};
	int ret;

	if (gd32_i2s_add_reg(cfg->reg) == 0U) {
		return -ENOTSUP;
	}
	if (gd32_i2s_active(&data->rx) || gd32_i2s_active(&data->tx) ||
	    data->rx.state == I2S_STATE_ERROR || data->tx.state == I2S_STATE_ERROR) {
		return -EBUSY;
	}
	if (i2s_cfg->frame_clk_freq != 0U) {
		ret = gd32_i2s_validate_config(I2S_DIR_TX, i2s_cfg);
		if (ret == 0) {
			ret = gd32_i2s_validate_config(I2S_DIR_RX, i2s_cfg);
		}
		if (ret < 0) {
			return ret;
		}
	}

	(void)gd32_i2s_configure_one(dev, I2S_DIR_RX, &disabled, true);
	(void)gd32_i2s_configure_one(dev, I2S_DIR_TX, &disabled, true);
	if (i2s_cfg->frame_clk_freq == 0U) {
		return 0;
	}

	ret = gd32_i2s_configure_one(dev, I2S_DIR_TX, i2s_cfg, true);
	if (ret == 0) {
		ret = gd32_i2s_configure_one(dev, I2S_DIR_RX, i2s_cfg, true);
		if (ret < 0) {
			(void)gd32_i2s_configure_one(dev, I2S_DIR_TX, &disabled, true);
		}
	}

	return ret;
}

static int i2s_gd32_configure(const struct device *dev, enum i2s_dir dir,
			      const struct i2s_config *i2s_cfg)
{
	struct i2s_gd32_data *data = dev->data;
	int ret;

	k_mutex_lock(&data->api_lock, K_FOREVER);
	if (dir == I2S_DIR_BOTH) {
		ret = gd32_i2s_configure_both(dev, i2s_cfg);
	} else if (dir == I2S_DIR_RX || dir == I2S_DIR_TX) {
		ret = gd32_i2s_configure_one(dev, dir, i2s_cfg, false);
	} else {
		ret = -EINVAL;
	}
	k_mutex_unlock(&data->api_lock);

	return ret;
}

static const struct i2s_config *i2s_gd32_config_get(const struct device *dev, enum i2s_dir dir)
{
	struct i2s_gd32_data *data = dev->data;
	struct stream *stream;

	if (dir == I2S_DIR_RX) {
		stream = &data->rx;
	} else if (dir == I2S_DIR_TX) {
		stream = &data->tx;
	} else {
		return NULL;
	}

	if (stream->state == I2S_STATE_NOT_READY) {
		return NULL;
	}

	return &stream->cfg;
}

static int i2s_gd32_timing_get(const struct device *dev, enum i2s_dir dir,
			       struct i2s_timing *timing)
{
	struct i2s_gd32_data *data = dev->data;
	struct stream *stream;
	k_spinlock_key_t key;

	if (timing == NULL) {
		return -EINVAL;
	}
	if (dir == I2S_DIR_RX) {
		stream = &data->rx;
	} else if (dir == I2S_DIR_TX) {
		stream = &data->tx;
	} else {
		return -EINVAL;
	}
	if (stream->state == I2S_STATE_NOT_READY) {
		return -ENODATA;
	}

	key = k_spin_lock(&data->lock);
	timing->frame_count = stream->frame_count;
	timing->timestamp_cycles = stream->timestamp_cycles;
	timing->frame_rate_num = stream->actual_frame_rate_num;
	timing->frame_rate_den = stream->actual_frame_rate_den;
	timing->timestamp_frequency = sys_clock_hw_cycles_per_sec();
	k_spin_unlock(&data->lock, key);

	return 0;
}

/* Returns a stream in the error state to READY with the configuration it had. */
static int gd32_i2s_prepare(const struct device *dev, struct stream *stream)
{
	const struct i2s_gd32_config *cfg = dev->config;
	const struct i2s_config saved = stream->cfg;
	int ret;

	ret = gd32_i2s_teardown(stream, I2S_STATE_NOT_READY);
	stream_queue_drop(stream);
	if (ret == 0) {
		ret = reset_line_toggle_dt(&cfg->reset);
	}
	if (ret == 0) {
		ret = gd32_i2s_configure_one(dev, stream->dir, &saved, true);
	}
	if (ret < 0) {
		stream->cfg = saved;
		stream->state = I2S_STATE_ERROR;
		LOG_ERR("%s %s prepare failed: %d", dev->name, gd32_i2s_dir_name(stream), ret);
	}

	return ret;
}

static int gd32_i2s_trigger_one(const struct device *dev, struct stream *stream,
				enum i2s_trigger_cmd cmd)
{
	struct i2s_gd32_data *data = dev->data;
	k_spinlock_key_t key;
	int ret;

	switch (cmd) {
	case I2S_TRIGGER_START:
		if (stream->state != I2S_STATE_READY) {
			return -EIO;
		}
		ret = gd32_i2s_stream_start(stream);
		if (ret == 0 && gd32_i2s_arms_on_ws(stream)) {
			gd32_i2s_enable_on_ws(stream, gd32_i2s_stream_reg(stream), 0U);
		}
		return ret;

	case I2S_TRIGGER_STOP:
	case I2S_TRIGGER_DRAIN:
		key = k_spin_lock(&data->lock);
		if (stream->state != I2S_STATE_RUNNING) {
			k_spin_unlock(&data->lock, key);
			return -EIO;
		}
		/* A receive stream drains as it stops: at the end of the current block. */
		stream->drain = cmd == I2S_TRIGGER_DRAIN && stream->dir == I2S_DIR_TX;
		stream->state = I2S_STATE_STOPPING;
		k_spin_unlock(&data->lock, key);
		return 0;

	case I2S_TRIGGER_DROP:
		if (stream->state == I2S_STATE_NOT_READY) {
			return -EIO;
		}
		ret = gd32_i2s_teardown(stream, I2S_STATE_READY);
		stream_queue_drop(stream);
		return ret;

	case I2S_TRIGGER_PREPARE:
		if (stream->state != I2S_STATE_ERROR) {
			return -EIO;
		}
		return gd32_i2s_prepare(dev, stream);

	default:
		return -EINVAL;
	}
}

/* Triggers a full-duplex pair. Both directions share the peripheral and change state together. */
static int gd32_i2s_trigger_both(const struct device *dev, enum i2s_trigger_cmd cmd)
{
	const struct i2s_gd32_config *cfg = dev->config;
	struct i2s_gd32_data *data = dev->data;
	const struct i2s_config disabled = {0};
	struct i2s_config rx_cfg;
	struct i2s_config tx_cfg;
	int ret;

	switch (cmd) {
	case I2S_TRIGGER_START:
		if (data->rx.state != I2S_STATE_READY || data->tx.state != I2S_STATE_READY) {
			return -EIO;
		}
		ret = gd32_i2s_stream_start(&data->rx);
		if (ret < 0) {
			return ret;
		}
		ret = gd32_i2s_stream_start(&data->tx);
		if (ret < 0) {
			(void)gd32_i2s_teardown(&data->rx, I2S_STATE_READY);
			return ret;
		}
		if (gd32_i2s_arms_on_ws(&data->tx)) {
			gd32_i2s_enable_on_ws(&data->tx, gd32_i2s_rx_reg(dev), cfg->reg);
		}
		return 0;

	case I2S_TRIGGER_STOP:
	case I2S_TRIGGER_DRAIN:
		if (data->rx.state != I2S_STATE_RUNNING || data->tx.state != I2S_STATE_RUNNING) {
			return -EIO;
		}
		ret = gd32_i2s_trigger_one(dev, &data->rx, cmd);
		return ret < 0 ? ret : gd32_i2s_trigger_one(dev, &data->tx, cmd);

	case I2S_TRIGGER_DROP:
		if (data->rx.state == I2S_STATE_NOT_READY || data->tx.state == I2S_STATE_NOT_READY) {
			return -EIO;
		}
		ret = gd32_i2s_trigger_one(dev, &data->rx, cmd);
		return ret < 0 ? ret : gd32_i2s_trigger_one(dev, &data->tx, cmd);

	case I2S_TRIGGER_PREPARE:
		if (data->rx.state != I2S_STATE_ERROR && data->tx.state != I2S_STATE_ERROR) {
			return -EIO;
		}
		rx_cfg = data->rx.cfg;
		tx_cfg = data->tx.cfg;
		ret = gd32_i2s_teardown(&data->rx, I2S_STATE_NOT_READY);
		if (ret == 0) {
			ret = gd32_i2s_teardown(&data->tx, I2S_STATE_NOT_READY);
		}
		stream_queue_drop(&data->rx);
		stream_queue_drop(&data->tx);
		if (ret == 0) {
			ret = reset_line_toggle_dt(&cfg->reset);
		}
		if (ret == 0) {
			data->duplex = false;
			ret = gd32_i2s_configure_one(dev, I2S_DIR_TX, &tx_cfg, true);
		}
		if (ret == 0) {
			ret = gd32_i2s_configure_one(dev, I2S_DIR_RX, &rx_cfg, true);
			if (ret < 0) {
				(void)gd32_i2s_configure_one(dev, I2S_DIR_TX, &disabled, true);
			}
		}
		if (ret < 0) {
			data->rx.cfg = rx_cfg;
			data->tx.cfg = tx_cfg;
			data->rx.state = I2S_STATE_ERROR;
			data->tx.state = I2S_STATE_ERROR;
			data->duplex = true;
			LOG_ERR("%s duplex prepare failed: %d", dev->name, ret);
		}
		return ret;

	default:
		return -EINVAL;
	}
}

static int i2s_gd32_trigger(const struct device *dev, enum i2s_dir dir, enum i2s_trigger_cmd cmd)
{
	const struct i2s_gd32_config *cfg = dev->config;
	struct i2s_gd32_data *data = dev->data;
	int ret;

	k_mutex_lock(&data->api_lock, K_FOREVER);
	if (dir == I2S_DIR_BOTH) {
		ret = gd32_i2s_add_reg(cfg->reg) != 0U ? gd32_i2s_trigger_both(dev, cmd) : -ENOTSUP;
	} else if (data->duplex) {
		LOG_ERR("%s configured duplex; trigger both directions together", dev->name);
		ret = -ENOTSUP;
	} else if (dir == I2S_DIR_RX || dir == I2S_DIR_TX) {
		ret = gd32_i2s_trigger_one(dev, dir == I2S_DIR_RX ? &data->rx : &data->tx, cmd);
	} else {
		ret = -EINVAL;
	}
	k_mutex_unlock(&data->api_lock);

	return ret;
}

static int i2s_gd32_read(const struct device *dev, void **mem_block, size_t *size)
{
	struct i2s_gd32_data *data = dev->data;
	struct stream *stream = &data->rx;
	int ret;

	if (stream->state == I2S_STATE_NOT_READY) {
		return -EIO;
	}

	/* After an error only the blocks already received remain. */
	ret = queue_get(stream->msgq, mem_block, size,
			stream->state == I2S_STATE_ERROR ? 0 : stream->cfg.timeout);
	if (ret == 0 && *mem_block != NULL) {
		return 0;
	}
	if (ret == 0 || stream->state == I2S_STATE_ERROR) {
		return -EIO;
	}

	return ret == -ENOMSG ? -EBUSY : ret;
}

static int i2s_gd32_write(const struct device *dev, void *mem_block, size_t size)
{
	struct i2s_gd32_data *data = dev->data;
	struct stream *stream = &data->tx;
	const size_t block_size = stream->cfg.block_size;
	k_spinlock_key_t key;
	int ret;

	if (stream->state != I2S_STATE_RUNNING && stream->state != I2S_STATE_READY) {
		return -EIO;
	}

	if (size == 0U || size > block_size || (size % gd32_i2s_frame_bytes(&stream->cfg)) != 0U) {
		return -EINVAL;
	}

	/* Both DMA banks have the configured length, so a shorter block ends in silence. */
	memset((uint8_t *)mem_block + size, 0, block_size - size);
	sys_cache_data_flush_range(mem_block, block_size);

	/* Queued and fed under the lock, so no completion can find the queue empty in between. */
	key = k_spin_lock(&data->lock);
	ret = queue_put(stream->msgq, mem_block, block_size, 0);
	if (ret == 0 && gd32_i2s_tx_feed_locked(stream) < 0) {
		gd32_i2s_fail_locked(stream);
	}
	k_spin_unlock(&data->lock, key);

	if (ret == -ENOMSG && stream->cfg.timeout != 0) {
		/*
		 * Room is made by completions, which may have found the queue empty meanwhile. A block
		 * accepted after the stream failed is released by I2S_TRIGGER_PREPARE.
		 */
		ret = queue_put(stream->msgq, mem_block, block_size, stream->cfg.timeout);
		key = k_spin_lock(&data->lock);
		if (ret == 0 && gd32_i2s_tx_feed_locked(stream) < 0) {
			gd32_i2s_fail_locked(stream);
		}
		k_spin_unlock(&data->lock, key);
	}

	return ret == -ENOMSG ? -EBUSY : ret;
}

static DEVICE_API(i2s, i2s_gd32_driver_api) = {
	.configure = i2s_gd32_configure,
	.config_get = i2s_gd32_config_get,
	.read = i2s_gd32_read,
	.write = i2s_gd32_write,
	.trigger = i2s_gd32_trigger,
	.timing_get = i2s_gd32_timing_get,
};

static int i2s_gd32_init(const struct device *dev)
{
	const struct i2s_gd32_config *cfg = dev->config;
	struct i2s_gd32_data *data = dev->data;
	uint32_t ch_filter;
	int ret;

	ret = clock_control_on(GD32_CLOCK_CONTROLLER, (clock_control_subsys_t)&cfg->clkid);
	if (ret < 0) {
		return ret;
	}

	ret = reset_line_toggle_dt(&cfg->reset);
	if (ret < 0) {
		return ret;
	}

	ret = pinctrl_apply_state(cfg->pcfg, PINCTRL_STATE_DEFAULT);
	if (ret != 0) {
		return ret;
	}

	if (!device_is_ready(cfg->dma_tx.dev) || !device_is_ready(cfg->dma_rx.dev)) {
		return -ENODEV;
	}

	if (cfg->ws_gpio.port != NULL && !gpio_is_ready_dt(&cfg->ws_gpio)) {
		return -ENODEV;
	}

	ch_filter = BIT(cfg->dma_tx.channel);
	ret = dma_request_channel(cfg->dma_tx.dev, &ch_filter);
	if (ret < 0) {
		return ret;
	}
	if ((uint32_t)ret != cfg->dma_tx.channel) {
		dma_release_channel(cfg->dma_tx.dev, ret);
		return -EINVAL;
	}

	ch_filter = BIT(cfg->dma_rx.channel);
	ret = dma_request_channel(cfg->dma_rx.dev, &ch_filter);
	if (ret < 0) {
		dma_release_channel(cfg->dma_tx.dev, cfg->dma_tx.channel);
		return ret;
	}
	if ((uint32_t)ret != cfg->dma_rx.channel) {
		dma_release_channel(cfg->dma_rx.dev, ret);
		dma_release_channel(cfg->dma_tx.dev, cfg->dma_tx.channel);
		return -EINVAL;
	}

	k_mutex_init(&data->api_lock);

	data->tx.dev = dev;
	data->tx.dir = I2S_DIR_TX;
	data->tx.state = I2S_STATE_NOT_READY;
	data->tx.dma_dev = cfg->dma_tx.dev;
	data->tx.dma_channel = cfg->dma_tx.channel;
	data->tx.dma_slot = cfg->dma_tx.slot;
	data->tx.dma_priority = GD32_DMA_CONFIG_PRIORITY(cfg->dma_tx.config);
	k_work_init_delayable(&data->tx.stop_work, gd32_i2s_stop_work_handler);

	data->rx.dev = dev;
	data->rx.dir = I2S_DIR_RX;
	data->rx.state = I2S_STATE_NOT_READY;
	data->rx.dma_dev = cfg->dma_rx.dev;
	data->rx.dma_channel = cfg->dma_rx.channel;
	data->rx.dma_slot = cfg->dma_rx.slot;
	data->rx.dma_priority = GD32_DMA_CONFIG_PRIORITY(cfg->dma_rx.config);
	k_work_init_delayable(&data->rx.stop_work, gd32_i2s_stop_work_handler);

	cfg->irq_configure(dev);
	return 0;
}

#define DMA_INITIALIZER(idx, dir)                                                                  \
	{                                                                                          \
		.dev = DEVICE_DT_GET(DT_INST_DMAS_CTLR_BY_NAME(idx, dir)),                         \
		.channel = DT_INST_DMAS_CELL_BY_NAME(idx, dir, channel),                           \
		.slot = DT_INST_DMAS_CELL_BY_NAME(idx, dir, slot),                                 \
		.config = DT_INST_DMAS_CELL_BY_NAME(idx, dir, config),                             \
		.fifo_threshold = GD32_DMA_DT_FIFO_MODE(                                           \
			DT_INST_DMAS_CELL_BY_NAME(idx, dir, fifo_threshold)),                      \
	}

#define I2S_GD32_INIT(inst)                                                                        \
	BUILD_ASSERT(DT_INST_DMAS_HAS_NAME(inst, rx), "GD32 I2S requires RX DMA");                 \
	BUILD_ASSERT(DT_INST_DMAS_HAS_NAME(inst, tx), "GD32 I2S requires TX DMA");                 \
	PINCTRL_DT_INST_DEFINE(inst);                                                              \
	K_MSGQ_DEFINE(rx_##inst##_queue, sizeof(struct queue_item),                                \
		      CONFIG_I2S_GD32_RX_BLOCK_COUNT, 4);                                          \
	K_MSGQ_DEFINE(tx_##inst##_queue, sizeof(struct queue_item),                                \
		      CONFIG_I2S_GD32_TX_BLOCK_COUNT, 4);                                          \
	static void i2s_gd32_irq_configure_##inst(const struct device *dev)                        \
	{                                                                                          \
		IRQ_CONNECT(DT_INST_IRQN(inst), DT_INST_IRQ(inst, priority), i2s_gd32_isr,         \
			    DEVICE_DT_INST_GET(inst), 0);                                          \
		irq_enable(DT_INST_IRQN(inst));                                                    \
		ARG_UNUSED(dev);                                                                   \
	}                                                                                          \
	static struct i2s_gd32_data i2s_gd32_data_##inst = {                                       \
		.rx = {.msgq = &rx_##inst##_queue, .state = I2S_STATE_NOT_READY},                  \
		.tx = {.msgq = &tx_##inst##_queue, .state = I2S_STATE_NOT_READY},                  \
	};                                                                                         \
	static const struct i2s_gd32_config i2s_gd32_config_##inst = {                             \
		.reg = DT_INST_REG_ADDR(inst),                                                     \
		.clkid = DT_INST_CLOCKS_CELL(inst, id),                                            \
		.reset = RESET_DT_SPEC_INST_GET(inst),                                             \
		.pcfg = PINCTRL_DT_INST_DEV_CONFIG_GET(inst),                                      \
		.ws_gpio = GPIO_DT_SPEC_INST_GET_OR(inst, ws_gpios, {0}),                          \
		.mck_enabled = DT_INST_PROP_OR(inst, mck_enabled, 0),                              \
		.irq_configure = i2s_gd32_irq_configure_##inst,                                    \
		.dma_rx = DMA_INITIALIZER(inst, rx),                                               \
		.dma_tx = DMA_INITIALIZER(inst, tx),                                               \
	};                                                                                         \
	DEVICE_DT_INST_DEFINE(inst, i2s_gd32_init, NULL, &i2s_gd32_data_##inst,                    \
			      &i2s_gd32_config_##inst, POST_KERNEL, CONFIG_I2S_INIT_PRIORITY,      \
			      &i2s_gd32_driver_api)

DT_INST_FOREACH_STATUS_OKAY(I2S_GD32_INIT)
