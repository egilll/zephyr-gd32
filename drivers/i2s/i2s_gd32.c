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
#include <zephyr/drivers/i2s.h>
#include <zephyr/drivers/pinctrl.h>
#include <zephyr/drivers/reset.h>
#include <zephyr/irq.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/clock.h>
#include <zephyr/sys/util.h>

#include <gd32_spi.h>

LOG_MODULE_REGISTER(i2s_gd32, CONFIG_I2S_LOG_LEVEL);

#define GD32_I2SPSC_DEFAULT_VALUE 0x00000002U
#define GD32_I2S_STOP_TIMEOUT_MS  10U

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
	struct dma_config dma_cfg;
	struct dma_block_config dma_blk;
	struct dma_block_config dma_blk_alt;
	void *mem_block;
	size_t mem_block_size;
	void *mem_block_alt;
	size_t mem_block_alt_size;
	uint64_t actual_frame_rate_num;
	uint64_t actual_frame_rate_den;
	uint64_t frame_count;
	uint64_t timestamp_cycles;
	struct k_work_delayable stop_work;
	int64_t stop_deadline;
	bool master;
	bool starting;
	bool tx_stop_for_drain;
	bool tx_switch_buffer;
	bool tx_drain_final;
	bool rx_switch_buffer;
};

struct i2s_gd32_data {
	struct stream rx;
	struct stream tx;
	bool duplex;
	bool paired_transition;
	bool clock_claimed;
};

static inline bool queue_is_empty(struct k_msgq *q)
{
	return k_msgq_num_used_get(q) == 0;
}

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

	if (stream->cfg.mem_slab == NULL) {
		return;
	}

	while (queue_get(stream->msgq, &mem_block, &size, 0) == 0) {
		k_mem_slab_free(stream->cfg.mem_slab, mem_block);
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

static void gd32_i2s_rx_errors_clear(uint32_t reg)
{
	volatile uint32_t clear_read;

	if ((SPI_STAT(reg) & SPI_STAT_RXORERR) != 0U) {
		clear_read = SPI_DATA(reg);
		clear_read = SPI_STAT(reg);
		(void)clear_read;
	}
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

static int gd32_i2s_rx_stream_disable(struct stream *stream)
{
	const uint32_t rx_reg = gd32_i2s_rx_reg(stream->dev);
	int ret;

	SPI_CTL1(rx_reg) &= ~SPI_CTL1_ERRIE;
	spi_dma_disable(rx_reg, SPI_DMA_RECEIVE);
	i2s_disable(rx_reg);
	ret = dma_stop(stream->dma_dev, stream->dma_channel);
	if (ret < 0) {
		stream->state = I2S_STATE_ERROR;
		return ret;
	}

	if (stream->mem_block != NULL) {
		k_mem_slab_free(stream->cfg.mem_slab, stream->mem_block);
		stream->mem_block = NULL;
	}
	if (stream->mem_block_alt != NULL) {
		k_mem_slab_free(stream->cfg.mem_slab, stream->mem_block_alt);
		stream->mem_block_alt = NULL;
	}

	stream->mem_block_size = 0U;
	stream->mem_block_alt_size = 0U;
	stream->rx_switch_buffer = false;
	stream->starting = false;
	return 0;
}

static int gd32_i2s_tx_stream_disable(struct stream *stream)
{
	const struct i2s_gd32_config *cfg = stream->dev->config;
	int ret;

	SPI_CTL1(cfg->reg) &= ~SPI_CTL1_ERRIE;
	spi_dma_disable(cfg->reg, SPI_DMA_TRANSMIT);
	i2s_disable(cfg->reg);
	(void)k_work_cancel_delayable(&stream->stop_work);
	ret = dma_stop(stream->dma_dev, stream->dma_channel);
	if (ret < 0) {
		stream->state = I2S_STATE_ERROR;
		return ret;
	}

	if (stream->mem_block != NULL) {
		k_mem_slab_free(stream->cfg.mem_slab, stream->mem_block);
		stream->mem_block = NULL;
	}
	if (stream->mem_block_alt != NULL) {
		k_mem_slab_free(stream->cfg.mem_slab, stream->mem_block_alt);
		stream->mem_block_alt = NULL;
	}

	stream->mem_block_size = 0U;
	stream->mem_block_alt_size = 0U;
	stream->tx_switch_buffer = false;
	stream->tx_drain_final = false;
	stream->starting = false;
	return 0;
}

static int gd32_i2s_stream_disable(struct stream *stream);

static void gd32_i2s_stream_fail(struct stream *stream)
{
	struct i2s_gd32_data *data = stream->dev->data;

	if (data->duplex) {
		const struct i2s_gd32_config *cfg = stream->dev->config;
		const uint32_t rx_reg = gd32_i2s_rx_reg(stream->dev);

		SPI_CTL1(rx_reg) &= ~SPI_CTL1_ERRIE;
		SPI_CTL1(cfg->reg) &= ~SPI_CTL1_ERRIE;
		spi_dma_disable(rx_reg, SPI_DMA_RECEIVE);
		spi_dma_disable(cfg->reg, SPI_DMA_TRANSMIT);
		i2s_disable(rx_reg);
		i2s_disable(cfg->reg);
		data->rx.state = I2S_STATE_ERROR;
		data->tx.state = I2S_STATE_ERROR;
		data->rx.starting = false;
		data->tx.starting = false;
	} else {
		const struct i2s_gd32_config *cfg = stream->dev->config;
		const uint32_t reg = stream->dir == I2S_DIR_RX ? gd32_i2s_rx_reg(stream->dev)
								 : cfg->reg;

		SPI_CTL1(reg) &= ~SPI_CTL1_ERRIE;
		spi_dma_disable(reg, stream->dir == I2S_DIR_RX ? SPI_DMA_RECEIVE
								      : SPI_DMA_TRANSMIT);
		i2s_disable(reg);
		stream->state = I2S_STATE_ERROR;
		stream->starting = false;
	}
}

static void gd32_i2s_stop_work_handler(struct k_work *work)
{
	struct k_work_delayable *dwork = k_work_delayable_from_work(work);
	struct stream *stream = CONTAINER_OF(dwork, struct stream, stop_work);
	const struct i2s_gd32_config *cfg = stream->dev->config;
	uint32_t stat;
	unsigned int key;
	int ret;

	if (stream->dir == I2S_DIR_TX) {
		stat = SPI_STAT(cfg->reg);
	} else {
		stat = SPI_STAT_TBE;
	}

	if (stream->dir == I2S_DIR_TX &&
	    ((stat & SPI_STAT_TBE) == 0U || (stat & SPI_STAT_TRANS) != 0U)) {
		if (k_uptime_get() < stream->stop_deadline) {
			k_work_reschedule(&stream->stop_work, K_USEC(20));
			return;
		}
		LOG_WRN("%s tx did not become idle before stop", stream->dev->name);
	}

	ret = gd32_i2s_stream_disable(stream);
	key = irq_lock();
	if (ret == 0 && stream->state == I2S_STATE_STOPPING) {
		stream->state = I2S_STATE_READY;
	} else if (ret < 0) {
		stream->state = I2S_STATE_ERROR;
	}
	irq_unlock(key);
}

static void gd32_i2s_tx_finish_async(struct stream *stream)
{
	const struct i2s_gd32_config *cfg = stream->dev->config;

	SPI_CTL1(cfg->reg) &= ~SPI_CTL1_ERRIE;
	spi_dma_disable(cfg->reg, SPI_DMA_TRANSMIT);
	stream->stop_deadline = k_uptime_get() + GD32_I2S_STOP_TIMEOUT_MS;
	k_work_reschedule(&stream->stop_work, K_NO_WAIT);
}

static void gd32_i2s_rx_finish_async(struct stream *stream)
{
	const uint32_t rx_reg = gd32_i2s_rx_reg(stream->dev);

	SPI_CTL1(rx_reg) &= ~SPI_CTL1_ERRIE;
	spi_dma_disable(rx_reg, SPI_DMA_RECEIVE);
	stream->stop_deadline = k_uptime_get() + GD32_I2S_STOP_TIMEOUT_MS;
	k_work_reschedule(&stream->stop_work, K_NO_WAIT);
}

static int gd32_i2s_stream_disable(struct stream *stream)
{
	if (stream->dir == I2S_DIR_RX) {
		return gd32_i2s_rx_stream_disable(stream);
	}

	return gd32_i2s_tx_stream_disable(stream);
}

static bool gd32_i2s_dma_busy(struct stream *stream)
{
	struct dma_status stat;

	if (dma_get_status(stream->dma_dev, stream->dma_channel, &stat) != 0) {
		return true;
	}

	return stat.busy;
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

static int gd32_i2s_validate_config(const struct i2s_config *cfg)
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

	if (cfg->block_size == 0U) {
		return -EINVAL;
	}

	if ((cfg->block_size % gd32_i2s_frame_bytes(cfg)) != 0U) {
		return -EINVAL;
	}
	if (cfg->block_size > cfg->mem_slab->info.block_size) {
		return -EINVAL;
	}

	if (cfg->timeout < -1) {
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

static int gd32_i2s_tx_acquire_block_to(struct stream *stream, void **block, size_t *size)
{
	int ret;

	ret = queue_get(stream->msgq, block, size, 0);
	if (ret < 0) {
		return ret;
	}

	if ((*size == 0U) || (*size > stream->cfg.block_size)) {
		size_t invalid_size = *size;

		k_mem_slab_free(stream->cfg.mem_slab, *block);
		*block = NULL;
		*size = 0U;
		LOG_ERR("%s tx invalid block size %u", stream->dev->name, (uint32_t)invalid_size);
		return -EINVAL;
	}

	if ((*size % 4U) != 0U) {
		size_t invalid_size = *size;

		k_mem_slab_free(stream->cfg.mem_slab, *block);
		*block = NULL;
		*size = 0U;
		LOG_ERR("%s tx block size %u not aligned", stream->dev->name,
			(uint32_t)invalid_size);
		return -EINVAL;
	}

	sys_cache_data_flush_range(*block, *size);

	return 0;
}

static int gd32_i2s_tx_acquire_block(struct stream *stream)
{
	return gd32_i2s_tx_acquire_block_to(stream, &stream->mem_block, &stream->mem_block_size);
}

static bool gd32_i2s_rx_queue_latest(struct stream *stream, void *block, size_t size)
{
	void *oldest;
	size_t oldest_size;

	if (queue_put(stream->msgq, block, size, 0) == 0) {
		return true;
	}

	if (queue_get(stream->msgq, &oldest, &oldest_size, 0) == 0) {
		k_mem_slab_free(stream->cfg.mem_slab, oldest);
		if (queue_put(stream->msgq, block, size, 0) == 0) {
			return true;
		}
	}

	k_mem_slab_free(stream->cfg.mem_slab, block);
	return false;
}

static void *gd32_i2s_rx_exchange_single(struct stream *stream, void *completed_block,
					 size_t completed_size)
{
	void *replacement = NULL;
	size_t replacement_size;

	if (k_mem_slab_alloc(stream->cfg.mem_slab, &replacement, K_NO_WAIT) != 0) {
		if (queue_get(stream->msgq, &replacement, &replacement_size, 0) != 0) {
			return completed_block;
		}
	}

	if (queue_put(stream->msgq, completed_block, completed_size, 0) == 0) {
		return replacement;
	}

	void *oldest;
	size_t oldest_size;

	if (queue_get(stream->msgq, &oldest, &oldest_size, 0) == 0) {
		k_mem_slab_free(stream->cfg.mem_slab, replacement);
		replacement = oldest;
		if (queue_put(stream->msgq, completed_block, completed_size, 0) == 0) {
			return replacement;
		}
	}

	k_mem_slab_free(stream->cfg.mem_slab, replacement);
	return completed_block;
}

static int gd32_i2s_inactive_block(struct stream *stream, void ***block, size_t **size)
{
	uint8_t active_bank;
	int ret;

	ret = dma_gd32_switch_buffer_active(stream->dma_dev, stream->dma_channel, &active_bank);
	if (ret < 0) {
		return ret;
	}

	if (active_bank == 0U) {
		*block = &stream->mem_block_alt;
		*size = &stream->mem_block_alt_size;
	} else {
		*block = &stream->mem_block;
		*size = &stream->mem_block_size;
	}

	return 0;
}

static void gd32_i2s_rx_publish(struct stream *stream, const void *completed_block,
				 size_t completed_size)
{
	void *output = NULL;
	size_t unused_size;
	uint8_t active_bank;

	if (k_mem_slab_alloc(stream->cfg.mem_slab, &output, K_NO_WAIT) != 0 &&
	    queue_get(stream->msgq, &output, &unused_size, 0) != 0) {
		return;
	}

	memcpy(output, completed_block, completed_size);

	if (dma_gd32_switch_buffer_active(stream->dma_dev, stream->dma_channel, &active_bank) < 0) {
		k_mem_slab_free(stream->cfg.mem_slab, output);
		return;
	}
	if ((active_bank == 0U && completed_block == stream->mem_block) ||
	    (active_bank == 1U && completed_block == stream->mem_block_alt)) {
		/* DMA caught the CPU while it copied the inactive bank. Discard the interval. */
		k_mem_slab_free(stream->cfg.mem_slab, output);
		return;
	}

	(void)gd32_i2s_rx_queue_latest(stream, output, completed_size);
}

static void gd32_i2s_tx_switch_callback(struct stream *stream)
{
	void **completed_block;
	size_t *completed_size;
	void *replacement = NULL;
	size_t replacement_size = 0U;
	uint8_t active_bank;
	int ret;

	ret = gd32_i2s_inactive_block(stream, &completed_block, &completed_size);
	if (ret < 0) {
		LOG_ERR("%s tx active bank unavailable: %d", stream->dev->name, ret);
		gd32_i2s_stream_fail(stream);
		return;
	}

	__ASSERT_NO_MSG(*completed_block != NULL);
	stream->frame_count += *completed_size / gd32_i2s_frame_bytes(&stream->cfg);
	stream->timestamp_cycles = k_cycle_get_64();

	if (stream->tx_drain_final ||
	    (stream->state == I2S_STATE_STOPPING && !stream->tx_stop_for_drain)) {
		gd32_i2s_tx_finish_async(stream);
		return;
	}

	ret = gd32_i2s_tx_acquire_block_to(stream, &replacement, &replacement_size);
	if (ret < 0) {
		if (stream->state == I2S_STATE_STOPPING && stream->tx_stop_for_drain) {
			memset(*completed_block, 0, *completed_size);
			sys_cache_data_flush_range(*completed_block, *completed_size);
			stream->tx_drain_final = true;
			return;
		}

		memset(*completed_block, 0, stream->cfg.block_size);
		sys_cache_data_flush_range(*completed_block, stream->cfg.block_size);
		*completed_size = stream->cfg.block_size;
		return;
	}

	if (replacement_size != stream->cfg.block_size) {
		k_mem_slab_free(stream->cfg.mem_slab, replacement);
		memset(*completed_block, 0, stream->cfg.block_size);
		sys_cache_data_flush_range(*completed_block, stream->cfg.block_size);
		*completed_size = stream->cfg.block_size;
		return;
	}

	memcpy(*completed_block, replacement, replacement_size);
	sys_cache_data_flush_range(*completed_block, replacement_size);
	k_mem_slab_free(stream->cfg.mem_slab, replacement);
	*completed_size = replacement_size;

	ret = dma_gd32_switch_buffer_active(stream->dma_dev, stream->dma_channel, &active_bank);
	if (ret < 0 || (active_bank == 0U && *completed_block == stream->mem_block) ||
	    (active_bank == 1U && *completed_block == stream->mem_block_alt)) {
		LOG_ERR("%s tx DMA bank changed while being refilled", stream->dev->name);
		gd32_i2s_stream_fail(stream);
	}
}

static void gd32_i2s_rx_switch_callback(struct stream *stream)
{
	void **completed_block;
	size_t *completed_size;
	int ret;

	ret = gd32_i2s_inactive_block(stream, &completed_block, &completed_size);
	if (ret < 0) {
		LOG_ERR("%s rx active bank unavailable: %d", stream->dev->name, ret);
		gd32_i2s_stream_fail(stream);
		return;
	}

	__ASSERT_NO_MSG(*completed_block != NULL);
	stream->frame_count += *completed_size / gd32_i2s_frame_bytes(&stream->cfg);
	stream->timestamp_cycles = k_cycle_get_64();
	sys_cache_data_invd_range(*completed_block, *completed_size);

	if (stream->state == I2S_STATE_STOPPING) {
		gd32_i2s_rx_publish(stream, *completed_block, *completed_size);
		gd32_i2s_rx_finish_async(stream);
		return;
	}

	gd32_i2s_rx_publish(stream, *completed_block, *completed_size);
}

static void gd32_i2s_dma_callback(const struct device *dma_dev, void *arg, uint32_t channel,
				  int status)
{
	struct stream *stream = arg;
	const struct device *dev = stream->dev;
	const struct i2s_gd32_config *cfg = dev->config;
	int ret;

	ARG_UNUSED(dma_dev);
	ARG_UNUSED(channel);

	/* dma_stop() can leave one already-pending completion in the IRQ path. */
	if (stream->state != I2S_STATE_RUNNING && stream->state != I2S_STATE_STOPPING &&
	    !stream->starting) {
		return;
	}
	if (stream->mem_block == NULL) {
		__ASSERT_NO_MSG(stream->state == I2S_STATE_STOPPING);
		return;
	}

	if (stream->dir == I2S_DIR_RX) {
		const uint32_t rx_reg = gd32_i2s_rx_reg(dev);

		if (status < 0) {
			LOG_ERR("%s dma rx error: ch=%u status=%d", dev->name, stream->dma_channel,
				status);
			gd32_i2s_stream_fail(stream);
			return;
		}

		if (stream->rx_switch_buffer) {
			gd32_i2s_rx_switch_callback(stream);
			return;
		}

		void *filled_block = stream->mem_block;

		stream->mem_block = NULL;
		stream->frame_count += stream->cfg.block_size / gd32_i2s_frame_bytes(&stream->cfg);
		stream->timestamp_cycles = k_cycle_get_64();
		sys_cache_data_invd_range(filled_block, stream->cfg.block_size);

		if (stream->state == I2S_STATE_STOPPING) {
			(void)gd32_i2s_rx_queue_latest(stream, filled_block,
						       stream->cfg.block_size);
			gd32_i2s_rx_finish_async(stream);
			return;
		}

		stream->mem_block =
			gd32_i2s_rx_exchange_single(stream, filled_block, stream->cfg.block_size);
		stream->mem_block_size = stream->cfg.block_size;
		sys_cache_data_invd_range(stream->mem_block, stream->cfg.block_size);
		ret = dma_reload(stream->dma_dev, stream->dma_channel, (uint32_t)&SPI_DATA(rx_reg),
				 (uint32_t)stream->mem_block, stream->cfg.block_size);
		if (ret < 0) {
			LOG_ERR("%s rx dma_reload failed: %d", dev->name, ret);
			gd32_i2s_stream_fail(stream);
			return;
		}

		ret = dma_start(stream->dma_dev, stream->dma_channel);
		if (ret < 0) {
			LOG_ERR("%s rx dma_start failed: %d", dev->name, ret);
			gd32_i2s_stream_fail(stream);
		}

		return;
	}

	if (status < 0) {
		LOG_ERR("%s dma tx error: ch=%u status=%d", dev->name, stream->dma_channel, status);
		gd32_i2s_stream_fail(stream);
		return;
	}

	if (stream->tx_switch_buffer) {
		gd32_i2s_tx_switch_callback(stream);
		return;
	}

	stream->frame_count += stream->mem_block_size / gd32_i2s_frame_bytes(&stream->cfg);
	stream->timestamp_cycles = k_cycle_get_64();

	if (stream->state == I2S_STATE_STOPPING && !stream->tx_stop_for_drain) {
		k_mem_slab_free(stream->cfg.mem_slab, stream->mem_block);
		stream->mem_block = NULL;
		stream->mem_block_size = 0U;
		gd32_i2s_tx_finish_async(stream);
		return;
	}

	void *replacement = NULL;
	size_t replacement_size = 0U;

	ret = gd32_i2s_tx_acquire_block_to(stream, &replacement, &replacement_size);
	if (ret < 0) {
		if (stream->state == I2S_STATE_STOPPING && stream->tx_stop_for_drain) {
			k_mem_slab_free(stream->cfg.mem_slab, stream->mem_block);
			stream->mem_block = NULL;
			stream->mem_block_size = 0U;
			gd32_i2s_tx_finish_async(stream);
		} else {
			memset(stream->mem_block, 0, stream->cfg.block_size);
			sys_cache_data_flush_range(stream->mem_block, stream->cfg.block_size);
			stream->mem_block_size = stream->cfg.block_size;
		}
	} else {
		k_mem_slab_free(stream->cfg.mem_slab, stream->mem_block);
		stream->mem_block = replacement;
		stream->mem_block_size = replacement_size;
	}
	if (stream->mem_block == NULL) {
		return;
	}

	ret = dma_reload(stream->dma_dev, stream->dma_channel, (uint32_t)stream->mem_block,
			 (uint32_t)&SPI_DATA(cfg->reg), stream->mem_block_size);
	if (ret < 0) {
		LOG_ERR("%s tx dma_reload failed: %d", dev->name, ret);
		gd32_i2s_stream_fail(stream);
		return;
	}

	ret = dma_start(stream->dma_dev, stream->dma_channel);
	if (ret < 0) {
		LOG_ERR("%s tx dma_start failed: %d", dev->name, ret);
		gd32_i2s_stream_fail(stream);
	}
}

static int gd32_i2s_stream_start(struct stream *stream)
{
	const struct i2s_gd32_config *cfg = stream->dev->config;
	int ret;

	memset(&stream->dma_cfg, 0, sizeof(stream->dma_cfg));
	memset(&stream->dma_blk, 0, sizeof(stream->dma_blk));
	memset(&stream->dma_blk_alt, 0, sizeof(stream->dma_blk_alt));

	stream->dma_cfg.dma_callback = gd32_i2s_dma_callback;
	stream->dma_cfg.user_data = stream;
	stream->dma_cfg.block_count = 1U;
	stream->dma_cfg.head_block = &stream->dma_blk;
	stream->dma_cfg.dma_slot = stream->dma_slot;
	stream->dma_cfg.channel_priority = stream->dma_priority;
	stream->dma_cfg.source_burst_length = 2U;
	stream->dma_cfg.dest_burst_length = 2U;
	stream->dma_cfg.source_data_size = 2U;
	stream->dma_cfg.dest_data_size = 2U;
	stream->dma_blk.fifo_mode_control =
		stream->dir == I2S_DIR_RX ? cfg->dma_rx.fifo_threshold : cfg->dma_tx.fifo_threshold;

	if (stream->dir == I2S_DIR_RX) {
		const uint32_t rx_reg = gd32_i2s_rx_reg(stream->dev);

		ret = k_mem_slab_alloc(stream->cfg.mem_slab, &stream->mem_block, K_NO_WAIT);
		if (ret < 0) {
			LOG_ERR("%s rx start failed: no slab block", stream->dev->name);
			return -ENOMEM;
		}

		stream->mem_block_size = stream->cfg.block_size;
		ret = k_mem_slab_alloc(stream->cfg.mem_slab, &stream->mem_block_alt, K_NO_WAIT);
		if (ret < 0) {
			LOG_ERR("%s rx start failed: no second slab block", stream->dev->name);
			gd32_i2s_rx_stream_disable(stream);
			return -ENOMEM;
		}
		stream->mem_block_alt_size = stream->cfg.block_size;
		stream->dma_cfg.channel_direction = PERIPHERAL_TO_MEMORY;
		stream->dma_cfg.block_count = 2U;
		stream->dma_cfg.cyclic = true;
		stream->dma_blk.source_address = (uint32_t)&SPI_DATA(rx_reg);
		stream->dma_blk.dest_address = (uint32_t)stream->mem_block;
		stream->dma_blk.source_addr_adj = DMA_ADDR_ADJ_NO_CHANGE;
		stream->dma_blk.dest_addr_adj = DMA_ADDR_ADJ_INCREMENT;
		stream->dma_blk.block_size = stream->cfg.block_size;
		stream->dma_blk.next_block = &stream->dma_blk_alt;
		stream->dma_blk_alt.source_address = (uint32_t)&SPI_DATA(rx_reg);
		stream->dma_blk_alt.dest_address = (uint32_t)stream->mem_block_alt;
		stream->dma_blk_alt.source_addr_adj = DMA_ADDR_ADJ_NO_CHANGE;
		stream->dma_blk_alt.dest_addr_adj = DMA_ADDR_ADJ_INCREMENT;
		stream->dma_blk_alt.block_size = stream->cfg.block_size;
		stream->dma_blk_alt.fifo_mode_control = stream->dma_blk.fifo_mode_control;
		stream->rx_switch_buffer = true;
		sys_cache_data_invd_range(stream->mem_block, stream->cfg.block_size);
		sys_cache_data_invd_range(stream->mem_block_alt, stream->cfg.block_size);

		ret = dma_config(stream->dma_dev, stream->dma_channel, &stream->dma_cfg);
		if (ret < 0) {
			LOG_ERR("%s rx dma_config failed: %d", stream->dev->name, ret);
			gd32_i2s_rx_stream_disable(stream);
			return ret;
		}

		ret = dma_start(stream->dma_dev, stream->dma_channel);
		if (ret < 0) {
			LOG_ERR("%s rx dma_start failed: %d", stream->dev->name, ret);
			gd32_i2s_rx_stream_disable(stream);
			return ret;
		}

		gd32_i2s_rx_errors_clear(rx_reg);
		SPI_CTL1(rx_reg) |= SPI_CTL1_ERRIE;
		spi_dma_enable(rx_reg, SPI_DMA_RECEIVE);
		i2s_enable(rx_reg);

		LOG_DBG("%s rx stream started", stream->dev->name);
		return 0;
	}

	ret = gd32_i2s_tx_acquire_block(stream);
	if (ret < 0) {
		LOG_ERR("%s tx start failed: no queued buffer", stream->dev->name);
		return -EIO;
	}
	ret = gd32_i2s_tx_acquire_block_to(stream, &stream->mem_block_alt,
					   &stream->mem_block_alt_size);
	if (ret == 0 && (stream->mem_block_size != stream->cfg.block_size ||
			 stream->mem_block_alt_size != stream->cfg.block_size)) {
		ret = queue_put(stream->msgq, stream->mem_block_alt, stream->mem_block_alt_size, 0);
		if (ret < 0) {
			k_mem_slab_free(stream->cfg.mem_slab, stream->mem_block_alt);
		}
		stream->mem_block_alt = NULL;
		stream->mem_block_alt_size = 0U;
	}

	stream->dma_cfg.channel_direction = MEMORY_TO_PERIPHERAL;
	stream->dma_blk.source_address = (uint32_t)stream->mem_block;
	stream->dma_blk.dest_address = (uint32_t)&SPI_DATA(cfg->reg);
	stream->dma_blk.source_addr_adj = DMA_ADDR_ADJ_INCREMENT;
	stream->dma_blk.dest_addr_adj = DMA_ADDR_ADJ_NO_CHANGE;
	stream->dma_blk.block_size = stream->mem_block_size;
	if (stream->mem_block_alt != NULL) {
		stream->dma_cfg.block_count = 2U;
		stream->dma_cfg.cyclic = true;
		stream->dma_blk.next_block = &stream->dma_blk_alt;
		stream->dma_blk_alt.source_address = (uint32_t)stream->mem_block_alt;
		stream->dma_blk_alt.dest_address = (uint32_t)&SPI_DATA(cfg->reg);
		stream->dma_blk_alt.source_addr_adj = DMA_ADDR_ADJ_INCREMENT;
		stream->dma_blk_alt.dest_addr_adj = DMA_ADDR_ADJ_NO_CHANGE;
		stream->dma_blk_alt.block_size = stream->mem_block_alt_size;
		stream->dma_blk_alt.fifo_mode_control = stream->dma_blk.fifo_mode_control;
		stream->tx_switch_buffer = true;
		stream->tx_drain_final = false;
	}

	ret = dma_config(stream->dma_dev, stream->dma_channel, &stream->dma_cfg);
	if (ret < 0) {
		LOG_ERR("%s tx dma_config failed: %d", stream->dev->name, ret);
		gd32_i2s_tx_stream_disable(stream);
		return ret;
	}

	ret = dma_start(stream->dma_dev, stream->dma_channel);
	if (ret < 0) {
		LOG_ERR("%s tx dma_start failed: %d", stream->dev->name, ret);
		gd32_i2s_tx_stream_disable(stream);
		return ret;
	}

	gd32_i2s_tx_errors_clear(cfg->reg);
	SPI_CTL1(cfg->reg) |= SPI_CTL1_ERRIE;
	spi_dma_enable(cfg->reg, SPI_DMA_TRANSMIT);
	i2s_enable(cfg->reg);
	LOG_DBG("%s tx stream started", stream->dev->name);
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

	if ((tx_stat & tx_err_mask) == 0U && (rx_stat & rx_err_mask) == 0U) {
		return;
	}

	LOG_ERR("%s i2s error tx=0x%08x rx=0x%08x", dev->name, tx_stat, rx_stat);
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

	if (data->duplex) {
		gd32_i2s_stream_fail(&data->rx);
		return;
	}

	if (rx_stat & SPI_STAT_RXORERR) {
		if (data->rx.state != I2S_STATE_NOT_READY) {
			gd32_i2s_stream_fail(&data->rx);
		}
	}
	if (tx_stat & SPI_STAT_TXURERR) {
		if (data->tx.state != I2S_STATE_NOT_READY) {
			gd32_i2s_stream_fail(&data->tx);
		}
	}
	if (rx_stat & (SPI_STAT_CONFERR | SPI_STAT_FERR)) {
		if (data->rx.state != I2S_STATE_NOT_READY && data->rx.state != I2S_STATE_ERROR) {
			gd32_i2s_stream_fail(&data->rx);
		}
	}
	if (tx_stat & (SPI_STAT_CONFERR | SPI_STAT_FERR)) {
		if (data->tx.state != I2S_STATE_NOT_READY && data->tx.state != I2S_STATE_ERROR) {
			gd32_i2s_stream_fail(&data->tx);
		}
	}
}

static int i2s_gd32_configure(const struct device *dev, enum i2s_dir dir,
			      const struct i2s_config *i2s_cfg)
{
	const struct i2s_gd32_config *cfg = dev->config;
	struct i2s_gd32_data *data = dev->data;
	struct stream *stream;
	struct stream *other;
	uint32_t std;
	uint32_t ckpl = I2S_CKPL_LOW;
	uint32_t mode;
	uint32_t frameformat;
	uint32_t add_reg = gd32_i2s_add_reg(cfg->reg);
	bool has_add = add_reg != 0U;
	bool forming_duplex;
	bool master;
	bool use_extension;
	uint32_t prescaler = GD32_I2SPSC_DEFAULT_VALUE;
	unsigned int key;
	int ret;
	struct i2s_config disabled_cfg = {0};

	if (dir == I2S_DIR_BOTH) {
		if (!has_add) {
			return -ENOTSUP;
		}

		data->paired_transition = true;
		ret = i2s_gd32_configure(dev, I2S_DIR_TX, i2s_cfg);
		if (ret < 0 || i2s_cfg->frame_clk_freq == 0U) {
			if (i2s_cfg->frame_clk_freq == 0U) {
				(void)i2s_gd32_configure(dev, I2S_DIR_RX, i2s_cfg);
				data->duplex = false;
			}
			data->paired_transition = false;
			return ret;
		}

		ret = i2s_gd32_configure(dev, I2S_DIR_RX, i2s_cfg);
		if (ret < 0) {
			(void)i2s_gd32_configure(dev, I2S_DIR_TX, &disabled_cfg);
		}
		data->paired_transition = false;
		return ret;
	}

	if (dir == I2S_DIR_RX) {
		stream = &data->rx;
	} else if (dir == I2S_DIR_TX) {
		stream = &data->tx;
	} else {
		return -EINVAL;
	}

	key = irq_lock();
	if (stream->state != I2S_STATE_NOT_READY && stream->state != I2S_STATE_READY) {
		irq_unlock(key);
		return -EINVAL;
	}

	if (data->rx.state == I2S_STATE_RUNNING || data->tx.state == I2S_STATE_RUNNING ||
	    data->rx.state == I2S_STATE_STOPPING || data->tx.state == I2S_STATE_STOPPING) {
		irq_unlock(key);
		return -EBUSY;
	}
	irq_unlock(key);

	if (i2s_cfg->frame_clk_freq == 0U) {
		if (data->duplex && !data->paired_transition) {
			return -EBUSY;
		}
		ret = gd32_i2s_stream_disable(stream);
		if (ret < 0) {
			return ret;
		}
		stream_queue_drop(stream);
		memset(&stream->cfg, 0, sizeof(stream->cfg));
		stream->state = I2S_STATE_NOT_READY;
		stream->tx_stop_for_drain = false;
		if (data->rx.state == I2S_STATE_NOT_READY &&
		    data->tx.state == I2S_STATE_NOT_READY) {
			data->duplex = false;
			gd32_i2s_clock_release(dev);
		}
		LOG_DBG("%s %s unconfigured", dev->name, dir == I2S_DIR_TX ? "tx" : "rx");
		return 0;
	}

	ret = gd32_i2s_validate_config(i2s_cfg);
	if (ret < 0) {
		return ret;
	}

	other = dir == I2S_DIR_RX ? &data->tx : &data->rx;
	forming_duplex = other->state != I2S_STATE_NOT_READY;

	if (forming_duplex && !has_add) {
		return -ENOTSUP;
	}
	if (forming_duplex && !gd32_i2s_configs_compatible(i2s_cfg, &other->cfg)) {
		LOG_ERR("%s duplex stream configurations differ", dev->name);
		return -EINVAL;
	}

	ret = gd32_std_from_config(i2s_cfg, &std);
	if (ret < 0) {
		return ret;
	}

	ret = gd32_frameformat_from_config(i2s_cfg, &frameformat);
	if (ret < 0) {
		return ret;
	}

	if ((i2s_cfg->format & I2S_FMT_CLK_FORMAT_MASK) & I2S_FMT_BIT_CLK_INV) {
		ckpl = I2S_CKPL_HIGH;
	}

	if (((i2s_cfg->options & I2S_OPT_FRAME_CLK_TARGET) != 0U) !=
	    ((i2s_cfg->options & I2S_OPT_BIT_CLK_TARGET) != 0U)) {
		return -EINVAL;
	}

	master = (i2s_cfg->options & I2S_OPT_FRAME_CLK_TARGET) == 0U;
	if (!master && !forming_duplex) {
		gd32_i2s_clock_release(dev);
	}
	use_extension = has_add && dir == I2S_DIR_RX && data->tx.state != I2S_STATE_NOT_READY;

	if (dir == I2S_DIR_TX) {
		mode = master ? I2S_MODE_MASTERTX : I2S_MODE_SLAVETX;
	} else {
		mode = master ? I2S_MODE_MASTERRX : I2S_MODE_SLAVERX;
	}

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
	memcpy(&stream->cfg, i2s_cfg, sizeof(stream->cfg));
	stream->master = master;
	stream->state = I2S_STATE_READY;
	stream->tx_stop_for_drain = false;
	stream->frame_count = 0U;
	stream->timestamp_cycles = 0U;

	LOG_DBG("%s configured %s mode=%s rate=%uHz block=%u", dev->name,
		dir == I2S_DIR_TX ? "tx" : "rx", master ? "master" : "slave",
		i2s_cfg->frame_clk_freq, i2s_cfg->block_size);

	return 0;
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
	unsigned int key;

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

	key = irq_lock();
	timing->frame_count = stream->frame_count;
	timing->timestamp_cycles = stream->timestamp_cycles;
	timing->frame_rate_num = stream->actual_frame_rate_num;
	timing->frame_rate_den = stream->actual_frame_rate_den;
	timing->timestamp_frequency = sys_clock_hw_cycles_per_sec();
	irq_unlock(key);

	return 0;
}

static int i2s_gd32_trigger(const struct device *dev, enum i2s_dir dir, enum i2s_trigger_cmd cmd)
{
	struct i2s_gd32_data *data = dev->data;
	const struct i2s_gd32_config *cfg = dev->config;
	struct stream *stream;
	unsigned int key;
	int ret;
	int tx_ret;

	if (dir == I2S_DIR_BOTH) {
		if (gd32_i2s_add_reg(cfg->reg) == 0U) {
			return -ENOTSUP;
		}

		if (cmd == I2S_TRIGGER_START) {
			key = irq_lock();
			if (data->rx.state != I2S_STATE_READY ||
			    data->tx.state != I2S_STATE_READY) {
				irq_unlock(key);
				return -EIO;
			}
			data->rx.starting = true;
			data->tx.starting = true;
			data->rx.tx_stop_for_drain = false;
			data->tx.tx_stop_for_drain = false;
			irq_unlock(key);

			ret = gd32_i2s_stream_start(&data->rx);
			if (ret == 0) {
				ret = gd32_i2s_stream_start(&data->tx);
			}
			if (ret < 0) {
				int rx_stop = gd32_i2s_rx_stream_disable(&data->rx);
				int tx_stop = gd32_i2s_tx_stream_disable(&data->tx);

				stream_queue_drop(&data->rx);
				stream_queue_drop(&data->tx);
				data->rx.state = rx_stop < 0 ? I2S_STATE_ERROR : I2S_STATE_READY;
				data->tx.state = tx_stop < 0 ? I2S_STATE_ERROR : I2S_STATE_READY;
				data->rx.starting = false;
				data->tx.starting = false;
				if (rx_stop < 0 || tx_stop < 0) {
					return rx_stop < 0 ? rx_stop : tx_stop;
				}
				return ret;
			}

			key = irq_lock();
			if (!data->rx.starting || !data->tx.starting ||
			    data->rx.state != I2S_STATE_READY ||
			    data->tx.state != I2S_STATE_READY) {
				data->rx.starting = false;
				data->tx.starting = false;
				irq_unlock(key);
				return -EIO;
			}
			data->rx.starting = false;
			data->tx.starting = false;
			data->rx.state = I2S_STATE_RUNNING;
			data->tx.state = I2S_STATE_RUNNING;
			irq_unlock(key);

			LOG_DBG("%s rx and tx started", dev->name);
			return 0;
		}

		if (cmd == I2S_TRIGGER_PREPARE) {
			struct i2s_config rx_cfg;
			struct i2s_config tx_cfg;

			key = irq_lock();
			if (data->rx.state != I2S_STATE_ERROR &&
			    data->tx.state != I2S_STATE_ERROR) {
				irq_unlock(key);
				return -EIO;
			}
			rx_cfg = data->rx.cfg;
			tx_cfg = data->tx.cfg;
			irq_unlock(key);

			ret = gd32_i2s_rx_stream_disable(&data->rx);
			if (ret < 0) {
				goto duplex_prepare_failed;
			}
			ret = gd32_i2s_tx_stream_disable(&data->tx);
			if (ret < 0) {
				goto duplex_prepare_failed;
			}
			stream_queue_drop(&data->rx);
			stream_queue_drop(&data->tx);
			ret = reset_line_toggle_dt(&cfg->reset);
			if (ret < 0) {
				goto duplex_prepare_failed;
			}
			data->duplex = false;
			data->rx.state = I2S_STATE_NOT_READY;
			data->tx.state = I2S_STATE_NOT_READY;
			ret = i2s_gd32_configure(dev, I2S_DIR_TX, &tx_cfg);
			if (ret < 0) {
				goto duplex_prepare_failed;
			}

			ret = i2s_gd32_configure(dev, I2S_DIR_RX, &rx_cfg);
			if (ret < 0) {
				struct i2s_config disabled_cfg = {0};

				(void)i2s_gd32_configure(dev, I2S_DIR_TX, &disabled_cfg);
				goto duplex_prepare_failed;
			}
			return 0;

duplex_prepare_failed:
			data->rx.cfg = rx_cfg;
			data->tx.cfg = tx_cfg;
			data->rx.state = I2S_STATE_ERROR;
			data->tx.state = I2S_STATE_ERROR;
			data->duplex = true;
			LOG_ERR("%s duplex prepare failed: %d", dev->name, ret);
			return ret;
		}

		if ((cmd == I2S_TRIGGER_STOP || cmd == I2S_TRIGGER_DRAIN) &&
		    (data->rx.state != I2S_STATE_RUNNING || data->tx.state != I2S_STATE_RUNNING)) {
			return -EIO;
		}
		if (cmd == I2S_TRIGGER_DROP && (data->rx.state == I2S_STATE_NOT_READY ||
						data->tx.state == I2S_STATE_NOT_READY)) {
			return -EIO;
		}
		if (cmd != I2S_TRIGGER_STOP && cmd != I2S_TRIGGER_DRAIN &&
		    cmd != I2S_TRIGGER_DROP) {
			return -EINVAL;
		}

		data->paired_transition = true;
		ret = i2s_gd32_trigger(dev, I2S_DIR_RX, cmd);
		tx_ret = i2s_gd32_trigger(dev, I2S_DIR_TX, cmd);
		data->paired_transition = false;

		return ret < 0 ? ret : tx_ret;
	}

	if (data->duplex && !data->paired_transition) {
		LOG_ERR("%s configured duplex; trigger both directions together", dev->name);
		return -ENOTSUP;
	}

	if (dir == I2S_DIR_RX) {
		stream = &data->rx;
	} else if (dir == I2S_DIR_TX) {
		stream = &data->tx;
	} else {
		return -EINVAL;
	}

	switch (cmd) {
	case I2S_TRIGGER_START:
		key = irq_lock();
		if (stream->state != I2S_STATE_READY) {
			irq_unlock(key);
			LOG_ERR("%s %s start invalid state %d", dev->name,
				dir == I2S_DIR_TX ? "tx" : "rx", stream->state);
			return -EIO;
		}
		stream->starting = true;
		stream->tx_stop_for_drain = false;
		irq_unlock(key);

		ret = gd32_i2s_stream_start(stream);
		if (ret < 0) {
			key = irq_lock();
			stream->starting = false;
			irq_unlock(key);
			return ret;
		}

		key = irq_lock();
		if (!stream->starting || stream->state != I2S_STATE_READY) {
			stream->starting = false;
			irq_unlock(key);
			return -EIO;
		}
		stream->starting = false;
		stream->state = I2S_STATE_RUNNING;
		irq_unlock(key);
		LOG_DBG("%s %s started", dev->name, dir == I2S_DIR_TX ? "tx" : "rx");
		return 0;

	case I2S_TRIGGER_STOP:
		key = irq_lock();
		if (stream->state != I2S_STATE_RUNNING) {
			irq_unlock(key);
			LOG_ERR("%s %s stop invalid state %d", dev->name,
				dir == I2S_DIR_TX ? "tx" : "rx", stream->state);
			return -EIO;
		}

		if (gd32_i2s_dma_busy(stream)) {
			stream->state = I2S_STATE_STOPPING;
			stream->tx_stop_for_drain = false;
			irq_unlock(key);
		} else {
			stream->state = I2S_STATE_STOPPING;
			stream->tx_stop_for_drain = false;
			irq_unlock(key);
			if (dir == I2S_DIR_TX) {
				gd32_i2s_tx_finish_async(stream);
			} else {
				ret = gd32_i2s_stream_disable(stream);
				if (ret < 0) {
					return ret;
				}
				stream->state = I2S_STATE_READY;
			}
		}

		LOG_DBG("%s %s stop requested", dev->name, dir == I2S_DIR_TX ? "tx" : "rx");
		return 0;

	case I2S_TRIGGER_DRAIN:
		key = irq_lock();
		if (stream->state != I2S_STATE_RUNNING) {
			irq_unlock(key);
			LOG_ERR("%s %s drain invalid state %d", dev->name,
				dir == I2S_DIR_TX ? "tx" : "rx", stream->state);
			return -EIO;
		}

		if (dir == I2S_DIR_TX) {
			if (!queue_is_empty(stream->msgq) || gd32_i2s_dma_busy(stream)) {
				stream->state = I2S_STATE_STOPPING;
				stream->tx_stop_for_drain = true;
				irq_unlock(key);
			} else {
				stream->state = I2S_STATE_STOPPING;
				stream->tx_stop_for_drain = false;
				irq_unlock(key);
				gd32_i2s_tx_finish_async(stream);
			}
		} else {
			if (gd32_i2s_dma_busy(stream)) {
				stream->state = I2S_STATE_STOPPING;
				irq_unlock(key);
			} else {
				irq_unlock(key);
				ret = gd32_i2s_stream_disable(stream);
				if (ret < 0) {
					return ret;
				}
				stream->state = I2S_STATE_READY;
			}
		}

		LOG_DBG("%s %s drain requested", dev->name, dir == I2S_DIR_TX ? "tx" : "rx");
		return 0;

	case I2S_TRIGGER_DROP:
		key = irq_lock();
		if (stream->state == I2S_STATE_NOT_READY) {
			irq_unlock(key);
			LOG_ERR("%s %s drop invalid state", dev->name,
				dir == I2S_DIR_TX ? "tx" : "rx");
			return -EIO;
		}

		stream->state = I2S_STATE_READY;
		stream->tx_stop_for_drain = false;
		irq_unlock(key);

		ret = gd32_i2s_stream_disable(stream);
		if (ret < 0) {
			return ret;
		}
		stream_queue_drop(stream);
		LOG_DBG("%s %s dropped", dev->name, dir == I2S_DIR_TX ? "tx" : "rx");
		return 0;

	case I2S_TRIGGER_PREPARE: {
		struct i2s_config cfg_copy;

		key = irq_lock();
		if (stream->state != I2S_STATE_ERROR) {
			irq_unlock(key);
			LOG_ERR("%s %s prepare invalid state %d", dev->name,
				dir == I2S_DIR_TX ? "tx" : "rx", stream->state);
			return -EIO;
		}

		cfg_copy = stream->cfg;
		irq_unlock(key);

		ret = gd32_i2s_stream_disable(stream);
		if (ret < 0) {
			return ret;
		}
		stream_queue_drop(stream);
		ret = reset_line_toggle_dt(&((const struct i2s_gd32_config *)dev->config)->reset);
		if (ret < 0) {
			LOG_ERR("%s %s reset failed: %d", dev->name,
				dir == I2S_DIR_TX ? "tx" : "rx", ret);
			return ret;
		}

		key = irq_lock();
		stream->state = I2S_STATE_NOT_READY;
		stream->tx_stop_for_drain = false;
		irq_unlock(key);

		ret = i2s_gd32_configure(dev, stream->dir, &cfg_copy);
		if (ret < 0) {
			stream->cfg = cfg_copy;
			stream->state = I2S_STATE_ERROR;
			LOG_ERR("%s %s prepare failed: %d", dev->name,
				dir == I2S_DIR_TX ? "tx" : "rx", ret);
			return ret;
		}

		LOG_DBG("%s %s prepared", dev->name, dir == I2S_DIR_TX ? "tx" : "rx");
		return 0;
	}

	default:
		return -EINVAL;
	}
}

static int i2s_gd32_read(const struct device *dev, void **mem_block, size_t *size)
{
	struct i2s_gd32_data *data = dev->data;
	int ret;

	if (data->rx.state == I2S_STATE_NOT_READY) {
		return -EIO;
	}

	ret = queue_get(data->rx.msgq, mem_block, size, data->rx.cfg.timeout);
	if (ret < 0) {
		return (data->rx.state == I2S_STATE_ERROR) ? -EIO : ret;
	}

	return 0;
}

static int i2s_gd32_write(const struct device *dev, void *mem_block, size_t size)
{
	struct i2s_gd32_data *data = dev->data;
	int ret;

	if (data->tx.state != I2S_STATE_RUNNING && data->tx.state != I2S_STATE_READY) {
		LOG_ERR("%s tx write invalid state %d", dev->name, data->tx.state);
		return -EIO;
	}

	if ((size == 0U) || (size > data->tx.cfg.block_size)) {
		LOG_ERR("%s tx write invalid size %u", dev->name, (uint32_t)size);
		return -EINVAL;
	}

	if ((size % 4U) != 0U) {
		LOG_ERR("%s tx write unaligned size %u", dev->name, (uint32_t)size);
		return -EINVAL;
	}

	ret = queue_put(data->tx.msgq, mem_block, size, data->tx.cfg.timeout);
	if (ret < 0) {
		LOG_ERR("%s tx queue put failed: %d", dev->name, ret);
	}

	return ret;
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

	ch_filter = BIT(cfg->dma_tx.channel);
	ret = dma_request_channel(cfg->dma_tx.dev, &ch_filter);
	if (ret < 0) {
		return ret;
	}
	if (ret != cfg->dma_tx.channel) {
		dma_release_channel(cfg->dma_tx.dev, ret);
		return -EINVAL;
	}

	ch_filter = BIT(cfg->dma_rx.channel);
	ret = dma_request_channel(cfg->dma_rx.dev, &ch_filter);
	if (ret < 0) {
		dma_release_channel(cfg->dma_tx.dev, cfg->dma_tx.channel);
		return ret;
	}
	if (ret != cfg->dma_rx.channel) {
		dma_release_channel(cfg->dma_rx.dev, ret);
		dma_release_channel(cfg->dma_tx.dev, cfg->dma_tx.channel);
		return -EINVAL;
	}

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
		.mck_enabled = DT_INST_PROP_OR(inst, mck_enabled, 0),                              \
		.irq_configure = i2s_gd32_irq_configure_##inst,                                    \
		.dma_rx = DMA_INITIALIZER(inst, rx),                                               \
		.dma_tx = DMA_INITIALIZER(inst, tx),                                               \
	};                                                                                         \
	DEVICE_DT_INST_DEFINE(inst, i2s_gd32_init, NULL, &i2s_gd32_data_##inst,                    \
			      &i2s_gd32_config_##inst, POST_KERNEL, CONFIG_I2S_INIT_PRIORITY,      \
			      &i2s_gd32_driver_api)

DT_INST_FOREACH_STATUS_OKAY(I2S_GD32_INIT)
