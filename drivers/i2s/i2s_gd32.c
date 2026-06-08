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
#include <zephyr/sys/util.h>

#include <gd32_rcu.h>
#include <gd32_spi.h>

LOG_MODULE_REGISTER(i2s_gd32, CONFIG_I2S_LOG_LEVEL);

#define GD32_I2SPSC_DEFAULT_VALUE 0x00000002U

struct gd32_i2s_clock_plan {
	uint16_t plli2s_n;
	uint8_t plli2s_r;
	uint16_t divider;
	bool odd;
	uint32_t source_rate_hz;
	uint32_t sample_rate_hz;
	uint32_t error_hz;
};

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
	void *mem_block;
	size_t mem_block_size;
	uint32_t last_block_start_cycle;
	bool master;
	bool tx_stop_for_drain;
};

struct i2s_gd32_data {
	struct stream rx;
	struct stream tx;
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

static inline uint32_t gd32_i2s_rx_reg(const struct i2s_gd32_config *cfg)
{
	uint32_t add = gd32_i2s_add_reg(cfg->reg);

	return add ? add : cfg->reg;
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

static int gd32_i2s_pll_input_rate_get(uint32_t *rate_hz)
{
	uint32_t pllm = RCU_PLL & RCU_PLL_PLLPSC;

	if (pllm == 0U) {
		return -EINVAL;
	}

	if ((RCU_PLL & RCU_PLL_PLLSEL) == RCU_PLLSRC_HXTAL) {
		*rate_hz = HXTAL_VALUE / pllm;
	} else {
		*rate_hz = IRC16M_VALUE / pllm;
	}

	return 0;
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

static uint32_t gd32_i2s_compute_sample_rate(uint32_t source_rate_hz, uint32_t frame_factor,
					     uint16_t divider, bool odd)
{
	uint32_t serial_div = (uint32_t)(divider * 2U) + (odd ? 1U : 0U);
	uint64_t denominator = (uint64_t)frame_factor * serial_div;

	return (uint32_t)((((uint64_t)source_rate_hz) + (denominator / 2U)) / denominator);
}

static int gd32_i2s_find_clock_plan(uint32_t target_sample_rate_hz, uint32_t frameformat,
				    uint32_t mckout, struct gd32_i2s_clock_plan *plan)
{
	uint32_t input_rate_hz;
	int ret = gd32_i2s_pll_input_rate_get(&input_rate_hz);
	uint32_t frame_factor;
	bool found = false;

	if (ret < 0) {
		return ret;
	}

	frame_factor = gd32_i2s_frame_factor(frameformat, mckout);

	for (uint16_t n = 50U; n <= 500U; n++) {
		for (uint8_t r = 2U; r <= 7U; r++) {
			uint32_t source_rate_hz = ((uint64_t)input_rate_hz * n) / r;

			for (uint16_t divider = 2U; divider <= 255U; divider++) {
				for (uint8_t odd = 0U; odd <= 1U; odd++) {
					uint32_t sample_rate_hz = gd32_i2s_compute_sample_rate(
						source_rate_hz, frame_factor, divider, odd != 0U);
					uint32_t error_hz = (sample_rate_hz > target_sample_rate_hz) ?
						(sample_rate_hz - target_sample_rate_hz) :
						(target_sample_rate_hz - sample_rate_hz);

					if (!found || (error_hz < plan->error_hz) ||
					    ((error_hz == plan->error_hz) &&
					     (source_rate_hz < plan->source_rate_hz))) {
						plan->plli2s_n = n;
						plan->plli2s_r = r;
						plan->divider = divider;
						plan->odd = odd != 0U;
						plan->source_rate_hz = source_rate_hz;
						plan->sample_rate_hz = sample_rate_hz;
						plan->error_hz = error_hz;
						found = true;

						if (error_hz == 0U) {
							return 0;
						}
					}
				}
			}
		}
	}

	return found ? 0 : -EINVAL;
}

static int gd32_i2s_master_clock_apply(const struct device *dev, uint32_t target_sample_rate_hz,
				       uint32_t frameformat)
{
	const struct i2s_gd32_config *cfg = dev->config;
	struct gd32_i2s_clock_plan plan = {0};
	struct gd32_i2s_clock_config clk_cfg = {
		.type = GD32_CLOCK_CONFIG_TYPE_I2S,
		.source = GD32_I2S_CLOCK_SRC_PLLI2S,
	};
	uint32_t mckout = cfg->mck_enabled ? I2S_MCKOUT_ENABLE : I2S_MCKOUT_DISABLE;
	int ret;

	ret = gd32_i2s_find_clock_plan(target_sample_rate_hz, frameformat, mckout, &plan);
	if (ret < 0) {
		return ret;
	}

	clk_cfg.plli2s_n = plan.plli2s_n;
	clk_cfg.plli2s_r = plan.plli2s_r;

	ret = clock_control_configure(GD32_CLOCK_CONTROLLER, (clock_control_subsys_t)&cfg->clkid,
				      &clk_cfg);
	if (ret < 0) {
		return ret;
	}

	SPI_I2SPSC(cfg->reg) = GD32_I2SPSC_DEFAULT_VALUE;
	SPI_I2SPSC(cfg->reg) = plan.divider | (plan.odd ? BIT(8) : 0U) | mckout;
	gd32_i2s_frameformat_apply(cfg->reg, frameformat);

	LOG_INF("%s master clock src=%uHz pll=%u/%u psc=%u%s sample=%uHz target=%uHz",
		dev->name, clk_cfg.actual_rate_hz, plan.plli2s_n, plan.plli2s_r, plan.divider,
		plan.odd ? "+odd" : "", plan.sample_rate_hz, target_sample_rate_hz);

	return 0;
}

static uint32_t gd32_i2s_dma_xfer_count(size_t bytes)
{
	return (uint32_t)(bytes / sizeof(uint16_t));
}

static void gd32_i2s_rx_stream_disable(struct stream *stream)
{
	const struct i2s_gd32_config *cfg = stream->dev->config;
	const uint32_t rx_reg = gd32_i2s_rx_reg(cfg);

	SPI_CTL1(cfg->reg) &= ~SPI_CTL1_ERRIE;
	spi_dma_disable(rx_reg, SPI_DMA_RECEIVE);
	(void)dma_stop(stream->dma_dev, stream->dma_channel);
	i2s_disable(rx_reg);

	if (stream->mem_block != NULL) {
		k_mem_slab_free(stream->cfg.mem_slab, stream->mem_block);
		stream->mem_block = NULL;
	}

	stream->mem_block_size = 0U;
}

static void gd32_i2s_tx_stream_disable(struct stream *stream)
{
	const struct i2s_gd32_config *cfg = stream->dev->config;

	SPI_CTL1(cfg->reg) &= ~SPI_CTL1_ERRIE;
	spi_dma_disable(cfg->reg, SPI_DMA_TRANSMIT);
	(void)dma_stop(stream->dma_dev, stream->dma_channel);
	i2s_disable(cfg->reg);

	if (stream->mem_block != NULL) {
		k_mem_slab_free(stream->cfg.mem_slab, stream->mem_block);
		stream->mem_block = NULL;
	}

	stream->mem_block_size = 0U;
}

static void gd32_i2s_stream_disable(struct stream *stream)
{
	if (stream->dir == I2S_DIR_RX) {
		gd32_i2s_rx_stream_disable(stream);
	} else {
		gd32_i2s_tx_stream_disable(stream);
	}
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

	return -ENOTSUP;
}

static int gd32_i2s_validate_config(const struct i2s_config *cfg)
{
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

	if (cfg->block_size == 0U) {
		return -EINVAL;
	}

	if ((cfg->block_size % 4U) != 0U) {
		return -EINVAL;
	}

	if ((cfg->block_size % sizeof(uint16_t)) != 0U) {
		return -EINVAL;
	}

	if (cfg->timeout < -1) {
		return -EINVAL;
	}

	return 0;
}

static int gd32_i2s_tx_acquire_block(struct stream *stream)
{
	int ret;

	ret = queue_get(stream->msgq, &stream->mem_block, &stream->mem_block_size, 0);
	if (ret < 0) {
		return ret;
	}

	if ((stream->mem_block_size == 0U) || (stream->mem_block_size > stream->cfg.block_size)) {
		size_t invalid_size = stream->mem_block_size;

		k_mem_slab_free(stream->cfg.mem_slab, stream->mem_block);
		stream->mem_block = NULL;
		stream->mem_block_size = 0U;
		LOG_ERR("%s tx invalid block size %u", stream->dev->name, (uint32_t)invalid_size);
		return -EINVAL;
	}

	if ((stream->mem_block_size % 4U) != 0U) {
		size_t invalid_size = stream->mem_block_size;

		k_mem_slab_free(stream->cfg.mem_slab, stream->mem_block);
		stream->mem_block = NULL;
		stream->mem_block_size = 0U;
		LOG_ERR("%s tx block size %u not aligned", stream->dev->name, (uint32_t)invalid_size);
		return -EINVAL;
	}

	sys_cache_data_flush_range(stream->mem_block, stream->mem_block_size);

	return 0;
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

	if (stream->dir == I2S_DIR_RX) {
		const uint32_t rx_reg = gd32_i2s_rx_reg(cfg);
		void *filled_block;

		if (status < 0) {
			LOG_ERR("%s dma rx error: ch=%u status=%d", dev->name, stream->dma_channel,
				status);
			stream->state = I2S_STATE_ERROR;
			gd32_i2s_rx_stream_disable(stream);
			return;
		}

		__ASSERT_NO_MSG(stream->mem_block != NULL);

		if (stream->state == I2S_STATE_ERROR) {
			gd32_i2s_rx_stream_disable(stream);
			return;
		}

		filled_block = stream->mem_block;
		stream->mem_block = NULL;
		sys_cache_data_invd_range(filled_block, stream->cfg.block_size);

		ret = queue_put(stream->msgq, filled_block, stream->cfg.block_size, 0);
		if (ret < 0) {
			LOG_ERR("%s rx queue full", dev->name);
			k_mem_slab_free(stream->cfg.mem_slab, filled_block);
			stream->state = I2S_STATE_ERROR;
			gd32_i2s_rx_stream_disable(stream);
			return;
		}

		if (stream->state == I2S_STATE_STOPPING) {
			stream->state = I2S_STATE_READY;
			gd32_i2s_rx_stream_disable(stream);
			return;
		}

		ret = k_mem_slab_alloc(stream->cfg.mem_slab, &stream->mem_block, K_NO_WAIT);
		if (ret < 0) {
			LOG_ERR("%s rx slab exhausted", dev->name);
			stream->state = I2S_STATE_ERROR;
			gd32_i2s_rx_stream_disable(stream);
			return;
		}

		stream->mem_block_size = stream->cfg.block_size;
		ret = dma_reload(stream->dma_dev, stream->dma_channel, (uint32_t)&SPI_DATA(rx_reg),
				 (uint32_t)stream->mem_block,
				 gd32_i2s_dma_xfer_count(stream->cfg.block_size));
		if (ret < 0) {
			LOG_ERR("%s rx dma_reload failed: %d", dev->name, ret);
			stream->state = I2S_STATE_ERROR;
			gd32_i2s_rx_stream_disable(stream);
			return;
		}

		ret = dma_start(stream->dma_dev, stream->dma_channel);
		if (ret < 0) {
			LOG_ERR("%s rx dma_start failed: %d", dev->name, ret);
			stream->state = I2S_STATE_ERROR;
			gd32_i2s_rx_stream_disable(stream);
		}

		return;
	}

	if (status < 0) {
		LOG_ERR("%s dma tx error: ch=%u status=%d", dev->name, stream->dma_channel, status);
		stream->state = I2S_STATE_ERROR;
		gd32_i2s_tx_stream_disable(stream);
		return;
	}

	__ASSERT_NO_MSG(stream->mem_block != NULL);

	k_mem_slab_free(stream->cfg.mem_slab, stream->mem_block);
	stream->mem_block = NULL;
	stream->mem_block_size = 0U;

	if (stream->state == I2S_STATE_ERROR) {
		gd32_i2s_tx_stream_disable(stream);
		return;
	}

	if (stream->state == I2S_STATE_STOPPING && !stream->tx_stop_for_drain) {
		stream->state = I2S_STATE_READY;
		gd32_i2s_tx_stream_disable(stream);
		return;
	}

	ret = gd32_i2s_tx_acquire_block(stream);
	if (ret < 0) {
		if (stream->state == I2S_STATE_STOPPING && stream->tx_stop_for_drain) {
			stream->state = I2S_STATE_READY;
		} else {
			LOG_ERR("%s tx underrun: no next buffer queued", dev->name);
			stream->state = I2S_STATE_ERROR;
		}

		gd32_i2s_tx_stream_disable(stream);
		return;
	}

	ret = dma_reload(stream->dma_dev, stream->dma_channel, (uint32_t)stream->mem_block,
			 (uint32_t)&SPI_DATA(cfg->reg),
			 gd32_i2s_dma_xfer_count(stream->mem_block_size));
	if (ret < 0) {
		LOG_ERR("%s tx dma_reload failed: %d", dev->name, ret);
		stream->state = I2S_STATE_ERROR;
		gd32_i2s_tx_stream_disable(stream);
		return;
	}

	stream->last_block_start_cycle = k_cycle_get_32();
	ret = dma_start(stream->dma_dev, stream->dma_channel);
	if (ret < 0) {
		LOG_ERR("%s tx dma_start failed: %d", dev->name, ret);
		stream->state = I2S_STATE_ERROR;
		gd32_i2s_tx_stream_disable(stream);
	}
}

static int gd32_i2s_stream_start(struct stream *stream)
{
	const struct i2s_gd32_config *cfg = stream->dev->config;
	int ret;

	memset(&stream->dma_cfg, 0, sizeof(stream->dma_cfg));
	memset(&stream->dma_blk, 0, sizeof(stream->dma_blk));

	stream->dma_cfg.dma_callback = gd32_i2s_dma_callback;
	stream->dma_cfg.user_data = stream;
	stream->dma_cfg.block_count = 1U;
	stream->dma_cfg.head_block = &stream->dma_blk;
	stream->dma_cfg.dma_slot = stream->dma_slot;
	stream->dma_cfg.channel_priority = stream->dma_priority;
	stream->dma_cfg.source_burst_length = 1U;
	stream->dma_cfg.dest_burst_length = 1U;
	stream->dma_cfg.source_data_size = 2U;
	stream->dma_cfg.dest_data_size = 2U;
	stream->dma_blk.fifo_mode_control = stream->fifo_threshold;

	if (stream->dir == I2S_DIR_RX) {
		const uint32_t rx_reg = gd32_i2s_rx_reg(cfg);

		ret = k_mem_slab_alloc(stream->cfg.mem_slab, &stream->mem_block, K_NO_WAIT);
		if (ret < 0) {
			LOG_ERR("%s rx start failed: no slab block", stream->dev->name);
			return -ENOMEM;
		}

		stream->mem_block_size = stream->cfg.block_size;
		stream->dma_cfg.channel_direction = PERIPHERAL_TO_MEMORY;
		stream->dma_blk.source_address = (uint32_t)&SPI_DATA(rx_reg);
		stream->dma_blk.dest_address = (uint32_t)stream->mem_block;
		stream->dma_blk.source_addr_adj = DMA_ADDR_ADJ_NO_CHANGE;
		stream->dma_blk.dest_addr_adj = DMA_ADDR_ADJ_INCREMENT;
		stream->dma_blk.block_size = gd32_i2s_dma_xfer_count(stream->cfg.block_size);

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

		SPI_CTL1(cfg->reg) |= SPI_CTL1_ERRIE;
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

	stream->dma_cfg.channel_direction = MEMORY_TO_PERIPHERAL;
	stream->dma_blk.source_address = (uint32_t)stream->mem_block;
	stream->dma_blk.dest_address = (uint32_t)&SPI_DATA(cfg->reg);
	stream->dma_blk.source_addr_adj = DMA_ADDR_ADJ_INCREMENT;
	stream->dma_blk.dest_addr_adj = DMA_ADDR_ADJ_NO_CHANGE;
	stream->dma_blk.block_size = gd32_i2s_dma_xfer_count(stream->mem_block_size);

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

	stream->last_block_start_cycle = k_cycle_get_32();
	SPI_CTL1(cfg->reg) |= SPI_CTL1_ERRIE;
	spi_dma_enable(cfg->reg, SPI_DMA_TRANSMIT);
	i2s_enable(cfg->reg);

	LOG_DBG("%s tx stream started", stream->dev->name);
	return 0;
}

uint32_t i2s_gd32_tx_block_start_cycle_get(const struct device *dev)
{
	struct i2s_gd32_data *data = dev->data;

	return data->tx.last_block_start_cycle;
}

static void i2s_gd32_isr(const struct device *dev)
{
	const struct i2s_gd32_config *cfg = dev->config;
	struct i2s_gd32_data *data = dev->data;
	const uint32_t err_mask = SPI_STAT_RXORERR | SPI_STAT_TXURERR | SPI_STAT_CONFERR |
				  SPI_STAT_FERR;
	uint32_t stat = SPI_STAT(cfg->reg);

	if ((stat & err_mask) == 0U) {
		return;
	}

	LOG_ERR("%s i2s error stat=0x%08x%s%s%s%s", dev->name, stat,
		(stat & SPI_STAT_RXORERR) ? " rxor" : "",
		(stat & SPI_STAT_TXURERR) ? " txur" : "",
		(stat & SPI_STAT_CONFERR) ? " conf" : "",
		(stat & SPI_STAT_FERR) ? " ferr" : "");

	data->rx.state = I2S_STATE_ERROR;
	data->tx.state = I2S_STATE_ERROR;
	gd32_i2s_rx_stream_disable(&data->rx);
	gd32_i2s_tx_stream_disable(&data->tx);
}

static int i2s_gd32_configure(const struct device *dev, enum i2s_dir dir,
			      const struct i2s_config *i2s_cfg)
{
	const struct i2s_gd32_config *cfg = dev->config;
	struct i2s_gd32_data *data = dev->data;
	struct stream *stream;
	uint32_t std;
	uint32_t ckpl = I2S_CKPL_LOW;
	uint32_t mode;
	uint32_t frameformat;
	uint32_t add_reg = gd32_i2s_add_reg(cfg->reg);
	bool has_add = add_reg != 0U;
	bool master;
	unsigned int key;
	int ret;

	if (dir == I2S_DIR_RX) {
		stream = &data->rx;
	} else if (dir == I2S_DIR_TX) {
		stream = &data->tx;
	} else if (dir == I2S_DIR_BOTH) {
		return -ENOSYS;
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
		gd32_i2s_stream_disable(stream);
		stream_queue_drop(stream);
		memset(&stream->cfg, 0, sizeof(stream->cfg));
		stream->state = I2S_STATE_NOT_READY;
		stream->tx_stop_for_drain = false;
		LOG_INF("%s %s unconfigured", dev->name, dir == I2S_DIR_TX ? "tx" : "rx");
		return 0;
	}

	ret = gd32_i2s_validate_config(i2s_cfg);
	if (ret < 0) {
		return ret;
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

	if (dir == I2S_DIR_TX) {
		mode = master ? I2S_MODE_MASTERTX : I2S_MODE_SLAVETX;
	} else {
		mode = master ? I2S_MODE_MASTERRX : I2S_MODE_SLAVERX;
	}

	if (!(has_add && dir == I2S_DIR_RX && data->tx.state != I2S_STATE_NOT_READY)) {
		spi_i2s_deinit(cfg->reg);
		i2s_init(cfg->reg, mode, std, ckpl);
	}

	if (master) {
		ret = gd32_i2s_master_clock_apply(dev, i2s_cfg->frame_clk_freq, frameformat);
		if (ret < 0) {
			LOG_ERR("%s failed to configure master clock: %d", dev->name, ret);
			return ret;
		}
	} else if (!(has_add && dir == I2S_DIR_RX && data->tx.state != I2S_STATE_NOT_READY)) {
		gd32_i2s_frameformat_apply(cfg->reg, frameformat);
		if (cfg->mck_enabled) {
			SPI_I2SPSC(cfg->reg) |= SPI_I2SPSC_MCKOEN;
		} else {
			SPI_I2SPSC(cfg->reg) &= ~SPI_I2SPSC_MCKOEN;
		}
	}

	if (has_add && dir == I2S_DIR_TX) {
		i2s_full_duplex_mode_config(add_reg, mode, std, ckpl, frameformat);
	}

	memcpy(&stream->cfg, i2s_cfg, sizeof(stream->cfg));
	stream->master = master;
	stream->state = I2S_STATE_READY;
	stream->tx_stop_for_drain = false;

	LOG_INF("%s configured %s mode=%s rate=%uHz block=%u", dev->name,
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

static int i2s_gd32_trigger(const struct device *dev, enum i2s_dir dir, enum i2s_trigger_cmd cmd)
{
	struct i2s_gd32_data *data = dev->data;
	struct stream *stream;
	unsigned int key;
	int ret;

	if (dir == I2S_DIR_RX) {
		stream = &data->rx;
	} else if (dir == I2S_DIR_TX) {
		stream = &data->tx;
	} else if (dir == I2S_DIR_BOTH) {
		return -ENOSYS;
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
		irq_unlock(key);

		ret = gd32_i2s_stream_start(stream);
		if (ret < 0) {
			return ret;
		}

		key = irq_lock();
		stream->state = I2S_STATE_RUNNING;
		stream->tx_stop_for_drain = false;
		irq_unlock(key);
		LOG_INF("%s %s started", dev->name, dir == I2S_DIR_TX ? "tx" : "rx");
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
			stream->state = I2S_STATE_READY;
			stream->tx_stop_for_drain = false;
			irq_unlock(key);
			gd32_i2s_stream_disable(stream);
		}

		LOG_INF("%s %s stop requested", dev->name, dir == I2S_DIR_TX ? "tx" : "rx");
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
				stream->state = I2S_STATE_READY;
				stream->tx_stop_for_drain = false;
				irq_unlock(key);
				gd32_i2s_stream_disable(stream);
			}
		} else {
			if (gd32_i2s_dma_busy(stream)) {
				stream->state = I2S_STATE_STOPPING;
				irq_unlock(key);
			} else {
				stream->state = I2S_STATE_READY;
				irq_unlock(key);
				gd32_i2s_stream_disable(stream);
			}
		}

		LOG_INF("%s %s drain requested", dev->name, dir == I2S_DIR_TX ? "tx" : "rx");
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

		gd32_i2s_stream_disable(stream);
		stream_queue_drop(stream);
		LOG_INF("%s %s dropped", dev->name, dir == I2S_DIR_TX ? "tx" : "rx");
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

		gd32_i2s_stream_disable(stream);
		stream_queue_drop(stream);
		(void)reset_line_toggle_dt(&((const struct i2s_gd32_config *)dev->config)->reset);

		key = irq_lock();
		stream->state = I2S_STATE_READY;
		stream->tx_stop_for_drain = false;
		irq_unlock(key);

		ret = i2s_gd32_configure(dev, stream->dir, &cfg_copy);
		if (ret < 0) {
			stream->state = I2S_STATE_ERROR;
			LOG_ERR("%s %s prepare failed: %d", dev->name,
				dir == I2S_DIR_TX ? "tx" : "rx", ret);
			return ret;
		}

		LOG_INF("%s %s prepared", dev->name, dir == I2S_DIR_TX ? "tx" : "rx");
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
};

static int i2s_gd32_init(const struct device *dev)
{
	const struct i2s_gd32_config *cfg = dev->config;
	struct i2s_gd32_data *data = dev->data;
	uint32_t ch_filter;
	int ret;

	(void)clock_control_on(GD32_CLOCK_CONTROLLER, (clock_control_subsys_t)&cfg->clkid);
	(void)reset_line_toggle_dt(&cfg->reset);

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

	ch_filter = BIT(cfg->dma_rx.channel);
	ret = dma_request_channel(cfg->dma_rx.dev, &ch_filter);
	if (ret < 0) {
		return ret;
	}

	data->tx.dev = dev;
	data->tx.dir = I2S_DIR_TX;
	data->tx.state = I2S_STATE_NOT_READY;
	data->tx.dma_dev = cfg->dma_tx.dev;
	data->tx.dma_channel = cfg->dma_tx.channel;
	data->tx.dma_slot = cfg->dma_tx.slot;
	data->tx.dma_priority = GD32_DMA_CONFIG_PRIORITY(cfg->dma_tx.config);

	data->rx.dev = dev;
	data->rx.dir = I2S_DIR_RX;
	data->rx.state = I2S_STATE_NOT_READY;
	data->rx.dma_dev = cfg->dma_rx.dev;
	data->rx.dma_channel = cfg->dma_rx.channel;
	data->rx.dma_slot = cfg->dma_rx.slot;
	data->rx.dma_priority = GD32_DMA_CONFIG_PRIORITY(cfg->dma_rx.config);

	cfg->irq_configure(dev);
	return 0;
}

#define DMA_INITIALIZER(idx, dir)                                                       \
	{                                                                                   \
		.dev = DEVICE_DT_GET(DT_INST_DMAS_CTLR_BY_NAME(idx, dir)),                      \
		.channel = DT_INST_DMAS_CELL_BY_NAME(idx, dir, channel),                        \
		.slot = COND_CODE_1(DT_HAS_COMPAT_STATUS_OKAY(gd_gd32_dma_v1),                  \
				    (DT_INST_DMAS_CELL_BY_NAME(idx, dir, slot)), (0)),          \
		.config = DT_INST_DMAS_CELL_BY_NAME(idx, dir, config),                          \
		.fifo_threshold = COND_CODE_1(DT_HAS_COMPAT_STATUS_OKAY(gd_gd32_dma_v1),        \
					      (GD32_DMA_DT_FIFO_MODE(                                    \
						      DT_INST_DMAS_CELL_BY_NAME(                          \
							      idx, dir, fifo_threshold))), (0)), \
	}

#define DMA_BY_NAME(idx, dir)                                                           \
	COND_CODE_1(DT_INST_DMAS_HAS_NAME(idx, dir), (DMA_INITIALIZER(idx, dir)), ({0}))

#define I2S_GD32_INIT(inst)                                                             \
	PINCTRL_DT_INST_DEFINE(inst);                                                       \
	K_MSGQ_DEFINE(rx_##inst##_queue, sizeof(struct queue_item),                          \
		      CONFIG_I2S_GD32_RX_BLOCK_COUNT, 4);                                   \
	K_MSGQ_DEFINE(tx_##inst##_queue, sizeof(struct queue_item),                          \
		      CONFIG_I2S_GD32_TX_BLOCK_COUNT, 4);                                   \
	static void i2s_gd32_irq_configure_##inst(const struct device *dev)                  \
	{                                                                                   \
		IRQ_CONNECT(DT_INST_IRQN(inst), DT_INST_IRQ(inst, priority),                     \
			    i2s_gd32_isr, DEVICE_DT_INST_GET(inst), 0);                          \
		irq_enable(DT_INST_IRQN(inst));                                                  \
		ARG_UNUSED(dev);                                                                 \
	}                                                                                   \
	static struct i2s_gd32_data i2s_gd32_data_##inst = {                                 \
		.rx = {.msgq = &rx_##inst##_queue, .state = I2S_STATE_NOT_READY},                \
		.tx = {.msgq = &tx_##inst##_queue, .state = I2S_STATE_NOT_READY},                \
	};                                                                                  \
	static const struct i2s_gd32_config i2s_gd32_config_##inst = {                       \
		.reg = DT_INST_REG_ADDR(inst),                                                  \
		.clkid = DT_INST_CLOCKS_CELL(inst, id),                                         \
		.reset = RESET_DT_SPEC_INST_GET(inst),                                          \
		.pcfg = PINCTRL_DT_INST_DEV_CONFIG_GET(inst),                                   \
		.mck_enabled = DT_INST_PROP_OR(inst, mck_enabled, 0),                           \
		.irq_configure = i2s_gd32_irq_configure_##inst,                                 \
		.dma_rx = DMA_BY_NAME(inst, rx),                                                \
		.dma_tx = DMA_BY_NAME(inst, tx),                                                \
	};                                                                                  \
	DEVICE_DT_INST_DEFINE(inst, i2s_gd32_init, NULL,                                    \
			      &i2s_gd32_data_##inst, &i2s_gd32_config_##inst,               \
			      POST_KERNEL, CONFIG_I2S_INIT_PRIORITY,                        \
			      &i2s_gd32_driver_api)

DT_INST_FOREACH_STATUS_OKAY(I2S_GD32_INIT)
