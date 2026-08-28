/*
 * Copyright (c) 2022 TOKITA Hiroshi <tokita.hiroshi@gmail.com>
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/device.h>
#include <zephyr/drivers/clock_control.h>
#include <zephyr/drivers/clock_control/gd32.h>
#include <zephyr/drivers/dma.h>
#include <zephyr/drivers/dma/dma_gd32.h>
#include <zephyr/drivers/reset.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

#include <gd32_dma.h>
#include <zephyr/irq.h>

#if DT_HAS_COMPAT_STATUS_OKAY(gd_gd32_dma_v1)
#define DT_DRV_COMPAT gd_gd32_dma_v1
#elif DT_HAS_COMPAT_STATUS_OKAY(gd_gd32_dma)
#define DT_DRV_COMPAT gd_gd32_dma
#endif

#if DT_HAS_COMPAT_STATUS_OKAY(gd_gd32_dma_v1)
#define CHXCTL_PERIEN_OFFSET          ((uint32_t)25U)
#define GD32_DMA_CHXCTL_DIR           BIT(6)
#define GD32_DMA_CHXCTL_M2M           BIT(7)
#define GD32_DMA_INTERRUPT_ERRORS     DMA_CHXCTL_TAEIE
#define GD32_DMA_INTERRUPT_EXCEPTIONS DMA_CHXCTL_SDEIE
#if defined(DMA_FLAG_FEE)
#define GD32_DMA_FLAG_FIFO_EVENT DMA_FLAG_FEE
#else
#define GD32_DMA_FLAG_FIFO_EVENT 0U
#endif
#define GD32_DMA_FLAG_ERRORS     DMA_FLAG_TAE
#define GD32_DMA_FLAG_EXCEPTIONS DMA_FLAG_SDE
#else
#define GD32_DMA_CHXCTL_DIR           BIT(4)
#define GD32_DMA_CHXCTL_M2M           BIT(14)
#define GD32_DMA_INTERRUPT_ERRORS     DMA_CHXCTL_ERRIE
#define GD32_DMA_INTERRUPT_EXCEPTIONS 0U
#define GD32_DMA_FLAG_ERRORS          DMA_FLAG_ERR
#define GD32_DMA_FLAG_FIFO_EVENT      0U
#define GD32_DMA_FLAG_EXCEPTIONS      0U
#endif

#ifdef CONFIG_SOC_SERIES_GD32F3X0
#undef DMA_INTF
#undef DMA_INTC
#undef DMA_CHCTL
#undef DMA_CHCNT
#undef DMA_CHPADDR
#undef DMA_CHMADDR

#define DMA_INTF(dma)        REG32(dma + 0x00UL)
#define DMA_INTC(dma)        REG32(dma + 0x04UL)
#define DMA_CHCTL(dma, ch)   REG32((dma + 0x08UL) + 0x14UL * (uint32_t)(ch))
#define DMA_CHCNT(dma, ch)   REG32((dma + 0x0CUL) + 0x14UL * (uint32_t)(ch))
#define DMA_CHPADDR(dma, ch) REG32((dma + 0x10UL) + 0x14UL * (uint32_t)(ch))
#define DMA_CHMADDR(dma, ch) REG32((dma + 0x14UL) + 0x14UL * (uint32_t)(ch))
#endif

#define GD32_DMA_INTF(dma)        DMA_INTF(dma)
#define GD32_DMA_INTC(dma)        DMA_INTC(dma)
#define GD32_DMA_CHCTL(dma, ch)   DMA_CHCTL((dma), (ch))
#define GD32_DMA_CHCNT(dma, ch)   DMA_CHCNT((dma), (ch))
#define GD32_DMA_CHPADDR(dma, ch) DMA_CHPADDR((dma), (ch))
#define GD32_DMA_CHMADDR(dma, ch) DMA_CHMADDR((dma), (ch))

LOG_MODULE_REGISTER(dma_gd32, CONFIG_DMA_LOG_LEVEL);

struct dma_gd32_config {
	uint32_t reg;
	uint32_t channels;
	uint16_t clkid;
	bool mem2mem;
#if DT_HAS_COMPAT_STATUS_OKAY(gd_gd32_dma_v1)
	struct reset_dt_spec reset;
#endif
	void (*irq_configure)(void);
};

struct dma_gd32_channel {
	dma_callback_t callback;
	void *user_data;
	uint8_t direction;
	uint8_t periph_width;
	uint8_t memory_width;
	uint8_t flags;
	uint32_t block_size;
	uint32_t peripheral_address;
};

#define DMA_GD32_CHANNEL_CONFIGURED BIT(0)
#define DMA_GD32_CHANNEL_CYCLIC     BIT(1)
#define DMA_GD32_CHANNEL_FIFO_MODE  BIT(2)
#define DMA_GD32_CHANNEL_POISONED   BIT(3)
#define DMA_GD32_CHANNEL_SWITCH_BUF BIT(4)
#define DMA_GD32_STOP_TIMEOUT_MS    2U

struct dma_gd32_data {
	struct dma_context ctx;
	struct dma_gd32_channel *channels;
};

struct dma_gd32_srcdst_config {
	uint32_t addr;
	uint32_t adj;
	uint32_t width;
};

static bool dma_gd32_channel_enabled(const struct dma_gd32_config *cfg, uint32_t channel)
{
	return (GD32_DMA_CHCTL(cfg->reg, channel) & DMA_CHXCTL_CHEN) != 0U;
}

static int dma_gd32_validate_transfer_size_and_alignment(
	const struct device *dev, uint32_t channel, const struct dma_config *dma_cfg,
	const struct dma_gd32_srcdst_config *src_cfg, const struct dma_gd32_srcdst_config *dst_cfg,
	const struct dma_gd32_srcdst_config *memory_cfg,
	const struct dma_gd32_srcdst_config *periph_cfg, uint32_t item_count)
{
	uint32_t block_size = dma_cfg->head_block->block_size;

	if ((block_size == 0U) || ((block_size % periph_cfg->width) != 0U)) {
		LOG_ERR("%s ch%" PRIu32 " invalid block size %" PRIu32
			" for peripheral width %" PRIu32,
			dev->name, channel, block_size, periph_cfg->width);
		return -EINVAL;
	}
	if ((block_size % memory_cfg->width) != 0U) {
		LOG_ERR("%s ch%" PRIu32 " invalid block size %" PRIu32 " for memory width %" PRIu32,
			dev->name, channel, block_size, memory_cfg->width);
		return -EINVAL;
	}
	if (item_count > UINT16_MAX) {
		LOG_ERR("%s ch%" PRIu32 " transfer count %" PRIu32 " exceeds hardware limit",
			dev->name, channel, item_count);
		return -EINVAL;
	}

	if ((src_cfg->addr % src_cfg->width) != 0U) {
		LOG_ERR("%s ch%" PRIu32 " unaligned source 0x%08" PRIx32 " for width %" PRIu32,
			dev->name, channel, src_cfg->addr, src_cfg->width);
		return -EINVAL;
	}

	if ((dst_cfg->addr % dst_cfg->width) != 0U) {
		LOG_ERR("%s ch%" PRIu32 " unaligned dest 0x%08" PRIx32 " for width %" PRIu32,
			dev->name, channel, dst_cfg->addr, dst_cfg->width);
		return -EINVAL;
	}

	return 0;
}

#if DT_HAS_COMPAT_STATUS_OKAY(gd_gd32_dma_v1)
static int dma_gd32_burst_bytes_to_beats(uint32_t bytes, uint32_t width, uint32_t *beats)
{
	if (bytes == 0U) {
		*beats = 1U;
		return 0;
	}
	if ((bytes % width) != 0U) {
		return -EINVAL;
	}

	*beats = bytes / width;
	switch (*beats) {
	case 1U:
	case 4U:
	case 8U:
	case 16U:
		return 0;
	default:
		return -ENOTSUP;
	}
}

static inline uint32_t dma_gd32_fifo_threshold(uint16_t fifo_mode_control)
{
	switch (GD32_DMA_FEATURES_FIFO_THRESHOLD(fifo_mode_control)) {
	case 0U:
		return DMA_FIFO_1_WORD;
	case 1U:
		return DMA_FIFO_2_WORD;
	case 2U:
		return DMA_FIFO_3_WORD;
	default:
		return DMA_FIFO_4_WORD;
	}
}

static inline int dma_gd32_burst_cfg(uint32_t beats, bool memory_burst, uint32_t *out)
{
	switch (beats) {
	case 1U:
		*out = memory_burst ? DMA_MEMORY_BURST_SINGLE : DMA_PERIPH_BURST_SINGLE;
		return 0;
	case 4U:
		*out = memory_burst ? DMA_MEMORY_BURST_4_BEAT : DMA_PERIPH_BURST_4_BEAT;
		return 0;
	case 8U:
		*out = memory_burst ? DMA_MEMORY_BURST_8_BEAT : DMA_PERIPH_BURST_8_BEAT;
		return 0;
	case 16U:
		*out = memory_burst ? DMA_MEMORY_BURST_16_BEAT : DMA_PERIPH_BURST_16_BEAT;
		return 0;
	default:
		return -ENOTSUP;
	}
}

static int dma_gd32_validate_fifo(const struct device *dev, uint32_t channel,
				  const struct dma_config *dma_cfg,
				  const struct dma_gd32_srcdst_config *memory_cfg,
				  const struct dma_gd32_srcdst_config *periph_cfg,
				  uint32_t memory_beats, uint32_t peripheral_beats,
				  bool use_multidata)
{
	uint16_t mode = dma_cfg->head_block->fifo_mode_control;

	if ((mode & ~(GD32_DMA_FEATURES_FIFO_REQUEST | 0x3U)) != 0U) {
		LOG_ERR("%s ch%" PRIu32 " invalid FIFO mode 0x%x", dev->name, channel, mode);
		return -EINVAL;
	}
	if (!use_multidata) {
		return 0;
	}

	uint32_t fifo_words = GD32_DMA_FEATURES_FIFO_THRESHOLD(mode) + 1U;
	uint32_t fifo_bytes = fifo_words * sizeof(uint32_t);
	uint32_t memory_burst_bytes = memory_beats * memory_cfg->width;

	if ((fifo_bytes % memory_burst_bytes) != 0U) {
		LOG_ERR("%s ch%" PRIu32 " FIFO %" PRIu32
			" bytes is incompatible with memory burst %" PRIu32 " bytes",
			dev->name, channel, fifo_bytes, memory_burst_bytes);
		return -EINVAL;
	}
	if ((dma_cfg->channel_direction == PERIPHERAL_TO_MEMORY) &&
	    ((peripheral_beats * periph_cfg->width) == 16U) && (fifo_words == 3U)) {
		LOG_ERR("%s ch%" PRIu32 " three-word FIFO can freeze a 16-byte peripheral burst",
			dev->name, channel);
		return -EINVAL;
	}

	return 0;
}
#endif

/*
 * Register access functions
 */

static inline void gd32_dma_periph_increase_enable(uint32_t reg, dma_channel_enum ch)
{
	GD32_DMA_CHCTL(reg, ch) |= DMA_CHXCTL_PNAGA;
}

static inline void gd32_dma_periph_increase_disable(uint32_t reg, dma_channel_enum ch)
{
	GD32_DMA_CHCTL(reg, ch) &= ~DMA_CHXCTL_PNAGA;
}

static inline void gd32_dma_transfer_set_memory_to_memory(uint32_t reg, dma_channel_enum ch)
{
	GD32_DMA_CHCTL(reg, ch) |= GD32_DMA_CHXCTL_M2M;
	GD32_DMA_CHCTL(reg, ch) &= ~GD32_DMA_CHXCTL_DIR;
}

static inline void gd32_dma_transfer_set_memory_to_periph(uint32_t reg, dma_channel_enum ch)
{
	GD32_DMA_CHCTL(reg, ch) &= ~GD32_DMA_CHXCTL_M2M;
	GD32_DMA_CHCTL(reg, ch) |= GD32_DMA_CHXCTL_DIR;
}

static inline void gd32_dma_transfer_set_periph_to_memory(uint32_t reg, dma_channel_enum ch)
{
	GD32_DMA_CHCTL(reg, ch) &= ~GD32_DMA_CHXCTL_M2M;
	GD32_DMA_CHCTL(reg, ch) &= ~GD32_DMA_CHXCTL_DIR;
}

static inline void gd32_dma_memory_increase_enable(uint32_t reg, dma_channel_enum ch)
{
	GD32_DMA_CHCTL(reg, ch) |= DMA_CHXCTL_MNAGA;
}

static inline void gd32_dma_memory_increase_disable(uint32_t reg, dma_channel_enum ch)
{
	GD32_DMA_CHCTL(reg, ch) &= ~DMA_CHXCTL_MNAGA;
}

static inline void gd32_dma_circulation_enable(uint32_t reg, dma_channel_enum ch)
{
	GD32_DMA_CHCTL(reg, ch) |= DMA_CHXCTL_CMEN;
}

static inline void gd32_dma_circulation_disable(uint32_t reg, dma_channel_enum ch)
{
	GD32_DMA_CHCTL(reg, ch) &= ~DMA_CHXCTL_CMEN;
}

static inline void gd32_dma_channel_enable(uint32_t reg, dma_channel_enum ch)
{
	GD32_DMA_CHCTL(reg, ch) |= DMA_CHXCTL_CHEN;
}

static inline void gd32_dma_channel_disable(uint32_t reg, dma_channel_enum ch)
{
	GD32_DMA_CHCTL(reg, ch) &= ~DMA_CHXCTL_CHEN;
}

static inline void gd32_dma_interrupt_enable(uint32_t reg, dma_channel_enum ch, uint32_t source)
{
	GD32_DMA_CHCTL(reg, ch) |= source;
}

static inline void gd32_dma_interrupt_disable(uint32_t reg, dma_channel_enum ch, uint32_t source)
{
	GD32_DMA_CHCTL(reg, ch) &= ~source;
}

#if DT_HAS_COMPAT_STATUS_OKAY(gd_gd32_dma_v1) && defined(DMA_CHXFCTL_FEEIE)
static inline void gd32_dma_fifo_error_interrupt_enable(uint32_t reg, dma_channel_enum ch)
{
	DMA_CHFCTL(reg, ch) |= DMA_CHXFCTL_FEEIE;
}

static inline void gd32_dma_fifo_error_interrupt_disable(uint32_t reg, dma_channel_enum ch)
{
	DMA_CHFCTL(reg, ch) &= ~DMA_CHXFCTL_FEEIE;
}
#else
static inline void gd32_dma_fifo_error_interrupt_enable(uint32_t reg, dma_channel_enum ch)
{
	ARG_UNUSED(reg);
	ARG_UNUSED(ch);
}

static inline void gd32_dma_fifo_error_interrupt_disable(uint32_t reg, dma_channel_enum ch)
{
	ARG_UNUSED(reg);
	ARG_UNUSED(ch);
}
#endif

static inline void gd32_dma_priority_config(uint32_t reg, dma_channel_enum ch, uint32_t priority)
{
	uint32_t ctl = GD32_DMA_CHCTL(reg, ch);

	GD32_DMA_CHCTL(reg, ch) = (ctl & (~DMA_CHXCTL_PRIO)) | priority;
}

static inline void gd32_dma_memory_width_config(uint32_t reg, dma_channel_enum ch, uint32_t mwidth)
{
	uint32_t ctl = GD32_DMA_CHCTL(reg, ch);

	GD32_DMA_CHCTL(reg, ch) = (ctl & (~DMA_CHXCTL_MWIDTH)) | mwidth;
}

static inline void gd32_dma_periph_width_config(uint32_t reg, dma_channel_enum ch, uint32_t pwidth)
{
	uint32_t ctl = GD32_DMA_CHCTL(reg, ch);

	GD32_DMA_CHCTL(reg, ch) = (ctl & (~DMA_CHXCTL_PWIDTH)) | pwidth;
}

#if DT_HAS_COMPAT_STATUS_OKAY(gd_gd32_dma_v1)
static inline void gd32_dma_channel_subperipheral_select(uint32_t reg, dma_channel_enum ch,
							 dma_subperipheral_enum sub_periph)
{
	uint32_t ctl = GD32_DMA_CHCTL(reg, ch);

	GD32_DMA_CHCTL(reg, ch) =
		(ctl & (~DMA_CHXCTL_PERIEN)) | ((uint32_t)sub_periph << CHXCTL_PERIEN_OFFSET);
}
#endif

static inline void gd32_dma_periph_address_config(uint32_t reg, dma_channel_enum ch, uint32_t addr)
{
	GD32_DMA_CHPADDR(reg, ch) = addr;
}

static inline void gd32_dma_memory_address_config(uint32_t reg, dma_channel_enum ch, uint32_t addr)
{
#if DT_HAS_COMPAT_STATUS_OKAY(gd_gd32_dma_v1)
	DMA_CHM0ADDR(reg, ch) = addr;
#else
	GD32_DMA_CHMADDR(reg, ch) = addr;
#endif
}

#if DT_HAS_COMPAT_STATUS_OKAY(gd_gd32_dma_v1)
static inline void gd32_dma_memory1_address_config(uint32_t reg, dma_channel_enum ch, uint32_t addr)
{
	DMA_CHM1ADDR(reg, ch) = addr;
}

static inline bool gd32_dma_uses_memory1(uint32_t reg, dma_channel_enum ch)
{
	return (GD32_DMA_CHCTL(reg, ch) & DMA_CHXCTL_MBS) != 0U;
}
#endif

static inline void gd32_dma_transfer_number_config(uint32_t reg, dma_channel_enum ch, uint32_t num)
{
	GD32_DMA_CHCNT(reg, ch) = (num & DMA_CHXCNT_CNT);
}

static inline uint32_t gd32_dma_transfer_number_get(uint32_t reg, dma_channel_enum ch)
{
	return GD32_DMA_CHCNT(reg, ch);
}

static inline void gd32_dma_interrupt_flag_clear(uint32_t reg, dma_channel_enum ch, uint32_t flag)
{
#if DT_HAS_COMPAT_STATUS_OKAY(gd_gd32_dma_v1)
	if (ch < DMA_CH4) {
		DMA_INTC0(reg) |= DMA_FLAG_ADD(flag, ch);
	} else {
		DMA_INTC1(reg) |= DMA_FLAG_ADD(flag, ch - DMA_CH4);
	}
#else
	GD32_DMA_INTC(reg) |= DMA_FLAG_ADD(flag, ch);
#endif
}

static inline void gd32_dma_flag_clear(uint32_t reg, dma_channel_enum ch, uint32_t flag)
{
#if DT_HAS_COMPAT_STATUS_OKAY(gd_gd32_dma_v1)
	if (ch < DMA_CH4) {
		DMA_INTC0(reg) |= DMA_FLAG_ADD(flag, ch);
	} else {
		DMA_INTC1(reg) |= DMA_FLAG_ADD(flag, ch - DMA_CH4);
	}
#else
	GD32_DMA_INTC(reg) |= DMA_FLAG_ADD(flag, ch);
#endif
}

static inline uint32_t gd32_dma_interrupt_flag_get(uint32_t reg, dma_channel_enum ch, uint32_t flag)
{
#if DT_HAS_COMPAT_STATUS_OKAY(gd_gd32_dma_v1)
	if (ch < DMA_CH4) {
		return (DMA_INTF0(reg) & DMA_FLAG_ADD(flag, ch));
	} else {
		return (DMA_INTF1(reg) & DMA_FLAG_ADD(flag, ch - DMA_CH4));
	}
#else
	return (GD32_DMA_INTF(reg) & DMA_FLAG_ADD(flag, ch));
#endif
}

static inline void gd32_dma_deinit(uint32_t reg, dma_channel_enum ch)
{
	GD32_DMA_CHCTL(reg, ch) &= ~DMA_CHXCTL_CHEN;

	GD32_DMA_CHCTL(reg, ch) = DMA_CHCTL_RESET_VALUE;
	GD32_DMA_CHCNT(reg, ch) = DMA_CHCNT_RESET_VALUE;
	GD32_DMA_CHPADDR(reg, ch) = DMA_CHPADDR_RESET_VALUE;
#if DT_HAS_COMPAT_STATUS_OKAY(gd_gd32_dma_v1)
	DMA_CHM0ADDR(reg, ch) = DMA_CHMADDR_RESET_VALUE;
	DMA_CHFCTL(reg, ch) = DMA_CHFCTL_RESET_VALUE;
	if (ch < DMA_CH4) {
		DMA_INTC0(reg) |= DMA_FLAG_ADD(DMA_CHINTF_RESET_VALUE, ch);
	} else {
		DMA_INTC1(reg) |= DMA_FLAG_ADD(DMA_CHINTF_RESET_VALUE, ch - DMA_CH4);
	}
#else
	GD32_DMA_CHMADDR(reg, ch) = DMA_CHMADDR_RESET_VALUE;
	GD32_DMA_INTC(reg) |= DMA_FLAG_ADD(DMA_CHINTF_RESET_VALUE, ch);
#endif
}

/*
 * Utility functions
 */

static inline uint32_t dma_gd32_priority(uint32_t prio)
{
	return CHCTL_PRIO(prio);
}

static inline uint32_t dma_gd32_memory_width(uint32_t width)
{
	switch (width) {
	case 4:
		return CHCTL_MWIDTH(2);
	case 2:
		return CHCTL_MWIDTH(1);
	default:
		return CHCTL_MWIDTH(0);
	}
}

static inline uint32_t dma_gd32_periph_width(uint32_t width)
{
	switch (width) {
	case 4:
		return CHCTL_PWIDTH(2);
	case 2:
		return CHCTL_PWIDTH(1);
	default:
		return CHCTL_PWIDTH(0);
	}
}

/*
 * API functions
 */

static int dma_gd32_config(const struct device *dev, uint32_t channel, struct dma_config *dma_cfg)
{
	const struct dma_gd32_config *cfg = dev->config;
	struct dma_gd32_data *data = dev->data;
	struct dma_gd32_srcdst_config src_cfg;
	struct dma_gd32_srcdst_config dst_cfg;
	struct dma_gd32_srcdst_config *memory_cfg = NULL;
	struct dma_gd32_srcdst_config *periph_cfg = NULL;
	bool use_multidata = false;
	bool use_switch_buffer = false;
#if DT_HAS_COMPAT_STATUS_OKAY(gd_gd32_dma_v1)
	uint32_t mem_burst = 0U;
	uint32_t periph_burst = 0U;
	uint32_t memory_beats = 1U;
	uint32_t peripheral_beats = 1U;
	uint32_t source_beats;
	uint32_t dest_beats;
#endif
	uint32_t item_count;
	int ret;

	if (channel >= cfg->channels) {
		LOG_ERR("channel must be < %" PRIu32 " (%" PRIu32 ")", cfg->channels, channel);
		return -EINVAL;
	}
	if ((data->channels[channel].flags & DMA_GD32_CHANNEL_POISONED) != 0U) {
		return -EIO;
	}
	if ((dma_cfg == NULL) || (dma_cfg->head_block == NULL)) {
		return -EINVAL;
	}
	if (dma_gd32_channel_enabled(cfg, channel)) {
		return -EBUSY;
	}

	if (dma_cfg->block_count != 1U) {
#if DT_HAS_COMPAT_STATUS_OKAY(gd_gd32_dma_v1)
		if (dma_cfg->block_count != 2U || !dma_cfg->cyclic ||
		    dma_cfg->head_block->next_block == NULL ||
		    dma_cfg->head_block->next_block->next_block != NULL) {
			LOG_ERR("only a two-block cyclic transfer list is supported");
			return -ENOTSUP;
		}
		use_switch_buffer = true;
#else
		LOG_ERR("chained block transfer not supported");
		return -ENOTSUP;
#endif
	}

	if (dma_cfg->channel_priority > 3) {
		LOG_ERR("channel_priority must be < 4 (%" PRIu32 ")", dma_cfg->channel_priority);
		return -EINVAL;
	}

	if (dma_cfg->head_block->source_addr_adj == DMA_ADDR_ADJ_DECREMENT) {
		LOG_ERR("source_addr_adj not supported DMA_ADDR_ADJ_DECREMENT");
		return -ENOTSUP;
	}

	if (dma_cfg->head_block->dest_addr_adj == DMA_ADDR_ADJ_DECREMENT) {
		LOG_ERR("dest_addr_adj not supported DMA_ADDR_ADJ_DECREMENT");
		return -ENOTSUP;
	}

	if (dma_cfg->head_block->source_addr_adj != DMA_ADDR_ADJ_INCREMENT &&
	    dma_cfg->head_block->source_addr_adj != DMA_ADDR_ADJ_NO_CHANGE) {
		LOG_ERR("invalid source_addr_adj %" PRIu16, dma_cfg->head_block->source_addr_adj);
		return -ENOTSUP;
	}
	if (dma_cfg->head_block->dest_addr_adj != DMA_ADDR_ADJ_INCREMENT &&
	    dma_cfg->head_block->dest_addr_adj != DMA_ADDR_ADJ_NO_CHANGE) {
		LOG_ERR("invalid dest_addr_adj %" PRIu16, dma_cfg->head_block->dest_addr_adj);
		return -ENOTSUP;
	}

	if (dma_cfg->source_data_size != 1 && dma_cfg->source_data_size != 2 &&
	    dma_cfg->source_data_size != 4) {
		LOG_ERR("source_data_size must be 1, 2, or 4 (%" PRIu32 ")",
			dma_cfg->source_data_size);
		return -EINVAL;
	}

	if (dma_cfg->dest_data_size != 1 && dma_cfg->dest_data_size != 2 &&
	    dma_cfg->dest_data_size != 4) {
		LOG_ERR("dest_data_size must be 1, 2, or 4 (%" PRIu32 ")", dma_cfg->dest_data_size);
		return -EINVAL;
	}

	if (dma_cfg->channel_direction > PERIPHERAL_TO_MEMORY) {
		LOG_ERR("channel_direction must be MEMORY_TO_MEMORY, "
			"MEMORY_TO_PERIPHERAL or PERIPHERAL_TO_MEMORY (%" PRIu32 ")",
			dma_cfg->channel_direction);
		return -ENOTSUP;
	}

	if (dma_cfg->channel_direction == MEMORY_TO_MEMORY && !cfg->mem2mem) {
		LOG_ERR("not supporting MEMORY_TO_MEMORY");
		return -ENOTSUP;
	}

#if DT_HAS_COMPAT_STATUS_OKAY(gd_gd32_dma_v1)
	if (dma_cfg->dma_slot > 7U) {
		LOG_ERR("dma_slot must be <= 7 (%" PRIu32 ")", dma_cfg->dma_slot);
		return -EINVAL;
	}
#else
	if (dma_cfg->head_block->flow_control_mode != 0U) {
		return -ENOTSUP;
	}
#endif

	src_cfg.addr = dma_cfg->head_block->source_address;
	src_cfg.adj = dma_cfg->head_block->source_addr_adj;
	src_cfg.width = dma_cfg->source_data_size;

	dst_cfg.addr = dma_cfg->head_block->dest_address;
	dst_cfg.adj = dma_cfg->head_block->dest_addr_adj;
	dst_cfg.width = dma_cfg->dest_data_size;

	switch (dma_cfg->channel_direction) {
	case MEMORY_TO_MEMORY:
		memory_cfg = &dst_cfg;
		periph_cfg = &src_cfg;
		break;
	case PERIPHERAL_TO_MEMORY:
		memory_cfg = &dst_cfg;
		periph_cfg = &src_cfg;
		break;
	case MEMORY_TO_PERIPHERAL:
		memory_cfg = &src_cfg;
		periph_cfg = &dst_cfg;
		break;
	}

#if DT_HAS_COMPAT_STATUS_OKAY(gd_gd32_dma_v1)
	if (use_switch_buffer) {
		const struct dma_block_config *second = dma_cfg->head_block->next_block;
		uint32_t second_memory_address;
		uint32_t second_peripheral_address;

		if (dma_cfg->channel_direction == MEMORY_TO_MEMORY ||
		    second->block_size != dma_cfg->head_block->block_size ||
		    second->source_addr_adj != dma_cfg->head_block->source_addr_adj ||
		    second->dest_addr_adj != dma_cfg->head_block->dest_addr_adj ||
		    second->fifo_mode_control != dma_cfg->head_block->fifo_mode_control ||
		    second->flow_control_mode != dma_cfg->head_block->flow_control_mode) {
			return -ENOTSUP;
		}

		if (dma_cfg->channel_direction == MEMORY_TO_PERIPHERAL) {
			second_memory_address = second->source_address;
			second_peripheral_address = second->dest_address;
		} else {
			second_memory_address = second->dest_address;
			second_peripheral_address = second->source_address;
		}

		if (second_peripheral_address != periph_cfg->addr ||
		    (second_memory_address % memory_cfg->width) != 0U) {
			return -EINVAL;
		}
	}
#endif

	item_count = dma_cfg->head_block->block_size / periph_cfg->width;

#if DT_HAS_COMPAT_STATUS_OKAY(gd_gd32_dma_v1)
	ret = dma_gd32_burst_bytes_to_beats(dma_cfg->source_burst_length, src_cfg.width,
					    &source_beats);
	if (ret != 0) {
		LOG_ERR("unsupported source burst length: %" PRIu32, dma_cfg->source_burst_length);
		return ret;
	}
	ret = dma_gd32_burst_bytes_to_beats(dma_cfg->dest_burst_length, dst_cfg.width, &dest_beats);
	if (ret != 0) {
		LOG_ERR("unsupported destination burst length: %" PRIu32,
			dma_cfg->dest_burst_length);
		return ret;
	}

	if (dma_cfg->channel_direction == MEMORY_TO_PERIPHERAL) {
		memory_beats = source_beats;
		peripheral_beats = dest_beats;
	} else {
		memory_beats = dest_beats;
		peripheral_beats = source_beats;
	}
	use_multidata = (dma_cfg->channel_direction == MEMORY_TO_MEMORY) ||
			(memory_cfg->width != periph_cfg->width) || (memory_beats > 1U) ||
			(peripheral_beats > 1U) ||
			GD32_DMA_FEATURES_FIFO_REQUESTED(dma_cfg->head_block->fifo_mode_control);
#endif

	ret = dma_gd32_validate_transfer_size_and_alignment(
		dev, channel, dma_cfg, &src_cfg, &dst_cfg, memory_cfg, periph_cfg, item_count);
	if (ret != 0) {
		return ret;
	}

#if DT_HAS_COMPAT_STATUS_OKAY(gd_gd32_dma_v1)
	if (dma_cfg->cyclic && (dma_cfg->head_block->flow_control_mode != 0U)) {
		return -ENOTSUP;
	}
	if (dma_cfg->cyclic &&
	    (((item_count % peripheral_beats) != 0U) ||
	     (((item_count * periph_cfg->width) % (memory_beats * memory_cfg->width)) != 0U))) {
		return -EINVAL;
	}
	ret = dma_gd32_validate_fifo(dev, channel, dma_cfg, memory_cfg, periph_cfg, memory_beats,
				     peripheral_beats, use_multidata);
	if (ret != 0) {
		return ret;
	}
#endif

	gd32_dma_deinit(cfg->reg, channel);

	switch (dma_cfg->channel_direction) {
	case MEMORY_TO_MEMORY:
		gd32_dma_transfer_set_memory_to_memory(cfg->reg, channel);
		break;
	case PERIPHERAL_TO_MEMORY:
		gd32_dma_transfer_set_periph_to_memory(cfg->reg, channel);
		break;
	case MEMORY_TO_PERIPHERAL:
		gd32_dma_transfer_set_memory_to_periph(cfg->reg, channel);
		break;
	}

	gd32_dma_memory_address_config(cfg->reg, channel, memory_cfg->addr);
	if (memory_cfg->adj == DMA_ADDR_ADJ_INCREMENT) {
		gd32_dma_memory_increase_enable(cfg->reg, channel);
	} else {
		gd32_dma_memory_increase_disable(cfg->reg, channel);
	}

	gd32_dma_periph_address_config(cfg->reg, channel, periph_cfg->addr);
	if (periph_cfg->adj == DMA_ADDR_ADJ_INCREMENT) {
		gd32_dma_periph_increase_enable(cfg->reg, channel);
	} else {
		gd32_dma_periph_increase_disable(cfg->reg, channel);
	}

	LOG_DBG("%s ch%" PRIu32 " slot %" PRIu32 " dir %" PRIu32 " width m=%" PRIu32 " p=%" PRIu32
		" items=%" PRIu32,
		dev->name, channel, dma_cfg->dma_slot, dma_cfg->channel_direction,
		memory_cfg->width, periph_cfg->width, item_count);

	gd32_dma_transfer_number_config(cfg->reg, channel, item_count);
	gd32_dma_priority_config(cfg->reg, channel, dma_gd32_priority(dma_cfg->channel_priority));
	gd32_dma_memory_width_config(cfg->reg, channel, dma_gd32_memory_width(memory_cfg->width));
	gd32_dma_periph_width_config(cfg->reg, channel, dma_gd32_periph_width(periph_cfg->width));

#if DT_HAS_COMPAT_STATUS_OKAY(gd_gd32_dma_v1)
	if (use_switch_buffer) {
		const struct dma_block_config *second = dma_cfg->head_block->next_block;
		uint32_t second_memory_address;

		if (dma_cfg->channel_direction == MEMORY_TO_PERIPHERAL) {
			second_memory_address = second->source_address;
		} else {
			second_memory_address = second->dest_address;
		}

		gd32_dma_memory1_address_config(cfg->reg, channel, second_memory_address);
		GD32_DMA_CHCTL(cfg->reg, channel) &= ~DMA_CHXCTL_MBS;
		GD32_DMA_CHCTL(cfg->reg, channel) |= DMA_CHXCTL_SBMEN;
	}

	uint32_t ctl = GD32_DMA_CHCTL(cfg->reg, channel);
	uint32_t fctl = DMA_CHFCTL(cfg->reg, channel);

	ctl &= ~(DMA_CHXCTL_PBURST | DMA_CHXCTL_MBURST);
	GD32_DMA_CHCTL(cfg->reg, channel) = ctl;
	fctl &= ~(DMA_CHXFCTL_MDMEN | DMA_CHXFCTL_FCCV);
#if defined(DMA_CHXFCTL_FEEIE)
	fctl &= ~DMA_CHXFCTL_FEEIE;
#endif
	DMA_CHFCTL(cfg->reg, channel) = fctl;

	(void)dma_gd32_burst_cfg(memory_beats, true, &mem_burst);
	(void)dma_gd32_burst_cfg(peripheral_beats, false, &periph_burst);

	if (use_multidata) {
		DMA_CHFCTL(cfg->reg, channel) |=
			(DMA_CHXFCTL_MDMEN |
			 dma_gd32_fifo_threshold(dma_cfg->head_block->fifo_mode_control));

		ctl = GD32_DMA_CHCTL(cfg->reg, channel);
		ctl &= ~(DMA_CHXCTL_PBURST | DMA_CHXCTL_MBURST);
		ctl |= periph_burst | mem_burst;
		GD32_DMA_CHCTL(cfg->reg, channel) = ctl;
	}

	if (dma_cfg->head_block->flow_control_mode != 0U) {
		GD32_DMA_CHCTL(cfg->reg, channel) |= DMA_CHXCTL_TFCS;
	} else {
		GD32_DMA_CHCTL(cfg->reg, channel) &= ~DMA_CHXCTL_TFCS;
	}
#endif

	if (dma_cfg->cyclic) {
		gd32_dma_circulation_enable(cfg->reg, channel);
	} else {
		gd32_dma_circulation_disable(cfg->reg, channel);
	}
#if DT_HAS_COMPAT_STATUS_OKAY(gd_gd32_dma_v1)
	if (dma_cfg->channel_direction != MEMORY_TO_MEMORY) {
		gd32_dma_channel_subperipheral_select(cfg->reg, channel, dma_cfg->dma_slot);
	}
#endif

	data->channels[channel].callback = dma_cfg->dma_callback;
	data->channels[channel].user_data = dma_cfg->user_data;
	data->channels[channel].direction = dma_cfg->channel_direction;
	data->channels[channel].periph_width = periph_cfg->width;
	data->channels[channel].memory_width = memory_cfg->width;
	data->channels[channel].block_size = dma_cfg->head_block->block_size;
	data->channels[channel].peripheral_address = periph_cfg->addr;
	data->channels[channel].flags = DMA_GD32_CHANNEL_CONFIGURED |
					(dma_cfg->cyclic ? DMA_GD32_CHANNEL_CYCLIC : 0U) |
					(use_multidata ? DMA_GD32_CHANNEL_FIFO_MODE : 0U) |
					(use_switch_buffer ? DMA_GD32_CHANNEL_SWITCH_BUF : 0U);

	return 0;
}

static int dma_gd32_reload(const struct device *dev, uint32_t ch, uint32_t src, uint32_t dst,
			   size_t size)
{
	const struct dma_gd32_config *cfg = dev->config;
	struct dma_gd32_data *data = dev->data;

	if (ch >= cfg->channels) {
		LOG_ERR("reload channel must be < %" PRIu32 " (%" PRIu32 ")", cfg->channels, ch);
		return -EINVAL;
	}
	if ((data->channels[ch].flags & DMA_GD32_CHANNEL_POISONED) != 0U) {
		return -EIO;
	}

	if ((data->channels[ch].flags & DMA_GD32_CHANNEL_CONFIGURED) == 0U) {
		return -EINVAL;
	}
	if (dma_gd32_channel_enabled(cfg, ch)) {
		/*
		 * MBS can change between selecting the inactive address register and
		 * writing it.  In switch-buffer mode that race writes the active bank,
		 * raises TAE, and terminates the channel.  Keep both addresses immutable
		 * for the whole running epoch; clients refill the inactive buffers in
		 * place after querying dma_gd32_switch_buffer_active().
		 */
		return -EBUSY;
	}

	if ((size == 0U) || (data->channels[ch].periph_width == 0U) ||
	    ((size % data->channels[ch].periph_width) != 0U) ||
	    (size / data->channels[ch].periph_width > UINT16_MAX)) {
		LOG_ERR("%s ch%" PRIu32 " invalid reload size %zu for peripheral width %" PRIu32,
			dev->name, ch, size, data->channels[ch].periph_width);
		return -EINVAL;
	}

	gd32_dma_transfer_number_config(cfg->reg, ch, size / data->channels[ch].periph_width);

	switch (data->channels[ch].direction) {
	case MEMORY_TO_MEMORY:
	case PERIPHERAL_TO_MEMORY:
		if ((dst % data->channels[ch].memory_width) != 0U ||
		    (src % data->channels[ch].periph_width) != 0U) {
			return -EINVAL;
		}
		gd32_dma_memory_address_config(cfg->reg, ch, dst);
		gd32_dma_periph_address_config(cfg->reg, ch, src);
		break;
	case MEMORY_TO_PERIPHERAL:
		if ((src % data->channels[ch].memory_width) != 0U ||
		    (dst % data->channels[ch].periph_width) != 0U) {
			return -EINVAL;
		}
		gd32_dma_memory_address_config(cfg->reg, ch, src);
		gd32_dma_periph_address_config(cfg->reg, ch, dst);
		break;
	}

	gd32_dma_interrupt_flag_clear(cfg->reg, ch,
				      DMA_FLAG_FTF | DMA_FLAG_HTF | GD32_DMA_FLAG_ERRORS |
					      GD32_DMA_FLAG_EXCEPTIONS | GD32_DMA_FLAG_FIFO_EVENT);
	gd32_dma_interrupt_enable(
		cfg->reg, ch,
		DMA_CHXCTL_FTFIE | GD32_DMA_INTERRUPT_ERRORS | GD32_DMA_INTERRUPT_EXCEPTIONS |
			(((data->channels[ch].flags & DMA_GD32_CHANNEL_CYCLIC) != 0U &&
			  (data->channels[ch].flags & DMA_GD32_CHANNEL_SWITCH_BUF) == 0U)
				 ? DMA_CHXCTL_HTFIE
				 : 0U));
	if ((data->channels[ch].flags & DMA_GD32_CHANNEL_FIFO_MODE) != 0U) {
		gd32_dma_fifo_error_interrupt_enable(cfg->reg, ch);
	}
	gd32_dma_channel_enable(cfg->reg, ch);

	return 0;
}

static int dma_gd32_get_attribute(const struct device *dev, uint32_t type, uint32_t *value)
{
	ARG_UNUSED(dev);

	if (value == NULL) {
		return -EINVAL;
	}

	switch ((enum dma_attribute_type)type) {
	case DMA_ATTR_BUFFER_ADDRESS_ALIGNMENT:
	case DMA_ATTR_BUFFER_SIZE_ALIGNMENT:
	case DMA_ATTR_COPY_ALIGNMENT:
		*value = 1U;
		return 0;
	case DMA_ATTR_MAX_BLOCK_COUNT:
#if DT_HAS_COMPAT_STATUS_OKAY(gd_gd32_dma_v1)
		*value = 2U;
#else
		*value = 1U;
#endif
		return 0;
	default:
		return -EINVAL;
	}
}

static int dma_gd32_start(const struct device *dev, uint32_t ch)
{
	const struct dma_gd32_config *cfg = dev->config;
	struct dma_gd32_data *data = dev->data;

	if (ch >= cfg->channels) {
		LOG_ERR("start channel must be < %" PRIu32 " (%" PRIu32 ")", cfg->channels, ch);
		return -EINVAL;
	}
	if ((data->channels[ch].flags & DMA_GD32_CHANNEL_POISONED) != 0U) {
		return -EIO;
	}
	if ((data->channels[ch].flags & DMA_GD32_CHANNEL_CONFIGURED) == 0U) {
		return -EINVAL;
	}
	if (dma_gd32_channel_enabled(cfg, ch)) {
		return 0;
	}

	/* Clear any stale flags before starting a new transfer. */
	gd32_dma_interrupt_flag_clear(cfg->reg, ch,
				      DMA_FLAG_FTF | DMA_FLAG_HTF | GD32_DMA_FLAG_ERRORS |
					      GD32_DMA_FLAG_EXCEPTIONS | GD32_DMA_FLAG_FIFO_EVENT);

	gd32_dma_interrupt_enable(
		cfg->reg, ch,
		DMA_CHXCTL_FTFIE | GD32_DMA_INTERRUPT_ERRORS | GD32_DMA_INTERRUPT_EXCEPTIONS |
			((data->channels[ch].flags & DMA_GD32_CHANNEL_CYCLIC) != 0U &&
					 (data->channels[ch].flags & DMA_GD32_CHANNEL_SWITCH_BUF) ==
						 0U
				 ? DMA_CHXCTL_HTFIE
				 : 0U));
	if ((data->channels[ch].flags & DMA_GD32_CHANNEL_FIFO_MODE) != 0U) {
		gd32_dma_fifo_error_interrupt_enable(cfg->reg, ch);
	}
	gd32_dma_channel_enable(cfg->reg, ch);

	return 0;
}

static int dma_gd32_stop(const struct device *dev, uint32_t ch)
{
	const struct dma_gd32_config *cfg = dev->config;
	struct dma_gd32_data *data = dev->data;
	int64_t deadline;

	if (ch >= cfg->channels) {
		LOG_ERR("stop channel must be < %" PRIu32 " (%" PRIu32 ")", cfg->channels, ch);
		return -EINVAL;
	}

	gd32_dma_interrupt_disable(cfg->reg, ch,
				   DMA_CHXCTL_FTFIE | GD32_DMA_INTERRUPT_ERRORS |
					   GD32_DMA_INTERRUPT_EXCEPTIONS | DMA_CHXCTL_HTFIE);
	gd32_dma_fifo_error_interrupt_disable(cfg->reg, ch);
	gd32_dma_channel_disable(cfg->reg, ch);

	if (dma_gd32_channel_enabled(cfg, ch) && k_is_in_isr()) {
		return -EWOULDBLOCK;
	}

	deadline = k_uptime_get() + DMA_GD32_STOP_TIMEOUT_MS;
	while (dma_gd32_channel_enabled(cfg, ch) && k_uptime_get() < deadline) {
		k_sleep(K_MSEC(1));
	}

	if (dma_gd32_channel_enabled(cfg, ch)) {
		data->channels[ch].flags |= DMA_GD32_CHANNEL_POISONED;
		return -ETIMEDOUT;
	}

	gd32_dma_interrupt_flag_clear(cfg->reg, ch,
				      DMA_FLAG_FTF | DMA_FLAG_HTF | GD32_DMA_FLAG_ERRORS |
					      GD32_DMA_FLAG_EXCEPTIONS | GD32_DMA_FLAG_FIFO_EVENT);

	return 0;
}

static int dma_gd32_get_status(const struct device *dev, uint32_t ch, struct dma_status *stat)
{
	const struct dma_gd32_config *cfg = dev->config;
	struct dma_gd32_data *data = dev->data;

	if (ch >= cfg->channels) {
		LOG_ERR("channel must be < %" PRIu32 " (%" PRIu32 ")", cfg->channels, ch);
		return -EINVAL;
	}
	if ((stat == NULL) || ((data->channels[ch].flags & DMA_GD32_CHANNEL_CONFIGURED) == 0U)) {
		return -EINVAL;
	}

	stat->pending_length =
		gd32_dma_transfer_number_get(cfg->reg, ch) * data->channels[ch].periph_width;
	stat->dir = data->channels[ch].direction;
	stat->busy = dma_gd32_channel_enabled(cfg, ch);

	return 0;
}

int dma_gd32_switch_buffer_active(const struct device *dev, uint32_t ch, uint8_t *active_bank)
{
#if DT_HAS_COMPAT_STATUS_OKAY(gd_gd32_dma_v1)
	const struct dma_gd32_config *cfg;
	struct dma_gd32_data *data;

	if (dev == NULL || active_bank == NULL) {
		return -EINVAL;
	}

	cfg = dev->config;
	data = dev->data;
	if (ch >= cfg->channels ||
	    (data->channels[ch].flags & DMA_GD32_CHANNEL_CONFIGURED) == 0U) {
		return -EINVAL;
	}
	if ((data->channels[ch].flags & DMA_GD32_CHANNEL_SWITCH_BUF) == 0U) {
		return -ENOTSUP;
	}

	*active_bank = gd32_dma_uses_memory1(cfg->reg, ch) ? 1U : 0U;
	return 0;
#else
	ARG_UNUSED(dev);
	ARG_UNUSED(ch);
	ARG_UNUSED(active_bank);
	return -ENOTSUP;
#endif
}

static bool dma_gd32_api_chan_filter(const struct device *dev, int ch, void *filter_param)
{
	uint32_t filter;

	if (!filter_param) {
		LOG_ERR("filter_param must not be NULL");
		return false;
	}

	filter = *((uint32_t *)filter_param);

	return (filter & BIT(ch));
}

static int dma_gd32_init(const struct device *dev)
{
	const struct dma_gd32_config *cfg = dev->config;

	(void)clock_control_on(GD32_CLOCK_CONTROLLER, (clock_control_subsys_t)&cfg->clkid);

#if DT_HAS_COMPAT_STATUS_OKAY(gd_gd32_dma_v1)
	(void)reset_line_toggle_dt(&cfg->reset);
#endif

	for (uint32_t i = 0; i < cfg->channels; i++) {
		gd32_dma_interrupt_disable(cfg->reg, i,
					   DMA_CHXCTL_FTFIE | GD32_DMA_INTERRUPT_ERRORS |
						   GD32_DMA_INTERRUPT_EXCEPTIONS |
						   DMA_CHXCTL_HTFIE);
		gd32_dma_fifo_error_interrupt_disable(cfg->reg, i);
		gd32_dma_deinit(cfg->reg, i);
	}

	cfg->irq_configure();

	return 0;
}

static void dma_gd32_isr(const struct device *dev)
{
	const struct dma_gd32_config *cfg = dev->config;
	struct dma_gd32_data *data = dev->data;

	for (uint32_t i = 0; i < cfg->channels; i++) {
		uint32_t errflag = gd32_dma_interrupt_flag_get(cfg->reg, i, GD32_DMA_FLAG_ERRORS);
		uint32_t exception_flag =
			gd32_dma_interrupt_flag_get(cfg->reg, i, GD32_DMA_FLAG_EXCEPTIONS);
		uint32_t fifo_flag =
			gd32_dma_interrupt_flag_get(cfg->reg, i, GD32_DMA_FLAG_FIFO_EVENT);
		uint32_t ftfflag = gd32_dma_interrupt_flag_get(cfg->reg, i, DMA_FLAG_FTF);
		uint32_t htfflag = gd32_dma_interrupt_flag_get(cfg->reg, i, DMA_FLAG_HTF);
		bool channel_enabled = dma_gd32_channel_enabled(cfg, i);
		uint32_t remaining = gd32_dma_transfer_number_get(cfg->reg, i);

		if (errflag == 0 && exception_flag == 0 && fifo_flag == 0 && ftfflag == 0 &&
		    htfflag == 0) {
			continue;
		}
		/**
		 * FEEIF is ambiguous on GD32F4: it denotes both a non-destructive FIFO
		 * under/overrun exception and an invalid FIFO configuration. Invalid
		 * configurations are rejected by dma_gd32_validate_fifo(), and hardware
		 * detects any remaining configuration fault before transferring an item.
		 * Consequently, an inactive channel with items remaining is an error,
		 * while an inactive zero-count channel completed even if its FTFIF
		 * publication was lost. This also matches STM32F4 SDIO/DMA behavior.
		 */
		bool fifo_config_error = (fifo_flag != 0U) && !channel_enabled && (remaining != 0U);
		bool complete_without_ftf =
			(fifo_flag != 0U) && !channel_enabled && (remaining == 0U);
		int status;

		if ((errflag != 0U) || fifo_config_error) {
			status = -EIO;
		} else if ((ftfflag != 0U) || complete_without_ftf) {
			status = 0;
		} else {
			status = DMA_STATUS_HALF_COMPLETE;
		}

		gd32_dma_interrupt_flag_clear(cfg->reg, i,
					      DMA_FLAG_FTF | DMA_FLAG_HTF | GD32_DMA_FLAG_ERRORS |
						      GD32_DMA_FLAG_EXCEPTIONS |
						      GD32_DMA_FLAG_FIFO_EVENT);

		if (((exception_flag != 0U) || (fifo_flag != 0U)) && (status > 0)) {
			LOG_DBG("%s ch%" PRIu32 " nonfatal exception ex=0x%x fifo=0x%x", dev->name,
				i, exception_flag, fifo_flag);
			continue;
		}

		LOG_DBG("%s ch%" PRIu32 " isr err=0x%x ex=0x%x fifo=0x%x htf=0x%x ftf=0x%x "
			"cnt=%" PRIu32 " status=%d",
			dev->name, i, errflag, exception_flag, fifo_flag, htfflag, ftfflag,
			remaining, status);

		if (data->channels[i].callback) {
			data->channels[i].callback(dev, data->channels[i].user_data, i, status);
		}
	}
}

static DEVICE_API(dma, dma_gd32_driver_api) = {
	.config = dma_gd32_config,
	.reload = dma_gd32_reload,
	.start = dma_gd32_start,
	.stop = dma_gd32_stop,
	.get_status = dma_gd32_get_status,
	.get_attribute = dma_gd32_get_attribute,
	.chan_filter = dma_gd32_api_chan_filter,
};

#define IRQ_CONFIGURE(n, inst)                                                                     \
	IRQ_CONNECT(DT_INST_IRQ_BY_IDX(inst, n, irq), DT_INST_IRQ_BY_IDX(inst, n, priority),       \
		    dma_gd32_isr, DEVICE_DT_INST_GET(inst), 0);                                    \
	irq_enable(DT_INST_IRQ_BY_IDX(inst, n, irq));

#define CONFIGURE_ALL_IRQS(inst, n) LISTIFY(n, IRQ_CONFIGURE, (), inst)

#define GD32_DMA_INIT(inst)                                                                        \
	static void dma_gd32##inst##_irq_configure(void)                                           \
	{                                                                                          \
		CONFIGURE_ALL_IRQS(inst, DT_NUM_IRQS(DT_DRV_INST(inst)));                          \
	}                                                                                          \
	static const struct dma_gd32_config dma_gd32##inst##_config = {                            \
		.reg = DT_INST_REG_ADDR(inst),                                                     \
		.channels = DT_INST_PROP(inst, dma_channels),                                      \
		.clkid = DT_INST_CLOCKS_CELL(inst, id),                                            \
		.mem2mem = DT_INST_PROP(inst, gd_mem2mem),                                         \
		IF_ENABLED(DT_HAS_COMPAT_STATUS_OKAY(gd_gd32_dma_v1),          \
			   (.reset = RESET_DT_SPEC_INST_GET(inst),)) .irq_configure =   \
						 dma_gd32##inst##_irq_configure,                   \
	};                                                                                         \
                                                                                                   \
	static struct dma_gd32_channel                                                             \
		dma_gd32##inst##_channels[DT_INST_PROP(inst, dma_channels)];                       \
	ATOMIC_DEFINE(dma_gd32_atomic##inst, DT_INST_PROP(inst, dma_channels));                    \
	static struct dma_gd32_data dma_gd32##inst##_data = {                                      \
		.ctx =                                                                             \
			{                                                                          \
				.magic = DMA_MAGIC,                                                \
				.atomic = dma_gd32_atomic##inst,                                   \
				.dma_channels = DT_INST_PROP(inst, dma_channels),                  \
			},                                                                         \
		.channels = dma_gd32##inst##_channels,                                             \
	};                                                                                         \
                                                                                                   \
	DEVICE_DT_INST_DEFINE(inst, dma_gd32_init, NULL, &dma_gd32##inst##_data,                   \
			      &dma_gd32##inst##_config, PRE_KERNEL_1, CONFIG_DMA_INIT_PRIORITY,    \
			      &dma_gd32_driver_api);

DT_INST_FOREACH_STATUS_OKAY(GD32_DMA_INIT)
