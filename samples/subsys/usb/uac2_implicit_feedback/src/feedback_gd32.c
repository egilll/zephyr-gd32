/*
 * Copyright (c) 2026 Ylhyra ehf.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/device.h>
#include <zephyr/drivers/i2s/i2s_gd32.h>
#include <zephyr/drivers/usb/udc_dwc2.h>
#include <zephyr/kernel.h>
#include <zephyr/sys_clock.h>
#include <zephyr/sys/util.h>

#include "feedback.h"

/*
 * GD32 implementation notes
 * ------------------------
 *
 * The upstream sample's "ideal" requirement is a hardware timestamp of I2S
 * frame start relative to the USB SOF. Without changing the generic USB stack,
 * we approximate SOF time using the SOF callback execution time
 * (k_cycle_get_32()) and use a GD32 I2S driver-provided timestamp for the most
 * recent TX DMA block start.
 *
 * This is sufficient to make the sample functional on GD32F450, but the
 * achievable accuracy depends on interrupt latency and scheduling jitter.
 */

struct feedback_ctx {
	const struct device *i2s_dev;
	const struct device *udc_dev;

	/* Q16 fixed-point values (cycle_count << 16) */
	uint64_t cycles_per_sample_q16;
	uint64_t cycles_per_frame_q16;
	int64_t rel_sof_offset_q16;
	int64_t base_sof_offset_q16;

	unsigned int nominal;
};

static struct feedback_ctx fb_ctx;

static inline int64_t normalize_offset_q16(int64_t offset_q16, uint64_t period_q16)
{
	const int64_t half = (int64_t)(period_q16 / 2U);

	if (offset_q16 > half) {
		return offset_q16 - (int64_t)period_q16;
	}

	if (offset_q16 < -half) {
		return offset_q16 + (int64_t)period_q16;
	}

	return offset_q16;
}

struct feedback_ctx *feedback_init(void)
{
	fb_ctx.i2s_dev = DEVICE_DT_GET(DT_NODELABEL(i2s_rxtx));
	fb_ctx.udc_dev = DEVICE_DT_GET(DT_NODELABEL(zephyr_udc0));
	fb_ctx.cycles_per_sample_q16 = 0U;
	fb_ctx.cycles_per_frame_q16 = 0U;
	fb_ctx.rel_sof_offset_q16 = 0;
	fb_ctx.base_sof_offset_q16 = 0;
	fb_ctx.nominal = 0U;

	return &fb_ctx;
}

void feedback_process(struct feedback_ctx *ctx)
{
	uint32_t sof_cycle;
	uint32_t block_cycle;
	int32_t delta_cycles;
	int64_t delta_q16;

	if (ctx->cycles_per_frame_q16 == 0U || ctx->cycles_per_sample_q16 == 0U) {
		return;
	}

	sof_cycle = udc_dwc2_sof_cycle_get(ctx->udc_dev);
	block_cycle = i2s_gd32_tx_block_start_cycle_get(ctx->i2s_dev);

	delta_cycles = (int32_t)(block_cycle - sof_cycle);
	delta_q16 = ((int64_t)delta_cycles) << 16;

	ctx->rel_sof_offset_q16 = normalize_offset_q16(delta_q16, ctx->cycles_per_frame_q16);
}

void feedback_reset_ctx(struct feedback_ctx *ctx)
{
	ARG_UNUSED(ctx);
}

void feedback_start(struct feedback_ctx *ctx, int i2s_blocks_queued, bool microframes)
{
	uint64_t cycles_per_sec = sys_clock_hw_cycles_per_sec();

	ctx->nominal = microframes ? (SAMPLE_RATE / 8000U) : (SAMPLE_RATE / 1000U);

	ctx->cycles_per_sample_q16 = (cycles_per_sec << 16) / SAMPLE_RATE;
	ctx->cycles_per_frame_q16 = ctx->cycles_per_sample_q16 * ctx->nominal;

	/* Treat the initial state as "half-frame late" so the first normalized
	 * measurement won't accidentally wrap the wrong way.
	 */
	ctx->rel_sof_offset_q16 = (int64_t)(ctx->cycles_per_frame_q16 / 2U);

	/* If there are more than 2 I2S TX blocks queued, the I2S start is delayed.
	 * Convert the backlog (in frames) into a base offset estimate.
	 */
	ctx->base_sof_offset_q16 = (int64_t)MAX(i2s_blocks_queued - 2, 0) *
				   (int64_t)ctx->cycles_per_frame_q16;
}

int feedback_samples_offset(struct feedback_ctx *ctx)
{
	int64_t offset_q16;

	if (ctx->cycles_per_sample_q16 == 0U) {
		return 0;
	}

	offset_q16 = ctx->rel_sof_offset_q16 + ctx->base_sof_offset_q16;
	return (int)(offset_q16 / (int64_t)ctx->cycles_per_sample_q16);
}
