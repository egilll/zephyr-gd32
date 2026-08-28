/*
 * Copyright (c) 2026 Ylhyra ehf.
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdint.h>
#include <string.h>

#include <sample_usbd.h>

#include <zephyr/device.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/ring_buffer.h>
#include <zephyr/sys/util.h>
#include <zephyr/usb/usbd.h>
#include <zephyr/usb/usb_ch9.h>
#include <zephyr/usb/class/usbd_uac1.h>
#include <zephyr/drivers/i2s.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(uac1_sample, LOG_LEVEL_INF);

#if defined(CONFIG_BOARD_GD32F450Z_EVAL)
#define SAMPLE_I2S_AUDIO_FORMAT (I2S_FMT_DATA_FORMAT_LEFT_JUSTIFIED | I2S_FMT_BIT_CLK_INV)
#else
#define SAMPLE_I2S_AUDIO_FORMAT I2S_FMT_DATA_FORMAT_I2S
#endif

#define STREAMING_IN_TERMINAL_ID UAC1_ENTITY_ID(DT_NODELABEL(in_terminal))
#define FEATURE_UNIT_ID          UAC1_ENTITY_ID(DT_NODELABEL(feature_unit))

#define FS_SAMPLES_PER_SOF  48
#define HS_SAMPLES_PER_SOF  6
#define HS_BINTERVAL        USB_HS_ISO_EP_INTERVAL(DT_PROP_OR(DT_NODELABEL(as_iso_out), \
							      polling_period_us, 1000))
#define HS_INTERVAL_UF      (1U << (HS_BINTERVAL - 1))
#define HS_SAMPLES_PER_XFER (HS_SAMPLES_PER_SOF * HS_INTERVAL_UF)
#define MAX_SAMPLES_PER_XFER MAX(FS_SAMPLES_PER_SOF, HS_SAMPLES_PER_XFER)
#define SAMPLE_FREQUENCY    (FS_SAMPLES_PER_SOF * 1000)
#define SAMPLE_BIT_WIDTH    16
#define NUMBER_OF_CHANNELS  2
#define BYTES_PER_SAMPLE    DIV_ROUND_UP(SAMPLE_BIT_WIDTH, 8)
#define BYTES_PER_SLOT      (BYTES_PER_SAMPLE * NUMBER_OF_CHANNELS)
#define USB_PACKET_SIZE      (MAX_SAMPLES_PER_XFER * BYTES_PER_SLOT)
#define USB_PACKET_MAX_SIZE  ((MAX_SAMPLES_PER_XFER + 1) * BYTES_PER_SLOT)
#define I2S_BLOCK_SIZE       (USB_PACKET_SIZE * 4)

#define I2S_BUFFERS_COUNT   7
#define USB_BUFFERS_COUNT         2
#define PCM_FIFO_SIZE             (I2S_BLOCK_SIZE * 3)
#define GAIN_Q15_UNITY            BIT(15)
#define GAIN_RAMP_FRAMES          (SAMPLE_FREQUENCY / 100)
#define GAIN_RAMP_STEP            ((int32_t)DIV_ROUND_UP(GAIN_Q15_UNITY, GAIN_RAMP_FRAMES))
#define FEEDBACK_TARGET_BLOCKS    4
#define FEEDBACK_FILTER_SHIFT     6
#define FEEDBACK_MAX_ADJUST       ((int32_t)BIT(9))
#define FEEDBACK_ADJUST_PER_BLOCK (FEEDBACK_MAX_ADJUST / FEEDBACK_TARGET_BLOCKS)
/* 10^(-1/20), the linear amplitude change for -1 dB, in Q1.31. */
#define GAIN_FACTOR_1DB_Q31       1913946816U
K_MEM_SLAB_DEFINE_STATIC(usb_rx_slab, ROUND_UP(USB_PACKET_MAX_SIZE, UDC_BUF_GRANULARITY),
			 USB_BUFFERS_COUNT, UDC_BUF_ALIGN);
K_MEM_SLAB_DEFINE_STATIC(i2s_tx_slab, ROUND_UP(I2S_BLOCK_SIZE, UDC_BUF_GRANULARITY),
			 I2S_BUFFERS_COUNT, UDC_BUF_ALIGN);
RING_BUF_DECLARE(pcm_fifo, PCM_FIFO_SIZE);

struct usb_i2s_ctx {
	const struct device *i2s_dev;
	bool terminal_enabled;
	bool i2s_started;
	bool microframes;
	uint8_t i2s_blocks_written;
	int32_t fb_adjust;
	uint32_t fb_level_avg_q16;
	uint32_t fb_nominal_fs;
	uint32_t fb_nominal_hs;
	int16_t volume[NUMBER_OF_CHANNELS + 1];
	bool mute[NUMBER_OF_CHANNELS + 1];
	atomic_t target_gain[NUMBER_OF_CHANNELS];
	int32_t current_gain[NUMBER_OF_CHANNELS];
};

static int32_t volume_to_gain_q15(int32_t volume)
{
	uint32_t gain = BIT(31);
	uint32_t attenuation = DIV_ROUND_CLOSEST(-volume, 256);

	if (volume == INT16_MIN) {
		return 0;
	}

	for (uint32_t i = 0; i < attenuation; i++) {
		gain = ((uint64_t)gain * GAIN_FACTOR_1DB_Q31 + BIT(30)) >> 31;
	}

	return (gain + BIT(15)) >> 16;
}

static void update_channel_gains(struct usb_i2s_ctx *ctx)
{
	for (uint8_t channel = 1; channel <= NUMBER_OF_CHANNELS; channel++) {
		int32_t gain = 0;

		if (!ctx->mute[0] && !ctx->mute[channel]) {
			int32_t volume;

			if (ctx->volume[0] == INT16_MIN || ctx->volume[channel] == INT16_MIN) {
				atomic_set(&ctx->target_gain[channel - 1], 0);
				continue;
			}

			volume = ctx->volume[0] + ctx->volume[channel];

			gain = volume_to_gain_q15(MAX(volume, -90 * 256));
		}

		atomic_set(&ctx->target_gain[channel - 1], gain);
	}
}

static void apply_volume(struct usb_i2s_ctx *ctx, void *buf, uint16_t size)
{
	int16_t *samples = buf;
	size_t frames = size / BYTES_PER_SLOT;

	for (size_t frame = 0; frame < frames; frame++) {
		for (uint8_t channel = 0; channel < NUMBER_OF_CHANNELS; channel++) {
			int32_t target = atomic_get(&ctx->target_gain[channel]);
			int32_t delta = target - ctx->current_gain[channel];

			ctx->current_gain[channel] += CLAMP(delta, -GAIN_RAMP_STEP, GAIN_RAMP_STEP);
			samples[frame * NUMBER_OF_CHANNELS + channel] =
				((int32_t)samples[frame * NUMBER_OF_CHANNELS + channel] *
				 ctx->current_gain[channel]) /
				GAIN_Q15_UNITY;
		}
	}
}

static void reset_feedback_controller(struct usb_i2s_ctx *ctx)
{
	ctx->fb_adjust = 0;
	ctx->fb_level_avg_q16 = FEEDBACK_TARGET_BLOCKS << 16;
}

static void uac1_reset_i2s_ctx(struct usb_i2s_ctx *ctx)
{
	ctx->i2s_started = false;
	ctx->i2s_blocks_written = 0;
	ctx->fb_adjust = 0;
	ctx->fb_level_avg_q16 = FEEDBACK_TARGET_BLOCKS << 16;
}

static void uac1_stream_event_cb(const struct device *dev, uint8_t terminal,
				enum uac1_stream_event event, bool microframes,
				void *user_data)
{
	struct usb_i2s_ctx *ctx = user_data;
	bool enabled = event == UAC1_STREAM_ACTIVATED || event == UAC1_STREAM_RESUMED;

	ARG_UNUSED(dev);
	if (terminal != STREAMING_IN_TERMINAL_ID) {
		LOG_ERR("Unexpected terminal %u", terminal);
		return;
	}

	ctx->microframes = microframes;
	ctx->terminal_enabled = enabled;
	LOG_INF("Audio streaming %s at %s speed", enabled ? "enabled" : "disabled",
		microframes ? "high" : "full");
	if (enabled) {
		reset_feedback_controller(ctx);
	}

	if (!enabled && (ctx->i2s_started || ctx->i2s_blocks_written != 0U)) {
		int ret = i2s_trigger(ctx->i2s_dev, I2S_DIR_TX, I2S_TRIGGER_DROP);

		if (ret < 0) {
			LOG_WRN("I2S drop failed on terminal disable: %d", ret);
		}

		uac1_reset_i2s_ctx(ctx);
	}
	if (!enabled) {
		ring_buf_reset(&pcm_fifo);
	}
}

static int uac1_rx_buf_acquire(const struct device *dev, uint8_t terminal,
			       uint16_t size, void **buf, void *user_data)
{
	ARG_UNUSED(dev);
	struct usb_i2s_ctx *ctx = user_data;

	if (terminal != STREAMING_IN_TERMINAL_ID || size > USB_PACKET_MAX_SIZE) {
		return -EINVAL;
	}

	if (!ctx->terminal_enabled) {
		return -EPIPE;
	}

	if (k_mem_slab_alloc(&usb_rx_slab, buf, K_NO_WAIT) != 0) {
		return -EAGAIN;
	}

	return 0;
}

static int queue_i2s_block(struct usb_i2s_ctx *ctx)
{
	void *block;
	int ret;

	if (ring_buf_size_get(&pcm_fifo) < I2S_BLOCK_SIZE) {
		return -EAGAIN;
	}
	if (k_mem_slab_alloc(&i2s_tx_slab, &block, K_NO_WAIT) != 0) {
		return -ENOMEM;
	}
	if (ring_buf_get(&pcm_fifo, block, I2S_BLOCK_SIZE) != I2S_BLOCK_SIZE) {
		k_mem_slab_free(&i2s_tx_slab, block);
		return -EIO;
	}

	ret = i2s_write(ctx->i2s_dev, block, I2S_BLOCK_SIZE);
	if (ret == -EIO) {
		uac1_reset_i2s_ctx(ctx);
		ret = i2s_trigger(ctx->i2s_dev, I2S_DIR_TX, I2S_TRIGGER_PREPARE);
		if (ret == 0) {
			ret = i2s_write(ctx->i2s_dev, block, I2S_BLOCK_SIZE);
		}
	}
	if (ret < 0) {
		LOG_ERR("i2s_write failed: %d", ret);
		k_mem_slab_free(&i2s_tx_slab, block);
		return ret;
	}

	ctx->i2s_blocks_written++;
	return 0;
}

static void uac1_rx_buf_release(const struct device *dev, uint8_t terminal,
			       void *buf, uint16_t size, int status, void *user_data)
{
	struct usb_i2s_ctx *ctx = user_data;
	int ret;

	ARG_UNUSED(dev);

	if (terminal != STREAMING_IN_TERMINAL_ID || status != 0) {
		k_mem_slab_free(&usb_rx_slab, buf);
		return;
	}

	if (!ctx->terminal_enabled) {
		k_mem_slab_free(&usb_rx_slab, buf);
		return;
	}

	if (!size) {
		size = ctx->microframes ? (HS_SAMPLES_PER_XFER * BYTES_PER_SLOT) :
					  (FS_SAMPLES_PER_SOF * BYTES_PER_SLOT);
		memset(buf, 0, size);
	}

	apply_volume(ctx, buf, size);
	if (ring_buf_space_get(&pcm_fifo) < size) {
		LOG_ERR("PCM FIFO overflow");
	} else {
		(void)ring_buf_put(&pcm_fifo, buf, size);
	}
	k_mem_slab_free(&usb_rx_slab, buf);

	while ((ret = queue_i2s_block(ctx)) == 0) {
	}
}

static uint32_t uac1_feedback_cb(const struct device *dev, uint8_t terminal,
				 void *user_data)
{
	ARG_UNUSED(dev);
	ARG_UNUSED(terminal);
	struct usb_i2s_ctx *ctx = user_data;

	if (ctx->microframes) {
		return ctx->fb_nominal_hs;
	}

	return ctx->fb_nominal_fs + ctx->fb_adjust;
}

static int uac1_feature_update_cb(const struct device *dev, uint8_t unit_id,
				  uint8_t control_selector,
				  const struct uac1_feature_update *update, void *user_data)
{
	struct usb_i2s_ctx *ctx = user_data;

	ARG_UNUSED(dev);

	if (unit_id != FEATURE_UNIT_ID || update == NULL ||
	    (update->channel_mask & ~BIT_MASK(NUMBER_OF_CHANNELS + 1U)) != 0U) {
		LOG_WRN("Unsupported Feature Unit update");
		return -ENOTSUP;
	}

	if ((control_selector == UAC1_FU_MUTE_CONTROL && update->value_size != sizeof(uint8_t)) ||
	    (control_selector == UAC1_FU_VOLUME_CONTROL && update->value_size != sizeof(int16_t)) ||
	    (control_selector != UAC1_FU_MUTE_CONTROL &&
	     control_selector != UAC1_FU_VOLUME_CONTROL)) {
		LOG_WRN("Unsupported Feature Unit control %u", control_selector);
		return -ENOTSUP;
	}

	uint8_t offset = 0U;
	for (uint8_t channel = 0U; channel <= NUMBER_OF_CHANNELS; channel++) {
		if ((update->channel_mask & BIT(channel)) == 0U) {
			continue;
		}
		if (control_selector == UAC1_FU_MUTE_CONTROL) {
			ctx->mute[channel] = ((const uint8_t *)update->values)[offset] != 0U;
		} else {
			memcpy(&ctx->volume[channel], (const uint8_t *)update->values + offset,
			       sizeof(ctx->volume[channel]));
		}
		offset += update->value_size;
	}

	update_channel_gains(ctx);
	return 0;
}

static void uac1_sof(const struct device *dev, void *user_data)
{
	ARG_UNUSED(dev);
	struct usb_i2s_ctx *ctx = user_data;

	if (!ctx->terminal_enabled) {
		return;
	}

	if (!ctx->i2s_started && ctx->i2s_blocks_written >= 2) {
		int ret = i2s_trigger(ctx->i2s_dev, I2S_DIR_TX, I2S_TRIGGER_START);

		if (ret == 0) {
			ctx->i2s_started = true;
		} else {
			LOG_ERR("I2S start failed: %d", ret);
		}
	}

	if (ctx->i2s_started && !ctx->microframes) {
		uint32_t used = k_mem_slab_num_used_get(&i2s_tx_slab);
		int32_t level_error_q16;

		ctx->fb_level_avg_q16 =
			(((uint64_t)ctx->fb_level_avg_q16 * (BIT(FEEDBACK_FILTER_SHIFT) - 1U)) +
			 (used << 16)) >>
			FEEDBACK_FILTER_SHIFT;
		level_error_q16 = (FEEDBACK_TARGET_BLOCKS << 16) - (int32_t)ctx->fb_level_avg_q16;
		ctx->fb_adjust = ((int64_t)level_error_q16 * FEEDBACK_ADJUST_PER_BLOCK) >> 16;
		ctx->fb_adjust = CLAMP(ctx->fb_adjust, -FEEDBACK_MAX_ADJUST, FEEDBACK_MAX_ADJUST);
	}
}

static struct uac1_ops usb_audio_ops = {
	.sof_cb = uac1_sof,
	.stream_event_cb = uac1_stream_event_cb,
	.rx_buf_acquire = uac1_rx_buf_acquire,
	.rx_buf_release = uac1_rx_buf_release,
	.feedback_cb = uac1_feedback_cb,
	.feature_update_cb = uac1_feature_update_cb,
};

static struct usb_i2s_ctx main_ctx;

static void sample_msg_cb(struct usbd_context *const ctx, const struct usbd_msg *const msg)
{
	ARG_UNUSED(ctx);

	LOG_INF("USBD message: %s", usbd_msg_type_string(msg->type));
	if (msg->type == USBD_MSG_CONFIGURATION) {
		LOG_INF("Configuration value %d", msg->status);
	}
}

int main(void)
{
	const struct device *uac1_dev = DEVICE_DT_GET(DT_NODELABEL(uac1_headphones));
	struct usbd_context *sample_usbd;
	struct i2s_config config;
	struct i2s_timing timing;
	int ret;

	main_ctx.i2s_dev = DEVICE_DT_GET(DT_NODELABEL(i2s_tx));
	for (uint8_t channel = 0; channel < NUMBER_OF_CHANNELS; channel++) {
		atomic_set(&main_ctx.target_gain[channel], GAIN_Q15_UNITY);
		main_ctx.current_gain[channel] = GAIN_Q15_UNITY;
	}
	if (!device_is_ready(main_ctx.i2s_dev)) {
		printk("%s is not ready\n", main_ctx.i2s_dev->name);
		return 0;
	}

	config.word_size = SAMPLE_BIT_WIDTH;
	config.channels = NUMBER_OF_CHANNELS;
	config.format = SAMPLE_I2S_AUDIO_FORMAT;
	config.options = I2S_OPT_BIT_CLK_CONTROLLER | I2S_OPT_FRAME_CLK_CONTROLLER;
	config.frame_clk_freq = SAMPLE_FREQUENCY;
	config.mem_slab = &i2s_tx_slab;
	config.block_size = I2S_BLOCK_SIZE;
	config.timeout = 0;

	ret = i2s_configure(main_ctx.i2s_dev, I2S_DIR_TX, &config);
	if (ret < 0) {
		printk("Failed to configure TX stream: %d\n", ret);
		return 0;
	}

	ret = i2s_timing_get(main_ctx.i2s_dev, I2S_DIR_TX, &timing);
	if (ret < 0) {
		timing.frame_rate_num = SAMPLE_FREQUENCY;
		timing.frame_rate_den = 1U;
	}

	main_ctx.fb_nominal_fs =
		DIV_ROUND_CLOSEST(timing.frame_rate_num << 14, timing.frame_rate_den * 1000U);
	main_ctx.fb_nominal_hs =
		DIV_ROUND_CLOSEST(timing.frame_rate_num << 16, timing.frame_rate_den * 8000U);
	LOG_INF("Feedback baseline uses I2S frame clock %llu/%llu Hz", timing.frame_rate_num,
		timing.frame_rate_den);

	ret = usbd_uac1_set_ops(uac1_dev, &usb_audio_ops, &main_ctx);
	if (ret < 0) {
		return ret;
	}

	sample_usbd = sample_usbd_init_device(sample_msg_cb);
	if (sample_usbd == NULL) {
		return -ENODEV;
	}

	ret = usbd_enable(sample_usbd);
	if (ret) {
		return ret;
	}

	return 0;
}
