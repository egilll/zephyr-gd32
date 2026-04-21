/*
 * Copyright (c) 2026, The Zephyr Project Contributors
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdint.h>
#include <string.h>

#include <sample_usbd.h>

#include <zephyr/device.h>
#include <zephyr/kernel.h>
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
#define MIN_BLOCK_SIZE      ((MAX_SAMPLES_PER_XFER - 1) * BYTES_PER_SLOT)
#define BLOCK_SIZE          (MAX_SAMPLES_PER_XFER * BYTES_PER_SLOT)
#define MAX_BLOCK_SIZE      ((MAX_SAMPLES_PER_XFER + 1) * BYTES_PER_SLOT)

#define I2S_BUFFERS_COUNT   7
K_MEM_SLAB_DEFINE_STATIC(i2s_tx_slab, ROUND_UP(MAX_BLOCK_SIZE, UDC_BUF_GRANULARITY),
			 I2S_BUFFERS_COUNT, UDC_BUF_ALIGN);

struct usb_i2s_ctx {
	const struct device *i2s_dev;
	bool terminal_enabled;
	bool i2s_started;
	bool microframes;
	uint8_t i2s_blocks_written;
	int32_t fb_adjust;
};

static void uac1_reset_i2s_ctx(struct usb_i2s_ctx *ctx)
{
	ctx->i2s_started = false;
	ctx->i2s_blocks_written = 0;
	ctx->fb_adjust = 0;
}

static void uac1_terminal_update_cb(const struct device *dev, uint8_t terminal,
				    bool enabled, bool microframes,
				    void *user_data)
{
	struct usb_i2s_ctx *ctx = user_data;

	ARG_UNUSED(dev);
	__ASSERT_NO_MSG(terminal == STREAMING_IN_TERMINAL_ID);

	ctx->microframes = microframes;
	ctx->terminal_enabled = enabled;

	if (!enabled && (ctx->i2s_started || ctx->i2s_blocks_written != 0U)) {
		int ret = i2s_trigger(ctx->i2s_dev, I2S_DIR_TX, I2S_TRIGGER_DROP);

		if (ret < 0) {
			LOG_WRN("I2S drop failed on terminal disable: %d", ret);
		}

		uac1_reset_i2s_ctx(ctx);
	}
}

static void *uac1_get_recv_buf(const struct device *dev, uint8_t terminal,
			       uint16_t size, void *user_data)
{
	ARG_UNUSED(dev);
	struct usb_i2s_ctx *ctx = user_data;
	void *buf = NULL;

	if (terminal != STREAMING_IN_TERMINAL_ID) {
		return NULL;
	}

	__ASSERT_NO_MSG(size <= MAX_BLOCK_SIZE);

	if (!ctx->terminal_enabled) {
		LOG_ERR("Buffer request on disabled terminal");
		return NULL;
	}

	if (k_mem_slab_alloc(&i2s_tx_slab, &buf, K_NO_WAIT) != 0) {
		return NULL;
	}

	return buf;
}

static void uac1_data_recv_cb(const struct device *dev, uint8_t terminal,
			      void *buf, uint16_t size, void *user_data)
{
	struct usb_i2s_ctx *ctx = user_data;
	int ret;

	ARG_UNUSED(dev);

	if (terminal != STREAMING_IN_TERMINAL_ID) {
		k_mem_slab_free(&i2s_tx_slab, buf);
		return;
	}

	if (!ctx->terminal_enabled) {
		k_mem_slab_free(&i2s_tx_slab, buf);
		return;
	}

	if (!size) {
		size = ctx->microframes ? (HS_SAMPLES_PER_XFER * BYTES_PER_SLOT) :
					  (FS_SAMPLES_PER_SOF * BYTES_PER_SLOT);
		memset(buf, 0, size);
	}

	ret = i2s_write(ctx->i2s_dev, buf, size);
	if (ret < 0) {
		LOG_ERR("i2s_write failed: %d", ret);
		uac1_reset_i2s_ctx(ctx);

		if (ret == -EIO) {
			ret = i2s_trigger(ctx->i2s_dev, I2S_DIR_TX, I2S_TRIGGER_PREPARE);
			if (ret < 0) {
				LOG_ERR("I2S prepare failed: %d", ret);
				k_mem_slab_free(&i2s_tx_slab, buf);
				return;
			}

			ret = i2s_write(ctx->i2s_dev, buf, size);
			if (ret < 0) {
				LOG_ERR("i2s_write retry failed: %d", ret);
				k_mem_slab_free(&i2s_tx_slab, buf);
				return;
			}
		} else {
			k_mem_slab_free(&i2s_tx_slab, buf);
			return;
		}
	}

	if (ret == 0) {
		ctx->i2s_blocks_written++;
	}
}

static void uac1_buf_release_cb(const struct device *dev, uint8_t terminal,
				void *buf, void *user_data)
{
	ARG_UNUSED(dev);
	ARG_UNUSED(terminal);
	ARG_UNUSED(buf);
	ARG_UNUSED(user_data);
}

static uint32_t uac1_feedback_cb(const struct device *dev, uint8_t terminal,
				 void *user_data)
{
	ARG_UNUSED(dev);
	ARG_UNUSED(terminal);
	struct usb_i2s_ctx *ctx = user_data;

	if (ctx->microframes) {
		return (HS_SAMPLES_PER_SOF << 16);
	}

	return (FS_SAMPLES_PER_SOF << 14) + ctx->fb_adjust;
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
		const int32_t target_used = 4;
		int32_t used = (int32_t)k_mem_slab_num_used_get(&i2s_tx_slab);
		int32_t err = used - target_used;

		ctx->fb_adjust -= CLAMP(err * 4, -64, 64);
		ctx->fb_adjust = CLAMP(ctx->fb_adjust, -(1 << 9), (1 << 9));
	}
}

static struct uac1_ops usb_audio_ops = {
	.sof_cb = uac1_sof,
	.terminal_update_cb = uac1_terminal_update_cb,
	.get_recv_buf = uac1_get_recv_buf,
	.data_recv_cb = uac1_data_recv_cb,
	.buf_release_cb = uac1_buf_release_cb,
	.feedback_cb = uac1_feedback_cb,
};

static struct usb_i2s_ctx main_ctx;

int main(void)
{
	const struct device *uac1_dev = DEVICE_DT_GET(DT_NODELABEL(uac1_headphones));
	struct usbd_context *sample_usbd;
	struct i2s_config config;
	int ret;

	main_ctx.i2s_dev = DEVICE_DT_GET(DT_NODELABEL(i2s_tx));

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
	config.block_size = MAX_BLOCK_SIZE;
	config.timeout = 0;

	ret = i2s_configure(main_ctx.i2s_dev, I2S_DIR_TX, &config);
	if (ret < 0) {
		printk("Failed to configure TX stream: %d\n", ret);
		return 0;
	}

	usbd_uac1_set_ops(uac1_dev, &usb_audio_ops, &main_ctx);

	sample_usbd = sample_usbd_init_device(NULL);
	if (sample_usbd == NULL) {
		return -ENODEV;
	}

	ret = usbd_enable(sample_usbd);
	if (ret) {
		return ret;
	}

	return 0;
}
