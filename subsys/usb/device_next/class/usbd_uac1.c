/*
 * Copyright (c) 2026 Ylhyra ehf.
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stddef.h>
#include <stdint.h>

#include <zephyr/kernel.h>
#include <zephyr/sys/__assert.h>
#include <zephyr/sys/atomic.h>
#include <zephyr/sys/byteorder.h>

#include <zephyr/usb/usbd.h>
#include <zephyr/usb/usb_ch9.h>
#include <zephyr/usb/class/usbd_uac1.h>
#include <zephyr/drivers/usb/udc.h>

#include "usbd_uac1_macros.h"

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(usbd_uac1, CONFIG_USBD_UAC1_LOG_LEVEL);

#define DT_DRV_COMPAT zephyr_uac1

#define COUNT_UAC1_AS_ENDPOINT_BUFFERS(node)					\
	IF_ENABLED(DT_NODE_HAS_COMPAT(node, zephyr_uac1_audio_streaming), (	\
		+ AS_HAS_ISOCHRONOUS_DATA_ENDPOINT(node)			\
		+ AS_IS_USB_ISO_IN(node) /* ISO IN double buffering */		\
		+ AS_IS_USB_ISO_OUT(node) /* ISO OUT double buffering */	\
		+ 2 * AS_HAS_EXPLICIT_FEEDBACK_ENDPOINT(node)))
#define COUNT_UAC1_EP_BUFFERS(i)						\
	+ DT_PROP(DT_DRV_INST(i), interrupt_endpoint)				\
	DT_INST_FOREACH_CHILD(i, COUNT_UAC1_AS_ENDPOINT_BUFFERS)
#define UAC1_NUM_EP_BUFFERS DT_INST_FOREACH_STATUS_OKAY(COUNT_UAC1_EP_BUFFERS)

struct uac1_buf_info {
	struct udc_buf_info udc;
	uint32_t generation;
	uint8_t as_idx;
	bool is_feedback;
};

UDC_BUF_POOL_DEFINE(uac1_pool, UAC1_NUM_EP_BUFFERS, 6,
		    sizeof(struct uac1_buf_info), NULL);

/* Audio class specific request types */
#define SET_CLASS_INTERFACE_REQUEST_TYPE	0x21
#define GET_CLASS_INTERFACE_REQUEST_TYPE	0xA1
#define SET_CLASS_ENDPOINT_REQUEST_TYPE		0x22
#define GET_CLASS_ENDPOINT_REQUEST_TYPE		0xA2

/* Audio Class-Specific Request Codes (Audio 1.0, Table A-9) */
#define SET_CUR				0x01
#define GET_CUR				0x81
#define SET_MIN				0x02
#define GET_MIN				0x82
#define SET_MAX				0x03
#define GET_MAX				0x83
#define SET_RES				0x04
#define GET_RES				0x84
#define GET_STAT			0xFF

/* A.12.2 Feature Unit Descriptor bmaControls bits: Mute (D0), Volume (D1) */
#define FU_CTRL_MUTE_BIT		BIT(0)
#define FU_CTRL_VOLUME_BIT		BIT(1)

/* Endpoint control selectors (Audio 1.0, Table 4-21) */
#define EP_SAMPLING_FREQ_CONTROL	0x01
#define EP_PITCH_CONTROL		0x02

#define CONTROL_ATTRIBUTE(setup)	(setup->bRequest)
#define CONTROL_ENTITY_ID(setup)	((setup->wIndex & 0xFF00) >> 8)
#define CONTROL_SELECTOR(setup)		((setup->wValue & 0xFF00) >> 8)
#define CONTROL_CHANNEL_NUMBER(setup)	((setup->wValue & 0x00FF))

struct uac1_fu_state {
	uint8_t mute;
	int16_t volume;
};

struct uac1_fu_cfg {
	uint8_t unit_id;
	uint8_t num_channels;
	const uint8_t *controls;
	int16_t vol_min;
	int16_t vol_max;
	int16_t vol_res;
	struct uac1_fu_state *state;
};

struct uac1_stream_runtime {
	atomic_t data_outstanding;
	atomic_t feedback_outstanding;
	atomic_t generation;
	uint32_t service_number;
};

struct uac1_ctx {
	const struct uac1_ops *ops;
	void *user_data;

	atomic_t as_selected;
	atomic_t suspended;
	atomic_t status_queued;
};

struct uac1_cfg {
	struct usbd_class_data *const c_data;
	const struct usb_desc_header **fs_descriptors;
	const struct usb_desc_header **hs_descriptors;

	const uint8_t *as_terminals;

	const uint8_t *fs_data_ep;
	const uint16_t *fs_data_mps;
	const uint8_t *fs_fb_ep;

	const uint8_t *hs_data_ep;
	const uint16_t *hs_data_mps;
	const uint8_t *hs_fb_ep;

	const uint32_t *const *as_frequencies;
	const uint8_t *as_frequencies_count;

	const struct uac1_fu_cfg *fu_cfg;
	struct uac1_stream_runtime *streams;
	uint8_t status_ep;
	uint8_t num_ifaces;
	uint8_t num_fu;
};

static bool stream_available(const struct uac1_ctx *ctx, uint8_t as_idx)
{
	return !atomic_get(&ctx->suspended) && atomic_test_bit(&ctx->as_selected, as_idx);
}

static bool outstanding_acquire(atomic_t *outstanding)
{
	atomic_val_t value = atomic_get(outstanding);

	while (value < 2) {
		if (atomic_cas(outstanding, value, value + 1)) {
			return true;
		}
		value = atomic_get(outstanding);
	}

	return false;
}

static void outstanding_release(atomic_t *outstanding)
{
	atomic_val_t value = atomic_get(outstanding);

	while (value > 0) {
		if (atomic_cas(outstanding, value, value - 1)) {
			return;
		}
		value = atomic_get(outstanding);
	}

	LOG_ERR("Completion without an outstanding request");
}

static const struct uac1_fu_cfg *find_fu_cfg(const struct uac1_cfg *cfg, uint8_t unit_id)
{
	for (uint8_t i = 0; i < cfg->num_fu; i++) {
		if (cfg->fu_cfg[i].unit_id == unit_id) {
			return &cfg->fu_cfg[i];
		}
	}

	return NULL;
}

static struct uac1_fu_state *find_fu_state(const struct uac1_fu_cfg *fu_cfg, uint8_t channel)
{
	if (channel > fu_cfg->num_channels) {
		return NULL;
	}

	return &fu_cfg->state[channel];
}

static uint8_t uac1_ac_interface(const struct uac1_cfg *cfg)
{
	const struct usb_association_descriptor *iad;
	const struct usb_desc_header **descriptors;

	descriptors = usbd_bus_speed(cfg->c_data->uds_ctx) == USBD_SPEED_FS ?
		cfg->fs_descriptors : cfg->hs_descriptors;
	__ASSERT_NO_MSG(descriptors != NULL);
	iad = (const struct usb_association_descriptor *)descriptors[0];

	return iad->bFirstInterface;
}

static int ep_to_as_interface(const struct device *dev, uint8_t ep, bool *is_feedback)
{
	const struct uac1_cfg *cfg = dev->config;
	const struct usbd_class_data *c_data = cfg->c_data;
	const uint8_t *data_eps;
	const uint8_t *fb_eps;

	if (usbd_bus_speed(c_data->uds_ctx) == USBD_SPEED_FS) {
		data_eps = cfg->fs_data_ep;
		fb_eps = cfg->fs_fb_ep;
	} else {
		data_eps = cfg->hs_data_ep;
		fb_eps = cfg->hs_fb_ep;
	}

	if (data_eps == NULL) {
		*is_feedback = false;
		return -ENOENT;
	}

	for (int i = 0; i < cfg->num_ifaces; i++) {
		if (data_eps[i] && data_eps[i] == ep) {
			*is_feedback = false;
			return i;
		}

		if (fb_eps != NULL && fb_eps[i] != 0U && fb_eps[i] == ep) {
			*is_feedback = true;
			return i;
		}
	}

	*is_feedback = false;
	return -ENOENT;
}

static uint16_t as_data_mps(const struct device *dev, int as_idx)
{
	const struct uac1_cfg *cfg = dev->config;
	const struct usbd_class_data *c_data = cfg->c_data;

	if (usbd_bus_speed(c_data->uds_ctx) == USBD_SPEED_FS) {
		return cfg->fs_data_mps ? cfg->fs_data_mps[as_idx] : 0U;
	}

	return cfg->hs_data_mps ? cfg->hs_data_mps[as_idx] : 0U;
}

static uint8_t as_data_ep(const struct device *dev, int as_idx)
{
	const struct uac1_cfg *cfg = dev->config;
	const struct usbd_class_data *c_data = cfg->c_data;

	if (usbd_bus_speed(c_data->uds_ctx) == USBD_SPEED_FS) {
		return cfg->fs_data_ep ? cfg->fs_data_ep[as_idx] : 0U;
	}

	return cfg->hs_data_ep ? cfg->hs_data_ep[as_idx] : 0U;
}

static uint8_t as_feedback_ep(const struct device *dev, int as_idx)
{
	const struct uac1_cfg *cfg = dev->config;
	const struct usbd_class_data *c_data = cfg->c_data;

	if (usbd_bus_speed(c_data->uds_ctx) == USBD_SPEED_FS) {
		return cfg->fs_fb_ep ? cfg->fs_fb_ep[as_idx] : 0U;
	}

	return cfg->hs_fb_ep ? cfg->hs_fb_ep[as_idx] : 0U;
}

static struct net_buf *uac1_buf_alloc(const uint8_t ep, void *data, uint16_t size,
				     uint8_t as_idx, uint32_t generation)
{
	struct net_buf *buf;
	struct uac1_buf_info *info;

	if (data == NULL || !IS_UDC_ALIGNED(data)) {
		return NULL;
	}

	buf = net_buf_alloc_with_data(&uac1_pool, data, size, K_NO_WAIT);
	if (!buf) {
		return NULL;
	}

	info = net_buf_user_data(buf);
	info->udc.ep = ep;
	info->generation = generation;
	info->as_idx = as_idx;
	info->is_feedback = false;

	if (USB_EP_DIR_IS_OUT(ep)) {
		buf->len = 0;
	}

	return buf;
}

static uint32_t find_closest(uint32_t input, const uint32_t *values, size_t values_count)
{
	size_t i;

	__ASSERT_NO_MSG(values_count);

	for (i = 0; i < values_count; i++) {
		if (input == values[i]) {
			return input;
		} else if (input < values[i]) {
			break;
		}
	}

	if (i == values_count) {
		return values[i - 1];
	}

	if (i == 0) {
		return values[i];
	}

	if ((values[i] - input) > (input - values[i - 1])) {
		return values[i - 1];
	}

	return values[i];
}

static void release_rx_buffer(const struct device *dev, uint8_t terminal, void *data,
			      uint16_t size, int status)
{
	struct uac1_ctx *ctx = dev->data;

	ctx->ops->rx_buf_release(dev, terminal, data, size, status, ctx->user_data);
}

static void release_tx_buffer(const struct device *dev, uint8_t terminal, void *data, int status)
{
	struct uac1_ctx *ctx = dev->data;

	ctx->ops->tx_buf_release(dev, terminal, data, status, ctx->user_data);
}

static void schedule_iso_out_read(struct usbd_class_data *const c_data, uint8_t as_idx)
{
	const struct device *dev = usbd_class_get_private(c_data);
	const struct uac1_cfg *cfg = dev->config;
	struct uac1_ctx *ctx = dev->data;
	struct uac1_stream_runtime *stream = &cfg->streams[as_idx];
	struct net_buf *buf;
	const uint8_t terminal = cfg->as_terminals[as_idx];
	const uint8_t ep = as_data_ep(dev, as_idx);
	const uint16_t mps = as_data_mps(dev, as_idx);
	void *data_buf = NULL;
	uint32_t generation;
	int ret;

	if (!stream_available(ctx, as_idx) || !outstanding_acquire(&stream->data_outstanding)) {
		return;
	}
	generation = atomic_get(&stream->generation);

	ret = ctx->ops->rx_buf_acquire(dev, terminal, mps, &data_buf, ctx->user_data);
	if (ret != 0 || data_buf == NULL) {
		if (ret != -EAGAIN) {
			LOG_ERR_RATELIMIT("No receive buffer for terminal %u: %d", terminal,
					  ret != 0 ? ret : -EINVAL);
		}
		outstanding_release(&stream->data_outstanding);
		return;
	}

	buf = uac1_buf_alloc(ep, data_buf, mps, as_idx, generation);
	if (!buf) {
		LOG_ERR("Invalid or unavailable receive buffer for terminal %u", terminal);
		release_rx_buffer(dev, terminal, data_buf, 0, -ENOMEM);
		outstanding_release(&stream->data_outstanding);
		return;
	}

	ret = usbd_ep_enqueue(c_data, buf);
	if (ret) {
		LOG_ERR("Failed to enqueue net_buf for 0x%02x", ep);
		net_buf_unref(buf);
		release_rx_buffer(dev, terminal, data_buf, 0, ret);
		outstanding_release(&stream->data_outstanding);
	}
}

static void schedule_iso_in_write(struct usbd_class_data *const c_data, uint8_t as_idx)
{
	const struct device *dev = usbd_class_get_private(c_data);
	const struct uac1_cfg *cfg = dev->config;
	struct uac1_ctx *ctx = dev->data;
	struct uac1_stream_runtime *stream = &cfg->streams[as_idx];
	const uint8_t terminal = cfg->as_terminals[as_idx];
	const uint8_t ep = as_data_ep(dev, as_idx);
	const uint16_t mps = as_data_mps(dev, as_idx);
	struct net_buf *buf;
	void *data = NULL;
	uint16_t size = 0U;
	uint32_t generation;
	int ret;

	if (!stream_available(ctx, as_idx) || !outstanding_acquire(&stream->data_outstanding)) {
		return;
	}
	generation = atomic_get(&stream->generation);

	const uint32_t service_number = stream->service_number++;

	ret = ctx->ops->tx_buf_acquire(dev, terminal, mps, service_number,
				      &data, &size, ctx->user_data);
	if (ret != 0) {
		if (ret != -EAGAIN) {
			LOG_ERR_RATELIMIT("No transmit buffer for terminal %u: %d", terminal, ret);
		}
		outstanding_release(&stream->data_outstanding);
		return;
	}

	if (data == NULL || size > mps || !IS_UDC_ALIGNED(data)) {
		LOG_ERR("Invalid transmit buffer for terminal %u", terminal);
		release_tx_buffer(dev, terminal, data, -EINVAL);
		outstanding_release(&stream->data_outstanding);
		return;
	}

	buf = uac1_buf_alloc(ep, data, size, as_idx, generation);
	if (buf == NULL) {
		release_tx_buffer(dev, terminal, data, -ENOMEM);
		outstanding_release(&stream->data_outstanding);
		return;
	}

	ret = usbd_ep_enqueue(c_data, buf);
	if (ret != 0) {
		net_buf_unref(buf);
		release_tx_buffer(dev, terminal, data, ret);
		outstanding_release(&stream->data_outstanding);
		return;
	}

}

static void write_explicit_feedback(struct usbd_class_data *const c_data, uint8_t as_idx)
{
	const struct device *dev = usbd_class_get_private(c_data);
	const struct uac1_cfg *cfg = dev->config;
	struct usbd_context *uds_ctx = usbd_class_get_ctx(c_data);
	struct uac1_ctx *ctx = dev->data;
	struct uac1_stream_runtime *stream = &cfg->streams[as_idx];
	const uint8_t terminal = cfg->as_terminals[as_idx];
	const uint8_t ep = as_feedback_ep(dev, as_idx);
	struct net_buf *buf;
	struct udc_buf_info *bi;
	struct uac1_buf_info *info;
	uint32_t generation;
	uint32_t fb_value;
	int ret;

	if (!stream_available(ctx, as_idx) ||
	    !outstanding_acquire(&stream->feedback_outstanding)) {
		return;
	}
	generation = atomic_get(&stream->generation);

	buf = net_buf_alloc(&uac1_pool, K_NO_WAIT);
	if (!buf) {
		LOG_ERR("No buf for feedback");
		outstanding_release(&stream->feedback_outstanding);
		return;
	}

	bi = udc_get_buf_info(buf);
	bi->ep = ep;
	info = net_buf_user_data(buf);
	info->generation = generation;
	info->as_idx = as_idx;
	info->is_feedback = true;

	fb_value = ctx->ops->feedback_cb(dev, terminal, ctx->user_data);

	if (usbd_bus_speed(uds_ctx) == USBD_SPEED_FS) {
		net_buf_add_le24(buf, fb_value);
	} else {
		net_buf_add_le32(buf, fb_value);
	}

	ret = usbd_ep_enqueue(c_data, buf);
	if (ret) {
		LOG_ERR("Failed to enqueue net_buf for 0x%02x", ep);
		net_buf_unref(buf);
		outstanding_release(&stream->feedback_outstanding);
		return;
	}
}

int usbd_uac1_set_ops(const struct device *dev, const struct uac1_ops *ops, void *user_data)
{
	const struct uac1_cfg *cfg = dev->config;
	struct uac1_ctx *ctx = dev->data;

	if (ops == NULL || ops->stream_event_cb == NULL) {
		return -EINVAL;
	}

	for (uint8_t i = 0U; i < cfg->num_ifaces; i++) {
		const uint8_t ep_fs = cfg->fs_data_ep ? cfg->fs_data_ep[i] : 0U;
		const uint8_t ep_hs = cfg->hs_data_ep ? cfg->hs_data_ep[i] : 0U;
		const uint8_t ep = ep_fs ? ep_fs : ep_hs;

		if (cfg->fs_fb_ep && cfg->fs_fb_ep[i]) {
			if (ops->feedback_cb == NULL) {
				return -EINVAL;
			}
		}
		if (cfg->hs_fb_ep && cfg->hs_fb_ep[i]) {
			if (ops->feedback_cb == NULL) {
				return -EINVAL;
			}
		}

		if (ep) {
			if (USB_EP_DIR_IS_OUT(ep)) {
				if (ops->rx_buf_acquire == NULL || ops->rx_buf_release == NULL) {
					return -EINVAL;
				}
			}
			if (USB_EP_DIR_IS_IN(ep)) {
				if (ops->tx_buf_acquire == NULL || ops->tx_buf_release == NULL) {
					return -EINVAL;
				}
			}
		}
	}

	ctx->ops = ops;
	ctx->user_data = user_data;
	return 0;
}

int usbd_uac1_status_notify(const struct device *dev, enum uac1_status_origin origin,
			    uint8_t originator, bool memory_changed)
{
	const struct uac1_cfg *cfg = dev->config;
	struct uac1_ctx *ctx = dev->data;
	struct udc_buf_info *bi;
	struct net_buf *buf;
	int ret;

	if (cfg->status_ep == 0U) {
		return -ENOTSUP;
	}

	if (origin > UAC1_STATUS_AUDIO_STREAMING_ENDPOINT) {
		return -EINVAL;
	}

	if (!atomic_cas(&ctx->status_queued, 0, 1)) {
		return -EAGAIN;
	}

	buf = net_buf_alloc(&uac1_pool, K_NO_WAIT);
	if (buf == NULL) {
		atomic_clear(&ctx->status_queued);
		return -ENOMEM;
	}

	bi = udc_get_buf_info(buf);
	bi->ep = cfg->status_ep;
	net_buf_add_u8(buf, (memory_changed ? BIT(6) : 0U) | origin);
	net_buf_add_u8(buf, originator);

	ret = usbd_ep_enqueue(cfg->c_data, buf);
	if (ret != 0) {
		net_buf_unref(buf);
		atomic_clear(&ctx->status_queued);
	}

	return ret;
}

static int feature_value_normalize(const struct uac1_fu_cfg *fu_cfg, uint8_t cs,
				   uint8_t channel, const void *value, uint8_t value_len,
				   uint8_t normalized[sizeof(int16_t)])
{
	struct uac1_fu_state *fu_state = find_fu_state(fu_cfg, channel);

	if (fu_state == NULL) {
		return -EINVAL;
	}

	if (cs == UAC1_FU_MUTE_CONTROL) {
		uint8_t mute;

		if (!(fu_cfg->controls[channel] & FU_CTRL_MUTE_BIT)) {
			return -ENOTSUP;
		}
		if (value_len != sizeof(mute)) {
			return -EINVAL;
		}

		mute = *(const uint8_t *)value != 0U;
		normalized[0] = mute;
		return 0;
	}

	if (cs == UAC1_FU_VOLUME_CONTROL) {
		int16_t volume;
		int32_t offset;

		if (!(fu_cfg->controls[channel] & FU_CTRL_VOLUME_BIT)) {
			return -ENOTSUP;
		}
		if (value_len != sizeof(volume)) {
			return -EINVAL;
		}

		volume = (int16_t)sys_get_le16(value);
		if (volume != INT16_MIN) {
			volume = CLAMP(volume, fu_cfg->vol_min, fu_cfg->vol_max);
			offset = volume - fu_cfg->vol_min;
			volume = fu_cfg->vol_min +
				 DIV_ROUND_CLOSEST(offset, fu_cfg->vol_res) * fu_cfg->vol_res;
			volume = MIN(volume, fu_cfg->vol_max);
		}

		sys_put_le16((uint16_t)volume, normalized);
		return 0;
	}

	return -ENOTSUP;
}

static void feature_value_commit(const struct uac1_fu_cfg *fu_cfg, uint8_t cs, uint8_t channel,
				 const uint8_t value[sizeof(int16_t)])
{
	struct uac1_fu_state *fu_state = find_fu_state(fu_cfg, channel);

	if (cs == UAC1_FU_MUTE_CONTROL) {
		fu_state->mute = value[0];
	} else {
		fu_state->volume = (int16_t)sys_get_le16(value);
	}
}

int usbd_uac1_feature_set(const struct device *dev, uint8_t unit_id,
			  uint8_t control_selector, uint8_t channel, const void *value,
			  uint8_t value_len)
{
	const struct uac1_cfg *cfg = dev->config;
	const struct uac1_fu_cfg *fu_cfg = find_fu_cfg(cfg, unit_id);
	uint8_t normalized[sizeof(int16_t)];
	int ret;

	if (fu_cfg == NULL) {
		return -ENOENT;
	}

	ret = feature_value_normalize(fu_cfg, control_selector, channel, value, value_len,
				      normalized);
	if (ret != 0) {
		return ret;
	}
	feature_value_commit(fu_cfg, control_selector, channel, normalized);

	ret = usbd_uac1_status_notify(dev, UAC1_STATUS_AUDIO_CONTROL_INTERFACE, unit_id, false);
	return ret == -ENOTSUP || ret == -EAGAIN ? 0 : ret;
}

static uint8_t feature_control_count(const struct uac1_fu_cfg *fu_cfg, uint8_t control_bit)
{
	uint8_t count = 0U;

	for (uint8_t channel = 0U; channel <= fu_cfg->num_channels; channel++) {
		count += (fu_cfg->controls[channel] & control_bit) != 0U;
	}

	return count;
}

static int handle_fu_get(const struct device *dev, const struct usb_setup_packet *setup,
			 struct net_buf *buf)
{
	const struct uac1_cfg *cfg = dev->config;
	const uint8_t unit_id = CONTROL_ENTITY_ID(setup);
	const uint8_t channel = CONTROL_CHANNEL_NUMBER(setup);
	const struct uac1_fu_cfg *fu_cfg;
	struct uac1_fu_state *fu_state = NULL;
	const uint8_t cs = CONTROL_SELECTOR(setup);
	uint8_t control_bit;
	uint8_t value_size;

	fu_cfg = find_fu_cfg(cfg, unit_id);
	if (!fu_cfg) {
		errno = -ENOTSUP;
		return 0;
	}

	if (channel != UINT8_MAX) {
		fu_state = find_fu_state(fu_cfg, channel);
		if (!fu_state) {
			errno = -EINVAL;
			return 0;
		}
	}

	if (cs == UAC1_FU_MUTE_CONTROL) {
		if (CONTROL_ATTRIBUTE(setup) != GET_CUR) {
			errno = -ENOTSUP;
			return 0;
		}
		control_bit = FU_CTRL_MUTE_BIT;
		value_size = sizeof(uint8_t);
	} else if (cs == UAC1_FU_VOLUME_CONTROL) {
		control_bit = FU_CTRL_VOLUME_BIT;
		value_size = sizeof(int16_t);
	} else {
		errno = -ENOTSUP;
		return 0;
	}

	if (channel != UINT8_MAX) {
		if (!(fu_cfg->controls[channel] & control_bit)) {
			errno = -ENOTSUP;
			return 0;
		}

		if (cs == UAC1_FU_MUTE_CONTROL) {
			net_buf_add_u8(buf, fu_state->mute ? 1 : 0);
			return 0;
		}

		if (!(fu_cfg->controls[channel] & FU_CTRL_VOLUME_BIT)) {
			errno = -ENOTSUP;
			return 0;
		}

		if (CONTROL_ATTRIBUTE(setup) == GET_CUR) {
			net_buf_add_le16(buf, fu_state->volume);
			return 0;
		}

		if (CONTROL_ATTRIBUTE(setup) == GET_MIN) {
			net_buf_add_le16(buf, fu_cfg->vol_min);
			return 0;
		}

		if (CONTROL_ATTRIBUTE(setup) == GET_MAX) {
			net_buf_add_le16(buf, fu_cfg->vol_max);
			return 0;
		}

		if (CONTROL_ATTRIBUTE(setup) == GET_RES) {
			net_buf_add_le16(buf, fu_cfg->vol_res);
			return 0;
		}
	}

	if (feature_control_count(fu_cfg, control_bit) == 0U) {
		errno = -ENOTSUP;
		return 0;
	}

	for (uint8_t i = 0U; i <= fu_cfg->num_channels; i++) {
		if (!(fu_cfg->controls[i] & control_bit)) {
			continue;
		}

		if (value_size == sizeof(uint8_t)) {
			net_buf_add_u8(buf, fu_cfg->state[i].mute ? 1 : 0);
		} else if (CONTROL_ATTRIBUTE(setup) == GET_CUR) {
			net_buf_add_le16(buf, fu_cfg->state[i].volume);
		} else if (CONTROL_ATTRIBUTE(setup) == GET_MIN) {
			net_buf_add_le16(buf, fu_cfg->vol_min);
		} else if (CONTROL_ATTRIBUTE(setup) == GET_MAX) {
			net_buf_add_le16(buf, fu_cfg->vol_max);
		} else if (CONTROL_ATTRIBUTE(setup) == GET_RES) {
			net_buf_add_le16(buf, fu_cfg->vol_res);
		} else {
			errno = -ENOTSUP;
			return 0;
		}
	}

	return 0;
}

static int handle_fu_set(const struct device *dev, const struct usb_setup_packet *setup,
			 const struct net_buf *buf)
{
	const struct uac1_cfg *cfg = dev->config;
	struct uac1_ctx *ctx = dev->data;
	const uint8_t unit_id = CONTROL_ENTITY_ID(setup);
	const struct uac1_fu_cfg *fu_cfg;
	const uint8_t cs = CONTROL_SELECTOR(setup);
	const uint8_t channel = CONTROL_CHANNEL_NUMBER(setup);
	uint8_t control_bit;
	uint8_t value_size;
	uint8_t normalized[32U * sizeof(int16_t)];
	struct uac1_feature_update update = {
		.values = normalized,
	};
	uint8_t input_offset = 0U;
	uint8_t output_offset = 0U;
	int ret;

	fu_cfg = find_fu_cfg(cfg, unit_id);
	if (!fu_cfg) {
		errno = -ENOTSUP;
		return 0;
	}

	if (cs == UAC1_FU_MUTE_CONTROL) {
		control_bit = FU_CTRL_MUTE_BIT;
		value_size = sizeof(uint8_t);
	} else if (cs == UAC1_FU_VOLUME_CONTROL) {
		control_bit = FU_CTRL_VOLUME_BIT;
		value_size = sizeof(int16_t);
	} else {
		errno = -ENOTSUP;
		return 0;
	}

	if (CONTROL_ATTRIBUTE(setup) != SET_CUR) {
		errno = -ENOTSUP;
		return 0;
	}

	if (channel != UINT8_MAX) {
		ret = feature_value_normalize(fu_cfg, cs, channel, buf->data, buf->len,
					      normalized);
		if (ret == 0) {
			update.channel_mask = BIT(channel);
			update.value_size = value_size;
			if (ctx->ops->feature_update_cb != NULL) {
				ret = ctx->ops->feature_update_cb(dev, unit_id, cs, &update,
							  ctx->user_data);
			}
		}
		if (ret == 0) {
			feature_value_commit(fu_cfg, cs, channel, normalized);
		}
		errno = ret;
		return 0;
	}

	if (buf->len != feature_control_count(fu_cfg, control_bit) * value_size) {
		errno = -EINVAL;
		return 0;
	}

	for (uint8_t i = 0U; i <= fu_cfg->num_channels; i++) {
		if (!(fu_cfg->controls[i] & control_bit)) {
			continue;
		}

		ret = feature_value_normalize(fu_cfg, cs, i, &buf->data[input_offset],
					      value_size, &normalized[output_offset]);
		if (ret != 0) {
			errno = ret;
			return 0;
		}
		update.channel_mask |= BIT(i);
		input_offset += value_size;
		output_offset += value_size;
	}

	update.value_size = value_size;
	if (ctx->ops->feature_update_cb != NULL) {
		ret = ctx->ops->feature_update_cb(dev, unit_id, cs, &update, ctx->user_data);
		if (ret != 0) {
			errno = ret;
			return 0;
		}
	}

	output_offset = 0U;
	for (uint8_t i = 0U; i <= fu_cfg->num_channels; i++) {
		if ((update.channel_mask & BIT(i)) == 0U) {
			continue;
		}
		feature_value_commit(fu_cfg, cs, i, &normalized[output_offset]);
		output_offset += value_size;
	}

	return 0;
}

static void uac1_ep_u24_put(struct net_buf *buf, uint32_t value)
{
	net_buf_add_u8(buf, value & 0xFF);
	net_buf_add_u8(buf, (value >> 8) & 0xFF);
	net_buf_add_u8(buf, (value >> 16) & 0xFF);
}

static int uac1_ep_u24_get(const struct net_buf *buf, uint32_t *out)
{
	if (buf->len != 3) {
		return -EINVAL;
	}

	*out = buf->data[0] | (buf->data[1] << 8) | (buf->data[2] << 16);
	return 0;
}

static int handle_ep_sample_rate_get(const struct device *dev, const struct usb_setup_packet *setup,
				     struct net_buf *buf)
{
	const struct uac1_cfg *cfg = dev->config;
	struct uac1_ctx *ctx = dev->data;
	const uint8_t ep = setup->wIndex & 0xFF;
	bool is_feedback;
	int as_idx;
	uint8_t terminal;
	const uint32_t *freqs;
	uint8_t count;

	as_idx = ep_to_as_interface(dev, ep, &is_feedback);
	if (as_idx < 0 || is_feedback) {
		errno = -ENOTSUP;
		return 0;
	}

	terminal = cfg->as_terminals[as_idx];
	freqs = cfg->as_frequencies[as_idx];
	count = cfg->as_frequencies_count[as_idx];

	if (CONTROL_SELECTOR(setup) != EP_SAMPLING_FREQ_CONTROL) {
		errno = -ENOTSUP;
		return 0;
	}

	if (CONTROL_ATTRIBUTE(setup) == GET_CUR) {
		if (count == 1) {
			uac1_ep_u24_put(buf, freqs[0]);
			return 0;
		}

		if (ctx->ops->get_sample_rate) {
			uac1_ep_u24_put(buf, ctx->ops->get_sample_rate(dev, terminal, ctx->user_data));
			return 0;
		}
	} else if (CONTROL_ATTRIBUTE(setup) == GET_MIN) {
		uac1_ep_u24_put(buf, freqs[0]);
		return 0;
	} else if (CONTROL_ATTRIBUTE(setup) == GET_MAX) {
		uac1_ep_u24_put(buf, freqs[count - 1]);
		return 0;
	} else if (CONTROL_ATTRIBUTE(setup) == GET_RES) {
		uac1_ep_u24_put(buf, 0);
		return 0;
	}

	errno = -ENOTSUP;
	return 0;
}

static int handle_ep_sample_rate_set(const struct device *dev, const struct usb_setup_packet *setup,
				     const struct net_buf *buf)
{
	const struct uac1_cfg *cfg = dev->config;
	struct uac1_ctx *ctx = dev->data;
	const uint8_t ep = setup->wIndex & 0xFF;
	bool is_feedback;
	int as_idx;
	uint8_t terminal;
	const uint32_t *freqs;
	uint8_t count;
	uint32_t requested, hz;
	int err;

	as_idx = ep_to_as_interface(dev, ep, &is_feedback);
	if (as_idx < 0 || is_feedback) {
		errno = -ENOTSUP;
		return 0;
	}

	terminal = cfg->as_terminals[as_idx];
	freqs = cfg->as_frequencies[as_idx];
	count = cfg->as_frequencies_count[as_idx];

	if (CONTROL_SELECTOR(setup) != EP_SAMPLING_FREQ_CONTROL || CONTROL_ATTRIBUTE(setup) != SET_CUR) {
		errno = -ENOTSUP;
		return 0;
	}

	err = uac1_ep_u24_get(buf, &requested);
	if (err) {
		errno = err;
		return 0;
	}

	hz = find_closest(requested, freqs, count);

	if (ctx->ops->set_sample_rate == NULL) {
		if (count > 1) {
			errno = -ENOTSUP;
		}
		return 0;
	}

	err = ctx->ops->set_sample_rate(dev, terminal, hz, ctx->user_data);
	if (err) {
		errno = err;
	}

	return 0;
}

static int uac1_control_to_dev(struct usbd_class_data *const c_data,
			       const struct usb_setup_packet *const setup,
			       const struct net_buf *const buf)
{
	const struct device *dev = usbd_class_get_private(c_data);
	const struct uac1_cfg *cfg = dev->config;

	if (setup->bmRequestType == SET_CLASS_INTERFACE_REQUEST_TYPE) {
		if ((setup->wIndex & 0xFFU) != uac1_ac_interface(cfg)) {
			errno = -EINVAL;
			return 0;
		}
		if (CONTROL_ATTRIBUTE(setup) == SET_CUR) {
			return handle_fu_set(dev, setup, buf);
		}
	} else if (setup->bmRequestType == SET_CLASS_ENDPOINT_REQUEST_TYPE) {
		if ((setup->wIndex & 0xFF00U) != 0U || (setup->wValue & 0xFFU) != 0U) {
			errno = -EINVAL;
			return 0;
		}
		return handle_ep_sample_rate_set(dev, setup, buf);
	}

	errno = -ENOTSUP;
	return 0;
}

static struct net_buf *uac1_control_to_host(struct usbd_class_data *const c_data,
					    const struct usb_setup_packet *const setup)
{
	const struct device *dev = usbd_class_get_private(c_data);
	const struct uac1_cfg *cfg = dev->config;
	struct net_buf *buf;
	int ret = 0;

	buf = usbd_ep_ctrl_data_in_alloc(usbd_class_get_ctx(c_data), setup->wLength);
	if (buf == NULL) {
		return NULL;
	}

	if (setup->bmRequestType == GET_CLASS_INTERFACE_REQUEST_TYPE) {
		if ((setup->wIndex & 0xFFU) != uac1_ac_interface(cfg)) {
			ret = -EINVAL;
			goto fail;
		}
		if (CONTROL_ATTRIBUTE(setup) == GET_STAT && setup->wValue == 0U) {
			return buf;
		}
		ret = handle_fu_get(dev, setup, buf);
	} else if (setup->bmRequestType == GET_CLASS_ENDPOINT_REQUEST_TYPE) {
		if ((setup->wIndex & 0xFF00U) != 0U) {
			ret = -EINVAL;
			goto fail;
		}
		if (CONTROL_ATTRIBUTE(setup) == GET_STAT && setup->wValue == 0U) {
			bool is_feedback;

			if (ep_to_as_interface(dev, setup->wIndex & 0xFFU, &is_feedback) < 0) {
				ret = -EINVAL;
				goto fail;
			}
			return buf;
		}
		if ((setup->wValue & 0xFFU) != 0U) {
			ret = -EINVAL;
			goto fail;
		}
		ret = handle_ep_sample_rate_get(dev, setup, buf);
	} else {
		ret = -ENOTSUP;
	}

	if (ret == 0 && errno == 0) {
		return buf;
	}

fail:
	net_buf_unref(buf);
	return NULL;
}

static void uac1_update(struct usbd_class_data *const c_data, uint8_t iface, uint8_t alternate)
{
	const struct device *dev = usbd_class_get_private(c_data);
	struct usbd_context *uds_ctx = usbd_class_get_ctx(c_data);
	const struct uac1_cfg *cfg = dev->config;
	struct uac1_ctx *ctx = dev->data;
	const struct usb_desc_header **descriptors;
	const struct usb_association_descriptor *iad;
	uint8_t as_idx;
	bool microframes;

	LOG_DBG("iface %d alt %d", iface, alternate);

	if (usbd_bus_speed(uds_ctx) == USBD_SPEED_FS) {
		microframes = false;
		descriptors = cfg->fs_descriptors;
	} else {
		microframes = true;
		descriptors = cfg->hs_descriptors;
	}

	if (!descriptors) {
		return;
	}

	iad = (const struct usb_association_descriptor *)descriptors[0];
	if (iface <= iad->bFirstInterface ||
	    iface >= iad->bFirstInterface + iad->bInterfaceCount) {
		LOG_WRN("Ignoring update for non-streaming interface %u", iface);
		return;
	}
	as_idx = iface - iad->bFirstInterface - 1;
	if (as_idx >= cfg->num_ifaces || alternate > 1U) {
		LOG_WRN("Ignoring invalid interface update %u alt %u", iface, alternate);
		return;
	}

	if (alternate == 0U) {
		if (atomic_test_and_clear_bit(&ctx->as_selected, as_idx)) {
			atomic_inc(&cfg->streams[as_idx].generation);
			ctx->ops->stream_event_cb(dev, cfg->as_terminals[as_idx],
						  UAC1_STREAM_DEACTIVATED, microframes,
						  ctx->user_data);
		}
		return;
	}

	if (!atomic_test_and_set_bit(&ctx->as_selected, as_idx)) {
		atomic_inc(&cfg->streams[as_idx].generation);
		cfg->streams[as_idx].service_number = 0U;
		ctx->ops->stream_event_cb(dev, cfg->as_terminals[as_idx],
					  UAC1_STREAM_ACTIVATED, microframes, ctx->user_data);
	}

	/*
	 * The update callback runs synchronously while Chapter 9 is handling
	 * SET_INTERFACE, before it has queued EP0's IN status stage.  Starting
	 * isochronous traffic here lets a controller service data/feedback IN
	 * endpoints ahead of that status stage and can leave the host waiting
	 * forever for the control transfer to finish.  The next SOF services all
	 * selected streams and provides the same double-buffered startup without
	 * coupling stream I/O to EP0 request processing.
	 */
}

static int uac1_request(struct usbd_class_data *const c_data, struct net_buf *buf, int err)
{
	const struct device *dev = usbd_class_get_private(c_data);
	const struct uac1_cfg *cfg = dev->config;
	struct uac1_ctx *ctx = dev->data;
	struct usbd_context *uds_ctx = usbd_class_get_ctx(c_data);
	struct udc_buf_info *bi;
	struct uac1_buf_info *info;
	uint8_t ep;
	uint8_t terminal;
	uint32_t generation;
	int as_idx;
	bool is_feedback;

	bi = udc_get_buf_info(buf);
	info = net_buf_user_data(buf);
	if (err) {
		if (err == -ECONNABORTED) {
			LOG_WRN("request ep 0x%02x, len %u cancelled", bi->ep, buf->len);
		} else {
			LOG_ERR("request ep 0x%02x, len %u failed", bi->ep, buf->len);
		}
	}

	ep = bi->ep;
	if (ep == cfg->status_ep) {
		atomic_clear(&ctx->status_queued);
		usbd_ep_buf_free(uds_ctx, buf);
		return 0;
	}

	as_idx = info->as_idx;
	is_feedback = info->is_feedback;
	if (as_idx < 0 || as_idx >= cfg->num_ifaces) {
		LOG_ERR("Completion for unknown endpoint 0x%02x", ep);
		usbd_ep_buf_free(uds_ctx, buf);
		return 0;
	}
	terminal = cfg->as_terminals[as_idx];
	generation = info->generation;

	for (struct net_buf *fragment = buf; fragment != NULL; fragment = fragment->frags) {
		if (is_feedback) {
			outstanding_release(&cfg->streams[as_idx].feedback_outstanding);
		} else {
			outstanding_release(&cfg->streams[as_idx].data_outstanding);
			if (USB_EP_DIR_IS_OUT(ep)) {
				release_rx_buffer(dev, terminal, fragment->__buf,
						  err == 0 ? fragment->len : 0U, err);
			} else {
				release_tx_buffer(dev, terminal, fragment->__buf, err);
			}
		}
	}

	usbd_ep_buf_free(uds_ctx, buf);
	if (err || generation != (uint32_t)atomic_get(&cfg->streams[as_idx].generation)) {
		return 0;
	}

	if (USB_EP_DIR_IS_OUT(ep)) {
		schedule_iso_out_read(c_data, as_idx);
	} else if (is_feedback) {
		write_explicit_feedback(c_data, as_idx);
	} else {
		schedule_iso_in_write(c_data, as_idx);
	}

	return 0;
}

static void uac1_sof(struct usbd_class_data *const c_data)
{
	const struct device *dev = usbd_class_get_private(c_data);
	const struct uac1_cfg *cfg = dev->config;
	struct uac1_ctx *ctx = dev->data;

	if (ctx->ops->sof_cb != NULL) {
		ctx->ops->sof_cb(dev, ctx->user_data);
	}

	for (int as_idx = 0; as_idx < cfg->num_ifaces; as_idx++) {
		const uint8_t data_ep = as_data_ep(dev, as_idx);
		const uint8_t fb_ep = as_feedback_ep(dev, as_idx);

		if (data_ep && USB_EP_DIR_IS_OUT(data_ep)) {
			schedule_iso_out_read(c_data, as_idx);
		} else if (data_ep) {
			schedule_iso_in_write(c_data, as_idx);
		}

		if (!fb_ep) {
			continue;
		}

		write_explicit_feedback(c_data, as_idx);
	}
}

static void *uac1_get_desc(struct usbd_class_data *const c_data, const enum usbd_speed speed)
{
	struct device *dev = usbd_class_get_private(c_data);
	const struct uac1_cfg *cfg = dev->config;

	if (USBD_SUPPORTS_HIGH_SPEED && speed == USBD_SPEED_HS) {
		return cfg->hs_descriptors;
	}

	return cfg->fs_descriptors;
}

static void uac1_notify_selected(struct usbd_class_data *const c_data, atomic_val_t selected,
				 enum uac1_stream_event event)
{
	const struct device *dev = usbd_class_get_private(c_data);
	const struct uac1_cfg *cfg = dev->config;
	struct uac1_ctx *ctx = dev->data;
	const bool microframes =
		USBD_SUPPORTS_HIGH_SPEED && usbd_bus_speed(c_data->uds_ctx) == USBD_SPEED_HS;

	while (selected != 0) {
		unsigned int as_idx = find_lsb_set(selected) - 1U;

		ctx->ops->stream_event_cb(dev, cfg->as_terminals[as_idx], event, microframes,
					 ctx->user_data);
		selected &= ~BIT(as_idx);
	}
}

static void uac1_suspended(struct usbd_class_data *const c_data)
{
	const struct device *dev = usbd_class_get_private(c_data);
	struct uac1_ctx *ctx = dev->data;

	if (atomic_cas(&ctx->suspended, 0, 1)) {
		uac1_notify_selected(c_data, atomic_get(&ctx->as_selected), UAC1_STREAM_SUSPENDED);
	}
}

static void uac1_resumed(struct usbd_class_data *const c_data)
{
	const struct device *dev = usbd_class_get_private(c_data);
	struct uac1_ctx *ctx = dev->data;
	atomic_val_t selected = atomic_get(&ctx->as_selected);

	if (!atomic_cas(&ctx->suspended, 1, 0)) {
		return;
	}
	uac1_notify_selected(c_data, selected, UAC1_STREAM_RESUMED);

	while (selected != 0) {
		unsigned int as_idx = find_lsb_set(selected) - 1U;
		const uint8_t data_ep = as_data_ep(dev, as_idx);
		const uint8_t fb_ep = as_feedback_ep(dev, as_idx);

		if (data_ep && USB_EP_DIR_IS_OUT(data_ep)) {
			schedule_iso_out_read(c_data, as_idx);
		} else if (data_ep) {
			schedule_iso_in_write(c_data, as_idx);
		}
		if (fb_ep) {
			write_explicit_feedback(c_data, as_idx);
		}

		selected &= ~BIT(as_idx);
	}
}

static void uac1_disable(struct usbd_class_data *const c_data)
{
	const struct device *dev = usbd_class_get_private(c_data);
	const struct uac1_cfg *cfg = dev->config;
	struct uac1_ctx *ctx = dev->data;
	atomic_val_t selected = atomic_clear(&ctx->as_selected);

	atomic_clear(&ctx->suspended);
	atomic_clear(&ctx->status_queued);
	for (uint8_t as_idx = 0U; as_idx < cfg->num_ifaces; as_idx++) {
		atomic_inc(&cfg->streams[as_idx].generation);
		cfg->streams[as_idx].service_number = 0U;
	}

	uac1_notify_selected(c_data, selected, UAC1_STREAM_DISABLED);
}

static int uac1_init(struct usbd_class_data *const c_data)
{
	const struct device *dev = usbd_class_get_private(c_data);
	struct uac1_ctx *ctx = dev->data;

	if (ctx->ops == NULL) {
		LOG_ERR("Application did not register UAC1 ops");
		return -EINVAL;
	}

	return 0;
}

static struct usbd_class_api uac1_api = {
	.update = uac1_update,
	.control_to_dev = uac1_control_to_dev,
	.control_to_host = uac1_control_to_host,
	.request = uac1_request,
	.suspended = uac1_suspended,
	.resumed = uac1_resumed,
	.sof = uac1_sof,
	.get_desc = uac1_get_desc,
	.disable = uac1_disable,
	.init = uac1_init,
};

#define DEFINE_AS_DATA_EP_ADDRS_FS(node)					\
	IF_ENABLED(DT_NODE_HAS_COMPAT(node, zephyr_uac1_audio_streaming), (	\
		COND_CODE_1(UAC1_ALLOWED_AT_FULL_SPEED(DT_PARENT(node)), (	\
			COND_CODE_1(AS_HAS_ISOCHRONOUS_DATA_ENDPOINT(node),	\
				(AS_DATA_EP_ADDR(node),), (0,))		\
		), (0,))							\
	))

#define DEFINE_AS_DATA_EP_ADDRS_HS(node)					\
	IF_ENABLED(DT_NODE_HAS_COMPAT(node, zephyr_uac1_audio_streaming), (	\
		COND_CODE_1(UAC1_ALLOWED_AT_HIGH_SPEED(DT_PARENT(node)), (	\
			COND_CODE_1(AS_HAS_ISOCHRONOUS_DATA_ENDPOINT(node),	\
				(AS_DATA_EP_ADDR(node),), (0,))		\
		), (0,))							\
	))

#define DEFINE_AS_FB_EP_ADDRS_FS(node)						\
	IF_ENABLED(DT_NODE_HAS_COMPAT(node, zephyr_uac1_audio_streaming), (	\
		COND_CODE_1(UAC1_ALLOWED_AT_FULL_SPEED(DT_PARENT(node)), (	\
			COND_CODE_1(AS_HAS_EXPLICIT_FEEDBACK_ENDPOINT(node),	\
				(AS_FEEDBACK_EP_ADDR(node),), (0,))	\
		), (0,))							\
	))

#define DEFINE_AS_FB_EP_ADDRS_HS(node)						\
	IF_ENABLED(DT_NODE_HAS_COMPAT(node, zephyr_uac1_audio_streaming), (	\
		COND_CODE_1(UAC1_ALLOWED_AT_HIGH_SPEED(DT_PARENT(node)), (	\
			COND_CODE_1(AS_HAS_EXPLICIT_FEEDBACK_ENDPOINT(node),	\
				(AS_FEEDBACK_EP_ADDR(node),), (0,))	\
		), (0,))							\
	))

#define DEFINE_AS_DATA_MPS_FS(node)						\
	IF_ENABLED(DT_NODE_HAS_COMPAT(node, zephyr_uac1_audio_streaming), (	\
		COND_CODE_1(UAC1_ALLOWED_AT_FULL_SPEED(DT_PARENT(node)), (	\
			COND_CODE_1(AS_HAS_ISOCHRONOUS_DATA_ENDPOINT(node),	\
				(AS_FS_DATA_EP_MAX_PACKET_SIZE(node),), (0,)) \
		), (0,))							\
	))

#define DEFINE_AS_DATA_MPS_HS(node)						\
	IF_ENABLED(DT_NODE_HAS_COMPAT(node, zephyr_uac1_audio_streaming), (	\
		COND_CODE_1(UAC1_ALLOWED_AT_HIGH_SPEED(DT_PARENT(node)), (	\
			COND_CODE_1(AS_HAS_ISOCHRONOUS_DATA_ENDPOINT(node),	\
				(AS_HS_DATA_EP_MAX_PACKET_SIZE(node),), (0,)) \
		), (0,))							\
	))

#define DEFINE_AS_TERMINALS(node)						\
	IF_ENABLED(DT_NODE_HAS_COMPAT(node, zephyr_uac1_audio_streaming), (	\
		ENTITY_ID(DT_PROP(node, linked_terminal)),			\
	))

#define FREQUENCY_TABLE_NAME(node, i) UTIL_CAT(freqs_##i##_, DT_NODE_CHILD_IDX(node))
#define DEFINE_AS_FREQUENCY_TABLE(node, i)					\
	IF_ENABLED(DT_NODE_HAS_COMPAT(node, zephyr_uac1_audio_streaming), (	\
		static const uint32_t FREQUENCY_TABLE_NAME(node, i)[] =		\
			DT_PROP(node, sampling_frequencies);			\
	))

#define DEFINE_AS_FREQUENCY_PTR(node, i)					\
	IF_ENABLED(DT_NODE_HAS_COMPAT(node, zephyr_uac1_audio_streaming), (	\
		FREQUENCY_TABLE_NAME(node, i),					\
	))

#define DEFINE_AS_FREQUENCY_COUNT(node)					\
	IF_ENABLED(DT_NODE_HAS_COMPAT(node, zephyr_uac1_audio_streaming), (	\
		DT_PROP_LEN(node, sampling_frequencies),			\
	))

#define DEFINE_FU_CONTROL(i, node)						\
	FEATURE_UNIT_CONTROLS_BY_IDX(i, node),

#define DEFINE_FU_STATE(i, node)						\
	{ .volume = (int16_t)DT_PROP_OR(node, volume_max, 0) },

#define FU_CHANNEL_CONTROL_HAS_VOLUME(node, prop, idx)				\
	(DT_PROP_BY_IDX(node, prop, idx) & FU_CTRL_VOLUME_BIT)

#define FU_CHANNEL_CONTROL_IS_VALID(node, prop, idx)				\
	((DT_PROP_BY_IDX(node, prop, idx) & ~(FU_CTRL_MUTE_BIT | FU_CTRL_VOLUME_BIT)) == 0)

#define FU_HAS_VOLUME(node)							\
	((DT_PROP(node, controls) & FU_CTRL_VOLUME_BIT) ||			\
	 COND_CODE_1(DT_NODE_HAS_PROP(node, channel_controls),		\
		(DT_FOREACH_PROP_ELEM_SEP(node, channel_controls,		\
			FU_CHANNEL_CONTROL_HAS_VOLUME, (||))), (0)))

#define FU_CONTROLS_ARE_VALID(node)						\
	(((DT_PROP(node, controls) & ~(FU_CTRL_MUTE_BIT | FU_CTRL_VOLUME_BIT)) == 0) &&\
	 COND_CODE_1(DT_NODE_HAS_PROP(node, channel_controls),		\
		(DT_FOREACH_PROP_ELEM_SEP(node, channel_controls,		\
			FU_CHANNEL_CONTROL_IS_VALID, (&&))), (1)))

#define FU_CHANNEL_CONTROLS_LENGTH_IS_VALID(node)				\
	COND_CODE_1(DT_NODE_HAS_PROP(node, channel_controls),			\
		(DT_PROP_LEN(node, channel_controls) == FEATURE_UNIT_NUM_CHANNELS(node)), (1))

#define DEFINE_FU_RUNTIME_DATA(node)						\
	IF_ENABLED(DT_NODE_HAS_COMPAT(node, zephyr_uac1_feature_unit), (	\
		BUILD_ASSERT(FEATURE_UNIT_NUM_CHANNELS(node) <= 31,		\
			     "Feature Units support at most 31 logical channels");\
		BUILD_ASSERT(FU_CHANNEL_CONTROLS_LENGTH_IS_VALID(node),		\
			     "channel-controls must have one entry per channel");\
		BUILD_ASSERT(FU_CONTROLS_ARE_VALID(node),			\
			     "only mute and volume Feature Unit controls are supported");\
		BUILD_ASSERT(!FU_HAS_VOLUME(node) ||				\
			     (DT_NODE_HAS_PROP(node, volume_min) &&		\
			      DT_NODE_HAS_PROP(node, volume_max) &&		\
			      DT_NODE_HAS_PROP(node, volume_res)),		\
			     "volume controls require volume-min, volume-max, and volume-res");\
		BUILD_ASSERT(!FU_HAS_VOLUME(node) ||				\
			     (int32_t)DT_PROP_OR(node, volume_res, 0) > 0,	\
			     "volume-res must be positive");			\
		BUILD_ASSERT(!FU_HAS_VOLUME(node) ||				\
			     (int32_t)DT_PROP_OR(node, volume_min, 0) <=	\
				     (int32_t)DT_PROP_OR(node, volume_max, 0),\
			     "volume-min must not exceed volume-max");		\
		BUILD_ASSERT(!FU_HAS_VOLUME(node) ||				\
			     ((int32_t)DT_PROP_OR(node, volume_min, 0) >= INT16_MIN &&\
			      (int32_t)DT_PROP_OR(node, volume_max, 0) <= INT16_MAX &&\
			      (int32_t)DT_PROP_OR(node, volume_res, 0) <= INT16_MAX),\
			     "volume range must fit in signed 16-bit values");	\
		BUILD_ASSERT(!FU_HAS_VOLUME(node) ||				\
			     ((int32_t)DT_PROP_OR(node, volume_max, 0) -	\
			      (int32_t)DT_PROP_OR(node, volume_min, 0)) %	\
				     (int32_t)DT_PROP_OR(node, volume_res, 1) == 0,\
			     "volume range must be divisible by volume-res");	\
		static const uint8_t DESCRIPTOR_NAME(fu_controls, node)[] = {\
			LISTIFY(UTIL_INC(FEATURE_UNIT_NUM_CHANNELS(node)),	\
				DEFINE_FU_CONTROL, (), node)			\
		};							\
		static struct uac1_fu_state DESCRIPTOR_NAME(fu_state, node)[] = {\
			LISTIFY(UTIL_INC(FEATURE_UNIT_NUM_CHANNELS(node)),	\
				DEFINE_FU_STATE, (), node)			\
		};							\
	))

#define DEFINE_FU_CFG(node)							\
	IF_ENABLED(DT_NODE_HAS_COMPAT(node, zephyr_uac1_feature_unit), (	\
		{								\
			.unit_id = ENTITY_ID(node),				\
			.num_channels = FEATURE_UNIT_NUM_CHANNELS(node),	\
			.controls = DESCRIPTOR_NAME(fu_controls, node),		\
			.vol_min = (int16_t)DT_PROP_OR(node, volume_min, 0),	\
			.vol_max = (int16_t)DT_PROP_OR(node, volume_max, 0),	\
			.vol_res = (int16_t)DT_PROP_OR(node, volume_res, 0),	\
			.state = DESCRIPTOR_NAME(fu_state, node),		\
		},								\
	))

#define COUNT_FU(node)								\
	IF_ENABLED(DT_NODE_HAS_COMPAT(node, zephyr_uac1_feature_unit), (+ 1))

#define DEFINE_UAC1_CLASS_DATA(inst)						\
	BUILD_ASSERT(UAC1_ALLOWED_AT_FULL_SPEED(DT_DRV_INST(inst)) ||		\
		     UAC1_ALLOWED_AT_HIGH_SPEED(DT_DRV_INST(inst)),		\
		     "UAC1 instance must enable at least one speed");		\
	static struct uac1_ctx uac1_ctx_##inst;					\
	UAC1_DESCRIPTOR_ARRAYS(DT_DRV_INST(inst))				\
	DT_INST_FOREACH_CHILD_STATUS_OKAY(inst, DEFINE_FU_RUNTIME_DATA)	\
	IF_ENABLED(UAC1_ALLOWED_AT_FULL_SPEED(DT_DRV_INST(inst)), (		\
		static const struct usb_desc_header *uac1_fs_desc_##inst[] =	\
			UAC1_FS_DESCRIPTOR_PTRS_ARRAY(DT_DRV_INST(inst));	\
	))									\
	IF_ENABLED(UAC1_ALLOWED_AT_HIGH_SPEED(DT_DRV_INST(inst)), (		\
		static const struct usb_desc_header *uac1_hs_desc_##inst[] =	\
			UAC1_HS_DESCRIPTOR_PTRS_ARRAY(DT_DRV_INST(inst));	\
	))									\
	static const uint8_t uac1_as_terminals_##inst[] = {			\
		DT_INST_FOREACH_CHILD_STATUS_OKAY(inst, DEFINE_AS_TERMINALS)	\
	};									\
	static struct uac1_stream_runtime uac1_streams_##inst[			\
		ARRAY_SIZE(uac1_as_terminals_##inst)];				\
	IF_ENABLED(UAC1_ALLOWED_AT_FULL_SPEED(DT_DRV_INST(inst)), (		\
		static const uint8_t uac1_fs_data_ep_##inst[] = {		\
			DT_INST_FOREACH_CHILD_STATUS_OKAY(inst, DEFINE_AS_DATA_EP_ADDRS_FS)\
		};								\
		static const uint16_t uac1_fs_data_mps_##inst[] = {		\
			DT_INST_FOREACH_CHILD_STATUS_OKAY(inst, DEFINE_AS_DATA_MPS_FS)\
		};								\
		static const uint8_t uac1_fs_fb_ep_##inst[] = {			\
			DT_INST_FOREACH_CHILD_STATUS_OKAY(inst, DEFINE_AS_FB_EP_ADDRS_FS)\
		};								\
	))									\
	IF_ENABLED(UAC1_ALLOWED_AT_HIGH_SPEED(DT_DRV_INST(inst)), (		\
		static const uint8_t uac1_hs_data_ep_##inst[] = {		\
			DT_INST_FOREACH_CHILD_STATUS_OKAY(inst, DEFINE_AS_DATA_EP_ADDRS_HS)\
		};								\
		static const uint16_t uac1_hs_data_mps_##inst[] = {		\
			DT_INST_FOREACH_CHILD_STATUS_OKAY(inst, DEFINE_AS_DATA_MPS_HS)\
		};								\
		static const uint8_t uac1_hs_fb_ep_##inst[] = {			\
			DT_INST_FOREACH_CHILD_STATUS_OKAY(inst, DEFINE_AS_FB_EP_ADDRS_HS)\
		};								\
	))									\
	DT_INST_FOREACH_CHILD_STATUS_OKAY_VARGS(inst, DEFINE_AS_FREQUENCY_TABLE, inst)\
	static const uint32_t *const uac1_as_freq_##inst[] = {			\
		DT_INST_FOREACH_CHILD_STATUS_OKAY_VARGS(inst, DEFINE_AS_FREQUENCY_PTR, inst)\
	};									\
	static const uint8_t uac1_as_freq_count_##inst[] = {			\
		DT_INST_FOREACH_CHILD_STATUS_OKAY(inst, DEFINE_AS_FREQUENCY_COUNT)\
	};									\
	static const struct uac1_fu_cfg uac1_fu_cfg_##inst[] = {		\
		DT_INST_FOREACH_CHILD_STATUS_OKAY(inst, DEFINE_FU_CFG)		\
		{ .unit_id = 0 },						\
	};									\
	USBD_DEFINE_CLASS(uac1_##inst, &uac1_api,				\
			  (void *)DEVICE_DT_GET(DT_DRV_INST(inst)), NULL);	\
	static const struct uac1_cfg uac1_cfg_##inst = {			\
		.c_data = &uac1_##inst,						\
		COND_CODE_1(UAC1_ALLOWED_AT_FULL_SPEED(DT_DRV_INST(inst)),	\
			(.fs_descriptors = uac1_fs_desc_##inst,			\
			 .fs_data_ep = uac1_fs_data_ep_##inst,		\
			 .fs_data_mps = uac1_fs_data_mps_##inst,		\
			 .fs_fb_ep = uac1_fs_fb_ep_##inst,),			\
			(.fs_descriptors = NULL,					\
			 .fs_data_ep = NULL,					\
			 .fs_data_mps = NULL,					\
			 .fs_fb_ep = NULL,)					\
		)								\
		COND_CODE_1(UAC1_ALLOWED_AT_HIGH_SPEED(DT_DRV_INST(inst)),	\
			(.hs_descriptors = uac1_hs_desc_##inst,			\
			 .hs_data_ep = uac1_hs_data_ep_##inst,		\
			 .hs_data_mps = uac1_hs_data_mps_##inst,		\
			 .hs_fb_ep = uac1_hs_fb_ep_##inst,),			\
			(.hs_descriptors = NULL,					\
			 .hs_data_ep = NULL,					\
			 .hs_data_mps = NULL,					\
			 .hs_fb_ep = NULL,)					\
		)								\
		.as_terminals = uac1_as_terminals_##inst,			\
		.as_frequencies = uac1_as_freq_##inst,				\
		.as_frequencies_count = uac1_as_freq_count_##inst,		\
		.fu_cfg = uac1_fu_cfg_##inst,					\
		.streams = uac1_streams_##inst,					\
		.status_ep = COND_CODE_1(DT_INST_PROP(inst, interrupt_endpoint),	\
					     (FIRST_IN_EP_ADDR), (0)),		\
		.num_ifaces = ARRAY_SIZE(uac1_as_terminals_##inst),		\
		.num_fu = ARRAY_SIZE(uac1_fu_cfg_##inst) - 1,			\
	};									\
	BUILD_ASSERT(ARRAY_SIZE(uac1_as_terminals_##inst) <= 32,		\
		     "UAC1 implementation supports up to 32 AS interfaces");	\
	DEVICE_DT_DEFINE(DT_DRV_INST(inst), NULL, NULL,				\
			 &uac1_ctx_##inst, &uac1_cfg_##inst,			\
			 POST_KERNEL, CONFIG_KERNEL_INIT_PRIORITY_DEVICE,	\
			 NULL);

DT_INST_FOREACH_STATUS_OKAY(DEFINE_UAC1_CLASS_DATA)
