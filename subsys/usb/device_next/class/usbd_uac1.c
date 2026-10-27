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
		+ USBD_UAC1_BUFFERS_PER_STREAM *				\
		  (AS_HAS_ISOCHRONOUS_DATA_ENDPOINT(node) +			\
		   AS_HAS_EXPLICIT_FEEDBACK_ENDPOINT(node))))
#define COUNT_UAC1_EP_BUFFERS(i)						\
	+ DT_PROP(DT_DRV_INST(i), interrupt_endpoint)				\
	DT_INST_FOREACH_CHILD(i, COUNT_UAC1_AS_ENDPOINT_BUFFERS)
#define UAC1_NUM_EP_BUFFERS DT_INST_FOREACH_STATUS_OKAY(COUNT_UAC1_EP_BUFFERS)

enum uac1_buf_kind {
	UAC1_BUF_DATA,
	UAC1_BUF_FEEDBACK,
	UAC1_BUF_STATUS,
};

struct uac1_buf_info {
	struct udc_buf_info udc;
	uint32_t generation;
	uint8_t as_idx;
	uint8_t kind;
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

/* Channel number addressing all of a Feature Unit's channels (Audio 1.0, 5.2.2.4.1) */
#define ALL_CHANNELS			0xFF

/* Status word D7 Interrupt Pending (Audio 1.0, Table 3-1) */
#define STATUS_INTERRUPT_PENDING	BIT(7)
#define STATUS_MEMORY_CHANGED		BIT(6)

#define CONTROL_ATTRIBUTE(setup)	(setup->bRequest)
#define CONTROL_ENTITY_ID(setup)	((setup->wIndex & 0xFF00) >> 8)
#define CONTROL_SELECTOR(setup)		((setup->wValue & 0xFF00) >> 8)
#define CONTROL_CHANNEL_NUMBER(setup)	((setup->wValue & 0x00FF))

/* Feature Units support at most 31 logical channels plus the master channel. */
#define FU_MAX_VALUES			32U
#define MAX_RESPONSE_SIZE		(FU_MAX_VALUES * sizeof(int16_t))

/* Standard AS Isochronous Audio Data or Synch Endpoint Descriptor
 * (Audio 1.0, Tables 4-20 and 4-22)
 */
struct uac1_ep_descriptor {
	uint8_t bLength;
	uint8_t bDescriptorType;
	uint8_t bEndpointAddress;
	uint8_t bmAttributes;
	uint16_t wMaxPacketSize;
	uint8_t bInterval;
	uint8_t bRefresh;
	uint8_t bSynchAddress;
} __packed;

/*
 * Descriptors of one bus speed. The stack assigns their interface numbers and
 * endpoint addresses when it initializes the configuration, so the class reads
 * those from here rather than keeping its own copies.
 */
struct uac1_speed_desc {
	const struct usb_desc_header **descriptors;
	struct usb_association_descriptor *iad;
	struct usb_if_descriptor *ac_interface;
	uint8_t *ac_header;
	struct uac1_ep_descriptor *status_ep;
	/* Per AudioStreaming interface, NULL if it has no such endpoint */
	struct uac1_ep_descriptor *const *data_eps;
	struct uac1_ep_descriptor *const *fb_eps;
};

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
	/* An interrupt status transfer is in flight */
	atomic_t status_busy;
	/* Units whose device-side change still has to be notified */
	atomic_t status_deferred;
};

struct uac1_cfg {
	struct usbd_class_data *const c_data;
	const struct uac1_speed_desc *fs;
	const struct uac1_speed_desc *hs;

	const uint8_t *as_terminals;
	const uint32_t *const *as_frequencies;
	const uint8_t *as_frequencies_count;

	const struct uac1_fu_cfg *fu_cfg;
	struct uac1_stream_runtime *streams;
	struct usbd_desc_node *const name_desc;
	uint8_t num_ifaces;
	uint8_t num_fu;
};

static const struct uac1_speed_desc *uac1_speed_desc(const struct uac1_cfg *cfg)
{
	/* An unregistered instance has no context; its full-speed set still describes it */
	if (USBD_SUPPORTS_HIGH_SPEED && cfg->c_data->uds_ctx != NULL &&
	    usbd_bus_speed(cfg->c_data->uds_ctx) == USBD_SPEED_HS) {
		return cfg->hs;
	}

	return cfg->fs;
}

static const struct uac1_ep_descriptor *as_data_ep(const struct uac1_cfg *cfg, uint8_t as_idx)
{
	const struct uac1_speed_desc *desc = uac1_speed_desc(cfg);

	return desc != NULL ? desc->data_eps[as_idx] : NULL;
}

static const struct uac1_ep_descriptor *as_feedback_ep(const struct uac1_cfg *cfg,
							uint8_t as_idx)
{
	const struct uac1_speed_desc *desc = uac1_speed_desc(cfg);

	return desc != NULL ? desc->fb_eps[as_idx] : NULL;
}

/* Bytes per service interval, without the additional transaction bits */
static uint16_t ep_transfer_size(const struct uac1_ep_descriptor *ep)
{
	return USB_MPS_TO_TPL(sys_le16_to_cpu(ep->wMaxPacketSize));
}

static uint8_t uac1_ac_interface(const struct uac1_cfg *cfg)
{
	const struct uac1_speed_desc *desc = uac1_speed_desc(cfg);

	__ASSERT_NO_MSG(desc != NULL);

	return desc->iad->bFirstInterface;
}

static bool stream_available(const struct uac1_ctx *ctx, uint8_t as_idx)
{
	return !atomic_get(&ctx->suspended) && atomic_test_bit(&ctx->as_selected, as_idx);
}

static bool outstanding_acquire(atomic_t *outstanding)
{
	atomic_val_t value = atomic_get(outstanding);

	while (value < USBD_UAC1_BUFFERS_PER_STREAM) {
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

static int ep_to_as_interface(const struct uac1_cfg *cfg, uint8_t ep, bool *is_feedback)
{
	for (uint8_t i = 0; i < cfg->num_ifaces; i++) {
		const struct uac1_ep_descriptor *data_ep = as_data_ep(cfg, i);
		const struct uac1_ep_descriptor *fb_ep = as_feedback_ep(cfg, i);

		if (data_ep != NULL && data_ep->bEndpointAddress == ep) {
			*is_feedback = false;
			return i;
		}

		if (fb_ep != NULL && fb_ep->bEndpointAddress == ep) {
			*is_feedback = true;
			return i;
		}
	}

	*is_feedback = false;
	return -ENOENT;
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
	info->kind = UAC1_BUF_DATA;

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
	const struct uac1_ep_descriptor *data_ep = as_data_ep(cfg, as_idx);
	const uint8_t terminal = cfg->as_terminals[as_idx];
	struct net_buf *buf;
	void *data_buf = NULL;
	uint32_t generation;
	uint16_t size;
	int ret;

	if (!stream_available(ctx, as_idx) || !outstanding_acquire(&stream->data_outstanding)) {
		return;
	}
	generation = atomic_get(&stream->generation);
	size = ep_transfer_size(data_ep);

	ret = ctx->ops->rx_buf_acquire(dev, terminal, size, &data_buf, ctx->user_data);
	if (ret != 0 || data_buf == NULL) {
		if (ret != -EAGAIN) {
			LOG_ERR_RATELIMIT("No receive buffer for terminal %u: %d", terminal,
					  ret != 0 ? ret : -EINVAL);
		}
		outstanding_release(&stream->data_outstanding);
		return;
	}

	buf = uac1_buf_alloc(data_ep->bEndpointAddress, data_buf, size, as_idx, generation);
	if (!buf) {
		LOG_ERR("Invalid or unavailable receive buffer for terminal %u", terminal);
		release_rx_buffer(dev, terminal, data_buf, 0, -ENOMEM);
		outstanding_release(&stream->data_outstanding);
		return;
	}

	ret = usbd_ep_enqueue(c_data, buf);
	if (ret) {
		LOG_ERR("Failed to enqueue net_buf for 0x%02x", data_ep->bEndpointAddress);
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
	const struct uac1_ep_descriptor *data_ep = as_data_ep(cfg, as_idx);
	const uint8_t terminal = cfg->as_terminals[as_idx];
	struct net_buf *buf;
	void *data = NULL;
	uint16_t size = 0U;
	uint16_t max_size;
	uint32_t generation;
	int ret;

	if (!stream_available(ctx, as_idx) || !outstanding_acquire(&stream->data_outstanding)) {
		return;
	}
	generation = atomic_get(&stream->generation);
	max_size = ep_transfer_size(data_ep);

	const uint32_t service_number = stream->service_number++;

	ret = ctx->ops->tx_buf_acquire(dev, terminal, max_size, service_number,
				      &data, &size, ctx->user_data);
	if (ret != 0) {
		if (ret != -EAGAIN) {
			LOG_ERR_RATELIMIT("No transmit buffer for terminal %u: %d", terminal, ret);
		}
		outstanding_release(&stream->data_outstanding);
		return;
	}

	if (data == NULL || size > max_size || !IS_UDC_ALIGNED(data)) {
		LOG_ERR("Invalid transmit buffer for terminal %u", terminal);
		release_tx_buffer(dev, terminal, data, -EINVAL);
		outstanding_release(&stream->data_outstanding);
		return;
	}

	buf = uac1_buf_alloc(data_ep->bEndpointAddress, data, size, as_idx, generation);
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
	}
}

static void write_explicit_feedback(struct usbd_class_data *const c_data, uint8_t as_idx)
{
	const struct device *dev = usbd_class_get_private(c_data);
	const struct uac1_cfg *cfg = dev->config;
	struct usbd_context *uds_ctx = usbd_class_get_ctx(c_data);
	struct uac1_ctx *ctx = dev->data;
	struct uac1_stream_runtime *stream = &cfg->streams[as_idx];
	const struct uac1_ep_descriptor *fb_ep = as_feedback_ep(cfg, as_idx);
	const uint8_t terminal = cfg->as_terminals[as_idx];
	struct uac1_buf_info *info;
	struct net_buf *buf;
	uint32_t fb_value;
	int ret;

	if (!stream_available(ctx, as_idx) ||
	    !outstanding_acquire(&stream->feedback_outstanding)) {
		return;
	}

	buf = net_buf_alloc(&uac1_pool, K_NO_WAIT);
	if (!buf) {
		LOG_ERR("No buf for feedback");
		outstanding_release(&stream->feedback_outstanding);
		return;
	}

	info = net_buf_user_data(buf);
	info->udc.ep = fb_ep->bEndpointAddress;
	info->generation = atomic_get(&stream->generation);
	info->as_idx = as_idx;
	info->kind = UAC1_BUF_FEEDBACK;

	fb_value = ctx->ops->feedback_cb(dev, terminal, ctx->user_data);

	if (usbd_bus_speed(uds_ctx) == USBD_SPEED_FS) {
		net_buf_add_le24(buf, fb_value);
	} else {
		net_buf_add_le32(buf, fb_value);
	}

	ret = usbd_ep_enqueue(c_data, buf);
	if (ret) {
		LOG_ERR("Failed to enqueue net_buf for 0x%02x", fb_ep->bEndpointAddress);
		net_buf_unref(buf);
		outstanding_release(&stream->feedback_outstanding);
	}
}

static void schedule_stream(struct usbd_class_data *const c_data, uint8_t as_idx)
{
	const struct device *dev = usbd_class_get_private(c_data);
	const struct uac1_cfg *cfg = dev->config;
	const struct uac1_ep_descriptor *data_ep = as_data_ep(cfg, as_idx);

	if (data_ep == NULL) {
		return;
	}

	if (USB_EP_DIR_IS_OUT(data_ep->bEndpointAddress)) {
		schedule_iso_out_read(c_data, as_idx);
	} else {
		schedule_iso_in_write(c_data, as_idx);
	}

	if (as_feedback_ep(cfg, as_idx) != NULL) {
		write_explicit_feedback(c_data, as_idx);
	}
}

static bool ops_cover_endpoints(const struct uac1_speed_desc *desc, uint8_t num_ifaces,
				const struct uac1_ops *ops)
{
	if (desc == NULL) {
		return true;
	}

	for (uint8_t i = 0U; i < num_ifaces; i++) {
		const struct uac1_ep_descriptor *data_ep = desc->data_eps[i];

		if (desc->fb_eps[i] != NULL && ops->feedback_cb == NULL) {
			return false;
		}

		if (data_ep == NULL) {
			continue;
		}

		if (USB_EP_DIR_IS_OUT(data_ep->bEndpointAddress) &&
		    (ops->rx_buf_acquire == NULL || ops->rx_buf_release == NULL)) {
			return false;
		}

		if (USB_EP_DIR_IS_IN(data_ep->bEndpointAddress) &&
		    (ops->tx_buf_acquire == NULL || ops->tx_buf_release == NULL)) {
			return false;
		}
	}

	return true;
}

const char *usbd_uac1_class_name(const struct device *dev)
{
	const struct uac1_cfg *cfg = dev->config;

	return cfg->c_data->name;
}

int usbd_uac1_set_ops(const struct device *dev, const struct uac1_ops *ops, void *user_data)
{
	const struct uac1_cfg *cfg = dev->config;
	struct uac1_ctx *ctx = dev->data;

	if (ops == NULL || ops->stream_event_cb == NULL ||
	    !ops_cover_endpoints(cfg->fs, cfg->num_ifaces, ops) ||
	    !ops_cover_endpoints(cfg->hs, cfg->num_ifaces, ops)) {
		return -EINVAL;
	}

	ctx->ops = ops;
	ctx->user_data = user_data;
	return 0;
}

int usbd_uac1_status_notify(const struct device *dev, enum uac1_status_origin origin,
			    uint8_t originator, bool memory_changed)
{
	const struct uac1_cfg *cfg = dev->config;
	const struct uac1_speed_desc *desc = uac1_speed_desc(cfg);
	struct uac1_ctx *ctx = dev->data;
	struct uac1_buf_info *info;
	struct net_buf *buf;
	int ret;

	if (desc == NULL || desc->status_ep == NULL) {
		return -ENOTSUP;
	}

	if (cfg->c_data->uds_ctx == NULL) {
		return -ENODEV;
	}

	if (origin > UAC1_STATUS_AUDIO_STREAMING_ENDPOINT) {
		return -EINVAL;
	}

	if (!atomic_cas(&ctx->status_busy, 0, 1)) {
		return -EAGAIN;
	}

	buf = net_buf_alloc(&uac1_pool, K_NO_WAIT);
	if (buf == NULL) {
		atomic_clear(&ctx->status_busy);
		return -ENOMEM;
	}

	info = net_buf_user_data(buf);
	info->udc.ep = desc->status_ep->bEndpointAddress;
	info->kind = UAC1_BUF_STATUS;
	net_buf_add_u8(buf, STATUS_INTERRUPT_PENDING |
			    (memory_changed ? STATUS_MEMORY_CHANGED : 0U) | origin);
	net_buf_add_u8(buf, originator);

	ret = usbd_ep_enqueue(cfg->c_data, buf);
	if (ret != 0) {
		net_buf_unref(buf);
		atomic_clear(&ctx->status_busy);
	}

	return ret;
}

/* Notifies the next unit whose change arrived while another was being reported. */
static void notify_deferred_status(const struct device *dev)
{
	struct uac1_ctx *ctx = dev->data;
	atomic_val_t deferred = atomic_get(&ctx->status_deferred);
	uint8_t unit_id;
	int ret;

	if (deferred == 0) {
		return;
	}

	unit_id = find_lsb_set(deferred) - 1U;
	if (!atomic_test_and_clear_bit(&ctx->status_deferred, unit_id)) {
		return;
	}

	ret = usbd_uac1_status_notify(dev, UAC1_STATUS_AUDIO_CONTROL_INTERFACE, unit_id, false);
	if (ret != 0 && ret != -ENOTSUP && ret != -ENODEV) {
		atomic_set_bit(&ctx->status_deferred, unit_id);
	}
}

static int feature_value_normalize(const struct uac1_fu_cfg *fu_cfg, uint8_t cs,
				   uint8_t channel, const void *value, uint8_t value_len,
				   uint8_t normalized[sizeof(int16_t)])
{
	if (channel > fu_cfg->num_channels) {
		return -EINVAL;
	}

	if (cs == UAC1_FU_MUTE_CONTROL) {
		if (!(fu_cfg->controls[channel] & FU_CTRL_MUTE_BIT)) {
			return -ENOTSUP;
		}
		if (value_len != sizeof(uint8_t)) {
			return -EINVAL;
		}

		normalized[0] = *(const uint8_t *)value != 0U;
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
		/* 0x8000 is -infinity dB (Audio 1.0, 5.2.2.4.3.2) and is kept as is */
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
	struct uac1_fu_state *fu_state = &fu_cfg->state[channel];

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
	struct uac1_ctx *ctx = dev->data;
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

	/* Without a configuration there is nobody to notify; the host reads the value later */
	ret = usbd_uac1_status_notify(dev, UAC1_STATUS_AUDIO_CONTROL_INTERFACE, unit_id, false);
	__ASSERT_NO_MSG(ret == -ENOTSUP || unit_id < ATOMIC_BITS);
	if (ret == -ENOTSUP || ret == -ENODEV) {
		return 0;
	}

	if (ret != 0) {
		atomic_set_bit(&ctx->status_deferred, unit_id);
	}

	return 0;
}

/* Control bit and value size of a Feature Unit control selector, 0 if unsupported */
static uint8_t feature_control(uint8_t cs, uint8_t *value_size)
{
	if (cs == UAC1_FU_MUTE_CONTROL) {
		*value_size = sizeof(uint8_t);
		return FU_CTRL_MUTE_BIT;
	}

	if (cs == UAC1_FU_VOLUME_CONTROL) {
		*value_size = sizeof(int16_t);
		return FU_CTRL_VOLUME_BIT;
	}

	return 0U;
}

/*
 * Resolves the channels a Feature Unit request addresses. A request to a
 * single channel needs that channel to have the control; a request to all
 * channels covers those that have it.
 */
static int feature_channels(const struct uac1_fu_cfg *fu_cfg, uint8_t channel,
			    uint8_t control_bit, uint8_t *first, uint8_t *last, uint8_t *count)
{
	if (channel == ALL_CHANNELS) {
		*first = 0U;
		*last = fu_cfg->num_channels;
	} else if (channel <= fu_cfg->num_channels) {
		*first = channel;
		*last = channel;
	} else {
		return -EINVAL;
	}

	*count = 0U;
	for (uint8_t i = *first; i <= *last; i++) {
		*count += (fu_cfg->controls[i] & control_bit) != 0U;
	}

	return *count != 0U ? 0 : -ENOTSUP;
}

static int handle_fu_get(const struct device *dev, const struct usb_setup_packet *setup,
			 uint8_t response[MAX_RESPONSE_SIZE])
{
	const struct uac1_cfg *cfg = dev->config;
	const struct uac1_fu_cfg *fu_cfg = find_fu_cfg(cfg, CONTROL_ENTITY_ID(setup));
	const uint8_t cs = CONTROL_SELECTOR(setup);
	const uint8_t request = CONTROL_ATTRIBUTE(setup);
	uint8_t first, last, count;
	uint8_t control_bit;
	uint8_t value_size;
	size_t len = 0U;
	int ret;

	if (fu_cfg == NULL) {
		return -ENOTSUP;
	}

	control_bit = feature_control(cs, &value_size);
	if (control_bit == 0U) {
		return -ENOTSUP;
	}

	/* Mute only has a current setting (Audio 1.0, 5.2.2.4.3.1) */
	if (request != GET_CUR &&
	    (cs == UAC1_FU_MUTE_CONTROL ||
	     (request != GET_MIN && request != GET_MAX && request != GET_RES))) {
		return -ENOTSUP;
	}

	ret = feature_channels(fu_cfg, CONTROL_CHANNEL_NUMBER(setup), control_bit,
			       &first, &last, &count);
	if (ret != 0) {
		return ret;
	}

	for (uint8_t i = first; i <= last; i++) {
		const struct uac1_fu_state *fu_state = &fu_cfg->state[i];
		int16_t volume;

		if (!(fu_cfg->controls[i] & control_bit)) {
			continue;
		}

		if (cs == UAC1_FU_MUTE_CONTROL) {
			response[len] = fu_state->mute ? 1U : 0U;
		} else {
			volume = request == GET_CUR ? fu_state->volume :
				 request == GET_MIN ? fu_cfg->vol_min :
				 request == GET_MAX ? fu_cfg->vol_max : fu_cfg->vol_res;
			sys_put_le16((uint16_t)volume, &response[len]);
		}
		len += value_size;
	}

	return len;
}

static int handle_fu_set(const struct device *dev, const struct usb_setup_packet *setup,
			 const struct net_buf *buf)
{
	const struct uac1_cfg *cfg = dev->config;
	struct uac1_ctx *ctx = dev->data;
	const uint8_t unit_id = CONTROL_ENTITY_ID(setup);
	const struct uac1_fu_cfg *fu_cfg = find_fu_cfg(cfg, unit_id);
	const uint8_t cs = CONTROL_SELECTOR(setup);
	uint8_t normalized[MAX_RESPONSE_SIZE];
	struct uac1_feature_update update = {
		.values = normalized,
	};
	uint8_t first, last, count;
	uint8_t control_bit;
	uint8_t value_size;
	size_t offset = 0U;
	int ret;

	if (fu_cfg == NULL || CONTROL_ATTRIBUTE(setup) != SET_CUR) {
		return -ENOTSUP;
	}

	control_bit = feature_control(cs, &value_size);
	if (control_bit == 0U) {
		return -ENOTSUP;
	}

	ret = feature_channels(fu_cfg, CONTROL_CHANNEL_NUMBER(setup), control_bit,
			       &first, &last, &count);
	if (ret != 0) {
		return ret;
	}

	if (setup->wLength != count * value_size) {
		return -EINVAL;
	}

	/* The first call validates the setup packet; the second supplies the OUT data. */
	if (buf == NULL) {
		return 0;
	}

	if (buf->len != count * value_size) {
		return -EINVAL;
	}

	for (uint8_t i = first; i <= last; i++) {
		if (!(fu_cfg->controls[i] & control_bit)) {
			continue;
		}

		ret = feature_value_normalize(fu_cfg, cs, i, &buf->data[offset], value_size,
					      &normalized[offset]);
		if (ret != 0) {
			return ret;
		}
		update.channel_mask |= BIT(i);
		offset += value_size;
	}

	update.value_size = value_size;
	if (ctx->ops->feature_update_cb != NULL) {
		ret = ctx->ops->feature_update_cb(dev, unit_id, cs, &update, ctx->user_data);
		if (ret != 0) {
			return ret;
		}
	}

	offset = 0U;
	for (uint8_t i = first; i <= last; i++) {
		if ((update.channel_mask & BIT(i)) == 0U) {
			continue;
		}
		feature_value_commit(fu_cfg, cs, i, &normalized[offset]);
		offset += value_size;
	}

	return 0;
}

/*
 * Sampling frequency requests are accepted for single-rate endpoints too, which
 * do not advertise the control: some hosts set the rate regardless.
 */
static int handle_ep_sample_rate_get(const struct device *dev, const struct usb_setup_packet *setup,
				     uint8_t response[MAX_RESPONSE_SIZE])
{
	const struct uac1_cfg *cfg = dev->config;
	struct uac1_ctx *ctx = dev->data;
	bool is_feedback;
	uint32_t rate;
	int as_idx;

	as_idx = ep_to_as_interface(cfg, setup->wIndex & 0xFF, &is_feedback);
	if (as_idx < 0 || is_feedback || CONTROL_SELECTOR(setup) != EP_SAMPLING_FREQ_CONTROL) {
		return -ENOTSUP;
	}

	const uint32_t *freqs = cfg->as_frequencies[as_idx];
	const uint8_t count = cfg->as_frequencies_count[as_idx];

	switch (CONTROL_ATTRIBUTE(setup)) {
	case GET_CUR:
		if (count == 1U) {
			rate = freqs[0];
		} else if (ctx->ops->get_sample_rate != NULL) {
			rate = ctx->ops->get_sample_rate(dev, cfg->as_terminals[as_idx],
							 ctx->user_data);
		} else {
			return -ENOTSUP;
		}
		break;
	case GET_MIN:
		rate = freqs[0];
		break;
	case GET_MAX:
		rate = freqs[count - 1U];
		break;
	case GET_RES:
		rate = 0U;
		break;
	default:
		return -ENOTSUP;
	}

	sys_put_le24(rate, response);
	return 3;
}

static int handle_ep_sample_rate_set(const struct device *dev, const struct usb_setup_packet *setup,
				     const struct net_buf *buf)
{
	const struct uac1_cfg *cfg = dev->config;
	struct uac1_ctx *ctx = dev->data;
	bool is_feedback;
	uint32_t rate;
	int as_idx;

	as_idx = ep_to_as_interface(cfg, setup->wIndex & 0xFF, &is_feedback);
	if (as_idx < 0 || is_feedback || CONTROL_SELECTOR(setup) != EP_SAMPLING_FREQ_CONTROL ||
	    CONTROL_ATTRIBUTE(setup) != SET_CUR) {
		return -ENOTSUP;
	}

	if (setup->wLength != 3U) {
		return -EINVAL;
	}

	/* The first call validates the setup packet; the second supplies the OUT data. */
	if (buf == NULL) {
		return 0;
	}

	if (buf->len != 3U) {
		return -EINVAL;
	}

	const uint8_t count = cfg->as_frequencies_count[as_idx];

	rate = find_closest(sys_get_le24(buf->data), cfg->as_frequencies[as_idx], count);

	if (ctx->ops->set_sample_rate == NULL) {
		return count > 1U ? -ENOTSUP : 0;
	}

	return ctx->ops->set_sample_rate(dev, cfg->as_terminals[as_idx], rate, ctx->user_data);
}

static int uac1_control_to_dev(struct usbd_class_data *const c_data,
			       const struct usb_setup_packet *const setup,
			       const struct net_buf *const buf)
{
	const struct device *dev = usbd_class_get_private(c_data);
	const struct uac1_cfg *cfg = dev->config;

	if (setup->bmRequestType == SET_CLASS_INTERFACE_REQUEST_TYPE) {
		if ((setup->wIndex & 0xFFU) != uac1_ac_interface(cfg)) {
			return -ENOTSUP;
		}

		return handle_fu_set(dev, setup, buf);
	}

	if (setup->bmRequestType == SET_CLASS_ENDPOINT_REQUEST_TYPE) {
		if ((setup->wIndex & 0xFF00U) != 0U || (setup->wValue & 0xFFU) != 0U) {
			return -EINVAL;
		}

		return handle_ep_sample_rate_set(dev, setup, buf);
	}

	return -ENOTSUP;
}

/* GET_STAT returns a null packet (Audio 1.0, 5.2.4.2) */
static int handle_get_stat(const struct uac1_cfg *cfg, const struct usb_setup_packet *setup)
{
	bool is_feedback;

	if (setup->wValue != 0U) {
		return -EINVAL;
	}

	if (setup->bmRequestType == GET_CLASS_ENDPOINT_REQUEST_TYPE &&
	    ep_to_as_interface(cfg, setup->wIndex & 0xFFU, &is_feedback) < 0) {
		return -EINVAL;
	}

	return 0;
}

static struct net_buf *uac1_control_to_host(struct usbd_class_data *const c_data,
					    const struct usb_setup_packet *const setup)
{
	const struct device *dev = usbd_class_get_private(c_data);
	const struct uac1_cfg *cfg = dev->config;
	uint8_t response[MAX_RESPONSE_SIZE];
	struct net_buf *buf;
	int len;

	/* Without a data stage there is no buffer to return the response in */
	if (setup->wLength == 0U) {
		return NULL;
	}

	if (setup->bmRequestType == GET_CLASS_INTERFACE_REQUEST_TYPE) {
		if ((setup->wIndex & 0xFFU) != uac1_ac_interface(cfg)) {
			return NULL;
		}

		len = CONTROL_ATTRIBUTE(setup) == GET_STAT ? handle_get_stat(cfg, setup) :
							     handle_fu_get(dev, setup, response);
	} else if (setup->bmRequestType == GET_CLASS_ENDPOINT_REQUEST_TYPE) {
		if ((setup->wIndex & 0xFF00U) != 0U) {
			return NULL;
		}

		if (CONTROL_ATTRIBUTE(setup) == GET_STAT) {
			len = handle_get_stat(cfg, setup);
		} else if ((setup->wValue & 0xFFU) != 0U) {
			len = -EINVAL;
		} else {
			len = handle_ep_sample_rate_get(dev, setup, response);
		}
	} else {
		len = -ENOTSUP;
	}

	if (len < 0) {
		return NULL;
	}

	/* A response longer than the host asked for is truncated (USB 2.0, 9.3.5) */
	len = MIN(len, setup->wLength);
	buf = usbd_ep_ctrl_data_in_alloc(usbd_class_get_ctx(c_data), len);
	if (buf != NULL) {
		net_buf_add_mem(buf, response, len);
	}

	return buf;
}

static void uac1_update(struct usbd_class_data *const c_data, uint8_t iface, uint8_t alternate)
{
	const struct device *dev = usbd_class_get_private(c_data);
	const struct uac1_cfg *cfg = dev->config;
	const struct uac1_speed_desc *desc = uac1_speed_desc(cfg);
	struct uac1_ctx *ctx = dev->data;
	const bool microframes =
		USBD_SUPPORTS_HIGH_SPEED && usbd_bus_speed(c_data->uds_ctx) == USBD_SPEED_HS;
	uint8_t first;
	uint8_t as_idx;

	LOG_DBG("iface %d alt %d", iface, alternate);

	if (desc == NULL) {
		return;
	}

	/* AudioStreaming interfaces follow the AudioControl interface in order */
	first = desc->iad->bFirstInterface + 1U;
	if (iface < first || iface - first >= cfg->num_ifaces || alternate > 1U) {
		LOG_WRN("Ignoring update for interface %u alt %u", iface, alternate);
		return;
	}
	as_idx = iface - first;

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
	const struct uac1_buf_info *info = net_buf_user_data(buf);
	const uint8_t ep = info->udc.ep;
	const uint8_t kind = info->kind;
	const uint8_t as_idx = info->as_idx;
	const uint32_t generation = info->generation;
	struct uac1_stream_runtime *stream;
	uint8_t terminal;
	int status;

	if (err == -ECONNABORTED) {
		LOG_DBG("request ep 0x%02x, len %u cancelled", ep, buf->len);
	} else if (err) {
		LOG_ERR("request ep 0x%02x, len %u failed: %d", ep, buf->len, err);
	}

	/*
	 * The class allocates every buffer from its own pool, so it frees them
	 * itself: a cancelled transfer can complete after the instance has been
	 * unregistered and has no stack context left.
	 */
	if (kind == UAC1_BUF_STATUS) {
		atomic_clear(&ctx->status_busy);
		net_buf_unref(buf);
		notify_deferred_status(dev);
		return 0;
	}

	__ASSERT_NO_MSG(as_idx < cfg->num_ifaces);
	stream = &cfg->streams[as_idx];
	terminal = cfg->as_terminals[as_idx];
	/* A transfer queued before the stream last stopped belongs to an earlier session */
	status = err != 0 ? err :
		 generation == (uint32_t)atomic_get(&stream->generation) ? 0 : -ECONNABORTED;

	if (kind == UAC1_BUF_FEEDBACK) {
		outstanding_release(&stream->feedback_outstanding);
	} else {
		outstanding_release(&stream->data_outstanding);
		if (USB_EP_DIR_IS_OUT(ep)) {
			release_rx_buffer(dev, terminal, buf->__buf,
					  status == 0 ? buf->len : 0U, status);
		} else {
			release_tx_buffer(dev, terminal, buf->__buf, status);
		}
	}

	net_buf_unref(buf);
	if (status != 0) {
		return 0;
	}

	if (kind == UAC1_BUF_FEEDBACK) {
		write_explicit_feedback(c_data, as_idx);
	} else if (USB_EP_DIR_IS_OUT(ep)) {
		schedule_iso_out_read(c_data, as_idx);
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

	/* A notification that could not be sent before is retried */
	if (!atomic_get(&ctx->status_busy)) {
		notify_deferred_status(dev);
	}

	for (uint8_t as_idx = 0U; as_idx < cfg->num_ifaces; as_idx++) {
		schedule_stream(c_data, as_idx);
	}
}

static void *uac1_get_desc(struct usbd_class_data *const c_data, const enum usbd_speed speed)
{
	struct device *dev = usbd_class_get_private(c_data);
	const struct uac1_cfg *cfg = dev->config;

	if (USBD_SUPPORTS_HIGH_SPEED && speed == USBD_SPEED_HS) {
		return cfg->hs != NULL ? cfg->hs->descriptors : NULL;
	}

	return cfg->fs != NULL ? cfg->fs->descriptors : NULL;
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

		schedule_stream(c_data, as_idx);
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
	atomic_clear(&ctx->status_deferred);
	for (uint8_t as_idx = 0U; as_idx < cfg->num_ifaces; as_idx++) {
		atomic_inc(&cfg->streams[as_idx].generation);
		cfg->streams[as_idx].service_number = 0U;
	}

	uac1_notify_selected(c_data, selected, UAC1_STREAM_DISABLED);
}

/*
 * Links the descriptors that refer to each other by number to the numbers the
 * stack assigned: the AC header's AudioStreaming interfaces, and each data
 * endpoint's explicit feedback endpoint.
 */
static void uac1_link_descriptors(const struct uac1_cfg *cfg,
				  const struct uac1_speed_desc *desc)
{
	uint8_t *const interface_numbers = &desc->ac_header[AC_HEADER_INTERFACE_NUMBERS_OFFSET];

	for (uint8_t i = 0U; i < cfg->num_ifaces; i++) {
		interface_numbers[i] = desc->iad->bFirstInterface + 1U + i;

		if (desc->data_eps[i] != NULL && desc->fb_eps[i] != NULL) {
			desc->data_eps[i]->bSynchAddress = desc->fb_eps[i]->bEndpointAddress;
		}
	}
}

static void uac1_set_name_index(const struct uac1_speed_desc *desc, uint8_t idx)
{
	desc->iad->iFunction = idx;
	desc->ac_interface->iInterface = idx;
}

/*
 * The stack calls this once for each speed, after it has assigned that
 * speed's interface numbers and endpoint addresses.
 */
static int uac1_init(struct usbd_class_data *const c_data)
{
	struct usbd_context *uds_ctx = usbd_class_get_ctx(c_data);
	const struct device *dev = usbd_class_get_private(c_data);
	const struct uac1_cfg *cfg = dev->config;
	const struct uac1_speed_desc *const speeds[] = {cfg->fs, cfg->hs};
	struct uac1_ctx *ctx = dev->data;
	uint8_t name_idx = 0U;

	if (ctx->ops == NULL) {
		LOG_ERR("Application did not register UAC1 ops");
		return -EINVAL;
	}

	if (cfg->name_desc != NULL) {
		if (!usbd_str_desc_get_idx(cfg->name_desc) &&
		    usbd_add_descriptor(uds_ctx, cfg->name_desc)) {
			LOG_ERR("Failed to add function name string descriptor");
		} else {
			name_idx = usbd_str_desc_get_idx(cfg->name_desc);
		}
	}

	ARRAY_FOR_EACH(speeds, i) {
		if (speeds[i] != NULL) {
			uac1_link_descriptors(cfg, speeds[i]);
			uac1_set_name_index(speeds[i], name_idx);
		}
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

/* Expands the node before DESCRIPTOR_NAME pastes it into a name. */
#define UAC1_INSTANCE_DESCRIPTOR(prefix, node) DESCRIPTOR_NAME(prefix, node)

#define DEFINE_AS_EP_PTR(node, prefix, has_endpoint)				\
	IF_ENABLED(DT_NODE_HAS_COMPAT(node, zephyr_uac1_audio_streaming), (	\
		COND_CODE_1(has_endpoint(node),					\
			((struct uac1_ep_descriptor *)DESCRIPTOR_NAME(prefix, node),),	\
			(NULL,))						\
	))

#define DEFINE_UAC1_SPEED_DESC(inst, speed)					\
	static struct uac1_ep_descriptor *const uac1_##speed##_data_eps_##inst[] = {	\
		DT_INST_FOREACH_CHILD_STATUS_OKAY_VARGS(inst, DEFINE_AS_EP_PTR,	\
			speed##_std_data_ep, AS_HAS_ISOCHRONOUS_DATA_ENDPOINT)	\
	};									\
	static struct uac1_ep_descriptor *const uac1_##speed##_fb_eps_##inst[] = {	\
		DT_INST_FOREACH_CHILD_STATUS_OKAY_VARGS(inst, DEFINE_AS_EP_PTR,	\
			speed##_feedback_ep, AS_HAS_EXPLICIT_FEEDBACK_ENDPOINT)	\
	};									\
	static const struct uac1_speed_desc uac1_##speed##_##inst = {		\
		.descriptors = uac1_##speed##_desc_##inst,			\
		.iad = (struct usb_association_descriptor *)			\
			UAC1_INSTANCE_DESCRIPTOR(speed##_iad, DT_DRV_INST(inst)),	\
		.ac_interface = (struct usb_if_descriptor *)			\
			UAC1_INSTANCE_DESCRIPTOR(speed##_ac_interface, DT_DRV_INST(inst)),\
		.ac_header = UAC1_INSTANCE_DESCRIPTOR(speed##_ac_header, DT_DRV_INST(inst)),\
		.status_ep = COND_CODE_1(DT_INST_PROP(inst, interrupt_endpoint),	\
			((struct uac1_ep_descriptor *)UAC1_INSTANCE_DESCRIPTOR(	\
				speed##_ac_endpoint, DT_DRV_INST(inst))), (NULL)),	\
		.data_eps = uac1_##speed##_data_eps_##inst,			\
		.fb_eps = uac1_##speed##_fb_eps_##inst,				\
	};

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
		BUILD_ASSERT(FEATURE_UNIT_NUM_CHANNELS(node) < FU_MAX_VALUES,	\
			     "Feature Units support at most 31 logical channels");\
		BUILD_ASSERT(!DT_PROP(DT_PARENT(node), interrupt_endpoint) ||	\
			     ENTITY_ID(node) < ATOMIC_BITS,			\
			     "Feature Units reporting status need IDs below 32");\
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
			     ((int32_t)DT_PROP_OR(node, volume_min, 0) > INT16_MIN &&\
			      (int32_t)DT_PROP_OR(node, volume_max, 0) <= INT16_MAX &&\
			      (int32_t)DT_PROP_OR(node, volume_res, 0) <= INT16_MAX),\
			     "volume range must fit in signed 16-bit values above -infinity");\
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
		DEFINE_UAC1_SPEED_DESC(inst, fs)				\
	))									\
	IF_ENABLED(UAC1_ALLOWED_AT_HIGH_SPEED(DT_DRV_INST(inst)), (		\
		static const struct usb_desc_header *uac1_hs_desc_##inst[] =	\
			UAC1_HS_DESCRIPTOR_PTRS_ARRAY(DT_DRV_INST(inst));	\
		DEFINE_UAC1_SPEED_DESC(inst, hs)				\
	))									\
	static const uint8_t uac1_as_terminals_##inst[] = {			\
		DT_INST_FOREACH_CHILD_STATUS_OKAY(inst, DEFINE_AS_TERMINALS)	\
	};									\
	static struct uac1_stream_runtime uac1_streams_##inst[			\
		ARRAY_SIZE(uac1_as_terminals_##inst)];				\
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
	IF_ENABLED(DT_INST_NODE_HAS_PROP(inst, label), (			\
	USBD_DESC_STRING_DEFINE(uac1_name_desc_##inst,				\
				DT_INST_PROP(inst, label),			\
				USBD_DUT_STRING_INTERFACE);			\
	))									\
	static const struct uac1_cfg uac1_cfg_##inst = {			\
		.c_data = &uac1_##inst,						\
		.fs = COND_CODE_1(UAC1_ALLOWED_AT_FULL_SPEED(DT_DRV_INST(inst)),	\
				  (&uac1_fs_##inst), (NULL)),			\
		.hs = COND_CODE_1(UAC1_ALLOWED_AT_HIGH_SPEED(DT_DRV_INST(inst)),	\
				  (&uac1_hs_##inst), (NULL)),			\
		.name_desc = COND_CODE_1(DT_INST_NODE_HAS_PROP(inst, label),	\
					 (&uac1_name_desc_##inst), (NULL)),	\
		.as_terminals = uac1_as_terminals_##inst,			\
		.as_frequencies = uac1_as_freq_##inst,				\
		.as_frequencies_count = uac1_as_freq_count_##inst,		\
		.fu_cfg = uac1_fu_cfg_##inst,					\
		.streams = uac1_streams_##inst,					\
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
