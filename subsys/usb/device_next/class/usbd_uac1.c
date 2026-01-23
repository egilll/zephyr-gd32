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

UDC_BUF_POOL_DEFINE(uac1_pool, UAC1_NUM_EP_BUFFERS, 6,
		    sizeof(struct udc_buf_info), NULL);

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

struct uac1_fu_cfg {
	uint8_t unit_id;
	uint16_t controls;
	int16_t vol_min;
	int16_t vol_max;
	int16_t vol_res;
};

struct uac1_fu_state {
	uint8_t unit_id;
	uint8_t mute;
	int16_t volume;
};

struct uac1_ctx {
	const struct uac1_ops *ops;
	void *user_data;

	atomic_t as_active;
	atomic_t as_queued;
	atomic_t as_double;
	uint32_t fb_queued;
	uint32_t fb_double;

	struct uac1_fu_state *fu_state;
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
	uint8_t num_ifaces;
	uint8_t num_fu;
};

static const struct uac1_fu_cfg *find_fu_cfg(const struct uac1_cfg *cfg, uint8_t unit_id)
{
	for (uint8_t i = 0; i < cfg->num_fu; i++) {
		if (cfg->fu_cfg[i].unit_id == unit_id) {
			return &cfg->fu_cfg[i];
		}
	}

	return NULL;
}

static struct uac1_fu_state *find_fu_state(const struct device *dev, uint8_t unit_id)
{
	const struct uac1_cfg *cfg = dev->config;
	struct uac1_ctx *ctx = dev->data;

	for (uint8_t i = 0; i < cfg->num_fu; i++) {
		if (ctx->fu_state[i].unit_id == unit_id) {
			return &ctx->fu_state[i];
		}
	}

	return NULL;
}

static int terminal_to_as_interface(const struct device *dev, uint8_t terminal)
{
	const struct uac1_cfg *cfg = dev->config;

	for (int as_idx = 0; as_idx < cfg->num_ifaces; as_idx++) {
		if (terminal == cfg->as_terminals[as_idx]) {
			return as_idx;
		}
	}

	return -ENOENT;
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

	if (!data_eps || !fb_eps) {
		*is_feedback = false;
		return -ENOENT;
	}

	for (int i = 0; i < cfg->num_ifaces; i++) {
		if (data_eps[i] && data_eps[i] == ep) {
			*is_feedback = false;
			return i;
		}

		if (fb_eps[i] && fb_eps[i] == ep) {
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

static struct net_buf *uac1_buf_alloc(const uint8_t ep, void *data, uint16_t size)
{
	struct net_buf *buf;
	struct udc_buf_info *bi;

	__ASSERT(IS_UDC_ALIGNED(data), "Application provided unaligned buffer");

	buf = net_buf_alloc_with_data(&uac1_pool, data, size, K_NO_WAIT);
	if (!buf) {
		return NULL;
	}

	bi = udc_get_buf_info(buf);
	bi->ep = ep;

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

static void schedule_iso_out_read(struct usbd_class_data *const c_data,
				  uint8_t ep, uint16_t mps, uint8_t terminal)
{
	const struct device *dev = usbd_class_get_private(c_data);
	const struct uac1_cfg *cfg = dev->config;
	struct uac1_ctx *ctx = dev->data;
	struct net_buf *buf;
	atomic_t *queued_bits = &ctx->as_queued;
	void *data_buf;
	int as_idx = terminal_to_as_interface(dev, terminal);
	int ret;

	if ((as_idx < 0) || (as_idx >= cfg->num_ifaces) ||
	    !atomic_test_bit(&ctx->as_active, as_idx)) {
		return;
	}

	if (atomic_test_and_set_bit(queued_bits, as_idx)) {
		queued_bits = &ctx->as_double;
		if (atomic_test_and_set_bit(queued_bits, as_idx)) {
			return;
		}
	}

	data_buf = ctx->ops->get_recv_buf(dev, terminal, mps, ctx->user_data);
	if (!data_buf) {
		LOG_ERR_RATELIMIT("No data buffer for terminal %d", terminal);
		atomic_clear_bit(queued_bits, as_idx);
		return;
	}

	buf = uac1_buf_alloc(ep, data_buf, mps);
	if (!buf) {
		LOG_ERR("No net_buf for read");
		ctx->ops->data_recv_cb(dev, terminal, data_buf, 0, ctx->user_data);
		atomic_clear_bit(queued_bits, as_idx);
		return;
	}

	ret = usbd_ep_enqueue(c_data, buf);
	if (ret) {
		LOG_ERR("Failed to enqueue net_buf for 0x%02x", ep);
		net_buf_unref(buf);
		ctx->ops->data_recv_cb(dev, terminal, data_buf, 0, ctx->user_data);
		atomic_clear_bit(queued_bits, as_idx);
	}
}

static void write_explicit_feedback(struct usbd_class_data *const c_data,
				    uint8_t ep, uint8_t terminal)
{
	const struct device *dev = usbd_class_get_private(c_data);
	struct usbd_context *uds_ctx = usbd_class_get_ctx(c_data);
	struct uac1_ctx *ctx = dev->data;
	struct net_buf *buf;
	struct udc_buf_info *bi;
	uint32_t fb_value;
	int as_idx = terminal_to_as_interface(dev, terminal);
	int ret;

	__ASSERT_NO_MSG(as_idx >= 0);

	buf = net_buf_alloc(&uac1_pool, K_NO_WAIT);
	if (!buf) {
		LOG_ERR("No buf for feedback");
		return;
	}

	bi = udc_get_buf_info(buf);
	bi->ep = ep;

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
		return;
	}

	if (ctx->fb_queued & BIT(as_idx)) {
		ctx->fb_double |= BIT(as_idx);
	} else {
		ctx->fb_queued |= BIT(as_idx);
	}
}

void usbd_uac1_set_ops(const struct device *dev, const struct uac1_ops *ops, void *user_data)
{
	const struct uac1_cfg *cfg = dev->config;
	struct uac1_ctx *ctx = dev->data;

	__ASSERT(ops->sof_cb, "SOF callback is mandatory");
	__ASSERT(ops->terminal_update_cb, "terminal_update_cb is mandatory");

	for (uint8_t i = 0U; i < cfg->num_ifaces; i++) {
		const uint8_t ep_fs = cfg->fs_data_ep ? cfg->fs_data_ep[i] : 0U;
		const uint8_t ep_hs = cfg->hs_data_ep ? cfg->hs_data_ep[i] : 0U;
		const uint8_t ep = ep_fs ? ep_fs : ep_hs;

		if (cfg->fs_fb_ep && cfg->fs_fb_ep[i]) {
			__ASSERT(ops->feedback_cb, "feedback_cb is mandatory");
		}
		if (cfg->hs_fb_ep && cfg->hs_fb_ep[i]) {
			__ASSERT(ops->feedback_cb, "feedback_cb is mandatory");
		}

		if (ep) {
			if (USB_EP_DIR_IS_OUT(ep)) {
				__ASSERT(ops->get_recv_buf, "get_recv_buf is mandatory");
				__ASSERT(ops->data_recv_cb, "data_recv_cb is mandatory");
			}
			if (USB_EP_DIR_IS_IN(ep)) {
				__ASSERT(ops->buf_release_cb, "buf_release_cb is mandatory");
			}
		}
	}

	ctx->ops = ops;
	ctx->user_data = user_data;
}

int usbd_uac1_send(const struct device *dev, uint8_t terminal, void *data, uint16_t size)
{
	const struct uac1_cfg *cfg = dev->config;
	struct uac1_ctx *ctx = dev->data;
	struct net_buf *buf;
	atomic_t *queued_bits = &ctx->as_queued;
	uint8_t ep;
	int as_idx = terminal_to_as_interface(dev, terminal);
	int ret;

	if (as_idx < 0) {
		return as_idx;
	}

	ep = as_data_ep(dev, as_idx);
	if (!ep) {
		LOG_ERR("No endpoint for terminal %d", terminal);
		return -ENOENT;
	}

	if (!atomic_test_bit(&ctx->as_active, as_idx)) {
		ctx->ops->buf_release_cb(dev, terminal, data, ctx->user_data);
		return 0;
	}

	if (atomic_test_and_set_bit(queued_bits, as_idx)) {
		queued_bits = &ctx->as_double;
		if (atomic_test_and_set_bit(queued_bits, as_idx)) {
			LOG_DBG("Already double queued on 0x%02x", ep);
			return -EAGAIN;
		}
	}

	buf = uac1_buf_alloc(ep, data, size);
	if (!buf) {
		LOG_ERR("No netbuf for send");
		atomic_clear_bit(queued_bits, as_idx);
		return -ENOMEM;
	}

	ret = usbd_ep_enqueue(cfg->c_data, buf);
	if (ret) {
		LOG_ERR("Failed to enqueue net_buf for 0x%02x", ep);
		net_buf_unref(buf);
		atomic_clear_bit(queued_bits, as_idx);
	}

	return ret;
}

static int handle_fu_get(const struct device *dev, const struct usb_setup_packet *setup,
			 struct net_buf *buf)
{
	const struct uac1_cfg *cfg = dev->config;
	const uint8_t unit_id = CONTROL_ENTITY_ID(setup);
	const struct uac1_fu_cfg *fu_cfg;
	struct uac1_fu_state *fu_state;
	const uint8_t cs = CONTROL_SELECTOR(setup);

	fu_cfg = find_fu_cfg(cfg, unit_id);
	fu_state = find_fu_state(dev, unit_id);
	if (!fu_cfg || !fu_state) {
		errno = -ENOTSUP;
		return 0;
	}

	if (cs == UAC1_FU_MUTE_CONTROL) {
		if (!(fu_cfg->controls & FU_CTRL_MUTE_BIT) ||
		    CONTROL_ATTRIBUTE(setup) != GET_CUR) {
			errno = -ENOTSUP;
			return 0;
		}

		net_buf_add_u8(buf, fu_state->mute ? 1 : 0);
		return 0;
	}

	if (cs == UAC1_FU_VOLUME_CONTROL) {
		if (!(fu_cfg->controls & FU_CTRL_VOLUME_BIT)) {
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

	errno = -ENOTSUP;
	return 0;
}

static int handle_fu_set(const struct device *dev, const struct usb_setup_packet *setup,
			 const struct net_buf *buf)
{
	const struct uac1_cfg *cfg = dev->config;
	struct uac1_ctx *ctx = dev->data;
	const uint8_t unit_id = CONTROL_ENTITY_ID(setup);
	const struct uac1_fu_cfg *fu_cfg;
	struct uac1_fu_state *fu_state;
	const uint8_t cs = CONTROL_SELECTOR(setup);
	const uint8_t channel = CONTROL_CHANNEL_NUMBER(setup);

	fu_cfg = find_fu_cfg(cfg, unit_id);
	fu_state = find_fu_state(dev, unit_id);
	if (!fu_cfg || !fu_state) {
		errno = -ENOTSUP;
		return 0;
	}

	if (cs == UAC1_FU_MUTE_CONTROL) {
		if (!(fu_cfg->controls & FU_CTRL_MUTE_BIT) ||
		    CONTROL_ATTRIBUTE(setup) != SET_CUR || buf->len != 1) {
			errno = -EINVAL;
			return 0;
		}

		fu_state->mute = buf->data[0] ? 1 : 0;
		if (ctx->ops->feature_update_cb) {
			ctx->ops->feature_update_cb(dev, unit_id, cs, channel,
						    &fu_state->mute, sizeof(fu_state->mute),
						    ctx->user_data);
		}
		return 0;
	}

	if (cs == UAC1_FU_VOLUME_CONTROL) {
		int16_t vol;

		if (!(fu_cfg->controls & FU_CTRL_VOLUME_BIT) ||
		    CONTROL_ATTRIBUTE(setup) != SET_CUR || buf->len != 2) {
			errno = -EINVAL;
			return 0;
		}

		vol = (int16_t)sys_get_le16(buf->data);
		if (vol < fu_cfg->vol_min) {
			vol = fu_cfg->vol_min;
		}
		if (vol > fu_cfg->vol_max) {
			vol = fu_cfg->vol_max;
		}
		fu_state->volume = vol;

		if (ctx->ops->feature_update_cb) {
			ctx->ops->feature_update_cb(dev, unit_id, cs, channel,
						    &fu_state->volume, sizeof(fu_state->volume),
						    ctx->user_data);
		}

		return 0;
	}

	errno = -ENOTSUP;
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

	if (setup->bmRequestType == SET_CLASS_INTERFACE_REQUEST_TYPE) {
		if (CONTROL_ATTRIBUTE(setup) == SET_CUR) {
			return handle_fu_set(dev, setup, buf);
		}
	} else if (setup->bmRequestType == SET_CLASS_ENDPOINT_REQUEST_TYPE) {
		return handle_ep_sample_rate_set(dev, setup, buf);
	}

	errno = -ENOTSUP;
	return 0;
}

static int uac1_control_to_host(struct usbd_class_data *const c_data,
				const struct usb_setup_packet *const setup,
				struct net_buf *const buf)
{
	const struct device *dev = usbd_class_get_private(c_data);

	if (setup->bmRequestType == GET_CLASS_INTERFACE_REQUEST_TYPE) {
		return handle_fu_get(dev, setup, buf);
	} else if (setup->bmRequestType == GET_CLASS_ENDPOINT_REQUEST_TYPE) {
		return handle_ep_sample_rate_get(dev, setup, buf);
	}

	errno = -ENOTSUP;
	return 0;
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
	uint8_t terminal;
	uint8_t data_ep;
	uint8_t fb_ep;
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
	__ASSERT_NO_MSG((iface > iad->bFirstInterface) &&
			(iface < iad->bFirstInterface + iad->bInterfaceCount));
	as_idx = iface - iad->bFirstInterface - 1;

	terminal = cfg->as_terminals[as_idx];
	ctx->ops->terminal_update_cb(dev, terminal, alternate, microframes, ctx->user_data);

	if (alternate == 0) {
		atomic_clear_bit(&ctx->as_active, as_idx);
		return;
	}

	atomic_set_bit(&ctx->as_active, as_idx);

	data_ep = as_data_ep(dev, as_idx);
	if (!data_ep) {
		return;
	}

	if (USB_EP_DIR_IS_OUT(data_ep)) {
		schedule_iso_out_read(c_data, data_ep, as_data_mps(dev, as_idx), terminal);

		fb_ep = as_feedback_ep(dev, as_idx);
		if (fb_ep) {
			write_explicit_feedback(c_data, fb_ep, terminal);
		}
	}
}

static int uac1_request(struct usbd_class_data *const c_data, struct net_buf *buf, int err)
{
	const struct device *dev = usbd_class_get_private(c_data);
	const struct uac1_cfg *cfg = dev->config;
	struct uac1_ctx *ctx = dev->data;
	struct usbd_context *uds_ctx = usbd_class_get_ctx(c_data);
	struct udc_buf_info *bi;
	uint8_t ep;
	uint8_t terminal;
	uint16_t mps;
	int as_idx;
	bool is_feedback;

	bi = udc_get_buf_info(buf);
	if (err) {
		if (err == -ECONNABORTED) {
			LOG_WRN("request ep 0x%02x, len %u cancelled", bi->ep, buf->len);
		} else {
			LOG_ERR("request ep 0x%02x, len %u failed", bi->ep, buf->len);
		}
	}

	mps = buf->size;
	ep = bi->ep;
	as_idx = ep_to_as_interface(dev, ep, &is_feedback);
	__ASSERT_NO_MSG((as_idx >= 0) && (as_idx < cfg->num_ifaces));
	terminal = cfg->as_terminals[as_idx];

	if (is_feedback) {
		bool clear_double = buf->frags;

		if (ctx->fb_queued & BIT(as_idx)) {
			ctx->fb_queued &= ~BIT(as_idx);
		} else {
			clear_double = true;
		}

		if (clear_double) {
			ctx->fb_double &= ~BIT(as_idx);
		}
	} else if (!atomic_test_and_clear_bit(&ctx->as_queued, as_idx) || buf->frags) {
		atomic_clear_bit(&ctx->as_double, as_idx);
	}

	if (USB_EP_DIR_IS_OUT(ep)) {
		ctx->ops->data_recv_cb(dev, terminal, buf->__buf, buf->len, ctx->user_data);
		if (buf->frags) {
			ctx->ops->data_recv_cb(dev, terminal, buf->frags->__buf,
					       buf->frags->len, ctx->user_data);
		}
	} else if (!is_feedback) {
		ctx->ops->buf_release_cb(dev, terminal, buf->__buf, ctx->user_data);
		if (buf->frags) {
			ctx->ops->buf_release_cb(dev, terminal, buf->frags->__buf, ctx->user_data);
		}
	}

	usbd_ep_buf_free(uds_ctx, buf);
	if (err) {
		return 0;
	}

	if (USB_EP_DIR_IS_OUT(ep)) {
		schedule_iso_out_read(c_data, ep, mps, terminal);
	} else if (is_feedback) {
		write_explicit_feedback(c_data, ep, terminal);
	}

	return 0;
}

static void uac1_sof(struct usbd_class_data *const c_data)
{
	const struct device *dev = usbd_class_get_private(c_data);
	const struct uac1_cfg *cfg = dev->config;
	struct uac1_ctx *ctx = dev->data;

	ctx->ops->sof_cb(dev, ctx->user_data);

	for (int as_idx = 0; as_idx < cfg->num_ifaces; as_idx++) {
		const uint8_t data_ep = as_data_ep(dev, as_idx);
		const uint8_t fb_ep = as_feedback_ep(dev, as_idx);

		if (data_ep && USB_EP_DIR_IS_OUT(data_ep)) {
			schedule_iso_out_read(c_data, data_ep, as_data_mps(dev, as_idx),
					      cfg->as_terminals[as_idx]);
		}

		if (!fb_ep) {
			continue;
		}

		if (ctx->fb_queued & ctx->fb_double & BIT(as_idx)) {
			continue;
		}

		if (!atomic_test_bit(&ctx->as_active, as_idx)) {
			continue;
		}

		write_explicit_feedback(c_data, fb_ep, cfg->as_terminals[as_idx]);
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
	.sof = uac1_sof,
	.get_desc = uac1_get_desc,
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

#define DEFINE_FU_CFG(node)							\
	IF_ENABLED(DT_NODE_HAS_COMPAT(node, zephyr_uac1_feature_unit), (	\
		{								\
			.unit_id = ENTITY_ID(node),				\
			.controls = (uint16_t)DT_PROP(node, controls),		\
			.vol_min = (int16_t)DT_PROP_OR(node, volume_min, 0),	\
			.vol_max = (int16_t)DT_PROP_OR(node, volume_max, 0),	\
			.vol_res = (int16_t)DT_PROP_OR(node, volume_res, 0),	\
		},								\
	))

#define DEFINE_FU_STATE(node)							\
	IF_ENABLED(DT_NODE_HAS_COMPAT(node, zephyr_uac1_feature_unit), (	\
		{								\
			.unit_id = ENTITY_ID(node),				\
			.mute = 0,						\
			.volume = 0,						\
		},								\
	))

#define COUNT_FU(node)								\
	IF_ENABLED(DT_NODE_HAS_COMPAT(node, zephyr_uac1_feature_unit), (+ 1))

#define DEFINE_UAC1_CLASS_DATA(inst)						\
	BUILD_ASSERT(UAC1_ALLOWED_AT_FULL_SPEED(DT_DRV_INST(inst)) ||		\
		     UAC1_ALLOWED_AT_HIGH_SPEED(DT_DRV_INST(inst)),		\
		     "UAC1 instance must enable at least one speed");		\
	static struct uac1_fu_state uac1_fu_state_##inst[] = {			\
		DT_INST_FOREACH_CHILD_STATUS_OKAY(inst, DEFINE_FU_STATE)	\
		{ .unit_id = 0 },						\
	};									\
	static struct uac1_ctx uac1_ctx_##inst = {				\
		.fu_state = uac1_fu_state_##inst,				\
	};									\
	UAC1_DESCRIPTOR_ARRAYS(DT_DRV_INST(inst))				\
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
