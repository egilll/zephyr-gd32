/*
 * Copyright (c) 2026 Ylhyra ehf.
 * SPDX-License-Identifier: Apache-2.0
 */

/*
 * Runs two UAC1 functions behind a loopback function through the virtual bus,
 * so the stack moves every interface number and endpoint address the class
 * generated, and checks what a host sees and sends end to end.
 */

#include <string.h>

#include <zephyr/kernel.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/usb/usbd.h>
#include <zephyr/usb/usbh.h>
#include <zephyr/usb/class/usbd_uac1.h>
#include <zephyr/drivers/usb/udc.h>
#include <zephyr/ztest.h>

#include <usbh_ch9.h>
#include <usbh_device.h>

#define DUPLEX_NODE DT_NODELABEL(uac1_duplex)
#define MONO_NODE DT_NODELABEL(uac1_mono)

#define DUPLEX_PLAYBACK_TERMINAL UAC1_ENTITY_ID(DT_NODELABEL(playback_input))
#define DUPLEX_FEATURE_UNIT UAC1_ENTITY_ID(DT_NODELABEL(playback_feature))
#define DUPLEX_CAPTURE_TERMINAL UAC1_ENTITY_ID(DT_NODELABEL(capture_output))
#define MONO_PLAYBACK_TERMINAL UAC1_ENTITY_ID(DT_NODELABEL(mono_input))
#define MONO_FEATURE_UNIT UAC1_ENTITY_ID(DT_NODELABEL(mono_feature))

#define SET_CUR 0x01
#define GET_CUR 0x81
#define GET_MIN 0x82
#define GET_MAX 0x83
#define GET_RES 0x84
#define SET_CLASS_INTERFACE 0x21
#define GET_CLASS_INTERFACE 0xA1
#define MUTE_CONTROL 0x01
#define VOLUME_CONTROL 0x02
#define ALL_CHANNELS 0xFF

#define CS_INTERFACE 0x24
#define AC_HEADER 0x01
#define AUDIO 0x01
#define AUDIOCONTROL 0x01
#define AUDIOSTREAMING 0x02
#define EP_USAGE_MASK 0x30U
#define EP_USAGE_FEEDBACK 0x10U

#define BUFFER_SIZE 256
#define CAPTURE_PACKET_SIZE 192

USBD_DEVICE_DEFINE(test_usbd, DEVICE_DT_GET(DT_NODELABEL(zephyr_udc0)), 0x2fe3, 0xfffe);
USBD_DESC_LANG_DEFINE(test_lang);
USBD_CONFIGURATION_DEFINE(test_config, USB_SCD_SELF_POWERED, 100, NULL);
USBH_CONTROLLER_DEFINE(uhs_ctx, DEVICE_DT_GET(DT_NODELABEL(zephyr_uhc0)));

K_MEM_SLAB_DEFINE_STATIC(buffers, ROUND_UP(BUFFER_SIZE, UDC_BUF_GRANULARITY), 12, UDC_BUF_ALIGN);

/* What one function's application callbacks saw. */
struct function_record {
	const struct device *dev;
	uint32_t feedback;
	atomic_t acquired;
	atomic_t released;
	atomic_t aborted;
	atomic_t activated;
	atomic_t deactivated;
	atomic_t disabled;
	uint8_t terminal;
	uint8_t data[BUFFER_SIZE];
	uint16_t len;
	struct k_sem received;
	uint8_t unit;
	uint8_t selector;
	uint32_t channel_mask;
	uint8_t values[4];
};

static struct function_record records[] = {
	{.dev = DEVICE_DT_GET(DUPLEX_NODE), .feedback = 0x0C0123},
	{.dev = DEVICE_DT_GET(MONO_NODE), .feedback = 0x0C0456},
};

static struct function_record *record_of(const struct device *dev)
{
	ARRAY_FOR_EACH(records, i) {
		if (records[i].dev == dev) {
			return &records[i];
		}
	}

	zassert_unreachable("Callback from an unknown function");
	return NULL;
}

static void stream_event(const struct device *dev, uint8_t terminal,
			 enum uac1_stream_event event, bool microframes, void *user_data)
{
	struct function_record *record = record_of(dev);

	record->terminal = terminal;
	if (event == UAC1_STREAM_ACTIVATED) {
		atomic_inc(&record->activated);
	} else if (event == UAC1_STREAM_DEACTIVATED) {
		atomic_inc(&record->deactivated);
	} else if (event == UAC1_STREAM_DISABLED) {
		atomic_inc(&record->disabled);
	}
}

static int rx_acquire(const struct device *dev, uint8_t terminal, uint16_t size, void **buf,
		      void *user_data)
{
	zassert_true(size <= BUFFER_SIZE);
	if (k_mem_slab_alloc(&buffers, buf, K_NO_WAIT) != 0) {
		return -EAGAIN;
	}

	atomic_inc(&record_of(dev)->acquired);
	return 0;
}

static void rx_release(const struct device *dev, uint8_t terminal, void *buf, uint16_t size,
		       int status, void *user_data)
{
	struct function_record *record = record_of(dev);

	atomic_inc(&record->released);
	if (status == -ECONNABORTED) {
		atomic_inc(&record->aborted);
	} else if (status == 0 && size != 0U) {
		record->terminal = terminal;
		memcpy(record->data, buf, size);
		record->len = size;
		k_sem_give(&record->received);
	}

	k_mem_slab_free(&buffers, buf);
}

static int tx_acquire(const struct device *dev, uint8_t terminal, uint16_t max_size,
		      uint32_t service_number, void **buf, uint16_t *size, void *user_data)
{
	uint8_t *data;

	if (k_mem_slab_alloc(&buffers, buf, K_NO_WAIT) != 0) {
		return -EAGAIN;
	}

	data = *buf;
	for (size_t i = 0; i < CAPTURE_PACKET_SIZE; i++) {
		data[i] = (uint8_t)(terminal + i);
	}
	*size = CAPTURE_PACKET_SIZE;
	atomic_inc(&record_of(dev)->acquired);
	return 0;
}

static void tx_release(const struct device *dev, uint8_t terminal, void *buf, int status,
		       void *user_data)
{
	struct function_record *record = record_of(dev);

	atomic_inc(&record->released);
	if (status == -ECONNABORTED) {
		atomic_inc(&record->aborted);
	}

	k_mem_slab_free(&buffers, buf);
}

static uint32_t feedback(const struct device *dev, uint8_t terminal, void *user_data)
{
	return record_of(dev)->feedback;
}

static int feature_update(const struct device *dev, uint8_t unit_id, uint8_t control_selector,
			  const struct uac1_feature_update *update, void *user_data)
{
	struct function_record *record = record_of(dev);

	record->unit = unit_id;
	record->selector = control_selector;
	record->channel_mask = update->channel_mask;
	memcpy(record->values, update->values,
	       MIN(sizeof(record->values), POPCOUNT(update->channel_mask) * update->value_size));
	return 0;
}

static const struct uac1_ops ops = {
	.stream_event_cb = stream_event,
	.rx_buf_acquire = rx_acquire,
	.rx_buf_release = rx_release,
	.tx_buf_acquire = tx_acquire,
	.tx_buf_release = tx_release,
	.feedback_cb = feedback,
	.feature_update_cb = feature_update,
};

/* A function's numbers as the host reads them from the configuration descriptor. */
struct function_layout {
	uint8_t ac_interface;
	uint8_t interface_count;
	uint8_t header_interfaces[4];
	uint8_t header_count;
	uint8_t as_interfaces[2];
	uint8_t data_ep[2];
	uint8_t feedback_ep[2];
};

static uint8_t config_desc[512];
static size_t config_len;
static struct function_layout layouts[ARRAY_SIZE(records)];

static struct usb_device *host_device(void)
{
	struct usb_device *udev = usbh_device_get_any(&uhs_ctx);

	zassert_not_null(udev, "No USB device available");
	return udev;
}

static int control(uint8_t request_type, uint8_t request, uint16_t value, uint16_t index,
		   uint8_t *data, uint16_t length, size_t *received)
{
	struct usb_device *udev = host_device();
	struct net_buf *buf = NULL;
	int err;

	if (length != 0U) {
		buf = usbh_xfer_buf_alloc(udev, length);
		zassert_not_null(buf);
		if ((request_type & USB_EP_DIR_IN) == 0U) {
			net_buf_add_mem(buf, data, length);
		}
	}

	err = usbh_req_setup(udev, request_type, request, value, index, length, buf);
	if (buf != NULL) {
		if (err == 0 && (request_type & USB_EP_DIR_IN) != 0U) {
			memcpy(data, buf->data, buf->len);
			if (received != NULL) {
				*received = buf->len;
			}
		}
		usbh_xfer_buf_free(udev, buf);
	}

	return err;
}

static int set_feature(uint8_t function, uint8_t selector, uint8_t channel, const void *value,
		       uint16_t length)
{
	return control(SET_CLASS_INTERFACE, SET_CUR, (selector << 8) | channel,
		       (function == 0 ? DUPLEX_FEATURE_UNIT : MONO_FEATURE_UNIT) << 8 |
		       layouts[function].ac_interface, (uint8_t *)value, length, NULL);
}

static int get_feature(uint8_t function, uint8_t request, uint8_t selector, uint8_t channel,
		       uint8_t *value, uint16_t length, size_t *received)
{
	return control(GET_CLASS_INTERFACE, request, (selector << 8) | channel,
		       (function == 0 ? DUPLEX_FEATURE_UNIT : MONO_FEATURE_UNIT) << 8 |
		       layouts[function].ac_interface, value, length, received);
}

/* Reads the configuration descriptor and each audio function's numbers in it. */
static void read_layouts(void)
{
	struct usb_device *udev = host_device();
	struct function_layout *layout = NULL;
	struct net_buf *buf;
	size_t function = 0;
	uint8_t alternate = 0;
	uint8_t subclass = 0;
	uint8_t as_idx = 0;
	int err;

	buf = usbh_xfer_buf_alloc(udev, sizeof(config_desc));
	zassert_not_null(buf);
	err = usbh_req_desc(udev, USB_DESC_CONFIGURATION, 0, 0, sizeof(config_desc), buf);
	zassert_ok(err, "Configuration descriptor request failed");
	config_len = buf->len;
	memcpy(config_desc, buf->data, config_len);
	usbh_xfer_buf_free(udev, buf);
	zassert_equal(sys_get_le16(&config_desc[2]), config_len);

	memset(layouts, 0, sizeof(layouts));
	for (size_t offset = 0; offset < config_len; offset += config_desc[offset]) {
		const uint8_t *desc = &config_desc[offset];

		zassert_true(desc[0] >= 2U && offset + desc[0] <= config_len);
		if (desc[1] == USB_DESC_INTERFACE_ASSOC && desc[4] == AUDIO) {
			zassert_true(function < ARRAY_SIZE(layouts));
			layout = &layouts[function++];
			layout->ac_interface = desc[2];
			layout->interface_count = desc[3];
		} else if (desc[1] == USB_DESC_INTERFACE) {
			alternate = desc[3];
			subclass = desc[5] == AUDIO ? desc[6] : 0U;
			if (layout != NULL && subclass == AUDIOCONTROL) {
				zassert_equal(desc[2], layout->ac_interface,
					      "AC interface is not the IAD's first interface");
			}
			if (layout != NULL && subclass == AUDIOSTREAMING) {
				as_idx = desc[2] - layout->ac_interface - 1U;
				zassert_true(as_idx < ARRAY_SIZE(layout->as_interfaces));
				layout->as_interfaces[as_idx] = desc[2];
			}
		} else if (desc[1] == CS_INTERFACE && desc[2] == AC_HEADER && layout != NULL &&
			   subclass == AUDIOCONTROL) {
			layout->header_count = desc[7];
			memcpy(layout->header_interfaces, &desc[8], desc[7]);
		} else if (desc[1] == USB_DESC_ENDPOINT && layout != NULL &&
			   subclass == AUDIOSTREAMING && alternate == 1U) {
			const bool is_feedback = (desc[3] & EP_USAGE_MASK) == EP_USAGE_FEEDBACK;

			if (is_feedback) {
				layout->feedback_ep[as_idx] = desc[2];
				zassert_equal(desc[7], 1U, "Feedback bRefresh must be 1..9");
				zassert_equal(desc[8], 0U, "Synch endpoint bSynchAddress must be 0");
				continue;
			}

			layout->data_ep[as_idx] = desc[2];
			if (desc[8] != 0U) {
				/* The synch endpoint follows its data endpoint's CS descriptor */
				const uint8_t *cs_ep = desc + desc[0];
				const uint8_t *synch = cs_ep + cs_ep[0];

				zassert_equal(synch[1], USB_DESC_ENDPOINT);
				zassert_equal(desc[8], synch[2],
					      "bSynchAddress does not name the feedback endpoint");
			}
		}
	}

	zassert_equal(function, ARRAY_SIZE(layouts), "Expected two audio functions");
}

ZTEST(uac1_runtime, test_1_descriptors_carry_the_assigned_numbers)
{
	const struct function_layout *duplex = &layouts[0];
	const struct function_layout *mono = &layouts[1];

	/* The loopback function owns the first interfaces and endpoints. */
	zassert_true(duplex->ac_interface > 0U, "Audio functions must follow loopback");
	zassert_equal(duplex->interface_count, 3U);
	zassert_equal(mono->ac_interface, duplex->ac_interface + duplex->interface_count);
	zassert_equal(mono->interface_count, 2U);

	zassert_equal(duplex->header_count, 2U);
	zassert_equal(duplex->header_interfaces[0], duplex->as_interfaces[0]);
	zassert_equal(duplex->header_interfaces[1], duplex->as_interfaces[1]);
	zassert_equal(mono->header_count, 1U);
	zassert_equal(mono->header_interfaces[0], mono->ac_interface + 1U);
	zassert_equal(mono->as_interfaces[0], mono->ac_interface + 1U);

	zassert_not_equal(duplex->data_ep[0], 0x01U, "Loopback owns endpoint 0x01");
	zassert_not_equal(duplex->feedback_ep[0], 0U);
	zassert_not_equal(mono->data_ep[0], 0U);
	zassert_not_equal(mono->feedback_ep[0], 0U);
	zassert_not_equal(mono->data_ep[0], duplex->data_ep[0]);
	zassert_not_equal(mono->feedback_ep[0], duplex->feedback_ep[0]);
	zassert_not_equal(mono->feedback_ep[0], duplex->data_ep[1]);
}

ZTEST(uac1_runtime, test_2_feature_unit_mute_and_volume_are_independent)
{
	uint8_t value[4];
	size_t received = 0;
	uint8_t mute = 1;
	uint8_t volume[2];

	zassert_ok(set_feature(0, MUTE_CONTROL, 0, &mute, 1));
	zassert_equal(records[0].unit, DUPLEX_FEATURE_UNIT);
	zassert_equal(records[0].channel_mask, BIT(0));

	sys_put_le16((uint16_t)(-10 * 256), volume);
	zassert_ok(set_feature(0, VOLUME_CONTROL, 0, volume, 2));
	zassert_ok(get_feature(0, GET_CUR, MUTE_CONTROL, 0, value, 1, &received));
	zassert_equal(value[0], 1U, "A volume change must not unmute");
	zassert_ok(get_feature(0, GET_CUR, VOLUME_CONTROL, 0, value, 2, &received));
	zassert_equal((int16_t)sys_get_le16(value), -10 * 256);

	/* -infinity dB is kept, and other values round to the resolution */
	sys_put_le16(0x8000U, volume);
	zassert_ok(set_feature(0, VOLUME_CONTROL, 0, volume, 2));
	zassert_ok(get_feature(0, GET_CUR, VOLUME_CONTROL, 0, value, 2, &received));
	zassert_equal(sys_get_le16(value), 0x8000U);
	sys_put_le16((uint16_t)(-10 * 256 - 100), volume);
	zassert_ok(set_feature(0, VOLUME_CONTROL, 0, volume, 2));
	zassert_ok(get_feature(0, GET_CUR, VOLUME_CONTROL, 0, value, 2, &received));
	zassert_equal((int16_t)sys_get_le16(value), -10 * 256);

	zassert_ok(get_feature(0, GET_RES, VOLUME_CONTROL, 0, value, 2, &received));
	zassert_equal(sys_get_le16(value), 256U);
	zassert_ok(get_feature(0, GET_MIN, VOLUME_CONTROL, 0, value, 2, &received));
	zassert_equal((int16_t)sys_get_le16(value), -90 * 256);
	zassert_ok(get_feature(0, GET_MAX, VOLUME_CONTROL, 0, value, 2, &received));
	zassert_equal((int16_t)sys_get_le16(value), 0);

	/* A short request gets the start of the value, not an overrun */
	zassert_ok(get_feature(0, GET_CUR, VOLUME_CONTROL, 0, value, 1, &received));
	zassert_equal(received, 1U);
	zassert_equal(value[0], 0x00U);

	/* All channels: the master and channel 1 have mute, channel 2 does not */
	zassert_ok(get_feature(0, GET_CUR, MUTE_CONTROL, ALL_CHANNELS, value, 2, &received));
	zassert_equal(received, 2U);
	zassert_equal(value[0], 1U);
	zassert_equal(value[1], 0U);

	zassert_not_ok(set_feature(0, MUTE_CONTROL, 2, &mute, 1), "Channel 2 has no mute");
	zassert_not_ok(get_feature(0, GET_MIN, MUTE_CONTROL, 0, value, 1, &received));
	zassert_not_ok(set_feature(0, MUTE_CONTROL, 0, volume, 2), "Mute is one byte");

	/* The second function answers on its own interface */
	mute = 0;
	zassert_ok(set_feature(1, MUTE_CONTROL, 0, &mute, 1));
	zassert_equal(records[1].unit, MONO_FEATURE_UNIT);
	zassert_ok(get_feature(1, GET_RES, VOLUME_CONTROL, 0, value, 2, &received));
	zassert_equal(sys_get_le16(value), 26U);
}

static int transfer_done(struct usb_device *const udev, struct uhc_transfer *const xfer)
{
	k_sem_give((struct k_sem *)xfer->priv);
	return 0;
}

static int transfer(uint8_t ep, uint8_t *data, size_t length, size_t *received)
{
	struct usb_device *udev = host_device();
	struct k_sem done;
	struct uhc_transfer *xfer;
	struct net_buf *buf;
	int err;

	k_sem_init(&done, 0, 1);
	xfer = usbh_xfer_alloc(udev, ep, transfer_done, &done);
	zassert_not_null(xfer, "Endpoint 0x%02x is not enabled", ep);
	buf = usbh_xfer_buf_alloc(udev, length);
	zassert_not_null(buf);
	if (USB_EP_DIR_IS_OUT(ep)) {
		net_buf_add_mem(buf, data, length);
	}
	zassert_ok(usbh_xfer_buf_add(udev, xfer, buf));
	zassert_ok(usbh_xfer_enqueue(udev, xfer));

	err = k_sem_take(&done, K_MSEC(500));
	if (err != 0) {
		(void)usbh_xfer_dequeue(udev, xfer);
		return err;
	}

	err = xfer->err;
	if (err == 0 && USB_EP_DIR_IS_IN(ep)) {
		memcpy(data, buf->data, buf->len);
		*received = buf->len;
	}
	usbh_xfer_buf_free(udev, buf);
	usbh_xfer_free(udev, xfer);

	return err;
}

static void select_alternate(uint8_t interface, uint8_t alternate)
{
	zassert_ok(usbh_device_interface_set(host_device(), interface, alternate, false));
	/* Let SOFs queue the stream's transfers */
	k_msleep(20);
}

static void assert_all_buffers_returned(const struct function_record *record)
{
	k_msleep(20);
	zassert_equal(atomic_get(&record->acquired), atomic_get(&record->released),
		      "%ld buffers acquired, %ld released", atomic_get(&record->acquired),
		      atomic_get(&record->released));
}

ZTEST(uac1_runtime, test_3_playback_reaches_its_own_function)
{
	struct function_record *mono = &records[1];
	struct function_record *duplex = &records[0];
	const struct function_layout *layout = &layouts[1];
	uint8_t packet[98];
	uint8_t fb[3];
	size_t received = 0;

	for (size_t i = 0; i < sizeof(packet); i++) {
		packet[i] = (uint8_t)(0xA0 + i);
	}

	select_alternate(layout->as_interfaces[0], 1);
	zassert_equal(atomic_get(&mono->activated), 1);
	zassert_equal(mono->terminal, MONO_PLAYBACK_TERMINAL);
	zassert_equal(atomic_get(&duplex->activated), 0);

	zassert_ok(transfer(layout->data_ep[0], packet, sizeof(packet), NULL));
	zassert_ok(k_sem_take(&mono->received, K_MSEC(200)), "Mono function got no audio");
	zassert_equal(mono->terminal, MONO_PLAYBACK_TERMINAL);
	zassert_equal(mono->len, sizeof(packet));
	zassert_mem_equal(mono->data, packet, sizeof(packet));
	zassert_not_ok(k_sem_take(&duplex->received, K_NO_WAIT),
		       "Audio for the mono function reached the duplex function");

	zassert_ok(transfer(layout->feedback_ep[0], fb, sizeof(fb), &received));
	zassert_equal(received, 3U);
	zassert_equal(sys_get_le24(fb), mono->feedback);

	select_alternate(layout->as_interfaces[0], 0);
	zassert_equal(atomic_get(&mono->deactivated), 1);
	assert_all_buffers_returned(mono);
	zassert_true(atomic_get(&mono->aborted) > 0, "Queued buffers must be cancelled");
}

ZTEST(uac1_runtime, test_4_capture_sends_application_packets)
{
	struct function_record *duplex = &records[0];
	const struct function_layout *layout = &layouts[0];
	uint8_t packet[196];
	size_t received = 0;

	select_alternate(layout->as_interfaces[1], 1);
	zassert_equal(duplex->terminal, DUPLEX_CAPTURE_TERMINAL);
	zassert_ok(transfer(layout->data_ep[1], packet, sizeof(packet), &received));
	zassert_equal(received, CAPTURE_PACKET_SIZE);
	for (size_t i = 0; i < received; i++) {
		zassert_equal(packet[i], (uint8_t)(DUPLEX_CAPTURE_TERMINAL + i));
	}

	select_alternate(layout->as_interfaces[1], 0);
	assert_all_buffers_returned(duplex);
}

ZTEST(uac1_runtime, test_5_shutdown_while_streaming_returns_every_buffer)
{
	struct function_record *duplex = &records[0];
	struct function_record *mono = &records[1];
	const struct function_layout *layout = &layouts[0];
	uint8_t packet[196] = {0};
	uint8_t fb[3];
	size_t received = 0;

	select_alternate(layout->as_interfaces[0], 1);
	select_alternate(layouts[1].as_interfaces[0], 1);
	zassert_ok(transfer(layout->data_ep[0], packet, sizeof(packet), NULL));
	zassert_ok(k_sem_take(&duplex->received, K_MSEC(200)));
	zassert_equal(duplex->terminal, DUPLEX_PLAYBACK_TERMINAL);
	zassert_ok(transfer(layout->feedback_ep[0], fb, sizeof(fb), &received));
	zassert_equal(sys_get_le24(fb), duplex->feedback);
	zassert_true(atomic_get(&mono->acquired) > atomic_get(&mono->released));

	/*
	 * As a change of functions does. A host reset takes the same path through
	 * the configuration reset; transfers still in flight complete after the
	 * functions are unregistered.
	 */
	zassert_ok(usbd_disable(&test_usbd));
	zassert_ok(usbd_shutdown(&test_usbd));
	k_msleep(50);
	zassert_equal(atomic_get(&duplex->disabled), 1);
	zassert_equal(atomic_get(&mono->disabled), 1);
	assert_all_buffers_returned(duplex);
	assert_all_buffers_returned(mono);
	zassert_equal(k_mem_slab_num_used_get(&buffers), 0U, "Buffers leaked");
}

static void *uac1_runtime_setup(void)
{
	ARRAY_FOR_EACH(records, i) {
		k_sem_init(&records[i].received, 0, 1);
		zassert_ok(usbd_uac1_set_ops(records[i].dev, &ops, NULL));
	}

	zassert_ok(usbh_init(&uhs_ctx));
	zassert_ok(usbh_enable(&uhs_ctx));
	zassert_ok(uhc_bus_reset(uhs_ctx.dev));
	zassert_ok(uhc_bus_resume(uhs_ctx.dev));
	zassert_ok(uhc_sof_enable(uhs_ctx.dev));

	zassert_ok(usbd_add_descriptor(&test_usbd, &test_lang));
	zassert_ok(usbd_add_configuration(&test_usbd, USBD_SPEED_FS, &test_config));
	zassert_ok(usbd_register_class(&test_usbd, "loopback_0", USBD_SPEED_FS, 1));
	ARRAY_FOR_EACH(records, i) {
		zassert_ok(usbd_register_class(&test_usbd, usbd_uac1_class_name(records[i].dev),
					       USBD_SPEED_FS, 1));
	}
	usbd_device_set_code_triple(&test_usbd, USBD_SPEED_FS, USB_BCC_MISCELLANEOUS, 0x02, 0x01);
	zassert_ok(usbd_init(&test_usbd));
	zassert_ok(usbd_enable(&test_usbd));

	/* Allow the host time to reset and configure the device. */
	k_msleep(200);
	read_layouts();

	return NULL;
}

ZTEST_SUITE(uac1_runtime, NULL, uac1_runtime_setup, NULL, NULL, NULL);
