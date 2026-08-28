/*
 * Copyright (c) 2026 Ylhyra ehf.
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/sys/byteorder.h>
#include <zephyr/usb/usb_ch9.h>
#include <zephyr/ztest.h>

#include <usbd_uac1_macros.h>

#define UAC1_NODE DT_NODELABEL(uac1_duplex)

UAC1_DESCRIPTOR_ARRAYS(UAC1_NODE)

static const struct usb_desc_header *const fs_descriptors[] =
	UAC1_FS_DESCRIPTOR_PTRS_ARRAY(UAC1_NODE);
static const struct usb_desc_header *const hs_descriptors[] =
	UAC1_HS_DESCRIPTOR_PTRS_ARRAY(UAC1_NODE);

static const struct usb_desc_header *find_interface(
	const struct usb_desc_header *const *descriptors, uint8_t interface, uint8_t alternate)
{
	for (size_t i = 0U; descriptors[i] != NULL; i++) {
		const struct usb_desc_header *header = descriptors[i];

		if (header->bDescriptorType != USB_DESC_INTERFACE) {
			continue;
		}

		const struct usb_if_descriptor *desc =
			(const struct usb_if_descriptor *)header;
		if (desc->bInterfaceNumber == interface && desc->bAlternateSetting == alternate) {
			return header;
		}
	}

	return NULL;
}

static const struct usb_ep_descriptor *find_endpoint(
	const struct usb_desc_header *const *descriptors, uint8_t address)
{
	for (size_t i = 0U; descriptors[i] != NULL; i++) {
		const struct usb_desc_header *header = descriptors[i];

		if (header->bDescriptorType == USB_DESC_ENDPOINT) {
			const struct usb_ep_descriptor *desc =
				(const struct usb_ep_descriptor *)header;

			if (desc->bEndpointAddress == address) {
				return desc;
			}
		}
	}

	return NULL;
}

ZTEST(uac1_descriptors, test_duplex_interface_and_endpoint_assignment)
{
	const struct usb_association_descriptor *iad =
		(const struct usb_association_descriptor *)fs_descriptors[0];

	zassert_equal(iad->bFirstInterface, 0U);
	zassert_equal(iad->bInterfaceCount, 3U);
	zassert_not_null(find_interface(fs_descriptors, 0U, 0U));
	zassert_not_null(find_interface(fs_descriptors, 1U, 0U));
	zassert_not_null(find_interface(fs_descriptors, 1U, 1U));
	zassert_not_null(find_interface(fs_descriptors, 2U, 0U));
	zassert_not_null(find_interface(fs_descriptors, 2U, 1U));
	zassert_is_null(find_interface(fs_descriptors, 0U, 1U));

	const struct usb_ep_descriptor *status = find_endpoint(fs_descriptors, 0x81U);
	const struct usb_ep_descriptor *playback = find_endpoint(fs_descriptors, 0x01U);
	const struct usb_ep_descriptor *feedback = find_endpoint(fs_descriptors, 0x82U);
	const struct usb_ep_descriptor *capture = find_endpoint(fs_descriptors, 0x83U);

	zassert_not_null(status);
	zassert_not_null(playback);
	zassert_not_null(feedback);
	zassert_not_null(capture);
	zassert_equal(sys_le16_to_cpu(playback->wMaxPacketSize), 196U);
	zassert_equal(sys_le16_to_cpu(capture->wMaxPacketSize), 196U);
	zassert_equal(((const uint8_t *)playback)[8], feedback->bEndpointAddress);
	zassert_equal(((const uint8_t *)feedback)[7], 0U);
}

ZTEST(uac1_descriptors, test_high_speed_feedback_and_polling_encoding)
{
	const struct usb_ep_descriptor *playback = find_endpoint(hs_descriptors, 0x01U);
	const struct usb_ep_descriptor *feedback = find_endpoint(hs_descriptors, 0x82U);
	const struct usb_ep_descriptor *capture = find_endpoint(hs_descriptors, 0x83U);

	zassert_not_null(playback);
	zassert_not_null(feedback);
	zassert_not_null(capture);
	zassert_equal(playback->bInterval, 4U);
	zassert_equal(capture->bInterval, 4U);
	zassert_equal(feedback->bInterval, 4U);
	zassert_equal(sys_le16_to_cpu(feedback->wMaxPacketSize), 4U);
}

ZTEST(uac1_descriptors, test_sparse_feature_controls_and_frequency_list)
{
	const uint8_t *feature = (const uint8_t *)fs_descriptors[4];
	const uint8_t *format = (const uint8_t *)fs_descriptors[12];

	zassert_equal(feature[0], 10U);
	zassert_equal(feature[5], 1U);
	zassert_equal(feature[6], 0x03U);
	zassert_equal(feature[7], 0x01U);
	zassert_equal(feature[8], 0x02U);

	zassert_equal(format[0], 14U);
	zassert_equal(format[7], 2U);
	zassert_equal(sys_get_le24(&format[8]), 44100U);
	zassert_equal(sys_get_le24(&format[11]), 48000U);
}

ZTEST_SUITE(uac1_descriptors, NULL, NULL, NULL, NULL, NULL);
