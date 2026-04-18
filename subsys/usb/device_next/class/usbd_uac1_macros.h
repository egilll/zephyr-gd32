/*
 * Copyright (c) 2026, The Zephyr Project Contributors
 * SPDX-License-Identifier: Apache-2.0
 */

/* Internal macros used to translate devicetree zephyr,uac1 compatible nodes
 * into descriptor byte arrays.
 */

#include <stdint.h>

#include <zephyr/sys/util.h>
#include <zephyr/usb/usb_ch9.h>

#ifndef ZEPHYR_SUBSYS_USB_DEVICE_NEXT_CLASS_USBD_UAC1_MACROS_H_
#define ZEPHYR_SUBSYS_USB_DEVICE_NEXT_CLASS_USBD_UAC1_MACROS_H_

#define U16_LE(value) ((value) & 0xFF), (((value) & 0xFF00) >> 8)
#define U24_LE(value)								\
	((value) & 0xFF),							\
	(((value) & 0xFF00) >> 8),						\
	(((value) & 0xFF0000) >> 16)

#define UAC1_U64(value) ((uint64_t)(value))

#define ARRAY_ELEMENT_LESS_THAN_NEXT(node, prop, idx)				\
	COND_CODE_1(IS_EQ(idx, UTIL_DEC(DT_PROP_LEN(node, prop))),		\
		(1 /* nothing to compare the last element against */),		\
		((DT_PROP_BY_IDX(node, prop, idx) <				\
		  DT_PROP_BY_IDX(node, prop, UTIL_INC(idx)))))
#define IS_ARRAY_SORTED(node, prop)						\
	DT_FOREACH_PROP_ELEM_SEP(node, prop, ARRAY_ELEMENT_LESS_THAN_NEXT, (&&))

#define FIRST_INTERFACE_NUMBER			0x00
#define FIRST_IN_EP_ADDR			0x81
#define FIRST_OUT_EP_ADDR			0x01

/* A.1 Audio Interface Class Code */
#define AUDIO					0x01

/* A.2 Audio Interface Subclass Codes */
#define AUDIOCONTROL				0x01
#define AUDIOSTREAMING				0x02

/* A.4 Audio Class-Specific Descriptor Types */
#define CS_INTERFACE				0x24
#define CS_ENDPOINT				0x25

/* A.5 Audio Class-Specific AC Interface Descriptor Subtypes */
#define AC_DESCRIPTOR_HEADER			0x01
#define AC_DESCRIPTOR_INPUT_TERMINAL		0x02
#define AC_DESCRIPTOR_OUTPUT_TERMINAL		0x03
#define AC_DESCRIPTOR_FEATURE_UNIT		0x06

/* A.6 Audio Class-Specific AS Interface Descriptor Subtypes */
#define AS_DESCRIPTOR_GENERAL			0x01
#define AS_DESCRIPTOR_FORMAT_TYPE		0x02

/* A.8 Audio Class-Specific Endpoint Descriptor Subtypes */
#define EP_GENERAL				0x01

/* Type I format */
#define FORMAT_TYPE_I				0x01
#define PCM_FORMAT_TAG				0x0001

/* Automatically assign entity IDs based on entities order in devicetree */
#define ENTITY_ID(e) UTIL_INC(DT_NODE_CHILD_IDX(e))

#define DESCRIPTOR_NAME(prefix, node) uac1_##prefix##_##node

#define UAC1_ALLOWED_AT_FULL_SPEED(node) DT_PROP(node, full_speed)
#define UAC1_ALLOWED_AT_HIGH_SPEED(node) DT_PROP(node, high_speed)

#define CONNECTED_ENTITY_ID(entity, phandle)					\
	COND_CODE_1(DT_NODE_HAS_PROP(entity, phandle),				\
		(ENTITY_ID(DT_PHANDLE_BY_IDX(entity, phandle, 0))), (0))

/* ADC v1.0 Audio Channel Cluster. */
#define NUM_SPATIAL_LOCATIONS(entity) DT_PROP(entity, channels)
#define SPATIAL_LOCATIONS(entity) U16_LE(DT_PROP(entity, channel_config))

#define FEATURE_UNIT_CHANNEL_CLUSTER(node)					\
	IF_ENABLED(DT_NODE_HAS_COMPAT(DT_PROP(node, data_source),		\
		zephyr_uac1_input_terminal), (					\
			DT_PROP(node, data_source)				\
	))

/* Track back Output Terminal data source to entity that has channel cluster */
#define OUTPUT_TERMINAL_CHANNEL_CLUSTER(node)					\
	IF_ENABLED(DT_NODE_HAS_COMPAT(DT_PROP(node, data_source),		\
		zephyr_uac1_input_terminal), (					\
			DT_PROP(node, data_source)				\
	))									\
	IF_ENABLED(DT_NODE_HAS_COMPAT(DT_PROP(node, data_source),		\
		zephyr_uac1_feature_unit), (					\
			FEATURE_UNIT_CHANNEL_CLUSTER(DT_PROP(node, data_source))\
	))

#define AUDIO_STREAMING_CHANNEL_CLUSTER(node)					\
	IF_ENABLED(DT_NODE_HAS_COMPAT(DT_PROP(node, linked_terminal),		\
		zephyr_uac1_input_terminal), (					\
			DT_PROP(node, linked_terminal)				\
	))									\
	IF_ENABLED(DT_NODE_HAS_COMPAT(DT_PROP(node, linked_terminal),		\
		zephyr_uac1_output_terminal), (OUTPUT_TERMINAL_CHANNEL_CLUSTER(	\
			DT_PROP(node, linked_terminal))				\
	))

#define AUDIO_STREAMING_NUM_CHANNELS(node)					\
	NUM_SPATIAL_LOCATIONS(AUDIO_STREAMING_CHANNEL_CLUSTER(node))

/* Entities */
#define INPUT_TERMINAL_DESCRIPTOR(entity)					\
	0x0C,						/* bLength */		\
	CS_INTERFACE,					/* bDescriptorType */	\
	AC_DESCRIPTOR_INPUT_TERMINAL,			/* bDescriptorSubtype */\
	ENTITY_ID(entity),				/* bTerminalID */	\
	U16_LE(DT_PROP(entity, terminal_type)),		/* wTerminalType */	\
	CONNECTED_ENTITY_ID(entity, assoc_terminal),	/* bAssocTerminal */	\
	NUM_SPATIAL_LOCATIONS(entity),			/* bNrChannels */	\
	SPATIAL_LOCATIONS(entity),			/* wChannelConfig */	\
	0x00,						/* iChannelNames */	\
	0x00,						/* iTerminal */

#define OUTPUT_TERMINAL_DESCRIPTOR(entity)					\
	0x09,						/* bLength */		\
	CS_INTERFACE,					/* bDescriptorType */	\
	AC_DESCRIPTOR_OUTPUT_TERMINAL,			/* bDescriptorSubtype */\
	ENTITY_ID(entity),				/* bTerminalID */	\
	U16_LE(DT_PROP(entity, terminal_type)),		/* wTerminalType */	\
	CONNECTED_ENTITY_ID(entity, assoc_terminal),	/* bAssocTerminal */	\
	CONNECTED_ENTITY_ID(entity, data_source),	/* bSourceID */		\
	0x00,						/* iTerminal */

/* A.12.2 Feature Unit Control Selectors: Mute (D0), Volume (D1) */
#define FEATURE_UNIT_CONTROLS(entity)						\
	DT_PROP(entity, controls)

#define FEATURE_UNIT_NUM_CHANNELS(entity)					\
	NUM_SPATIAL_LOCATIONS(DT_PHANDLE_BY_IDX(entity, data_source, 0))

#define FEATURE_UNIT_BMA_CONTROLS_BY_IDX(i, entity)				\
	U16_LE(FEATURE_UNIT_CONTROLS(entity))

#define FEATURE_UNIT_BMA_CONTROLS_ARRAY(entity)				\
	LISTIFY(UTIL_INC(FEATURE_UNIT_NUM_CHANNELS(entity)),			\
		FEATURE_UNIT_BMA_CONTROLS_BY_IDX, (,), entity)

#define FEATURE_UNIT_DESCRIPTOR_LENGTH(entity)					\
	(7 + (UTIL_INC(FEATURE_UNIT_NUM_CHANNELS(entity)) * 2) + 1)

#define FEATURE_UNIT_DESCRIPTOR(entity)					\
	FEATURE_UNIT_DESCRIPTOR_LENGTH(entity),		/* bLength */		\
	CS_INTERFACE,					/* bDescriptorType */	\
	AC_DESCRIPTOR_FEATURE_UNIT,			/* bDescriptorSubtype */\
	ENTITY_ID(entity),				/* bUnitID */		\
	CONNECTED_ENTITY_ID(entity, data_source),	/* bSourceID */		\
	0x02,						/* bControlSize */	\
	FEATURE_UNIT_BMA_CONTROLS_ARRAY(entity),	/* bmaControls */	\
	0x00,						/* iFeature */

#define ENTITY_HEADER(entity)							\
	IF_ENABLED(DT_NODE_HAS_COMPAT(entity, zephyr_uac1_input_terminal), (	\
		INPUT_TERMINAL_DESCRIPTOR(entity)				\
	))									\
	IF_ENABLED(DT_NODE_HAS_COMPAT(entity, zephyr_uac1_output_terminal), (	\
		OUTPUT_TERMINAL_DESCRIPTOR(entity)				\
	))									\
	IF_ENABLED(DT_NODE_HAS_COMPAT(entity, zephyr_uac1_feature_unit), (	\
		FEATURE_UNIT_DESCRIPTOR(entity)				\
	))

#define ENTITY_HEADER_ARRAYS(entity)						\
	IF_ENABLED(UTIL_NOT(IS_EMPTY(ENTITY_HEADER(entity))), (			\
		static uint8_t DESCRIPTOR_NAME(ac_entity, entity)[] = {		\
			ENTITY_HEADER(entity)					\
		};								\
	))

#define ENTITY_HEADER_PTRS(entity)						\
	IF_ENABLED(UTIL_NOT(IS_EMPTY(ENTITY_HEADER(entity))), (			\
		(struct usb_desc_header *) &DESCRIPTOR_NAME(ac_entity, entity),	\
	))

#define ENTITY_HEADERS(node) DT_FOREACH_CHILD(node, ENTITY_HEADER)
#define ENTITY_HEADERS_ARRAYS(node) DT_FOREACH_CHILD(node, ENTITY_HEADER_ARRAYS)
#define ENTITY_HEADERS_PTRS(node) DT_FOREACH_CHILD(node, ENTITY_HEADER_PTRS)
#define ENTITY_HEADERS_LENGTH(node) sizeof((uint8_t []){ENTITY_HEADERS(node)})

/* AudioStreaming interface helpers */
#define IS_AUDIOSTREAMING_INTERFACE(node)					\
	DT_NODE_HAS_COMPAT(node, zephyr_uac1_audio_streaming)

#define AS_INTERFACE_NUMBER_IF_AUDIOSTREAMING(node)				\
	IF_ENABLED(IS_AUDIOSTREAMING_INTERFACE(node), (AS_INTERFACE_NUMBER(node),))

#define FIND_AUDIOSTREAMING(node, fn, ...)					\
	IF_ENABLED(IS_AUDIOSTREAMING_INTERFACE(node), (fn(node, __VA_ARGS__)))

#define FOR_EACH_AUDIOSTREAMING_INTERFACE(node, fn, ...)			\
	DT_FOREACH_CHILD_VARGS(node, FIND_AUDIOSTREAMING, fn, __VA_ARGS__)

#define COUNT_AS_INTERFACES_BEFORE_IDX(node, idx)				\
	+ 1 * (DT_NODE_CHILD_IDX(node) < idx)

#define AS_INTERFACE_NUMBER(node)						\
	FIRST_INTERFACE_NUMBER + 1 /* AudioControl interface */	+		\
	FOR_EACH_AUDIOSTREAMING_INTERFACE(DT_PARENT(node),			\
		COUNT_AS_INTERFACES_BEFORE_IDX, DT_NODE_CHILD_IDX(node))

#define UAC1_NUM_AUDIOSTREAMING(node)						\
	DT_FOREACH_CHILD_SEP(node, IS_AUDIOSTREAMING_INTERFACE, (+))

#define UAC1_NUM_INTERFACES(node)						\
	1 /* AudioControl interface */ + UAC1_NUM_AUDIOSTREAMING(node)

/* sync-type: asynchronous(0), adaptive(1), synchronous(2) */
#define UAC1_SYNC_TYPE(node) DT_ENUM_IDX(node, sync_type)
#define UAC1_SYNC_TYPE_ASYNCHRONOUS 0
#define UAC1_SYNC_TYPE_ADAPTIVE 1
#define UAC1_SYNC_TYPE_SYNCHRONOUS 2

#define UAC1_WINDOWS_FULL_SPEED_COMPAT(node)					\
	DT_PROP(node, windows_full_speed_compat)

#define AS_HAS_ISOCHRONOUS_DATA_ENDPOINT(node)					\
	UTIL_NOT(DT_PROP(node, external_interface))

#define AS_IS_USB_ISO_IN(node)							\
	UTIL_AND(AS_HAS_ISOCHRONOUS_DATA_ENDPOINT(node),			\
		DT_NODE_HAS_COMPAT(DT_PROP(node, linked_terminal),		\
			zephyr_uac1_output_terminal))

#define AS_IS_USB_ISO_OUT(node)							\
	UTIL_AND(AS_HAS_ISOCHRONOUS_DATA_ENDPOINT(node),			\
		DT_NODE_HAS_COMPAT(DT_PROP(node, linked_terminal),		\
			zephyr_uac1_input_terminal))

#define AS_HAS_EXPLICIT_FEEDBACK_ENDPOINT(node)					\
	UTIL_AND(UTIL_AND(AS_HAS_ISOCHRONOUS_DATA_ENDPOINT(node),		\
			  UTIL_NOT(DT_PROP(node, implicit_feedback))),		\
		 UTIL_AND(AS_IS_USB_ISO_OUT(node),				\
			  IS_EQ(UAC1_SYNC_TYPE(node),				\
				UAC1_SYNC_TYPE_ASYNCHRONOUS)))

#define COUNT_AS_OUT_ENDPOINTS_BEFORE_IDX(node, idx)				\
	+ AS_IS_USB_ISO_OUT(node) * (DT_NODE_CHILD_IDX(node) < idx)

#define COUNT_AS_IN_ENDPOINTS_BEFORE_IDX(node, idx)				\
	+ (AS_IS_USB_ISO_IN(node) + AS_HAS_EXPLICIT_FEEDBACK_ENDPOINT(node)) *	\
	  (DT_NODE_CHILD_IDX(node) < idx)

#define AS_NEXT_OUT_EP_ADDR(node)						\
	FIRST_OUT_EP_ADDR +							\
	FOR_EACH_AUDIOSTREAMING_INTERFACE(DT_PARENT(node),			\
		COUNT_AS_OUT_ENDPOINTS_BEFORE_IDX, DT_NODE_CHILD_IDX(node))

#define AS_NEXT_IN_EP_ADDR(node)						\
	FIRST_IN_EP_ADDR + DT_PROP(DT_PARENT(node), interrupt_endpoint) +	\
	FOR_EACH_AUDIOSTREAMING_INTERFACE(DT_PARENT(node),			\
		COUNT_AS_IN_ENDPOINTS_BEFORE_IDX, DT_NODE_CHILD_IDX(node))

#define AS_DATA_EP_ADDR(node)							\
	COND_CODE_1(AS_IS_USB_ISO_OUT(node), (AS_NEXT_OUT_EP_ADDR(node)),	\
		(AS_NEXT_IN_EP_ADDR(node)))

#define AS_FEEDBACK_EP_ADDR(node)						\
	AS_NEXT_IN_EP_ADDR(node)

#define AS_DATA_EP_USAGE_TYPE(node)						\
	COND_CODE_1(UTIL_AND(DT_PROP(node, implicit_feedback),			\
		UTIL_NOT(AS_IS_USB_ISO_OUT(node))), (0x2 << 4), (0x0 << 4))

#define AS_DATA_EP_SYNC_TYPE_BITS(node)					\
	COND_CODE_1(IS_EQ(UAC1_SYNC_TYPE(node), UAC1_SYNC_TYPE_ASYNCHRONOUS),	\
		(0x1 << 2),							\
		(COND_CODE_1(IS_EQ(UAC1_SYNC_TYPE(node), UAC1_SYNC_TYPE_ADAPTIVE),\
			(0x2 << 2), (0x3 << 2)))				\
	)

#define AS_DATA_EP_ATTR(node)							\
	USB_EP_TYPE_ISO | AS_DATA_EP_SYNC_TYPE_BITS(node) |			\
	AS_DATA_EP_USAGE_TYPE(node)

#define AS_FEEDBACK_EP_ATTR							\
	(USB_EP_TYPE_ISO | (0x1 << 4))

/* Sampling frequencies are required to be sorted ascending. */
#define AS_CLK_MAX_FREQUENCY(node)						\
	DT_PROP_BY_IDX(node, sampling_frequencies,				\
		       UTIL_DEC(DT_PROP_LEN(node, sampling_frequencies)))

#define AS_BYTES_PER_SAMPLE(node) DT_PROP(node, subslot_size)

#define AS_FS_DATA_EP_BINTERVAL(node)						\
	USB_FS_ISO_EP_INTERVAL(DT_PROP_OR(node, polling_period_us, 1000))

#define AS_HS_DATA_EP_BINTERVAL(node)						\
	USB_HS_ISO_EP_INTERVAL(DT_PROP_OR(node, polling_period_us, 1000))

#define AS_EXTRA_SAMPLE(node)							\
	UTIL_OR(IS_EQ(UAC1_SYNC_TYPE(node), UAC1_SYNC_TYPE_ASYNCHRONOUS),	\
		IS_EQ(UAC1_SYNC_TYPE(node), UAC1_SYNC_TYPE_ADAPTIVE))

#define AS_SAMPLES_PER_FRAME(node)						\
	((ROUND_UP(AS_CLK_MAX_FREQUENCY(node), 1000) / 1000) + AS_EXTRA_SAMPLE(node))

#define AS_SAMPLES_PER_MICROFRAME(node)						\
	(DIV_ROUND_UP(UAC1_U64(AS_CLK_MAX_FREQUENCY(node)) *			\
		      (UAC1_U64(1) << (AS_HS_DATA_EP_BINTERVAL(node) - 1)),	\
		      8000ULL) + AS_EXTRA_SAMPLE(node))

#define AS_FS_DATA_EP_MAX_PACKET_SIZE(node)					\
	AUDIO_STREAMING_NUM_CHANNELS(node) *					\
	AS_BYTES_PER_SAMPLE(node) * AS_SAMPLES_PER_FRAME(node)

#define AS_HS_DATA_EP_TPL(node)							\
	USB_TPL_ROUND_UP(AUDIO_STREAMING_NUM_CHANNELS(node) *			\
			 AS_BYTES_PER_SAMPLE(node) *				\
			 AS_SAMPLES_PER_MICROFRAME(node))

#define AS_HS_DATA_EP_MAX_PACKET_SIZE(node)					\
	USB_TPL_TO_MPS(AS_HS_DATA_EP_TPL(node))

#define AS_FEEDBACK_EP_BINTERVAL(node)						\
	USB_FS_ISO_EP_INTERVAL(DT_PROP_OR(node, feedback_period_us,		\
					  DT_PROP_OR(node, polling_period_us, 1000)))

#define AS_FEEDBACK_EP_HS_BINTERVAL(node)					\
	USB_HS_ISO_EP_INTERVAL(DT_PROP_OR(node, feedback_period_us,		\
					  DT_PROP_OR(node, polling_period_us, 1000)))

#define AS_INTERFACE_NUM_ENDPOINTS(node)					\
	(1 + AS_HAS_EXPLICIT_FEEDBACK_ENDPOINT(node))

/* Interface Association Descriptor */
#define UAC1_INTERFACE_ASSOCIATION_DESCRIPTOR(node)				\
	0x08,						/* bLength */		\
	USB_DESC_INTERFACE_ASSOC,			/* bDescriptorType */	\
	FIRST_INTERFACE_NUMBER,				/* bFirstInterface */	\
	UAC1_NUM_INTERFACES(node),			/* bInterfaceCount */	\
	AUDIO,						/* bFunctionClass */	\
	0x00,						/* bFunctionSubclass */ \
	0x00,						/* bFunctionProtocol */	\
	0x00						/* iFunction */

#define UAC1_INTERFACE_ASSOCIATION_DESCRIPTOR_ARRAY(node)			\
	IF_ENABLED(UAC1_ALLOWED_AT_FULL_SPEED(node), (				\
		static uint8_t DESCRIPTOR_NAME(fs_iad, node)[] = {		\
			UAC1_INTERFACE_ASSOCIATION_DESCRIPTOR(node)		\
		};								\
	))									\
	IF_ENABLED(UAC1_ALLOWED_AT_HIGH_SPEED(node), (				\
		static uint8_t DESCRIPTOR_NAME(hs_iad, node)[] = {		\
			UAC1_INTERFACE_ASSOCIATION_DESCRIPTOR(node)		\
		};								\
	))

#define UAC1_INTERFACE_ASSOCIATION_FS_DESCRIPTOR_PTR(node)			\
	(struct usb_desc_header *) &DESCRIPTOR_NAME(fs_iad, node),
#define UAC1_INTERFACE_ASSOCIATION_HS_DESCRIPTOR_PTR(node)			\
	(struct usb_desc_header *) &DESCRIPTOR_NAME(hs_iad, node),

/* Standard AC Interface Descriptor */
#define AC_INTERFACE_DESCRIPTOR(node)						\
	0x09,						/* bLength */		\
	USB_DESC_INTERFACE,				/* bDescriptorType */	\
	FIRST_INTERFACE_NUMBER,				/* bInterfaceNumber */	\
	0x00,						/* bAlternateSetting */ \
	DT_PROP(node, interrupt_endpoint),		/* bNumEndpoints */	\
	AUDIO,						/* bInterfaceClass */	\
	AUDIOCONTROL,					/* bInterfaceSubClass */\
	0x00,						/* bInterfaceProtocol */\
	0x00						/* iInterface */

#define AC_INTERFACE_DESCRIPTOR_ARRAY(node)					\
	IF_ENABLED(UAC1_ALLOWED_AT_FULL_SPEED(node), (				\
		static uint8_t DESCRIPTOR_NAME(fs_ac_interface, node)[] = {	\
			AC_INTERFACE_DESCRIPTOR(node)				\
		};								\
	))									\
	IF_ENABLED(UAC1_ALLOWED_AT_HIGH_SPEED(node), (				\
		static uint8_t DESCRIPTOR_NAME(hs_ac_interface, node)[] = {	\
			AC_INTERFACE_DESCRIPTOR(node)				\
		};								\
	))

#define AC_INTERFACE_FS_DESCRIPTOR_PTR(node)					\
	(struct usb_desc_header *) &DESCRIPTOR_NAME(fs_ac_interface, node),
#define AC_INTERFACE_HS_DESCRIPTOR_PTR(node)					\
	(struct usb_desc_header *) &DESCRIPTOR_NAME(hs_ac_interface, node),

/* Standard AC Interrupt Endpoint Descriptor */
#define AC_ENDPOINT_DESCRIPTOR(binterval)					\
	0x09,						/* bLength */		\
	USB_DESC_ENDPOINT,				/* bDescriptorType */	\
	FIRST_IN_EP_ADDR,				/* bEndpointAddress */	\
	USB_EP_TYPE_INTERRUPT,				/* bmAttributes */	\
	U16_LE(0x0002),					/* wMaxPacketSize */	\
	binterval,					/* bInterval */		\
	0x00,						/* bRefresh */		\
	0x00						/* bSynchAddress */

#define AC_ENDPOINT_DESCRIPTOR_ARRAY(node)					\
	IF_ENABLED(UAC1_ALLOWED_AT_FULL_SPEED(node), (				\
		static uint8_t DESCRIPTOR_NAME(fs_ac_endpoint, node)[] = {	\
			AC_ENDPOINT_DESCRIPTOR(USB_FS_INT_EP_INTERVAL(10000U))	\
		};								\
	))									\
	IF_ENABLED(UAC1_ALLOWED_AT_HIGH_SPEED(node), (				\
		static uint8_t DESCRIPTOR_NAME(hs_ac_endpoint, node)[] = {	\
			AC_ENDPOINT_DESCRIPTOR(USB_HS_INT_EP_INTERVAL(10000U))	\
		};								\
	))

#define AC_ENDPOINT_FS_DESCRIPTOR_PTR(node)					\
	(struct usb_desc_header *) &DESCRIPTOR_NAME(fs_ac_endpoint, node),

#define AC_ENDPOINT_HS_DESCRIPTOR_PTR(node)					\
	(struct usb_desc_header *) &DESCRIPTOR_NAME(hs_ac_endpoint, node),

/* Class-Specific AC Interface Header Descriptor */
#define AC_HEADER_DESCRIPTOR_LENGTH(node)					\
	(8 + UAC1_NUM_AUDIOSTREAMING(node))

#define AC_TOTAL_LENGTH(node)							\
	(AC_HEADER_DESCRIPTOR_LENGTH(node) + ENTITY_HEADERS_LENGTH(node))

#define AC_INTERFACE_HEADER_DESCRIPTOR(node)					\
	AC_HEADER_DESCRIPTOR_LENGTH(node),		/* bLength */		\
	CS_INTERFACE,					/* bDescriptorType */	\
	AC_DESCRIPTOR_HEADER,				/* bDescriptorSubtype */\
	U16_LE(0x0100),					/* bcdADC */		\
	U16_LE(AC_TOTAL_LENGTH(node)),			/* wTotalLength */	\
	UAC1_NUM_AUDIOSTREAMING(node),			/* bInCollection */	\
	DT_FOREACH_CHILD(node, AS_INTERFACE_NUMBER_IF_AUDIOSTREAMING)

#define AC_INTERFACE_HEADER_DESCRIPTOR_ARRAY(node)				\
	static uint8_t DESCRIPTOR_NAME(ac_header, node)[] = {			\
		AC_INTERFACE_HEADER_DESCRIPTOR(node)				\
	};

#define AC_INTERFACE_HEADER_DESCRIPTOR_PTR(node)				\
	(struct usb_desc_header *) &DESCRIPTOR_NAME(ac_header, node),

/* Standard AS Interface Descriptor */
#define AS_INTERFACE_DESCRIPTOR(node, alternate, numendpoints)			\
	0x09,						/* bLength */		\
	USB_DESC_INTERFACE,				/* bDescriptorType */	\
	AS_INTERFACE_NUMBER(node),			/* bInterfaceNumber */	\
	alternate,					/* bAlternateSetting */	\
	numendpoints,					/* bNumEndpoints */	\
	AUDIO,						/* bInterfaceClass */	\
	AUDIOSTREAMING,					/* bInterfaceSubClass */\
	0x00,						/* bInterfaceProtocol */\
	0x00						/* iInterface */

#define AS_INTERFACE_FS_DESCRIPTOR_ARRAY(node, alternate, numendpoints)		\
	static uint8_t DESCRIPTOR_NAME(fs_as_if_alt##alternate, node)[] = {	\
		AS_INTERFACE_DESCRIPTOR(node, alternate, numendpoints)		\
	};

#define AS_INTERFACE_HS_DESCRIPTOR_ARRAY(node, alternate, numendpoints)		\
	static uint8_t DESCRIPTOR_NAME(hs_as_if_alt##alternate, node)[] = {	\
		AS_INTERFACE_DESCRIPTOR(node, alternate, numendpoints)		\
	};

#define AS_INTERFACE_DESCRIPTOR_ARRAY(node, alternate, numendpoints)		\
	IF_ENABLED(UAC1_ALLOWED_AT_FULL_SPEED(DT_PARENT(node)), (		\
		AS_INTERFACE_FS_DESCRIPTOR_ARRAY(node, alternate, numendpoints)	\
	))									\
	IF_ENABLED(UAC1_ALLOWED_AT_HIGH_SPEED(DT_PARENT(node)), (		\
		AS_INTERFACE_HS_DESCRIPTOR_ARRAY(node, alternate, numendpoints)	\
	))

#define AS_INTERFACE_FS_DESCRIPTOR_PTR(node, altnum)				\
	(struct usb_desc_header *) &DESCRIPTOR_NAME(fs_as_if_alt##altnum, node),
#define AS_INTERFACE_HS_DESCRIPTOR_PTR(node, altnum)				\
	(struct usb_desc_header *) &DESCRIPTOR_NAME(hs_as_if_alt##altnum, node),

/* Class-Specific AS Interface Descriptor */
#define AUDIO_STREAMING_GENERAL_DESCRIPTOR(node)				\
	0x07,						/* bLength */		\
	CS_INTERFACE,					/* bDescriptorType */	\
	AS_DESCRIPTOR_GENERAL,				/* bDescriptorSubtype */\
	CONNECTED_ENTITY_ID(node, linked_terminal),	/* bTerminalLink */	\
	0x00,						/* bDelay */		\
	U16_LE(PCM_FORMAT_TAG)				/* wFormatTag */

#define AUDIO_STREAMING_FORMAT_DESCRIPTOR_LENGTH(node)				\
	(8 + (DT_PROP_LEN(node, sampling_frequencies) * 3))

#define U24_LE_BY_IDX(node_id, prop, idx)					\
	U24_LE(DT_PROP_BY_IDX(node_id, prop, idx))

#define AUDIO_STREAMING_FORMAT_TYPE_DESCRIPTOR(node)				\
	AUDIO_STREAMING_FORMAT_DESCRIPTOR_LENGTH(node),/* bLength */		\
	CS_INTERFACE,					/* bDescriptorType */	\
	AS_DESCRIPTOR_FORMAT_TYPE,			/* bDescriptorSubtype */\
	FORMAT_TYPE_I,					/* bFormatType */	\
	AUDIO_STREAMING_NUM_CHANNELS(node),		/* bNrChannels */	\
	DT_PROP(node, subslot_size),			/* bSubFrameSize */	\
	DT_PROP(node, bit_resolution),			/* bBitResolution */	\
	DT_PROP_LEN(node, sampling_frequencies),	/* bSamFreqType */	\
	DT_FOREACH_PROP_ELEM_SEP(node, sampling_frequencies, U24_LE_BY_IDX, (,))

#define VALIDATE_AS_BANDWIDTH(node)						\
	IF_ENABLED(AS_HAS_ISOCHRONOUS_DATA_ENDPOINT(node), (			\
		IF_ENABLED(UAC1_ALLOWED_AT_FULL_SPEED(DT_PARENT(node)), (	\
			BUILD_ASSERT(AS_FS_DATA_EP_MAX_PACKET_SIZE(node) <= 1023,	\
				     "Full-Speed bandwidth exceeded");		\
			BUILD_ASSERT(AS_FS_DATA_EP_BINTERVAL(node) == 1,	\
				     "UAC1 requires 1ms FS polling");		\
		))								\
		IF_ENABLED(UAC1_ALLOWED_AT_HIGH_SPEED(DT_PARENT(node)), (	\
			BUILD_ASSERT(USB_TPL_IS_VALID(AS_HS_DATA_EP_TPL(node)),	\
				     "High-Speed bandwidth exceeded");		\
			BUILD_ASSERT(AS_HS_DATA_EP_BINTERVAL(node) == 4,	\
				     "UAC1 requires 1ms HS polling (bInterval=4)");\
		))								\
		IF_ENABLED(AS_HAS_EXPLICIT_FEEDBACK_ENDPOINT(node), (		\
			IF_ENABLED(UAC1_ALLOWED_AT_FULL_SPEED(DT_PARENT(node)), (\
				BUILD_ASSERT(AS_FEEDBACK_EP_BINTERVAL(node) == 1,	\
					     "UAC1 requires 1ms FS feedback polling");\
			))							\
			IF_ENABLED(UAC1_ALLOWED_AT_HIGH_SPEED(DT_PARENT(node)), (\
				BUILD_ASSERT(AS_FEEDBACK_EP_HS_BINTERVAL(node) == 4,	\
					     "UAC1 requires 1ms HS feedback polling");\
			))							\
		))								\
	))

#define AUDIO_STREAMING_INTERFACE_DESCRIPTORS_ARRAYS(node)			\
	BUILD_ASSERT(DT_PROP_LEN(node, sampling_frequencies),			\
		     "sampling-frequencies must not be empty");			\
	BUILD_ASSERT(IS_ARRAY_SORTED(node, sampling_frequencies),		\
		     "sampling-frequencies must be sorted");			\
	BUILD_ASSERT(DT_PROP(node, subslot_size) >= 1 && DT_PROP(node, subslot_size) <= 4, \
		     "subslot-size must be 1..4");				\
	BUILD_ASSERT(DT_PROP(node, bit_resolution) <= (DT_PROP(node, subslot_size) * 8), \
		     "bit-resolution exceeds subslot-size");			\
	BUILD_ASSERT(UTIL_NOT(UTIL_AND(					\
			UAC1_WINDOWS_FULL_SPEED_COMPAT(DT_PARENT(node)),	\
			UTIL_AND(UAC1_ALLOWED_AT_FULL_SPEED(DT_PARENT(node)),	\
				 IS_EQ(DT_PROP(node, subslot_size), 3)))),	\
		     "Windows-compatible FS UAC1 does not support packed 24-bit audio");\
	BUILD_ASSERT(UTIL_NOT(UTIL_AND(					\
			UAC1_WINDOWS_FULL_SPEED_COMPAT(DT_PARENT(node)),	\
			UTIL_AND(UAC1_ALLOWED_AT_FULL_SPEED(DT_PARENT(node)),	\
				 UTIL_AND(AS_IS_USB_ISO_OUT(node),		\
					  IS_EQ(UAC1_SYNC_TYPE(node),		\
						UAC1_SYNC_TYPE_ASYNCHRONOUS))))),\
		     "Windows-compatible FS UAC1 requires adaptive or synchronous USB ISO OUT");\
	BUILD_ASSERT(UTIL_OR(DT_NODE_HAS_COMPAT(DT_PROP(node, linked_terminal),	\
						zephyr_uac1_input_terminal),	\
			     DT_NODE_HAS_COMPAT(DT_PROP(node, linked_terminal),	\
						zephyr_uac1_output_terminal)),	\
		     "linked-terminal must be Input/Output Terminal");		\
	BUILD_ASSERT(UTIL_NOT(UTIL_AND(AS_IS_USB_ISO_IN(node),			\
				       IS_EQ(UAC1_SYNC_TYPE(node),		\
					     UAC1_SYNC_TYPE_ADAPTIVE))),	\
		     "Adaptive USB ISO IN is not supported");			\
	VALIDATE_AS_BANDWIDTH(node)						\
	static uint8_t DESCRIPTOR_NAME(as_general_desc, node)[] = {		\
		AUDIO_STREAMING_GENERAL_DESCRIPTOR(node)			\
	};									\
	static uint8_t DESCRIPTOR_NAME(as_format_desc, node)[] = {		\
		AUDIO_STREAMING_FORMAT_TYPE_DESCRIPTOR(node)			\
	};

#define AUDIO_STREAMING_INTERFACE_DESCRIPTORS_PTRS(node)			\
	(struct usb_desc_header *) &DESCRIPTOR_NAME(as_general_desc, node),	\
	(struct usb_desc_header *) &DESCRIPTOR_NAME(as_format_desc, node),

/* Standard AS Isochronous Audio Data Endpoint Descriptor (expanded to 9 bytes) */
#define STANDARD_AS_ISOCHRONOUS_DATA_ENDPOINT_DESCRIPTOR(node, binterval, mps)	\
	0x09,						/* bLength */		\
	USB_DESC_ENDPOINT,				/* bDescriptorType */	\
	AS_DATA_EP_ADDR(node),				/* bEndpointAddress */	\
	AS_DATA_EP_ATTR(node),				/* bmAttributes */	\
	U16_LE(mps),					/* wMaxPacketSize */	\
	binterval,					/* bInterval */		\
	0x00,						/* bRefresh */		\
	COND_CODE_1(AS_HAS_EXPLICIT_FEEDBACK_ENDPOINT(node),			\
		(AS_FEEDBACK_EP_ADDR(node)), (0x00))				\
							/* bSynchAddress */

#define AS_ISOCHRONOUS_DATA_ENDPOINT_FS_DESCRIPTORS_ARRAYS(node)		\
	static uint8_t DESCRIPTOR_NAME(fs_std_data_ep, node)[] = {		\
		STANDARD_AS_ISOCHRONOUS_DATA_ENDPOINT_DESCRIPTOR(		\
			node, AS_FS_DATA_EP_BINTERVAL(node),			\
			AS_FS_DATA_EP_MAX_PACKET_SIZE(node))			\
	};

#define AS_ISOCHRONOUS_DATA_ENDPOINT_HS_DESCRIPTORS_ARRAYS(node)		\
	static uint8_t DESCRIPTOR_NAME(hs_std_data_ep, node)[] = {		\
		STANDARD_AS_ISOCHRONOUS_DATA_ENDPOINT_DESCRIPTOR(		\
			node, AS_HS_DATA_EP_BINTERVAL(node),			\
			AS_HS_DATA_EP_MAX_PACKET_SIZE(node))			\
	};

/* Class-Specific AS Isochronous Audio Data Endpoint Descriptor */
#define DATA_EP_HAS_SAMPLING_FREQ_CONTROL(node)				\
	UTIL_NOT(IS_EQ(DT_PROP_LEN(node, sampling_frequencies), 1))

#define CLASS_SPECIFIC_AS_ISOCHRONOUS_DATA_ENDPOINT_DESCRIPTOR(node)		\
	0x07,						/* bLength */		\
	CS_ENDPOINT,					/* bDescriptorType */	\
	EP_GENERAL,					/* bDescriptorSubtype */\
	COND_CODE_1(DATA_EP_HAS_SAMPLING_FREQ_CONTROL(node), (0x01), (0x00)),	\
	0x00,						/* bLockDelayUnits */	\
	U16_LE(0x0000)					/* wLockDelay */

#define AS_ISOCHRONOUS_DATA_ENDPOINT_CS_DESCRIPTORS_ARRAYS(node)		\
	static uint8_t DESCRIPTOR_NAME(cs_data_ep, node)[] = {			\
		CLASS_SPECIFIC_AS_ISOCHRONOUS_DATA_ENDPOINT_DESCRIPTOR(node)	\
	};

#define AS_ISOCHRONOUS_DATA_ENDPOINT_FS_DESCRIPTORS_PTRS(node)			\
	(struct usb_desc_header *) &DESCRIPTOR_NAME(fs_std_data_ep, node),	\
	(struct usb_desc_header *) &DESCRIPTOR_NAME(cs_data_ep, node),

#define AS_ISOCHRONOUS_DATA_ENDPOINT_HS_DESCRIPTORS_PTRS(node)			\
	(struct usb_desc_header *) &DESCRIPTOR_NAME(hs_std_data_ep, node),	\
	(struct usb_desc_header *) &DESCRIPTOR_NAME(cs_data_ep, node),

/* Standard AS Isochronous Synch (explicit feedback) Endpoint Descriptor */
#define AS_EXPLICIT_FEEDBACK_ENDPOINT_DESCRIPTOR(node, binterval, mps)		\
	0x09,						/* bLength */		\
	USB_DESC_ENDPOINT,				/* bDescriptorType */	\
	AS_FEEDBACK_EP_ADDR(node),			/* bEndpointAddress */	\
	AS_FEEDBACK_EP_ATTR,				/* bmAttributes */	\
	U16_LE(mps),					/* wMaxPacketSize */	\
	binterval,					/* bInterval */		\
	DT_PROP_OR(node, feedback_refresh, 1),		/* bRefresh */		\
	0x00						/* bSynchAddress */

#define AS_EXPLICIT_FEEDBACK_FS_DESCRIPTOR_ARRAY(node)				\
	static uint8_t DESCRIPTOR_NAME(fs_feedback_ep, node)[] = {		\
		AS_EXPLICIT_FEEDBACK_ENDPOINT_DESCRIPTOR(			\
			node, AS_FEEDBACK_EP_BINTERVAL(node), 0x0003)		\
	};

#define AS_EXPLICIT_FEEDBACK_HS_DESCRIPTOR_ARRAY(node)				\
	static uint8_t DESCRIPTOR_NAME(hs_feedback_ep, node)[] = {		\
		AS_EXPLICIT_FEEDBACK_ENDPOINT_DESCRIPTOR(			\
			node, AS_FEEDBACK_EP_HS_BINTERVAL(node), 0x0004)	\
	};

#define AS_EXPLICIT_FEEDBACK_ENDPOINT_FS_DESCRIPTOR_PTR(node)			\
	(struct usb_desc_header *) &DESCRIPTOR_NAME(fs_feedback_ep, node),

#define AS_EXPLICIT_FEEDBACK_ENDPOINT_HS_DESCRIPTOR_PTR(node)			\
	(struct usb_desc_header *) &DESCRIPTOR_NAME(hs_feedback_ep, node),

#define AS_FS_DESCRIPTORS_ARRAYS(node)						\
	IF_ENABLED(AS_HAS_ISOCHRONOUS_DATA_ENDPOINT(node), (			\
		AS_ISOCHRONOUS_DATA_ENDPOINT_FS_DESCRIPTORS_ARRAYS(node)	\
		IF_ENABLED(AS_HAS_EXPLICIT_FEEDBACK_ENDPOINT(node), (		\
			AS_EXPLICIT_FEEDBACK_FS_DESCRIPTOR_ARRAY(node)))	\
	))

#define AS_HS_DESCRIPTORS_ARRAYS(node)						\
	IF_ENABLED(AS_HAS_ISOCHRONOUS_DATA_ENDPOINT(node), (			\
		AS_ISOCHRONOUS_DATA_ENDPOINT_HS_DESCRIPTORS_ARRAYS(node)	\
		IF_ENABLED(AS_HAS_EXPLICIT_FEEDBACK_ENDPOINT(node), (		\
			AS_EXPLICIT_FEEDBACK_HS_DESCRIPTOR_ARRAY(node)))	\
	))

#define AS_DESCRIPTORS_ARRAYS(node)						\
	AS_INTERFACE_DESCRIPTOR_ARRAY(node, 0, 0)				\
	IF_ENABLED(AS_HAS_ISOCHRONOUS_DATA_ENDPOINT(node), (			\
		AS_INTERFACE_DESCRIPTOR_ARRAY(node, 1,				\
			AS_INTERFACE_NUM_ENDPOINTS(node))))			\
	AUDIO_STREAMING_INTERFACE_DESCRIPTORS_ARRAYS(node)			\
	IF_ENABLED(UAC1_ALLOWED_AT_FULL_SPEED(DT_PARENT(node)), (		\
		AS_FS_DESCRIPTORS_ARRAYS(node)))				\
	IF_ENABLED(UAC1_ALLOWED_AT_HIGH_SPEED(DT_PARENT(node)), (		\
		AS_HS_DESCRIPTORS_ARRAYS(node)))				\
	IF_ENABLED(AS_HAS_ISOCHRONOUS_DATA_ENDPOINT(node), (			\
		AS_ISOCHRONOUS_DATA_ENDPOINT_CS_DESCRIPTORS_ARRAYS(node)))

#define AS_FS_DESCRIPTORS_PTRS(node)						\
	AS_INTERFACE_FS_DESCRIPTOR_PTR(node, 0)					\
	IF_ENABLED(AS_HAS_ISOCHRONOUS_DATA_ENDPOINT(node), (			\
		AS_INTERFACE_FS_DESCRIPTOR_PTR(node, 1)))			\
	AUDIO_STREAMING_INTERFACE_DESCRIPTORS_PTRS(node)			\
	IF_ENABLED(AS_HAS_ISOCHRONOUS_DATA_ENDPOINT(node), (			\
		AS_ISOCHRONOUS_DATA_ENDPOINT_FS_DESCRIPTORS_PTRS(node)		\
		IF_ENABLED(AS_HAS_EXPLICIT_FEEDBACK_ENDPOINT(node), (		\
			AS_EXPLICIT_FEEDBACK_ENDPOINT_FS_DESCRIPTOR_PTR(node)))\
	))

#define AS_HS_DESCRIPTORS_PTRS(node)						\
	AS_INTERFACE_HS_DESCRIPTOR_PTR(node, 0)					\
	IF_ENABLED(AS_HAS_ISOCHRONOUS_DATA_ENDPOINT(node), (			\
		AS_INTERFACE_HS_DESCRIPTOR_PTR(node, 1)))			\
	AUDIO_STREAMING_INTERFACE_DESCRIPTORS_PTRS(node)			\
	IF_ENABLED(AS_HAS_ISOCHRONOUS_DATA_ENDPOINT(node), (			\
		AS_ISOCHRONOUS_DATA_ENDPOINT_HS_DESCRIPTORS_PTRS(node)		\
		IF_ENABLED(AS_HAS_EXPLICIT_FEEDBACK_ENDPOINT(node), (		\
			AS_EXPLICIT_FEEDBACK_ENDPOINT_HS_DESCRIPTOR_PTR(node)))\
	))

#define AS_DESCRIPTORS_ARRAYS_IF_AUDIOSTREAMING(node)				\
	IF_ENABLED(IS_AUDIOSTREAMING_INTERFACE(node), (AS_DESCRIPTORS_ARRAYS(node)))
#define AS_FS_DESCRIPTORS_PTRS_IF_AUDIOSTREAMING(node)				\
	IF_ENABLED(IS_AUDIOSTREAMING_INTERFACE(node), (AS_FS_DESCRIPTORS_PTRS(node)))
#define AS_HS_DESCRIPTORS_PTRS_IF_AUDIOSTREAMING(node)				\
	IF_ENABLED(IS_AUDIOSTREAMING_INTERFACE(node), (AS_HS_DESCRIPTORS_PTRS(node)))

#define UAC1_AUDIO_CONTROL_DESCRIPTOR_ARRAYS(node)				\
	AC_INTERFACE_DESCRIPTOR_ARRAY(node)					\
	AC_INTERFACE_HEADER_DESCRIPTOR_ARRAY(node)				\
	ENTITY_HEADERS_ARRAYS(node)						\
	IF_ENABLED(DT_PROP(node, interrupt_endpoint), (				\
		AC_ENDPOINT_DESCRIPTOR_ARRAY(node)))

#define UAC1_AUDIO_CONTROL_COMMON_DESCRIPTOR_PTRS(node)				\
	AC_INTERFACE_HEADER_DESCRIPTOR_PTR(node)				\
	ENTITY_HEADERS_PTRS(node)

#define UAC1_AUDIO_CONTROL_FS_DESCRIPTOR_PTRS(node)				\
	AC_INTERFACE_FS_DESCRIPTOR_PTR(node)					\
	UAC1_AUDIO_CONTROL_COMMON_DESCRIPTOR_PTRS(node)			\
	IF_ENABLED(DT_PROP(node, interrupt_endpoint), (				\
		AC_ENDPOINT_FS_DESCRIPTOR_PTR(node)))

#define UAC1_AUDIO_CONTROL_HS_DESCRIPTOR_PTRS(node)				\
	AC_INTERFACE_HS_DESCRIPTOR_PTR(node)					\
	UAC1_AUDIO_CONTROL_COMMON_DESCRIPTOR_PTRS(node)			\
	IF_ENABLED(DT_PROP(node, interrupt_endpoint), (				\
		AC_ENDPOINT_HS_DESCRIPTOR_PTR(node)))

#define UAC1_DESCRIPTOR_ARRAYS(node)						\
	UAC1_INTERFACE_ASSOCIATION_DESCRIPTOR_ARRAY(node)			\
	UAC1_AUDIO_CONTROL_DESCRIPTOR_ARRAYS(node)				\
	DT_FOREACH_CHILD(node, AS_DESCRIPTORS_ARRAYS_IF_AUDIOSTREAMING)

#define UAC1_FS_DESCRIPTOR_PTRS(node)						\
	UAC1_INTERFACE_ASSOCIATION_FS_DESCRIPTOR_PTR(node)			\
	UAC1_AUDIO_CONTROL_FS_DESCRIPTOR_PTRS(node)				\
	DT_FOREACH_CHILD(node, AS_FS_DESCRIPTORS_PTRS_IF_AUDIOSTREAMING)	\
	NULL

#define UAC1_HS_DESCRIPTOR_PTRS(node)						\
	UAC1_INTERFACE_ASSOCIATION_HS_DESCRIPTOR_PTR(node)			\
	UAC1_AUDIO_CONTROL_HS_DESCRIPTOR_PTRS(node)				\
	DT_FOREACH_CHILD(node, AS_HS_DESCRIPTORS_PTRS_IF_AUDIOSTREAMING)	\
	NULL

#define UAC1_FS_DESCRIPTOR_PTRS_ARRAY(node)					\
	COND_CODE_1(UAC1_ALLOWED_AT_FULL_SPEED(node),				\
		({UAC1_FS_DESCRIPTOR_PTRS(node)}), ({NULL}))

#define UAC1_HS_DESCRIPTOR_PTRS_ARRAY(node)					\
	COND_CODE_1(UAC1_ALLOWED_AT_HIGH_SPEED(node),				\
		({UAC1_HS_DESCRIPTOR_PTRS(node)}), ({NULL}))

#endif /* ZEPHYR_SUBSYS_USB_DEVICE_NEXT_CLASS_USBD_UAC1_MACROS_H_ */
