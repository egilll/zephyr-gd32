/*
 * Copyright (c) 2026 Ylhyra ehf.
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file
 * @brief USB Audio Class 1 device public header
 *
 * This header describes class API interaction with the application.
 * The audio device itself is modelled with devicetree zephyr,uac1 compatible.
 *
 * This API is currently considered experimental.
 */

#ifndef ZEPHYR_INCLUDE_USB_CLASS_USBD_UAC1_H_
#define ZEPHYR_INCLUDE_USB_CLASS_USBD_UAC1_H_

#include <zephyr/device.h>

/**
 * @brief USB Audio Class 1 device API
 * @defgroup uac1_device USB Audio Class 1 device API
 * @ingroup usb
 * @{
 */

/**
 * @brief Get entity ID
 *
 * @param node node identifier
 */
#define UAC1_ENTITY_ID(node)							\
	({									\
		BUILD_ASSERT(DT_NODE_HAS_COMPAT(DT_PARENT(node), zephyr_uac1));	\
		UTIL_INC(DT_NODE_CHILD_IDX(node));				\
	})

/**
 * @brief USB Audio Class 1 Feature Unit control selectors
 *
 * Values are defined by USB Audio Device Class 1.0 (Appendix A.12).
 */
enum uac1_fu_control_selector {
	UAC1_FU_MUTE_CONTROL = 0x01,
	UAC1_FU_VOLUME_CONTROL = 0x02,
};

/**
 * @brief USB Audio 1 application event handlers
 */
struct uac1_ops {
	/**
	 * @brief Start of Frame callback
	 *
	 * Notifies application about SOF event on the bus.
	 * This callback is mandatory to register.
	 *
	 * @param dev USB Audio 1 device
	 * @param user_data Opaque user data pointer
	 */
	void (*sof_cb)(const struct device *dev, void *user_data);

	/**
	 * @brief Terminal update callback
	 *
	 * Notifies application that host has enabled or disabled a terminal.
	 * This callback is mandatory to register.
	 *
	 * @param dev USB Audio 1 device
	 * @param terminal Terminal ID linked to AudioStreaming interface
	 * @param enabled True if host enabled terminal, False otherwise
	 * @param microframes True if USB connection speed uses microframes
	 * @param user_data Opaque user data pointer
	 */
	void (*terminal_update_cb)(const struct device *dev, uint8_t terminal,
				   bool enabled, bool microframes,
				   void *user_data);

	/**
	 * @brief Get receive buffer address
	 *
	 * USB stack calls this function to obtain receive buffer address for
	 * AudioStreaming ISO OUT endpoint. The buffer is owned by USB stack
	 * until @ref data_recv_cb callback is called. The buffer must be
	 * sufficiently aligned and otherwise suitable for use by UDC driver.
	 * This callback is mandatory to register for devices receiving USB
	 * audio from the USB host.
	 *
	 * @param dev USB Audio 1 device
	 * @param terminal Input Terminal ID linked to AudioStreaming interface
	 * @param size Maximum number of bytes USB stack will write to buffer
	 * @param user_data Opaque user data pointer
	 */
	void *(*get_recv_buf)(const struct device *dev, uint8_t terminal,
			      uint16_t size, void *user_data);

	/**
	 * @brief Data received
	 *
	 * Releases buffer obtained in @ref get_recv_buf after USB has written
	 * data to the buffer and/or no longer needs it.
	 * This callback is mandatory to register for devices receiving USB
	 * audio from the USB host.
	 *
	 * @param dev USB Audio 1 device
	 * @param terminal Input Terminal ID linked to AudioStreaming interface
	 * @param buf Buffer previously obtained via @ref get_recv_buf
	 * @param size Number of bytes written to buffer
	 * @param user_data Opaque user data pointer
	 */
	void (*data_recv_cb)(const struct device *dev, uint8_t terminal,
			     void *buf, uint16_t size, void *user_data);

	/**
	 * @brief Transmit buffer release callback
	 *
	 * Releases buffer provided in @ref usbd_uac1_send when the class no
	 * longer needs it. This callback is mandatory to register if calling
	 * @ref usbd_uac1_send.
	 *
	 * @param dev USB Audio 1 device
	 * @param terminal Output Terminal ID linked to AudioStreaming interface
	 * @param buf Buffer previously provided via @ref usbd_uac1_send
	 * @param user_data Opaque user data pointer
	 */
	void (*buf_release_cb)(const struct device *dev, uint8_t terminal,
			       void *buf, void *user_data);

	/**
	 * @brief Get explicit feedback value
	 *
	 * Feedback value format depends on bus speed:
	 * - Full-Speed: Q10.14 stored on 24 LSBs (3 bytes)
	 * - High-Speed: Q16.16 (4 bytes)
	 *
	 * This callback is mandatory to register if there is an explicit
	 * feedback endpoint.
	 *
	 * @param dev USB Audio 1 device
	 * @param terminal Terminal ID whose feedback should be returned
	 * @param user_data Opaque user data pointer
	 *
	 * @return Feedback value
	 */
	uint32_t (*feedback_cb)(const struct device *dev, uint8_t terminal,
				void *user_data);

	/**
	 * @brief Get active sample rate
	 *
	 * USB stack calls this function when host asks for active sample rate.
	 * This callback may be NULL if all AudioStreaming interfaces support
	 * only one sample rate.
	 *
	 * @param dev USB Audio 1 device
	 * @param terminal Terminal ID linked to AudioStreaming interface
	 * @param user_data Opaque user data pointer
	 *
	 * @return Active sample rate in Hz
	 */
	uint32_t (*get_sample_rate)(const struct device *dev, uint8_t terminal,
				    void *user_data);

	/**
	 * @brief Set active sample rate
	 *
	 * USB stack calls this function when host sets active sample rate.
	 * This callback may be NULL if all AudioStreaming interfaces support
	 * only one sample rate. USB stack sanitizes the sample rate to closest
	 * valid rate for the given interface.
	 *
	 * @param dev USB Audio 1 device
	 * @param terminal Terminal ID linked to AudioStreaming interface
	 * @param rate Sample rate in Hz
	 * @param user_data Opaque user data pointer
	 *
	 * @return 0 on success, negative value on error
	 */
	int (*set_sample_rate)(const struct device *dev, uint8_t terminal,
			       uint32_t rate, void *user_data);

	/**
	 * @brief Feature Unit control update callback
	 *
	 * Notifies application that the host has updated a Feature Unit control
	 * (e.g. mute or volume).
	 *
	 * @param dev USB Audio 1 device
	 * @param unit_id Feature Unit ID
	 * @param control_selector Feature Unit control selector
	 * @param channel Channel number (0 = master)
	 * @param value Pointer to control value
	 * @param value_len Length of @p value in bytes
	 * @param user_data Opaque user data pointer
	 */
	void (*feature_update_cb)(const struct device *dev, uint8_t unit_id,
				  uint8_t control_selector, uint8_t channel,
				  const void *value, uint8_t value_len,
				  void *user_data);
};

/**
 * @brief Register USB Audio 1 application callbacks.
 *
 * @param dev USB Audio 1 device instance
 * @param ops USB Audio 1 callback structure
 * @param user_data Opaque user data to pass to ops callbacks
 */
void usbd_uac1_set_ops(const struct device *dev,
		       const struct uac1_ops *ops, void *user_data);

/**
 * @brief Send audio data to output terminal
 *
 * Data buffer must be sufficiently aligned and otherwise suitable for use by
 * UDC driver.
 *
 * @note Buffer ownership is transferred to the stack on success. On error the
 * caller retains ownership.
 *
 * @param dev USB Audio 1 device
 * @param terminal Output Terminal ID linked to AudioStreaming interface
 * @param data Buffer containing outgoing data
 * @param size Number of bytes to send
 *
 * @return 0 on success, negative value on error
 */
int usbd_uac1_send(const struct device *dev, uint8_t terminal,
		   void *data, uint16_t size);

/**
 * @}
 */

#endif /* ZEPHYR_INCLUDE_USB_CLASS_USBD_UAC1_H_ */
