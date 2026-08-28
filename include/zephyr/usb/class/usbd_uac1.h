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

#ifdef __cplusplus
extern "C" {
#endif

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

/** USB Audio 1 interrupt status originator types. */
enum uac1_status_origin {
	UAC1_STATUS_AUDIO_CONTROL_INTERFACE = 0,
	UAC1_STATUS_AUDIO_STREAMING_INTERFACE = 1,
	UAC1_STATUS_AUDIO_STREAMING_ENDPOINT = 2,
};

/** AudioStreaming lifecycle events. */
enum uac1_stream_event {
	UAC1_STREAM_ACTIVATED,
	UAC1_STREAM_DEACTIVATED,
	UAC1_STREAM_SUSPENDED,
	UAC1_STREAM_RESUMED,
	UAC1_STREAM_DISABLED,
};

/**
 * @brief USB Audio 1 Feature Unit update
 *
 * Values in a multi-channel update are packed in increasing channel order.
 * Only channels whose bits are set in @ref channel_mask are present.
 */
struct uac1_feature_update {
	uint32_t channel_mask;
	const void *values;
	uint8_t value_size;
};

/**
 * @brief USB Audio 1 application event handlers
 */
struct uac1_ops {
	/**
	 * @brief Start of Frame callback
	 *
	 * Notifies application about SOF event on the bus.
	 * This callback is optional and executes in the cooperative USB device
	 * thread. It must not block.
	 *
	 * @param dev USB Audio 1 device
	 * @param user_data Opaque user data pointer
	 */
	void (*sof_cb)(const struct device *dev, void *user_data);

	/**
	 * @brief AudioStreaming lifecycle callback
	 *
	 * This callback executes in the cooperative USB device thread and must not
	 * block. The selected alternate setting is preserved across suspend and
	 * resume.
	 *
	 * @param dev USB Audio 1 device
	 * @param terminal Terminal ID linked to AudioStreaming interface
	 * @param event Stream lifecycle event
	 * @param microframes True if USB connection speed uses microframes
	 * @param user_data Opaque user data pointer
	 */
	void (*stream_event_cb)(const struct device *dev, uint8_t terminal,
				enum uac1_stream_event event, bool microframes,
				void *user_data);

	/**
	 * @brief Acquire an AudioStreaming OUT receive buffer
	 *
	 * USB stack calls this function to obtain receive buffer address for
	 * AudioStreaming ISO OUT endpoint. The class owns the buffer until
	 * @ref rx_buf_release callback is called. The buffer must be
	 * sufficiently aligned and otherwise suitable for use by UDC driver.
	 * This callback is mandatory to register for devices receiving USB
	 * audio from the USB host.
	 *
	 * @param dev USB Audio 1 device
	 * @param terminal Input Terminal ID linked to AudioStreaming interface
	 * @param size Maximum number of bytes USB stack will write to buffer
	 * @param buf Receives the application-owned buffer
	 * @param user_data Opaque user data pointer
	 *
	 * @retval 0 Buffer acquired and ownership transferred to the class.
	 * @retval -EAGAIN No buffer is currently available.
	 * @return Other negative errno on failure.
	 */
	int (*rx_buf_acquire)(const struct device *dev, uint8_t terminal,
			      uint16_t size, void **buf, void *user_data);

	/**
	 * @brief Release an AudioStreaming OUT receive buffer
	 *
	 * Releases a buffer obtained from @ref rx_buf_acquire. This callback is
	 * made exactly once for every successfully acquired buffer.
	 * This callback is mandatory to register for devices receiving USB
	 * audio from the USB host.
	 *
	 * @param dev USB Audio 1 device
	 * @param terminal Input Terminal ID linked to AudioStreaming interface
	 * @param buf Buffer previously obtained via @ref rx_buf_acquire
	 * @param size Number of bytes written to buffer
	 * @param status 0 on completion or negative errno on cancellation/failure
	 * @param user_data Opaque user data pointer
	 */
	void (*rx_buf_release)(const struct device *dev, uint8_t terminal,
			       void *buf, uint16_t size, int status, void *user_data);

	/**
	 * @brief Acquire an AudioStreaming IN transmit buffer
	 *
	 * The class calls this callback at the endpoint cadence. The callback must
	 * return a prepared buffer without blocking. On success, ownership is
	 * transferred to the class until @ref tx_buf_release is called.
	 *
	 * @param dev USB Audio 1 device
	 * @param terminal Output Terminal ID linked to AudioStreaming interface
	 * @param max_size Maximum packet size for the active bus speed
	 * @param service_number Monotonic packet service number for this stream
	 * @param buf Receives the application-owned buffer
	 * @param size Receives the number of bytes to transmit
	 * @param user_data Opaque user data pointer
	 *
	 * @retval 0 Buffer acquired and ownership transferred to the class.
	 * @retval -EAGAIN No packet is ready for this service interval.
	 * @return Other negative errno on failure.
	 */
	int (*tx_buf_acquire)(const struct device *dev, uint8_t terminal,
			      uint16_t max_size, uint32_t service_number,
			      void **buf, uint16_t *size, void *user_data);

	/**
	 * @brief Release an AudioStreaming IN transmit buffer
	 *
	 * This callback is made exactly once for every successfully acquired
	 * transmit buffer.
	 *
	 * @param dev USB Audio 1 device
	 * @param terminal Output Terminal ID linked to AudioStreaming interface
	 * @param buf Buffer previously obtained from @ref tx_buf_acquire
	 * @param status 0 on completion or negative errno on cancellation/failure
	 * @param user_data Opaque user data pointer
	 */
	void (*tx_buf_release)(const struct device *dev, uint8_t terminal,
			       void *buf, int status, void *user_data);

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
	 * @param update Atomic set of channel values accepted or rejected together
	 * @param user_data Opaque user data pointer
	 *
	 * @return 0 on success, negative errno to stall the control request.
	 */
	int (*feature_update_cb)(const struct device *dev, uint8_t unit_id,
				 uint8_t control_selector,
				 const struct uac1_feature_update *update,
				 void *user_data);
};

/**
 * @brief Register USB Audio 1 application callbacks.
 *
 * @param dev USB Audio 1 device instance
 * @param ops USB Audio 1 callback structure
 * @param user_data Opaque user data to pass to ops callbacks
 *
 * @retval 0 Callbacks registered.
 * @retval -EINVAL Missing callback required by the device topology.
 */
int usbd_uac1_set_ops(const struct device *dev,
		      const struct uac1_ops *ops, void *user_data);

/**
 * @brief Update a cached Feature Unit control from the device side.
 *
 * This is intended for controls changed outside USB, such as a physical mute
 * button or volume encoder. If the AudioControl interrupt endpoint is enabled,
 * a status notification is sent to the host.
 *
 * @param dev USB Audio 1 device.
 * @param unit_id Feature Unit ID.
 * @param control_selector Feature Unit control selector.
 * @param channel Channel number (0 = master).
 * @param value Pointer to the new control value.
 * @param value_len Length of @p value in bytes.
 *
 * @retval 0 Success.
 * @retval -EINVAL Invalid value or channel.
 * @retval -ENOENT Unknown Feature Unit.
 * @retval -ENOTSUP Control is not advertised.
 */
int usbd_uac1_feature_set(const struct device *dev, uint8_t unit_id, uint8_t control_selector,
			  uint8_t channel, const void *value, uint8_t value_len);

/**
 * @brief Send an AudioControl interrupt status notification.
 *
 * @param dev USB Audio 1 device.
 * @param origin Status originator type.
 * @param originator Entity ID, interface number, or endpoint number.
 * @param memory_changed True if the entity memory contents changed.
 *
 * @retval 0 Notification queued.
 * @retval -EAGAIN Another notification is pending.
 * @retval -ENOTSUP No AudioControl interrupt endpoint is configured.
 */
int usbd_uac1_status_notify(const struct device *dev, enum uac1_status_origin origin,
			    uint8_t originator, bool memory_changed);

/**
 * @}
 */

#ifdef __cplusplus
}
#endif

#endif /* ZEPHYR_INCLUDE_USB_CLASS_USBD_UAC1_H_ */
