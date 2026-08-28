/*
 * Copyright (c) 2022 Teslabs Engineering S.L.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_INCLUDE_DRIVERS_CLOCK_CONTROL_GD32_H_
#define ZEPHYR_INCLUDE_DRIVERS_CLOCK_CONTROL_GD32_H_

#include <stdbool.h>
#include <stdint.h>

#include <zephyr/device.h>

/**
 * @brief Obtain a reference to the GD32 clock controller.
 *
 * There is a single clock controller in the GD32: cctl. The device can be
 * used without checking for it to be ready since it has no initialization
 * code subject to failures.
 */
#define GD32_CLOCK_CONTROLLER DEVICE_DT_GET(DT_NODELABEL(cctl))

enum gd32_clock_config_type {
	GD32_CLOCK_CONFIG_TYPE_I2S = 1,
	GD32_CLOCK_CONFIG_TYPE_USB48M = 2,
};

enum gd32_i2s_clock_source {
	GD32_I2S_CLOCK_SRC_PLLI2S = 0,
	GD32_I2S_CLOCK_SRC_EXTERNAL = 1,
};

enum gd32_i2s_clock_action {
	/** Configure and reserve the global I2S clock for the requesting peripheral. */
	GD32_I2S_CLOCK_ACQUIRE,
	/** Release the requesting peripheral's reservation. */
	GD32_I2S_CLOCK_RELEASE,
};

/** Runtime I2S clock request and resulting peripheral divider. */
struct gd32_i2s_clock_config {
	/** Must be @ref GD32_CLOCK_CONFIG_TYPE_I2S. */
	enum gd32_clock_config_type type;
	/** Whether to acquire/configure or release the clock. */
	enum gd32_i2s_clock_action action;
	/** I2S kernel-clock source. */
	enum gd32_i2s_clock_source source;
	/** Requested audio frame rate. */
	uint32_t target_rate_hz;
	/** Maximum accepted frame-rate error; zero accepts the closest rate. */
	uint32_t max_error_ppm;
	/** Number of I2S kernel-clock cycles per serial divider and audio frame. */
	uint16_t frame_factor;
	/** External I2S kernel-clock rate, when that source is selected. */
	uint32_t external_rate_hz;
	/** Resulting peripheral divider value. */
	uint16_t divider;
	/** Resulting odd-divider bit. */
	bool odd;
	/** Rounded resulting audio frame rate. */
	uint32_t actual_rate_hz;
	/** Exact resulting audio frame-rate numerator. */
	uint64_t rate_num;
	/** Exact resulting audio frame-rate denominator. */
	uint32_t rate_den;
};

/**
 * @brief Request the 48 MHz USB clock (CK48M) to be sourced and stable.
 *
 * Drivers that need CK48M (typically the embedded full-speed USB PHY) pass
 * this struct to clock_control_configure(). The SoC-specific implementation
 * picks a route, programs any required PLLs, and waits until the resulting
 * source is stable. Returns 0 on success, negative errno otherwise.
 *
 * Empty payload so the ABI remains stable as future options (preferred source,
 * tolerance) are added.
 */
struct gd32_usb48m_clock_config {
	enum gd32_clock_config_type type;
};

#endif /* ZEPHYR_INCLUDE_DRIVERS_CLOCK_CONTROL_GD32_H_ */
