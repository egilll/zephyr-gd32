/*
 * Copyright (c) 2022 Teslabs Engineering S.L.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_INCLUDE_DRIVERS_CLOCK_CONTROL_GD32_H_
#define ZEPHYR_INCLUDE_DRIVERS_CLOCK_CONTROL_GD32_H_

#include <zephyr/device.h>
#include <stdint.h>

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
};

enum gd32_i2s_clock_source {
	GD32_I2S_CLOCK_SRC_PLLI2S = 0,
	GD32_I2S_CLOCK_SRC_EXTERNAL = 1,
};

struct gd32_i2s_clock_config {
	enum gd32_clock_config_type type;
	enum gd32_i2s_clock_source source;
	uint32_t external_rate_hz;
	uint16_t plli2s_n;
	uint8_t plli2s_r;
	uint32_t actual_rate_hz;
};

#endif /* ZEPHYR_INCLUDE_DRIVERS_CLOCK_CONTROL_GD32_H_ */
