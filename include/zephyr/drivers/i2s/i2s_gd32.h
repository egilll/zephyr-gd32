/* Copyright (c) 2026 Ylhyra ehf. SPDX-License-Identifier: Apache-2.0 */
#ifndef ZEPHYR_INCLUDE_DRIVERS_I2S_GD32_H_
#define ZEPHYR_INCLUDE_DRIVERS_I2S_GD32_H_
#include <zephyr/device.h>
#include <zephyr/drivers/i2s.h>
#ifdef __cplusplus
extern "C" {
#endif
struct i2s_gd32_diagnostics {
	uint32_t generation;
	uint32_t silent_frames;
	uint32_t discarded_frames;
	uint32_t maximum_service_cycles;
	uint32_t minimum_margin_cycles;
	uint32_t first_fault_cycle;
};
typedef void (*i2s_gd32_service_callback_t)(const struct device *dev, void *user_data);
/** Nonblocking ISR notification after DMA progress or a fault; caller owns callback lifetime. */
int i2s_gd32_service_callback_set(const struct device *dev,
				i2s_gd32_service_callback_t callback, void *user_data);
/** Snapshot counters for the current configuration; first fault persists until reconfigure. */
int i2s_gd32_diagnostics_get(const struct device *dev, enum i2s_dir dir,
			   struct i2s_gd32_diagnostics *diagnostics);
#ifdef __cplusplus
}
#endif
#endif
