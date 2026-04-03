/*
 * Copyright (c) 2026 Ylhyra ehf.
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_INCLUDE_DRIVERS_I2S_I2S_GD32_H_
#define ZEPHYR_INCLUDE_DRIVERS_I2S_I2S_GD32_H_

#include <stdint.h>
#include <zephyr/device.h>

/**
 * @brief Return the most recent TX DMA block start timestamp.
 *
 * The returned value is the system cycle counter value captured by the GD32 I2S
 * driver immediately before starting TX DMA for a new block.
 *
 * @param dev GD32 I2S device.
 *
 * @return Cycle counter value from k_cycle_get_32().
 */
uint32_t i2s_gd32_tx_block_start_cycle_get(const struct device *dev);

#endif /* ZEPHYR_INCLUDE_DRIVERS_I2S_I2S_GD32_H_ */
