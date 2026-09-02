/*
 * Copyright (c) 2022 TOKITA Hiroshi <tokita.hiroshi@gmail.com>
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_INCLUDE_DRIVERS_DMA_DMA_GD32_H_
#define ZEPHYR_INCLUDE_DRIVERS_DMA_DMA_GD32_H_

#include <stdint.h>

#include <zephyr/device.h>

#define GD32_DMA_CONFIG_DIRECTION(config)	     ((config >> 6) & 0x3)
#define GD32_DMA_CONFIG_PERIPH_ADDR_INC(config)	     ((config >> 9) & 0x1)
#define GD32_DMA_CONFIG_MEMORY_ADDR_INC(config)	     ((config >> 10) & 0x1)
#define GD32_DMA_CONFIG_PERIPH_WIDTH(config)	     ((config >> 11) & 0x3)
#define GD32_DMA_CONFIG_MEMORY_WIDTH(config)	     ((config >> 13) & 0x3)
#define GD32_DMA_CONFIG_PERIPHERAL_INC_FIXED(config) ((config >> 15) & 0x1)
#define GD32_DMA_CONFIG_PRIORITY(config)	     ((config >> 16) & 0x3)

#define GD32_DMA_FEATURES_FIFO_THRESHOLD(threshold) ((threshold) & 0x3U)
#define GD32_DMA_FEATURES_FIFO_REQUEST	      0x4U
#define GD32_DMA_FEATURES_FIFO_REQUESTED(fifo_mode_control)                                   \
	((((fifo_mode_control) & GD32_DMA_FEATURES_FIFO_REQUEST) != 0U) ||                   \
	 (GD32_DMA_FEATURES_FIFO_THRESHOLD(fifo_mode_control) != 0U))
#define GD32_DMA_DT_FIFO_MODE(threshold)                                                     \
	(GD32_DMA_FEATURES_FIFO_THRESHOLD(threshold) | GD32_DMA_FEATURES_FIFO_REQUEST)

/**
 * @brief Read the memory bank currently used by a GD32 switch-buffer channel.
 *
 * @param dev DMA controller device.
 * @param channel DMA channel number.
 * @param active_bank Destination for the active bank index (zero or one).
 *
 * @retval 0 on success.
 * @retval -EINVAL for an invalid argument or unconfigured channel.
 * @retval -ENOTSUP if the channel is not configured for switch-buffer mode.
 */
int dma_gd32_switch_buffer_active(const struct device *dev, uint32_t channel,
				  uint8_t *active_bank);

#endif /* ZEPHYR_INCLUDE_DRIVERS_DMA_DMA_GD32_H_ */
