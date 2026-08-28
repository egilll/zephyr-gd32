/*
 * Copyright (c) 2026 Ylhyra ehf.
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_INCLUDE_DRIVERS_SDHC_KINETIS_H_
#define ZEPHYR_INCLUDE_DRIVERS_SDHC_KINETIS_H_

#include <zephyr/device.h>

#ifdef __cplusplus
extern "C" {
#endif

/** Mask the next data transfer's terminal interrupt to exercise status polling. */
void sdhc_kinetis_test_mask_next_terminal_interrupt(const struct device *dev);

/** Force a DMA error during the next DMA transfer. */
void sdhc_kinetis_test_force_next_dma_error(const struct device *dev);

/** Force the next otherwise-DMA-eligible request to use the data port. */
void sdhc_kinetis_test_force_next_pio(const struct device *dev);

/** Force DTOE when the next data transfer latches TC. */
void sdhc_kinetis_test_force_next_tc_dtoe(const struct device *dev);

/** Return the number of DMA failures which consumed a retry as a PIO fallback. */
uint32_t sdhc_kinetis_test_pio_fallback_count(const struct device *dev);

#ifdef __cplusplus
}
#endif

#endif /* ZEPHYR_INCLUDE_DRIVERS_SDHC_KINETIS_H_ */
