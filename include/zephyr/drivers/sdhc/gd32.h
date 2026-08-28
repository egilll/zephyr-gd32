/*
 * Copyright (c) 2026, Ylhyra ehf.
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_INCLUDE_DRIVERS_SDHC_GD32_H_
#define ZEPHYR_INCLUDE_DRIVERS_SDHC_GD32_H_

#ifdef __cplusplus
extern "C" {
#endif

/** Suppress the next DMA completion callback to exercise hardware-count polling. */
void gd32_sdhc_test_drop_next_dma_callback(void);

/** Mask the next transfer's terminal SDIO interrupts to exercise status polling. */
void gd32_sdhc_test_mask_next_terminal_interrupts(void);

/** Mask only the next transfer's DTBLKEND interrupt. */
void gd32_sdhc_test_mask_next_dtblkend_interrupt(void);

/** Inject a transient data CRC failure into the next active data transfer. */
void gd32_sdhc_test_inject_next_data_crc_error(void);

#ifdef __cplusplus
}
#endif

#endif /* ZEPHYR_INCLUDE_DRIVERS_SDHC_GD32_H_ */
