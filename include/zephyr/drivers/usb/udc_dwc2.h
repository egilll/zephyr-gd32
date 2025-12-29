/*
 * Copyright (c) 2026 Ylhyra ehf.
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_INCLUDE_DRIVERS_USB_UDC_DWC2_H_
#define ZEPHYR_INCLUDE_DRIVERS_USB_UDC_DWC2_H_

#include <stdint.h>
#include <zephyr/device.h>

/**
 * @brief Return the most recent SOF timestamp captured in the DWC2 ISR.
 *
 * The returned value is the system cycle counter value captured by the DWC2
 * UDC driver when handling the SOF interrupt.
 *
 * @param dev DWC2 UDC device.
 *
 * @return Cycle counter value from k_cycle_get_32().
 */
uint32_t udc_dwc2_sof_cycle_get(const struct device *dev);

#endif /* ZEPHYR_INCLUDE_DRIVERS_USB_UDC_DWC2_H_ */
