/*
 * Copyright (c) 2026 Ylhyra ehf.
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_INCLUDE_DRIVERS_USB_UDC_DWC2_H_
#define ZEPHYR_INCLUDE_DRIVERS_USB_UDC_DWC2_H_

#include <stdint.h>
#include <stdbool.h>
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
#ifdef __cplusplus
extern "C" {
#endif

uint32_t udc_dwc2_sof_cycle_get(const struct device *dev);

struct udc_dwc2_diagnostics {
	uint32_t rx_read_attempts;
	uint32_t rx_read_max_attempts;
	uint32_t rx_read_failures;
	uint32_t maximum_isr_cycles;
	uint32_t active_iso_requests;
	uint32_t feedback_completions;
	uint32_t feedback_gap_one;
	uint32_t feedback_gap_two;
	uint32_t feedback_gap_larger;
	uint32_t feedback_gap_max;
	/** Milliseconds since a successful feedback transfer, UINT32_MAX if none. */
	uint32_t feedback_age_ms;
	bool quarantined;
};
int udc_dwc2_diagnostics_get(const struct device *dev, struct udc_dwc2_diagnostics *out);

#if defined(CONFIG_UDC_DWC2_TEST_HOOKS)
int udc_dwc2_test_quarantine(const struct device *dev, bool fail_reset);
void udc_dwc2_test_reset_failure_set(const struct device *dev, bool fail_reset);
#endif
#ifdef __cplusplus
}
#endif

#endif /* ZEPHYR_INCLUDE_DRIVERS_USB_UDC_DWC2_H_ */
