/* Copyright (c) 2026 Ylhyra ehf. SPDX-License-Identifier: Apache-2.0 */
#ifndef ZEPHYR_DRIVERS_USB_UDC_DWC2_ISO_H_
#define ZEPHYR_DRIVERS_USB_UDC_DWC2_ISO_H_
#include <stdbool.h>
#include <stdint.h>
struct net_buf;
struct dwc2_iso_slot {
	struct net_buf *request;
	uint16_t target;
	uint32_t generation;
	struct net_buf *disable_request;
	uint32_t disable_generation;
	bool disabling;
};
/** Comparisons are valid within half a bus-frame epoch; suspend/reset retire the epoch. */
static inline bool dwc2_iso_due(uint16_t now, uint16_t target, uint16_t mask)
{
	return ((now - target) & mask) <= (mask >> 1);
}
static inline void dwc2_iso_retire(struct dwc2_iso_slot *slot)
{
	slot->request = 0;
	slot->generation++;
}
static inline bool dwc2_iso_begin_disable(struct dwc2_iso_slot *slot)
{
	if (slot->request == 0 || slot->disabling) {
		return false;
	}
	slot->disable_request = slot->request;
	slot->disable_generation = slot->generation;
	slot->disabling = true;
	return true;
}
/** 1: original request active, 2: original completed, 0: no intent, -1: identity lost. */
static inline int dwc2_iso_disable_state(const struct dwc2_iso_slot *slot)
{
	if (!slot->disabling) {
		return 0;
	}
	if (slot->request == slot->disable_request && slot->generation == slot->disable_generation) {
		return 1;
	}
	return slot->request == 0 && slot->generation == slot->disable_generation + 1U ? 2 : -1;
}
static inline void dwc2_iso_end_disable(struct dwc2_iso_slot *slot)
{
	slot->disabling = false;
	slot->disable_request = 0;
}
static inline void dwc2_iso_reset(struct dwc2_iso_slot *slot)
{
	dwc2_iso_retire(slot);
	dwc2_iso_end_disable(slot);
}
#endif
