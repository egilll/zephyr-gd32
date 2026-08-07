/*
 * Copyright (c) 2022, Teslabs Engineering S.L.
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/devicetree.h>
#include <zephyr/init.h>
#include <soc.h>

#include <gd32e50x_rcu.h>

static void gd32e50x_ckout0_init(void)
{
#if DT_NODE_HAS_PROP(DT_NODELABEL(rcu), gd_ckout0_source)
	BUILD_ASSERT(DT_ENUM_IDX(DT_NODELABEL(rcu), gd_ckout0_source) == 1,
		     "GD32E50x CK_OUT0 supports the pll2 devicetree source");
	BUILD_ASSERT(DT_PROP(DT_NODELABEL(rcu), gd_ckout0_pll2_multiplier) == 10,
		     "GD32E50x Ethernet CK_OUT0 requires PLL2 multiplication by 10");

	rcu_pll2_config(RCU_PLL2_MUL10);
	rcu_osci_on(RCU_PLL2_CK);
	if (rcu_osci_stab_wait(RCU_PLL2_CK) == SUCCESS) {
		rcu_ckout0_config(RCU_CKOUT0SRC_CKPLL2);
	}
#endif
}

void soc_early_init_hook(void)
{
	SystemInit();
	gd32e50x_ckout0_init();
}
