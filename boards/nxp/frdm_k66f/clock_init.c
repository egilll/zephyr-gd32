/*
 * SPDX-License-Identifier: Apache-2.0
 *
 * Override the __weak clock_init() from zephyr/soc/nxp/kinetis/k6x/soc.c.
 *
 * The board defconfig selects CONFIG_OSC_EXTERNAL=y, which puts the K66F SoC
 * code into "bypass" mode expecting a square-wave clock from the OpenSDA MCU.
 * The J-Link OB firmware on the FRDM-K66F does NOT provide that clock.
 *
 * This override uses the on-board 12 MHz crystal (X501) in oscillator mode
 * and boots the MCG to PEE (PLL Engaged External) at 180 MHz:
 *
 *   PLL ref   = 12 MHz / 1   = 12 MHz   (PRDIV=0, within 8-16 MHz range)
 *   VCO       = 12 MHz * 30  = 360 MHz  (VDIV=0xE, within 180-360 MHz range)
 *   MCGPLLCLK = 360 / 2      = 180 MHz  (hardware /2 post-divider)
 *
 * SIM_CLKDIV1 dividers:
 *   Core  / 1  = 180 MHz   (max 180 MHz in HSRUN)
 *   Bus   / 3  =  60 MHz   (max  60 MHz)
 *   FB    / 3  =  60 MHz   (max  60 MHz)
 *   Flash / 7  = ~25.7 MHz (max ~28 MHz)
 *
 * SDHC clock is routed to OSCERCLK (12 MHz).  Using the 180 MHz core
 * clock causes bus faults on SD writes -- root cause under investigation.
 *
 * ENET RMII clock is sourced from the external 50 MHz oscillator (U13) on
 * PTE26 (ENET_1588_CLKIN).  The 1588 timestamp clock uses OSCERCLK.
 */

#include <fsl_clock.h>

static const osc_config_t oscConfig = {
	.freq = 12000000U,
	.capLoad = 0U,
	.workMode = kOSC_ModeOscLowPower,
	.oscerConfig = {
		.enableMode = kOSC_ErClkEnable,
#if (defined(FSL_FEATURE_OSC_HAS_EXT_REF_CLOCK_DIVIDER) && \
	FSL_FEATURE_OSC_HAS_EXT_REF_CLOCK_DIVIDER)
		.erclkDiv = 0U,
#endif
	},
};

static const mcg_pll_config_t pll0Config = {
	.enableMode = 0U,
	.prdiv = 0x0U,	/* Divide by 1 → PLL ref = 12 MHz */
	.vdiv = 0xEU,	/* Multiply by 30 → VCO = 360 MHz → PLL out = 180 MHz */
};

void clock_init(void)
{
	CLOCK_SetSimSafeDivs();

	/* Initialize OSC0 with the 12 MHz crystal in low-power oscillator mode. */
	CLOCK_InitOsc0(&oscConfig);
	CLOCK_SetXtal0Freq(oscConfig.freq);

	/* Boot MCG through FEI → FBE → PBE → PEE. */
	CLOCK_BootToPeeMode(kMCG_OscselOsc, kMCG_PllClkSelPll0, &pll0Config);

	/* Keep slow IRC enabled for MCGIRCLK (used by some peripherals). */
	CLOCK_SetInternalRefClkConfig(kMCG_IrclkEnable, kMCG_IrcSlow, 1U);

	/* Set final bus dividers:
	 *   Core  = /1  (180 MHz)
	 *   Bus   = /3  ( 60 MHz)
	 *   FB    = /3  ( 60 MHz)
	 *   Flash = /7  (~25.7 MHz)
	 */
	SIM->CLKDIV1 = SIM_CLKDIV1_OUTDIV1(0U) |
		       SIM_CLKDIV1_OUTDIV2(2U) |
		       SIM_CLKDIV1_OUTDIV3(2U) |
		       SIM_CLKDIV1_OUTDIV4(6U);

	/* Route SDHC clock to OSCERCLK (12 MHz).
	 * The overlay limits max-bus-freq to 6 MHz to match this source.
	 * Using the 180 MHz core clock causes immediate bus faults on the
	 * first SD write — root cause is still under investigation.
	 */
	SIM->SOPT2 = (SIM->SOPT2 & ~SIM_SOPT2_SDHCSRC_MASK) |
		     SIM_SOPT2_SDHCSRC(2U);

	/* ENET: 1588 timestamp clock from OSCERCLK, RMII ref clock from
	 * external 50 MHz oscillator (U13) via ENET_1588_CLKIN (PTE26).
	 * Note: these are also set by the default soc.c when
	 * CONFIG_ETH_NXP_ENET / CONFIG_ETH_NXP_ENET_RMII_EXT_CLK are enabled,
	 * but since we override clock_init we must replicate them here.
	 */
	CLOCK_SetEnetTime0Clock(2U);  /* TIMESRC = OSCERCLK */
	CLOCK_SetRmii0Clock(1U);     /* RMIISRC = ENET_1588_CLKIN */
}
