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

#define FRDM_K66F_PLLFLLSEL_IRC48MCLK 3U
#define FRDM_K66F_SDHC_SRC_OSCERCLK   2U
#define FRDM_K66F_ENET_TIME_SRC_CORE  0U
#define FRDM_K66F_RMII_SRC_CLKIN      1U
#define FRDM_K66F_LPUART_SRC_OSCERCLK 2U
#define FRDM_K66F_TPM_SRC_PLLFLLSEL   1U
#define FRDM_K66F_CLKOUT_SRC_FLEXBUS  0U
#define FRDM_K66F_TRACE_SRC_MCGOUTCLK 0U
#define FRDM_K66F_TRACE_DIV_2         1U
#define FRDM_K66F_TRACE_FRAC_1        0U
#define FRDM_K66F_RTC_CLKOUT_1HZ      0U
#define FRDM_K66F_USB_CLK_HZ          48000000U

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

	/* Match the old SDK board clocks: keep MCGIRCLK enabled before PEE setup. */
	CLOCK_SetInternalRefClkConfig(kMCG_IrclkEnable, kMCG_IrcSlow, 1U);

	/* Boot MCG through FEI → FBE → PBE → PEE. */
	CLOCK_BootToPeeMode(kMCG_OscselOsc, kMCG_PllClkSelPll0, &pll0Config);

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

	/* Match the old SDK SIM clock-source configuration rather than relying on
	 * reset defaults or piecemeal SOPT2 updates.
	 */
	CLOCK_SetPllFllSelClock(FRDM_K66F_PLLFLLSEL_IRC48MCLK, 0U, 0U);
	CLOCK_SetRtcClkOutClock(FRDM_K66F_RTC_CLKOUT_1HZ);
	CLOCK_EnableUsbfs0Clock(kCLOCK_UsbSrcIrc48M, FRDM_K66F_USB_CLK_HZ);
	CLOCK_SetEnetTime0Clock(FRDM_K66F_ENET_TIME_SRC_CORE);
	CLOCK_SetRmii0Clock(FRDM_K66F_RMII_SRC_CLKIN);
	CLOCK_SetSdhc0Clock(FRDM_K66F_SDHC_SRC_OSCERCLK);
	CLOCK_SetLpuartClock(FRDM_K66F_LPUART_SRC_OSCERCLK);
	CLOCK_SetTpmClock(FRDM_K66F_TPM_SRC_PLLFLLSEL);
	CLOCK_SetClkOutClock(FRDM_K66F_CLKOUT_SRC_FLEXBUS);
	CLOCK_SetTraceClock(FRDM_K66F_TRACE_SRC_MCGOUTCLK, FRDM_K66F_TRACE_DIV_2,
			    FRDM_K66F_TRACE_FRAC_1);

	SystemCoreClock = CLOCK_GetFreq(kCLOCK_CoreSysClk);
}
