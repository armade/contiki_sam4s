/**
 * \file
 *
 * \brief Sleep mode access
 *
 * Copyright (c) 2012-2016 Atmel Corporation. All rights reserved.
 *
 * \asf_license_start
 *
 * \page License
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * 3. The name of Atmel may not be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * 4. This software may only be redistributed and used in connection with an
 *    Atmel microcontroller product.
 *
 * THIS SOFTWARE IS PROVIDED BY ATMEL "AS IS" AND ANY EXPRESS OR IMPLIED
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT ARE
 * EXPRESSLY AND SPECIFICALLY DISCLAIMED. IN NO EVENT SHALL ATMEL BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 * STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 * \asf_license_stop
 *
 */
/*
 * Support and FAQ: visit <a href="http://www.atmel.com/design-support/">Atmel Support</a>
 */

#include "sam_sleep.h"
#include <interrupt_sam_nvic.h>
#include <sam4s.h>

/* SAM3,SAM4,SAMG,SAMV,SAMS and SAME series */
# include "pmc.h"

# define BOARD_OSC_STARTUP_US    (15625UL)

#if !defined(EFC0)
# define EFC0 EFC
#endif

/**
 * Save clock settings and shutdown PLLs
 */
static void pmc_save_clock_settings(
		uint32_t *p_osc_setting,
		uint32_t *p_pll0_setting,
		uint32_t *p_pll1_setting,
		uint32_t *p_mck_setting,
		uint32_t *p_fmr_setting,
#if defined(EFC1)
		uint32_t *p_fmr_setting1,
#endif
		const char disable_xtal)
{
	uint32_t mor  = PMC->CKGR_MOR;
	uint32_t mckr = PMC->PMC_MCKR;
	uint32_t fmr  = EFC0->EEFC_FMR;
# if defined(EFC1)
	uint32_t fmr1 = EFC1->EEFC_FMR;
# endif

	if (p_osc_setting) {
		*p_osc_setting = mor;
	}
	if (p_pll0_setting) {
		*p_pll0_setting = PMC->CKGR_PLLAR;
	}
	if (p_pll1_setting) {
		*p_pll1_setting = PMC->CKGR_PLLBR;
	}
	if (p_mck_setting) {
		*p_mck_setting  = mckr;
	}
	if (p_fmr_setting) {
		*p_fmr_setting  = fmr;
	}
#if defined(EFC1)
	if (p_fmr_setting1) {
		*p_fmr_setting1 = fmr1;
	}
#endif

	/* Enable FAST RC */
	PMC->CKGR_MOR = CKGR_MOR_KEY_PASSWD | mor | CKGR_MOR_MOSCRCEN;
	/* if MCK source is PLL, switch to mainck */
	if ((mckr & PMC_MCKR_CSS_Msk) > PMC_MCKR_CSS_MAIN_CLK) {
		/* MCK -> MAINCK */
		mckr = (mckr & (~PMC_MCKR_CSS_Msk)) | PMC_MCKR_CSS_MAIN_CLK;
		PMC->PMC_MCKR = mckr;
		while(!(PMC->PMC_SR & PMC_SR_MCKRDY));
	}
	/* MCK prescale -> 1 */
	if (mckr & PMC_MCKR_PRES_Msk) {
		mckr = (mckr & (~PMC_MCKR_PRES_Msk));
		PMC->PMC_MCKR = mckr;
		while(!(PMC->PMC_SR & PMC_SR_MCKRDY));
	}
	/* Disable PLLs */
	pmc_disable_pllack();
	pmc_disable_pllbck();

	/* Prepare for entering WAIT mode */
	/* Wait fast RC ready */
	while (!(PMC->PMC_SR & PMC_SR_MOSCRCS));

	/* Switch mainck to FAST RC */
	PMC->CKGR_MOR = (PMC->CKGR_MOR & ~CKGR_MOR_MOSCSEL) |
			CKGR_MOR_KEY_PASSWD;
	while (!(PMC->PMC_SR & PMC_SR_MOSCSELS));

	/* FWS update */
	EFC0->EEFC_FMR = fmr & (~EEFC_FMR_FWS_Msk);
#if defined(EFC1)
	EFC1->EEFC_FMR = fmr1 & (~EEFC_FMR_FWS_Msk);
#endif

	/* Disable XTALs */
	if (disable_xtal) {
		PMC->CKGR_MOR = (PMC->CKGR_MOR & ~CKGR_MOR_MOSCXTEN) |
				CKGR_MOR_KEY_PASSWD;
	}
}

/**
 * Restore clock settings
 */
static void pmc_restore_clock_setting(
		const uint32_t osc_setting,
		const uint32_t pll0_setting,
		const uint32_t pll1_setting,
		const uint32_t mck_setting,
		const uint32_t fmr_setting
#if defined(EFC1)
		, const uint32_t fmr_setting1
#endif
		)
{
	uint32_t mckr;
	uint32_t pll_sr = 0;

	/* Switch mainck to external xtal */
	if (CKGR_MOR_MOSCXTBY == (osc_setting & CKGR_MOR_MOSCXTBY)) {
		/* Bypass mode */
		PMC->CKGR_MOR = (PMC->CKGR_MOR & ~CKGR_MOR_MOSCXTEN) |
				CKGR_MOR_KEY_PASSWD | CKGR_MOR_MOSCXTBY |
				CKGR_MOR_MOSCSEL;
		PMC->CKGR_MOR = (PMC->CKGR_MOR & ~CKGR_MOR_MOSCRCEN &
					~CKGR_MOR_MOSCRCF_Msk)
				| CKGR_MOR_KEY_PASSWD;
	} else if (CKGR_MOR_MOSCXTEN == (osc_setting & CKGR_MOR_MOSCXTEN)) {
		/* Enable External XTAL */
		if (!(PMC->CKGR_MOR & CKGR_MOR_MOSCXTEN)) {
			PMC->CKGR_MOR = (PMC->CKGR_MOR & ~CKGR_MOR_MOSCXTBY) |
					CKGR_MOR_KEY_PASSWD | CKGR_MOR_MOSCXTEN;
			/* Wait the Xtal to stabilize */
			while (!(PMC->PMC_SR & PMC_SR_MOSCXTS));
		}
		/* Select External XTAL */
		if (!(PMC->CKGR_MOR & CKGR_MOR_MOSCSEL)) {
			PMC->CKGR_MOR |= CKGR_MOR_KEY_PASSWD | CKGR_MOR_MOSCSEL;
			while (!(PMC->PMC_SR & PMC_SR_MOSCSELS));
		}
		/* Disable Fast RC */
		PMC->CKGR_MOR = (PMC->CKGR_MOR & ~CKGR_MOR_MOSCRCEN &
						~CKGR_MOR_MOSCRCF_Msk)
					| CKGR_MOR_KEY_PASSWD;
	}

	if (pll0_setting & CKGR_PLLAR_MULA_Msk) {
		PMC->CKGR_PLLAR = CKGR_PLLAR_ONE | pll0_setting;
		pll_sr |= PMC_SR_LOCKA;
	}
	if (pll1_setting & CKGR_PLLBR_MULB_Msk) {
		PMC->CKGR_PLLBR = pll1_setting;
		pll_sr |= PMC_SR_LOCKB;
	}
	/* Wait MCK source ready */
	switch(mck_setting & PMC_MCKR_CSS_Msk) {
	case PMC_MCKR_CSS_PLLA_CLK:
		while (!(PMC->PMC_SR & PMC_SR_LOCKA));
		break;
	case PMC_MCKR_CSS_PLLB_CLK:
		while (!(PMC->PMC_SR & PMC_SR_LOCKB));
		break;
	}

	/* Switch to faster clock */
	mckr = PMC->PMC_MCKR;

	/* Set PRES */
	PMC->PMC_MCKR = (mckr & ~PMC_MCKR_PRES_Msk)
		| (mck_setting & PMC_MCKR_PRES_Msk);
	while (!(PMC->PMC_SR & PMC_SR_MCKRDY));

	/* Restore flash wait states */
	EFC0->EEFC_FMR = fmr_setting;
#if defined(EFC1)
	EFC1->EEFC_FMR = fmr_setting1;
#endif

	/* Set CSS and others */
	PMC->PMC_MCKR = mck_setting;
	while (!(PMC->PMC_SR & PMC_SR_MCKRDY));

	/* Waiting all restored PLLs ready */
	while (!(PMC->PMC_SR & pll_sr));
}

/** If clocks are switched for some sleep mode */
static volatile char b_is_sleep_clock_used = 0;
/** Callback invoked once when clocks are restored */

void pmc_sleep(int sleep_mode)
{
	switch (sleep_mode) {
	case SAM_PM_SMODE_SLEEP_WFI:
	case SAM_PM_SMODE_SLEEP_WFE:
		SCB->SCR &= (uint32_t)~SCR_SLEEPDEEP;
		cpu_irq_enable();
		__DSB();
		__WFI();
		break;

	case SAM_PM_SMODE_WAIT_FAST:
	case SAM_PM_SMODE_WAIT: {
		uint32_t mor, pllr0, pllr1, mckr;
		uint32_t fmr;
#if defined(EFC1)
		uint32_t fmr1;
#endif
		(sleep_mode == SAM_PM_SMODE_WAIT_FAST) ?
				pmc_set_flash_in_wait_mode(PMC_FSMR_FLPM_FLASH_STANDBY) :
				pmc_set_flash_in_wait_mode(PMC_FSMR_FLPM_FLASH_DEEP_POWERDOWN);
		cpu_irq_disable();
		b_is_sleep_clock_used = 1;

		pmc_save_clock_settings(&mor, &pllr0, &pllr1, &mckr, &fmr,
#if defined(EFC1)
				&fmr1,
#endif
				(sleep_mode == SAM_PM_SMODE_WAIT));

		/* Enter wait mode */
		cpu_irq_enable();

		pmc_enable_waitmode();

		cpu_irq_disable();
		pmc_restore_clock_setting(mor, pllr0, pllr1, mckr, fmr
#if defined(EFC1)
				, fmr1
#endif
				);

		b_is_sleep_clock_used = 0;
		cpu_irq_enable();

		break;
	}
	case SAM_PM_SMODE_BACKUP:
		SCB->SCR |= SCR_SLEEPDEEP;
		SUPC->SUPC_CR = SUPC_CR_KEY_PASSWD | SUPC_CR_VROFF_STOP_VREG;
		cpu_irq_enable();
		__WFI() ;
		break;
	}
}

char pmc_is_wakeup_clocks_restored(void)
{
	return !b_is_sleep_clock_used;
}


