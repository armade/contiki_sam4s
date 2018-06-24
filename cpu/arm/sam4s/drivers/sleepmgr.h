/**
 * \file
 *
 * \brief SAM3/SAM4 Sleep manager implementation.
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

#ifndef SAM_SLEEPMGR_INCLUDED
#define SAM_SLEEPMGR_INCLUDED

#ifdef __cplusplus
extern "C" {
#endif

#include <sam4s.h>
#include <sam_sleep.h>
#include <interrupt_sam_nvic.h>

/**
 * \weakgroup sleepmgr_group
 * @{
 */
enum sleepmgr_mode {
	//! Active mode.
	SLEEPMGR_ACTIVE = 0,
	/*! WFE sleep mode.
	 *  Potential Wake Up sources:
	 *  fast startup events (USB, RTC, RTT, WKUPs),
	 *  interrupt, and events. */
	SLEEPMGR_SLEEP_WFE,
	/*! WFI sleep mode.
	 * Potential Wake Up sources: fast startup events and interrupt. */
	SLEEPMGR_SLEEP_WFI,
	/*! Wait mode, wakeup fast (in 3ms).
	 *  XTAL is not disabled when sleep.
	 *  Potential Wake Up sources: fast startup events */
	SLEEPMGR_WAIT_FAST,
	/*! Wait mode.
	 *  Potential Wake Up sources: fast startup events */
	SLEEPMGR_WAIT,
#if (!(SAMG51 || SAMG53 || SAMG54))
	//! Backup mode. Potential Wake Up sources: WKUPs, SM, RTT, RTC.
	SLEEPMGR_BACKUP,
#endif
	SLEEPMGR_NR_OF_MODES,
};

/**
 * \internal
 * \name Internal arrays
 * @{
 */
//! Sleep mode lock counters
extern uint8_t sleepmgr_locks[SLEEPMGR_NR_OF_MODES];


static inline void sleepmgr_init(void)
{
	uint8_t i;

	for (i = 0; i < SLEEPMGR_NR_OF_MODES - 1; i++) {
		sleepmgr_locks[i] = 0;
	}
	sleepmgr_locks[SLEEPMGR_NR_OF_MODES - 1] = 1;
}

static inline void sleepmgr_lock_mode(enum sleepmgr_mode mode)
{
	irqflags_t flags;

	if(sleepmgr_locks[mode] >= 0xff) {
		while (1) {
			// Warning: maximum value of sleepmgr_locks buffer is no more than 255.
			// Check APP or change the data type to uint16_t.
		}
	}

	// Enter a critical section
	flags = cpu_irq_save();

	++sleepmgr_locks[mode];

	// Leave the critical section
	cpu_irq_restore(flags);
}

static inline void sleepmgr_unlock_mode(enum sleepmgr_mode mode)
{
	irqflags_t flags;

	if(sleepmgr_locks[mode] == 0) {
		while (1) {
			// Warning: minimum value of sleepmgr_locks buffer is no less than 0.
			// Check APP.
		}
	}

	// Enter a critical section
	flags = cpu_irq_save();

	--sleepmgr_locks[mode];

	// Leave the critical section
	cpu_irq_restore(flags);
}

/**
 * \internal
 * \name Internal arrays
 * @{
 */
//! Sleep mode lock counters
extern uint8_t sleepmgr_locks[];
//! @}


static inline void sleepmgr_sleep(const enum sleepmgr_mode sleep_mode)
{
	cpu_irq_disable();

	// Atomically enable the global interrupts and enter the sleep mode.
	pmc_sleep(sleep_mode);

}
static inline enum sleepmgr_mode sleepmgr_get_sleep_mode(void)
{
	enum sleepmgr_mode sleep_mode = SLEEPMGR_ACTIVE;

	uint8_t *lock_ptr = sleepmgr_locks;

	// Find first non-zero lock count, starting with the shallowest modes.
	while (!(*lock_ptr)) {
		lock_ptr++;
		sleep_mode = (enum sleepmgr_mode)(sleep_mode + 1);
	}



	return sleep_mode;
}

static inline void sleepmgr_enter_sleep(void)
{
	enum sleepmgr_mode sleep_mode;

	cpu_irq_disable();

	// Find the deepest allowable sleep mode
	sleep_mode = sleepmgr_get_sleep_mode();
	// Return right away if first mode (ACTIVE) is locked.
	if (sleep_mode==SLEEPMGR_ACTIVE) {
		cpu_irq_enable();
		return;
	}
	// Enter the deepest allowable sleep mode with interrupts enabled
	sleepmgr_sleep(sleep_mode);
}

//! @}

#ifdef __cplusplus
}
#endif

#endif /* SAM_SLEEPMGR_INCLUDED */
