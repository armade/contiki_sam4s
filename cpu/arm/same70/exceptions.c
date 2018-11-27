/* ---------------------------------------------------------------------------- */
/*                  Atmel Microcontroller Software Support                      */
/*                       SAM Software Package License                           */
/* ---------------------------------------------------------------------------- */
/* Copyright (c) 2015, Atmel Corporation                                        */
/*                                                                              */
/* All rights reserved.                                                         */
/*                                                                              */
/* Redistribution and use in source and binary forms, with or without           */
/* modification, are permitted provided that the following condition is met:    */
/*                                                                              */
/* - Redistributions of source code must retain the above copyright notice,     */
/* this list of conditions and the disclaimer below.                            */
/*                                                                              */
/* Atmel's name may not be used to endorse or promote products derived from     */
/* this software without specific prior written permission.                     */
/*                                                                              */
/* DISCLAIMER:  THIS SOFTWARE IS PROVIDED BY ATMEL "AS IS" AND ANY EXPRESS OR   */
/* IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF */
/* MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT ARE   */
/* DISCLAIMED. IN NO EVENT SHALL ATMEL BE LIABLE FOR ANY DIRECT, INDIRECT,      */
/* INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT */
/* LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA,  */
/* OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF    */
/* LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING         */
/* NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, */
/* EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.                           */
/* ---------------------------------------------------------------------------- */

/**
 * \file
 * This file contains the default exception handlers.
 *
 * \note
 * The exception handler has weak aliases.
 * As they are weak aliases, any function with the same name will override
 * this definition.
 */

/*----------------------------------------------------------------------------
 *        Headers
 *----------------------------------------------------------------------------*/

#include "chip.h"
#include <stdio.h>
/*----------------------------------------------------------------------------
 *        Exported functions
 *----------------------------------------------------------------------------*/
extern void Dummy_Handler(void);
/**
 * \brief Default NMI interrupt handler.
 */
void NMI_Handler(void)
{
	printf("E - Enter NMI_Handler!\n");

	//while (1);
	//__ASM volatile("BKPT #01");
	Dummy_Handler();
}

/**
 * \brief This function back trace the stack to give exact address where fault
 happened
**/
__STATIC_INLINE uint32_t StackUnwind(void)
{
	uint32_t Fault_Add;

#if defined (__CC_ARM)
	uint32_t temp;
	__ASM("mrs temp, msp ");
	__ASM{ ldr Fault_Add, [temp, #28]}
#else
	__ASM("mrs r0, msp ");
	__ASM("ldr %0, [r0,#28]" : "=r" (Fault_Add));
#endif
	return Fault_Add;
}

/**
 * \brief If Other Faults are enabled then HardFault error will look for those
 *  errors to give more detail about fault
**/
static void HardFault_reason(void)
{
	uint32_t CFSRValue;
	printf("E -  Hard Fault Handler\n");
	printf("SCB->HFSR = 0x%08x\n", (unsigned int)SCB->HFSR);

	if ((SCB->HFSR & SCB_HFSR_DEBUGEVT_Msk)) {
		printf("Debug Event Hard Fault\n");
		printf("SCB->DFSR = 0x%08x\n", (unsigned int)SCB->DFSR);
	}

	if ((SCB->HFSR & SCB_HFSR_VECTTBL_Msk)) {
		printf("Fault was due to vector table read on \
			exception processing\n");
	}

	// Forced HardFault
	if ((SCB->HFSR & SCB_HFSR_FORCED_Msk)) {
		printf("Forced Hard Fault\n");
		printf("SCB->CFSR = 0x%08x\n", (unsigned int)SCB->CFSR);

		// Usage Fault
		if ((SCB->CFSR & SCB_CFSR_USGFAULTSR_Msk)) {
			CFSRValue = SCB->CFSR;
			printf("Usage fault: \n");
			CFSRValue >>= SCB_CFSR_USGFAULTSR_Pos;

			if ((CFSRValue & (1 << 9)))
				printf("Divide by zero\n");

			if ((CFSRValue & (1 << 8)))
				printf("Unaligned access error\n");

			if ((CFSRValue & (1 << 3)))
				printf("Coprocessor access error\n");

			if ((CFSRValue & (1 << 2)))
				printf("Integrity check error on EXC_RETURN\n");
		}

		// Bus Fault
		if ((SCB->CFSR & SCB_CFSR_BUSFAULTSR_Msk)) {
			CFSRValue = SCB->CFSR;
			printf("Bus fault: \n");
			CFSRValue >>= SCB_CFSR_BUSFAULTSR_Pos;

			if ((CFSRValue & (1 << 7)) && (CFSRValue & (1 << 1))) {
				printf("Precise data access error. Bus Fault Address \
					Register is: %x \n", (unsigned int)SCB->BFAR);
			}

			if ((CFSRValue & (1 << 4)))
				printf("Bus fault has occurred on exception entry\n");

			if ((CFSRValue & (1 << 3)))
				printf("bus fault has occurred on exception return\n");

			if ((CFSRValue & (1 << 2)))
				printf("Imprecise data access error\n");

			if ((CFSRValue & (1 << 0))) {
				printf("This bit indicates a bus fault on an instruction \
					pre-fetch. \n");
			}
		}
	}

	// MemoryFault
	if ((SCB->CFSR & SCB_CFSR_MEMFAULTSR_Msk)) {
		CFSRValue = SCB->CFSR;
		printf("Memory fault: \n");
		CFSRValue >>= SCB_CFSR_MEMFAULTSR_Pos;

		if ((CFSRValue & (1 << 9)) != 0)
			printf("Divide by zero\n");
	}

	__ISB();
	__DMB();
	//__ASM volatile("BKPT #01");
	Dummy_Handler();
}
/**
 * \brief Default HardFault interrupt handler.
 */

void HardFault_Handler(void)
{
	printf("E - HardFault at address 0X%x\n", (int)StackUnwind());
	__ISB();
	__DMB();
	HardFault_reason();
}

#ifndef MPU_EXAMPLE_FEATURE
/**
 * \brief Default MemManage interrupt handler.
 */
void MemManage_Handler(void)
{
	printf("E - MemoryMemFault (MPU fault) at address 0X%x\n",
		   (int)StackUnwind());
	__ISB();
	__DMB();
	//__ASM volatile("BKPT #01");
	Dummy_Handler();
}
#endif

/**
 * \brief Default BusFault interrupt handler.
 */
void BusFault_Handler(void)
{
	__ASM("nop");
	__ASM("nop");
	printf("E - Bus Fault at address 0X%x\n", (int)StackUnwind());

	__ISB();
	__DMB();
	//__ASM volatile("BKPT #01");
	Dummy_Handler();
}

/**
 * \brief Default UsageFault interrupt handler.
 */
void UsageFault_Handler(void)
{
	printf("E - Usage fault at address 0X%x\n", (int)StackUnwind());

	__ISB();
	__DMB();
	//__ASM volatile("BKPT #01");
	Dummy_Handler();
}
