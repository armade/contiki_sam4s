/*
 * csprng.c
 *
 *  Created on: 9. maj 2017
 *      Author: pm
 */

#include <string.h>
#include "contiki-conf.h"
#include "platform-conf.h"
#include "compiler.h"
#include "sys/ctimer.h"
#include "rtimer-arch.h"
#include "platform-conf.h"
#include "stdint.h"
#include "clock.h"
#include "drivers/trng.h"
#include "drivers/pmc.h"

#include "csprng.h"
#include "sha256.h"
#include "uECC.h"


volatile unsigned csprng_state[8]; //256 bit entropy pool
volatile unsigned csprng_feedix, csprng_ready, csprng_count;

#define RAMS(x) (x)
const unsigned short ramtable[16] = { RAMS(48), RAMS(192), RAMS(384), RAMS(6),
		RAMS(24), RAMS(4), RAMS(80), RAMS(160), RAMS(8), RAMS(16), RAMS(32),
		RAMS(64), RAMS(128), RAMS(256), RAMS(96), RAMS(512) };



static inline void csprng_feed(unsigned d)
{
	int fix = csprng_feedix;

	d ^= csprng_state[fix & 7];
	d *= 1234567891;
	d ^= d >> 16;
	++fix;
	csprng_state[fix & 7] += d;
	csprng_feedix = fix;
}

void TRNG_Handler(void)
{
	uint32_t status;

	status = trng_get_interrupt_status(TRNG);

	if ((status & TRNG_ISR_DATRDY) == TRNG_ISR_DATRDY) {
		csprng_feed(trng_read_output_data(TRNG));
	}
}

int RNG_Function(uint8_t *dest, unsigned size)
{
	return csprng_get(dest, size);
}

void csprng_start(void)
{
	/* Configure PMC */
	pmc_enable_periph_clk(ID_TRNG);

	/* Enable TRNG */
	trng_enable(TRNG);

	/* Enable TRNG interrupt */
	NVIC_DisableIRQ(TRNG_IRQn);
	NVIC_ClearPendingIRQ(TRNG_IRQn);
	NVIC_SetPriority(TRNG_IRQn, 0);
	//NVIC_EnableIRQ(TRNG_IRQn);
	//trng_enable_interrupt(TRNG);

	uECC_set_rng(RNG_Function);
}

int csprng_get(unsigned char *dst, int bytes)
{
	union
	{
		unsigned char b[32];
		unsigned u[8];
	} a;
	SHA256_CTX ctx;
	int i;

	//while(!csprng_ready);

	while(bytes){
		a.u[0] = csprng_feedix;								//feed counter
		a.u[1] = (unsigned) &a;								//current stack frame
		a.u[2] = SysTick->VAL;								//tick
		a.u[3] = __sync_add_and_fetch(&csprng_count, 1);	//generator counter
		a.u[4] = rtimer_arch_now();							//slow clock
		sha2_sha256_init(&ctx);
		sha2_sha256_update(&ctx, a.b, 16);
		sha2_sha256_update(&ctx, (unsigned char*) csprng_state,
				sizeof(csprng_state));
		sha2_sha256_final(&ctx, a.b);
		for (i = 0; i < 32 ; ++i){
			*dst++ = a.b[i];
			if(!--bytes)
				break;
		}
	}
	return 1;
}

