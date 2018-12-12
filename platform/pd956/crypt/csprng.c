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
#include "lib/random.h"

#include "csprng.h"
#include "sha256.h"
#include "uECC.h"


volatile unsigned csprng_state[8]; //256 bit entropy pool
volatile unsigned csprng_feedix, /*csprng_ready,*/ csprng_count;

#define RAMS(x) (x)
const unsigned short ramtable[16] = { RAMS(48), RAMS(192), RAMS(384), RAMS(6),
		RAMS(24), RAMS(4), RAMS(80), RAMS(160), RAMS(8), RAMS(16), RAMS(32),
		RAMS(64), RAMS(128), RAMS(256), RAMS(96), RAMS(512) };



static struct ctimer rng_timer;

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

// This function is called with normal clock,
// and adding the rtimer as feed.
// The rtimer runs on 32kHz rc clock.
static void
RNG_handler(void *not_used)
{
	csprng_feed(rtimer_arch_now()+rand());
	ctimer_reset(&rng_timer);
	//csprng_ready = 1;
}

int RNG_Function(uint8_t *dest, unsigned size)
{
	return csprng_get(dest, size);
}

void csprng_start(void)
{
	volatile unsigned *src, *top;
	unsigned h, i;

	h = (CHIPID->CHIPID_CIDR >> 16) & 15; //ram size
	h = (ramtable[h] << 10) + 0x20000000; //end of ram
	top = (volatile unsigned*) h;
	src = top - 8192; //last 32k

	h = 0;
	i = 0;
	do{
		h ^= *src++;
		h *= 123456789;
		h ^= h >> 16;
		if(++i == 191){
			csprng_feed(h);
			i = 0;
			h = 0;
		}
	} while(src != top);
	top[-1]++; //increment last word in ram - paranoia (reset tricks)

	csprng_feed(h);

	for(i=0;i<4;i++){
		get_eeprom(Flash_unique_id[i], h);
		csprng_feed(h);
		csprng_feed(rand()); // initialized in radio with random seed
	}
	csprng_feed(clock_get_unix_time()); // if clock is set this will be good.
	csprng_feedix = 0;
	//csprng_ready = 0;

	ctimer_set(&rng_timer, 20, RNG_handler, NULL);

	uECC_set_rng(RNG_Function);
	random_set_func(RNG_Function);
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

