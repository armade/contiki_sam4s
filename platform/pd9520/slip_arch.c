/*
 * slip_arch.c
 *
 *  Created on: 5. jul. 2017
 *      Author: pm
 */
#include "slip.h"
#include "compiler.h"
#include "uhi_cdc.h"

volatile char enumeration_complete;
PROCESS(slip_arch_process, "SLIP USB ACM");


void slip_arch_init(unsigned long ubr)
{
	process_start(&slip_arch_process, NULL);
}


void slip_arch_writeb(unsigned char c)
{
	// Potential deadlock if tx buffers become full
	if(enumeration_complete)
		uhi_cdc_putc(0, c);
}


PROCESS_THREAD(slip_arch_process, ev, data)
{
	iram_size_t number_og_bytes;
	PROCESS_BEGIN();

 	while(1) {

 		PROCESS_YIELD_UNTIL(ev == PROCESS_EVENT_POLL);
 		number_og_bytes = uhi_cdc_get_nb_received(0);

 		while(number_og_bytes--){
 			slip_input_byte(uhi_cdc_getc(0));
 		}

 	}
	PROCESS_END();
}

// called from USB IRQ when receiving data
void rx_poll(void)
{
	process_poll(&slip_arch_process);
}
