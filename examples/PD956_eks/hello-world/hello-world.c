/*
 * Copyright (c) 2006, Swedish Institute of Computer Science.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. Neither the name of the Institute nor the names of its contributors
 *    may be used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE INSTITUTE AND CONTRIBUTORS ``AS IS'' AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE INSTITUTE OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 *
 * This file is part of the Contiki operating system.
 *
 */

/**
 * \file
 *         A very simple Contiki application showing how Contiki programs look
 * \author
 *         Adam Dunkels <adam@sics.se>
 */

#include "contiki.h"
#include "contiki-net.h"
#include "ip64.h"
#include "net/netstack.h"
#include "net/rpl/rpl-dag-root.h"
#include "ntpd.h"
#include "httpd-simple.h"
#include "httpd_post_handlers.h"
#include "sensors.h"
#include "board-peripherals.h"
#include "leds.h"
#include "slip.h"
#include "uip.h"


#ifdef NODE_GPS
#include "gpsd.h"
#endif

#include <stdio.h> /* For printf() */
/*---------------------------------------------------------------------------*/
PROCESS(hello_world_process, "Hello world process");
AUTOSTART_PROCESSES(&hello_world_process);

PROCESS(button_process, "Hello world process");
/*---------------------------------------------------------------------------*/

static void trigger_sensors(void)
{
	const struct sensors_sensor *sensors_ptr;
	for (sensors_ptr = sensors_first(); sensors_ptr != NULL; sensors_ptr = sensors_next(sensors_ptr))
	{
		sensors_ptr->configure(SENSORS_ACTIVE, 1);
	}
}


PROCESS_THREAD(hello_world_process, ev, data) {
	PROCESS_BEGIN()
	;
	uip_ip4addr_t ipv4addr, netmask;
	printf("Hello, world\n");

	/* Set us up as a RPL root node. */
	//rpl_dag_root_init_dag();
	ip64_init();
	/* Initialize the IP64 module so we'll start translating packets */
	uip_ipaddr(&ipv4addr, 10, 42, 0, 7);
	uip_ipaddr(&netmask, 255, 255, 255, 0);
	ip64_set_ipv4_address(&ipv4addr, &netmask);


	trigger_sensors();
	process_start(&button_process, NULL);

#ifdef NODE_GPS
	process_start(&gpsd_process, NULL);
#endif
	process_start(&httpd_simple_process, NULL);
	register_http_post_handlers();


	while (1) {
		PROCESS_WAIT_EVENT()
		;
	}
PROCESS_END();
}
/*---------------------------------------------------------------------------*/
struct etimer button_reload;

PROCESS_THREAD(button_process, ev, data) {
	PROCESS_BEGIN();
	volatile static int timedif;

	while (1) {
		PROCESS_WAIT_EVENT();

		if(ev == PROCESS_EVENT_TIMER && etimer_expired(&button_reload)) {
			timedif = ((clock_time()*1000)/CLOCK_SECOND) - button_sensor.status(STATUS_ACTIVATION_TIME);

			if((timedif >= (CLOCK_SECOND * 5)) && (timedif < (CLOCK_SECOND * 10))) {
			  leds_set(LEDS_BLUE);
			} else if((timedif >= (CLOCK_SECOND * 10)) && (timedif < (CLOCK_SECOND * 15)))  {
			  leds_set(LEDS_BLUE | LEDS_RED);
			}
			if(button_sensor.value(0) == BUTTON_ACTIVE)
				etimer_set(&button_reload, CLOCK_SECOND*1);
		}

		if(ev == sensors_event && data == &button_sensor) {
			if(button_sensor.value(0) == BUTTON_ACTIVE)
				etimer_set(&button_reload, CLOCK_SECOND*1);

			if(button_sensor.value(0) == BUTTON_DEACTIVE){
				leds_set(LEDS_GREEN);
				etimer_stop(&button_reload);
			}
		}
	}
PROCESS_END();
}

