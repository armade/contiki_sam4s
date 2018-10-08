/*
 * SNTP implementation for Contiki
 *
 * Copyright (C) 2011 Anuj Sehgal <s.anuj@jacobs-university.de>
 *
 * This program is part of free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
 *
 */

#include "contiki.h"
#include "contiki-lib.h"
#include "contiki-net.h"
#include "weather.h"
#include "clock.h"

#include <string.h>

#define _DEBUG_                 0
#if _DEBUG_
#define PRINTF(...)       printf(__VA_ARGS__)
#else
#define PRINTF(...)
#endif

#define UIP_IP_BUF   ((struct uip_ip_hdr *)&uip_buf[UIP_LLH_LEN])

static unsigned short SEND_INTERVAL = 60 * 60; // every hour




#define SERVER_PORT 52202
#define REMOTE_PORT 80
static struct simple_udp_connection unicast_connection;
static uint8_t flash_is_sleeping = 1;
static uint16_t seqno_end;
static uint16_t streamno_current;
/*---------------------------------------------------------------------------*/
PROCESS(udpstream_process, "UDP weather Process");
AUTOSTART_PROCESSES(&weather_process);

/*---------------------------------------------------------------------------*/
static void
receiver(struct simple_udp_connection *c,
         const uip_ipaddr_t *sender_addr,
         uint16_t sender_port,
         const uip_ipaddr_t *receiver_addr,
         uint16_t receiver_port,
         const uint8_t *data,
         uint16_t datalen)
{
	uint8_t *diagram = (struct msg *)data;

	asm volatile("NOP");

}

/*---------------------------------------------------------------------------*/
PROCESS_THREAD(weather_process, ev, data)
{
	//static uip_ds6_addr_t *ip_addr;
	//static uip_ip4addr_t remote_ip_addr = {95,85,63,65};
	PROCESS_BEGIN();
/*
	// Start service registration
	servreg_hack_init();

    // The sink creates a dag and waits for UDP datagrams
    //servreg_hack_register(SERVICE_ID, &ip_addr->ipaddr);
    simple_udp_register(&unicast_connection, SERVER_PORT,
                        NULL, REMOTE_PORT, receiver);
    while(1) {
    	PROCESS_WAIT_EVENT();
    	simple_udp_sendto(&unicast_connection,
    			"GET http://api.openweathermap.org/data/2.5/weather?lat=56.1837005&lon=9.5350504&appid=c592e14137c3471fa9627b44f6649db4",
    			sizeof("GET http://api.openweathermap.org/data/2.5/weather?lat=56.1837005&lon=9.5350504&appid=c592e14137c3471fa9627b44f6649db4")-1,
    			remote_ip_addr);
    }

*/
  PROCESS_END();
}
/*---------------------------------------------------------------------------*/

    //  tcp_socket_send_str(&socket, "GET http://api.openweathermap.org/data/2.5/weather?lat=56.1837005&lon=9.5350504&appid=c592e14137c3471fa9627b44f6649db4");
