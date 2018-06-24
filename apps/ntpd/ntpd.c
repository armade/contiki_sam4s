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
#include "ntpd.h"
#include "clock.h"

#include <string.h>

#define _DEBUG_                 0
#if _DEBUG_
#define PRINTF(...)       printf(__VA_ARGS__)
#else
#define PRINTF(...)
#endif

static unsigned short SEND_INTERVAL = 60 * 60; // every hour
static unsigned long StartTime = 0;
static unsigned long CurrTime = 0;

static const ntp_packet_t ntpmsg = { .li = 0, .ver = 3, .mode = 3, 0 };

static struct uip_udp_conn *ntp_conn;
PROCESS(ntpd_process, "ntpd");

static struct ctimer delay_timer;
volatile uint8_t NTP_BUSY = 0;

static void
notify_ready(void *not_used)
{
	// Timeout. Stop waiting.
	NTP_BUSY = 0;
}
/*---------------------------------------------------------------------------*/
unsigned long getCurrTime(void)
{
	if((StartTime == 0) && (CurrTime == 0))
		return 0;
	return (clock_seconds() - StartTime + CurrTime);
}
/*---------------------------------------------------------------------------*/
// Extract the 32 bits that represent the time-stamp seconds (since NTP epoch) from when the packet received.
// Subtract 70 years worth of seconds from the seconds. (NTP base is 1900 and UNIX base is 1970).
// This leaves the seconds since the UNIX epoch of 1970.
// (1900)------------------(1970)**************************************(Time Packet Left the Server)
static void tcpip_handler(void)
{
	ntp_packet_t * NTP = (ntp_packet_t *) uip_appdata;
	ntp_packet_t NTP_server = { .li = 0, .ver = 3, .mode = 4, 0 };
	clock_time_t time;

	// Check port!!!!!!
	if(uip_newdata() && (uip_datalen() == 48)){

		switch(NTP->mode)
		{
			case 4:
				CurrTime = uip_ntohl(NTP->tx_Time_s) - NTP_EPOCH; //UNIX time
				clock_set_unix_time(CurrTime);
				StartTime = clock_seconds();
				// We got response so NTP is happy
				NTP_BUSY = 0;
				break;

			case 3:
				time = clock_get_unix_time();
				if(time){
					NTP_server.tx_Time_s = uip_ntohl(time + NTP_EPOCH);
					uip_udp_packet_send(ntp_conn, &NTP_server, 48);
					// Wait 50 ms so we have time to send packet
					ctimer_set(&delay_timer, 50, notify_ready, NULL);
					NTP_BUSY = 1;
				}
				break;

			default:
				break;
		}
	}
}
/*---------------------------------------------------------------------------*/
static void timeout_handler(void)
{
	uip_udp_packet_send(ntp_conn, &ntpmsg, 48);
	// Wait 50 ms for response
	ctimer_set(&delay_timer, 50, notify_ready, NULL);
	NTP_BUSY = 1;
}
/*---------------------------------------------------------------------------*/
/*static void
 set_connection_address(uip_ipaddr_t *ipaddr)
 {
	uip_ip6addr(ipaddr,0xaaaa,0,0,0,0,0,0,0x0001);
 }*/
/*---------------------------------------------------------------------------*/
PROCESS_THREAD(ntpd_process, ev, data)
{
	static struct etimer et;
	static uip_ipaddr_t ipaddr;

	PROCESS_BEGIN();
	PRINTF("ntpd process started\n");

	//set_connection_address(&ipaddr);

	/* find the IP of router */
	//etimer_set(&et, CLOCK_SECOND);
	while(1){
		if(uip_ds6_defrt_choose()){
			uip_ipaddr_copy(&ipaddr, uip_ds6_defrt_choose());
			break;
		}
		etimer_set(&et, CLOCK_SECOND);
		PROCESS_YIELD_UNTIL(etimer_expired(&et));
	}

	/* new connection with remote host */
	ntp_conn = udp_new(&ipaddr, UIP_HTONS(NTPD_PORT), NULL);

	etimer_set(&et, SEND_INTERVAL * CLOCK_SECOND);
	while(1){
		PROCESS_YIELD();
		if(etimer_expired(&et)){
			timeout_handler();
			etimer_set(&et, SEND_INTERVAL * CLOCK_SECOND);
		} else if(ev == tcpip_event){
			tcpip_handler();
		}
	}

	PROCESS_END();
}
/*---------------------------------------------------------------------------*/
