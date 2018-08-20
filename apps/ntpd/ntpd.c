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

#define UIP_IP_BUF   ((struct uip_ip_hdr *)&uip_buf[UIP_LLH_LEN])

static unsigned short SEND_INTERVAL = 60 * 60; // every hour
static unsigned long StartTime = 0;
static unsigned long CurrTime = 0;

static ntp_packet_t ntpmsg;
static struct etimer Send_NTP_request_timer;
static struct etimer Update_parrent_timer;

static struct simple_udp_connection unicast_connection;

//static struct uip_udp_conn *ntp_conn;
PROCESS(ntpd_process, "ntpd");

static struct ctimer delay_timer;
volatile uint8_t NTP_BUSY = 0;

static void
notify_ready(void *not_used)
{
	// Timeout. Stop waiting.
	NTP_BUSY = 0;
}

uint8_t NTP_status(void)
{
	return NTP_BUSY;
}
/*---------------------------------------------------------------------------*/
/*unsigned long getCurrTime(void)
{
	if((StartTime == 0) && (CurrTime == 0))
		return 0;
	return (clock_seconds() - StartTime + CurrTime);
}*/
/*---------------------------------------------------------------------------*/
// Extract the 32 bits that represent the time-stamp seconds (since NTP epoch) from when the packet received.
// Subtract 70 years worth of seconds from the seconds. (NTP base is 1900 and UNIX base is 1970).
// This leaves the seconds since the UNIX epoch of 1970.
// (1900)------------------(1970)**************************************(Time Packet Left the Server)
static void
receiver(struct simple_udp_connection *c,
         const uip_ipaddr_t *sender_addr,
         uint16_t sender_port,
         const uip_ipaddr_t *receiver_addr,
         uint16_t receiver_port,
         const uint8_t *data,
         uint16_t datalen)
{
	ntp_packet_t * NTP = (ntp_packet_t *) data;
	ntp_packet_t NTP_server;
	clock_time_t time;
	uint8_t current_stranum;

	if(datalen == 48){

		switch(NTP->mode)
		{
			case 4: // We got a reply for the request
				PRINTF("ntpd response received\n");
				if((clock_quality(READ_STRANUM) > NTP->stratum ) && (NTP->ver == 3))
				{
					CurrTime = uip_ntohl(NTP->tx_Time_s) - NTP_EPOCH; //UNIX time
					if(CurrTime < 1514764800) { //Monday, 01/01-2018 00:00:00 UTC
						return;
					}
					// Debug
					if(CurrTime > 1546300800) //Tuesday, 01/01-2019 00:00:00 UTC
					{
						asm volatile("NOP");
					}
					clock_set_unix_time(CurrTime,1);
					StartTime = clock_seconds();
					current_stranum = (NTP->stratum + 1)>15?15:(NTP->stratum + 1);
					clock_quality(current_stranum);
				}
				// We got response so NTP is happy
				NTP_BUSY = 0;
				break;

			case 3: // Someone is asking us for the time
				PRINTF("ntpd request received\n");
				time = clock_get_unix_time();
				if(time){
					memset(&NTP_server,0,sizeof(NTP_server));
					NTP_server.mode = 4;
					NTP_server.ver = 3;
					NTP_server.stratum = clock_quality(READ_STRANUM);
					NTP_server.tx_Time_s = uip_ntohl(time + NTP_EPOCH);

					simple_udp_sendto(&unicast_connection, &NTP_server, 48, sender_addr);
	
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
static void Send_NTP_request(uip_ipaddr_t *ipaddr)
{
	if(!NTP_BUSY){
		PRINTF("ntpd requesting time from parent\n");
		NTP_BUSY = 1;
		ntpmsg.mode = 3;
		ntpmsg.ver = 3;
		// Request time
		simple_udp_sendto(&unicast_connection, &ntpmsg, 48, ipaddr);
		// Wait 50 ms for response
		ctimer_set(&delay_timer, 50, notify_ready, NULL);
	}else{
		etimer_set(&Send_NTP_request_timer, 60 * CLOCK_SECOND);
	}
}
/*---------------------------------------------------------------------------*/
static void Send_NTP_time_to_parent(uip_ipaddr_t *ipaddr)
{
	ntp_packet_t NTP_server;
	clock_time_t time;
	PRINTF("ntpd sending time to parent\n");
	time = clock_get_unix_time();
	if(time && !NTP_BUSY){
		NTP_BUSY = 1;
		NTP_server.stratum = clock_quality(READ_STRANUM);
		NTP_server.tx_Time_s = uip_ntohl(time + NTP_EPOCH);
		NTP_server.ver = 3;
		NTP_server.mode = 4;
		simple_udp_sendto(&unicast_connection, &NTP_server, 48, ipaddr);
		// Wait 50 ms so we have time to send packet
		ctimer_set(&delay_timer, 50*CLOCK_SECOND/1000, notify_ready, NULL);
	}else{
		etimer_set(&Update_parrent_timer, 60 * CLOCK_SECOND);
	}
}
/*---------------------------------------------------------------------------*/
#ifdef NODE_ROUTER
static void
 set_connection_address(uip_ipaddr_t *ipaddr)
 {
	uip_ip6addr(ipaddr,0xaaaa,0,0,0,0,0,0,0x0001);
 }
#endif
/*---------------------------------------------------------------------------*/
PROCESS_THREAD(ntpd_process, ev, data)
{

	static uip_ipaddr_t ipaddr;
	int stranum_val;

	PROCESS_BEGIN();
	PRINTF("ntpd process started\n");

#ifdef NODE_ROUTER
	set_connection_address(&ipaddr);
#else
	/* find the IP of router */
	while(1){
		if(uip_ds6_defrt_choose()){
			uip_ipaddr_copy(&ipaddr, uip_ds6_defrt_choose());
			break;
		}
		etimer_set(&Send_NTP_request_timer, CLOCK_SECOND);
		PROCESS_YIELD_UNTIL(etimer_expired(&Send_NTP_request_timer));
	}
#endif
	simple_udp_register(&unicast_connection, NTPD_PORT,
                      NULL, NTPD_PORT, receiver);

	etimer_set(&Send_NTP_request_timer, 10 * CLOCK_SECOND);
	etimer_set(&Update_parrent_timer, 25 * CLOCK_SECOND);
	while(1){
		PROCESS_YIELD();
		stranum_val = clock_quality(READ_STRANUM);
		if(etimer_expired(&Send_NTP_request_timer)){
			Send_NTP_request(&ipaddr);
			if(stranum_val == 15)
				etimer_set(&Send_NTP_request_timer, 10*CLOCK_SECOND); // 10sec
			else
				etimer_set(&Send_NTP_request_timer, SEND_INTERVAL * CLOCK_SECOND/stranum_val); // 1hr - 4min
		}else if(etimer_expired(&Update_parrent_timer)){
#ifndef NODE_ROUTER
			Send_NTP_time_to_parent(&ipaddr);
			etimer_set(&Update_parrent_timer, SEND_INTERVAL * CLOCK_SECOND * (stranum_val/15));
#endif
		}
		
	}

	PROCESS_END();
}
/*---------------------------------------------------------------------------*/
