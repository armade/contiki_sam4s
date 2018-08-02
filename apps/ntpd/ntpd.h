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

#ifndef __NTPD_H__
#define __NTPD_H__

#include "contiki.h"
#include "contiki-lib.h"
#include "contiki-net.h"


#define NTP_EPOCH            (86400U * (365U * 70U + 17U))
#define NTPD_PORT             123

//#define UDP_IP_BUF   ((struct uip_udpip_hdr *)&uip_buf[UIP_LLH_LEN])

/*	0                   1                   2                   3
    0 1 2 3 4 5 6 7 8 9 0 1 2 3 4 5 6 7 8 9 0 1 2 3 4 5 6 7 8 9 0 1
   +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
   |LI | Ver | Mode|    Stranum    |    Poll       |  Precision    |
   +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
   |                       Root delay                              |
   +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
   |                     Root dispersion                           |
   +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
   |                  Reference Clock Identifier                   |
   +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
   |                                                               |
   |                 Reference Timestamp (64 bits)                 |
   |                                                               |
   +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
   |                                                               |
   |                 Originate Timestamp (64 bits)                 |
   |                                                               |
   +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
   |                                                               |
   |                  Receive Timestamp (64 bits)                  |
   |                                                               |
   +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
   |                                                               |
   |                  Transmit Timestamp (64 bits)                 |
   |                                                               |
   +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
 Leap Indicator (LI)

      Code warning of impending leap-second to be inserted at the end of
      the last day of the current month. Bits are coded as follows:

         	00      no warning
         	01      +1 second (following minute has 61 seconds)
         	10      -1 second (following minute has 59 seconds)
         	11      reserved for future use

 Mode
			000		reserved.
			001		Symmetric active.
			010		Symmetric passive.
			011		Client.
			100		Server.
			101		Broadcast.
			110		NTP control message.
			111		private use.

 Reference timestamp. 64 bits.
		The local time at which the local clock was last set or corrected.

 Originate timestamp. 64 bits.
		The local time when the client sent the request.

 Receive timestamp. 64 bits.
		The local time when the request was received by the server.

 Transmit timestamp. 64 bits.
		The local time when the reply was sent from the server.
*/
#pragma pack(1)
typedef struct
{
	// Eight bits. li, vn, and mode.
	uint8_t mode:3;       		// mode. Three bits. Client will pick mode 3 for client.
	uint8_t ver:3;				// vn.   Three bits. Version number of the protocol.
	uint8_t li:2;				// li.   Two bits.   Leap indicator.

	uint8_t stratum;			// Eight bits. Stratum level of the local clock.
	uint8_t poll;				// Eight bits. Maximum interval between successive messages.
	uint8_t precision;			// Eight bits. Precision of the local clock.

	uint32_t rootDelay;			// 32 bits. Total round trip delay time.
	uint32_t rootDispersion; 	// 32 bits. Max error aloud from primary clock source.
	uint32_t refId;				// 32 bits. Reference clock identifier.

	uint32_t ref_Time_s;		// 32 bits. Reference time-stamp seconds.
	uint32_t ref_Time_f;		// 32 bits. Reference time-stamp fraction of a second.

	uint32_t orig_Time_s;		// 32 bits. Originate time-stamp seconds.
	uint32_t orig_Time_f;		// 32 bits. Originate time-stamp fraction of a second.

	uint32_t rx_Time_s;			// 32 bits. Received time-stamp seconds.
	uint32_t rx_Time_f;			// 32 bits. Received time-stamp fraction of a second.

	uint32_t tx_Time_s;			// 32 bits and the most important field the client cares about. Transmit time-stamp seconds.
	uint32_t tx_Time_f;			// 32 bits. Transmit time-stamp fraction of a second.

} ntp_packet_t;					// Total: 384 bits or 48 bytes.
#pragma pack()
PROCESS_NAME(ntpd_process);

//unsigned long getCurrTime(void);
uint8_t NTP_status(void);

//static void tcpip_handler(void);
//static void timeout_handler(void);
#endif /* __NTPD_H__ */

