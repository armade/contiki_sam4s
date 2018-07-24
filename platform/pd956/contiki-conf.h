/*
 * Copyright (c) 2005, Swedish Institute of Computer Science
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

#ifndef CONTIKI_CONF_H_
#define CONTIKI_CONF_H_

#include <inttypes.h>

#define BOARD_STRING 					"PD956-01"
#define SENSOR_ERROR        			0x80000000

#define CC_CONF_REGISTER_ARGS          1
#define CC_CONF_FUNCTION_POINTER_ARGS  1
#define CC_CONF_VA_ARGS                1
/*#define CC_CONF_INLINE                 inline*/

#ifndef EEPROM_CONF_SIZE
#define EEPROM_CONF_SIZE				512
#endif

#define CLOCK_CONF_SECOND 				1000UL

#define IEEE802154_CONF_PANID           0xABCD
#define RF_CHANNEL              		13

#define RDC_CONF_MCU_SLEEP				1
//#define NETSTACK_RDC_CHANNEL_CHECK_RATE 16
//#define RDC_CONF_MCU_SLEEP				1
#define RPL_CONF_WITH_DAO_ACK          1

#define CCIF
#define CLIF

#define SLIP_BRIDGE_CONF_NO_PUTCHAR 1

typedef unsigned long clock_time_t;
/* These names are deprecated, use C99 names. */
typedef unsigned short 	uip_stats_t;
typedef unsigned char 	u8_t;
typedef unsigned short 	u16_t;
typedef unsigned long 	u32_t;
typedef long 			s32_t;

// Core rtimer.h defaults to 16 bit timer unless RTIMER_CLOCK_DIFF is defined
typedef u32_t rtimer_clock_t;
#define RTIMER_CLOCK_DIFF(a,b)     ((s32_t)((a)-(b)))


typedef unsigned short uip_stats_t;

#define UIP_CONF_UDP             1
#define UIP_CONF_MAX_CONNECTIONS 40
#define UIP_CONF_MAX_LISTENPORTS 40
#define UIP_CONF_BUFFER_SIZE     1280
#define UIP_CONF_BYTE_ORDER      UIP_LITTLE_ENDIAN
#define UIP_CONF_TCP       		 1
#define UIP_CONF_TCP_SPLIT       0
#define UIP_CONF_LOGGING         0
#define UIP_CONF_UDP_CHECKSUMS   1

/* Max Number of buffer to stored in a Queue */
#define QUEUEBUF_CONF_NUM                     12

#ifndef NETSTACK_CONF_RDC_CHANNEL_CHECK_RATE
#define NETSTACK_CONF_RDC_CHANNEL_CHECK_RATE 4
#endif /* NETSTACK_CONF_RDC_CHANNEL_CHECK_RATE */

/* Configure NullRDC for when it's selected */
#define NULLRDC_802154_AUTOACK                  0
#define NULLRDC_802154_AUTOACK_HW               1
#define NULLRDC_CONF_802154_AUTOACK_HW 			1
#define RDC_CONF_HARDWARE_CSMA     				1
#define HW_CSMA_FRAME_RETRIES 					1

#define NULLRDC_CONF_ACK_WAIT_TIME          (50 * RTIMER_SECOND/1000)

#define LINKADDR_CONF_SIZE              8

#ifndef NETSTACK_CONF_MAC
#define NETSTACK_CONF_MAC     nullmac_driver//csma_driver//nullmac_driver
#endif /* NETSTACK_CONF_MAC */

#ifndef NETSTACK_CONF_RDC
#define NETSTACK_CONF_RDC     nullrdc_driver//nullrdc_driver//contikimac_driver
#endif /* NETSTACK_CONF_RDC */

#define NULLRDC_CONF_802154_AUTOACK       1

#ifndef NETSTACK_CONF_RADIO
#define NETSTACK_CONF_RADIO   RF231_radio_driver
#endif /* NETSTACK_CONF_RADIO */

#ifndef NETSTACK_CONF_FRAMER
#define NETSTACK_CONF_FRAMER  framer_802154
#endif /* NETSTACK_CONF_FRAMER */

#ifndef NETSTACK_CONF_NETWORK
#define NETSTACK_CONF_NETWORK       sicslowpan_driver
#endif

//#define NETSTACK_CONF_LLSEC XTEAsec_driver


#define UIP_CONF_ROUTER                 1

#define SICSLOWPAN_CONF_COMPRESSION             SICSLOWPAN_COMPRESSION_HC06
#define SICSLOWPAN_CONF_MAXAGE                  8
#define SICSLOWPAN_CONF_MAX_ADDR_CONTEXTS       2


/* 6LoWPAn Fragmentation and Reassembly config options */
#define SICSLOWPAN_CONF_FRAG                  1


#define UIP_CONF_IPV6_CHECKS     				1
#define UIP_CONF_IPV6_QUEUE_PKT  				1
#define UIP_CONF_IPV6_REASSEMBLY 				0
#define UIP_CONF_NETIF_MAX_ADDRESSES  			3
#define UIP_CONF_ICMP6           				1

/* configure number of neighbors and routes */

#define NBR_TABLE_CONF_MAX_NEIGHBORS     30


#define UIP_CONF_MAX_ROUTES   30


#define UIP_CONF_ND6_SEND_RA			0
#define UIP_CONF_ND6_REACHABLE_TIME     600000
#define UIP_CONF_ND6_RETRANS_TIMER      10000

#define UIP_CONF_IP_FORWARD             0


#define UIP_CONF_LLH_LEN                0
#define UIP_CONF_LL_802154              1

#define UIP_CONF_ICMP_DEST_UNREACH 1

#define UIP_CONF_DHCP_LIGHT
#define UIP_CONF_RECEIVE_WINDOW  (64*4)
#define UIP_CONF_TCP_MSS         (64*4)
#define UIP_CONF_UDP_CONNS       12
#define UIP_CONF_FWCACHE_SIZE    30
#define UIP_CONF_BROADCAST       1
#define UIP_ARCH_IPCHKSUM        1
#define UIP_CONF_UDP             1
#define UIP_CONF_UDP_CHECKSUMS   1
#define UIP_CONF_PINGADDRCONF    0
#define UIP_CONF_LOGGING         0


#define UIP_CONF_IPV6_RPL		1

#include <ctype.h>
#define ctk_arch_isprint isprint

#define LOG_CONF_ENABLED 1

#define PROGRAM_HANDLER_CONF_MAX_NUMDSCS 10
#define PROGRAM_HANDLER_CONF_QUIT_MENU   1

#define EMAIL_CONF_WIDTH  78
#define EMAIL_CONF_HEIGHT 17
#ifndef PLATFORM_BUILD
#define EMAIL_CONF_ERASE   0
#endif

#define IRC_CONF_WIDTH         78
#define IRC_CONF_HEIGHT        17
#define IRC_CONF_SYSTEM_STRING "*nix"

#define SHELL_CONF_WITH_PROGRAM_HANDLER 1

#define SHELL_GUI_CONF_XSIZE 78
#define SHELL_GUI_CONF_YSIZE 17

#ifdef PLATFORM_BUILD
#define TELNETD_CONF_GUI 1
#endif /* PLATFORM_BUILD */

#ifdef PLATFORM_BUILD
#define WWW_CONF_WEBPAGE_WIDTH  78
#define WWW_CONF_WEBPAGE_HEIGHT 17
#endif /* PLATFORM_BUILD */

/* Not part of C99 but actually present */
int strcasecmp(const char*, const char*);

/* include the project config */
/* PROJECT_CONF_H might be defined in the project Makefile */
#ifdef PROJECT_CONF_H
#include PROJECT_CONF_H
#endif /* PROJECT_CONF_H */

#define BAUD2UBR(x) (x)

#endif /* CONTIKI_CONF_H_ */
