/*
 * Copyright (c) 2011, Swedish Institute of Computer Science.
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
 */

#include "contiki.h"
#include "net/netstack.h"
#include "net/ip/uip.h"
#include "net/packetbuf.h"
#include "dev/slip.h"
#include "uip.h"
#include "net/netstack.h"
#include "uip-ds6.h"
#include <stdio.h>
#include "ip64.h"
#include "ip64-eth.h"


#define SLIP_END     0300
#define SLIP_ESC     0333
#define SLIP_ESC_END 0334
#define SLIP_ESC_ESC 0335

#define DEBUG 1

#define BUF ((struct uip_eth_hdr *)&uip_buf[0])
#define UIP_IP_BUF        ((struct uip_ip_hdr *)&uip_buf[UIP_LLH_LEN])

void ETH_out(void)
{
	int len, ret;
	struct ip64_eth_hdr *ethhdr;
	ethhdr = (struct ip64_eth_hdr *) &ip64_packet_buffer[UIP_LLH_LEN];
	len = uip_len;

	memcpy(&ip64_packet_buffer[sizeof(struct ip64_eth_hdr)],
			&uip_buf[UIP_LLH_LEN], uip_len);

	ret = ip64_arp_create_ethhdr(ip64_packet_buffer,
			&ip64_packet_buffer[sizeof(struct ip64_eth_hdr)]);
	if(ret > 0)
		len += ret;
	ethhdr->type = UIP_HTONS(IP64_ETH_TYPE_IPV6);
	return IP64_ETH_DRIVER.output(ip64_packet_buffer, len);
}

static void slip_input_callback(void)
{
	uint8_t i;
	printf("Slipnet: input\n");

	if(uip_buf[0] == '?'){
		if(uip_buf[1] == 'P'){
			uip_ipaddr_t prefix;
			/* Here we set a prefix !!! */
			uip_buf[0] = '!';
			uip_ip6addr(&prefix, UIP_DS6_DEFAULT_PREFIX, 0, 0, 0, 0, 0, 0, 0);
			memcpy(&uip_buf[2], &prefix.u8, sizeof(uip_ipaddr_t));
			uip_len = 10;
			slip_send();
			printf("Slipnet: sending prefix\n");
			memset(uip_buf, 0, 100);
			uip_len = 0;

			//uip_ip6addr(&prefix, UIP_DS6_DEFAULT_PREFIX, 0, 0, 0, 0x3030, 0x3531, 0x3130, 0x3036);
			//uip_ds6_defrt_add(&prefix,0);
		}
	}else if(uip_buf[0] == 0x60){
		ETH_out();
		uip_len = 0;
	}
}


uint8_t my_slip_output(const uip_lladdr_t *addr)
{
	slipnet_driver.input();
	return 1;
}

/*---------------------------------------------------------------------------*/
void
slipnet_init(void)
{
	printf("Slipnet: init\n");

	slip_arch_init(0);
	  process_start(&slip_process, NULL);
	  slip_set_input_callback(slip_input_callback);

	  tcpip_set_outputfunc(my_slip_output);
}

/*---------------------------------------------------------------------------*/
void
slipnet_input(void)
{
	  printf("Slipnet: output\n");
	  slip_send();
}
/*---------------------------------------------------------------------------*/
const struct network_driver slipnet_driver = {
  "slipnet",
  slipnet_init,
  slipnet_input
};
/*---------------------------------------------------------------------------*/

