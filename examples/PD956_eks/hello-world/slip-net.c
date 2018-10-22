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

#define SLIP_END     0300
#define SLIP_ESC     0333
#define SLIP_ESC_END 0334
#define SLIP_ESC_ESC 0335

#define DEBUG 1

#define BUF ((struct uip_eth_hdr *)&uip_buf[0])
#define UIP_IP_BUF        ((struct uip_ip_hdr *)&uip_buf[UIP_LLH_LEN])





static void
slip_input_callback(void)
{

	uint8_t i;
	printf("Slipnet: input\n");

	//uip_neighbor_add(&BUF->srcipaddr, (struct uip_neighbor_addr *)&BUF->src);
  /*PRINTF("SIN: %u\n", uip_len);*/
  if(uip_buf[0] == '?') {
    if(uip_buf[1] == 'P') {
      uip_ipaddr_t prefix;
      /* Here we set a prefix !!! */
      uip_buf[0] = '!';
      uip_ip6addr(&prefix, UIP_DS6_DEFAULT_PREFIX, 0, 0, 0, 0, 0, 0, 0);
      memcpy(&uip_buf[2],&prefix.u8,sizeof(uip_ipaddr_t));
      uip_len = 10;
	  slip_send();
	  printf("Slipnet: sending prefix\n");
	  memset(uip_buf,0,100);

	  //uip_ip6addr(&prefix, UIP_DS6_DEFAULT_PREFIX, 0, 0, 0, 0x3030, 0x3531, 0x3130, 0x3036);
	  ////uip_ds6_route_add(UIP_DS6_DEFAULT_PREFIX, 64, &prefix);
	  //linkaddr_t lladdr = {0x30,0x30, 0x35,0x31, 0x31,0x30, 0x30,0x36};
	 // nbr_table_add_lladdr(ds6_neighbors, &lladdr
	 //                                             , 0, NULL);
	  //uip_ds6_defrt_add(&prefix, 1);
    }

  }else{ /* Save the last sender received over SLIP to avoid bouncing the
       packet back if no route is found */

    uint16_t len = ip64_6to4(&uip_buf[UIP_LLH_LEN], uip_len,
			     ip64_packet_buffer);
    if(len > 0) {
      memcpy(&uip_buf[UIP_LLH_LEN], ip64_packet_buffer, len);
      uip_len = len;
      /*      PRINTF("send len %d\n", len); */
    } else {

    }
}
}

uint8_t my_slip_output(const uip_lladdr_t *addr)

{
	printf("Slipnet: output\n");
	slipnet_driver.input();
	//slip_send();
	return 1;
}

/*---------------------------------------------------------------------------*/
void
slipnet_init(void)
{
	printf("Slipnet init\n");

	slip_arch_init(0);
	  process_start(&slip_process, NULL);
	  slip_set_input_callback(slip_input_callback);

	  tcpip_set_outputfunc(my_slip_output);
}

/*---------------------------------------------------------------------------*/
void
slipnet_input(void)
{
	  int len;

	  printf("ip64-slip-interface: output source ");

	  /*
	  PRINT6ADDR(&UIP_IP_BUF->srcipaddr);
	  PRINTF(" destination ");
	  PRINT6ADDR(&UIP_IP_BUF->destipaddr);
	  PRINTF("\n");
	  */
	    len = ip64_6to4(&uip_buf[UIP_LLH_LEN], uip_len,
			    ip64_packet_buffer);
	    printf("ip64-interface: output len %d\n", len);
	    if(len > 0) {
	      memcpy(&uip_buf[UIP_LLH_LEN], ip64_packet_buffer, len);
	      uip_len = len;
	      slip_send();
	      return len;
	    }else
	    	slip_send();
	  return 0;
	//slip_send();
}
/*---------------------------------------------------------------------------*/
const struct network_driver slipnet_driver = {
  "slipnet",
  slipnet_init,
  slipnet_input
};
/*---------------------------------------------------------------------------*/

const struct uip_fallback_interface slipnet_fallback_interface = {
		slipnet_init, slipnet_input
};
