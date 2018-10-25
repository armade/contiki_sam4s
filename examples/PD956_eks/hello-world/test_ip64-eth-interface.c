/*
 * Copyright (c) 2012, Thingsquare, http://www.thingsquare.com/.
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
 *
 * 3. Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * ``AS IS'' AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL THE
 * COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 * STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED
 * OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */
#include "net/ip/uip.h"
#include "net/ipv6/uip-ds6.h"
#include "dev/slip.h"

#include "ip64.h"
#include "ip64-arp.h"
#include "ip64-eth-interface.h"

#include <string.h>

#define UIP_IP_BUF        ((struct uip_ip_hdr *)&uip_buf[UIP_LLH_LEN])

#define DEBUG DEBUG_NONE
#include "net/ip/uip-debug.h"

/*---------------------------------------------------------------------------*/
void
ip64_eth_interface_input(uint8_t *packet, uint16_t len)
{
  struct ip64_eth_hdr *ethhdr;
  ethhdr = (struct ip64_eth_hdr *)packet;
  uip_ipaddr_t prefix;
  uip_ip6addr(&prefix, UIP_DS6_DEFAULT_PREFIX, 0, 0, 0, 0, 0, 0, 0);

  //PRINTF("%s: eth_type = %x\n",__func__,ethhdr->type);

  if(ethhdr->type == UIP_HTONS(IP64_ETH_TYPE_ARP)) {
    len = ip64_arp_arp_input(packet, len);

    if(len > 0) {
      IP64_ETH_DRIVER.output(packet, len);
    }
  } else if(ethhdr->type == UIP_HTONS(IP64_ETH_TYPE_IP) &&
	    len > sizeof(struct ip64_eth_hdr)) {
	  PRINTF("-------------->\n");
    uip_len = ip64_4to6(&packet[sizeof(struct ip64_eth_hdr)],
			len - sizeof(struct ip64_eth_hdr),
			&uip_buf[UIP_LLH_LEN]);
    if(uip_len > 0) {
    	PRINTF("ip64_interface_process: converted %d bytes\n", uip_len);

    	PRINTF("ip64-interface: (IPv4)input source ");
      PRINT6ADDR(&UIP_IP_BUF->srcipaddr);
      PRINTF(" destination ");
      PRINT6ADDR(&UIP_IP_BUF->destipaddr);
      PRINTF("\n");

      tcpip_input();
      PRINTF("Done\n");
    }
  } else if(ethhdr->type == UIP_HTONS(IP64_ETH_TYPE_IPV6)){ // TODO: If the packet is for us...
	  memcpy(&uip_buf[UIP_LLH_LEN],&packet[sizeof(struct ip64_eth_hdr)],len - sizeof(struct ip64_eth_hdr));
	  PRINTF("ip64-interface: (IPv6)input source ");
		PRINT6ADDR(&UIP_IP_BUF->srcipaddr);
		PRINTF(" destination ");
		PRINT6ADDR(&UIP_IP_BUF->destipaddr);
		PRINTF("\n");
		uip_len = len - sizeof(struct ip64_eth_hdr);


		  int i;
		  uint8_t state;


		  // internal
		  for(i = 0; i < UIP_DS6_ADDR_NB; i++) {
		    state = uip_ds6_if.addr_list[i].state;
		    if(uip_ds6_if.addr_list[i].isused &&
		       (state == ADDR_TENTATIVE || state == ADDR_PREFERRED)) {
		    		if(uip_ipaddr_cmp(&uip_ds6_if.addr_list[i].ipaddr,&UIP_IP_BUF->destipaddr)){
		    			printf("handled local\n");
		    			tcpip_input();
		    			uip_len = 0;
		    			return;
		    		}
		    }
		  }
		  slip_send();
		  uip_ip6addr(&prefix, 0xff02, 0, 0, 0, 0, 0, 0, 0);
		  if(uip_ipaddr_prefixcmp(&UIP_IP_BUF->destipaddr, &prefix, 16)) //mcast
			{
					tcpip_input();
			}





	  //if(uip_ipaddr_prefixcmp(&UIP_IP_BUF->destipaddr, &prefix, 16)) // Only if prefix is aaaa
		  //slip_send();
	  //else
		//  tcpip_input();
	  uip_len = 0;
	}
}
/*---------------------------------------------------------------------------*/
uint8_t
ip64_eth_interface_output(const uip_lladdr_t *addr)
{
  int len, ret;

  uip_ipaddr_t prefix;
   uip_ip6addr(&prefix, UIP_DS6_DEFAULT_PREFIX, 0, 0, 0, 0, 0, 0, 0);

  PRINTF("ip64-interface: output source ");
  PRINT6ADDR(&UIP_IP_BUF->srcipaddr);
  PRINTF(" destination ");
  PRINT6ADDR(&UIP_IP_BUF->destipaddr);
  PRINTF("\n");

  PRINTF("<--------------\n");
  len = ip64_6to4(&uip_buf[UIP_LLH_LEN], uip_len,
		  &ip64_packet_buffer[sizeof(struct ip64_eth_hdr)]);

  PRINTF("ip64-interface: output len %d\n", len);
  if(len > 0) {
    if(ip64_arp_check_cache(&ip64_packet_buffer[sizeof(struct ip64_eth_hdr)])) {
      PRINTF("Create header\n");
      ret = ip64_arp_create_ethhdr(ip64_packet_buffer,
				   &ip64_packet_buffer[sizeof(struct ip64_eth_hdr)]);
      if(ret > 0) {
	len += ret;
	IP64_ETH_DRIVER.output(ip64_packet_buffer, len);
      }
    } else {
      PRINTF("Create request\n");
      len = ip64_arp_create_arp_request(ip64_packet_buffer,
					&ip64_packet_buffer[sizeof(struct ip64_eth_hdr)]);
      return IP64_ETH_DRIVER.output(ip64_packet_buffer, len);
    }
  }
  else{ // if failed to convert to ipv4, just send it as ipv6

	  if(uip_ipaddr_prefixcmp(&UIP_IP_BUF->destipaddr, &prefix, 16)) //mcast
	  {
		  slip_send();
		  return 0;
	  }
	  else{
	  struct ip64_eth_hdr *ethhdr;
	    ethhdr = (struct ip64_eth_hdr *)&ip64_packet_buffer[UIP_LLH_LEN];
	    len=uip_len;

	    memcpy(&ip64_packet_buffer[sizeof(struct ip64_eth_hdr)],&uip_buf[UIP_LLH_LEN],uip_len);

	    ret = ip64_arp_create_ethhdr(ip64_packet_buffer,
	    				   &ip64_packet_buffer[sizeof(struct ip64_eth_hdr)]);
	        if(ret > 0)
	        	len += ret;
	    ethhdr->type = UIP_HTONS(IP64_ETH_TYPE_IPV6);
	  return IP64_ETH_DRIVER.output(ip64_packet_buffer, len);
	  }
  }

  return 0;
}

/*---------------------------------------------------------------------------*/
static void
ip64_eth_interface_init(void)
{
  PRINTF("ip64-eth-interface: init\n");
  tcpip_set_outputfunc(ip64_eth_interface_output);
}
/*---------------------------------------------------------------------------*/
const struct network_driver ip64_eth_driver = {
  "ip64_eth",
  ip64_eth_interface_init,
  ip64_eth_interface_input,
};
/*---------------------------------------------------------------------------*/
