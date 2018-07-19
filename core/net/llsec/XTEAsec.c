/*
 * Copyright (c) 2018, PROCES-DATA.
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
 *         secure link layer security driver.
 * \author
 *         Peter Mikkelsen <pm@proces-data.com.com>
 */

/**
 * \addtogroup nullsec
 * @{
 */

#include "net/llsec/XTEAsec.h"
#include "net/mac/frame802154.h"
#include "net/netstack.h"
#include "net/packetbuf.h"
#include "net/ipv6/uip-ds6.h"

void encipher(unsigned num_rounds, uint32_t *v, unsigned const key[4])
{
    unsigned i;
    unsigned v0=v[0], v1=v[1], sum=0, delta=0x9E3779B9;
    for (i=0; i < num_rounds; i++){
        v0 += (((v1 << 4) ^ (v1 >> 5)) + v1) ^ (sum + key[sum & 3]);
        sum += delta;
        v1 += (((v0 << 4) ^ (v0 >> 5)) + v0) ^ (sum + key[(sum>>11) & 3]);
    }
    v[0]=v0; v[1]=v1;
}

void decipher(unsigned num_rounds, uint32_t *v, unsigned const key[4])
{
    unsigned i;
    unsigned v0=v[0], v1=v[1], delta=0x9E3779B9, sum=delta*num_rounds;
    for (i=0; i < num_rounds; i++) {
        v1 -= (((v0 << 4) ^ (v0 >> 5)) + v0) ^ (sum + key[(sum>>11) & 3]);
        sum -= delta;
        v0 -= (((v1 << 4) ^ (v1 >> 5)) + v1) ^ (sum + key[sum & 3]);
    }
    v[0]=v0; v[1]=v1;
}

void encipher_payload_xtea(uint8_t * payload, const uint32_t *key, uint32_t length, uint64_t iv)
{
	uint32_t i;

	if(length & 7)
		return;

	for (i=0;i<length;i+=8){
		*(uint64_t *)payload ^= iv;
		encipher(32, (uint32_t *)payload , (void *)key);
		iv = *(uint64_t *)payload;
		payload += 8;
	}
}

void decipher_payload_xtea(uint8_t * payload, const uint32_t *key, uint32_t length, uint64_t iv)
{
	uint32_t i;
	uint64_t hold;

	if(length & 7)
		return;

	for (i=0;i<length;i+=8){
		hold = *(uint64_t *)payload;
		decipher(32, (uint32_t *)payload, (void *)key);
		*(uint64_t *)payload ^= iv;
		iv = hold;
		payload += 8;
	}
}


uint8_t crypt_key32[32] = {
		0x72, 0xab, 0x7b, 0x23, 0x5f, 0x2d, 0xdd, 0x19,
		0x55, 0x13, 0xba, 0xdd, 0xa1, 0xf8, 0x92, 0x14,
		0xc3, 0xad, 0x3d, 0x41, 0x82, 0xa2, 0xa8, 0xfa,
		0x5f, 0xbe, 0xb5, 0xd9, 0x54, 0x25, 0x78, 0x77
};
uint8_t IV_crypt[8] = {
		0x44, 0xc3, 0xba, 0x43, 0x50, 0x82, 0x09, 0xf3
};

/*---------------------------------------------------------------------------*/
static void
init(void)
{

}
/*---------------------------------------------------------------------------*/
static void
send(mac_callback_t sent, void *ptr)
{
	uip_ds6_nbr_t *nbr;
	uint32_t len = packetbuf_datalen();
	uint8_t *data = packetbuf_dataptr();
	uint32_t space_left = packetbuf_remaininglen();
	uint8_t hotfix;
	uint8_t buf[100] = {0};

	if(uip_conn->lport == UIP_HTONS(1883))
	{
		//nbr = uip_ds6_nbr_lookup(UIP_IP_BUF->srcipaddr);
		hotfix = len & 7;
		if(hotfix && space_left > 8)
		len -= hotfix;

		memcpy(buf,data,len);

		encipher_payload_xtea(buf,(void *)crypt_key32, len,*(uint64_t *)IV_crypt);
		memcpy(data,buf,len);
		packetbuf_set_attr(PACKETBUF_ATTR_FRAME_TYPE, FRAME802154_DATAFRAME);
	}
	NETSTACK_MAC.send(sent, ptr);
}
/*---------------------------------------------------------------------------*/
static void
input(void)
{
	uint32_t len = packetbuf_datalen();
	uint8_t *data = packetbuf_dataptr();
	uint8_t hotfix;
	uint8_t buf[100] = {0};

	if(uip_conn->lport == UIP_HTONS(1883))
	{
		hotfix = len & 7;
		len -= hotfix;

		memcpy(buf,data,len);

		decipher_payload_xtea(buf,(void *)crypt_key32, len,*(uint64_t *)IV_crypt);
		memcpy(data,buf,len);
	}
	NETSTACK_NETWORK.input();
}
/*---------------------------------------------------------------------------*/
const struct llsec_driver XTEAsec_driver = {
  "XTEAsec",
  init,
  send,
  input
};
/*---------------------------------------------------------------------------*/

/** @} */
