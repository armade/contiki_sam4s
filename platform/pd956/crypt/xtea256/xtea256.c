/*
* xtea256.c
*
*  Created on: 04/12/2018
*      Author: pm
*
*   Permission to use, copy, modify, and distribute this software for any
*   purpose with or without fee is hereby granted, provided that the above
*   copyright notice and this permission notice appear in all copies.
*
*   THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
*   WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
*   MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR
*   ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
*   WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN
*   ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF
*   OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.
*/

#include "xtea256.h"

static
void encipher(unsigned num_rounds, uint32_t *v, unsigned const key[4])
{
    unsigned i;
    unsigned v0=v[0], v1=v[1], sum=0, delta=0x9E3779B9; //0x9E3779CD
    for (i=0; i < num_rounds; i++){
        v0 += (((v1 << 4) ^ (v1 >> 5)) + v1) ^ (sum + key[sum & 3]);
        sum += delta;
        v1 += (((v0 << 4) ^ (v0 >> 5)) + v0) ^ (sum + key[(sum>>11) & 3]);
    }
    v[0]=v0; v[1]=v1;
}

static
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


void xtea256_encrypt_cbc(uint8_t * payload, const uint32_t *key, uint32_t length, uint64_t iv)
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


void xtea256_decrypt_cbc(uint8_t * payload, const uint32_t *key, uint32_t length, uint64_t iv)
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
