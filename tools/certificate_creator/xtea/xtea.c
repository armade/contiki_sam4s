#include <time.h>
#include <stdlib.h>
#include <stdint.h>
#include "xtea.h"

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
