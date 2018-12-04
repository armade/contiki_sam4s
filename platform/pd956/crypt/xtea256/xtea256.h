/*
 * xtea256.h
 *
 *  Created on: 04/12/2018
 *      Author: pm
 */

#ifndef XTEA256_H_
#define XTEA256_H_

#include <stdint.h>

void xtea256_encrypt_cbc(uint8_t * payload, const uint32_t *key, uint32_t length, uint64_t iv);
void xtea256_decrypt_cbc(uint8_t * payload, const uint32_t *key, uint32_t length, uint64_t iv);


#endif /* XTEA256_H_ */
