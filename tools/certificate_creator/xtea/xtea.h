/*
 * xtea.h
 *
 *  Created on: Jan 22, 2016
 *      Author: pm
 */

#ifndef XTEA_H_
#define XTEA_H_

void encipher_payload_xtea(uint8_t * payload, const uint32_t *key, uint32_t length, uint64_t iv);
void decipher_payload_xtea(uint8_t * payload, const uint32_t *key, uint32_t length, uint64_t iv);

#endif /* XTEA_H_ */
