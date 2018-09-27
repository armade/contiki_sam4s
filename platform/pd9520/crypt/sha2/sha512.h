/**
 * \file sha512.h
 *
 * \brief SHA-384 and SHA-512 cryptographic hash function
 *
 *  Copyright (C) 2006-2015, ARM Limited, All Rights Reserved
 *  SPDX-License-Identifier: Apache-2.0
 *
 *  Licensed under the Apache License, Version 2.0 (the "License"); you may
 *  not use this file except in compliance with the License.
 *  You may obtain a copy of the License at
 *
 *  http://www.apache.org/licenses/LICENSE-2.0
 *
 *  Unless required by applicable law or agreed to in writing, software
 *  distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 *  WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 *  See the License for the specific language governing permissions and
 *  limitations under the License.
 *
 *  This file is part of mbed TLS (https://tls.mbed.org)
 */
#ifndef MBEDTLS_SHA512_H
#define MBEDTLS_SHA512_H


#include <stddef.h>
#include <stdint.h>

#if !defined(MBEDTLS_SHA512_ALT)
// Regular implementation
//

#ifdef __cplusplus
extern "C" {
#endif

/**
 * \brief          SHA-512 context structure
 */
typedef struct
{
    uint64_t total[2];          /*!< number of bytes processed  */
    uint64_t state[8];          /*!< intermediate digest state  */
    unsigned char buffer[128];  /*!< data block being processed */
}SHA512_CTX;

typedef struct
{
    uint64_t total[2];          /*!< number of bytes processed  */
    uint64_t state[8];          /*!< intermediate digest state  */
    unsigned char buffer[128];  /*!< data block being processed */
}SHA384_CTX;

/**
 * \brief          Initialize SHA-512 context
 *
 * \param ctx      SHA-512 context to be initialized
 */
void sha2_sha512_init( SHA512_CTX *ctx);
void sha2_sha384_init( SHA384_CTX *ctx);


/**
 * \brief          SHA-512 process buffer
 *
 * \param ctx      SHA-512 context
 * \param input    buffer holding the  data
 * \param ilen     length of the input data
 */
void sha2_sha512_update( SHA512_CTX *ctx, const uint8_t data[], size_t len);
void sha2_sha384_update( SHA384_CTX *ctx, const uint8_t data[], size_t len);

/**
 * \brief          SHA-512 final digest
 *
 * \param ctx      SHA-512 context
 * \param output   SHA-384/512 checksum result
 */
void sha2_sha512_final( SHA512_CTX *ctx, uint8_t hash[]);
void sha2_sha384_final( SHA384_CTX *ctx, uint8_t hash[]);

#ifdef __cplusplus
}
#endif

#else  /* MBEDTLS_SHA512_ALT */
#include "sha512_alt.h"
#endif /* MBEDTLS_SHA512_ALT */

#ifdef __cplusplus
extern "C" {
#endif

/**
 * \brief          Output = SHA-512( input buffer )
 *
 * \param input    buffer holding the  data
 * \param ilen     length of the input data
 * \param output   SHA-384/512 checksum result
 * \param is384    0 = use SHA512, 1 = use SHA384
 */
void sha2_sha512( const unsigned char *input, size_t len,
             unsigned char hash[]);
void sha2_sha384( const unsigned char *input, size_t len,
             unsigned char hash[]);


#ifdef __cplusplus
}
#endif

#endif /* mbedtls_sha512.h */
