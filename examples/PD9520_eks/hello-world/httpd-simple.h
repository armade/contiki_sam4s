/*
 * Copyright (c) 2010, Swedish Institute of Computer Science.
 * Copyright (c) 2014, Texas Instruments Incorporated - http://www.ti.com/
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
 */
/*---------------------------------------------------------------------------*/
/**
 * \file
 *         Header file for the HTTPD of the cc26xx web demo example.
 * \author
 *         Adam Dunkels <adam@sics.se>
 *         Niclas Finne <nfi@sics.se>
 *         Joakim Eriksson <joakime@sics.se>
 *         Texas Instruments Incorporated - http://www.ti.com/
 */
/*---------------------------------------------------------------------------*/
#ifndef HTTPD_SIMPLE_H_
#define HTTPD_SIMPLE_H_
/*---------------------------------------------------------------------------*/
#include "contiki-net.h"
#include "sys/process.h"
/*---------------------------------------------------------------------------*/
/* Ideally a multiple of TCP_MSS */
#ifdef HTTPD_SIMPLE_CONF_MAIN_BUF_SIZE
#define HTTPD_SIMPLE_MAIN_BUF_SIZE HTTPD_SIMPLE_CONF_MAIN_BUF_SIZE
#else
#define HTTPD_SIMPLE_MAIN_BUF_SIZE (UIP_TCP_MSS*4)
#endif
/*---------------------------------------------------------------------------*/
#define HTTPD_PATHLEN  31
#define HTTPD_INBUF_LEN (HTTPD_PATHLEN )

#define TMP_BUF_SIZE   (UIP_TCP_MSS + 1)
/*---------------------------------------------------------------------------*/
/* POST request handlers */
#define HTTPD_SIMPLE_POST_HANDLER_OK      1
#define HTTPD_SIMPLE_POST_HANDLER_UNKNOWN 0
#define HTTPD_SIMPLE_POST_HANDLER_ERROR   0xFFFFFFFF

/**
 * \brief Datatype for a handler which can process incoming POST requests
 * \param key The configuration key to be updated
 * \param key_len The length of the key argument
 * \param val The new configuration value for key
 * \param val_len The length of the value argument
 *
 * \return 1: HTTPD_SIMPLE_POST_HANDLER_OK if the function can handle the
 * request, HTTPD_SIMPLE_POST_HANDLER_UNKNOWN if it does not know how to handle
 * it. HTTPD_SIMPLE_POST_HANDLER_ERROR if it does know how to handle it but
 * the request was malformed.
 */
typedef struct httpd_simple_post_handler {
  struct httpd_simple_post_handler *next;
  int (*handler)(char *key, int key_len, char *val, int val_len);
} httpd_simple_post_handler_t;

/* Declare a handler */
#define HTTPD_SIMPLE_POST_HANDLER(name, fp) \
  httpd_simple_post_handler_t name##_handler = { NULL, fp }

/**
 * \brief Register a handler for POST requests
 * \param h A pointer to the handler structure
 */
void httpd_simple_register_post_handler(httpd_simple_post_handler_t *h);
/*---------------------------------------------------------------------------*/
/*
 * An event generated by the HTTPD when a new configuration request has been
 * received
 */
extern process_event_t httpd_simple_event_new_config;
/*---------------------------------------------------------------------------*/
PROCESS_NAME(httpd_simple_process);
/*---------------------------------------------------------------------------*/
#endif /* HTTPD_SIMPLE_H_ */
