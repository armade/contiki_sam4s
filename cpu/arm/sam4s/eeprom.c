/*
 * Copyright (c) 2018, Swedish Institute of Computer Science.
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
 * Author: Peter Mikkelsen <pm@proces-data.com>
 *
 */

#include <stdio.h>
#include <string.h>

#include "dev/eeprom.h"
#include "flash_efc.h"

static uint8_t usr_sig[512] = { 0 };


/* NB: SAM4S does not have EEPROM so we use the userpage cached
 * in ram for speedy reads. When accessing the userpage, read
 * and write from/to flash is prohibited.
 */

/*---------------------------------------------------------------------------*/
void eeprom_write(eeprom_addr_t addr, unsigned char *buf, int size)
{
	if((addr + size) > sizeof(usr_sig))
		return;
	// Don't program if content is the same
	if(!memcmp((void*) &usr_sig[addr], buf, size))
		return;

	//modify write
	memcpy((void*) &usr_sig[addr], buf, size);
	// The page must be erased before programming
	flash_erase_user_signature();
	flash_write_user_signature(usr_sig, 128);// NB operates in 32 bit
}
/*---------------------------------------------------------------------------*/
void eeprom_read(eeprom_addr_t addr, unsigned char *buf, int size)
{
	if((addr + size) > sizeof(usr_sig))
		return;
	memcpy(buf, (void*) &usr_sig[addr], size);
}
/*---------------------------------------------------------------------------*/
void eeprom_init(void)
{
	flash_read_user_signature((void*) usr_sig, 128); // NB operates in 32 bit
}
