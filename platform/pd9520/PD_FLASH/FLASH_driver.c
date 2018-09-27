/*
 * FLASH_driver.c
 *
 *  Created on: 14. jun. 2017
 *      Author: pm
 */
#include <stdint.h>
#include <stdio.h>
#include "FLASH_driver.h"
#include "hl_spiflash.h"


static uint32_t flash_size = 0;

int32_t flash_read_df(uint8_t *dst, uint32_t size, uint32_t addr)
{
	if(flash_size == 0)
		return 1;

	sf_sread_start(addr);
	while(--size)
		*dst++ = sf_sread();

	*dst++ = sf_sread_end();

	return 0;
}

int32_t flash_write_df(uint32_t dst, uint8_t *src, uint32_t bytes)
{
	if(flash_size == 0)
		return 1;

	return sf_write(dst, src, bytes);
}

int32_t flash_erase_df(uint32_t addr, erase_block_t block)
{
	if(flash_size == 0)
		return 1;

	if(block == erase_4k_block)
		return sf_erase4k(addr);

	if(block == erase_64k_block)
		return sf_erase64k(addr);

	return 1;
}

uint32_t flash_size_df(void)
{
	return flash_size;
}

void flash_enter_deep_sleep(void)
{
	sf_deepsleep_enter();
}

void flash_leave_deep_sleep(void)
{
	sf_deepsleep_leave();
}

void flash_init_df(void)
{
	if(1 == sf_flashinit()){
		flash_size = 0;
		printf("E - Serial FLASH NOT found\n\r");
	}
	else{
		flash_size = sf_totalsize();
	}

	flash_enter_deep_sleep();
}


