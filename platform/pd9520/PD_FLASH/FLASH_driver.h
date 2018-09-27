/*
 * FLASH_driver.h
 *
 *  Created on: 14. jun. 2017
 *      Author: pm
 */

#ifndef COMMON_SERVICES_USB_CLASS_CDC_DEVICE_EXAMPLE_PD_FLASH_FLASH_DRIVER_H_
#define COMMON_SERVICES_USB_CLASS_CDC_DEVICE_EXAMPLE_PD_FLASH_FLASH_DRIVER_H_

typedef enum{
	erase_4k_block,
	erase_64k_block
}erase_block_t;

int32_t flash_read_df(uint8_t *dst, uint32_t size, uint32_t addr);
int32_t flash_write_df(uint32_t dst, uint8_t *src, uint32_t bytes);
int32_t flash_erase_df(uint32_t addr, erase_block_t block);
uint32_t flash_size_df(void);
void flash_enter_deep_sleep(void);
void flash_leave_deep_sleep(void);
void flash_init_df(void);

#endif
