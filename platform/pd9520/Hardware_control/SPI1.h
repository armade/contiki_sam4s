
#ifndef SPI1_h
#define SPI1_h

#include "stdint.h"


typedef enum {
	SD_card,
	Seriel_flash_1,
	Seriel_flash_2,
	Seriel_flash_3,
	Seriel_flash_4,
	Seriel_flash_5,
	Seriel_flash_6,
	Seriel_flash_7,
	DAC,
	MUX,
	ADC,
	none
}spi_device_t;

void SPI1_select(spi_device_t spi_device);
void SPI1_write_buffer_wait(uint8_t *wData, uint16_t len);
void SPI1_read_buffer_wait(uint8_t *read, uint16_t len,	uint16_t dummy);
void SPI1_init(void);
void SPI1_set_baudrate(unsigned clock, unsigned pol, unsigned phase);

void SPI_Write(uint32_t dwNpcs, uint16_t wData);
uint32_t SPI_Read(void);
void sf_array_startstreamread(void);
unsigned char sf_array_sread(void);
unsigned char sf_array_endstreamread(void);
#endif 
