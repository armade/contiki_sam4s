
#include <stdint.h>
#include "hw_spiflash.h"
#include "hl_spiflash.h"
#include "spi_master.h"

#define SPI_FLASH SPI
#define NUMCS_FLASH 0

#define MAXFLASHCHIPS 1

#define SPCK_F        60000000 /// SPI clock frequency, in Hz.


#if SF_MAXFLASHCHIPS!=1
	unsigned char sf_select;
#endif


void kickdog(void) {};

void sf_waiting(void)
{
	kickdog();
}




static void tdelay(void) //min 250ns delay, and at least 10 bit times with lowest bitrate
{
	volatile uint8_t i;
	for(i=0;i<30;i++)
		asm volatile("NOP");
}

static void delay_microseconds(unsigned tik)
{
	volatile uint32_t i;
	uint32_t time = tik*120;

	for(i=0;i<time;i++)
		asm volatile("NOP");
}

void sf_tshortdelay(void) //min 30us
{
	delay_microseconds(30);
}

void sf_tlongdelay(void) //min 400us
{
	delay_microseconds(400);
}


static void s_selectChip(Spi *p_spi, unsigned char id)
{
	spi_set_peripheral_chip_select_value(p_spi, (~(1 << id)));
}

#define NONE_CHIP_SELECT_ID 0x0f
static void s_unselectChip(Spi *p_spi)
{

	tdelay();
	while (!spi_is_tx_empty(p_spi));
	spi_set_peripheral_chip_select_value(p_spi, NONE_CHIP_SELECT_ID);
	spi_set_lastxfer(p_spi);
	tdelay();
}




void sf_disable(void) //min 50ns = 8 bus cycles
{
	s_unselectChip(SPI);
	tdelay();
	SPI_FLASH->SPI_MR &= ~(SPI_MR_WDRBT);
}

void sf_enable(void)
{
	//se om vi allerede er enabled, i saa fald disable
	if (SPI->SPI_MR & SPI_MR_WDRBT)
		sf_disable();

	SPI_FLASH->SPI_MR |= SPI_MR_WDRBT;
	s_selectChip(SPI,0);
}



unsigned char sf_rw(unsigned char data)
{
	Spi *p_spi=SPI_FLASH;
	while (!spi_is_tx_ready(p_spi));
	spi_write_single(p_spi, data);
	while (!spi_is_rx_ready(p_spi));
	return  p_spi->SPI_RDR;
}



unsigned char sf_r(void)
{
	return sf_rw(0);
}





void sf_startstreamread(void)
{
	volatile Spi *p_spi=SPI_FLASH;
	while (!(p_spi->SPI_SR & SPI_SR_TDRE)) ;
	p_spi->SPI_TDR = 0;
}


unsigned char sf_sread(void)
{

	volatile Spi *p_spi=SPI_FLASH;
	unsigned hold;

 	while ((p_spi->SPI_SR & (SPI_SR_TDRE|SPI_SR_RDRF)) != (SPI_SR_TDRE|SPI_SR_RDRF)) { }
	hold=p_spi->SPI_RDR;
	p_spi->SPI_TDR = hold;
 	return hold;
}

unsigned char sf_endstreamread(void)  //the failing sf_read ends up with 01000006 in SR here= TX empty, TDRE not empty, RFRF full
{
	volatile Spi *p_spi=SPI_FLASH;
	unsigned d;
	while ((p_spi->SPI_SR & (SPI_SR_TDRE|SPI_SR_RDRF)) != (SPI_SR_TDRE|SPI_SR_RDRF)) { }
	d=  p_spi->SPI_RDR;
	return d;
}

void hw_spi_init(void)
{
	struct spi_device device;

	device.id =0;

	spi_master_init(SPI_FLASH);
	spi_master_setup_device(SPI_FLASH, &device,	SPI_MODE_0, SPCK_F, 0);
	spi_set_transfer_delay(SPI_FLASH, device.id, 15, 0);
	spi_set_bits_per_transfer(SPI_FLASH, device.id,	8);

	spi_enable(SPI_FLASH);
}
