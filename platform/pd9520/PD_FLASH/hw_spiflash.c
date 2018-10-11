
#include <stdint.h>
#include "hw_spiflash.h"
#include "hl_spiflash.h"
#include "spi_master.h"
#include "drivers/pmc.h"
#include "platform-conf.h"

#define FLASH_SPI_ID ID_QSPI
#define SPI_FLASH QSPI
#define NUMCS_FLASH 0

#define MAXFLASHCHIPS 1

#define SPCK 75000000		// SPI clock frequency, in Hz.


/** Calculates the value of the CSR SCBR field given the baudrate and MCK. */
#define QSPI_SCBR(masterClock,baudrate) \
	((uint32_t) (masterClock / baudrate) << 8)

/** Calculates the value of the CSR DLYBS field given the desired delay (in ns) */
#define QSPI_DLYBS(masterClock,delay) \
	((uint32_t) (((masterClock / 1000000) * delay) / 1000) << 16)

#define CSRxF_Q          (\
						 QSPI_DLYBS(M_CPU, 250) | \
						 QSPI_SCBR(M_CPU, SPCK))



#if SF_MAXFLASHCHIPS!=1
	unsigned char sf_select;
#endif


void kickdog(void) {};

void sf_waiting(void)
{
	kickdog();
}



void Configure_flash_delay_timer(void)
{
	pmc_enable_periph_clk(FLASH_TIMER_ID);
	TC0->TC_CHANNEL[0].TC_CCR =  TC_CCR_CLKDIS;
	TC0->TC_CHANNEL[0].TC_IDR =  0xFFFFFFFF;
	TC0->TC_CHANNEL[0].TC_SR;
	TC0->TC_CHANNEL[0].TC_CMR = 1; //E70:1=mck div 8
	TC0->TC_CHANNEL[0].TC_CCR = TC_CCR_CLKEN | TC_CCR_SWTRG;
}

void tdelay(void) //min 250ns delay, og mindst 10 bittider ved laveste bitrate
{
	unsigned dt,t0;
	t0=TC0->TC_CHANNEL[0].TC_CV; //18.75MHz, 9.4 clk per 500ns, det er 11 bittider ved 22MHz
	do {
		dt=(TC0->TC_CHANNEL[0].TC_CV-t0)&0xFFFF;
	} while (dt<10);
}

void delay_microseconds(unsigned tik)
{
	unsigned dt,t0,t1;
	t1=TC0->TC_CHANNEL[0].TC_CV;
	dt=0;
	//tik*=19; //150MHz /8 = 18.75 MHz

	tik=(tik*19)-(tik>>2);
	do {
		t0=t1;
		t1=TC0->TC_CHANNEL[0].TC_CV;
		dt+=(t1-t0)&0xFFFF;
	} while (dt<tik);
}

void sf_tlongdelay(void) //min 400us
{
	delay_microseconds(400);
}

void sf_tshortdelay(void) //min 30us
{
	delay_microseconds(30);
}

static void s_unselectChip(Qspi *p_spi)
{

	p_spi->QSPI_CR = QSPI_CR_QSPIEN | QSPI_CR_LASTXFER;
}

void f_disable(void) //min 50ns = 8 bus cycles
{
	sf_tshortdelay();
	s_unselectChip(SPI_FLASH);

}


void sf_disable(void) //min 50ns = 8 bus cycles
{
	//QSPI errata workaround version
		QSPI->QSPI_CR = QSPI_CR_LASTXFER;
		tdelay();
		SPI_FLASH->QSPI_MR =  QSPI_MR_CSMODE(1) /*uden wdrbt flag*/; /*WDRBT flaget bruger vi som CS-indikator-variabel*/

}

void sf_enable(void)
{
	//se om vi allerede er enabled, i saa fald disable
		if (QSPI->QSPI_MR & QSPI_MR_WDRBT) sf_disable();
		SPI_FLASH->QSPI_MR =  QSPI_MR_CSMODE(1)|QSPI_MR_WDRBT; /*WDRBT flaget bruger vi som CS-indikator-variabel*/
		//autoenabler ved 1. byte

}



unsigned char sf_rw(unsigned char data)
{
	volatile Qspi *spi = SPI_FLASH;

	while (!(spi->QSPI_SR & QSPI_SR_TDRE))			//SPI Ready
		;
	spi->QSPI_TDR = data;							//Clock Data byte ud
	while (!(spi->QSPI_SR & (QSPI_SR_RDRF)))
		; 									   		//Byte clocket ud/ind
	return spi->QSPI_RDR;							//Data byte som er clocket ind
}



unsigned char sf_r(void)
{
	return sf_rw(0);
}





void sf_startstreamread(void)
{
	volatile Qspi *spi=SPI_FLASH;
		while (!(spi->QSPI_SR & QSPI_SR_TDRE)) ;
		spi->QSPI_TDR = 0;

}


unsigned char sf_sread(void)
{

	volatile Qspi *spi=SPI_FLASH;
		unsigned hold;

	 	while ((spi->QSPI_SR & (QSPI_SR_TDRE|QSPI_SR_RDRF)) != (QSPI_SR_TDRE|QSPI_SR_RDRF)) { }
		//while (!(spi->SPI_SR & AT91C_SPI_TDRE)) ;
		hold=spi->QSPI_RDR;
		spi->QSPI_TDR = hold;
	 	return hold;

}

unsigned char sf_endstreamread(void)  //the failing sf_read ends up with 01000006 in SR here= TX empty, TDRE not empty, RFRF full
{

	volatile Qspi *spi=SPI_FLASH;
		unsigned d;
		while ((spi->QSPI_SR & (QSPI_SR_TDRE|QSPI_SR_RDRF)) != (QSPI_SR_TDRE|QSPI_SR_RDRF)) { }
		d=  spi->QSPI_RDR;
		return d;

}

void hw_spi_init(void)
{
	volatile Qspi *spi = SPI_FLASH;

	pmc_enable_periph_clk(ID_QSPI);  				// Enable FLASH SPI peripheral

  	spi->QSPI_CR = QSPI_CR_SWRST;	  					//Reset SPI
  	spi->QSPI_CR = QSPI_CR_SWRST;	  					//Twice, according to framework...

	spi->QSPI_CR = QSPI_CR_QSPIEN;					//Enable the SPI TX & RX
	while(!(spi->QSPI_SR & QSPI_SR_QSPIENS));

	spi->QSPI_MR =  QSPI_MR_CSMODE(1) | QSPI_MR_WDRBT_ENABLED;			//Chip Select Mode = 1 når der køres uden DMA
  	spi->QSPI_SCR = CSRxF_Q;

  	Configure_flash_delay_timer();
}
