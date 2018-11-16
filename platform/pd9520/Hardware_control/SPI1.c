#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include "contiki.h"
#include "platform-conf.h"
#include "same70.h"
#include "gpio.h"
#include "pio_handler.h"
#include "SPI1.h"
#include "component/spi.h"
#include "drivers/spi.h"


#define DEBUG 1
#if DEBUG
#define PRINTF(...) printf(__VA_ARGS__)
#else
#define PRINTF(...)
#endif

#define SPI1_MISO_BIT 26
#define SPI1_MOSI_BIT 27
#define SPI1_SCK_BIT 24
#define SPI1_CS0_BIT 25
#define SPI1_CS1_BIT 28
#define SPI1_CS2_BIT 29
#define SPI1PORT PIOC

#define DATAADR_A_BIT PIO_PD14
#define DATAADR_B_BIT PIO_PD13
#define DATAADR_C_BIT PIO_PD12

#define SPI1_CS0 PIO_PC25
#define SPI1_CS1 PIO_PC28
#define SPI1_CS2 PIO_PC29

#define PIN_mux_CS PIO_PA16





void __Set_port(Pio *p_pio,uint32_t pin, uint8_t val)
{
	val?
		(p_pio->PIO_SODR = pin):
		(p_pio->PIO_CODR = pin);
}

volatile unsigned UPS = 0;
spi_device_t old_spi_device=0;

void SPI1_select(spi_device_t spi_device)
{

	if((UPS==1) && (spi_device != none))
		PRINTF("BAD SPI mutex!!\n");

	switch(spi_device)
	{
		case SD_card: // NB: SD card has it's own CS pin, that is not controlled here, since the driver must have full control. (CS-SD)
			UPS = 1;
			PRINTF("SD_card cs\n");
			__Set_port(PIOA,PIN_mux_CS,0); // MISO is switched to data storage
			__Set_port(PIOC,DATAADR_C_BIT,1); // 0b111 is sd card
			__Set_port(PIOC,DATAADR_B_BIT,1);
			__Set_port(PIOC,DATAADR_A_BIT,1);
			SPI1PORT->PIO_SODR = (1<<SPI1_CS0_BIT)|(1<<SPI1_CS1_BIT)|(1<<SPI1_CS2_BIT); // Deselect MUX, DAC, ADC
			break;
			
		case Seriel_flash_1:
			PRINTF("Seriel_flash_1\n");
			UPS = 1;
			__Set_port(PIOA,PIN_mux_CS,0); // MISO is switched to data storage
			__Set_port(PIOC,DATAADR_C_BIT,0); // 0b000 is Seriel_flash_1
			__Set_port(PIOC,DATAADR_B_BIT,0);
			__Set_port(PIOC,DATAADR_A_BIT,0);
			SPI1PORT->PIO_SODR = (1<<SPI1_CS0_BIT)|(1<<SPI1_CS1_BIT)|(1<<SPI1_CS2_BIT); // Deselect MUX, DAC, ADC
			break;
			
		case Seriel_flash_2:
			PRINTF("Seriel_flash_2\n");
			UPS = 1;
			__Set_port(PIOA,PIN_mux_CS,0); // MISO is switched to data storage
			__Set_port(PIOC,DATAADR_C_BIT,1); // 0b001 is Seriel_flash_2
			__Set_port(PIOC,DATAADR_B_BIT,0);
			__Set_port(PIOC,DATAADR_A_BIT,0);
			SPI1PORT->PIO_SODR = (1<<SPI1_CS0_BIT)|(1<<SPI1_CS1_BIT)|(1<<SPI1_CS2_BIT); // Deselect MUX, DAC, ADC
			break;
			
		case Seriel_flash_3:
			PRINTF("Seriel_flash_3\n");
			UPS = 1;
			__Set_port(PIOA,PIN_mux_CS,0); // MISO is switched to data storage
			__Set_port(PIOC,DATAADR_C_BIT,0); // 0b010 is Seriel_flash_3
			__Set_port(PIOC,DATAADR_B_BIT,1);
			__Set_port(PIOC,DATAADR_A_BIT,0);
			SPI1PORT->PIO_SODR = (1<<SPI1_CS0_BIT)|(1<<SPI1_CS1_BIT)|(1<<SPI1_CS2_BIT); // Deselect MUX, DAC, ADC
			break;
			
		case Seriel_flash_4:
			PRINTF("Seriel_flash_4\n");
			UPS = 1;
			__Set_port(PIOA,PIN_mux_CS,0); // MISO is switched to data storage
			__Set_port(PIOC,DATAADR_C_BIT,1); // 0b011 is Seriel_flash_4
			__Set_port(PIOC,DATAADR_B_BIT,1);
			__Set_port(PIOC,DATAADR_A_BIT,0);
			SPI1PORT->PIO_SODR = (1<<SPI1_CS0_BIT)|(1<<SPI1_CS1_BIT)|(1<<SPI1_CS2_BIT); // Deselect MUX, DAC, ADC
			break;
			
		case Seriel_flash_5:
			PRINTF("Seriel_flash_5\n");
			UPS = 1;
			__Set_port(PIOA,PIN_mux_CS,0); // MISO is switched to data storage
			__Set_port(PIOC,DATAADR_C_BIT,0); // 0b100 is Seriel_flash_5
			__Set_port(PIOC,DATAADR_B_BIT,0);
			__Set_port(PIOC,DATAADR_A_BIT,1);
			SPI1PORT->PIO_SODR = (1<<SPI1_CS0_BIT)|(1<<SPI1_CS1_BIT)|(1<<SPI1_CS2_BIT); // Deselect MUX, DAC, ADC
			break;
			
		case Seriel_flash_6:
			PRINTF("Seriel_flash_6\n");
			UPS = 1;
			__Set_port(PIOA,PIN_mux_CS,0); // MISO is switched to data storage
			__Set_port(PIOC,DATAADR_C_BIT,1); // 0b101 is Seriel_flash_6
			__Set_port(PIOC,DATAADR_B_BIT,0);
			__Set_port(PIOC,DATAADR_A_BIT,1);
			SPI1PORT->PIO_SODR = (1<<SPI1_CS0_BIT)|(1<<SPI1_CS1_BIT)|(1<<SPI1_CS2_BIT); // Deselect MUX, DAC, ADC
			break;
	
		case Seriel_flash_7:
			PRINTF("Seriel_flash_7\n");
			UPS = 1;
			__Set_port(PIOA,PIN_mux_CS,0); // MISO is switched to data storage
			__Set_port(PIOC,DATAADR_C_BIT,1); // 0b110 is Seriel_flash_7
			__Set_port(PIOC,DATAADR_B_BIT,1);
			__Set_port(PIOC,DATAADR_A_BIT,0);
			SPI1PORT->PIO_SODR = (1<<SPI1_CS0_BIT)|(1<<SPI1_CS1_BIT)|(1<<SPI1_CS2_BIT); // Deselect MUX, DAC, ADC
			break;
			
		case DAC:
			UPS = 1;
			PRINTF("DAC\n");
			SPI1_set_baudrate(40000000,SPI_CSR_CPOL,SPI_CSR_NCPHA);
			__Set_port(PIOA,PIN_mux_CS,1); // MISO is switched to floating generator
			SPI1PORT->PIO_CODR = 1<<SPI1_CS1_BIT;
			break;	
			
		case ADC:
			UPS = 1;
			PRINTF("ADC\n");
			SPI1_set_baudrate(8000000,0,SPI_CSR_NCPHA);//SPI_CSR_CPOL
			__Set_port(PIOA,PIN_mux_CS,1); // MISO is switched to floating generator
			SPI1PORT->PIO_CODR = 1<<SPI1_CS0_BIT;
			break;	
			
		case MUX:
			UPS = 1;
			PRINTF("MUX\n");
			SPI1_set_baudrate(10000000,SPI_CSR_CPOL,SPI_CSR_NCPHA);
			__Set_port(PIOA,PIN_mux_CS,1); // MISO is switched to floating generator
			SPI1PORT->PIO_CODR = 1<<SPI1_CS2_BIT;
			break;

		case none:
		default:
			SPI1_set_baudrate(60000000,SPI_CSR_CPOL,SPI_CSR_NCPHA);
			PRINTF("Default - none\n");
			__Set_port(PIOA,PIN_mux_CS,1); // MISO is switched to floating generator
			SPI1PORT->PIO_SODR = (1<<SPI1_CS0_BIT)|(1<<SPI1_CS1_BIT)|(1<<SPI1_CS2_BIT); // clr all cs
			UPS = 0;
			break;
	}
	old_spi_device = spi_device;
}	
#define SPI_PCS(npcs)       SPI_MR_PCS((~(1 << npcs) & 0xF))
void SPI_Write(uint32_t dwNpcs, uint16_t wData)
{
	/* Send data */
	while ((SPI1->SPI_SR & SPI_SR_TXEMPTY) == 0);

	SPI1->SPI_TDR = wData | SPI_PCS(dwNpcs);

	while ((SPI1->SPI_SR & SPI_SR_TDRE) == 0);
}

uint32_t SPI_Read(void)
{
	while ((SPI1->SPI_SR & SPI_SR_RDRF) == 0);

	return SPI1->SPI_RDR & 0xFFFF;
}

uint32_t SPI_IsFinished(Spi *spi)
{
	return ((spi->SPI_SR & SPI_SR_TXEMPTY) != 0);
}


void SPI1_write_buffer_wait(uint8_t *wData, uint16_t len)
{
	unsigned timeout;
	while(len--)
		SPI_Write( 0 , *wData++);
	
	timeout = 10000;
	while(!SPI_IsFinished(SPI1)){
		if(timeout-- == 0)
			return;
	}
}



void SPI1_read_buffer_wait(uint8_t *read, uint16_t len,	uint16_t dummy)
{
	unsigned timeout;
	while(len--)
	{
		timeout = 10000;
		SPI_Write( 0 , dummy);
		while(!SPI_IsFinished(SPI1)){
			if(timeout-- == 0)
				return;
		}
		*read++ = SPI_Read();
	}
}

void SPI1_init(void)
{
	pio_set_peripheral(PIOC, PIO_PERIPH_C, PIO_PC26);	// SPI MISO
	pio_set_peripheral(PIOC, PIO_PERIPH_C, PIO_PC27);	// SPI MOSI
	pio_set_peripheral(PIOC, PIO_PERIPH_C, PIO_PC24);	// SPI SPCK

	pio_set_peripheral(PIOD, PIO_OUTPUT_1, DATAADR_C_BIT);
	pio_set_peripheral(PIOD, PIO_OUTPUT_1, DATAADR_B_BIT);
	pio_set_peripheral(PIOD, PIO_OUTPUT_1, DATAADR_A_BIT);
	pio_set_peripheral(PIOC, PIO_OUTPUT_1, SPI1_CS0);
	pio_set_peripheral(PIOC, PIO_OUTPUT_1, SPI1_CS1);
	pio_set_peripheral(PIOC, PIO_OUTPUT_1, SPI1_CS2);

	pio_set_peripheral(PIOC, PIO_OUTPUT_1, PIN_mux_CS);

	spi_enable_clock(SPI1);
	SPI1->SPI_CR = SPI_CR_SPIDIS;
	SPI1->SPI_CR = SPI_CR_SWRST;
	SPI1->SPI_CR = SPI_CR_SWRST;
	SPI1->SPI_MR =(SPI_MR_MSTR | SPI_MR_MODFDIS );

	SPI1->SPI_CR = SPI_CR_SPIEN;//SPI_Enable(SPI1);
	SPI1_set_baudrate(60000000,SPI_CSR_CPOL,SPI_CSR_NCPHA);
}

void SPI_ConfigureNPCS(Spi *spi, uint32_t dwNpcs, uint32_t dwConfiguration)
{
	spi->SPI_CSR[dwNpcs] = dwConfiguration;
}


/** Calculates the value of the CSR SCBR field given the baudrate and MCK. */
#define SPI_SCBR(baudrate, masterClock) \
	SPI_CSR_SCBR((uint32_t)(masterClock / baudrate))

/** Calculates the value of the CSR DLYBS field given the desired delay (in ns) */
#define SPI_DLYBS(delay, masterClock)  \
	SPI_CSR_DLYBS((uint32_t) (((masterClock / 1000000) * delay) / 1000)+1)

/** Calculates the value of the CSR DLYBCT field given the desired delay (in ns) */
#define SPI_DLYBCT(delay, masterClock) \
	SPI_CSR_DLYBCT ((uint32_t) (((masterClock / 1000000) * delay) / 32000)+1)

void SPI1_set_baudrate(unsigned clock, unsigned pol, unsigned phase)
{
	SPI_ConfigureNPCS( SPI1,
					0,
					SPI_DLYBCT( 200, M_CPU ) |
					SPI_DLYBS( 200, M_CPU) |
					SPI_SCBR( clock, M_CPU) |
					pol | phase) ;
}

void sf_array_startstreamread(void)
{
	volatile Spi *spi=SPI1;
		while (!(spi->SPI_SR & SPI_SR_TDRE)) ;
		spi->SPI_TDR = 0;

}


unsigned char sf_array_sread(void)
{

	volatile Spi *spi=SPI1;
		unsigned hold;

	 	while ((spi->SPI_SR & (SPI_SR_TDRE|SPI_SR_RDRF)) != (SPI_SR_TDRE|SPI_SR_RDRF)) { }
		//while (!(spi->SPI_SR & AT91C_SPI_TDRE)) ;
		hold=spi->SPI_RDR;
		spi->SPI_TDR = hold;
	 	return hold;

}

unsigned char sf_array_endstreamread(void)  //the failing sf_read ends up with 01000006 in SR here= TX empty, TDRE not empty, RFRF full
{

	volatile Spi *spi=SPI1;
		unsigned d;
		while ((spi->SPI_SR & (SPI_SR_TDRE|SPI_SR_RDRF)) != (SPI_SR_TDRE|SPI_SR_RDRF)) { }
		d=  spi->SPI_RDR;
		return d;

}
