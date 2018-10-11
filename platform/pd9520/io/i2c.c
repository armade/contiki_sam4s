/*
 * i2s.c
 *
 *  Created on: 04/10/2018
 *      Author: pm
 */
#include <stdio.h>

#include "same70.h"
#include "twihs.h"
#include "gpio.h"
#include "drivers/pmc.h"

#define SCL            PIO_PA4
#define SDA            PIO_PA3

#define SOFT_I2C_SDA_LOW	PIOA->PIO_CODR = (SDA)
#define SOFT_I2C_SDA_HIGH	PIOA->PIO_SODR = (SDA)

#define SOFT_I2C_SCL_LOW	PIOA->PIO_CODR = (SCL)
#define SOFT_I2C_SCL_HIGH	PIOA->PIO_SODR = (SCL)
#define NOP(NO, unused)      asm volatile("NOP");
#define SOFT_I2C_READ_PIN(x)	((PIOA->PIO_PDSR & x)?1:0)

#define H_DEL MREPEAT(130,NOP,~) MREPEAT(140,NOP,~)//270	//ca. 900 ns


static void SoftI2CSync(void)
{
	volatile int retries = 20;
	// Enable PIO to controle the pin
	PIOA->PIO_PER = (SDA) | (SCL);

	// Set PIO high before enabling output to eliminate glitches
	PIOA->PIO_SODR = (SDA) | (SCL);
	PIOA->PIO_PUER = (SDA) | (SCL);

	// Enable OUTPUT
	PIOA->PIO_OER = (SDA) | (SCL);

	// Multidrive enable
	PIOA->PIO_MDER = (SDA) | (SCL);
	while(!SOFT_I2C_READ_PIN(SDA)){
		SOFT_I2C_SCL_HIGH;
		H_DEL;
		SOFT_I2C_SCL_LOW;
		H_DEL;
		SOFT_I2C_SCL_HIGH;

		if(!retries--)	break;
	}
	PIOA->PIO_PDR = (SDA) | (SCL);
}

static void TWI_Master( Twihs *pTwi, unsigned dwTwCk, unsigned dwMCk )
{
	unsigned dwCkDiv = 0;
	unsigned dwClDiv;
	unsigned dwOk = 0;

	SoftI2CSync();

	pTwi->TWIHS_CR = TWIHS_CR_SWRST;
	pTwi->TWIHS_RHR;

	pTwi->TWIHS_CR = TWIHS_CR_SVDIS;
	pTwi->TWIHS_CR = TWIHS_CR_MSEN;

	while ( !dwOk ) {
		dwClDiv = ((dwMCk / (2 * dwTwCk)) - 4) / (1<<dwCkDiv);

		if ( dwClDiv <= 255 ) 	dwOk = 1;
		else					dwCkDiv++;
	}

	pTwi->TWIHS_CWGR = 0;
	pTwi->TWIHS_CWGR = (dwCkDiv << 16) | (dwClDiv << 8) | dwClDiv;
}


#define _TWI_Stop( pTwi ) (pTwi->TWIHS_CR = TWIHS_CR_STOP)
#define _TWI_Read_Byte(pTwi) pTwi->TWIHS_RHR
#define _TWI_Write_Byte(pTwi, byte) (pTwi->TWIHS_THR = byte)
#define _TWI_Byte_Received(pTwi) ((pTwi->TWIHS_SR & TWIHS_SR_RXRDY) == TWIHS_SR_RXRDY)
#define _TWI_Transfer_Complete(pTwi) ((pTwi->TWIHS_SR & TWIHS_SR_TXCOMP) == TWIHS_SR_TXCOMP)
#define _TWI_Send_STOP_Condition(pTwi) (pTwi->TWIHS_CR |= TWIHS_CR_STOP)
#define _TWI_Byte_Sent(pTwi) ((pTwi->TWIHS_SR & TWIHS_SR_TXRDY) == TWIHS_SR_TXRDY)

static void TWI_Start_Read(Twihs *pTwi, unsigned char address, unsigned iaddress, unsigned isize)
{
	pTwi->TWIHS_MMR = (isize << 8) | TWIHS_MMR_MREAD | (address << 16);
	pTwi->TWIHS_IADR = iaddress;
	pTwi->TWIHS_CR = TWIHS_CR_START;
}

static void TWI_Start_Write( Twihs *pTwi, unsigned char address, unsigned iaddress, unsigned isize, unsigned char byte)
{
	pTwi->TWIHS_MMR = (isize << 8) | (address << 16);
	pTwi->TWIHS_IADR = iaddress;
	_TWI_Write_Byte(pTwi, byte);
}

volatile unsigned timeout = 100000;
unsigned TWI_Read(Twihs *pTwi, unsigned char address, unsigned iaddress, unsigned isize, unsigned char *pData, unsigned num)
{
	timeout = 100000;

	if (num == 1)
		_TWI_Stop(pTwi);

	TWI_Start_Read(pTwi, address, iaddress, isize);

	while (num > 0) {
		while( !_TWI_Byte_Received(pTwi))
		{

			if(!--timeout) {
				//printf("timeout i2c (addr:0x%x, reg: 0x%x num:%d)\n",address,iaddress,num);
				return 1;
			}
		}
		*pData++ = _TWI_Read_Byte(pTwi);
		if (--num == 1)
			_TWI_Stop(pTwi);
	}

	while( !_TWI_Transfer_Complete(pTwi) );

	return 0;
}

unsigned TWI_Write(Twihs *pTwi, unsigned char address, unsigned iaddress, unsigned isize, unsigned char *pData, unsigned num)
{
	timeout = 100000;

	TWI_Start_Write(pTwi, address, iaddress, isize, *pData++);
	num--;
	while (num > 0) {
		while( !_TWI_Byte_Sent(pTwi) )
		{

			if(!--timeout) {
				//printf("timeout i2c (addr:0x%x, reg: 0x%x num:%d)\n",address,iaddress,num);
				return 1;
			}
		}
		_TWI_Write_Byte(pTwi, *pData++);
		num--;
	}
	_TWI_Send_STOP_Condition(pTwi);
	while( !_TWI_Transfer_Complete(pTwi) ) ;

	return 0;
}

void init_i2c(void)
{
	pmc_enable_periph_clk(ID_TWIHS0);
	TWI_Master(TWIHS0, 400000, 150000000);
}

int i2c_write(uint8_t addr, uint8_t reg, uint8_t *val, uint8_t size)
{
	return TWI_Write(TWIHS0, addr, reg, 1, val, size);
}

int i2c_read(uint8_t addr, uint8_t reg, uint8_t *val, uint8_t size)
{
	return TWI_Read(TWIHS0, addr, reg, 1, val, size);
}
