/**********************************************************

Software I2C Library for AVR Devices.

Copyright 2008-2012
eXtreme Electronics, India
www.eXtremeElectronics.co.in
**********************************************************/
 


#include "i2csoft.h"
#include "platform-conf.h"
#include "gpio.h"
#include "pio_handler.h"
#if defined(NODE_HTU21D) || defined(NODE_BMP280) || defined(NODE_LM73)
Pio *I2C_base = (Pio *)PIOB;

#define NOP(NO, unused)      asm volatile("NOP");

#if !LOW_CLOCK
	#define Q_DEL MREPEAT(36,NOP,~)		//ca. 300 ns
	#define H_DEL MREPEAT(108,NOP,~)	//ca. 900 ns
#else
	#define Q_DEL MREPEAT(9,NOP,~)		//ca. 300 ns
	#define H_DEL MREPEAT(27,NOP,~)		//ca. 900 ns
#endif




void SoftI2CInit(void)
{
	// NB: Hijack system IO. The pins have 10k pull-up, and access to 3.3V and gnd.
	// OBS: can't be used when debugging
	if(PIO_PB7==SCL)
		REG_CCFG_SYSIO |= (SCL);

	if(PIO_PB6==SDA)
		REG_CCFG_SYSIO |= (SDA);

	// Enable PIO to controle the pin
	I2C_base->PIO_PER = (SDA) | (SCL);

	// Set PIO high before enabling output to eliminate glitches
	I2C_base->PIO_SODR = (SDA) | (SCL);
	I2C_base->PIO_PUER = (SDA) | (SCL);

	// Enable OUTPUT
	I2C_base->PIO_OER = (SDA) | (SCL);

	// Multidrive enable
	I2C_base->PIO_MDER = (SDA) | (SCL);

	// If I2C bus is out of sync, resync
	SoftI2CSync();
}

void SoftI2CSync(void)
{
	volatile int retries = 20;

	while(!SOFT_I2C_READ_PIN(SDA)){
		SOFT_I2C_SCL_HIGH;
		H_DEL;
		SOFT_I2C_SCL_LOW;
		H_DEL;
		SOFT_I2C_SCL_HIGH;

		if(!retries--)	break;
	}
}

void SoftI2CStart(void)
{
	SOFT_I2C_SCL_HIGH;
	Q_DEL;
	SOFT_I2C_SDA_LOW;	
	H_DEL;  	
}

void SoftI2CStop(void)
{
	 SOFT_I2C_SDA_LOW;
	 H_DEL;
	 SOFT_I2C_SCL_HIGH;
	 Q_DEL;
	 SOFT_I2C_SDA_HIGH;
	 H_DEL;
}

uint8_t SoftI2CWriteByte(uint8_t data)
{
	 
	 uint8_t i;
	 volatile int retries;
	 	
	 for(i=0;i<8;i++)
	 {
		SOFT_I2C_SCL_LOW;
		
		if(data & 0x80)		SOFT_I2C_SDA_HIGH;
		else				SOFT_I2C_SDA_LOW;
		
		H_DEL;
		
		SOFT_I2C_SCL_HIGH;
		H_DEL;
		
		retries = 2000;
		while(SOFT_I2C_READ_PIN(SCL)==0){
			if(!retries--)	return 0;
		}
			
		data=data<<1;
	}
	 
	//The 9th clock (ACK Phase)
	SOFT_I2C_SCL_LOW;
	Q_DEL;
		
	SOFT_I2C_SDA_HIGH;
	H_DEL;
		
	SOFT_I2C_SCL_HIGH;
	H_DEL;

	retries = 2000;
	while(SOFT_I2C_READ_PIN(SCL)==0){
		if(!retries--)	return 0;
	}

	uint8_t ack=!(SOFT_I2C_READ_PIN(SDA));

	SOFT_I2C_SCL_LOW;
	H_DEL;
	
	return ack;
	 
}
 
 
uint8_t SoftI2CReadByte(uint8_t ack)
{
	uint8_t data=0x00;
	uint8_t i;
	volatile int retries;
			
	for(i=0;i<8;i++)
	{
		SOFT_I2C_SCL_LOW;
		H_DEL;
		SOFT_I2C_SCL_HIGH;
		H_DEL;
			
		retries = 2000;
		while(SOFT_I2C_READ_PIN(SCL)==0){
			if(!retries--)	return 0;
		}
		
		if(SOFT_I2C_READ_PIN(SDA))
			data|=(0x80>>i);
	}
		
	SOFT_I2C_SCL_LOW;
	Q_DEL;						//Soft_I2C_Put_Ack
	
	if(ack)		SOFT_I2C_SDA_LOW;
	else		SOFT_I2C_SDA_HIGH;

	H_DEL;
	
	SOFT_I2C_SCL_HIGH;
	H_DEL;
	
	SOFT_I2C_SCL_LOW;
	H_DEL;
	SOFT_I2C_SDA_HIGH;

	return data;
	
}
/*
uint8_t SoftI2Cread_char_register(uint8_t Addr, uint8_t reg)
{
	uint8_t lsb;

    SoftI2CStart();
	SoftI2CWriteByte((Addr<<1)&0xFE); //clr LSB for write
	SoftI2CWriteByte(reg);

	SoftI2CStart();
	SoftI2CWriteByte((Addr<<1)|1); //set LSB for read

	lsb=SoftI2CReadByte(0);

	SoftI2CStop();

	return lsb;
}

void SoftI2Cwrite_char_register(uint8_t Addr, uint8_t reg, uint8_t data)
{
    SoftI2CStart();
	SoftI2CWriteByte((Addr<<1)&0xFE); //clr LSB for write
	SoftI2CWriteByte(reg);
	SoftI2CWriteByte(data);

	SoftI2CStop();

	return;
}

uint16_t SoftI2Cread_int_register(uint8_t Addr, uint8_t reg)
{
	uint8_t msb, lsb;

    SoftI2CStart();
	SoftI2CWriteByte((Addr<<1)&0xFE); //clr LSB for write
	SoftI2CWriteByte(reg);

	SoftI2CStart();
	SoftI2CWriteByte((Addr<<1)|1); //set LSB for read

	msb=SoftI2CReadByte(1);
	lsb=SoftI2CReadByte(0);

	SoftI2CStop();

	return (msb<<8)|lsb;
}

void SoftI2Cwrite_int_register(uint8_t Addr, uint8_t reg, uint16_t data)
{
	uint8_t msb, lsb;

	msb = data>>8;
	lsb = data & 0xff;

    SoftI2CStart();
	SoftI2CWriteByte((Addr<<1)&0xFE); //clr LSB for write
	SoftI2CWriteByte(reg);
	SoftI2CWriteByte(lsb);

	SoftI2CWriteByte(reg+1);
	SoftI2CWriteByte(msb);

	SoftI2CStop();

	return;
}
*/
/*
uint16_t SoftI2Cread_register(uint8_t Addr, uint8_t reg, uint8_t *Data, uint8_t len)
{
	uint8_t i;
    SoftI2CStart();
	SoftI2CWriteByte((Addr<<1)&0xFE); //clr LSB for write
	SoftI2CWriteByte(reg);

	SoftI2CStart();
	SoftI2CWriteByte((Addr<<1)|1); //set LSB for read

	for(i=0;i<(len-1);i++)
		*Data++=SoftI2CReadByte(1);

	*Data++=SoftI2CReadByte(0);

	SoftI2CStop();

	return 1;
}
*/

#endif


