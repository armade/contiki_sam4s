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

Pio *I2C_base = (Pio *)PIOA;

#define NOP(NO, unused)      asm volatile("NOP");

#define Q_DEL MREPEAT(90,NOP,~)		//ca. 300 ns
#define H_DEL MREPEAT(130,NOP,~) MREPEAT(140,NOP,~)//270	//ca. 900 ns


void SoftI2CInit(void)
{
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
	 	
	 for(i=0;i<8;i++)
	 {
		SOFT_I2C_SCL_LOW;
		
		if(data & 0x80)		SOFT_I2C_SDA_HIGH;
		else				SOFT_I2C_SDA_LOW;
		
		H_DEL;
		
		SOFT_I2C_SCL_HIGH;
		H_DEL;
		
		while(SOFT_I2C_READ_PIN(SCL)==0);
			
		data=data<<1;
	}
	 
	//The 9th clock (ACK Phase)
	SOFT_I2C_SCL_LOW;
	Q_DEL;
		
	SOFT_I2C_SDA_HIGH;
	H_DEL;
		
	SOFT_I2C_SCL_HIGH;
	H_DEL;

	while(SOFT_I2C_READ_PIN(SCL)==0);

	uint8_t ack=!(SOFT_I2C_READ_PIN(SDA));

	SOFT_I2C_SCL_LOW;
	H_DEL;
	SOFT_I2C_SCL_HIGH;
	return ack;
}
 
 
uint8_t SoftI2CReadByte(uint8_t ack)
{
	uint8_t data=0x00;
	uint8_t i;
			
	for(i=0;i<8;i++)
	{
		SOFT_I2C_SCL_LOW;
		H_DEL;
		SOFT_I2C_SCL_HIGH;
		H_DEL;
			
		while(SOFT_I2C_READ_PIN(SCL)==0);
		
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




