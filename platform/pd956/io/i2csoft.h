/**********************************************************

Software I2C Library for AVR Devices.

Copyright 2008-2012
eXtreme Electronics, India
www.eXtremeElectronics.co.in
**********************************************************/


#ifndef _I2CSOFT_H
#define _I2CSOFT_H

/* 
I/O Configuration 
*/
#include "gpio.h"
#include "board-peripherals.h"

#define DEBUGGING 1
#if DEBUGGING // This is more or less the default setting

	#ifdef NODE_BMP280
		#define SCL            PIO_PB3
		#define SDA            PIO_PB2
	#endif

	#ifdef NODE_HTU21D
		#define SCL            PIO_PB2
		#define SDA            PIO_PB3

		//#define SCL            PIO_PA6
		//#define SDA            PIO_PA8
	#endif

	#ifndef SCL
		#define SCL            PIO_PA6
		#define SDA            PIO_PA8
	#endif

#else
	#define SCL            PIO_PB7
	#define SDA            PIO_PB6
#endif



#define SOFT_I2C_SDA_LOW	I2C_base->PIO_CODR = (SDA)
#define SOFT_I2C_SDA_HIGH	I2C_base->PIO_SODR = (SDA)

#define SOFT_I2C_SCL_LOW	I2C_base->PIO_CODR = (SCL)
#define SOFT_I2C_SCL_HIGH	I2C_base->PIO_SODR = (SCL)

#define SOFT_I2C_READ_PIN(x)	((I2C_base->PIO_PDSR & x)?1:0)


/**********************************************************
SoftI2CInit()

Description:
	Initializes the Soft I2C Engine.
	Must be called before using any other lib functions.
	
Arguments:
	NONE
	
Returns:
	Nothing

**********************************************************/
void SoftI2CInit(void);

/**********************************************************
SoftI2CStart()

Description:
	Generates a START(S) condition on the bus.
	NOTE: Can also be used for generating repeat start(Sr)
	condition too.
	
Arguments:
	NONE
	
Returns:
	Nothing

**********************************************************/
void SoftI2CStart(void);

/**********************************************************
SoftI2CStop()

Description:
	Generates a STOP(P) condition on the bus.
	NOTE: Can also be used for generating repeat start
	condition too.
	
Arguments:
	NONE
	
Returns:
	Nothing

**********************************************************/
void SoftI2CStop(void);

/**********************************************************
SoftI2CWriteByte()

Description:
	Sends a Byte to the slave.
	
Arguments:
	8 bit date to send to the slave.
	
Returns:
	non zero if slave acknowledge the data receipt.
	zero other wise.

**********************************************************/
uint8_t SoftI2CWriteByte(uint8_t data);

/**********************************************************
SoftI2CReadByte()

Description:
	Reads a byte from slave.
	
Arguments:
	1 if you want to acknowledge the receipt to slave.
	0 if you don't want to acknowledge the receipt to slave.
	
Returns:
	The 8 bit data read from the slave.

**********************************************************/
uint8_t SoftI2CReadByte(uint8_t ack);


void SoftI2CSync(void);


//uint8_t SoftI2Cread_char_register(uint8_t Addr, uint8_t reg);
//void SoftI2Cwrite_char_register(uint8_t Addr, uint8_t reg, uint8_t data);
uint16_t SoftI2Cread_int_register(uint8_t Addr, uint8_t reg);
void SoftI2Cwrite_int_register(uint8_t Addr, uint8_t reg, uint16_t data);

//uint16_t SoftI2Cread_register(uint8_t Addr, uint8_t reg, uint8_t *Data, uint8_t len);
#endif
