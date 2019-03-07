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

#define DEBUG 0
#if DEBUG
#include <stdio.h>
#define PRINTF(...) printf(__VA_ARGS__)
#else
#define PRINTF(...)
#endif

void i2c_reset(void);

//Pio *I2C_base = (Pio *)PIOB;
Pio *I2C_base = (Pio *)PIOA;

#define NOP(NO, unused)      asm volatile("NOP");

#if !LOW_CLOCK
	#define H_DEL MREPEAT(108,NOP,~)	//ca. 900 ns
#else
	#define H_DEL MREPEAT(30,NOP,~)		//ca. 2000 ns
#endif


// Pull: drives the line to level LOW
void _i2c_pull(int pin)
{
	I2C_base->PIO_CODR = pin;
	H_DEL;
}

// Release: releases the line and return line status
int _i2c_release(int pin)
{
	I2C_base->PIO_SODR = pin;
	H_DEL;
	return SOFT_I2C_READ_PIN(pin);
}

// In case of clock stretching or busy bus we must wait
// But not wait forever though
void _i2c_release_wait(int pin)
{
	int n = 0;

	while (!_i2c_release(pin)) {
		if (++n >= 5000000)	{
			PRINTF("Warning: I2C Bus busy or defective. Pin %d is LOW for 4.5s.\n", pin);
			return;
		}
	}

}

// Start: pull SDA while SCL is up
// Best practice is to ensure the bus is not busy before start
void i2c_start(void)
{
	if (!_i2c_release(SDA))
		i2c_reset();
    _i2c_release_wait(SCL);

	_i2c_pull(SDA);
	_i2c_pull(SCL);
}

// Stop: release SDA while SCL is up
void i2c_stop(void)
{
	_i2c_release_wait(SCL);
	if (!_i2c_release(SDA))
		i2c_reset();
}


// Reset bus sequence
void i2c_reset(void)
{
	//uint8_t i;
	uint16_t n = 0;

	_i2c_release(SDA);

	do {
		if (++n >= 1000) {
			PRINTF("Warning: I2C Bus busy or defective. SDA doesn't go UP after reset.\n");
			return;
		}
		_i2c_pull(SCL);
		_i2c_release_wait(SCL);
	} while (!_i2c_release(SDA));

	//_i2c_pull(SCL);
	//_i2c_pull(SDA);

	i2c_stop();
}

// Sends 0 or 1:
// Clock down, send bit, clock up, wait, clock down again
// In clock stretching, slave holds the clock line down in order
// to force master to wait before send more data
void i2c_send_bit(int bit)
{
	if (bit)	_i2c_release(SDA);
	else		_i2c_pull(SDA);

	_i2c_release_wait(SCL);
	_i2c_pull(SCL);
	_i2c_pull(SDA);//why??
}

// Reads a bit from sda
int i2c_read_bit(void)
{
	int s;

	_i2c_release(SDA);//why?? perhaps valid in first round. but remove from here...
	_i2c_release_wait(SCL);
	s = SOFT_I2C_READ_PIN(SDA);
	_i2c_pull(SCL);
	//_i2c_pull(SDA);//why??

	return s;
}

// Sends 8 bit in a row, MSB first and reads ACK.
// Returns I2C_ACK if device ack'ed
int i2c_send_byte(uint8_t byte)
{
	uint8_t i;

	for (i = 0; i < 8; i++) {
		i2c_send_bit(byte & 0x80);
		byte = byte << 1;
	}

	return !i2c_read_bit();
}

// Reads a byte, MSB first
uint8_t i2c_read_byte(void) {
	uint8_t byte = 0x00;
	uint8_t i;

	for (i=0; i<8; i++)
		byte = (byte << 1) | i2c_read_bit();

	return byte;
}
//////////////////////////////////////////////////////////////////////////////////////////////

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
	i2c_reset();
}

void SoftI2CSync(void)
{
	i2c_reset();
}

void SoftI2CStart(void)
{
	i2c_start();
}

void SoftI2CStop(void)
{
	i2c_stop();
}

uint8_t SoftI2CWriteByte(uint8_t data)
{
	return i2c_send_byte(data);
}
 
#define I2C_ACK		0
#define I2C_NACK 	1

uint8_t SoftI2CReadByte(uint8_t ack)
{
	uint8_t data;

	data = i2c_read_byte();
	
	if (ack)
		i2c_send_bit(I2C_ACK); // 0 is a positive ack
	else
		i2c_send_bit(I2C_NACK); // 1 is NOT acknowledge
	
	return data;

}




