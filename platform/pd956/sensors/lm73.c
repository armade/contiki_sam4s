/*
 * Copyright © 2019, Peter Mikkelsen
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * ``AS IS'' AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL THE
 * COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 * STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED
 * OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */
#include "contiki.h"
#include "contiki-conf.h"
#include "lib/sensors.h"
#include <stdio.h>

#include "compiler.h"
#include <gpio.h>
#include "pio_handler.h"
#include "i2c.h"
#include "i2csoft.h"
#include "lm73.h"
#include "board-peripherals.h"

/*---------------------------------------------------------------------------*/
#define DEBUG 0
#if DEBUG
#define PRINTF(...) printf(__VA_ARGS__)
#else
#define PRINTF(...)
#endif

const uint8_t convertion_times[] = {
		14,//ms 11 bit res
		28,//ms	12 bit res
		56,//ms 13 bit res
		112,//ms 14 bit res
};
uint32_t lm73_temperature_conversion_time = 14;

static uint8_t LM73_addr;
#define LM73_ADDRESS 	LM73_addr

static int temperature = SENSOR_ERROR;

static struct ctimer startup_timer;
/*---------------------------------------------------------------------------*/

#define alarm_pin PIO_PB2

static int sensor_status = SENSOR_STATUS_DISABLED;



#define BUSYWAIT_UNTIL(cond, max_time)                                  \
  do {                                                                  \
    static rtimer_clock_t t0;                                                  \
    t0 = RTIMER_NOW();                                                  \
    while(!(cond) && RTIMER_CLOCK_LT(RTIMER_NOW(), t0 + (max_time)));   \
  } while(0)
/*---------------------------------------------------------------------------*/


#define LM73_0_float_addr 	0b1001000 //72 0x48
#define LM73_0_gnd_addr 	0b1001001 //73 0x49
#define LM73_0_vdd_addr 	0b1001010 //74 0x4a
#define LM73_1_float_addr 	0b1001100 //76 0x4c
#define LM73_1_gnd_addr 	0b1001101 //77 0x4d
#define LM73_1_vdd_addr 	0b1001110 //78 0x4e

uint8_t LM73_addr_holder[6] = {
	LM73_0_float_addr,
	LM73_0_gnd_addr,
	LM73_0_vdd_addr,
	LM73_1_float_addr,
	LM73_1_gnd_addr,
	LM73_1_vdd_addr
	};

#define LM73_addr(x)	LM73_addr_holder[x]

#define Swap16(x) x=__builtin_bswap16(x)

#define I2C_r	SoftI2Cread_char_register
#define I2C_w	SoftI2Cwrite_char_register

static uint8_t SoftI2Cread_char_register(uint8_t Addr, uint8_t reg, uint8_t *val, uint8_t size)
{
	uint8_t lsb;

	SoftI2CStart();
	lsb = SoftI2CWriteByte((Addr << 1) & 0xFE); //clr LSB for write
	if(!lsb)
		return 1;
	SoftI2CWriteByte(reg);

	SoftI2CStart();
	SoftI2CWriteByte((Addr << 1) | 1); //set LSB for read

	while(--size)
		*val++ = SoftI2CReadByte(1);
	*val = SoftI2CReadByte(0);

	SoftI2CStop();

	return 0;
}

static
void SoftI2Cwrite_char_register(uint8_t Addr, uint8_t reg, uint8_t *val, uint8_t size)
{
	SoftI2CStart();
	SoftI2CWriteByte((Addr << 1) & 0xFE); //clr LSB for write
	SoftI2CWriteByte(reg);
	while(size--)
	SoftI2CWriteByte(*val++);

	SoftI2CStop();

	return;
}

uint16_t Detect_lm73_addr(void)
{
	uint8_t i;
	uint16_t id, ret;

	for(i=0;i<6;i++)
	{
		LM73_addr = LM73_addr(i);
		ret =  I2C_r(LM73_addr, 0x07, (uint8_t *)&id, sizeof(id));
		if(!ret){
			Swap16(id);
			printf("LM73 found (0x%x)\n",id);
			return id;
		}
	}
	printf("E - LM73 not found!!!!\n");
	return 0;
}

/*---------------------------------------------------------------------------*/

void lm73_alarm_irq(uint32_t a, uint32_t b)
{

}
/*---------------------------------------------------------------------------*/
void lm73_set_resolution(uint16_t res)
{
	uint8_t CR_register;
	unsigned char new_res = res - 11;

	if(new_res < 4){
		I2C_r(LM73_ADDRESS, 0x04, &CR_register, sizeof(CR_register));
		CR_register &= ~(0b11 << 5);
		CR_register |= (new_res << 5);
		I2C_w(LM73_ADDRESS, 0x04, &CR_register, sizeof(CR_register));
	}

	lm73_temperature_conversion_time = convertion_times[new_res];
}

uint16_t lm73_get_resolution(void)
{
	uint8_t CR_register;

	I2C_r(LM73_ADDRESS, 0x04, &CR_register, sizeof(CR_register));
	CR_register &= (0b11 << 5);
	CR_register >>= 5;

	return CR_register + 11;
}
/*---------------------------------------------------------------------------*/
void lm73_set_mode_power_down(void)
{
	uint8_t CR_register;

	I2C_r(LM73_ADDRESS, 0x01, &CR_register, sizeof(CR_register));
	CR_register |= (1 << 7);
	I2C_w(LM73_ADDRESS, 0x01, &CR_register, sizeof(CR_register));
}

void lm73_clr_mode_power_down(void)
{
	uint8_t CR_register;

	I2C_r(LM73_ADDRESS, 0x01, &CR_register, sizeof(CR_register));
	CR_register &= (1 << 7);
	I2C_w(LM73_ADDRESS, 0x01, &CR_register, sizeof(CR_register));
}
/*---------------------------------------------------------------------------*/
void lm73_oneshot(void) // 14-112ms depending on resolution
{
	uint8_t CR_register;

	I2C_r(LM73_ADDRESS, 0x01, &CR_register, sizeof(CR_register));
	CR_register |= (1 << 2);
	I2C_w(LM73_ADDRESS, 0x01, &CR_register, sizeof(CR_register));
}
/*---------------------------------------------------------------------------*/
void lm73_T_high_set(uint16_t temp)
{
	uint8_t TH_register = temp << 7;
	I2C_w(LM73_ADDRESS, 0x02, &TH_register, sizeof(TH_register));
}

uint16_t lm73_T_high_get(void)
{
	uint8_t TH_register;
	I2C_r(LM73_ADDRESS, 0x02, &TH_register, sizeof(TH_register));

	return TH_register >> 7;
}
/*---------------------------------------------------------------------------*/
void lm73_T_low_set(uint16_t temp)
{
	uint8_t TL_register = temp << 7;
	I2C_w(LM73_ADDRESS, 0x03, &TL_register, sizeof(TL_register));
}

uint16_t lm73_T_low_get(void)
{
	uint8_t TL_register;
	I2C_r(LM73_ADDRESS, 0x03, &TL_register, sizeof(TL_register));

	return TL_register >> 7;
}
/*---------------------------------------------------------------------------*/
uint16_t lm73_Identification_Register(void)
{
	uint16_t id;

	I2C_r(LM73_ADDRESS, 0x07, (uint8_t *)&id, sizeof(id));
	Swap16(id);
	return id;
}
#if 0
static int lm73_get_temperature(int type)
{
	int16_t dat;
	float temperature;
	// In power down mode the temperature register will read -256ï¿½C
	// after a oneshot request. When the measurement is done the
	// register will be updated with the correct value.
	do{
		I2C_r(LM73_ADDRESS, 0x00, (uint8_t *)&dat, 2);
	}while(dat == 0x8000);

	temperature = (float)dat/128.0;
	printf("lm73 temp = %f\n",temperature);
	return ((int)dat*1000)>>7;
}
#endif
#if 1
/*---------------------------------------------------------------------------*/
/**
 * \brief Returns a reading from the sensor
 * \return Temperature (ï¿½C * 1000).
 */
static int lm73_get_temperature()
{
	volatile int16_t dat;
	volatile int32_t tmp;
	if(sensor_status == SENSOR_STATUS_DISABLED)
		return SENSOR_ERROR;
	// In power down mode the temperature register will read -256ï¿½C
	// after a oneshot request. When the measurement is done the
	// register will be updated with the correct value.
	tmp = 0x8000;

	do{
		//I2C_r(LM73_ADDRESS, 0x00, (uint8_t *)&dat, sizeof(dat));
		I2C_r(LM73_ADDRESS, 0x00, (uint8_t *)&dat, 2);
		Swap16(dat);
		tmp = dat;
		//printf("lm73 temp raw = 0x%x  (0x%x)\n",dat,tmp);
	}while(((tmp == 0x8000)||(tmp == 0)));
	 // 14-112ms depending on resolution. Here 114ms

	if(dat == 0xffff)
		return SENSOR_ERROR;


	tmp = dat;
	tmp *= 1000; // preserve some decimals
	tmp = tmp >> 7; // Divide by 128 to get ï¿½C

	//printf("lm73 temp = %d\n",tmp);
	return tmp;
}
#endif

static void lm73_notify_ready(void *not_used)
{
	sensor_status = SENSOR_STATUS_READY;
	temperature = lm73_get_temperature();
	sensors_changed(&LM73_sensor);
}

static int lm73_get_value(int type)
{
	return temperature;
}

/*---------------------------------------------------------------------------*/
/**
 * \brief Configuration function for the sensor.
 *
 * \param type Activate, enable or disable the sensor. See below
 * \param enable
 *
 * When type == SENSORS_HW_INIT we turn on the hardware
 * When type == SENSORS_ACTIVE and enable==1 we enable the sensor
 * When type == SENSORS_ACTIVE and enable==0 we disable the sensor
 */
static int lm73_init(int type, int enable)
{
	switch(type)
	{

		case SENSORS_HW_INIT:
			softI2CInit();
			if(Detect_lm73_addr() != 0x0190)
				return SENSOR_STATUS_DISABLED;

			lm73_set_resolution(14);
			if(lm73_get_resolution() != 14){
				printf("LM73 bad resolution\n");
				return SENSOR_STATUS_DISABLED;
			}
			lm73_get_temperature(type);
			lm73_set_mode_power_down();
			sensor_status = SENSOR_STATUS_INITIALISED;
			break;

		case SENSORS_ACTIVE:
			if(sensor_status == SENSOR_STATUS_DISABLED)
				return SENSOR_STATUS_DISABLED;

			if(enable){
				lm73_oneshot();
				sensor_status = SENSOR_STATUS_BUSY;
				ctimer_set(&startup_timer,
							(lm73_temperature_conversion_time * 1000) / CLOCK_SECOND,
							lm73_notify_ready,
							NULL);
			} else{
				ctimer_stop(&startup_timer);
				sensor_status = SENSOR_STATUS_NOT_READY;
			}
			break;

	}

	return sensor_status;
}

/*---------------------------------------------------------------------------*/
/**
 * \brief Returns the status of the sensor
 * \param type SENSORS_ACTIVE or SENSORS_READY
 * \return 1 if the sensor is enabled
 */
static int lm73_status(int type)
{
	return sensor_status;
}

/*---------------------------------------------------------------------------*/
SENSORS_SENSOR(LM73_sensor, "LM73", lm73_get_value, lm73_init,
		lm73_status);
/*---------------------------------------------------------------------------*/
