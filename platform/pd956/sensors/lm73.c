#include "contiki.h"
#include "contiki-conf.h"
#include "lib/sensors.h"

#include "compiler.h"
#include <gpio.h>
#include "pio_handler.h"
#include "i2csoft.h"
#include "lm73.h"

/*---------------------------------------------------------------------------*/
#define DEBUG 0
#if DEBUG
#define PRINTF(...) printf(__VA_ARGS__)
#else
#define PRINTF(...)
#endif
/*---------------------------------------------------------------------------*/

#define alarm_pin PIO_PB2

#define SENSOR_STATUS_DISABLED     0
#define SENSOR_STATUS_INITIALISED  1
#define SENSOR_STATUS_NOT_READY    2
#define SENSOR_STATUS_READY        3

static int sensor_status = SENSOR_STATUS_DISABLED;
static int current_type = 0;

#define BUSYWAIT_UNTIL(cond, max_time)                                  \
  do {                                                                  \
    static rtimer_clock_t t0;                                                  \
    t0 = RTIMER_NOW();                                                  \
    while(!(cond) && RTIMER_CLOCK_LT(RTIMER_NOW(), t0 + (max_time)));   \
  } while(0)
/*---------------------------------------------------------------------------*/
static uint8_t SoftI2Cread_char_register(uint8_t Addr, uint8_t reg)
{
	uint8_t lsb;

	SoftI2CStart();
	SoftI2CWriteByte((Addr << 1) & 0xFE); //clr LSB for write
	SoftI2CWriteByte(reg);

	SoftI2CStart();
	SoftI2CWriteByte((Addr << 1) | 1); //set LSB for read

	lsb = SoftI2CReadByte(0);

	SoftI2CStop();

	return lsb;
}
static
void SoftI2Cwrite_char_register(uint8_t Addr, uint8_t reg, uint8_t data)
{
	SoftI2CStart();
	SoftI2CWriteByte((Addr << 1) & 0xFE); //clr LSB for write
	SoftI2CWriteByte(reg);
	SoftI2CWriteByte(data);

	SoftI2CStop();

	return;
}

/*---------------------------------------------------------------------------*/

void lm73_alarm_irq(uint32_t a, uint32_t b)
{

}
/*---------------------------------------------------------------------------*/
void lm73_set_resolution(uint16_t res)
{
	uint16_t CR_register;
	unsigned char new_res = res - 11;

	if(new_res < 4){
		CR_register = SoftI2Cread_char_register(LM73_ADDRESS, 0x04);
		CR_register &= ~(0b11 << 5);
		CR_register |= (new_res << 5);
		SoftI2Cwrite_char_register(LM73_ADDRESS, 0x04, CR_register);
	}
}

uint16_t lm73_get_resolution(void)
{
	uint16_t CR_register;

	CR_register = SoftI2Cread_char_register(LM73_ADDRESS, 0x04);
	CR_register &= (0b11 << 5);
	CR_register >>= 5;

	return CR_register + 11;
}
/*---------------------------------------------------------------------------*/
void lm73_set_mode_power_down(void)
{
	uint16_t CR_register;

	CR_register = SoftI2Cread_char_register(LM73_ADDRESS, 0x01);
	CR_register |= (1 << 7);
	SoftI2Cwrite_char_register(LM73_ADDRESS, 0x01, CR_register);
}

void lm73_clr_mode_power_down(void)
{
	uint16_t CR_register;

	CR_register = SoftI2Cread_char_register(LM73_ADDRESS, 0x01);
	CR_register &= (1 << 7);
	SoftI2Cwrite_char_register(LM73_ADDRESS, 0x01, CR_register);
}
/*---------------------------------------------------------------------------*/
void lm73_oneshot(void) // 14-112ms depending on resolution
{
	uint16_t CR_register;

	CR_register = SoftI2Cread_char_register(LM73_ADDRESS, 0x01);
	CR_register |= (1 << 2);
	SoftI2Cwrite_char_register(LM73_ADDRESS, 0x01, CR_register);
}
/*---------------------------------------------------------------------------*/
void lm73_T_high_set(uint16_t temp)
{
	uint16_t TH_register = temp << 7;
	SoftI2Cwrite_char_register(LM73_ADDRESS, 0x02, TH_register);
}

uint16_t lm73_T_high_get(void)
{
	uint16_t TH_register;
	TH_register = SoftI2Cread_char_register(LM73_ADDRESS, 0x02);

	return TH_register >> 7;
}
/*---------------------------------------------------------------------------*/
void lm73_T_low_set(uint16_t temp)
{
	uint16_t TL_register = temp << 7;
	SoftI2Cwrite_char_register(LM73_ADDRESS, 0x03, TL_register);
}

uint16_t lm73_T_low_get(void)
{
	uint16_t TL_register;
	TL_register = SoftI2Cread_char_register(LM73_ADDRESS, 0x03);

	return TL_register >> 7;
}
/*---------------------------------------------------------------------------*/
uint16_t lm73_Identification_Register(void)
{
	return SoftI2Cread_char_register(LM73_ADDRESS, 0x07);
}
#if 0
float lm73_get_temperature(void)
{
	int16_t dat;
	float temperature;
	// In power down mode the temperature register will read -256ºC
	// after a oneshot request. When the measurement is done the
	// register will be updated with the correct value.
	do{
		dat = i2c_read_register(LM73_ADDRESS, 0x00);
	}while(dat == 0x8000);

	temperature = (float)dat/128.0;

	return temperature;
}
#endif
/*---------------------------------------------------------------------------*/
/**
 * \brief Returns a reading from the sensor
 * \return Temperature (ºC * 1000).
 */
static int lm73_get_temperature(int type)
{
	int16_t dat;
	int32_t tmp;

	if(current_type == LM73_SENSOR_TYPE_POWERDOWN)
		lm73_oneshot();
	// In power down mode the temperature register will read -256ºC
	// after a oneshot request. When the measurement is done the
	// register will be updated with the correct value.

	BUSYWAIT_UNTIL((dat=SoftI2Cread_int_register(LM73_ADDRESS, 0x00)) != 0x8000,
			114 * RTIMER_SECOND / 1000); // 14-112ms depending on resolution. Here 114ms

	if(dat == 0xffff)
		return SENSOR_ERROR;

	tmp = dat * 1000; // preserve some decimals
	dat = tmp >> 7; // Divide by 128 to get ºC
	return dat;
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
			if(lm73_Identification_Register() != 0x0190)
				return SENSOR_STATUS_DISABLED;

			lm73_set_resolution(14);
			if(lm73_get_resolution() != 14)
				return SENSOR_STATUS_DISABLED;

			//lm73_set_mode_power_down();
			enable = SENSOR_STATUS_INITIALISED;
			break;

		case SENSORS_ACTIVE:
			if(sensor_status == SENSOR_STATUS_DISABLED)
				return SENSOR_STATUS_DISABLED;

			if(enable){
				//lm73_clr_mode_power_down();
				sensor_status = SENSOR_STATUS_READY;
			} else{
				//lm73_set_mode_power_down();
				sensor_status = SENSOR_STATUS_NOT_READY;
			}
			break;

		case LM73_SENSOR_TYPE_POWERDOWN:
			lm73_set_mode_power_down();
			current_type = LM73_SENSOR_TYPE_POWERDOWN;

			pio_disable_pin_interrupt(alarm_pin);
			break;

		case LM73_SENSOR_TYPE_CONTINUESLY:
			lm73_clr_mode_power_down();
			current_type = LM73_SENSOR_TYPE_CONTINUESLY;

			pio_disable_pin_interrupt(alarm_pin);
			break;
		case LM73_SENSOR_TYPE_CONTINUESLY_alarm:
			lm73_clr_mode_power_down();
			current_type = LM73_SENSOR_TYPE_CONTINUESLY;

			// NO don't use !!!!!ioport_set_pin_dir(alarm_pin,IOPORT_DIR_INPUT);
			pio_handler_set(PIOA, ID_PIOA, alarm_pin, PIO_IT_HIGH_LEVEL,
					lm73_alarm_irq);
			NVIC_EnableIRQ((IRQn_Type) ID_PIOA);
			pio_enable_interrupt(PIOA, alarm_pin);
			pio_enable_pin_interrupt(alarm_pin);
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
SENSORS_SENSOR(LM73_sensor, "LM73", lm73_get_temperature, lm73_init,
		lm73_status);
/*---------------------------------------------------------------------------*/
