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
#include "contiki-conf.h"
#include "lib/sensors.h"
#include "htu21d-sensor.h"
#include "sys/ctimer.h"
#include "lib/sensors.h"
#include "i2csoft.h"

#include <stdint.h>
#include <string.h>
#include <stdio.h>

#include "board-peripherals.h"
#if 1
//#ifdef NODE_HTU21D
/*---------------------------------------------------------------------------*/
#define DEBUG 1
#if DEBUG
#define PRINTF(...) printf(__VA_ARGS__)
#else
#define PRINTF(...)
#endif
// HTU21 device address
#define HTU21_ADDR											0x40 //0b1000000

// HTU21 device commands
#define HTU21_RESET_COMMAND									0xFE
#define HTU21_READ_TEMPERATURE_HOLD_COMMAND					0xE3
#define HTU21_READ_TEMPERATURE_NO_HOLD_COMMAND				0xF3
#define HTU21_READ_HUMIDITY_HOLD_COMMAND					0xE5
#define HTU21_READ_HUMIDITY_NO_HOLD_COMMAND					0xF5
#define HTU21_READ_SERIAL_FIRST_8BYTES_COMMAND				0xFA0F
#define HTU21_READ_SERIAL_LAST_6BYTES_COMMAND				0xFCC9
#define HTU21_WRITE_USER_REG_COMMAND						0xE6
#define HTU21_READ_USER_REG_COMMAND							0xE7

#define RESET_TIME											15			// ms value

// Processing constants
#define HTU21_TEMPERATURE_COEFFICIENT						(-15)
#define HTU21_CONSTANT_A									(float)(8.1332)
#define HTU21_CONSTANT_B									(float)(1762.39)
#define HTU21_CONSTANT_C									(float)(235.66)

// Coefficients for temperature computation
#define TEMPERATURE_COEFF_MUL								(175720)
#define TEMPERATURE_COEFF_ADD								(-46850)

// Coefficients for relative humidity computation
#define HUMIDITY_COEFF_MUL									(125000)
#define HUMIDITY_COEFF_ADD									(-6000)

// Conversion timings
#define HTU21_TEMPERATURE_CONVERSION_TIME_T_14b_RH_12b		50 	//ms
#define HTU21_TEMPERATURE_CONVERSION_TIME_T_13b_RH_10b		25	//ms
#define HTU21_TEMPERATURE_CONVERSION_TIME_T_12b_RH_8b		13	//ms
#define HTU21_TEMPERATURE_CONVERSION_TIME_T_11b_RH_11b		7	//ms
#define HTU21_HUMIDITY_CONVERSION_TIME_T_14b_RH_12b			16	//ms
#define HTU21_HUMIDITY_CONVERSION_TIME_T_13b_RH_10b			5	//ms
#define HTU21_HUMIDITY_CONVERSION_TIME_T_12b_RH_8b			3	//ms
#define HTU21_HUMIDITY_CONVERSION_TIME_T_11b_RH_11b			8	//ms

// HTU21 User Register masks and bit position
#define HTU21_USER_REG_RESOLUTION_MASK						0x81
#define HTU21_USER_REG_END_OF_BATTERY_MASK					0x40
#define HTU21_USER_REG_ENABLE_ONCHIP_HEATER_MASK			0x4
#define HTU21_USER_REG_DISABLE_OTP_RELOAD_MASK				0x2
#define HTU21_USER_REG_RESERVED_MASK						(~(		HTU21_USER_REG_RESOLUTION_MASK			\
																|	HTU21_USER_REG_END_OF_BATTERY_MASK		\
																|	HTU21_USER_REG_ENABLE_ONCHIP_HEATER_MASK	\
																|	HTU21_USER_REG_DISABLE_OTP_RELOAD_MASK ))
enum htu21_resolution
{
	htu21_resolution_t_14b_rh_12b = 0,
	htu21_resolution_t_12b_rh_8b,
	htu21_resolution_t_13b_rh_10b,
	htu21_resolution_t_11b_rh_11b
};
// HTU User Register values
// Resolution
#define HTU21_USER_REG_RESOLUTION_T_14b_RH_12b				0x00
#define HTU21_USER_REG_RESOLUTION_T_13b_RH_10b				0x80
#define HTU21_USER_REG_RESOLUTION_T_12b_RH_8b				0x01
#define HTU21_USER_REG_RESOLUTION_T_11b_RH_11b				0x81

// End of battery status
#define HTU21_USER_REG_END_OF_BATTERY_VDD_ABOVE_2_25V		0x00
#define HTU21_USER_REG_END_OF_BATTERY_VDD_BELOW_2_25V		0x40
// Enable on chip heater
#define HTU21_USER_REG_ONCHIP_HEATER_ENABLE					0x04
#define HTU21_USER_REG_OTP_RELOAD_DISABLE					0x02

static uint32_t htu21_temperature_conversion_time =
		HTU21_TEMPERATURE_CONVERSION_TIME_T_14b_RH_12b;
static uint32_t htu21_humidity_conversion_time =
		HTU21_HUMIDITY_CONVERSION_TIME_T_14b_RH_12b;
/*---------------------------------------------------------------------------*/

#define SENSOR_STATUS_TEMP_MEAS    20
#define SENSOR_STATUS_HUMID_MEAS   30


static int sensor_status = SENSOR_STATUS_DISABLED;
static uint8_t retry_count;

/*---------------------------------------------------------------------------*/
static struct ctimer startup_timer;
/*---------------------------------------------------------------------------*/
/* Wait SENSOR_STARTUP_DELAY clock ticks for the sensor to be ready - ~80ms */
#define SENSOR_STARTUP_DELAY 80*(1000/CLOCK_SECOND)



static int32_t HTU21_temp = 0;
static int32_t HTU21_humid = 0;

/*---------------------------------------------------------------------------*/
static inline uint16_t SoftI2Cread_register(uint8_t Addr, uint8_t *Data,
		uint8_t len)
{
	uint8_t i, ack;

	SoftI2CStart();
	ack = SoftI2CWriteByte((Addr << 1) | 1); //set LSB for read
	if(!ack){
		PRINTF("HTU21: NO ACK\n");
		return 0;
	}

	for (i = 0; i < (len - 1) ; i++)
		*Data++ = SoftI2CReadByte(1);

	*Data++ = SoftI2CReadByte(0);

	SoftI2CStop();

	return 1;
}

/*---------------------------------------------------------------------------*/

static
void SoftI2C_cmd(uint8_t Addr, uint8_t cmd, uint8_t stopbit)
{
	SoftI2CStart();
	SoftI2CWriteByte((Addr << 1) & 0xFE); //clr LSB for write
	SoftI2CWriteByte(cmd);
	if(stopbit)
		SoftI2CStop();
}

/*---------------------------------------------------------------------------*/

int htu21_read_user_register(uint8_t Addr, uint8_t *reg_value)
{
	SoftI2CStart();
	SoftI2CWriteByte((Addr << 1) & 0xFE); //clr LSB for write
	SoftI2CWriteByte(HTU21_READ_USER_REG_COMMAND);

	SoftI2CStart();
	SoftI2CWriteByte((Addr << 1) | 1); //set LSB for read
	*reg_value = SoftI2CReadByte(0);

	return 1;
}

/*---------------------------------------------------------------------------*/

int htu21_write_user_register(uint8_t Addr, uint8_t reg_value)
{
	SoftI2CStart();
	SoftI2CWriteByte((Addr << 1) & 0xFE); //clr LSB for write
	SoftI2CWriteByte(HTU21_WRITE_USER_REG_COMMAND);
	SoftI2CWriteByte(reg_value);
	SoftI2CStop();

	return 1;
}

/*---------------------------------------------------------------------------*/

static
int htu21_crc_check(uint16_t value, uint8_t crc)
{
	uint32_t polynom = 0x988000; // x^8 + x^5 + x^4 + 1
	uint32_t msb = 0x800000;
	uint32_t mask = 0xFF8000;
	uint32_t result = (uint32_t) value << 8; // Pad with zeros as specified in spec

	while(msb != 0x80){

		// Check if msb of current value is 1 and apply XOR mask
		if(result & msb)
			result = ((result ^ polynom) & mask) | (result & ~mask);

		// Shift by one
		msb >>= 1;
		mask >>= 1;
		polynom >>= 1;
	}
	if(result == crc)
		return 1;
	else{
		PRINTF("HTU21: Bad CRC (HTU21d)\n");
		return 0;
	}
}

/*---------------------------------------------------------------------------*/

static int htu21_read_serial_number(uint64_t * serial_number)
{
	uint8_t Last[6];
	uint8_t First[8];

	SoftI2CStart();
	SoftI2CWriteByte((HTU21_ADDR << 1) & 0xFE); //clr LSB for write
	SoftI2CWriteByte(HTU21_READ_SERIAL_FIRST_8BYTES_COMMAND >> 8);
	SoftI2CWriteByte(HTU21_READ_SERIAL_FIRST_8BYTES_COMMAND & 0xff);
	SoftI2Cread_register(HTU21_ADDR, First, 8); //SNB_3, crc, SNB2, crc, SNB1, crc, SNB_0, crc

	SoftI2CStart();
	SoftI2CWriteByte((HTU21_ADDR << 1) & 0xFE); //clr LSB for write
	SoftI2CWriteByte(HTU21_READ_SERIAL_LAST_6BYTES_COMMAND >> 8);
	SoftI2CWriteByte(HTU21_READ_SERIAL_LAST_6BYTES_COMMAND & 0xff);
	SoftI2Cread_register(HTU21_ADDR, Last, 6); // SNC_1, SNC_0, crc, SNA_1, SNA_0, crc
	SoftI2CStop();
	// Arranged: SNA_1 SNA_0 SNB_3 SNB_2 SNB_1 SNB_0 SNC_1 SNC_0
	// FIXED VALUES
	//	SNA_1 = 0x48 ,
	//	SNA_0 = 0x54 ,
	//	SNB_3 = 0x00 ,
	//	SNC_1 = 0x32
	*serial_number = ((uint64_t) Last[3] << 56) | ((uint64_t) Last[4] << 48)
			| ((uint64_t) First[0] << 40) | ((uint64_t) First[2] << 32)
			| ((uint64_t) First[4] << 24) | ((uint64_t) First[6] << 16)
			| ((uint64_t) Last[0] << 8) | ((uint64_t) Last[1] << 0);

	return 1;

}

/*---------------------------------------------------------------------------*/

static
int htu21_set_resolution(enum htu21_resolution res)
{
	uint8_t reg_value, tmp = 0;

	if(res == htu21_resolution_t_14b_rh_12b){
		tmp = HTU21_USER_REG_RESOLUTION_T_14b_RH_12b;
		htu21_temperature_conversion_time =	HTU21_TEMPERATURE_CONVERSION_TIME_T_14b_RH_12b;
		htu21_humidity_conversion_time = HTU21_HUMIDITY_CONVERSION_TIME_T_14b_RH_12b;
	} else if(res == htu21_resolution_t_13b_rh_10b){
		tmp = HTU21_USER_REG_RESOLUTION_T_13b_RH_10b;
		htu21_temperature_conversion_time =	HTU21_TEMPERATURE_CONVERSION_TIME_T_13b_RH_10b;
		htu21_humidity_conversion_time = HTU21_HUMIDITY_CONVERSION_TIME_T_13b_RH_10b;
	} else if(res == htu21_resolution_t_12b_rh_8b){
		tmp = HTU21_USER_REG_RESOLUTION_T_12b_RH_8b;
		htu21_temperature_conversion_time =	HTU21_TEMPERATURE_CONVERSION_TIME_T_12b_RH_8b;
		htu21_humidity_conversion_time = HTU21_HUMIDITY_CONVERSION_TIME_T_12b_RH_8b;
	} else if(res == htu21_resolution_t_11b_rh_11b){
		tmp = HTU21_USER_REG_RESOLUTION_T_11b_RH_11b;
		htu21_temperature_conversion_time = HTU21_TEMPERATURE_CONVERSION_TIME_T_11b_RH_11b;
		htu21_humidity_conversion_time = HTU21_HUMIDITY_CONVERSION_TIME_T_11b_RH_11b;
	}

	htu21_read_user_register(HTU21_ADDR, &reg_value);

	// Clear the resolution bits
	reg_value &= ~HTU21_USER_REG_RESOLUTION_MASK;
	reg_value |= tmp & HTU21_USER_REG_RESOLUTION_MASK;

	htu21_write_user_register(HTU21_ADDR, reg_value);

	return 1;
}

/*---------------------------------------------------------------------------*/

static
char htu21_is_connected(void)
{
	char i2c_status;
	// Do the transfer
	SoftI2CStart();
	i2c_status = SoftI2CWriteByte((HTU21_ADDR << 1) & 0xFE); //clr LSB for write
	SoftI2CStop();
	return i2c_status;
}

/*---------------------------------------------------------------------------*/
static int32_t htu21_compute_compensated_humidity(int32_t temperature,
		int32_t relative_humidity)
{
	return (relative_humidity - (((25000 - temperature) * 78643) >> 19)); // 78643/2^19 = 0.1499996
}

static
void htu21_convert_temperature_and_relative_humidity(int32_t *temperature,int32_t *humidity)
{
	int64_t adc = *temperature;

	if (*temperature != SENSOR_ERROR)
		*temperature = ((adc * TEMPERATURE_COEFF_MUL) >> 16) + TEMPERATURE_COEFF_ADD;

	adc = *humidity;

	if (*humidity != SENSOR_ERROR) {
		*humidity = ((adc * HUMIDITY_COEFF_MUL) >> 16) + HUMIDITY_COEFF_ADD;
		*humidity = htu21_compute_compensated_humidity(*temperature, *humidity);

		if (*humidity >= 100000)
			*humidity = 100000;
	}

	return;

}

static void htu21_error(void)
{
	PRINTF("HTU21: ERROR encountered\n");
	HTU21_humid = SENSOR_ERROR;
	HTU21_temp = SENSOR_ERROR;
	SoftI2CSync();
	SoftI2C_cmd(HTU21_ADDR, HTU21_RESET_COMMAND, 1);
	// Apply default values
	htu21_temperature_conversion_time = HTU21_TEMPERATURE_CONVERSION_TIME_T_14b_RH_12b;
	htu21_humidity_conversion_time = HTU21_HUMIDITY_CONVERSION_TIME_T_14b_RH_12b;
	sensor_status = SENSOR_STATUS_READY;
	sensors_changed(&HTU21D_sensor);
}
/*
static void htu21_hold_func(void) {
	uint8_t buffer[3] = { 0 };
	//uint16_t ret;

	sensor_status = SENSOR_STATUS_TEMP_MEAS;
	SoftI2C_cmd(HTU21_ADDR, HTU21_READ_TEMPERATURE_HOLD_COMMAND, 0);
	SoftI2Cread_register(HTU21_ADDR, buffer, 3);
	HTU21_temp = (buffer[0] << 8) | buffer[1];
	if (!htu21_crc_check(HTU21_temp, buffer[2])) {
		PRINTF("HTU21: CRC check failed (temperature)\n");
		htu21_error();
		return;
	}
	sensor_status = SENSOR_STATUS_HUMID_MEAS;
	SoftI2C_cmd(HTU21_ADDR, HTU21_READ_HUMIDITY_HOLD_COMMAND, 0);
	SoftI2Cread_register(HTU21_ADDR, buffer, 3);
	HTU21_humid = (buffer[0] << 8) | buffer[1];
	if (!htu21_crc_check(HTU21_humid, buffer[2])) {
		PRINTF("HTU21: CRC check failed (humidity)\n");
		htu21_error();
		return;
	}

	//Zero out the status bits but keep them in place
	HTU21_temp &= 0xFFFC;
	HTU21_humid &= 0xFFFC;
	// All done now convert and notify
	htu21_convert_temperature_and_relative_humidity(&HTU21_temp,
			&HTU21_humid);
	sensor_status = SENSOR_STATUS_READY;
	SoftI2CStop();
	sensors_changed(&HTU21D_sensor);
}*/

static void htu21_notify_ready(void *not_used)
{
	uint8_t buffer[3] = { 0 };
	uint16_t ret;

	if(retry_count > 5){
		htu21_error();
		return;
	}

	if(sensor_status == SENSOR_STATUS_TEMP_MEAS){

		ret = SoftI2Cread_register(HTU21_ADDR, buffer, 3);
		// If the internal processing is finished, the HTU21D(F)
		// sensor acknowledges the poll of the MCU and data can
		// be read by the MCU. If the measurement processing is
		// not finished, the HTU21D(F) sensor answers NACK
		// and start condition must be issued once more.
		//
		if(!ret){
			PRINTF("HTU21: Not ready for temperature reading\n");
			retry_count++;
			ctimer_set(&startup_timer, (1 * 1000) / CLOCK_SECOND, htu21_notify_ready, NULL); // 1ms retry
			return;
		}

		HTU21_temp = (buffer[0] << 8) | buffer[1];
		if(!htu21_crc_check(HTU21_temp, buffer[2])){
			PRINTF("HTU21: CRC check failed (temperature)\n");
			htu21_error();
			return;
		}
		//PRINTF("HTU21_temp: 0x%.2x 0x%.2x 0x%.2x\n",
		//		buffer[0], buffer[1], buffer[2]);

		// Initiate humidity measurement
		SoftI2C_cmd(HTU21_ADDR, HTU21_READ_HUMIDITY_NO_HOLD_COMMAND, 0);
		SoftI2CStop();
		sensor_status = SENSOR_STATUS_HUMID_MEAS;
		ctimer_set(&startup_timer, (htu21_humidity_conversion_time * 1000) / CLOCK_SECOND,
				htu21_notify_ready, NULL);
		// Test: if not working just remove
		//SoftI2CStop();
		retry_count = 0;

		return;
	}
	if(sensor_status == SENSOR_STATUS_HUMID_MEAS){

		ret = SoftI2Cread_register(HTU21_ADDR, buffer, 3);
		// If the internal processing is finished, the HTU21D(F)
		// sensor acknowledges the poll of the MCU and data can
		// be read by the MCU. If the measurement processing is
		// not finished, the HTU21D(F) sensor answers no ACK bit
		// and start condition must be issued once more.
		//
		if(!ret){
			PRINTF("HTU21: Not ready for humidity reading\n");
			retry_count++;
			ctimer_set(&startup_timer, (1 * 1000) / CLOCK_SECOND, htu21_notify_ready, NULL); // 1ms retry
			return;
		}

		HTU21_humid = (buffer[0] << 8) | buffer[1];
		if(!htu21_crc_check(HTU21_humid, buffer[2])){
			PRINTF("HTU21: CRC check failed (humidity)\n");
			htu21_error();
			return;
		}
		//PRINTF("HTU21_humid: 0x%.2x 0x%.2x 0x%.2x\n",
		//	buffer[0], buffer[1], buffer[2]);

		// Now calculate values

		//Zero out the status bits but keep them in place
		HTU21_temp &= 0xFFFC;
		HTU21_humid &= 0xFFFC;
		//PRINTF("0x%x   0x%x",HTU21_temp,HTU21_humid);
		// All done now convert and notify
		htu21_convert_temperature_and_relative_humidity(&HTU21_temp, &HTU21_humid);
		//PRINTF("    %d   %d\n",HTU21_temp,HTU21_humid);
		sensor_status = SENSOR_STATUS_READY;
		SoftI2CStop();
		SoftI2CSync(); // If i2c bus gets out of sync the next measurement is bad.
		sensors_changed(&HTU21D_sensor);
	}

}
/*---------------------------------------------------------------------------*/
/**
 * \brief Initalise the sensor
 */
static void htu21_init(void)
{
	// Reset the sensor
	SoftI2C_cmd(HTU21_ADDR, HTU21_RESET_COMMAND, 1);
	// Apply default values
	htu21_temperature_conversion_time = HTU21_TEMPERATURE_CONVERSION_TIME_T_14b_RH_12b;
	htu21_humidity_conversion_time = HTU21_HUMIDITY_CONVERSION_TIME_T_14b_RH_12b;
}
/*---------------------------------------------------------------------------*/
/**
 * \brief Enable/disable measurements
 * \param enable 0: disable, enable otherwise
 *
 * @return      none
 */
static void htu21_enable_sensor(bool enable)
{
	if(enable){
		//PRINTF("HTU21: Trig measurement\n");
		retry_count = 0;
		SoftI2C_cmd(HTU21_ADDR, HTU21_READ_TEMPERATURE_NO_HOLD_COMMAND, 0);
		SoftI2CStop();
		//htu21_hold_func();
	}
}

/*---------------------------------------------------------------------------*/
/**
 * \brief Returns a reading from the sensor
 * \param type HTU21D_SENSOR_TYPE_TEMP or HTU21D_SENSOR_TYPE_HUMID
 * \return Temperature (ï¿½C * 1000) or Humidity (%RH * 1000).
 */
static int htu21_value(int type)
{
	/*if(sensor_status != SENSOR_STATUS_READY){
		PRINTF("HTU21: Sensor disabled or starting up (%d)\n", sensor_status);
		return SENSOR_ERROR;
	}*/

	if((type != HTU21D_SENSOR_TYPE_TEMP) && type != HTU21D_SENSOR_TYPE_HUMID){
		PRINTF("HTU21: Invalid type\n");
		return SENSOR_ERROR;
	} else{

		if(type == HTU21D_SENSOR_TYPE_TEMP){
			return (int) HTU21_temp;
		} else if(type == HTU21D_SENSOR_TYPE_HUMID){
			return (int) HTU21_humid;
		}
		return SENSOR_ERROR;
	}
}
/*---------------------------------------------------------------------------*/
/**
 * \brief Configuration function for the BMP280 sensor.
 *
 * \param type Activate, enable or disable the sensor. See below
 * \param enable
 *
 * When type == SENSORS_HW_INIT we turn on the hardware
 * When type == SENSORS_ACTIVE and enable==1 we enable the sensor
 * When type == SENSORS_ACTIVE and enable==0 we disable the sensor
 */
static uint64_t HTU21_serial_number;
static int htu21_configure(int type, int enable)
{

	switch(type)
	{
		case SENSORS_HW_INIT:
			SoftI2CInit();
			if(!htu21_is_connected())
				return SENSOR_STATUS_DISABLED;

			htu21_read_serial_number(&HTU21_serial_number);
			uint8_t *snr_ptr = (uint8_t *) &HTU21_serial_number;
			PRINTF("HTU21: serial nr. 0x%.2x 0x%.2x 0x%.2x 0x%.2x 0x%.2x 0x%.2x 0x%.2x 0x%.2x\n",
					snr_ptr[0],	snr_ptr[1],	snr_ptr[2],	snr_ptr[3],
					snr_ptr[4],	snr_ptr[5],	snr_ptr[6],	snr_ptr[7]);

			htu21_init();
			sensor_status = SENSOR_STATUS_INITIALISED;

			break;
		case SENSORS_ACTIVE:
			/* Must be initialised first */
			if(sensor_status == SENSOR_STATUS_DISABLED){
				//sensors_changed(&HTU21D_sensor);
				return sensor_status;
			}
			if(enable){
				htu21_enable_sensor(1);
				sensor_status = SENSOR_STATUS_TEMP_MEAS;
				ctimer_set(&startup_timer,
						htu21_temperature_conversion_time*CLOCK_SECOND/1000, htu21_notify_ready,
						NULL);
			} else{
				ctimer_stop(&startup_timer);
				htu21_enable_sensor(0);
				sensor_status = SENSOR_STATUS_INITIALISED;
			}
			break;
		default:
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
static int htu21_status(int type)
{
	switch(type)
	{
		case SENSORS_ACTIVE:
		case SENSORS_READY:
			return sensor_status;

		default:
			return SENSOR_STATUS_DISABLED;
	}
}
/*---------------------------------------------------------------------------*/
SENSORS_SENSOR(HTU21D_sensor, "HTU21D", htu21_value, htu21_configure,
		htu21_status);
/*---------------------------------------------------------------------------*/
/** @} */

#endif
