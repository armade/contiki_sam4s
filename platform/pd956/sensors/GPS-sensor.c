/*
 * Copyright (c) 2014, Texas Instruments Incorporated - http://www.ti.com/
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
 */
/*---------------------------------------------------------------------------*/
/**
 * \addtogroup sensortag-cc26xx-bmp-sensor
 * @{
 *
 * \file
 *  Driver for the Sensortag BMP280 Altimeter / Pressure Sensor
 */
/*---------------------------------------------------------------------------*/
#include "contiki-conf.h"
#include "lib/sensors.h"
#include "GPS-sensor.h"
#include "sys/ctimer.h"
#include "lib/sensors.h"
#include "apps/gpsd/minmea.h"


#include <stdint.h>
#include <string.h>
#include <stdio.h>
/*---------------------------------------------------------------------------*/
#define DEBUG 0
#if DEBUG
#define PRINTF(...) printf(__VA_ARGS__)
#else
#define PRINTF(...)
#endif

#define SENSOR_STARTUP_DELAY 1*(1000/CLOCK_SECOND)
static struct ctimer startup_timer;
static volatile float Temp_float_val;
static volatile float Temp_double_val;
/*---------------------------------------------------------------------------*/
static int enabled = SENSOR_STATUS_DISABLED;
/*---------------------------------------------------------------------------*/
static void
notify_ready(void *not_used)
{
	enabled = SENSOR_STATUS_READY;
	sensors_changed(&GPS_sensor);
}

/**
 * \brief Returns a reading from the sensor
 */
static int
value(int type)
{
	int rv;

	if(enabled != SENSOR_STATUS_READY) {
		PRINTF("Sensor disabled or starting up (%d)\n", enabled);
		return SENSOR_ERROR;
	}

	// We are not going to perform any calculations on any of the parameter.
	// We should just parse on the raw value. This is only for testing.
	switch(type){
		case GPS_SENSOR_TYPE_LAT:	Temp_double_val = minmea_tocoord_double(&frame_gga.latitude);	return &Temp_double_val;
		case GPS_SENSOR_TYPE_LONG:	Temp_double_val = minmea_tocoord_double(&frame_gga.longitude);	return &Temp_double_val;
		case GPS_SENSOR_TYPE_ALT:	Temp_float_val = minmea_tofloat(&frame_gga.altitude);			return &Temp_float_val;
		case GPS_SENSOR_TYPE_SPEED:	Temp_float_val = minmea_tofloat(&frame_vtg.speed_kph);			return &Temp_float_val;
		default:	return SENSOR_ERROR;
	}


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
static int
configure(int type, int enable)
{
	uint8_t ID;
	switch(type) {
		case SENSORS_HW_INIT:
			enabled = SENSOR_STATUS_INITIALISED;

			break;
		case SENSORS_ACTIVE:
			// Must be initialised first
			if(enabled == SENSOR_STATUS_DISABLED) {
			  return SENSOR_STATUS_DISABLED;
			}
			if(enable) {
				ctimer_set(&startup_timer, SENSOR_STARTUP_DELAY, notify_ready, NULL);
			  	enabled = SENSOR_STATUS_NOT_READY;
			} else {
			  enabled = SENSOR_STATUS_INITIALISED;
			}
			break;
		default:
			break;
	}

  return enabled;
}
/*---------------------------------------------------------------------------*/
/**
 * \brief Returns the status of the sensor
 * \param type SENSORS_ACTIVE or SENSORS_READY
 * \return 1 if the sensor is enabled
 */
static int
status(int type)
{
	switch(type) {
		case SENSORS_ACTIVE:
		case SENSORS_READY:
			return enabled;
			break;
		default:
			break;
	}
	return SENSOR_STATUS_DISABLED;
}
/*---------------------------------------------------------------------------*/
SENSORS_SENSOR(GPS_sensor, "GPS", value, configure, status);
/*---------------------------------------------------------------------------*/
/** @} */
