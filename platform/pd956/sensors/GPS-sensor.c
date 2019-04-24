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
#include "GPS-sensor.h"
#include "sys/ctimer.h"
#include "lib/sensors.h"
#include "apps/gpsd/minmea.h"
#include "board-peripherals.h"

#include <stdint.h>
#include <string.h>
#include <stdio.h>
#include <math.h>
/*---------------------------------------------------------------------------*/
#define DEBUG 0
#if DEBUG
#define PRINTF(...) printf(__VA_ARGS__)
#else
#define PRINTF(...)
#endif

#define SENSOR_STARTUP_DELAY (1 * 1000) / CLOCK_SECOND
static struct ctimer startup_timer;
static float Temp_float_val;
/*---------------------------------------------------------------------------*/
static int enabled = SENSOR_STATUS_DISABLED;
/*---------------------------------------------------------------------------*/

extern struct minmea_sentence_rmc frame_rmc;
extern struct minmea_sentence_gga frame_gga;
extern struct minmea_sentence_gst frame_gst;
extern struct minmea_sentence_gsv frame_gsv;
extern struct minmea_sentence_vtg frame_vtg;
extern struct minmea_sentence_zda frame_zda;

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
gps_value(int type)
{

	if(enabled != SENSOR_STATUS_READY) {
		PRINTF("Sensor disabled or starting up (%d)\n", enabled);
		return SENSOR_ERROR;
	}

	// We are not going to perform any calculations on any of the parameter.
	// We should just parse on the raw value. This is only for testing.
	switch(type){
		case GPS_SENSOR_TYPE_LAT:
			if(frame_gga.latitude.scale == 0)
				return SENSOR_ERROR;
			Temp_float_val = minmea_tocoord(&frame_gga.latitude);
			return (int)&Temp_float_val;
		case GPS_SENSOR_TYPE_LONG:
			if(frame_gga.longitude.scale == 0)
				return SENSOR_ERROR;
			Temp_float_val = minmea_tocoord(&frame_gga.longitude);
			return (int)&Temp_float_val;
		case GPS_SENSOR_TYPE_ALT:
			if(frame_gga.altitude.scale == 0)
				return SENSOR_ERROR;
			Temp_float_val = minmea_tofloat(&frame_gga.altitude);
			return (int)&Temp_float_val;
		case GPS_SENSOR_TYPE_SPEED:
			if(frame_vtg.speed_kph.scale == 0)
				return SENSOR_ERROR;
			Temp_float_val = minmea_tofloat(&frame_vtg.speed_kph);
			return (int)&Temp_float_val;
		default:	return SENSOR_ERROR;
	}

	return SENSOR_ERROR;
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
gps_configure(int type, int enable)
{
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
gps_status(int type)
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
SENSORS_SENSOR(GPS_sensor, "GPS", gps_value, gps_configure, gps_status);
/*---------------------------------------------------------------------------*/
/** @} */
