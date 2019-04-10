/*
 * Copyright (c) 2017, Proces-data A/S, http://www.proces-data.com/.
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
#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include "contiki.h"
#include "lib/sensors.h"
#include "button-sensor.h"
#include "sys/timer.h"
#include "gpio.h"
#include "pio_handler.h"
#include "board-peripherals.h"

#define BUTTON_PIN            PIO_PB2
Pio *PIR_base = (Pio *)PIOB;

#define PIR_READ_PIN(x)	((PIR_base->PIO_PDSR & x)?1:0)
#define SENSOR_PIR_DELAY (1 * 1000) / CLOCK_SECOND//~1ms
static struct ctimer PIR_timer;

static int sensor_status = SENSOR_STATUS_DISABLED;

static void
notify_ready(void *not_used)
{
	sensor_status = SENSOR_STATUS_READY;
	sensors_changed(&PIR_sensor);
}

/*---------------------------------------------------------------------------*/
static int
PIR_sensor_value(int type)
{
	return PIR_READ_PIN(BUTTON_PIN);
}
/*---------------------------------------------------------------------------*/

static int
PIR_sensor_configure(int type, int enable)
{
	switch(type) {

		case SENSORS_HW_INIT:
			pio_set_input(PIOB,BUTTON_PIN,PIO_PULLUP);
			sensor_status = SENSOR_STATUS_INITIALISED;
			break;

		case SENSORS_ACTIVE:
			if(sensor_status == SENSOR_STATUS_DISABLED)
				return SENSOR_STATUS_DISABLED;

			 if(enable) {
				// Enable the sensor
				 SUPC->SUPC_WUIR = SUPC_WUIR_WKUPEN12_ENABLE | SUPC_WUIR_WKUPT12_HIGH;
				 ctimer_set(&PIR_timer, SENSOR_PIR_DELAY, notify_ready, NULL);
				sensor_status = SENSOR_STATUS_READY;
			 } else {
				 // Disable the sensor
				 SUPC->SUPC_WUIR = SUPC_WUIR_WKUPEN12_DISABLE;
				 sensor_status = SENSOR_STATUS_INITIALISED;
			 }
			 break;
	}

	return sensor_status;
}

/*---------------------------------------------------------------------------*/
static int
PIR_sensor_status(int type)
{
	if(type == STATUS_STATE)
		return sensor_status;


	return SENSOR_ERROR;
}
/*---------------------------------------------------------------------------*/
SENSORS_SENSOR(PIR_sensor, "PIR sensor",PIR_sensor_value, PIR_sensor_configure, PIR_sensor_status);
/*---------------------------------------------------------------------------*/
