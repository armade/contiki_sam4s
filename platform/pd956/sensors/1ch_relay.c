/*
 * Copyright � 2019, Peter Mikkelsen
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
#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include "contiki.h"
#include "lib/sensors.h"
#include "1ch_relay.h"
#include "sys/timer.h"
#include "gpio.h"
#include "pio_handler.h"
#include "board-peripherals.h"

#ifdef NODE_1_ch_relay

#define ACTIVE_HIGH		1
#define ACTIVE_LOW		2
 
#define MODE 			ACTIVE_HIGH

/*---------------------------------------------------------------------------*/

#define CH1_PIN            PIO_PA6

unsigned pin_array[] = {
	CH1_PIN,
};


static int sensor_status = SENSOR_STATUS_DISABLED;

#define SENSOR_SWITCH_DELAY (10 * 1000) / CLOCK_SECOND//~10ms
static struct ctimer switch_timer;

static void
notify_ready(void *not_used)
{
	sensor_status = SENSOR_STATUS_READY;
	sensors_changed(&ch1_relay_PD956);
}

/*---------------------------------------------------------------------------*/
static int
relay_value(int type)
{
	// Status
	if((type >= ch1_STATUS_MIN) && (type <= ch1_STATUS_MAX))
#if (MODE == ACTIVE_LOW)
		return (PIOA->PIO_ODSR & pin_array[type-10])?0:1; // Active low
#else
		return (PIOA->PIO_ODSR & pin_array[type-10])?1:0; // Active high
#endif
	// Input validate for set
	if((type > ch1_RELAY_MAX) || (type < ch1_RELAY_MIN))
		return SENSOR_ERROR;

	// set values
	if(type&1){
#if (MODE == ACTIVE_LOW)
		PIOA->PIO_SODR = pin_array[(type-15)>>1]; //RELAY_OFF
#else
		PIOA->PIO_CODR = pin_array[(type-15)>>1]; //RELAY_OFF
#endif
	}else{
#if (MODE == ACTIVE_LOW)
		PIOA->PIO_CODR = pin_array[(type-14)>>1]; //RELAY_ON
#else
		PIOA->PIO_SODR = pin_array[(type-14)>>1]; //RELAY_ON
#endif
}
	sensor_status = SENSOR_STATUS_NOT_READY;
	ctimer_set(&switch_timer, SENSOR_SWITCH_DELAY, notify_ready, NULL);
	return 0; // Active low
}
/*---------------------------------------------------------------------------*/

static int
relay_configure(int type, int enable)
{
	switch(type) {

		case SENSORS_HW_INIT:
			//pio_set_input(PIOA, PIO_PA3 | PIO_PA28, 0); // NB: pa3 and pa28 is connected to pa9 and pa10.
#if (MODE == ACTIVE_LOW)
			pio_set_output(PIOA, CH1_PIN,	1, 0, 0);
#else
			pio_set_output(PIOA, CH1_PIN,	0, 0, 0);
#endif
			sensor_status = SENSOR_STATUS_INITIALISED;
			break;

		case SENSORS_ACTIVE:
			if(sensor_status == SENSOR_STATUS_DISABLED)
				return SENSOR_STATUS_DISABLED;

			if(enable) {// Enable the sensor
				sensor_status = SENSOR_STATUS_READY;
				ctimer_set(&switch_timer, SENSOR_SWITCH_DELAY, notify_ready, NULL);
			} else {// Disable the sensor
				sensor_status = SENSOR_STATUS_INITIALISED;
			}
			break;
	}

	return sensor_status;
}
/*---------------------------------------------------------------------------*/
static int
relay_status(int type)
{
	return sensor_status;
}
/*---------------------------------------------------------------------------*/
SENSORS_SENSOR(ch1_relay_PD956, "1 Ch relay",relay_value, relay_configure, relay_status);
/*---------------------------------------------------------------------------*/
#endif
