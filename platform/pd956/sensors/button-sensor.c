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

#define BUTTON_PIN            PIO_PA6
Pio *button_base = (Pio *)PIOA;

#define BUTTON_READ_PIN(x)	((button_base->PIO_PDSR & x)?1:0)

volatile uint8_t IRQ_type;
#define falling_egde 	2
#define rising_egde 	1


static int sensor_status = SENSOR_STATUS_DISABLED;

clock_time_t falling_timestamp;
clock_time_t rising_timestamp;

/*---------------------------------------------------------------------------*/
static void
button_detection_callback(uint32_t a, uint32_t b)
{
	sensors_changed(&button_sensor);

	// Change interrupt condition so we get interrupt on both falling and rising edge.
	// We have 300ms bounce time so this should be safe
	if ((IRQ_type == falling_egde) && (BUTTON_READ_PIN(BUTTON_PIN)==0)) {
		// Rising Edge
		button_base->PIO_REHLSR = BUTTON_PIN;
		falling_timestamp = (clock_time()*1000)/CLOCK_SECOND; //ms
		IRQ_type = rising_egde;
	} else if((IRQ_type == rising_egde) && (BUTTON_READ_PIN(BUTTON_PIN)==1)){
		// Falling Edge
		button_base->PIO_FELLSR = BUTTON_PIN;
		rising_timestamp = (clock_time()*1000)/CLOCK_SECOND; //ms
		IRQ_type = falling_egde;
	}
}
/*---------------------------------------------------------------------------*/
static int
button_sensor_value(int type)
{
	return BUTTON_READ_PIN(BUTTON_PIN);
}
/*---------------------------------------------------------------------------*/

static int
button_sensor_configure(int type, int enable)
{
	switch(type) {

		case SENSORS_HW_INIT:
			pio_set_input(PIOA,BUTTON_PIN,PIO_PULLUP);
			pio_set_debounce_filter(PIOA, BUTTON_PIN, 5000); // aprox. 300ms
			pio_handler_set(PIOA, ID_PIOA, BUTTON_PIN, PIO_IT_FALL_EDGE, button_detection_callback);
			IRQ_type = falling_egde;
			NVIC_EnableIRQ((IRQn_Type)ID_PIOA);

			sensor_status = SENSOR_STATUS_INITIALISED;
			break;

		case SENSORS_ACTIVE:
			if(sensor_status == SENSOR_STATUS_DISABLED)
				return SENSOR_STATUS_DISABLED;

			 if(enable) {
				// Enable the sensor
				pio_enable_interrupt(PIOA, BUTTON_PIN);
				sensor_status = SENSOR_STATUS_READY;
			 } else {
				 // Disable the sensor
				 pio_disable_interrupt(PIOA, BUTTON_PIN);
				 sensor_status = SENSOR_STATUS_INITIALISED;
			 }
			 break;
	}

	return sensor_status;
}

/*---------------------------------------------------------------------------*/
static int
button_sensor_status(int type)
{
	if(type == STATUS_STATE)
		return sensor_status;

	// Returns how long the button was pressed
	//NB: returns negative number if still active
	if(type == STATUS_TIME)
		return rising_timestamp-falling_timestamp;//ms

	// Returns timestamp for activation.
	// Can be used to give feedback on how long
	// time the button is current active. clock_now() - activation time.
	if(type == STATUS_ACTIVATION_TIME)
		return falling_timestamp;//ms

	return SENSOR_ERROR;
}
/*---------------------------------------------------------------------------*/
SENSORS_SENSOR(button_sensor, "Button sensor",button_sensor_value, button_sensor_configure, button_sensor_status);
/*---------------------------------------------------------------------------*/
