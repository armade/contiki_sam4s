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
#include "PIR_SR501_sensor.h"
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
	sensors_changed(&PIR_SR501_sensor);
}

volatile uint8_t IRQ_type;
#define falling_egde 	2
#define rising_egde 	1

clock_time_t falling_timestamp;
clock_time_t rising_timestamp;

/*---------------------------------------------------------------------------*/
static void
PIR_detection_callback(uint32_t a, uint32_t b)
{
	sensors_changed(&PIR_SR501_sensor);

	// Change interrupt condition so we get interrupt on both falling and rising edge.
	// We have 300ms bounce time so this should be safe
	if ((IRQ_type == falling_egde) && (PIR_READ_PIN(BUTTON_PIN)==0)) {
		// Rising Edge
		PIR_base->PIO_REHLSR = BUTTON_PIN;
		falling_timestamp = (clock_time()*1000)/CLOCK_SECOND; //ms
		IRQ_type = rising_egde;
	} else if((IRQ_type == rising_egde) && (PIR_READ_PIN(BUTTON_PIN)==1)){
		// Falling Edge
		PIR_base->PIO_FELLSR = BUTTON_PIN;
		rising_timestamp = (clock_time()*1000)/CLOCK_SECOND; //ms
		IRQ_type = falling_egde;
	}
}

/*---------------------------------------------------------------------------*/
static int
PIR_SR501_sensor_value(int type)
{
	return PIR_READ_PIN(BUTTON_PIN);
}
/*---------------------------------------------------------------------------*/

static int
PIR_SR501_sensor_configure(int type, int enable)
{
	switch(type) {

		case SENSORS_HW_INIT:
			pio_set_input(PIOB,BUTTON_PIN,PIO_PULLUP);

			pio_handler_set(PIOB, ID_PIOB, BUTTON_PIN, PIO_IT_RISE_EDGE, PIR_detection_callback);
						IRQ_type = rising_egde;
						NVIC_EnableIRQ((IRQn_Type)ID_PIOB);

			sensor_status = SENSOR_STATUS_INITIALISED;
			break;

		case SENSORS_ACTIVE:
			if(sensor_status == SENSOR_STATUS_DISABLED)
				return SENSOR_STATUS_DISABLED;

			 if(enable) {
				// Enable the sensor
				 //SUPC->SUPC_WUIR = SUPC_WUIR_WKUPEN12_ENABLE | SUPC_WUIR_WKUPT12_HIGH;
				 //ctimer_set(&PIR_timer, SENSOR_PIR_DELAY, notify_ready, NULL);

				 pio_enable_interrupt(PIOB, BUTTON_PIN);
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
PIR_SR501_sensor_status(int type)
{
	if(type == STATUS_STATE)
		return sensor_status;


	return SENSOR_ERROR;
}
/*---------------------------------------------------------------------------*/
SENSORS_SENSOR(PIR_SR501_sensor, "PIR SR501 sensor",PIR_SR501_sensor_value, PIR_SR501_sensor_configure, PIR_SR501_sensor_status);
/*---------------------------------------------------------------------------*/
