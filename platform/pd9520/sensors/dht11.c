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
#include "platform-conf.h"
#include "lib/sensors.h"
#include "board-peripherals.h"

#include "compiler.h"
#include <gpio.h>
#include "pio_handler.h"
#include "dht11.h"

/*---------------------------------------------------------------------------*/
#define DEBUG 0
#if DEBUG
#define PRINTF(...) printf(__VA_ARGS__)
#else
#define PRINTF(...)
#endif
/*---------------------------------------------------------------------------*/
#define data_pin PIO_PB2

static int sensor_status = SENSOR_STATUS_DISABLED;

static int dht_state;
static uint64_t data_bytes;
volatile int timer,old_timer,new_timer;
volatile char bitcounter;

int temperature = SENSOR_ERROR, humidity = SENSOR_ERROR, parity;
int temperature_split = SENSOR_ERROR, humidity_split = SENSOR_ERROR;

#define BUSYWAIT_UNTIL(cond, max_time)                                  \
  do {                                                                  \
    static rtimer_clock_t t0;                                           \
    t0 = RTIMER_NOW();                                                  \
    while(!(cond) && RTIMER_CLOCK_LT(RTIMER_NOW(), t0 + (max_time)));   \
  } while(0)
/*---------------------------------------------------------------------------*/
struct ctimer dht_timeout_timer;
volatile uint8_t par;
static void
dht11_timeout(void *data)
{
	if(dht_state == 3)
	{
		parity = (data_bytes>>(40-8))&0xff;
		parity += (data_bytes>>(40-16))&0xff;
		parity += (data_bytes>>(40-24))&0xff;
		parity += (data_bytes>>(40-32))&0xff;

		par = (data_bytes&0xff);

		if((parity&0xff) == (data_bytes&0xff))
		{
			humidity =  (data_bytes>>(40-8))&0xff;
			humidity *= 1000;
			humidity += (data_bytes>>(40-16))&0xff;

			temperature =  (data_bytes>>(40-24))&0xff;
			temperature *= 1000;
			temperature += (data_bytes>>(40-32))&0xff;

			humidity_split =(data_bytes>>(40-16))&0xffff;
			temperature_split = (data_bytes>>(40-32))&0xffff;
		}
	}
	else
	{
		pio_disable_interrupt(PIOB, data_pin);
		dht_state = 77;
		temperature = SENSOR_ERROR;
		humidity = SENSOR_ERROR;
		humidity_split = SENSOR_ERROR;
		temperature_split = SENSOR_ERROR;
	}

	sensor_status = SENSOR_STATUS_READY;
	sensors_changed(&dht11_sensor);
}

#ifdef DEBUG_TIME
volatile int debug_time[42];
#define LOG_TIME(index,time) debug_time[index] = time
#else
#define LOG_TIME(index,time)
#endif
/*
 * Sample on falling edge. depending on the time between samples
 * we can determine if the bit is '1' or '0'.
 */
void dht11_data_pin_irq(uint32_t a, uint32_t b)
{
	new_timer = SysTick->VAL;
	timer = old_timer - new_timer; // systimer is counting down
	if(timer < 0) // if tickcounter ticks :)
		timer += SysTick->LOAD;
	timer = (timer*1000000)/F_CPU;

	switch(dht_state)
	{
		// Delay DHT 20-40us
		case 0:
			LOG_TIME(0, timer);
			bitcounter=0;
			dht_state++;
			break;

		// Start 80us*2
		case 1:
			LOG_TIME(1, timer);
			dht_state++;
			break;
		// Data
		case 2:
			data_bytes <<=1;
			LOG_TIME(2+bitcounter, timer);
			bitcounter++;
			if((timer < 130) && (timer > 110)) 		// 120us = '1'
				data_bytes |= 1;
			else if((timer < 90) && (timer > 65)) 	// 76-78us = '0'
				data_bytes &= ~(1ULL);
			else									//got to error if it's not a '0' or a '1'.
				dht_state = 77;

			if(bitcounter==40)
			{
				// After max 1 sec from start we timeout and hit 'dht11_timeout()'.
				// If we have completed and no errors detected dht_state is 3.
				// if state is 2 then we have hit a timeout.
				// If state is 77 we have a data error.
				dht_state++;
				pio_disable_interrupt(PIOB, data_pin);
				ctimer_stop(&dht_timeout_timer);
				dht11_timeout(NULL);
			}

			break;

		default:
			break;
	}
	old_timer = new_timer;
}
/*---------------------------------------------------------------------------*/
/**
 * \brief Returns a reading from the sensor
 * \ Temperature = temp *1000
 * \ Humidity = hud * 1000
 * \ Temperature_split = temp<<8 | temp_dec
 * \ Humidity_split = hud<<8 | hud_dec
 */
static int
dht11_value(int type)
{
	if(type == TEMPERATURE_READING) // temperature
		return temperature;

	if(type == HUMIDITY_READING)
		return humidity;
// Split reading represent the same as in the transfer.
	// 8 bit for integer and 8 bit for decimal
	if(type==TEMPERATURE_READING_SPLIT)
		return temperature_split;

	if(type==HUMIDITY_READING_SPLIT)
		return humidity_split;

	return SENSOR_ERROR;
}

struct ctimer dht11_timer;

// TODO: pio supports multidrive. use it.
void
dht11_start_measurement(void *ptr)
{
	dht_state = 0;
	data_bytes = 0;
	pio_set_output(PIOB, data_pin,0,0,1);
	BUSYWAIT_UNTIL(0,(18 * RTIMER_SECOND) / 1000);//18ms
	pio_set_pin_group_high(PIOB,data_pin);

	// Set timeout to 1 sec
	ctimer_set(&dht_timeout_timer, (1*CLOCK_SECOND), dht11_timeout, NULL);
	pio_set_input(PIOB,data_pin,PIO_PULLUP);
	pio_enable_interrupt(PIOB, data_pin);
	old_timer = SysTick->VAL;

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
dht11_configure(int type, int enable)
{
	switch(type) {

		case SENSORS_HW_INIT:
			pio_set_input(PIOB,data_pin,PIO_PULLUP);

			pio_handler_set(PIOB, ID_PIOB, data_pin, PIO_IT_FALL_EDGE, dht11_data_pin_irq);
			// We don't have an input capture at our disposal,
			// so we have to use high priority interrupt.
			NVIC_SetPriority((IRQn_Type) ID_PIOB, 0);//level 0 is the highest interrupt priority (0-15)
			NVIC_EnableIRQ((IRQn_Type)ID_PIOB);

			sensor_status = SENSOR_STATUS_INITIALISED;

			break;

		case SENSORS_ACTIVE:
			if(sensor_status == SENSOR_STATUS_DISABLED)
				return SENSOR_STATUS_DISABLED;

			 if(enable) {
				 sensor_status = SENSOR_STATUS_NOT_READY;
				 // Give the sensor some time to stabilize. 15 milliseconds should be sufficient.
				 ctimer_set(&dht11_timer,(15 * 1000) / CLOCK_SECOND, dht11_start_measurement, NULL);
			 } else {
				 ctimer_stop(&dht11_timer);
				 sensor_status = SENSOR_STATUS_INITIALISED;
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
static int
dht11_status(int type)
{
	return sensor_status;
}

/*---------------------------------------------------------------------------*/
SENSORS_SENSOR(dht11_sensor, "DHT11", dht11_value, dht11_configure, dht11_status);
/*---------------------------------------------------------------------------*/
