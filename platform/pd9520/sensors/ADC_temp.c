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
#include "sys/ctimer.h"
#include "ADC_temp.h"
#include "drivers/pmc.h"
#include "drivers/afec.h"
#include "platform-conf.h"
#include "board-peripherals.h"

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
/*---------------------------------------------------------------------------*/


static int sensor_status = SENSOR_STATUS_DISABLED;

#define ADC_DONE (adc_get_status(ADC) & (1<<ADC_TEMPERATURE_SENSOR))
int ADC_temperature;;

#define SENSOR_STARTUP_DELAY 50 //~50ms
static struct ctimer startup_timer;
/*---------------------------------------------------------------------------*/

static void
notify_ready(void *not_used)
{
	int ret = 0;
	float temp = 0;

	if(afec_channel_get_status(AFEC0,AFEC_TEMPERATURE_SENSOR))
		ret = afec_channel_get_value(AFEC0, AFEC_TEMPERATURE_SENSOR);

	if(ret == 0){
		ADC_temperature = SENSOR_ERROR;
		sensors_changed(&SAM4S_ADC_TS_sensor);
		return;
	}

	//PRINTF("val: 0x%x\n",ret);

	temp = (ret * 3300.0) / 4096.0;
	//PRINTF("voltage: %f [mV]\n",temp);

	ADC_temperature =  (float)(temp - 720) * 212 + 27000;

	//PRINTF("temperature: %d [mï¿½C]\n",ADC_temperature);
	sensor_status = SENSOR_STATUS_READY;
	sensors_changed(&SAM4S_ADC_TS_sensor);
}
/**
 * \brief Returns a reading from the sensor
 * \return Temperature (°C * 1000).
 */
static int
SAM4S_ADC_value(int type)
{
	if(sensor_status != SENSOR_STATUS_READY) {
		PRINTF("Sensor disabled or starting up (%d)\n", sensor_status);
		return SENSOR_ERROR;
	}

	if(type != ADC_TS_SENSOR_TYPE_TEMP) {
		PRINTF("Invalid type\n");
		return SENSOR_ERROR;
	}

	return ADC_temperature;
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
SAM4S_ADC_configure(int type, int enable)
{
	switch(type) {

	case SENSORS_HW_INIT:
		// Enable peripheral clock.
		afec_enable(AFEC0);
		// Initialize ADC.
		struct afec_config afec_cfg;

			afec_get_config_defaults(&afec_cfg);

			afec_init(AFEC0, &afec_cfg);

			afec_set_trigger(AFEC0, AFEC_TRIG_SW);

			struct afec_ch_config afec_ch_cfg;
			afec_ch_get_config_defaults(&afec_ch_cfg);
			afec_ch_cfg.gain = AFEC_GAINVALUE_0;
			afec_ch_set_config(AFEC0, AFEC_TEMPERATURE_SENSOR, &afec_ch_cfg);

			/*
			 * Because the internal ADC offset is 0x200, it should cancel it and shift
			 * down to 0.
			 */
			afec_channel_set_analog_offset(AFEC0, AFEC_TEMPERATURE_SENSOR, 0x200);

			struct afec_temp_sensor_config afec_temp_sensor_cfg;

				afec_temp_sensor_get_config_defaults(&afec_temp_sensor_cfg);
				afec_temp_sensor_cfg.rctc = true;
				afec_temp_sensor_set_config(AFEC0, &afec_temp_sensor_cfg);


				afec_set_power_mode(AFEC0,AFEC_POWER_MODE_2);


		// Enable channel(s).
		sensor_status = SENSOR_STATUS_INITIALISED;
		break;

	case SENSORS_ACTIVE:
	     if(enable) {
	    	 afec_channel_enable(AFEC0,AFEC_TEMPERATURE_SENSOR);
	    	 ctimer_set(&startup_timer, SENSOR_STARTUP_DELAY, notify_ready, NULL);
	    	 afec_start_software_conversion(AFEC0);
	    	 sensor_status = SENSOR_STATUS_NOT_READY;
	     } else {
	    	 afec_channel_disable(AFEC0,AFEC_TEMPERATURE_SENSOR);
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
SAM4S_ADC_status(int type)
{
	return sensor_status;
}
/*---------------------------------------------------------------------------*/
SENSORS_SENSOR(SAM4S_ADC_TS_sensor, "ADC TS", SAM4S_ADC_value, SAM4S_ADC_configure, SAM4S_ADC_status);
/*---------------------------------------------------------------------------*/
