
#include "contiki-conf.h"
#include "lib/sensors.h"
#include "sys/ctimer.h"
#include "ADC_temp.h"
#include "pmc.h"
#include "adc.h"
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
static int ADC_temperature;;

#define SENSOR_STARTUP_DELAY (50 * 1000) / CLOCK_SECOND //~50ms
static struct ctimer startup_timer;
/*---------------------------------------------------------------------------*/

static void
notify_ready(void *not_used)
{
	int32_t ret = 0;
	int32_t temp = 0;

	if((adc_get_status(ADC) & (1<<ADC_TEMPERATURE_SENSOR)))
		ret = adc_get_channel_value(ADC, ADC_TEMPERATURE_SENSOR);

	if(ret == 0){
		ADC_temperature = SENSOR_ERROR;
		sensors_changed(&SAM4S_ADC_TS_sensor);
		return;
	}

	//PRINTF("val: 0x%x\n",ret);

	//temp = (ret * 3300.0) / 4096.0;
	//PRINTF("voltage: %f [mV]\n",temp);
	//ADC_temperature =  (float)(temp - 1440) * 212 + 27000;

	temp = (ret*3300)>>12;
	ADC_temperature = (temp - 1440) * 212 + 27000;

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
		pmc_enable_periph_clk(ID_ADC);
		// Initialize ADC.
		adc_init(ADC, F_CPU, 20000000, ADC_MR_STARTUP_SUT832);
		adc_configure_timing(ADC, 1, ADC_SETTLING_TIME_3, 1);
		//adc_check(ADC, sysclk_get_cpu_hz());
		adc_configure_power_save(ADC, 1, 0);

		// Software trigger.
		adc_configure_trigger(ADC, ADC_TRIG_SW, 0);
		// Enable channel(s).
		adc_enable_channel(ADC, ADC_TEMPERATURE_SENSOR);
		sensor_status = SENSOR_STATUS_INITIALISED;
		break;

	case SENSORS_ACTIVE:
	     if(enable) {
	    	 adc_enable_ts(ADC);
	    	 ctimer_set(&startup_timer, SENSOR_STARTUP_DELAY, notify_ready, NULL);
	    	 adc_start(ADC);
	    	 sensor_status = SENSOR_STATUS_NOT_READY;
	     } else {
	    	 adc_disable_ts(ADC);
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
