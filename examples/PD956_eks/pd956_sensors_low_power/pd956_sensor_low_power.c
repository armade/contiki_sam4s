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
/**
 * \addtogroup cc26xx-web-demo
 * @{
 *
 * \file
 *   Main module for the CC26XX web demo. Activates on-device resources,
 *   takes sensor readings periodically and caches them for all other modules
 *   to use.
 */
/*---------------------------------------------------------------------------*/
#include "contiki.h"
#include "contiki-net.h"
#include "board-peripherals.h"
#include "lib/sensors.h"
#include "lib/list.h"
#include "sys/process.h"
#include "net/ipv6/sicslowpan.h"
#include "httpd-simple.h"
#include "pd956_sensor_low_power.h"
#include "mqtt-client.h"
#include "ntpd.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "PD_FLASH/FLASH_driver.h"

/*---------------------------------------------------------------------------*/

PROCESS(PD956_MAIN_process, "PD956 MQTT");
/*---------------------------------------------------------------------------*/
/*
 * Update sensor readings in a staggered fashion every SENSOR_READING_PERIOD
 * ticks + a random interval between 0 and SENSOR_READING_RANDOM ticks
 */



/*---------------------------------------------------------------------------*/
process_event_t MQTT_publish_sensor_data_event;
process_event_t config_loaded_event;
process_event_t load_config_defaults;
process_event_t Trig_sensors;
/*---------------------------------------------------------------------------*/
/* Saved settings on flash: store, offset, magic */
#define CONFIG_FLASH_OFFSET        0
#define CONFIG_MAGIC      0x956a5007

Device_config_t web_demo_config;
/*---------------------------------------------------------------------------*/
/* A cache of sensor values. Updated periodically or upon key press */
LIST(sensor_list);
/*---------------------------------------------------------------------------*/
/* The objects representing sensors used in this demo */
#define DEMO_SENSOR(name, type, descr, xml_element, form_field, units, hass_component) \
		MQTT_sensor_reading_t name##_reading = \
  { NULL, 0, 0,0, descr, xml_element, form_field, units, type, 1, 1, hass_component}

/*sensors */
DEMO_SENSOR(temp, PD956_WEB_DEMO_SENSOR_ADC, "Internaltemperature",
		"Internaltemperature", "Internaltemperature", UNIT_TEMP,
		"sensor");

#ifdef NODE_STEP_MOTOR
DEMO_SENSOR(step_motor, PD956_WEB_DEMO_SENSOR_STEP, "Step_Position", "Step_position", "Step_position", UNIT_STEP,"sensor");
#endif

#ifdef NODE_LIGHT
DEMO_SENSOR(RGB_sensor, PD956_WEB_DEMO_SENSOR_RGB, "RGB_light", "RGB_light", "RGB_light", UNIT_NONE,"sensor");
#endif

#ifdef NODE_HARD_LIGHT
DEMO_SENSOR(hard_RGB_sensor, PD956_WEB_DEMO_SENSOR_RGB, "RGB_light", "RGB_light", "RGB_light", UNIT_NONE,"sensor");
#endif

#if defined(NODE_STEP_MOTOR) || defined(NODE_4_ch_relay)
DEMO_SENSOR(dht11_temperature, PD956_WEB_DEMO_SENSOR_DHT11_TEMP, "Temperature", "Temperature", "Temperature", UNIT_TEMP,"sensor");
DEMO_SENSOR(dht11_humidity, PD956_WEB_DEMO_SENSOR_DHT11_HUMIDITY, "Humidity", "Humidity", "Humidity", UNIT_HUMIDITY,"sensor");
#endif

#ifdef NODE_PRESSURE
DEMO_SENSOR(bmp_280_sensor_press,PD956_WEB_DEMO_SENSOR_BMP280_PRES, "Pressure", "Pressure", "Pressure", UNIT_PRES,"sensor");
DEMO_SENSOR(bmp_280_sensor_temp,PD956_WEB_DEMO_SENSOR_BMP280_TEMP, "Temperature", "Temperature", "Temperature", UNIT_TEMP,"sensor");
#endif

#ifdef NODE_HTU21D
DEMO_SENSOR(HTU21D_sensor_humid, PD956_WEB_DEMO_SENSOR_HTU21D_humid, "Humidity",
		"Humidity", "Humidity", UNIT_HUMIDITY, "sensor");
DEMO_SENSOR(HTU21D_sensor_temp, PD956_WEB_DEMO_SENSOR_HTU21D_TEMP,
		"Temperature", "Temperature", "Temperature", UNIT_TEMP,
		"sensor");
#endif

#ifdef NODE_GPS
	DEMO_SENSOR(GPS_sensor_LONG, PD956_WEB_DEMO_SENSOR_GPS_LONG, "Long", "Longitude", "Longitude", UNIT_ANGLE,"sensor");
	DEMO_SENSOR(GPS_sensor_LAT,  PD956_WEB_DEMO_SENSOR_GPS_LAT , "Lat", "Latitude", "Latitude", UNIT_ANGLE,"sensor");
	DEMO_SENSOR(GPS_sensor_ALT, PD956_WEB_DEMO_SENSOR_GPS_ALT, "Alt", "Altitude", "Altitude", UNIT_DISTANCE,"sensor");
	DEMO_SENSOR(GPS_sensor_SPEED, PD956_WEB_DEMO_SENSOR_GPS_SPEED, "spd", "speed", "speed", UNIT_SPEED,"sensor");
#endif
/*---------------------------------------------------------------------------*/
/*static void
 publish_led_off(void *d)
 {
 //leds_off(CC26XX_WEB_DEMO_STATUS_LED);
 }*/
/*---------------------------------------------------------------------------*/
static void save_config()
{

	int ret;
	MQTT_sensor_reading_t *reading = NULL;

	flash_leave_deep_sleep();

	ret = flash_erase_df(CONFIG_FLASH_OFFSET, erase_4k_block);

	if(ret){
		printf("Error erasing flash\n");
	} else{
		web_demo_config.magic = CONFIG_MAGIC;
		web_demo_config.len = sizeof(Device_config_t);
		web_demo_config.sensors_bitmap = 0;

		for (reading = list_head(sensor_list); reading != NULL ; reading =
				list_item_next(reading)){
			if(reading->publish){
				web_demo_config.sensors_bitmap |= (1 << reading->type);
			}
		}
		ret = flash_write_df(CONFIG_FLASH_OFFSET, (uint8_t *) &web_demo_config,
				sizeof(Device_config_t));

		if(ret){
			printf("Error saving config\n");
		}
	}
	flash_enter_deep_sleep();

}
/*---------------------------------------------------------------------------*/
static uint8_t load_config()
{

	int ret;
	/* Read from flash into a temp buffer */
	Device_config_t tmp_cfg;
	MQTT_sensor_reading_t *reading = NULL;

	flash_leave_deep_sleep();

	ret = flash_read_df((uint8_t *) &tmp_cfg, sizeof(tmp_cfg),
			CONFIG_FLASH_OFFSET);

	if(ret){
		printf("Error loading config\n");
		flash_enter_deep_sleep();
		return 1;
	}

	if(tmp_cfg.magic == CONFIG_MAGIC && tmp_cfg.len == sizeof(tmp_cfg)){
		memcpy(&web_demo_config, &tmp_cfg, sizeof(web_demo_config));

		for (reading = list_head(sensor_list); reading != NULL ; reading =
				list_item_next(reading)){
			if(web_demo_config.sensors_bitmap & (1 << reading->type)){
				reading->publish = 1;
			} else{
				reading->publish = 0;
				snprintf(reading->converted, SENSOR_CONVERTED_LEN,
						"\"N/A\"");
			}
		}
	} else{
		printf("Error bad header in config\n");
		flash_enter_deep_sleep();
		return 1;
	}

	flash_enter_deep_sleep();
	return 0;

}
/*---------------------------------------------------------------------------*/
/* Don't start everything here, we need to dictate order of initialisation */
AUTOSTART_PROCESSES(&PD956_MAIN_process);
/*---------------------------------------------------------------------------*/
int ipaddr_sprintf(char *buf, uint8_t buf_len,
		const uip_ipaddr_t *addr)
{
	uint16_t a;
	uint8_t len = 0;
	int i, f;
	for (i = 0, f = 0; i < sizeof(uip_ipaddr_t) ; i += 2){
		a = (addr->u8[i] << 8) + addr->u8[i + 1];
		if(a == 0 && f >= 0){
			if(f++ == 0){
				len += snprintf(&buf[len], buf_len - len, "::");
			}
		} else{
			if(f > 0){
				f = -1;
			} else if(i > 0){
				len += snprintf(&buf[len], buf_len - len, ":");
			}
			len += snprintf(&buf[len], buf_len - len, "%x", a);
		}
	}

	return len;
}

/*---------------------------------------------------------------------------*/
MQTT_sensor_reading_t *
MQTT_sensor_first()
{
	return list_head(sensor_list);
}
/*---------------------------------------------------------------------------*/
void Restore_defaults(void)
{
	MQTT_sensor_reading_t *reading = NULL;

	for (reading = list_head(sensor_list); reading != NULL ; reading =
			list_item_next(reading)){
		reading->publish = 1;
	}

	process_post_synch(&mqtt_client_process,
			load_config_defaults, NULL);

	save_config();

}

/*---------------------------------------------------------------------------*/

static void get_temp_reading(void)
{
	int value;
	int low, high;
	char *buf;

	if(temp_reading.publish){
		value = SAM4S_ADC_TS_sensor.value(ADC_TS_SENSOR_TYPE_TEMP);
		if(value != SENSOR_ERROR){
			temp_reading.raw = value;

			buf = temp_reading.converted;
			memset(buf, 0, SENSOR_CONVERTED_LEN);
			high = value / 1000;
			low = value - high * 1000;
			snprintf(buf, SENSOR_CONVERTED_LEN, "%d.%d", high, low);
		} else{
			snprintf(buf, SENSOR_CONVERTED_LEN, "\"N/A\"");
		}
	}

	SENSORS_DEACTIVATE(SAM4S_ADC_TS_sensor);
}

#ifdef NODE_LIGHT
static void
get_RGB_reading(void)
{
	int value;
	char *buf;
	RGB_t rgb;

	if(RGB_sensor_reading.publish){

		value = RGB_sensor.value(SENSOR_ERROR);
		if(value != SENSOR_ERROR){
			RGB_sensor_reading.raw = value;
			rgb.all = value;

			buf = RGB_sensor_reading.converted;
			memset(buf, 0, SENSOR_CONVERTED_LEN);

			snprintf(buf, SENSOR_CONVERTED_LEN, "%d,%d,%d",rgb.led.r,rgb.led.g,rgb.led.b);
		}
		else
		{
			snprintf(buf, SENSOR_CONVERTED_LEN, "\"N/A\"");
		}
	}
}
#endif

#ifdef NODE_STEP_MOTOR
static void
get_step_reading(void)
{
	int value;
	char *buf;

	if(step_motor_reading.publish){

		value = step_sensor.value(SENSOR_ERROR);
		if(value != SENSOR_ERROR){
			step_motor_reading.raw = value;

			buf = step_motor_reading.converted;
			memset(buf, 0, SENSOR_CONVERTED_LEN);

			snprintf(buf, SENSOR_CONVERTED_LEN, "%d",value);
		}
		else
		{
			snprintf(buf, SENSOR_CONVERTED_LEN, "\"N/A\"");
		}
	}
}
#endif

#if defined(NODE_STEP_MOTOR) || defined(NODE_4_ch_relay)
static void
get_dht11_temperature_reading(void)
{
	int value;
	char *buf;

	if(dht11_temperature_reading.publish){

		value = dht11_sensor.value(TEMPERATURE_READING_SPLIT);
		buf = dht11_temperature_reading.converted;
		if(value != SENSOR_ERROR){
			dht11_temperature_reading.raw = value;

			memset(buf, 0, SENSOR_CONVERTED_LEN);

			snprintf(buf, SENSOR_CONVERTED_LEN, "%d.%d",(value>>8),value&0xff);
		}
		else
		{
			snprintf(buf, SENSOR_CONVERTED_LEN, "");
		}
	}

	if(dht11_humidity_reading.publish){

		value = dht11_sensor.value(HUMIDITY_READING_SPLIT);
		buf = dht11_humidity_reading.converted;
		if(value != SENSOR_ERROR){
			dht11_humidity_reading.raw = value;

			memset(buf, 0, SENSOR_CONVERTED_LEN);

			snprintf(buf, SENSOR_CONVERTED_LEN, "%d.%d",(value>>8),value&0xff);
		}
		else
		{
			snprintf(buf, SENSOR_CONVERTED_LEN, "\"N/A\"");
		}
	}
	SENSORS_DEACTIVATE(dht11_sensor);
}
#endif
/*---------------------------------------------------------------------------*/
#ifdef NODE_PRESSURE

static void
get_bmp_reading()
{
	int value;
	int low,high;
	char *buf;

	if(bmp_280_sensor_press_reading.publish){

		value = bmp_280_sensor.value(BMP_280_SENSOR_TYPE_PRESS);
		buf = bmp_280_sensor_press_reading.converted;
		if(value != SENSOR_ERROR){
			bmp_280_sensor_press_reading.raw = value;

			memset(buf, 0, SENSOR_CONVERTED_LEN);
			high = value/100;
			low = value-high*100;
			snprintf(buf, SENSOR_CONVERTED_LEN, "%d.%d", high,low);
		}
		else
		{
			snprintf(buf, SENSOR_CONVERTED_LEN, "\"N/A\"");
		}
	}

	if(bmp_280_sensor_temp_reading.publish){

		value = bmp_280_sensor.value(BMP_280_SENSOR_TYPE_TEMP);
		buf = bmp_280_sensor_temp_reading.converted;
		if(value != SENSOR_ERROR){
			bmp_280_sensor_temp_reading.raw = value;

			memset(buf, 0, SENSOR_CONVERTED_LEN);
			high = value/100;
			low = value-high*100;
			snprintf(buf, SENSOR_CONVERTED_LEN, "%d.%d", high,low);
		}
		else
		{
			snprintf(buf, SENSOR_CONVERTED_LEN, "\"N/A\"");
		}
	}

	SENSORS_DEACTIVATE(bmp_280_sensor);

}
#endif

#ifdef NODE_HTU21D

static void get_HTU21D_reading()
{
	int value;
	int low, high;
	char *buf;

	if(HTU21D_sensor_humid_reading.publish){

		value = HTU21D_sensor.value(HTU21D_SENSOR_TYPE_HUMID);
		buf = HTU21D_sensor_humid_reading.converted;
		if(value != SENSOR_ERROR){
			HTU21D_sensor_humid_reading.raw = value;

			memset(buf, 0, SENSOR_CONVERTED_LEN);
			high = value / 1000;
			low = value - high * 1000;
			snprintf(buf, SENSOR_CONVERTED_LEN, "%d.%d", high, low);
		} else{
			snprintf(buf, SENSOR_CONVERTED_LEN, "\"N/A\"");
		}
	}

	if(HTU21D_sensor_temp_reading.publish){

		value = HTU21D_sensor.value(HTU21D_SENSOR_TYPE_TEMP);
		buf = HTU21D_sensor_temp_reading.converted;
		if(value != SENSOR_ERROR){
			HTU21D_sensor_temp_reading.raw = value;

			memset(buf, 0, SENSOR_CONVERTED_LEN);
			high = value / 1000;
			low = value - high * 1000;
			snprintf(buf, SENSOR_CONVERTED_LEN, "%d.%d", high, low);
		} else{
			snprintf(buf, SENSOR_CONVERTED_LEN, "\"N/A\"");
		}
	}

	SENSORS_DEACTIVATE(HTU21D_sensor);

}
#endif

#ifdef NODE_GPS

static void get_GPS_reading()
{
	char *buf;
	float value;
	double value_d;
	int ret;

	if(GPS_sensor_LONG_reading.publish){
		ret = GPS_sensor.value(GPS_SENSOR_TYPE_LONG);
		buf = GPS_sensor_LONG_reading.converted;
		value_d = *(double *)ret;
		if(ret != SENSOR_ERROR){
			GPS_sensor_LONG_reading.raw_f = value;

			memset(buf, 0, SENSOR_CONVERTED_LEN);
			snprintf(buf, SENSOR_CONVERTED_LEN, "%f", value_d);
		} else{
			snprintf(buf, SENSOR_CONVERTED_LEN, "\"N/A\"");
		}
	}

	if(GPS_sensor_LAT_reading.publish){
		ret = GPS_sensor.value(GPS_SENSOR_TYPE_LAT);
		buf = GPS_sensor_LAT_reading.converted;
		value_d = *(double *)ret;
		if(ret != SENSOR_ERROR){
			GPS_sensor_LONG_reading.raw_f = value;

			memset(buf, 0, SENSOR_CONVERTED_LEN);
			snprintf(buf, SENSOR_CONVERTED_LEN, "%f", value_d);
		} else{
			snprintf(buf, SENSOR_CONVERTED_LEN, "\"N/A\"");
		}
	}

	if(GPS_sensor_ALT_reading.publish){
		ret = GPS_sensor.value(GPS_SENSOR_TYPE_ALT);
		buf = GPS_sensor_ALT_reading.converted;
		value = *(float *)ret;
		if(ret != SENSOR_ERROR){
			GPS_sensor_LONG_reading.raw_f = value;

			memset(buf, 0, SENSOR_CONVERTED_LEN);
			snprintf(buf, SENSOR_CONVERTED_LEN, "%f", value);
		} else{
			snprintf(buf, SENSOR_CONVERTED_LEN, "\"N/A\"");
		}
	}

	if(GPS_sensor_SPEED_reading.publish){
		ret = GPS_sensor.value(GPS_SENSOR_TYPE_SPEED);
		buf = GPS_sensor_SPEED_reading.converted;
		value = *(float *)ret;
		if(ret != SENSOR_ERROR){
			GPS_sensor_LONG_reading.raw_f = value;

			memset(buf, 0, SENSOR_CONVERTED_LEN);
			snprintf(buf, SENSOR_CONVERTED_LEN, "%f", value);
		} else{
			snprintf(buf, SENSOR_CONVERTED_LEN, "\"N/A\"");
		}
	}

	SENSORS_DEACTIVATE(GPS_sensor);

}
#endif
/*---------------------------------------------------------------------------*/

/*---------------------------------------------------------------------------*/
static void init_sensors(void)
{
	list_add(sensor_list, &temp_reading);
	snprintf(temp_reading.converted, SENSOR_CONVERTED_LEN, "\"N/A\"");

#ifdef NODE_STEP_MOTOR
	list_add(sensor_list, &step_motor_reading);
	snprintf(step_motor_reading.converted, SENSOR_CONVERTED_LEN, "\"N/A\"");

	list_add(sensor_list, &dht11_temperature_reading);
	list_add(sensor_list, &dht11_humidity_reading);
	snprintf(dht11_temperature_reading.converted, SENSOR_CONVERTED_LEN, "\"N/A\"");
	snprintf(dht11_humidity_reading.converted, SENSOR_CONVERTED_LEN, "\"N/A\"");
#endif

#ifdef NODE_LIGHT
	snprintf(RGB_sensor_reading.converted, SENSOR_CONVERTED_LEN, "\"N/A\"");
#endif

#ifdef NODE_HARD_LIGHT
	snprintf(hard_RGB_sensor_reading.converted, SENSOR_CONVERTED_LEN, "\"N/A\"");
#endif

#ifdef NODE_4_ch_relay
	list_add(sensor_list, &dht11_temperature_reading);
	list_add(sensor_list, &dht11_humidity_reading);
	snprintf(dht11_temperature_reading.converted, SENSOR_CONVERTED_LEN, "\"N/A\"");
	snprintf(dht11_humidity_reading.converted, SENSOR_CONVERTED_LEN, "\"N/A\"");

#endif

#ifdef NODE_PRESSURE
	list_add(sensor_list, &bmp_280_sensor_press_reading);
	list_add(sensor_list, &bmp_280_sensor_temp_reading);
	snprintf(bmp_280_sensor_press_reading.converted, SENSOR_CONVERTED_LEN, "\"N/A\"");
	snprintf(bmp_280_sensor_temp_reading.converted, SENSOR_CONVERTED_LEN, "\"N/A\"");
#endif

#ifdef NODE_HTU21D
	list_add(sensor_list, &HTU21D_sensor_humid_reading);
	list_add(sensor_list, &HTU21D_sensor_temp_reading);
	snprintf(HTU21D_sensor_humid_reading.converted,
			SENSOR_CONVERTED_LEN, "\"N/A\"");
	snprintf(HTU21D_sensor_temp_reading.converted,
			SENSOR_CONVERTED_LEN, "\"N/A\"");
#endif

#ifdef NODE_GPS
	list_add(sensor_list, &GPS_sensor_LAT_reading);
	list_add(sensor_list, &GPS_sensor_LONG_reading);
	list_add(sensor_list, &GPS_sensor_ALT_reading);
	list_add(sensor_list, &GPS_sensor_SPEED_reading);
	snprintf(GPS_sensor_LAT_reading.converted, SENSOR_CONVERTED_LEN, "\"N/A\"");
	snprintf(GPS_sensor_LONG_reading.converted, SENSOR_CONVERTED_LEN, "\"N/A\"");
	snprintf(GPS_sensor_ALT_reading.converted, SENSOR_CONVERTED_LEN, "\"N/A\"");
	snprintf(GPS_sensor_SPEED_reading.converted, SENSOR_CONVERTED_LEN, "\"N/A\"");
#endif
}


static void trigger_sensors(void)
{
	SENSORS_ACTIVATE(SAM4S_ADC_TS_sensor);

#ifdef NODE_STEP_MOTOR
	SENSORS_ACTIVATE(step_sensor);
	SENSORS_ACTIVATE(dht11_sensor);
#endif

#ifdef NODE_LIGHT
	SENSORS_ACTIVATE(RGB_sensor);
#endif

#ifdef NODE_HARD_LIGHT
	SENSORS_ACTIVATE(hard_RGB_sensor);
#endif

#ifdef NODE_4_ch_relay
	SENSORS_ACTIVATE(dht11_sensor);
	SENSORS_ACTIVATE(ch4_relay_PD956);
#endif

#ifdef NODE_PRESSURE
	SENSORS_ACTIVATE(bmp_280_sensor);
#endif

#ifdef NODE_HTU21D
	SENSORS_ACTIVATE(HTU21D_sensor);
#endif

#ifdef NODE_GPS
	SENSORS_ACTIVATE(GPS_sensor);
#endif
}
extern void
register_http_post_handlers(void);

uint32_t sensor_busy;
/*---------------------------------------------------------------------------*/
PROCESS_THREAD(PD956_MAIN_process, ev, data)
{
	uint8_t ret;
	PROCESS_BEGIN();

	printf("PD956 MQTT Process\n");

	init_sensors();

	MQTT_publish_sensor_data_event = process_alloc_event();
	config_loaded_event = process_alloc_event();
	load_config_defaults = process_alloc_event();
	Trig_sensors = process_alloc_event();

	/* Start all other (enabled) processes first */
	process_start(&httpd_simple_process, NULL);
	register_http_post_handlers();

	MQTT_init_config();

	process_start(&ntpd_process, NULL);

	/*
	 * Now that processes have set their own config default values, set our
	 * own defaults and restore saved config from flash...
	 */
	web_demo_config.sensors_bitmap = 0xFFFFFFFF; /* all on by default */
	web_demo_config.def_rt_ping_interval = DEFAULT_RSSI_MEAS_INTERVAL;
	ret = load_config();

	if(!ret)
		process_start(&mqtt_client_process, NULL);

	/*
	 * Notify all other processes (basically the ones in this demo) that the
	 * configuration has been loaded from flash, in case they care
	 */
	process_post(PROCESS_BROADCAST, config_loaded_event, NULL);


	if(!ret){
		sensor_busy = web_demo_config.sensors_bitmap;
		trigger_sensors();
	}else{
		sensor_busy = 0xffffffff;
	}


	while(1){

		if(ev == Trig_sensors){
			trigger_sensors();
		}

		if(ev == httpd_simple_event_new_config){
			save_config();
		} else if(ev == sensors_event && data == &SAM4S_ADC_TS_sensor){
			get_temp_reading();
			sensor_busy &= ~(1ul << PD956_WEB_DEMO_SENSOR_ADC);

#ifdef NODE_STEP_MOTOR
		} else if(ev == sensors_event && data == &step_sensor){
			get_step_reading();
			sensor_busy &= ~(1ul<<PD956_WEB_DEMO_SENSOR_STEP);
#endif

#ifdef NODE_LIGHT
		} else if(ev == sensors_event && data == &RGB_sensor){
			get_RGB_reading();
			sensor_busy &= ~(1ul<<PD956_WEB_DEMO_SENSOR_RGB);
#endif

#if defined(NODE_STEP_MOTOR) || defined(NODE_4_ch_relay)
		} else if(ev == sensors_event && data == &dht11_sensor){
			get_dht11_temperature_reading();
			sensor_busy &= ~(1ul<<PD956_WEB_DEMO_SENSOR_DHT11_TEMP);
			sensor_busy &= ~(1ul<<PD956_WEB_DEMO_SENSOR_DHT11_HUMIDITY);
#endif

#ifdef NODE_HTU21D
		} else if(ev == sensors_event && data == &HTU21D_sensor){
			get_HTU21D_reading();
			sensor_busy &= ~(1ul << PD956_WEB_DEMO_SENSOR_HTU21D_TEMP);
			sensor_busy &= ~(1ul << PD956_WEB_DEMO_SENSOR_HTU21D_humid);
#endif

#ifdef NODE_PRESSURE
		} else if(ev == sensors_event && data == &bmp_280_sensor){
			get_bmp_reading();
			sensor_busy &= ~(1ul<<PD956_WEB_DEMO_SENSOR_BMP280_PRES);
			sensor_busy &= ~(1ul<<PD956_WEB_DEMO_SENSOR_BMP280_TEMP);
#endif

#ifdef NODE_GPS
		} else if(ev == sensors_event && data == &GPS_sensor){
			get_GPS_reading();
			sensor_busy &= ~(1ul << PD956_WEB_DEMO_SENSOR_GPS_LONG);
			sensor_busy &= ~(1ul << PD956_WEB_DEMO_SENSOR_GPS_LAT);
			sensor_busy &= ~(1ul << PD956_WEB_DEMO_SENSOR_GPS_ALT);
			sensor_busy &= ~(1ul << PD956_WEB_DEMO_SENSOR_GPS_SPEED);
#endif

		}
		if(!sensor_busy){
			sensor_busy = web_demo_config.sensors_bitmap;
			process_post(PROCESS_BROADCAST, MQTT_publish_sensor_data_event,
					NULL);
		}

		PROCESS_YIELD();
	}

PROCESS_END();
}
/*---------------------------------------------------------------------------*/

/**
 * @}
 */
