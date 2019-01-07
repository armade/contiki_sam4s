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
#include "ftp.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "PD_FLASH/FLASH_driver.h"
#ifdef NODE_GPS
#include "gpsd.h"
#endif

/*---------------------------------------------------------------------------*/

PROCESS(PD956_MAIN_process, "PD956 MQTT");
/*---------------------------------------------------------------------------*/
/*
 * Update sensor readings in a staggered fashion every SENSOR_READING_PERIOD
 * ticks + a random interval between 0 and SENSOR_READING_RANDOM ticks
 */

#define INSERT_TXT(x,y) memcpy(x,y,sizeof(y))
#define INSERT_NA(x) 	INSERT_TXT(x,"\"N/A\"")

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
LIST(MQTT_sensor_list);
/*---------------------------------------------------------------------------*/
/* The objects representing sensors used in this demo */

#define DEMO_SENSOR2(name,description,unit,type_def,HASS_CLASS,config_topic,sub_topic,sub_handler,class) \
		static MQTT_sensor_reading_t name = {\
			.next = NULL,\
			.raw = 0,\
			.last = 0,\
			.descr =  description,\
			.form_field = description,\
			.units = unit,\
			.type = type_def,\
			.HASS_class = HASS_CLASS,\
			.publish = 1,\
			.changed = 1,\
			.component_topic_config = (char *)config_topic,\
			.component_topic_sub = (char *)sub_topic,\
			.MQTT_subscr_ele.data_handler = sub_handler,\
			.device_class =class,\
		};

/*sensors */
DEMO_SENSOR2(temp_reading,						"CPUtemperature",UNIT_TEMP,		PD956_WEB_DEMO_SENSOR_ADC,				sensor_class,		sensor_config_topic,NULL,NULL,None);


#ifdef step_motor_reading
DEMO_SENSOR2(temp_reading,						"Step_Position",UNIT_STEP,		PD956_WEB_DEMO_SENSOR_STEP,				cover_class,		NULL,NULL,NULL,None);
#endif

#ifdef NODE_LIGHT
												// NB names are hardcoded. Do not change.
DEMO_SENSOR2(soft_RGB_switch_sensor_reading,	"switch",		UNIT_NONE,		PD956_WEB_DEMO_SENSOR_RGB,				light_class,		light_config_topic,	light_sub_topic,pub_light_soft_switch_handler,None);
DEMO_SENSOR2(soft_RGB_bright_sensor_reading,	"brightness",	UNIT_NONE,		PD956_WEB_DEMO_SENSOR_RGB,				light_class,		NULL,				light_sub_topic,pub_light_soft_brightness_handler,None);
DEMO_SENSOR2(soft_RGB_rgb_sensor_reading,		"color",		UNIT_NONE,		PD956_WEB_DEMO_SENSOR_RGB,				light_class,		NULL,				light_sub_topic,pub_light_soft_rgb_handler,None);
DEMO_SENSOR2(soft_RGB_effect_sensor_reading,	"effect",		UNIT_NONE,		PD956_WEB_DEMO_SENSOR_RGB,				light_class,		NULL,				light_sub_topic,pub_light_soft_effect_handler,None);
#endif

#ifdef NODE_HARD_LIGHT																													//NB: only one config is necessary
												// NB names are hardcoded. Do not change.
DEMO_SENSOR2(hard_RGB_switch_sensor_reading,	"switch",		UNIT_NONE,		PD956_WEB_DEMO_SENSOR_RGB,				light_class,		light_config_topic,	light_sub_topic,pub_light_hard_switch_handler,None);
DEMO_SENSOR2(hard_RGB_bright_sensor_reading,	"brightness",	UNIT_NONE,		PD956_WEB_DEMO_SENSOR_RGB,				light_class,		NULL,				light_sub_topic,pub_light_hard_brightness_handler,None);
DEMO_SENSOR2(hard_RGB_rgb_sensor_reading,		"color",		UNIT_NONE,		PD956_WEB_DEMO_SENSOR_RGB,				light_class,		NULL,				light_sub_topic,pub_light_hard_rgb_handler,None);
DEMO_SENSOR2(hard_RGB_effect_sensor_reading,	"effect",		UNIT_NONE,		PD956_WEB_DEMO_SENSOR_RGB,				light_class,		NULL,				light_sub_topic,pub_light_hard_effect_handler,None);
#endif

#ifdef NODE_DHT11
DEMO_SENSOR2(dht11_temperature_reading,			"Temperature",	UNIT_TEMP,		PD956_WEB_DEMO_SENSOR_DHT11_TEMP,		sensor_class,		sensor_config_topic,NULL,NULL,None);
DEMO_SENSOR2(dht11_humidity_reading,			"Humidity",		UNIT_HUMIDITY,	PD956_WEB_DEMO_SENSOR_DHT11_HUMIDITY,	sensor_class, 		sensor_config_topic,NULL,NULL,None);
#endif

#ifdef NODE_BMP280
DEMO_SENSOR2(bmp_280_sensor_press_reading,		"Pressure",		UNIT_PRES,		PD956_WEB_DEMO_SENSOR_BMP280_PRES,		sensor_class,		sensor_config_topic,NULL,NULL,None);
DEMO_SENSOR2(bmp_280_sensor_temp_reading,		"Temperature",	UNIT_TEMP,		PD956_WEB_DEMO_SENSOR_BMP280_TEMP,		sensor_class,		sensor_config_topic,NULL,NULL,None);
#endif

#ifdef NODE_HTU21D
DEMO_SENSOR2(HTU21D_sensor_humid_reading,		"Humidity",		UNIT_HUMIDITY,	PD956_WEB_DEMO_SENSOR_HTU21D_humid,		sensor_class,		sensor_config_topic,NULL,NULL,None);
DEMO_SENSOR2(HTU21D_sensor_temp_reading,		"Temperature",	UNIT_TEMP,		PD956_WEB_DEMO_SENSOR_HTU21D_TEMP,		sensor_class,		sensor_config_topic,NULL,NULL,None);
#endif


#ifdef NODE_GPS
DEMO_SENSOR2(GPS_sensor_LONG_reading,			"Lon",			UNIT_ANGLE,		PD956_WEB_DEMO_SENSOR_GPS_LONG,			sensor_class,		sensor_config_topic,NULL,NULL,None);
DEMO_SENSOR2(GPS_sensor_LAT_reading,			"Lat",			UNIT_ANGLE,		PD956_WEB_DEMO_SENSOR_GPS_LAT,			sensor_class,		sensor_config_topic,NULL,NULL,None);
DEMO_SENSOR2(GPS_sensor_ALT_reading,			"Alt",			UNIT_DISTANCE,	PD956_WEB_DEMO_SENSOR_GPS_ALT,			sensor_class,		sensor_config_topic,NULL,NULL,None);
DEMO_SENSOR2(GPS_sensor_SPEED_reading,			"spd",			UNIT_SPEED,		PD956_WEB_DEMO_SENSOR_GPS_SPEED,		sensor_class,		sensor_config_topic,NULL,NULL,None);
#endif

#ifdef NODE_4_ch_relay
DEMO_SENSOR2(NODE_4_ch_relay1_reading,			"relay1",		UNIT_NONE,		PD956_WEB_DEMO_SENSOR_RELAY1,			switch_class,		switch_config_topic,switch_sub_topic,pub_relay1_handler,None);
DEMO_SENSOR2(NODE_4_ch_relay2_reading,			"relay2",		UNIT_NONE,		PD956_WEB_DEMO_SENSOR_RELAY2,			switch_class,		switch_config_topic,switch_sub_topic,pub_relay2_handler,None);
DEMO_SENSOR2(NODE_4_ch_relay3_reading,			"relay3",		UNIT_NONE,		PD956_WEB_DEMO_SENSOR_RELAY3,			switch_class,		switch_config_topic,switch_sub_topic,pub_relay3_handler,None);
DEMO_SENSOR2(NODE_4_ch_relay4_reading,			"relay4",		UNIT_NONE,		PD956_WEB_DEMO_SENSOR_RELAY4,			switch_class,		switch_config_topic,switch_sub_topic,pub_relay4_handler,None);
#endif


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

		for (reading = list_head(MQTT_sensor_list); reading != NULL ; reading =
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

		for (reading = list_head(MQTT_sensor_list); reading != NULL ; reading =
				list_item_next(reading)){
			if(web_demo_config.sensors_bitmap & (1 << reading->type)){
				reading->publish = 1;
			} else{
				reading->publish = 0;
				INSERT_NA(reading->converted);
			}
		}
	} else{
		// If Bad config show all off.
		for (reading = list_head(MQTT_sensor_list); reading != NULL ; reading =	list_item_next(reading)){
				reading->publish = 0;
				INSERT_NA(reading->converted);
		}
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
	return list_head(MQTT_sensor_list);
}
/*---------------------------------------------------------------------------*/
void Restore_defaults(void)
{
	MQTT_sensor_reading_t *reading = NULL;

	for (reading = list_head(MQTT_sensor_list); reading != NULL ; reading =
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
			INSERT_NA(buf);
		}
	}

	SENSORS_DEACTIVATE(SAM4S_ADC_TS_sensor);
}

#ifdef NODE_LIGHT
static void
get_RGB_soft_reading(void)
{
	static int status_val;
	char *buf;
	RGB_soft_t tmp;

	tmp.all = ((RGB_soft_t *) soft_RGB_ctrl_sensor.value(SENSOR_ERROR))->all;
	status_val = soft_RGB_ctrl_sensor.status(0);

	if(soft_RGB_switch_sensor_reading.publish){
		if((status_val&0xf) == SENSOR_STATUS_READY){
			soft_RGB_switch_sensor_reading.raw = status_val;

			buf = soft_RGB_switch_sensor_reading.converted;
			memset(buf, 0, SENSOR_CONVERTED_LEN);
			INSERT_TXT(buf,"\"ON\"");
		}
		else
		{
			soft_RGB_switch_sensor_reading.raw = status_val;

			buf = soft_RGB_switch_sensor_reading.converted;
			memset(buf, 0, SENSOR_CONVERTED_LEN);
			INSERT_TXT(buf,"\"OFF\"");
		}
	}

	if(soft_RGB_effect_sensor_reading.publish){
		if((status_val>>12) == 1){
			soft_RGB_effect_sensor_reading.raw = status_val;

			buf = soft_RGB_effect_sensor_reading.converted;
			memset(buf, 0, SENSOR_CONVERTED_LEN);
			INSERT_TXT(buf,"\"colorloop\"");
		}
		else if((status_val>>12) == 2)
		{
			soft_RGB_effect_sensor_reading.raw = status_val;

			buf = soft_RGB_effect_sensor_reading.converted;
			memset(buf, 0, SENSOR_CONVERTED_LEN);
			INSERT_TXT(buf,"\"random\"");
		}
		else
		{
			buf = soft_RGB_effect_sensor_reading.converted;
			memset(buf, 0, SENSOR_CONVERTED_LEN);
			INSERT_TXT(buf,"\"none\"");
		}
	}

	if(soft_RGB_bright_sensor_reading.publish){

		soft_RGB_bright_sensor_reading.raw =  tmp.led.brightness;

		buf = soft_RGB_bright_sensor_reading.converted;
		memset(buf, 0, SENSOR_CONVERTED_LEN);

		snprintf(buf, SENSOR_CONVERTED_LEN, "%d", tmp.led.brightness);
	}

	if(soft_RGB_rgb_sensor_reading.publish){

		soft_RGB_rgb_sensor_reading.raw =  tmp.led.r;

		buf = soft_RGB_rgb_sensor_reading.converted;
		memset(buf, 0, SENSOR_CONVERTED_LEN);

		snprintf(buf, SENSOR_CONVERTED_LEN, "[%d,%d,%d]", (tmp.led.r>>4), (tmp.led.g>>4), (tmp.led.b>>4));
	}

}
#endif

#ifdef NODE_HARD_LIGHT
static void
get_RGB_hard_reading(void)
{
	static int status_val;
	char *buf;
	RGB_hard_t tmp;

	tmp.all = ((RGB_hard_t *) hard_RGB_ctrl_sensor.value(SENSOR_ERROR))->all;
	status_val = hard_RGB_ctrl_sensor.status(0);

	if(hard_RGB_switch_sensor_reading.publish){
		if((status_val&0xf) == SENSOR_STATUS_READY){
			hard_RGB_switch_sensor_reading.raw = status_val;

			buf = hard_RGB_switch_sensor_reading.converted;
			memset(buf, 0, SENSOR_CONVERTED_LEN);
			INSERT_TXT(buf,"\"ON\"");
		}
		else
		{
			hard_RGB_switch_sensor_reading.raw = status_val;

			buf = hard_RGB_switch_sensor_reading.converted;
			memset(buf, 0, SENSOR_CONVERTED_LEN);
			INSERT_TXT(buf,"\"OFF\"");
		}
	}

	if(hard_RGB_effect_sensor_reading.publish){
		if((status_val>>12) == 1){
			hard_RGB_effect_sensor_reading.raw = status_val;

			buf = hard_RGB_effect_sensor_reading.converted;
			memset(buf, 0, SENSOR_CONVERTED_LEN);
			INSERT_TXT(buf,"\"colorloop\"");
		}
		else if((status_val>>12) == 2)
		{
			hard_RGB_effect_sensor_reading.raw = status_val;

			buf = hard_RGB_effect_sensor_reading.converted;
			memset(buf, 0, SENSOR_CONVERTED_LEN);
			INSERT_TXT(buf,"\"random\"");
		}
		else{
			buf = hard_RGB_effect_sensor_reading.converted;
			memset(buf, 0, SENSOR_CONVERTED_LEN);
			INSERT_TXT(buf,"\"none\"");
		}

	}

	if(hard_RGB_bright_sensor_reading.publish){

		hard_RGB_bright_sensor_reading.raw =  tmp.led.brightness;

		buf = hard_RGB_bright_sensor_reading.converted;
		memset(buf, 0, SENSOR_CONVERTED_LEN);

		snprintf(buf, SENSOR_CONVERTED_LEN, "%d", tmp.led.brightness);
	}

	if(hard_RGB_rgb_sensor_reading.publish){

		hard_RGB_rgb_sensor_reading.raw =  tmp.led.r;

		buf = hard_RGB_rgb_sensor_reading.converted;
		memset(buf, 0, SENSOR_CONVERTED_LEN);

		snprintf(buf, SENSOR_CONVERTED_LEN,"[%d,%d,%d]", (tmp.led.r>>4), (tmp.led.g>>4), (tmp.led.b>>4));
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
			INSERT_NA(buf);
		}
	}
	SENSORS_DEACTIVATE(step_sensor);

	/* Flow.
	 *  1) MQTT set value (received)
	 *  2) SENSOR_VALUE(pos) // set final pos
	 *  2) process_post(PROCESS_BROADCAST, Trig_sensors, NULL); //reset sensorbits. sensor is called with enable=1 - initialize stepmotor control (motor is now running)
	 *  3) POS=request_POS sensor_changed // when position is reached signal sensor_changed
	 *  4) get_step_reading() // get position and publish it
	 *  7) SENSOR_DEACTIVATE() // remove pio control
	 */
}
#endif

#ifdef NODE_DHT11
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

			snprintf(buf, SENSOR_CONVERTED_LEN, "%d.%d",(value>>8)&0xff,value&0xff);
		}
		else
		{
			INSERT_NA(buf);
		}
	}

	if(dht11_humidity_reading.publish){

		value = dht11_sensor.value(HUMIDITY_READING_SPLIT);
		buf = dht11_humidity_reading.converted;
		if(value != SENSOR_ERROR){
			dht11_humidity_reading.raw = value;

			memset(buf, 0, SENSOR_CONVERTED_LEN);

			snprintf(buf, SENSOR_CONVERTED_LEN, "%d.%d",(value>>8)&0xff,value&0xff);
		}
		else
		{
			INSERT_NA(buf);
		}
	}
	SENSORS_DEACTIVATE(dht11_sensor);
}
#endif
/*---------------------------------------------------------------------------*/
#ifdef NODE_BMP280

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
			INSERT_NA(buf);
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
			INSERT_NA(buf);
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
			INSERT_NA(buf);
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
			INSERT_NA(buf);
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
	int ret;
	int low, high;

	if(GPS_sensor_LONG_reading.publish){
		ret = GPS_sensor.value(GPS_SENSOR_TYPE_LONG);
		buf = GPS_sensor_LONG_reading.converted;

		if(ret != SENSOR_ERROR){
			value = *(float *)ret;

			memset(buf, 0, SENSOR_CONVERTED_LEN);
			high = value;
			low = (value - high) * 10000000;
			snprintf(buf, SENSOR_CONVERTED_LEN, "%d.%.7d", high, low);
		} else{
			INSERT_NA(buf);
		}
	}

	if(GPS_sensor_LAT_reading.publish){
		ret = GPS_sensor.value(GPS_SENSOR_TYPE_LAT);
		buf = GPS_sensor_LAT_reading.converted;

		if(ret != SENSOR_ERROR){
			value = *(float *)ret;

			memset(buf, 0, SENSOR_CONVERTED_LEN);
			high = value;
			low = (value - high) * 10000000;
			snprintf(buf, SENSOR_CONVERTED_LEN, "%d.%.7d", high, low);
		} else{
			INSERT_NA(buf);
		}
	}

	if(GPS_sensor_ALT_reading.publish){
		ret = GPS_sensor.value(GPS_SENSOR_TYPE_ALT);
		buf = GPS_sensor_ALT_reading.converted;

		if(ret != SENSOR_ERROR){
			value = *(float *)ret;

			memset(buf, 0, SENSOR_CONVERTED_LEN);
			high = value;
			low = (value - high) * 10000;
			snprintf(buf, SENSOR_CONVERTED_LEN, "%d.%.4d", high, low);
		} else{
			INSERT_NA(buf);
		}
	}

	if(GPS_sensor_SPEED_reading.publish){
		ret = GPS_sensor.value(GPS_SENSOR_TYPE_SPEED);
		buf = GPS_sensor_SPEED_reading.converted;

		if(ret != SENSOR_ERROR){
			value = *(float *)ret;

			memset(buf, 0, SENSOR_CONVERTED_LEN);
			high = value;
			low = (value - high) * 10000;
			snprintf(buf, SENSOR_CONVERTED_LEN, "%d.%d", high, low);
		} else{
			INSERT_NA(buf);
		}
	}

	SENSORS_DEACTIVATE(GPS_sensor);

}
#endif

#ifdef NODE_4_ch_relay
static void get_relay_reading()
{
	char *buf;
	int ret;

	if(NODE_4_ch_relay1_reading.publish){
		ret = ch4_relay_PD956.value(STATUS_CH1);
		buf = NODE_4_ch_relay1_reading.converted;

		if(ret != SENSOR_ERROR){
			NODE_4_ch_relay1_reading.raw = ret;
			memset(buf, 0, SENSOR_CONVERTED_LEN);
			if(ret)
				INSERT_TXT(buf,"\"ON\"");
			else
				INSERT_TXT(buf,"\"OFF\"");
		} else{
			INSERT_NA(buf);
		}
	}

	if(NODE_4_ch_relay2_reading.publish){
		ret = ch4_relay_PD956.value(STATUS_CH2);
		buf = NODE_4_ch_relay2_reading.converted;

		if(ret != SENSOR_ERROR){
			NODE_4_ch_relay2_reading.raw = ret;
			memset(buf, 0, SENSOR_CONVERTED_LEN);
			if(ret)
				INSERT_TXT(buf,"\"ON\"");
			else
				INSERT_TXT(buf,"\"OFF\"");
		} else{
			INSERT_NA(buf);
		}
	}

	if(NODE_4_ch_relay3_reading.publish){
		ret = ch4_relay_PD956.value(STATUS_CH3);
		buf = NODE_4_ch_relay3_reading.converted;

		if(ret != SENSOR_ERROR){
			NODE_4_ch_relay3_reading.raw = ret;
			memset(buf, 0, SENSOR_CONVERTED_LEN);
			if(ret)
				INSERT_TXT(buf,"\"ON\"");
			else
				INSERT_TXT(buf,"\"OFF\"");
		} else{
			INSERT_NA(buf);
		}
	}

	if(NODE_4_ch_relay4_reading.publish){
		ret = ch4_relay_PD956.value(STATUS_CH4);
		buf = NODE_4_ch_relay4_reading.converted;

		if(ret != SENSOR_ERROR){
			NODE_4_ch_relay4_reading.raw = ret;
			memset(buf, 0, SENSOR_CONVERTED_LEN);
			if(ret)
				INSERT_TXT(buf,"\"ON\"");
			else
				INSERT_TXT(buf,"\"OFF\"");
		} else{
			INSERT_NA(buf);
		}
	}

	SENSORS_DEACTIVATE(ch4_relay_PD956);
}
#endif
/*---------------------------------------------------------------------------*/

/*---------------------------------------------------------------------------*/
static void init_sensors(void)
{
	list_add(MQTT_sensor_list, &temp_reading);
	INSERT_NA(temp_reading.converted);

#ifdef NODE_STEP_MOTOR
	list_add(MQTT_sensor_list, &step_motor_reading);
	INSERT_NA(step_motor_reading.converted);
#endif

#ifdef NODE_LIGHT
	list_add(MQTT_sensor_list, &soft_RGB_switch_sensor_reading);
	INSERT_NA(soft_RGB_switch_sensor_reading.converted);

	list_add(MQTT_sensor_list, &soft_RGB_bright_sensor_reading);
	INSERT_NA(soft_RGB_switch_sensor_reading.converted);

	list_add(MQTT_sensor_list, &soft_RGB_rgb_sensor_reading);
	INSERT_NA(soft_RGB_switch_sensor_reading.converted);

	list_add(MQTT_sensor_list, &soft_RGB_effect_sensor_reading);
	INSERT_NA(soft_RGB_effect_sensor_reading.converted);
#endif

#ifdef NODE_HARD_LIGHT
	list_add(MQTT_sensor_list, &hard_RGB_switch_sensor_reading);
	INSERT_NA(hard_RGB_switch_sensor_reading.converted);

	list_add(MQTT_sensor_list, &hard_RGB_bright_sensor_reading);
	INSERT_NA(hard_RGB_bright_sensor_reading.converted);

	list_add(MQTT_sensor_list, &hard_RGB_rgb_sensor_reading);
	INSERT_NA(hard_RGB_rgb_sensor_reading.converted);

	list_add(MQTT_sensor_list, &hard_RGB_effect_sensor_reading);
	INSERT_NA(hard_RGB_effect_sensor_reading.converted);
#endif

#ifdef NODE_DHT11
	list_add(MQTT_sensor_list, &dht11_temperature_reading);
	list_add(MQTT_sensor_list, &dht11_humidity_reading);
	INSERT_NA(dht11_temperature_reading.converted);
	INSERT_NA(dht11_humidity_reading.converted);
#endif

#ifdef NODE_BMP280
	list_add(MQTT_sensor_list, &bmp_280_sensor_press_reading);
	list_add(MQTT_sensor_list, &bmp_280_sensor_temp_reading);
	INSERT_NA(bmp_280_sensor_press_reading.converted);
	INSERT_NA(bmp_280_sensor_temp_reading.converted);
#endif

#ifdef NODE_HTU21D
	list_add(MQTT_sensor_list, &HTU21D_sensor_humid_reading);
	list_add(MQTT_sensor_list, &HTU21D_sensor_temp_reading);
	INSERT_NA(HTU21D_sensor_humid_reading.converted);
	INSERT_NA(HTU21D_sensor_temp_reading.converted);
#endif

#ifdef NODE_GPS
	list_add(MQTT_sensor_list, &GPS_sensor_LAT_reading);
	list_add(MQTT_sensor_list, &GPS_sensor_LONG_reading);
	list_add(MQTT_sensor_list, &GPS_sensor_ALT_reading);
	list_add(MQTT_sensor_list, &GPS_sensor_SPEED_reading);
	INSERT_NA(GPS_sensor_LAT_reading.converted);
	INSERT_NA(GPS_sensor_LONG_reading.converted);
	INSERT_NA(GPS_sensor_ALT_reading.converted);
	INSERT_NA(GPS_sensor_SPEED_reading.converted);
#endif

#ifdef NODE_4_ch_relay
	list_add(MQTT_sensor_list, &NODE_4_ch_relay1_reading);
	list_add(MQTT_sensor_list, &NODE_4_ch_relay2_reading);
	list_add(MQTT_sensor_list, &NODE_4_ch_relay3_reading);
	list_add(MQTT_sensor_list, &NODE_4_ch_relay4_reading);
	INSERT_NA(NODE_4_ch_relay1_reading.converted);
	INSERT_NA(NODE_4_ch_relay2_reading.converted);
	INSERT_NA(NODE_4_ch_relay3_reading.converted);
	INSERT_NA(NODE_4_ch_relay4_reading.converted);
#endif
}


static void trigger_sensors(void)
{
	const struct sensors_sensor *sensors_ptr;
	for (sensors_ptr = sensors_first(); sensors_ptr != NULL; sensors_ptr = sensors_next(sensors_ptr))
	{
		sensors_ptr->configure(SENSORS_ACTIVE, 1);
	}
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
#ifdef NODE_GPS
	process_start(&gpsd_process, NULL);
#endif

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

	// TODO: test if this works now
	//process_start(&ftp_process, NULL);
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

#ifdef NODE_DHT11
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

#ifdef NODE_BMP280
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

#ifdef NODE_4_ch_relay
		} else if(ev == sensors_event && data == &ch4_relay_PD956){
			get_relay_reading();
			sensor_busy &= ~(1ul << PD956_WEB_DEMO_SENSOR_RELAY1);
			sensor_busy &= ~(1ul << PD956_WEB_DEMO_SENSOR_RELAY2);
			sensor_busy &= ~(1ul << PD956_WEB_DEMO_SENSOR_RELAY3);
			sensor_busy &= ~(1ul << PD956_WEB_DEMO_SENSOR_RELAY4);
#endif

#ifdef NODE_HARD_LIGHT
		} else if(ev == sensors_event && data == &hard_RGB_ctrl_sensor){
			get_RGB_hard_reading();
			sensor_busy &= ~(1ul<<PD956_WEB_DEMO_SENSOR_RGB);
#endif

#ifdef NODE_LIGHT
		} else if(ev == sensors_event && data == &soft_RGB_ctrl_sensor){
			get_RGB_soft_reading();
			sensor_busy &= ~(1ul<<PD956_WEB_DEMO_SENSOR_RGB);
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
