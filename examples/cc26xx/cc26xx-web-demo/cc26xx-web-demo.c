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
#include "rest-engine.h"
#include "board-peripherals.h"
#include "lib/sensors.h"
#include "lib/list.h"
#include "sys/process.h"
#include "net/ipv6/sicslowpan.h"
#include "button-sensor.h"
//#include "batmon-sensor.h"
#include "httpd-simple.h"
#include "cc26xx-web-demo.h"
#include "mqtt-client.h"
#include "coap-server.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "PD_FLASH/FLASH_driver.h"

//#include "ti-lib.h"
/*---------------------------------------------------------------------------*/
PROCESS_NAME(cetic_6lbr_client_process);
PROCESS(cc26xx_web_demo_process, "CC26XX Web Demo");
/*---------------------------------------------------------------------------*/
/*
 * Update sensor readings in a staggered fashion every SENSOR_READING_PERIOD
 * ticks + a random interval between 0 and SENSOR_READING_RANDOM ticks
 */

struct ctimer Internal_ADC_timer;
#ifdef NODE_STEP_MOTOR
struct ctimer Step_motor_timer;
#endif
#if defined(NODE_STEP_MOTOR) || defined(NODE_4_ch_relay)
struct ctimer dht11_temperature_timer;
struct ctimer dht11_humidity_timer;
#endif
#ifdef NODE_PREASURE
struct ctimer bmp_timer;
#endif
#ifdef NODE_LIGHT
struct ctimer RGB_light_timer;
#endif
#ifdef NODE_HTU21D
struct ctimer HTU21D_timer;
#endif
/*---------------------------------------------------------------------------*/
/* Provide visible feedback via LEDS while searching for a network */
#define NO_NET_LED_DURATION        (CC26XX_WEB_DEMO_NET_CONNECT_PERIODIC >> 1)

static struct etimer et;
//static struct ctimer ct;
/*---------------------------------------------------------------------------*/
/* Parent RSSI functionality */
#if CC26XX_WEB_DEMO_READ_PARENT_RSSI
static struct uip_icmp6_echo_reply_notification echo_reply_notification;
static struct etimer echo_request_timer;
int def_rt_rssi = 0;
#endif

/*---------------------------------------------------------------------------*/
process_event_t cc26xx_web_demo_publish_event;
process_event_t cc26xx_web_demo_config_loaded_event;
process_event_t cc26xx_web_demo_load_config_defaults;
/*---------------------------------------------------------------------------*/
/* Saved settings on flash: store, offset, magic */
#define CONFIG_FLASH_OFFSET        0
#define CONFIG_MAGIC      0x956a5007

cc26xx_web_demo_config_t web_demo_config;
/*---------------------------------------------------------------------------*/
/* A cache of sensor values. Updated periodically or upon key press */
LIST(sensor_list);
/*---------------------------------------------------------------------------*/
/* The objects representing sensors used in this demo */
#define DEMO_SENSOR(name, type, descr, xml_element, form_field, units, hass_component) \
  cc26xx_web_demo_sensor_reading_t name##_reading = \
  { NULL, 0, 0, descr, xml_element, form_field, units, type, 1, 1, hass_component}

/*sensors */
DEMO_SENSOR(temp, PD956_WEB_DEMO_SENSOR_ADC, "Internal temperature", "Internal-temperature", "Internal_temperature", CC26XX_WEB_DEMO_UNIT_TEMP,"sensor");

#ifdef NODE_STEP_MOTOR
DEMO_SENSOR(step_motor, PD956_WEB_DEMO_SENSOR_STEP, "Step-Position", "Step-position", "Step-position", CC26XX_WEB_DEMO_UNIT_STEP,"sensor");
#endif

#ifdef NODE_LIGHT
DEMO_SENSOR(RGB_sensor, PD956_WEB_DEMO_SENSOR_RGB, "RGB-light", "RGB-light", "RGB-light", CC26XX_WEB_DEMO_UNIT_NONE,"sensor");
#endif

#ifdef NODE_HARD_LIGHT
DEMO_SENSOR(hard_RGB_sensor, PD956_WEB_DEMO_SENSOR_RGB, "RGB-light", "RGB-light", "RGB-light", CC26XX_WEB_DEMO_UNIT_NONE,"sensor");
#endif

#if defined(NODE_STEP_MOTOR) || defined(NODE_4_ch_relay)
DEMO_SENSOR(dht11_temperature, PD956_WEB_DEMO_SENSOR_DHT11_TEMP,  "Temperature", "Temperature", "Temperature", CC26XX_WEB_DEMO_UNIT_TEMP,"sensor");
DEMO_SENSOR(dht11_humidity, PD956_WEB_DEMO_SENSOR_DHT11_HUMIDITY, "Humidity", "Humidity", "Humidity", CC26XX_WEB_DEMO_UNIT_HUMIDITY,"sensor");
#endif

#ifdef NODE_PREASURE
DEMO_SENSOR(bmp_280_sensor_press,PD956_WEB_DEMO_SENSOR_BMP280_PRES, "Pressure", "Pressure", "Pressure",	CC26XX_WEB_DEMO_UNIT_PRES,"sensor");
DEMO_SENSOR(bmp_280_sensor_temp,PD956_WEB_DEMO_SENSOR_BMP280_TEMP,  "Temperature", "Temperature", "Temperature", CC26XX_WEB_DEMO_UNIT_TEMP,"sensor");
#endif

#ifdef NODE_HTU21D
DEMO_SENSOR(HTU21D_sensor_humid,PD956_WEB_DEMO_SENSOR_HTU21D_humid, "Humidity", "Humidity", "Humidity",	CC26XX_WEB_DEMO_UNIT_HUMIDITY,"sensor");
DEMO_SENSOR(HTU21D_sensor_temp,PD956_WEB_DEMO_SENSOR_HTU21D_TEMP,  "Temperature", "Temperature", "Temperature", CC26XX_WEB_DEMO_UNIT_TEMP,"sensor");
#endif
/*---------------------------------------------------------------------------*/
/*static void
publish_led_off(void *d)
{
  //leds_off(CC26XX_WEB_DEMO_STATUS_LED);
}*/
/*---------------------------------------------------------------------------*/
static void
save_config()
{

	int ret;
	cc26xx_web_demo_sensor_reading_t *reading = NULL;

	flash_leave_deep_sleep();

	ret = flash_erase_df(CONFIG_FLASH_OFFSET, erase_4k_block);

	if(ret) {
	    printf("Error erasing flash\n");
	} else {
		web_demo_config.magic = CONFIG_MAGIC;
		web_demo_config.len = sizeof(cc26xx_web_demo_config_t);
		web_demo_config.sensors_bitmap = 0;

		for(reading = list_head(sensor_list);
			reading != NULL;
			reading = list_item_next(reading)) {
		  if(reading->publish) {
			web_demo_config.sensors_bitmap |= (1 << reading->type);
		  }
		}
		ret = flash_write_df(CONFIG_FLASH_OFFSET, (uint8_t *)&web_demo_config, sizeof(cc26xx_web_demo_config_t));

		if(ret) {
		  printf("Error saving config\n");
		}
	}
	flash_enter_deep_sleep();

}
/*---------------------------------------------------------------------------*/
static void
load_config()
{

	int ret;
	/* Read from flash into a temp buffer */
    cc26xx_web_demo_config_t tmp_cfg;
    cc26xx_web_demo_sensor_reading_t *reading = NULL;

    flash_leave_deep_sleep();

    ret = flash_read_df((uint8_t *)&tmp_cfg, sizeof(tmp_cfg), CONFIG_FLASH_OFFSET);


    if(ret) {
      printf("Error loading config\n");
      flash_enter_deep_sleep();
      return;
    }

    if(tmp_cfg.magic == CONFIG_MAGIC && tmp_cfg.len == sizeof(tmp_cfg)) {
    	memcpy(&web_demo_config, &tmp_cfg, sizeof(web_demo_config));

		for(reading = list_head(sensor_list); reading != NULL; reading = list_item_next(reading)) {
			if(web_demo_config.sensors_bitmap & (1 << reading->type)) {
				reading->publish = 1;
			} else {
				reading->publish = 0;
				snprintf(reading->converted, CC26XX_WEB_DEMO_CONVERTED_LEN, "\"N/A\"");
			}
		}
    }
    flash_enter_deep_sleep();

}
/*---------------------------------------------------------------------------*/
/* Don't start everything here, we need to dictate order of initialisation */
AUTOSTART_PROCESSES(&cc26xx_web_demo_process);
/*---------------------------------------------------------------------------*/
int
cc26xx_web_demo_ipaddr_sprintf(char *buf, uint8_t buf_len,
                               const uip_ipaddr_t *addr)
{
  uint16_t a;
  uint8_t len = 0;
  int i, f;
  for(i = 0, f = 0; i < sizeof(uip_ipaddr_t); i += 2) {
    a = (addr->u8[i] << 8) + addr->u8[i + 1];
    if(a == 0 && f >= 0) {
      if(f++ == 0) {
        len += snprintf(&buf[len], buf_len - len, "::");
      }
    } else {
      if(f > 0) {
        f = -1;
      } else if(i > 0) {
        len += snprintf(&buf[len], buf_len - len, ":");
      }
      len += snprintf(&buf[len], buf_len - len, "%x", a);
    }
  }

  return len;
}
/*---------------------------------------------------------------------------*/
const cc26xx_web_demo_sensor_reading_t *
cc26xx_web_demo_sensor_lookup(int sens_type)
{
  cc26xx_web_demo_sensor_reading_t *reading = NULL;

  for(reading = list_head(sensor_list);
      reading != NULL;
      reading = list_item_next(reading)) {
    if(reading->type == sens_type) {
      return reading;
    }
  }

  return NULL;
}
/*---------------------------------------------------------------------------*/
const cc26xx_web_demo_sensor_reading_t *
cc26xx_web_demo_sensor_first()
{
  return list_head(sensor_list);
}
/*---------------------------------------------------------------------------*/
void
cc26xx_web_demo_restore_defaults(void)
{
  cc26xx_web_demo_sensor_reading_t *reading = NULL;

  //leds_on(LEDS_ALL);

  for(reading = list_head(sensor_list);
      reading != NULL;
      reading = list_item_next(reading)) {
    reading->publish = 1;
  }

#if CC26XX_WEB_DEMO_MQTT_CLIENT
  process_post_synch(&mqtt_client_process,
                     cc26xx_web_demo_load_config_defaults, NULL);
#endif

#if CC26XX_WEB_DEMO_NET_UART
  process_post_synch(&net_uart_process, cc26xx_web_demo_load_config_defaults,
                     NULL);
#endif

  save_config();

  //leds_off(LEDS_ALL);
}
/*---------------------------------------------------------------------------*/
static int
defaults_post_handler(char *key, int key_len, char *val, int val_len)
{
  if(key_len != strlen("defaults") ||
     strncasecmp(key, "defaults", strlen("defaults")) != 0) {
    /* Not ours */
    return HTTPD_SIMPLE_POST_HANDLER_UNKNOWN;
  }

  cc26xx_web_demo_restore_defaults();

  return HTTPD_SIMPLE_POST_HANDLER_OK;
}
/*---------------------------------------------------------------------------*/
static int
sensor_readings_handler(char *key, int key_len, char *val, int val_len)
{
  cc26xx_web_demo_sensor_reading_t *reading = NULL;
  int ret;

  for(reading = list_head(sensor_list);
      reading != NULL;
      reading = list_item_next(reading)) {
    if(key_len == strlen(reading->form_field) &&
       strncmp(reading->form_field, key, strlen(key)) == 0) {

      ret = atoi(val);

      /* Be pedantic: only accept 0 and 1, not just any non-zero value */
      if(ret == 0) {
        reading->publish = 0;
        snprintf(reading->converted, CC26XX_WEB_DEMO_CONVERTED_LEN, "\"N/A\"");
      } else if(ret == 1) {
        reading->publish = 1;
      } else {
        return HTTPD_SIMPLE_POST_HANDLER_ERROR;
      }

      return HTTPD_SIMPLE_POST_HANDLER_OK;
    }
  }

  return HTTPD_SIMPLE_POST_HANDLER_UNKNOWN;
}
/*---------------------------------------------------------------------------*/
#if CC26XX_WEB_DEMO_READ_PARENT_RSSI
static int
ping_interval_post_handler(char *key, int key_len, char *val, int val_len)
{
  int ret = 0;

  if(key_len != strlen("ping_interval") ||
     strncasecmp(key, "ping_interval", strlen("ping_interval")) != 0) {
    /* Not ours */
    return HTTPD_SIMPLE_POST_HANDLER_UNKNOWN;
  }

  ret = atoi(val);

  if(ret < CC26XX_WEB_DEMO_RSSI_MEASURE_INTERVAL_MIN ||
     ret > CC26XX_WEB_DEMO_RSSI_MEASURE_INTERVAL_MAX) {
    return HTTPD_SIMPLE_POST_HANDLER_ERROR;
  }

  web_demo_config.def_rt_ping_interval = ret * CLOCK_SECOND;

  return HTTPD_SIMPLE_POST_HANDLER_OK;
}
#endif
/*---------------------------------------------------------------------------*/
HTTPD_SIMPLE_POST_HANDLER(sensor, sensor_readings_handler);
HTTPD_SIMPLE_POST_HANDLER(defaults, defaults_post_handler);

#if CC26XX_WEB_DEMO_READ_PARENT_RSSI
HTTPD_SIMPLE_POST_HANDLER(ping_interval, ping_interval_post_handler);
/*---------------------------------------------------------------------------*/
static void
echo_reply_handler(uip_ipaddr_t *source, uint8_t ttl, uint8_t *data, uint16_t datalen)
{
	if(uip_ip6addr_cmp(source, uip_ds6_defrt_choose()))
		def_rt_rssi = sicslowpan_get_last_rssi();

}
/*---------------------------------------------------------------------------*/
static void
ping_parent(void)
{
	if(uip_ds6_get_global(ADDR_PREFERRED) == NULL)
		return;

	uip_icmp6_send(uip_ds6_defrt_choose(), ICMP6_ECHO_REQUEST, 0,
                 CC26XX_WEB_DEMO_ECHO_REQ_PAYLOAD_LEN);
}
#endif
/*---------------------------------------------------------------------------*/

static void
init_temp_reading(void *not_used)
{
  SENSORS_ACTIVATE(SAM4S_ADC_TS_sensor);
}

static void
get_temp_reading(void)
{
	int value;
	int low,high;
	char *buf;
	clock_time_t next = web_demo_config.mqtt_config.pub_interval;

	if(temp_reading.publish) {
		value = SAM4S_ADC_TS_sensor.value(ADC_TS_SENSOR_TYPE_TEMP);
		if(value != SENSOR_ERROR) {
			temp_reading.raw = value;

			buf = temp_reading.converted;
			memset(buf, 0, CC26XX_WEB_DEMO_CONVERTED_LEN);
			high = value/1000;
			low = value-high*1000;
			snprintf(buf, CC26XX_WEB_DEMO_CONVERTED_LEN, "%d.%d", high,low);
		}
		else
		{
			snprintf(buf, CC26XX_WEB_DEMO_CONVERTED_LEN, "N/A");
		}
	}

	SENSORS_DEACTIVATE(SAM4S_ADC_TS_sensor);
	ctimer_set(&Internal_ADC_timer, next, init_temp_reading, NULL);
}

#ifdef NODE_LIGHT
static void
get_RGB_reading(void)
{
	int value;
	char *buf;
	//clock_time_t next =  web_demo_config.mqtt_config.pub_interval;
	RGB_t rgb;

	if(RGB_sensor_reading.publish) {

		value = RGB_sensor.value(SENSOR_ERROR);
		if(value != SENSOR_ERROR) {
			RGB_sensor_reading.raw = value;
			rgb.all = value;

			buf = RGB_sensor_reading.converted;
			memset(buf, 0, CC26XX_WEB_DEMO_CONVERTED_LEN);

			snprintf(buf, CC26XX_WEB_DEMO_CONVERTED_LEN, "%d,%d,%d",rgb.led.r,rgb.led.g,rgb.led.b);
		}
		else
		{
			snprintf(buf, CC26XX_WEB_DEMO_CONVERTED_LEN, "N/A");
		}
	}
	//ctimer_set(&RGB_light_timer, next, get_RGB_reading, NULL);
}
#endif


#ifdef NODE_STEP_MOTOR
static void
get_step_reading(void)
{
	int value;
	char *buf;
	//clock_time_t next =  web_demo_config.mqtt_config.pub_interval;

	if(step_motor_reading.publish) {

		value = step_sensor.value(SENSOR_ERROR);
		if(value != SENSOR_ERROR) {
			step_motor_reading.raw = value;

			buf = step_motor_reading.converted;
			memset(buf, 0, CC26XX_WEB_DEMO_CONVERTED_LEN);

			snprintf(buf, CC26XX_WEB_DEMO_CONVERTED_LEN, "%d",value);
		}
		else
		{
			snprintf(buf, CC26XX_WEB_DEMO_CONVERTED_LEN, "N/A");
		}
	}
	//ctimer_set(&Step_motor_timer, next, get_step_reading, NULL);
}
#endif

#if defined(NODE_STEP_MOTOR) || defined(NODE_4_ch_relay)
static void
init_dht11_temperature_reading(void *not_used)
{
  SENSORS_ACTIVATE(dht11_sensor);
}
static void
get_dht11_temperature_reading(void)
{
	int value;
	char *buf;
	clock_time_t next =  web_demo_config.mqtt_config.pub_interval;

	if(dht11_temperature_reading.publish) {

		value = dht11_sensor.value(TEMPERATURE_READING_SPLIT);
		buf = dht11_temperature_reading.converted;
		if(value != SENSOR_ERROR) {
			dht11_temperature_reading.raw = value;

			memset(buf, 0, CC26XX_WEB_DEMO_CONVERTED_LEN);

			snprintf(buf, CC26XX_WEB_DEMO_CONVERTED_LEN, "%d.%d",(value>>8),value&0xff);
		}
		else
		{
			snprintf(buf, CC26XX_WEB_DEMO_CONVERTED_LEN, "N/A");
		}
	}

	if(dht11_humidity_reading.publish) {

		value = dht11_sensor.value(HUMIDITY_READING_SPLIT);
		buf = dht11_humidity_reading.converted;
		if(value != SENSOR_ERROR) {
			dht11_humidity_reading.raw = value;


			memset(buf, 0, CC26XX_WEB_DEMO_CONVERTED_LEN);

			snprintf(buf, CC26XX_WEB_DEMO_CONVERTED_LEN, "%d.%d",(value>>8),value&0xff);
		}
		else
		{
			snprintf(buf, CC26XX_WEB_DEMO_CONVERTED_LEN, "N/A");
		}
	}
	SENSORS_DEACTIVATE(dht11_sensor);
	ctimer_set(&dht11_temperature_timer, next, init_dht11_temperature_reading, NULL);
}
#endif
/*---------------------------------------------------------------------------*/
#ifdef NODE_PREASURE
static void
init_bmp_reading(void *not_used)
{
  SENSORS_ACTIVATE(bmp_280_sensor);
}

static void
get_bmp_reading()
{
  int value;
  int low,high;
  clock_time_t next =  web_demo_config.mqtt_config.pub_interval;
  char *buf;

  if(bmp_280_sensor_press_reading.publish) {

	  value = bmp_280_sensor.value(BMP_280_SENSOR_TYPE_PRESS);
	  buf = bmp_280_sensor_press_reading.converted;
	  if(value != SENSOR_ERROR) {
		  bmp_280_sensor_press_reading.raw = value;

		memset(buf, 0, CC26XX_WEB_DEMO_CONVERTED_LEN);
		high = value/100;
		low = value-high*100;
		snprintf(buf, CC26XX_WEB_DEMO_CONVERTED_LEN, "%d.%d", high,low);
		/*snprintf(buf, CC26XX_WEB_DEMO_CONVERTED_LEN, "%d.%02d", value / 100,
		value % 100);*/
	  }
	  else
	  {
		snprintf(buf, CC26XX_WEB_DEMO_CONVERTED_LEN, "N/A");
	  }
  }

  if(bmp_280_sensor_temp_reading.publish) {

  	  value = bmp_280_sensor.value(BMP_280_SENSOR_TYPE_TEMP);
  	  buf = bmp_280_sensor_temp_reading.converted;
  	  if(value != SENSOR_ERROR) {
  		bmp_280_sensor_temp_reading.raw = value;

  		memset(buf, 0, CC26XX_WEB_DEMO_CONVERTED_LEN);
  		high = value/100;
		low = value-high*100;
		snprintf(buf, CC26XX_WEB_DEMO_CONVERTED_LEN, "%d.%d", high,low);
  		/*snprintf(buf, CC26XX_WEB_DEMO_CONVERTED_LEN, "%d.%02d", value / 100,
  		value % 100);*/
  	  }
  	  else
  	  {
  		snprintf(buf, CC26XX_WEB_DEMO_CONVERTED_LEN, "N/A");
  	  }
    }

  SENSORS_DEACTIVATE(bmp_280_sensor);

  ctimer_set(&bmp_timer, next, init_bmp_reading, NULL);
}
#endif

#ifdef NODE_HTU21D
static void
init_HTU21D_reading(void *not_used)
{
  SENSORS_ACTIVATE(HTU21D_sensor);
}

static void
get_HTU21D_reading()
{
  int value;
  int low,high;
  clock_time_t next =  web_demo_config.mqtt_config.pub_interval;
  char *buf;

  if(HTU21D_sensor_humid_reading.publish) {

	  value = HTU21D_sensor.value(HTU21D_SENSOR_TYPE_HUMID);
	  buf = HTU21D_sensor_humid_reading.converted;
	  if(value != SENSOR_ERROR) {
		  HTU21D_sensor_humid_reading.raw = value;

		memset(buf, 0, CC26XX_WEB_DEMO_CONVERTED_LEN);
		high = value/1000;
		low = value-high*1000;
		snprintf(buf, CC26XX_WEB_DEMO_CONVERTED_LEN, "%d.%d", high,low);
		/*snprintf(buf, CC26XX_WEB_DEMO_CONVERTED_LEN, "%d.%02d", value / 100,
		value % 100);*/
	  }
	  else
	  {
		snprintf(buf, CC26XX_WEB_DEMO_CONVERTED_LEN, "N/A");
	  }
  }

  if(HTU21D_sensor_temp_reading.publish) {

  	  value = HTU21D_sensor.value(HTU21D_SENSOR_TYPE_TEMP);
  	  buf = HTU21D_sensor_temp_reading.converted;
  	  if(value != SENSOR_ERROR) {
  		HTU21D_sensor_temp_reading.raw = value;

  		memset(buf, 0, CC26XX_WEB_DEMO_CONVERTED_LEN);
  		high = value/1000;
		low = value-high*1000;
		snprintf(buf, CC26XX_WEB_DEMO_CONVERTED_LEN, "%d.%d", high,low);
  		/*snprintf(buf, CC26XX_WEB_DEMO_CONVERTED_LEN, "%d.%02d", value / 100,
  		value % 100);*/
  	  }
  	  else
  	  {
  		snprintf(buf, CC26XX_WEB_DEMO_CONVERTED_LEN, "N/A");
  	  }
    }

  SENSORS_DEACTIVATE(HTU21D_sensor);

  ctimer_set(&HTU21D_timer, next, init_HTU21D_reading, NULL);
}
#endif
/*---------------------------------------------------------------------------*/


/*---------------------------------------------------------------------------*/
static void
init_sensors(void)
{
	SENSORS_ACTIVATE(SAM4S_ADC_TS_sensor);
	list_add(sensor_list, &temp_reading);
	snprintf(temp_reading.converted, CC26XX_WEB_DEMO_CONVERTED_LEN, "N/A");

#ifdef NODE_STEP_MOTOR
	SENSORS_ACTIVATE(step_sensor);
	list_add(sensor_list, &step_motor_reading);
	snprintf(step_motor_reading.converted, CC26XX_WEB_DEMO_CONVERTED_LEN, "N/A");

	SENSORS_ACTIVATE(dht11_sensor);
	list_add(sensor_list, &dht11_temperature_reading);
	list_add(sensor_list, &dht11_humidity_reading);
	snprintf(dht11_temperature_reading.converted, CC26XX_WEB_DEMO_CONVERTED_LEN, "N/A");
	snprintf(dht11_humidity_reading.converted, CC26XX_WEB_DEMO_CONVERTED_LEN, "N/A");
#endif

#ifdef NODE_LIGHT
	SENSORS_ACTIVATE(RGB_sensor);
	snprintf(RGB_sensor_reading.converted, CC26XX_WEB_DEMO_CONVERTED_LEN, "N/A");
#endif

#ifdef NODE_HARD_LIGHT
	SENSORS_ACTIVATE(hard_RGB_sensor);
	snprintf(hard_RGB_sensor_reading.converted, CC26XX_WEB_DEMO_CONVERTED_LEN, "N/A");
#endif

#ifdef NODE_4_ch_relay
	SENSORS_ACTIVATE(dht11_sensor);
	list_add(sensor_list, &dht11_temperature_reading);
	list_add(sensor_list, &dht11_humidity_reading);
	snprintf(dht11_temperature_reading.converted, CC26XX_WEB_DEMO_CONVERTED_LEN, "N/A");
	snprintf(dht11_humidity_reading.converted, CC26XX_WEB_DEMO_CONVERTED_LEN, "N/A");

	SENSORS_ACTIVATE(ch4_relay_PD956);
#endif

#ifdef NODE_PREASURE
	SENSORS_ACTIVATE(bmp_280_sensor);
	list_add(sensor_list, &bmp_280_sensor_press_reading);
	list_add(sensor_list, &bmp_280_sensor_temp_reading);
	snprintf(bmp_280_sensor_press_reading.converted, CC26XX_WEB_DEMO_CONVERTED_LEN, "N/A");
	snprintf(bmp_280_sensor_temp_reading.converted, CC26XX_WEB_DEMO_CONVERTED_LEN, "N/A");
#endif

#ifdef NODE_HTU21D
	SENSORS_ACTIVATE(HTU21D_sensor);
	list_add(sensor_list, &HTU21D_sensor_humid_reading);
	list_add(sensor_list, &HTU21D_sensor_temp_reading);
	snprintf(HTU21D_sensor_humid_reading.converted, CC26XX_WEB_DEMO_CONVERTED_LEN, "N/A");
	snprintf(HTU21D_sensor_temp_reading.converted, CC26XX_WEB_DEMO_CONVERTED_LEN, "N/A");
#endif
}
/*---------------------------------------------------------------------------*/
PROCESS_THREAD(cc26xx_web_demo_process, ev, data)
{
  PROCESS_BEGIN();

  printf("CC26XX Web Demo Process\n");

  init_sensors();

  cc26xx_web_demo_publish_event = process_alloc_event();
  cc26xx_web_demo_config_loaded_event = process_alloc_event();
  cc26xx_web_demo_load_config_defaults = process_alloc_event();

  /* Start all other (enabled) processes first */
  process_start(&httpd_simple_process, NULL);

#if CC26XX_WEB_DEMO_COAP_SERVER
  process_start(&coap_server_process, NULL);
#endif

#if CC26XX_WEB_DEMO_MQTT_CLIENT
  process_start(&mqtt_client_process, NULL);
#endif





  /*
   * Now that processes have set their own config default values, set our
   * own defaults and restore saved config from flash...
   */
  web_demo_config.sensors_bitmap = 0xFFFFFFFF; /* all on by default */
  web_demo_config.def_rt_ping_interval = CC26XX_WEB_DEMO_DEFAULT_RSSI_MEAS_INTERVAL;
  load_config();

  /*
   * Notify all other processes (basically the ones in this demo) that the
   * configuration has been loaded from flash, in case they care
   */
  process_post(PROCESS_BROADCAST, cc26xx_web_demo_config_loaded_event, NULL);

  httpd_simple_register_post_handler(&sensor_handler);
  httpd_simple_register_post_handler(&defaults_handler);

#if CC26XX_WEB_DEMO_READ_PARENT_RSSI
  httpd_simple_register_post_handler(&ping_interval_handler);

  def_rt_rssi = 0x8000000;
  uip_icmp6_echo_reply_callback_add(&echo_reply_notification,
                                    echo_reply_handler);
  etimer_set(&echo_request_timer, CC26XX_WEB_DEMO_NET_CONNECT_PERIODIC);
#endif

  etimer_set(&et, CC26XX_WEB_DEMO_NET_CONNECT_PERIODIC);

  /*
   * Update all sensor readings on a configurable sensors_event
   * (e.g a button press / or reed trigger)
   */
  while(1) {
    if(ev == PROCESS_EVENT_TIMER && etimer_expired(&et)) {
      if(uip_ds6_get_global(ADDR_PREFERRED) == NULL) {
        //leds_on(CC26XX_WEB_DEMO_STATUS_LED);
        //ctimer_set(&ct, NO_NET_LED_DURATION, publish_led_off, NULL);
        etimer_set(&et, CC26XX_WEB_DEMO_NET_CONNECT_PERIODIC);
      }
    }

#if CC26XX_WEB_DEMO_READ_PARENT_RSSI
    if(ev == PROCESS_EVENT_TIMER && etimer_expired(&echo_request_timer)) {
      if(uip_ds6_get_global(ADDR_PREFERRED) == NULL) {
        etimer_set(&echo_request_timer, CC26XX_WEB_DEMO_NET_CONNECT_PERIODIC);
      } else {
        ping_parent();
        etimer_set(&echo_request_timer, web_demo_config.def_rt_ping_interval);
      }
    }
#endif

    /*if(ev == sensors_event && data == CC26XX_WEB_DEMO_SENSOR_READING_TRIGGER) {
      if((CC26XX_WEB_DEMO_SENSOR_READING_TRIGGER)->value(
           BUTTON_SENSOR_VALUE_DURATION) > CLOCK_SECOND * 5) {
        printf("Restoring defaults!\n");
        cc26xx_web_demo_restore_defaults();
      } else {
        init_sensor_readings();

        process_post(PROCESS_BROADCAST, cc26xx_web_demo_publish_event, NULL);
      }
    } else*/ if(ev == httpd_simple_event_new_config) {
      save_config();
    } else if(ev == sensors_event && data == &SAM4S_ADC_TS_sensor) {
          get_temp_reading();

#ifdef NODE_STEP_MOTOR
    } else if(ev == sensors_event && data == &step_sensor) {
          get_step_reading();
#endif

#ifdef NODE_LIGHT
    } else if(ev == sensors_event && data == &RGB_sensor) {
    	get_RGB_reading();
#endif

#if defined(NODE_STEP_MOTOR) || defined(NODE_4_ch_relay)
    } else if(ev == sensors_event && data == &dht11_sensor) {
              get_dht11_temperature_reading();
#endif

#ifdef NODE_HTU21D
    } else if(ev == sensors_event && data == &HTU21D_sensor) {
              get_HTU21D_reading();
#endif

#ifdef NODE_PREASURE
    } else if(ev == sensors_event && data == &bmp_280_sensor) {
              get_bmp_reading();
#endif

#if BOARD_SENSORTAG
    } else if(ev == sensors_event && data == &bmp_280_sensor) {
      get_bmp_reading();
    } else if(ev == sensors_event && data == &opt_3001_sensor) {
      get_light_reading();
    } else if(ev == sensors_event && data == &hdc_1000_sensor) {
      get_hdc_reading();
    } else if(ev == sensors_event && data == &tmp_007_sensor) {
      get_tmp_reading();
    } else if(ev == sensors_event && data == &mpu_9250_sensor) {
      get_mpu_reading();
#endif
    }

    PROCESS_YIELD();
  }

  PROCESS_END();
}
/*---------------------------------------------------------------------------*/

/**
 * @}
 */
