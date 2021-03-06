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
 * \addtogroup cc26xx-examples
 * @{
 *
 * \defgroup PD956 MQTT
 * @{
 *
 *   An example demonstrating:
 *   * how to use a CC26XX-powered node in a deployment driven by a 6LBR
 *   * how to expose on-device sensors as CoAP resources
 *   * how to build a small web page which reports networking and sensory data
 *   * how to configure functionality through the aforementioned web page using
 *     HTTP POST requests
 *   * a network-based UART
 *
 * \file
 *   Main header file for the CC26XX web demo.
 */
/*---------------------------------------------------------------------------*/
#ifndef PD956_SENSOR_LOW_POWER_H
#define PD956_SENSOR_LOW_POWER_H
/*---------------------------------------------------------------------------*/
#include "dev/leds.h"
#include "sys/process.h"
#include "mqtt-client.h"

#include <stdint.h>

/*---------------------------------------------------------------------------*/

#define CC26XX_WEB_DEMO_RSSI_MEASURE_INTERVAL_MAX 86400 /* secs: 1 day */
#define CC26XX_WEB_DEMO_RSSI_MEASURE_INTERVAL_MIN     5 /* secs */
/*---------------------------------------------------------------------------*/
/* User configuration */

/* Payload length of ICMPv6 echo requests used to measure RSSI with def rt */
#define ECHO_REQ_PAYLOAD_LEN   20

/*---------------------------------------------------------------------------*/
/* A timeout used when waiting to connect to a network */
#define NET_CONNECT_PERIODIC        (CLOCK_SECOND >> 3)
/*---------------------------------------------------------------------------*/
/* Default configuration values */
#define DEFAULT_ORG_ID              "PD"

#define DEFAULT_TYPE_ID             "956"

#define DEFAULT_EVENT_TYPE_ID       "Unknown"
#define DEFAULT_BROKER_PORT         1883
#define DEFAULT_PUBLISH_INTERVAL    (30 * CLOCK_SECOND)
#define DEFAULT_RSSI_MEAS_INTERVAL  (CLOCK_SECOND * 30)
/*---------------------------------------------------------------------------*/
/*
 * You normally won't have to change anything from here onwards unless you are
 * extending the example
 */
/*---------------------------------------------------------------------------*/
/* Sensor types */
#define PD956_WEB_DEMO_SENSOR_ADC     				20
#define PD956_WEB_DEMO_SENSOR_STEP     				21
#define PD956_WEB_DEMO_SENSOR_DHT11_TEMP  			22
#define PD956_WEB_DEMO_SENSOR_DHT11_HUMIDITY  		23
#define PD956_WEB_DEMO_SENSOR_BMP280_PRES  			24
#define PD956_WEB_DEMO_SENSOR_BMP280_TEMP  			25
#define PD956_WEB_DEMO_SENSOR_RGB  					26
#define PD956_WEB_DEMO_SENSOR_HTU21D_humid  		27
#define PD956_WEB_DEMO_SENSOR_HTU21D_TEMP  			28

#define PD956_WEB_DEMO_SENSOR_GPS_LONG  			1
#define PD956_WEB_DEMO_SENSOR_GPS_LAT	  			2
#define PD956_WEB_DEMO_SENSOR_GPS_ALT	  			3
#define PD956_WEB_DEMO_SENSOR_GPS_SPEED  			4

#define PD956_WEB_DEMO_SENSOR_RELAY1	  			5
#define PD956_WEB_DEMO_SENSOR_RELAY2	  			6
#define PD956_WEB_DEMO_SENSOR_RELAY3	  			7
#define PD956_WEB_DEMO_SENSOR_RELAY4	  			8

#define PD956_WEB_DEMO_SENSOR_PIR	  				9

#define PD956_WEB_DEMO_SENSOR_CHRISTMAS_LIGHT		10
/*---------------------------------------------------------------------------*/
extern process_event_t MQTT_publish_sensor_data_event;
extern process_event_t config_loaded_event;
extern process_event_t load_config_defaults;
extern process_event_t Trig_sensors;
/*---------------------------------------------------------------------------*/
#define UNIT_TEMP		"°C"
#define UNIT_VOLT		"V"
#define UNIT_PRES		"hPa"
#define UNIT_HUMIDITY	"%RH"
#define UNIT_LIGHT		"lux"
#define UNIT_ACC		"G"
#define UNIT_GYRO		"deg per sec"
#define UNIT_STEP		"ticks"
#define UNIT_NONE		""
#define UNIT_SPEED		"km/hr"
#define UNIT_ANGLE		"deg"
#define UNIT_DISTANCE	"m"
/*---------------------------------------------------------------------------*/

typedef struct MQTT_config_ele
{
	struct MQTT_config_ele *next;
	char topic[64];
	char arg[64 * 14];
} MQTT_config_ele_t;

typedef struct MQTT_sub_ele
{
	struct MQTT_sub_ele *next;
	char topic[64];
	void(* data_handler)(uint8_t *,uint16_t);
} MQTT_sub_ele_t;
/*---------------------------------------------------------------------------*/

typedef enum{
	sensor_class,
	cover_class,
	switch_class,
	binary_sensor_class,
	light_class
}HASS_class_t;

/* A data type for sensor readings, internally stored in a linked list */
#define SENSOR_CONVERTED_LEN        30

typedef struct sensor_reading {
  struct sensor_reading *next;
  int raw;
  int last;
  const char *descr;
  const char *form_field;
  char *units;
  uint8_t type;
  HASS_class_t HASS_class;
  uint8_t publish;
  uint8_t changed;
  char converted[SENSOR_CONVERTED_LEN];
  /* can be:
   * 	- binary_sensor
   * 	- cover
   * 	- fan
   * 	- light
   * 	- sensor
   * 	- switch
   */
  //char *component_type_config;
  char *component_topic_config;
  char *component_topic_sub;
  MQTT_config_ele_t MQTT_config_ele;
  MQTT_sub_ele_t MQTT_subscr_ele;
  DEVICE_CLASS_t device_class;
} MQTT_sensor_reading_t;
/*---------------------------------------------------------------------------*/
/* Global configuration */
typedef struct web_config_s {
  uint32_t magic;
  int len;
  char sensor_name_str[20];
  uint32_t sensors_bitmap;
  mqtt_client_config_t mqtt_config;
 //
} Device_config_t;

extern Device_config_t web_demo_config;
/*---------------------------------------------------------------------------*/

/**
 * \brief Returns the first available sensor reading
 * \return A pointer to the reading data structure or NULL
 */
MQTT_sensor_reading_t *MQTT_sensor_first(void);

/**
 * \brief Print an IPv6 address into a buffer
 * \param buf A pointer to the buffer where this function will print the IPv6
 *        address
 * \param buf_len the length of the buffer
 * \param addr A pointer to the IPv6 address
 * \return The number of bytes written to the buffer
 *
 * It is the caller's responsibility to allocate enough space for buf
 */
int ipaddr_sprintf(char *buf, uint8_t buf_len,
                                   const uip_ipaddr_t *addr);

/**
 * \brief Resets the example to a default configuration
 */
void Restore_defaults(void);
/*---------------------------------------------------------------------------*/
#endif /* PD956_SENSOR_LOW_POWER_H */
/*---------------------------------------------------------------------------*/
/**
 * @}
 * @}
 */
