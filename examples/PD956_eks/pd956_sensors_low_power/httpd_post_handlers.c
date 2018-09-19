/*
 * httpd_post_handlers.c
 *
 *  Created on: 03/04/2018
 *      Author: pm
 */




#include "contiki-conf.h"
#include "rpl/rpl-private.h"
#include "net/rpl/rpl.h"
#include "net/ip/uip.h"
#include "net/ipv6/uip-icmp6.h"
#include "sys/etimer.h"
#include "sys/ctimer.h"
#include "lib/sensors.h"
#include "board-peripherals.h"
#include "pd956_sensor_low_power.h"
#include "httpd-simple.h"
#include "mqtt-client.h"
#include "clock.h"

#include <string.h>
#include <strings.h>
#include <stdio.h>
#include <stdlib.h>

#define ADDRESS_CONVERSION_OK       1
#define ADDRESS_CONVERSION_ERROR    0

extern void
new_net_config(void);

#ifdef NODE_LIGHT
static int
RGB_blue_post_handler(char *key, int key_len, char *val, int val_len)
{
  int ret = HTTPD_SIMPLE_POST_HANDLER_UNKNOWN;
  RGB_soft_t tmp;
  if(key_len != strlen("RGB_blue") ||
     strncasecmp(key, "RGB_blue", strlen("RGB_blue")) != 0) {
    /* Not ours */
    return HTTPD_SIMPLE_POST_HANDLER_UNKNOWN;
  }

  ret = atoi(val);

   if(ret < 0 ||
      ret > 256) {
     return HTTPD_SIMPLE_POST_HANDLER_ERROR;
   }

   tmp.all = soft_RGB_ctrl_sensor.value(SENSOR_ERROR);
	tmp.led.b = ret;
	soft_RGB_ctrl_sensor.value(tmp.all);

   return HTTPD_SIMPLE_POST_HANDLER_OK;
}

/*---------------------------------------------------------------------------*/

static int
RGB_green_post_handler(char *key, int key_len, char *val, int val_len)
{
  int ret = HTTPD_SIMPLE_POST_HANDLER_UNKNOWN;
  RGB_soft_t tmp;
  if(key_len != strlen("RGB_green") ||
     strncasecmp(key, "RGB_green", strlen("RGB_green")) != 0) {
    /* Not ours */
    return HTTPD_SIMPLE_POST_HANDLER_UNKNOWN;
  }

  ret = atoi(val);

   if(ret < 0 ||
      ret > 256) {
     return HTTPD_SIMPLE_POST_HANDLER_ERROR;
   }

   tmp.all = soft_RGB_ctrl_sensor.value(SENSOR_ERROR);
  tmp.led.g = ret;
  soft_RGB_ctrl_sensor.value(tmp.all);

   return HTTPD_SIMPLE_POST_HANDLER_OK;
}

/*---------------------------------------------------------------------------*/
static int
RGB_red_post_handler(char *key, int key_len, char *val, int val_len)
{
  int ret = HTTPD_SIMPLE_POST_HANDLER_UNKNOWN;
  RGB_soft_t tmp;
  if(key_len != strlen("RGB_red") ||
     strncasecmp(key, "RGB_red", strlen("RGB_red")) != 0) {
    /* Not ours */
    return HTTPD_SIMPLE_POST_HANDLER_UNKNOWN;
  }

  ret = atoi(val);

   if(ret < 0 ||
      ret > 256) {
     return HTTPD_SIMPLE_POST_HANDLER_ERROR;
   }

   tmp.all = soft_RGB_ctrl_sensor.value(SENSOR_ERROR);
   tmp.led.r = ret;
   soft_RGB_ctrl_sensor.value(tmp.all);

   return HTTPD_SIMPLE_POST_HANDLER_OK;
}

/*---------------------------------------------------------------------------*/
static int
RGB_brightness_post_handler(char *key, int key_len, char *val, int val_len)
{
  int ret = HTTPD_SIMPLE_POST_HANDLER_UNKNOWN;
  RGB_soft_t tmp;
  if(key_len != strlen("RGB_brightness") ||
     strncasecmp(key, "RGB_brightness", strlen("RGB_brightness")) != 0) {
    /* Not ours */
    return HTTPD_SIMPLE_POST_HANDLER_UNKNOWN;
  }

  ret = atoi(val);

   if(ret < 0 ||
      ret > 256) {
     return HTTPD_SIMPLE_POST_HANDLER_ERROR;
   }

   tmp.all = soft_RGB_ctrl_sensor.value(SENSOR_ERROR);
   tmp.led.brightness = ret;
   soft_RGB_ctrl_sensor.value(tmp.all);

   return HTTPD_SIMPLE_POST_HANDLER_OK;
}
#endif

#ifdef NODE_HARD_LIGHT
static int
RGB_blue_post_handler(char *key, int key_len, char *val, int val_len)
{
  int ret = HTTPD_SIMPLE_POST_HANDLER_UNKNOWN;
  RGB_hard_t tmp;
  if(key_len != strlen("RGB_blue") ||
     strncasecmp(key, "RGB_blue", strlen("RGB_blue")) != 0) {
    /* Not ours */
    return HTTPD_SIMPLE_POST_HANDLER_UNKNOWN;
  }

  ret = atoi(val);

   if(ret < 0 ||
      ret > 4096) {
     return HTTPD_SIMPLE_POST_HANDLER_ERROR;
   }

   tmp.all = ((RGB_hard_t *) hard_RGB_ctrl_sensor.value(SENSOR_ERROR))->all;
	tmp.led.b = ret;
	hard_RGB_ctrl_sensor.value((int)&tmp);

   return HTTPD_SIMPLE_POST_HANDLER_OK;
}

/*---------------------------------------------------------------------------*/

static int
RGB_green_post_handler(char *key, int key_len, char *val, int val_len)
{
  int ret = HTTPD_SIMPLE_POST_HANDLER_UNKNOWN;
  RGB_hard_t tmp;
  if(key_len != strlen("RGB_green") ||
     strncasecmp(key, "RGB_green", strlen("RGB_green")) != 0) {
    /* Not ours */
    return HTTPD_SIMPLE_POST_HANDLER_UNKNOWN;
  }

  ret = atoi(val);

   if(ret < 0 ||
      ret > 4096) {
     return HTTPD_SIMPLE_POST_HANDLER_ERROR;
   }

   tmp.all = ((RGB_hard_t *) hard_RGB_ctrl_sensor.value(SENSOR_ERROR))->all;
  tmp.led.g = ret;
  hard_RGB_ctrl_sensor.value((int)&tmp);

   return HTTPD_SIMPLE_POST_HANDLER_OK;
}

/*---------------------------------------------------------------------------*/
static int
RGB_red_post_handler(char *key, int key_len, char *val, int val_len)
{
  int ret = HTTPD_SIMPLE_POST_HANDLER_UNKNOWN;
  RGB_hard_t tmp;
  if(key_len != strlen("RGB_red") ||
     strncasecmp(key, "RGB_red", strlen("RGB_red")) != 0) {
    /* Not ours */
    return HTTPD_SIMPLE_POST_HANDLER_UNKNOWN;
  }

  ret = atoi(val);

   if(ret < 0 ||
      ret > 4096) {
     return HTTPD_SIMPLE_POST_HANDLER_ERROR;
   }

   tmp.all = ((RGB_hard_t *) hard_RGB_ctrl_sensor.value(SENSOR_ERROR))->all;
   tmp.led.r = ret;
   hard_RGB_ctrl_sensor.value((int)&tmp);

   return HTTPD_SIMPLE_POST_HANDLER_OK;
}

/*---------------------------------------------------------------------------*/
static int
RGB_brightness_post_handler(char *key, int key_len, char *val, int val_len)
{
  int ret = HTTPD_SIMPLE_POST_HANDLER_UNKNOWN;
  RGB_hard_t tmp;
  if(key_len != strlen("RGB_brightness") ||
     strncasecmp(key, "RGB_brightness", strlen("RGB_brightness")) != 0) {
    /* Not ours */
    return HTTPD_SIMPLE_POST_HANDLER_UNKNOWN;
  }

  ret = atoi(val);

   if(ret < 0 ||
      ret > 256) {
     return HTTPD_SIMPLE_POST_HANDLER_ERROR;
   }

   tmp.all = ((RGB_hard_t *) hard_RGB_ctrl_sensor.value(SENSOR_ERROR))->all;
   tmp.led.brightness = ret;
   hard_RGB_ctrl_sensor.value((int)&tmp);

   return HTTPD_SIMPLE_POST_HANDLER_OK;
}
#endif
/*---------------------------------------------------------------------------*/
#ifdef NODE_STEP_MOTOR
static int
step_motor_position_post_handler(char *key, int key_len, char *val, int val_len)
{
  int ret = HTTPD_SIMPLE_POST_HANDLER_UNKNOWN;
  if(key_len != strlen("Step_motor_position") ||
     strncasecmp(key, "Step_motor_position", strlen("Step_motor_position")) != 0) {
    /* Not ours */
    return HTTPD_SIMPLE_POST_HANDLER_UNKNOWN;
  }

  ret = atoi(val);

   if(ret < -2147483647 ||
      ret > 2147483647) {
     return HTTPD_SIMPLE_POST_HANDLER_ERROR;
   }

   step_sensor.value(ret);
   step_sensor.configure(SENSORS_ACTIVE, 1);

   return HTTPD_SIMPLE_POST_HANDLER_OK;
}
#endif

#ifdef NODE_4_ch_relay
static int
Relay_post_handler(char *key, int key_len, char *val, int val_len)
{
	int ret = HTTPD_SIMPLE_POST_HANDLER_UNKNOWN;
	uint8_t tmp;
	if(key_len != strlen("relayx") ||
			strncasecmp(key, "relay", strlen("relay")) != 0) {
		/* Not ours */
		return HTTPD_SIMPLE_POST_HANDLER_UNKNOWN;
	}

	ret = atoi(val);
	tmp = *(key+5)-0x30;
	if(ret == 0 ){
		ch4_relay_PD956.value((tmp<<1)+15);
	}
	else if(ret == 1){
		ch4_relay_PD956.value((tmp<<1)+14);
	}
	else{
		return HTTPD_SIMPLE_POST_HANDLER_ERROR;
	}

	return HTTPD_SIMPLE_POST_HANDLER_OK;
}
#endif
/*---------------------------------------------------------------------------*/
static int
org_id_post_handler(char *key, int key_len, char *val, int val_len)
{
  int ret = HTTPD_SIMPLE_POST_HANDLER_UNKNOWN;
  if(key_len != strlen("Company") ||
     strncasecmp(key, "Company", strlen("Company")) != 0) {
    /* Not ours */
    return HTTPD_SIMPLE_POST_HANDLER_UNKNOWN;
  }

  if(val_len > MQTT_CLIENT_CONFIG_ORG_ID_LEN) {
    /* Ours but bad value */
    ret = HTTPD_SIMPLE_POST_HANDLER_ERROR;
  } else {
    memset(conf->Company, 0, MQTT_CLIENT_CONFIG_ORG_ID_LEN);
    memcpy(conf->Company, val, val_len);

    ret = HTTPD_SIMPLE_POST_HANDLER_OK;
  }

  new_net_config();

  return ret;
}
/*---------------------------------------------------------------------------*/
static int
type_id_post_handler(char *key, int key_len, char *val, int val_len)
{
  int ret = HTTPD_SIMPLE_POST_HANDLER_UNKNOWN;
  if(key_len != strlen("Modul_type") ||
     strncasecmp(key, "Modul_type", strlen("Modul_type")) != 0) {
    /* Not ours */
    return HTTPD_SIMPLE_POST_HANDLER_UNKNOWN;
  }

  if(val_len > MQTT_CLIENT_CONFIG_TYPE_ID_LEN) {
    /* Ours but bad value */
    ret = HTTPD_SIMPLE_POST_HANDLER_ERROR;
  } else {
    memset(conf->Modul_type, 0, MQTT_CLIENT_CONFIG_TYPE_ID_LEN);
    memcpy(conf->Modul_type, val, val_len);

    ret = HTTPD_SIMPLE_POST_HANDLER_OK;
  }

  new_net_config();

  return ret;
}
/*---------------------------------------------------------------------------*/
static int
event_type_id_post_handler(char *key, int key_len, char *val, int val_len)
{
  int ret = HTTPD_SIMPLE_POST_HANDLER_UNKNOWN;
  if(key_len != strlen("Username") ||
     strncasecmp(key, "Username", strlen("Username")) != 0) {
    /* Not ours */
    return HTTPD_SIMPLE_POST_HANDLER_UNKNOWN;
  }

  if(val_len > MQTT_CLIENT_CONFIG_EVENT_TYPE_ID_LEN) {
    /* Ours but bad value */
    ret = HTTPD_SIMPLE_POST_HANDLER_ERROR;
  } else {
    memset(conf->Username, 0, MQTT_CLIENT_CONFIG_EVENT_TYPE_ID_LEN);
    memcpy(conf->Username, val, val_len);

    ret = HTTPD_SIMPLE_POST_HANDLER_OK;
  }

  new_net_config();

  return ret;
}

/*---------------------------------------------------------------------------*/
static int
MQTT_user_name_post_handler(char *key, int key_len, char *val, int val_len)
{
  int ret = HTTPD_SIMPLE_POST_HANDLER_UNKNOWN;
  if(key_len != strlen("MQTTusername") ||
     strncasecmp(key, "MQTTusername", strlen("MQTTusername")) != 0) {
    /* Not ours */
    return HTTPD_SIMPLE_POST_HANDLER_UNKNOWN;
  }

  if(val_len > MQTT_CLIENT_CONFIG_USER_NAME_LEN) {
    /* Ours but bad value */
    ret = HTTPD_SIMPLE_POST_HANDLER_ERROR;
  } else {
    memset(conf->MQTT_user_name, 0, MQTT_CLIENT_CONFIG_USER_NAME_LEN);
    memcpy(conf->MQTT_user_name, val, val_len);

    ret = HTTPD_SIMPLE_POST_HANDLER_OK;
  }

  new_net_config();

  return ret;
}
/*---------------------------------------------------------------------------*/
static int
auth_token_post_handler(char *key, int key_len, char *val, int val_len)
{
  int ret = HTTPD_SIMPLE_POST_HANDLER_UNKNOWN;
  if(key_len != strlen("Password") ||
     strncasecmp(key, "Password", strlen("Password")) != 0) {
    /* Not ours */
    return HTTPD_SIMPLE_POST_HANDLER_UNKNOWN;
  }

  if(val_len > MQTT_CLIENT_CONFIG_AUTH_TOKEN_LEN) {
    /* Ours but bad value */
    ret = HTTPD_SIMPLE_POST_HANDLER_ERROR;
  } else {
    memset(conf->MQTT_user_Password, 0, MQTT_CLIENT_CONFIG_AUTH_TOKEN_LEN);
    memcpy(conf->MQTT_user_Password, val, val_len);

    ret = HTTPD_SIMPLE_POST_HANDLER_OK;
  }

  new_net_config();

  return ret;
}
/*---------------------------------------------------------------------------*/
static int
interval_post_handler(char *key, int key_len, char *val, int val_len)
{
  int ret = 0;

  if(key_len != strlen("interval") ||
     strncasecmp(key, "interval", strlen("interval")) != 0) {
    /* Not ours */
    return HTTPD_SIMPLE_POST_HANDLER_UNKNOWN;
  }

  ret = atoi(val);

  if(ret < MQTT_CLIENT_PUBLISH_INTERVAL_MIN ||
     ret > MQTT_CLIENT_PUBLISH_INTERVAL_MAX) {
    return HTTPD_SIMPLE_POST_HANDLER_ERROR;
  }

  conf->pub_interval = ret * CLOCK_SECOND;

  return HTTPD_SIMPLE_POST_HANDLER_OK;
}
/*---------------------------------------------------------------------------*/
static int
port_post_handler(char *key, int key_len, char *val, int val_len)
{
  int ret = 0;

  if(key_len != strlen("broker_port") ||
     strncasecmp(key, "broker_port", strlen("broker_port")) != 0) {
    /* Not ours */
    return HTTPD_SIMPLE_POST_HANDLER_UNKNOWN;
  }

  ret = atoi(val);

  if(ret <= 65535 && ret > 0) {
    conf->broker_port = ret;
  } else {
    return HTTPD_SIMPLE_POST_HANDLER_ERROR;
  }

  new_net_config();

  return HTTPD_SIMPLE_POST_HANDLER_OK;
}
/*---------------------------------------------------------------------------*/
static int
ip_addr_post_handler(char *key, int key_len, char *val, int val_len)
{
  int ret = HTTPD_SIMPLE_POST_HANDLER_UNKNOWN;

  /*
   * uiplib_ip6addrconv will immediately start writing into the supplied buffer
   * even if it subsequently fails. Thus, pass an intermediate buffer
   */
  uip_ip6addr_t tmp_addr;

  if(key_len != strlen("broker_ip") ||
     strncasecmp(key, "broker_ip", strlen("broker_ip")) != 0) {
    /* Not ours */
    return HTTPD_SIMPLE_POST_HANDLER_UNKNOWN;
  }

  if(val_len > MQTT_CLIENT_CONFIG_IP_ADDR_STR_LEN
          || uiplib_ip6addrconv(val, &tmp_addr) != ADDRESS_CONVERSION_OK) {
    /* Ours but bad value */
    ret = HTTPD_SIMPLE_POST_HANDLER_ERROR;
  } else {
    memset(conf->broker_ip, 0, MQTT_CLIENT_CONFIG_IP_ADDR_STR_LEN);
    memcpy(conf->broker_ip, val, val_len);

    ret = HTTPD_SIMPLE_POST_HANDLER_OK;
  }

  new_net_config();

  return ret;
}
/*---------------------------------------------------------------------------*/
static int
reconnect_post_handler(char *key, int key_len, char *val, int val_len)
{
  if(key_len != strlen("reconnect") ||
     strncasecmp(key, "reconnect", strlen("reconnect")) != 0) {
    /* Not ours */
    return HTTPD_SIMPLE_POST_HANDLER_UNKNOWN;
  }

  new_net_config();

  return HTTPD_SIMPLE_POST_HANDLER_OK;
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

  Restore_defaults();

  return HTTPD_SIMPLE_POST_HANDLER_OK;
}
/*---------------------------------------------------------------------------*/
static int
timestamp_post_handler(char *key, int key_len, char *val, int val_len)
{
  clock_time_t unixtime;
  char *endptr;

  if(key_len != strlen("Timestamp") ||
     strncasecmp(key, "Timestamp", strlen("Timestamp")) != 0) {
    /* Not ours */
    return HTTPD_SIMPLE_POST_HANDLER_UNKNOWN;
  }

  unixtime = strtoul(val,&endptr,10);

   if(unixtime < 1514764800) { //Monday, 01/01-2018 00:00:00 UTC
     return HTTPD_SIMPLE_POST_HANDLER_ERROR;
   }

   //always set time
  //if(clock_quality(READ_STRANUM) > PC_TIME){
	clock_set_unix_time(unixtime,1);
	clock_quality(PC_TIME);
  //}

   return HTTPD_SIMPLE_POST_HANDLER_OK;
}
/*---------------------------------------------------------------------------*/
static int
timezone_post_handler(char *key, int key_len, char *val, int val_len)
{
  clock_time_t Timezone;
  char *endptr;

  if(key_len != strlen("Timezone") ||
     strncasecmp(key, "Timezone", strlen("Timezone")) != 0) {
    /* Not ours */
    return HTTPD_SIMPLE_POST_HANDLER_UNKNOWN;
  }

  Timezone = strtoul(val,&endptr,10);

  clock_set_unix_timezone(Timezone);

   return HTTPD_SIMPLE_POST_HANDLER_OK;
}
/*---------------------------------------------------------------------------*/

static int
sensor_readings_handler(char *key, int key_len, char *val, int val_len)
{
  MQTT_sensor_reading_t *reading = NULL;
  int ret;

  for(reading = MQTT_sensor_first();
      reading != NULL;
      reading = list_item_next(reading)) {
    if(key_len == strlen(reading->form_field) &&
       strncmp(reading->form_field, key, strlen(key)) == 0) {

      ret = atoi(val);

      /* Be pedantic: only accept 0 and 1, not just any non-zero value */
      if(ret == 0) {
        reading->publish = 0;
        snprintf(reading->converted, SENSOR_CONVERTED_LEN, "\"N/A\"");
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
#if defined(NODE_LIGHT) || defined(NODE_HARD_LIGHT)
HTTPD_SIMPLE_POST_HANDLER(RGB_blue,RGB_blue_post_handler);
HTTPD_SIMPLE_POST_HANDLER(RGB_green,RGB_green_post_handler);
HTTPD_SIMPLE_POST_HANDLER(RGB_red,RGB_red_post_handler);
HTTPD_SIMPLE_POST_HANDLER(RGB_brightness,RGB_brightness_post_handler);
#endif
#ifdef NODE_STEP_MOTOR
HTTPD_SIMPLE_POST_HANDLER(Step_motor_position,step_motor_position_post_handler);
#endif
#ifdef NODE_4_ch_relay
HTTPD_SIMPLE_POST_HANDLER(Relay,Relay_post_handler);
#endif
HTTPD_SIMPLE_POST_HANDLER(Company, org_id_post_handler);
HTTPD_SIMPLE_POST_HANDLER(Modul_type, type_id_post_handler);
HTTPD_SIMPLE_POST_HANDLER(Username, event_type_id_post_handler);
HTTPD_SIMPLE_POST_HANDLER(MQTT_user_name, MQTT_user_name_post_handler);
HTTPD_SIMPLE_POST_HANDLER(Password, auth_token_post_handler);
HTTPD_SIMPLE_POST_HANDLER(ip_addr, ip_addr_post_handler);
HTTPD_SIMPLE_POST_HANDLER(broker_port, port_post_handler);
HTTPD_SIMPLE_POST_HANDLER(interval, interval_post_handler);
HTTPD_SIMPLE_POST_HANDLER(reconnect, reconnect_post_handler);
/*---------------------------------------------------------------------------*/
HTTPD_SIMPLE_POST_HANDLER(sensor, sensor_readings_handler);
HTTPD_SIMPLE_POST_HANDLER(defaults, defaults_post_handler);
/*---------------------------------------------------------------------------*/
HTTPD_SIMPLE_POST_HANDLER(timestamp,timestamp_post_handler);
HTTPD_SIMPLE_POST_HANDLER(timezone,timezone_post_handler);


void
register_http_post_handlers(void)
{
#if defined(NODE_LIGHT) || defined(NODE_HARD_LIGHT)
	httpd_simple_register_post_handler(&RGB_blue_handler);
	httpd_simple_register_post_handler(&RGB_green_handler);
	httpd_simple_register_post_handler(&RGB_red_handler);
	httpd_simple_register_post_handler(&RGB_brightness_handler);
#endif
#ifdef NODE_STEP_MOTOR
	httpd_simple_register_post_handler(&Step_motor_position_handler);
#endif
#ifdef NODE_4_ch_relay
	httpd_simple_register_post_handler(&Relay_handler);
#endif
	httpd_simple_register_post_handler(&Company_handler);
	httpd_simple_register_post_handler(&Modul_type_handler);
	httpd_simple_register_post_handler(&Username_handler);
	httpd_simple_register_post_handler(&MQTT_user_name_handler);
	httpd_simple_register_post_handler(&Password_handler);
	httpd_simple_register_post_handler(&interval_handler);
	httpd_simple_register_post_handler(&broker_port_handler);
	httpd_simple_register_post_handler(&ip_addr_handler);
	httpd_simple_register_post_handler(&reconnect_handler);

	httpd_simple_register_post_handler(&sensor_handler);
	httpd_simple_register_post_handler(&defaults_handler);
	httpd_simple_register_post_handler(&timestamp_handler);
	httpd_simple_register_post_handler(&timezone_handler);
}
