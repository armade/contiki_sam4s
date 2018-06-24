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
 *   MQTT/IBM cloud service client for the CC26XX web demo.
 */
/*---------------------------------------------------------------------------*/
#include "contiki-conf.h"
#include "rpl/rpl-private.h"
#include "mqtt.h"
#include "net/rpl/rpl.h"
#include "net/ip/uip.h"
#include "net/ipv6/uip-icmp6.h"
#include "sys/etimer.h"
#include "sys/ctimer.h"
#include "lib/sensors.h"
#include "button-sensor.h"
#include "board-peripherals.h"
#include "pd956_sensor.h"
#include "dev/leds.h"
#include "mqtt-client.h"
#include "httpd-simple.h"

#include <string.h>
#include <strings.h>
/*---------------------------------------------------------------------------*/
/*
 * IBM server: messaging.quickstart.internetofthings.ibmcloud.com
 * (184.172.124.189) mapped in an NAT64 (prefix 64:ff9b::/96) IPv6 address
 * Note: If not able to connect; lookup the IP address again as it may change.
 *
 * If the node has a broker IP setting saved on flash, this value here will
 * get ignored
 */
static const char *broker_ip = "aaaa::1";
/*---------------------------------------------------------------------------*/
#define ADDRESS_CONVERSION_OK       1
#define ADDRESS_CONVERSION_ERROR    0
/*---------------------------------------------------------------------------*/
/*
 * A timeout used when waiting for something to happen (e.g. to connect or to
 * disconnect)
 */
#define STATE_MACHINE_PERIODIC     (CLOCK_SECOND >> 1)
/*---------------------------------------------------------------------------*/
/* Provide visible feedback via LEDS during various states */
/* When connecting to broker */
#define CONNECTING_LED_DURATION    (CLOCK_SECOND >> 3)

/* Each time we try to publish */
#define PUBLISH_LED_ON_DURATION    (CLOCK_SECOND)
/*---------------------------------------------------------------------------*/
/* Connections and reconnections */
#define RETRY_FOREVER              0xFF
#define RECONNECT_INTERVAL         (CLOCK_SECOND * 2)

/*
 * Number of times to try reconnecting to the broker.
 * Can be a limited number (e.g. 3, 10 etc) or can be set to RETRY_FOREVER
 */
#define RECONNECT_ATTEMPTS         RETRY_FOREVER
#define CONNECTION_STABLE_TIME     (CLOCK_SECOND * 5)
#define NEW_CONFIG_WAIT_INTERVAL   (CLOCK_SECOND * 20)
static struct timer connection_life;
static uint8_t connect_attempt;
/*---------------------------------------------------------------------------*/
/* Various states */
static uint8_t state;
#define MQTT_CLIENT_STATE_INIT            0
#define MQTT_CLIENT_STATE_REGISTERED      1
#define MQTT_CLIENT_STATE_CONNECTING      2
#define MQTT_CLIENT_STATE_CONNECTED       3
#define MQTT_CLIENT_STATE_PUBLISHING      4
#define MQTT_CLIENT_STATE_DISCONNECTED    5
#define MQTT_CLIENT_STATE_NEWCONFIG       6
#define MQTT_CLIENT_STATE_CONFIG_ERROR 0xFE
#define MQTT_CLIENT_STATE_ERROR        0xFF
/*---------------------------------------------------------------------------*/
/* Maximum TCP segment size for outgoing segments of our socket */
#define MQTT_CLIENT_MAX_SEGMENT_SIZE    32
/*---------------------------------------------------------------------------*/
/*
 * Buffers for Client ID and Topic.
 * Make sure they are large enough to hold the entire respective string
 *
 * d:quickstart:status:EUI64 is 32 bytes long
 * iot-2/evt/status/fmt/json is 25 bytes
 * We also need space for the null termination
 */
#define BUFFER_SIZE 64
static char client_id[BUFFER_SIZE];
static char pub_topic[BUFFER_SIZE];
static char sub_topic[BUFFER_SIZE];
/*---------------------------------------------------------------------------*/
/*
 * The main MQTT buffers.
 * We will need to increase if we start publishing more data.
 */
#define APP_BUFFER_SIZE 512
static struct mqtt_connection conn;
static char app_buffer[APP_BUFFER_SIZE];
/*---------------------------------------------------------------------------*/
#define QUICKSTART "PD"

LIST(MQTT_subscribe_list);
LIST(MQTT_publish_list);
LIST(MQTT_config_list);

typedef struct MQTT_config_ele {
  struct MQTT_config_ele *next;
  char topic[BUFFER_SIZE];
  char arg[BUFFER_SIZE*10];
} MQTT_config_ele_t;

typedef struct MQTT_sub_ele {
  struct MQTT_sub_ele *next;
  char topic[BUFFER_SIZE];
} MQTT_sub_ele_t;

MQTT_config_ele_t MQTT_dht11_temperature_config;
MQTT_config_ele_t MQTT_dht11_humidity_config;
MQTT_config_ele_t MQTT_step_motor_config;

MQTT_sub_ele_t MQTT_step_motor_sub_cmd;
MQTT_sub_ele_t MQTT_step_motor_sub_tilt;
/*---------------------------------------------------------------------------*/
static struct mqtt_message *msg_ptr = 0;
static struct etimer publish_periodic_timer;
static struct ctimer ct;
static char *buf_ptr;
static uint16_t seq_nr_value = 0;
/*---------------------------------------------------------------------------*/
static uip_ip6addr_t def_route;
/*---------------------------------------------------------------------------*/
/* Parent RSSI functionality */
extern int def_rt_rssi;
/*---------------------------------------------------------------------------*/
const static cc26xx_web_demo_sensor_reading_t *reading;
/*---------------------------------------------------------------------------*/
mqtt_client_config_t *conf;
/*---------------------------------------------------------------------------*/
PROCESS(mqtt_client_process, "CC26XX MQTT Client");
/*---------------------------------------------------------------------------*/
static void
new_net_config(void)
{
  /*
   * We got a new configuration over the net.
   *
   * Disconnect from the current broker and stop the periodic timer.
   *
   * When the source of the new configuration is done, we will get notified
   * via an event.
   */
  if(state == MQTT_CLIENT_STATE_NEWCONFIG) {
    return;
  }

  state = MQTT_CLIENT_STATE_NEWCONFIG;

  etimer_stop(&publish_periodic_timer);
  mqtt_disconnect(&conn);
}
#ifdef NODE_LIGHT
static int
RGB_blue_post_handler(char *key, int key_len, char *val, int val_len)
{
  int ret = HTTPD_SIMPLE_POST_HANDLER_UNKNOWN;
  RGB_t tmp;
  if(key_len != strlen("RGB_blue") ||
     strncasecmp(key, "RGB_blue", strlen("RGB_blue")) != 0) {
    /* Not ours */
    return HTTPD_SIMPLE_POST_HANDLER_UNKNOWN;
  }

  ret = atoi(val);

   if(ret < 0 ||
      ret > 255) {
     return HTTPD_SIMPLE_POST_HANDLER_ERROR;
   }

   tmp.all = RGB_sensor.value(SENSOR_ERROR);
	tmp.led.b = ret;
	RGB_sensor.value(tmp.all);

   return HTTPD_SIMPLE_POST_HANDLER_OK;
}

/*---------------------------------------------------------------------------*/

static int
RGB_green_post_handler(char *key, int key_len, char *val, int val_len)
{
  int ret = HTTPD_SIMPLE_POST_HANDLER_UNKNOWN;
  RGB_t tmp;
  if(key_len != strlen("RGB_green") ||
     strncasecmp(key, "RGB_green", strlen("RGB_green")) != 0) {
    /* Not ours */
    return HTTPD_SIMPLE_POST_HANDLER_UNKNOWN;
  }

  ret = atoi(val);

   if(ret < 0 ||
      ret > 255) {
     return HTTPD_SIMPLE_POST_HANDLER_ERROR;
   }

   tmp.all = RGB_sensor.value(SENSOR_ERROR);
  tmp.led.g = ret;
  RGB_sensor.value(tmp.all);

   return HTTPD_SIMPLE_POST_HANDLER_OK;
}

/*---------------------------------------------------------------------------*/
static int
RGB_red_post_handler(char *key, int key_len, char *val, int val_len)
{
  int ret = HTTPD_SIMPLE_POST_HANDLER_UNKNOWN;
  RGB_t tmp;
  if(key_len != strlen("RGB_red") ||
     strncasecmp(key, "RGB_red", strlen("RGB_red")) != 0) {
    /* Not ours */
    return HTTPD_SIMPLE_POST_HANDLER_UNKNOWN;
  }

  ret = atoi(val);

   if(ret < 0 ||
      ret > 255) {
     return HTTPD_SIMPLE_POST_HANDLER_ERROR;
   }

   tmp.all = RGB_sensor.value(SENSOR_ERROR);
   tmp.led.r = ret;
   RGB_sensor.value(tmp.all);

   return HTTPD_SIMPLE_POST_HANDLER_OK;
}

/*---------------------------------------------------------------------------*/
static int
RGB_brightness_post_handler(char *key, int key_len, char *val, int val_len)
{
  int ret = HTTPD_SIMPLE_POST_HANDLER_UNKNOWN;
  RGB_t tmp;
  if(key_len != strlen("RGB_brightness") ||
     strncasecmp(key, "RGB_brightness", strlen("RGB_brightness")) != 0) {
    /* Not ours */
    return HTTPD_SIMPLE_POST_HANDLER_UNKNOWN;
  }

  ret = atoi(val);

   if(ret < 0 ||
      ret > 255) {
     return HTTPD_SIMPLE_POST_HANDLER_ERROR;
   }

   tmp.all = RGB_sensor.value(SENSOR_ERROR);
   tmp.led.brightness = ret;
   RGB_sensor.value(tmp.all);

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

   tmp.all = ((RGB_hard_t *) hard_RGB_sensor.value(SENSOR_ERROR))->all;
	tmp.led.b = ret;
	hard_RGB_sensor.value(&tmp);

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

   tmp.all = ((RGB_hard_t *) hard_RGB_sensor.value(SENSOR_ERROR))->all;
  tmp.led.g = ret;
  hard_RGB_sensor.value(&tmp);

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

   tmp.all = ((RGB_hard_t *) hard_RGB_sensor.value(SENSOR_ERROR))->all;
   tmp.led.r = ret;
   hard_RGB_sensor.value(&tmp);

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
      ret > 128) {
     return HTTPD_SIMPLE_POST_HANDLER_ERROR;
   }

   tmp.all = ((RGB_hard_t *) hard_RGB_sensor.value(SENSOR_ERROR))->all;
   tmp.led.brightness = ret;
   hard_RGB_sensor.value(&tmp);

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
static void
pub_handler(const char *topic, uint16_t topic_len, const uint8_t *chunk,
            uint16_t chunk_len)
{
  DBG("Pub Handler: topic='%s' (len=%u), chunk_len=%u\n", topic, topic_len,
      chunk_len);

  /* If we don't like the length, ignore */
  if(topic_len != 23 || chunk_len != 1) {
    printf("Incorrect topic or chunk len. Ignored\n");
    return;
  }

  /* If the format != json, ignore */
  if(strncmp(&topic[topic_len - 4], "json", 4) != 0) {
    printf("Incorrect format\n");
  }

  if(strncmp(&topic[10], "leds", 4) == 0) {
    if(chunk[0] == '1') {
      leds_on(LEDS_RED);
    } else if(chunk[0] == '0') {
      leds_off(LEDS_RED);
    }
    return;
  }

#if BOARD_SENSORTAG
  if(strncmp(&topic[10], "buzz", 4) == 0) {
    if(chunk[0] == '1') {
      buzzer_start(1000);
    } else if(chunk[0] == '0') {
      buzzer_stop();
    }
    return;
  }
#endif
}
/*---------------------------------------------------------------------------*/
static void
mqtt_event(struct mqtt_connection *m, mqtt_event_t event, void *data)
{
  switch(event) {
  case MQTT_EVENT_CONNECTED: {
    DBG("APP - Application has a MQTT connection\n");
    timer_set(&connection_life, CONNECTION_STABLE_TIME);
    state = MQTT_CLIENT_STATE_CONNECTED;
    break;
  }
  case MQTT_EVENT_DISCONNECTED: {
    DBG("APP - MQTT Disconnect. Reason %u\n", *((mqtt_event_t *)data));

    /* Do nothing if the disconnect was the result of an incoming config */
    if(state != MQTT_CLIENT_STATE_NEWCONFIG) {
      state = MQTT_CLIENT_STATE_DISCONNECTED;
      process_poll(&mqtt_client_process);
    }
    break;
  }
  case MQTT_EVENT_PUBLISH: {
    msg_ptr = data;

    /* Implement first_flag in publish message? */
    if(msg_ptr->first_chunk) {
      msg_ptr->first_chunk = 0;
      DBG("APP - Application received a publish on topic '%s'. Payload "
          "size is %i bytes. Content:\n\n",
          msg_ptr->topic, msg_ptr->payload_length);
    }

    pub_handler(msg_ptr->topic, strlen(msg_ptr->topic), msg_ptr->payload_chunk,
                msg_ptr->payload_length);
    break;
  }
  case MQTT_EVENT_SUBACK: {
    DBG("APP - Application is subscribed to topic successfully\n");
    break;
  }
  case MQTT_EVENT_UNSUBACK: {
    DBG("APP - Application is unsubscribed to topic successfully\n");
    break;
  }
  case MQTT_EVENT_PUBACK: {
    DBG("APP - Publishing complete.\n");
    break;
  }
  default:
    DBG("APP - Application got a unhandled MQTT event: %i\n", event);
    break;
  }
}
/*---------------------------------------------------------------------------*/
static int
construct_pub_topic(void)
{
  int len = snprintf(pub_topic, BUFFER_SIZE, "Hass/%s/%s/%s/state",
		  	  	  	  "sensor",
		  	  	  	  client_id,
					  conf->Username);

  /* len < 0: Error. Len >= BUFFER_SIZE: Buffer too small */
  if(len < 0 || len >= BUFFER_SIZE) {
    printf("Pub Topic: %d, Buffer %d\n", len, BUFFER_SIZE);
    return 0;
  }

  return 1;
}
/*---------------------------------------------------------------------------*/
static int
construct_configs(void)
{
//--------------------------------------------------------------------------------------------
	snprintf(MQTT_dht11_temperature_config.topic, sizeof(MQTT_dht11_temperature_config.topic),
			"Hass/%s/%s/%s/%s/config",
	  	  	"sensor", client_id, conf->Username, "Temperature");

	snprintf(MQTT_dht11_temperature_config.arg, sizeof(MQTT_dht11_temperature_config.arg),
			"{\"device_class\": \"sensor\","
			" \"name\": \"Temperature\","
			" \"state_topic\": \"Hass/%s/%s/%s/%s/state\","
			" \"unit_of_measurement\": \"�C\","
			" \"value_template\": \"{{ value_json.%s}}\" }",
			"sensor", client_id, conf->Username,"dht11", "Temperature"); // Last must be equal reading->descr

	list_add(MQTT_config_list,&MQTT_dht11_temperature_config);
//--------------------------------------------------------------------------------------------
	snprintf(MQTT_dht11_humidity_config.topic, sizeof(MQTT_dht11_humidity_config.topic),
			"Hass/%s/%s/%s/%s/config",
		  	"sensor", client_id, conf->Username, "Humidity");

	snprintf(MQTT_dht11_humidity_config.arg, sizeof(MQTT_dht11_humidity_config.arg),
			"{\"device_class\": \"sensor\","
			"\"name\": \"Humidity\","
			" \"state_topic\": \"Hass/%s/%s/%s/%s/state\","
			" \"unit_of_measurement\": \"�C\","
			" \"value_template\": \"{{ value_json.%s}}\" }",
			"sensor", client_id, conf->Username,"dht11", "Humidity"); // Last must be equal reading->descr

	list_add(MQTT_config_list,&MQTT_dht11_humidity_config);
//--------------------------------------------------------------------------------------------

	snprintf(MQTT_step_motor_config.topic, sizeof(MQTT_step_motor_config.topic),
			"Hass/%s/%s/%s/%s/config",
		  	"cover", client_id, conf->Username, "Step-Position");

	snprintf(MQTT_step_motor_config.arg, sizeof(MQTT_step_motor_config.arg),
			"{\"device_class\": \"cover\","
			"\"name\": \"%s Cover\","
			"\"command_topic\": \"Hass/%s/%s/%s/%s/set\","
			"\"state_topic\": \"Hass/%s/%s/%s/%s/state\","
			"\"retain\": true,"
			"\"payload_open\": \"OPEN\","
			"\"payload_close\": \"CLOSE\","
			"\"payload_stop\": \"STOP\","
			"\"state_open\": \"open\","
			"\"state_closed\": \"closed\","
			//"\"payload_available\": \"online\","
			//"\"payload_not_available\": \"offline\","
			"\"value_template\": \"{{ value_json.%s}}\","
			"\"tilt_command_topic\": \"Hass/%s/%s/%s/%s/tilt\","
			"\"tilt_status_topic\": \"Hass/%s/%s/%s/%s/tilt-state\","
			"\"tilt_min\": 0,"
			"\"tilt_max\": 4096,"
			"\"tilt_closed_value\": 70,"
			"\"tilt_opened_value\": 4000,"
			" }",
			conf->Username,
			"cover", client_id, conf->Username,"Step-Position",
			"cover", client_id, conf->Username,"Step-Position",
			"Step-Position",
			"cover", client_id, conf->Username,"Step-Position",
			"cover", client_id, conf->Username,"Step-Position"); // Last must be equal reading->descr

	list_add(MQTT_config_list,&MQTT_step_motor_config);
	return 1;
}
//--------------------------------------------------------------------------------------------

static int
construct_sub_topic(void){


	snprintf(MQTT_step_motor_sub_cmd.topic, sizeof(MQTT_step_motor_sub_cmd.topic),
				"Hass/%s/%s/%s/%s/set",
		  	  	"cover", client_id, conf->Username, "Step-Position");
	list_add(MQTT_subscribe_list,&MQTT_step_motor_sub_cmd);
//--------------------------------------------------------------------------------------------
	snprintf(MQTT_step_motor_sub_tilt.topic, sizeof(MQTT_step_motor_sub_tilt.topic),
				"Hass/%s/%s/%s/%s/tilt",
				"cover", client_id, conf->Username, "Step-Position");
	list_add(MQTT_subscribe_list,&MQTT_step_motor_sub_tilt);
//--------------------------------------------------------------------------------------------
  int len = snprintf(sub_topic, BUFFER_SIZE, "Hass/%s/%s/%s/set",
	  	  	  "sensor",
	  	  	  client_id,
			  conf->Username);

  /* len < 0: Error. Len >= BUFFER_SIZE: Buffer too small */
  if(len < 0 || len >= BUFFER_SIZE) {
    printf("Sub Topic: %d, Buffer %d\n", len, BUFFER_SIZE);
    return 0;
  }

  return 1;
}
/*---------------------------------------------------------------------------*/
static int
construct_client_id(void)
{
  int len = snprintf(client_id, BUFFER_SIZE, "d:%s:%s:%02x%02x%02x%02x%02x%02x",
                     conf->Company, conf->Modul_type,
                     linkaddr_node_addr.u8[0], linkaddr_node_addr.u8[1],
                     linkaddr_node_addr.u8[2], linkaddr_node_addr.u8[5],
                     linkaddr_node_addr.u8[6], linkaddr_node_addr.u8[7]);

  /* len < 0: Error. Len >= BUFFER_SIZE: Buffer too small */
  if(len < 0 || len >= BUFFER_SIZE) {
    printf("Client ID: %d, Buffer %d\n", len, BUFFER_SIZE);
    return 0;
  }

  return 1;
}
/*---------------------------------------------------------------------------*/
static void
update_config(void)
{
  if(construct_client_id() == 0) {
    /* Fatal error. Client ID larger than the buffer */
    state = MQTT_CLIENT_STATE_CONFIG_ERROR;
    return;
  }

  construct_configs();

  if(construct_sub_topic() == 0) {
    /* Fatal error. Topic larger than the buffer */
    state = MQTT_CLIENT_STATE_CONFIG_ERROR;
    return;
  }

  if(construct_pub_topic() == 0) {
    /* Fatal error. Topic larger than the buffer */
    state = MQTT_CLIENT_STATE_CONFIG_ERROR;
    return;
  }

  /* Reset the counter */
  seq_nr_value = 0;

  state = MQTT_CLIENT_STATE_INIT;

  /*
   * Schedule next timer event ASAP
   *
   * If we entered an error state then we won't do anything when it fires.
   *
   * Since the error at this stage is a config error, we will only exit this
   * error state if we get a new config.
   */
  etimer_set(&publish_periodic_timer, 0);

  return;
}
/*---------------------------------------------------------------------------*/
static int
init_config()
{
  /* Populate configuration with default values */
  memset(conf, 0, sizeof(mqtt_client_config_t));

  memcpy(conf->Company, CC26XX_WEB_DEMO_DEFAULT_ORG_ID, sizeof(CC26XX_WEB_DEMO_DEFAULT_ORG_ID));
  memcpy(conf->Modul_type, CC26XX_WEB_DEMO_DEFAULT_TYPE_ID, sizeof(CC26XX_WEB_DEMO_DEFAULT_TYPE_ID));
  memcpy(conf->Username, CC26XX_WEB_DEMO_DEFAULT_EVENT_TYPE_ID, 7);
  memcpy(conf->broker_ip, broker_ip, strlen(broker_ip));

  conf->broker_port = CC26XX_WEB_DEMO_DEFAULT_BROKER_PORT;
  conf->pub_interval = CC26XX_WEB_DEMO_DEFAULT_PUBLISH_INTERVAL;

  return 1;
}
/*---------------------------------------------------------------------------*/

static void
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
}
/*---------------------------------------------------------------------------*/
static void
subscribe(void)
{
  /* Publish MQTT topic in IBM quickstart format */
  mqtt_status_t status;

  status = mqtt_subscribe(&conn, NULL, sub_topic, MQTT_QOS_LEVEL_0);

  DBG("APP - Subscribing!\n");
  if(status == MQTT_STATUS_OUT_QUEUE_FULL) {
    DBG("APP - Tried to subscribe but command queue was full!\n");
  }
}
/*---------------------------------------------------------------------------*/
static void
publish(void)
{
  /* Publish MQTT topic in IBM quickstart format */
  int len;
  int remaining = APP_BUFFER_SIZE;
  //char def_rt_str[64];

  seq_nr_value++;

  buf_ptr = app_buffer;

  len = snprintf(buf_ptr, remaining,
		  "{"
		  "\"Board\":\"%s\","
		  "\"Node\":\"%s\","
		  "\"Name\":\"%s\","
		  "\"Seq #\":%d,"
		  "\"Uptime [s]\":%lu",
                 BOARD_STRING,SENSOR_STRING, conf->Username ,seq_nr_value, clock_seconds());

  if(len < 0 || len >= remaining) {
    printf("Buffer too short. Have %d, need %d + \\0\n", remaining, len);
    return;
  }

  remaining -= len;
  buf_ptr += len;


  for(reading = cc26xx_web_demo_sensor_first();
      reading != NULL; reading = reading->next) {
    if(reading->publish && reading->raw != SENSOR_ERROR) {
      len = snprintf(buf_ptr, remaining,
                     ",\"%s\":%s", reading->descr, reading->converted);

      if(len < 0 || len >= remaining) {
        printf("Buffer too short. Have %d, need %d + \\0\n", remaining, len);
        return;
      }
      remaining -= len;
      buf_ptr += len;
    }
  }

  len = snprintf(buf_ptr, remaining, "}");

  if(len < 0 || len >= remaining) {
    printf("Buffer too short. Have %d, need %d + \\0\n", remaining, len);
    return;
  }

  mqtt_publish(&conn, NULL, pub_topic, (uint8_t *)app_buffer,
               strlen(app_buffer), MQTT_QOS_LEVEL_0, MQTT_RETAIN_OFF);

  DBG("APP - Publish!\n");
}
/*---------------------------------------------------------------------------*/
static void
connect_to_broker(void)
{
  /* Connect to MQTT server */
  mqtt_status_t conn_attempt_result = mqtt_connect(&conn, conf->broker_ip,
                                                   conf->broker_port,
                                                   conf->pub_interval * 3);

  if(conn_attempt_result == MQTT_STATUS_OK) {
    state = MQTT_CLIENT_STATE_CONNECTING;
  } else {
    state = MQTT_CLIENT_STATE_CONFIG_ERROR;
  }
}
/*---------------------------------------------------------------------------*/
static void
state_machine(void)
{

  switch(state) {
  case MQTT_CLIENT_STATE_INIT:
    /* If we have just been configured register MQTT connection */
    mqtt_register(&conn, &mqtt_client_process, client_id, mqtt_event,
                  MQTT_CLIENT_MAX_SEGMENT_SIZE);

    /*
     * If we are not using the quickstart service (thus we are an IBM
     * registered device), we need to provide user name and password
     */

      if(strlen(conf->MQTT_user_Password) == 0 || strlen(conf->MQTT_user_name) == 0) {
        printf("User name or password is empty\n");

        //break;
      } else {
        mqtt_set_username_password(&conn, conf->MQTT_user_name,
                                   conf->MQTT_user_Password);
      }


    /* _register() will set auto_reconnect. We don't want that. */
    conn.auto_reconnect = 0;
    connect_attempt = 1;

    /*
     * Wipe out the default route so we'll republish it every time we switch to
     * a new broker
     */
    memset(&def_route, 0, sizeof(def_route));

    state = MQTT_CLIENT_STATE_REGISTERED;
    DBG("Init\n");
    /* Continue */
  case MQTT_CLIENT_STATE_REGISTERED:
    if(uip_ds6_get_global(ADDR_PREFERRED) != NULL) {
      /* Registered and with a public IP. Connect */
      DBG("Registered. Connect attempt %u\n", connect_attempt);
      connect_to_broker();
    }
    etimer_set(&publish_periodic_timer, CC26XX_WEB_DEMO_NET_CONNECT_PERIODIC);
    return;
    break;
  case MQTT_CLIENT_STATE_CONNECTING:
    /* Not connected yet. Wait */
    DBG("Connecting (%u)\n", connect_attempt);
    break;
  case MQTT_CLIENT_STATE_CONNECTED:
    /* Don't subscribe unless we are a registered device */
    if(strncasecmp(conf->Company, QUICKSTART, strlen(conf->Company)) == 0) {
      DBG("Using 'quickstart': Skipping subscribe\n");
      state = MQTT_CLIENT_STATE_PUBLISHING;
    }
    /* Continue */
  case MQTT_CLIENT_STATE_PUBLISHING:
    /* If the timer expired, the connection is stable. */
    if(timer_expired(&connection_life)) {
      /*
       * Intentionally using 0 here instead of 1: We want RECONNECT_ATTEMPTS
       * attempts if we disconnect after a successful connect
       */
      connect_attempt = 0;
    }

    if(mqtt_ready(&conn) && conn.out_buffer_sent) {
      /* Connected. Publish */
      if(state == MQTT_CLIENT_STATE_CONNECTED) {
        subscribe();
        state = MQTT_CLIENT_STATE_PUBLISHING;
      } else {
        publish();
      }
      etimer_set(&publish_periodic_timer, conf->pub_interval);

      DBG("Publishing\n");
      /* Return here so we don't end up rescheduling the timer */
      return;
    } else {
      /*
       * Our publish timer fired, but some MQTT packet is already in flight
       * (either not sent at all, or sent but not fully ACKd).
       *
       * This can mean that we have lost connectivity to our broker or that
       * simply there is some network delay. In both cases, we refuse to
       * trigger a new message and we wait for TCP to either ACK the entire
       * packet after retries, or to timeout and notify us.
       */
      DBG("Publishing... (MQTT state=%d, q=%u)\n", conn.state,
          conn.out_queue_full);
    }
    break;
  case MQTT_CLIENT_STATE_DISCONNECTED:
    DBG("Disconnected\n");
    if(connect_attempt < RECONNECT_ATTEMPTS ||
       RECONNECT_ATTEMPTS == RETRY_FOREVER) {
      /* Disconnect and backoff */
      clock_time_t interval;
      mqtt_disconnect(&conn);
      connect_attempt++;

      interval = connect_attempt < 3 ? RECONNECT_INTERVAL << connect_attempt :
        RECONNECT_INTERVAL << 3;

      DBG("Disconnected. Attempt %u in %lu ticks\n", connect_attempt, interval);

      etimer_set(&publish_periodic_timer, interval);

      state = MQTT_CLIENT_STATE_REGISTERED;
      return;
    } else {
      /* Max reconnect attempts reached. Enter error state */
      state = MQTT_CLIENT_STATE_ERROR;
      DBG("Aborting connection after %u attempts\n", connect_attempt - 1);
    }
    break;
  case MQTT_CLIENT_STATE_NEWCONFIG:
    /* Only update config after we have disconnected or in the case of an error */
    if(conn.state == MQTT_CONN_STATE_NOT_CONNECTED || conn.state == MQTT_CONN_STATE_ERROR) {
      update_config();
      DBG("New config\n");

      /* update_config() scheduled next pass. Return */
      return;
    }
    break;
  case MQTT_CLIENT_STATE_CONFIG_ERROR:
    /* Idle away. The only way out is a new config */
    printf("Bad configuration.\n");
    return;
  case MQTT_CLIENT_STATE_ERROR:
  default:
    leds_on(CC26XX_WEB_DEMO_STATUS_LED);
    /*
     * 'default' should never happen.
     *
     * If we enter here it's because of some error. Stop timers. The only thing
     * that can bring us out is a new config event
     */
    printf("Default case: State=0x%02x\n", state);
    return;
  }

  /* If we didn't return so far, reschedule ourselves */
  etimer_set(&publish_periodic_timer, STATE_MACHINE_PERIODIC);
}
/*---------------------------------------------------------------------------*/
PROCESS_THREAD(mqtt_client_process, ev, data)
{

  PROCESS_BEGIN();

  printf("CC26XX MQTT Client Process\n");

  conf = &web_demo_config.mqtt_config;
  if(init_config() != 1) {
    PROCESS_EXIT();
  }

  register_http_post_handlers();

  update_config();

  /* Main loop */
  while(1) {

    PROCESS_YIELD();

    if(ev == sensors_event) {
      if(state == MQTT_CLIENT_STATE_ERROR) {
        connect_attempt = 1;
        state = MQTT_CLIENT_STATE_REGISTERED;
      }
    }

    if(ev == httpd_simple_event_new_config) {
      /*
       * Schedule next pass in a while. When HTTPD sends us this event, it is
       * also in the process of sending the config page. Wait a little before
       * reconnecting, so as to not cause congestion.
       */
      etimer_set(&publish_periodic_timer, NEW_CONFIG_WAIT_INTERVAL);
    }

    if((ev == PROCESS_EVENT_TIMER && data == &publish_periodic_timer) ||
       ev == PROCESS_EVENT_POLL ||
       ev == cc26xx_web_demo_publish_event /*||
       (ev == sensors_event)*/) {
      state_machine();
    }

    if(ev == cc26xx_web_demo_load_config_defaults) {
      init_config();
      etimer_set(&publish_periodic_timer, NEW_CONFIG_WAIT_INTERVAL);
    }

  }

  PROCESS_END();
}
/*---------------------------------------------------------------------------*/
/**
 * @}
 */
