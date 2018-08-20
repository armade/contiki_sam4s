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
 *     Header file for the CC26xx web demo MQTT client functionality
 */
/*---------------------------------------------------------------------------*/
#ifndef MQTT_CLIENT_H_
#define MQTT_CLIENT_H_
/*---------------------------------------------------------------------------*/
#define MQTT_CLIENT_CONFIG_ORG_ID_LEN        32
#define MQTT_CLIENT_CONFIG_TYPE_ID_LEN       32
#define MQTT_CLIENT_CONFIG_AUTH_TOKEN_LEN    32
#define MQTT_CLIENT_CONFIG_USER_NAME_LEN    32
#define MQTT_CLIENT_CONFIG_EVENT_TYPE_ID_LEN 32
#define MQTT_CLIENT_CONFIG_IP_ADDR_STR_LEN   64
/*---------------------------------------------------------------------------*/
#define MQTT_CLIENT_PUBLISH_INTERVAL_MAX      86400 /* secs: 1 day */
#define MQTT_CLIENT_PUBLISH_INTERVAL_MIN          5 /* secs */
/*---------------------------------------------------------------------------*/
PROCESS_NAME(mqtt_client_process);
/*---------------------------------------------------------------------------*/
uint8_t MQTT_ready(void);
uint8_t MQTT_init_config(void);
/**
 * \brief Data structure declaration for the MQTT client configuration
 */
typedef struct mqtt_client_config {
  char Company[MQTT_CLIENT_CONFIG_ORG_ID_LEN];
  char Modul_type[MQTT_CLIENT_CONFIG_TYPE_ID_LEN];
  char MQTT_user_name[MQTT_CLIENT_CONFIG_USER_NAME_LEN];
  char MQTT_user_Password[MQTT_CLIENT_CONFIG_AUTH_TOKEN_LEN];
  char Username[MQTT_CLIENT_CONFIG_EVENT_TYPE_ID_LEN];
  char broker_ip[MQTT_CLIENT_CONFIG_IP_ADDR_STR_LEN];
  clock_time_t pub_interval;
  uint16_t broker_port;
} mqtt_client_config_t;

extern mqtt_client_config_t *conf;

// username + function eg. kontor internal temperature
//client_id, conf->Username -> Hass/sensor/%s/%s/state
//reading->descr -> value_json.%s
static const char sensor_config_payload[] =	"{\"name\": \"%s %s\"," \
										"\"state_topic\": \"Hass/common/%s/%s/state\"," \
										"\"unit_of_measurement\": \"%s\","\
										"\"value_template\":\"{{ value_json.%s}}\" }";

static const char sensor_config_topic[] = "Hass/sensor/%s%02x%02x%02x%02x%02x%02x/config";

static const char cover_config_payload[] = "{\"device_class\": \"cover\"," \
		"\"name\": \"%s %s\"," \
		"\"command_topic\": \"Hass/common/%s/%s/set\"," \
		"\"state_topic\": \"Hass/cover/%s/%s/state\"," \
		"\"retain\": true," \
		"\"payload_open\": \"OPEN\"," \
		"\"payload_close\": \"CLOSE\"," \
		"\"payload_stop\": \"STOP\"," \
		"\"state_open\": \"open\"," \
		"\"state_closed\": \"closed\"," \
		"\"value_template\": \"{{ value_json.%s}}\"," \
		"\"tilt_command_topic\": \"Hass/cover/%s/%s/tilt\"," \
		"\"tilt_status_topic\": \"Hass/cover/%s/%s/tilt-state\"," \
		"\"tilt_min\": 0," \
		"\"tilt_max\": 4096," \
		"\"tilt_closed_value\": 70," \
		"\"tilt_opened_value\": 4000,}";

static const char cover_config_topic[] = "Hass/cover/%s%02x%02x%02x%02x%02x%02x/config";

static const char switch_config_payload[] = "{\"name\": \"%s %s\"," \
				"\"state_topic\": \"Hass/common/%s/%s/state\"," \
				"\"value_template\":\"{{ value_json.%s}}\"," \
				"\"mdi\":\"lightbulb\"," \
				"\"command_topic\": \"Hass/switch/%s/%s/%s/set\"}";

static const char switch_config_topic[] = "Hass/switch/%s%02x%02x%02x%02x%02x%02x/config";
static const char switch_sub_topic[] = "Hass/switch/%s/%s/%s/set";

void pub_relay1_handler(uint8_t *payload, uint16_t len);
void pub_relay2_handler(uint8_t *payload, uint16_t len);
void pub_relay3_handler(uint8_t *payload, uint16_t len);
void pub_relay4_handler(uint8_t *payload, uint16_t len);
/*---------------------------------------------------------------------------*/
#endif /* MQTT_CLIENT_H_ */
/*---------------------------------------------------------------------------*/
/**
 * @}
 */
