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
/*---------------------------------------------------------------------------*/
/* Binary Sensor
    None,
    battery,//: On means low, Off means normal
    cold,//: On means cold, Off means normal
    connectivity,//: On means connected, Off means disconnected
    door,//: On means open, Off means closed
    garage_door,//: On means open, Off means closed
    gas,//: On means gas detected, Off means no gas (clear)
    heat,//: On means hot, Off means normal
    light,//: On means light detected, Off means no light
    lock,//: On means open (unlocked), Off means closed (locked)
    moisture,//: On means moisture detected (wet), Off means no moisture (dry)
    motion,//: On means motion detected, Off means no motion (clear)
    moving,//: On means moving, Off means not moving (stopped)
    occupancy,//: On means occupied, Off means not occupied (clear)
    opening,//: On means open, Off means closed
    plug,//: On means device is plugged in, Off means device is unplugged
    power,//: On means power detected, Off means no power
    presence,//: On means home, Off means away
    problem,//: On means problem detected, Off means no problem (OK),//:    safety: On means unsafe, Off means safe
    smoke,//: On means smoke detected, Off means no smoke (clear)
    sound,//: On means sound detected, Off means no sound (clear)
    vibration,//: On means vibration detected, Off means no vibration (clear)
    window,//: On means open, Off means closed
*/

#define FOREACH_DEVICE_CLASS(BINARY_DEVICE_CLASS) \
		BINARY_DEVICE_CLASS(None)   \
		BINARY_DEVICE_CLASS(battery)   \
		BINARY_DEVICE_CLASS(cold)  \
		BINARY_DEVICE_CLASS(connectivity)   \
		BINARY_DEVICE_CLASS(door)  \
		BINARY_DEVICE_CLASS(garage_door)  \
		BINARY_DEVICE_CLASS(gas)  \
		BINARY_DEVICE_CLASS(heat)  \
		BINARY_DEVICE_CLASS(light)  \
		BINARY_DEVICE_CLASS(lock)  \
		BINARY_DEVICE_CLASS(moisture)  \
		BINARY_DEVICE_CLASS(motion)  \
		BINARY_DEVICE_CLASS(moving)  \
		BINARY_DEVICE_CLASS(occupancy)  \
		BINARY_DEVICE_CLASS(opening)  \
		BINARY_DEVICE_CLASS(plug)  \
		BINARY_DEVICE_CLASS(power)  \
		BINARY_DEVICE_CLASS(presence)  \
		BINARY_DEVICE_CLASS(problem)  \
		BINARY_DEVICE_CLASS(smoke)  \
		BINARY_DEVICE_CLASS(sound)  \
		BINARY_DEVICE_CLASS(vibration)  \
		BINARY_DEVICE_CLASS(window)  \


#define DEVICE_CLASS_GENERATE_ENUM(ENUM) ENUM,
#define DEVICE_CLASS_GENERATE_STRING(STRING) #STRING,

typedef enum  {
	FOREACH_DEVICE_CLASS(DEVICE_CLASS_GENERATE_ENUM)
}DEVICE_CLASS_t;

__attribute__((used)) static const char *DEVICE_CLASS_lookup[] = {
	FOREACH_DEVICE_CLASS(DEVICE_CLASS_GENERATE_STRING)
};
/*---------------------------------------------------------------------------*/




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
//----------------------------------------------------------------------------------------------------------
static const char sensor_config_topic[] = "Hass/sensor/%s%02x%02x%02x%02x%02x%02x/config";
//----------------------------------------------------------------------------------------------------------
static const char switch_config_topic[] = "Hass/switch/%s%02x%02x%02x%02x%02x%02x/config";
static const char switch_sub_topic[] = "Hass/switch/%s/%s/set";
//----------------------------------------------------------------------------------------------------------
static const char light_config_topic[] = "Hass/light/%s%02x%02x%02x%02x%02x%02x/config";
static const char light_sub_topic[] = "Hass/light/%s/%s/set";
//----------------------------------------------------------------------------------------------------------
static const char binary_sensor_config_topic[] = "Hass/binary_sensor/%s%02x%02x%02x%02x%02x%02x/config";
//----------------------------------------------------------------------------------------------------------
void pub_relay1_handler(uint8_t *payload, uint16_t len);
void pub_relay2_handler(uint8_t *payload, uint16_t len);
void pub_relay3_handler(uint8_t *payload, uint16_t len);
void pub_relay4_handler(uint8_t *payload, uint16_t len);

void pub_christmas_light_handler(uint8_t *payload, uint16_t len);

void pub_light_hard_switch_handler(uint8_t *payload, uint16_t len);
void pub_light_hard_brightness_handler(uint8_t *payload, uint16_t len);
void pub_light_hard_rgb_handler(uint8_t *payload, uint16_t len);
void pub_light_hard_effect_handler(uint8_t *payload, uint16_t len);

void pub_light_soft_switch_handler(uint8_t *payload, uint16_t len);
void pub_light_soft_brightness_handler(uint8_t *payload, uint16_t len);
void pub_light_soft_rgb_handler(uint8_t *payload, uint16_t len);
void pub_light_soft_effect_handler(uint8_t *payload, uint16_t len);
/*---------------------------------------------------------------------------*/
#endif /* MQTT_CLIENT_H_ */
/*---------------------------------------------------------------------------*/
/**
 * @}
 */
