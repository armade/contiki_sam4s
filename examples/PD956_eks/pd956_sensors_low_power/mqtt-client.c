#include "contiki-conf.h"
#include "rpl/rpl-private.h"
#include "mqtt.h"
#include "net/rpl/rpl.h"
#include "net/ip/uip.h"
#include "net/ipv6/uip-icmp6.h"
#include "sys/etimer.h"
#include "sys/ctimer.h"
#include "lib/sensors.h"
#include "board-peripherals.h"
#include "pd956_sensor_low_power.h"
#include "dev/leds.h"
#include "mqtt-client.h"
#include "httpd-simple.h"
#include "rf231.h"
#include "platform-conf.h"
#include "clock.h"
#include "ntpd.h"
#include "compiler.h"

#include <string.h>
#include <strings.h>

#define DEBUG 1
#if DEBUG
#define PRINTF(...) printf(__VA_ARGS__)
#else
#define PRINTF(...)
#endif
/*---------------------------------------------------------------------------*/
static const char *broker_ip = "bbbb::1";
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
#define MQTT_CLIENT_STATE_SUBSCRIBING     4
#define MQTT_CLIENT_STATE_PUBLISHING      5
#define MQTT_CLIENT_STATE_PUBLISHING_DONE 6
#define MQTT_CLIENT_STATE_DISCONNECTED    7
#define MQTT_CLIENT_STATE_NEWCONFIG       8
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
static char client_id[20];
static char pub_topic[BUFFER_SIZE];
//static char sub_topic[BUFFER_SIZE];
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
LIST(MQTT_config_list);


static MQTT_sub_ele_t *subscribe_ele;
static MQTT_sub_ele_t MQTT_COMMON_NO_SLEEP_sub_cmd;
static MQTT_sub_ele_t MQTT_COMMON_RESET_sub_cmd;
static MQTT_sub_ele_t MQTT_COMMON_NAME_sub_cmd;
/*---------------------------------------------------------------------------*/
static struct mqtt_message *msg_ptr = 0;
static struct etimer publish_periodic_timer;
static struct etimer sleep_retry_timer;
static struct etimer timeout_timer;
static char *buf_ptr;
static uint16_t seq_nr_value = 0;
/*---------------------------------------------------------------------------*/
static uip_ip6addr_t def_route;
/*---------------------------------------------------------------------------*/
static MQTT_sensor_reading_t *reading;
/*---------------------------------------------------------------------------*/
mqtt_client_config_t *conf;
static process_event_t MQTT_publish_sensor_data_done_event;
#if defined(NODE_GPS) || defined(NODE_4_ch_relay) || defined(NODE_1_ch_relay) || defined(NODE_HARD_LIGHT) || defined(NODE_LIGHT) || defined(NODE_STEP_MOTOR)
#define no_sleep_allowed 1
#else
static volatile uint8_t no_sleep_allowed = 0;
#endif
/*---------------------------------------------------------------------------*/
PROCESS(mqtt_client_process, "PD956 MQTT Client");
/*---------------------------------------------------------------------------*/
/*---------------------------------------------------------------------------*/
void new_net_config(void)
{
	/*
	 * We got a new configuration over the net.
	 *
	 * Disconnect from the current broker and stop the periodic timer.
	 *
	 * When the source of the new configuration is done, we will get notified
	 * via an event.
	 */
	if (state == MQTT_CLIENT_STATE_NEWCONFIG)
	{
		return;
	}

	state = MQTT_CLIENT_STATE_NEWCONFIG;

	etimer_stop(&publish_periodic_timer);
	mqtt_disconnect(&conn);
}

// TODO: We only verify payload and not payload size.

void pub_sleep_handler(uint8_t *payload, uint16_t len)
{
#if defined(NODE_GPS) || defined(NODE_4_ch_relay) || defined(NODE_1_ch_relay) || defined(NODE_HARD_LIGHT) || defined(NODE_LIGHT) || defined(NODE_STEP_MOTOR)
	// these nodes can't sleep so i decide to ignore this command
#else
	if(!memcmp(payload,"wake",4))
		no_sleep_allowed = 1;
	else if(!memcmp(payload,"sleep",5))
		no_sleep_allowed = 0;
#endif
}

static struct ctimer reset_timer;

static void
reset_now(void *not_used)
{
	asm volatile ("dmb 0xF":::"memory");
	asm volatile ("isb 0xF":::"memory");
	*((uint32_t *)0x400E1400) = 0xa500000D;
}

void pub_reset_handler(uint8_t *payload, uint16_t len)
{
	if(!memcmp(payload,"reset",5)){
		// Wait 5 sec before resetting, to allow broadcast
		ctimer_set(&reset_timer, 5*CLOCK_SECOND, reset_now, NULL);
	}
}

void Name_change(uint8_t *payload, uint16_t len)
{
	if((MQTT_CLIENT_CONFIG_EVENT_TYPE_ID_LEN > len) && strncmp(conf->Username,payload,sizeof(conf->Username))){
		memcpy(conf->Username, payload, len);
		process_post(PROCESS_BROADCAST, httpd_simple_event_new_config, NULL);
		new_net_config();
	}
}

void pub_christmas_light_handler(uint8_t *payload, uint16_t len)
{
	len -= 2;
	if(len>1)
		return;
	//TODO: check len so payload ONLINE dos'nt turn on
	if(!memcmp(payload,"ON",2))
		christmas_light.value(CH1_RELAY_ON);
	else if(!memcmp(payload,"OFF",3))
		christmas_light.value(CH1_RELAY_OFF);

	process_post(PROCESS_BROADCAST, Trig_sensors, NULL);
}

void pub_relay1_handler(uint8_t *payload, uint16_t len)
{
	len -= 2;
	if(len>1)
		return;
	//TODO: check len so payload ONLINE dos'nt turn on
	if(!memcmp(payload,"ON",2)){
#ifdef NODE_4_ch_relay
		ch4_relay_PD956.value(CH1_RELAY_ON);
#endif
#ifdef NODE_1_ch_relay
		ch1_relay_PD956.value(CH1_RELAY_ON);
#endif
	}else if(!memcmp(payload,"OFF",3)){
#ifdef NODE_4_ch_relay
		ch4_relay_PD956.value(CH1_RELAY_OFF);
#endif
#ifdef NODE_1_ch_relay
		ch1_relay_PD956.value(CH1_RELAY_OFF);
#endif
}
	process_post(PROCESS_BROADCAST, Trig_sensors, NULL);
}

void pub_relay2_handler(uint8_t *payload, uint16_t len)
{
	len -= 2;
	if(len>1)
		return;

	if(!memcmp(payload,"ON",2))
		ch4_relay_PD956.value(CH2_RELAY_ON);
	else if(!memcmp(payload,"OFF",3))
		ch4_relay_PD956.value(CH2_RELAY_OFF);

	process_post(PROCESS_BROADCAST, Trig_sensors, NULL);
}
void pub_relay3_handler(uint8_t *payload, uint16_t len)
{
	len -= 2;
	if(len>1)
		return;

	if(!memcmp(payload,"ON",2))
		ch4_relay_PD956.value(CH3_RELAY_ON);
	else if(!memcmp(payload,"OFF",3))
		ch4_relay_PD956.value(CH3_RELAY_OFF);

	process_post(PROCESS_BROADCAST, Trig_sensors, NULL);
}
void pub_relay4_handler(uint8_t *payload, uint16_t len)
{
	len -= 2;
	if(len>1)
		return;

	if(!memcmp(payload,"ON",2))
		ch4_relay_PD956.value(CH4_RELAY_ON);
	else if(!memcmp(payload,"OFF",3))
		ch4_relay_PD956.value(CH4_RELAY_OFF);

	process_post(PROCESS_BROADCAST, Trig_sensors, NULL);
}

void pub_light_hard_switch_handler(uint8_t *payload, uint16_t len)
{
	len -= 2;
	if(len>1)
		return;

	if(!memcmp(payload,"ON",2))
		hard_RGB_ctrl_sensor.configure(SENSORS_ACTIVE,7);
	else if(!memcmp(payload,"OFF",3))
		hard_RGB_ctrl_sensor.configure(SENSORS_ACTIVE,8);

	process_post(PROCESS_BROADCAST, Trig_sensors, NULL);
}

void pub_light_hard_brightness_handler(uint8_t *payload, uint16_t len)
{
	int brightness = strtol((char *)payload,NULL,10);
	RGB_hard_t tmp;

	if((brightness < 0) ||  (brightness > 256))
		return;

	tmp.all = ((RGB_hard_t *) hard_RGB_ctrl_sensor.value(SENSOR_ERROR))->all;
	tmp.led.brightness = brightness;
	hard_RGB_ctrl_sensor.value((int)&tmp.all);
}

void pub_light_hard_rgb_handler(uint8_t *payload, uint16_t len)
{
	uint16_t color[3];
	uint8_t color_index = 0;
	char chr;
	RGB_hard_t tmp;

	memset(color,0,sizeof(color));

	while(*payload)
	{
		chr = *payload++;
		if((chr < 0x30) || (chr > 0x39) || (chr == ',')){
			if(chr == ',')
				color_index++;
			continue;
		}
		color[color_index] *= 10;
		color[color_index] += chr-0x30;
	}

	if((color[0] > 256))	color[0] = 256;
	if((color[1] > 256))	color[1] = 256;
	if((color[2] > 256))	color[2] = 256;

	tmp.all = ((RGB_hard_t *) hard_RGB_ctrl_sensor.value(SENSOR_ERROR))->all;
	tmp.led.r = color[0];	tmp.led.r *=16;
	tmp.led.g = color[1];	tmp.led.g *=16;
	tmp.led.b = color[2];	tmp.led.b *=16;
	hard_RGB_ctrl_sensor.value((int)&tmp.all);
}

void pub_light_hard_effect_handler(uint8_t *payload, uint16_t len)
{
	if(!memcmp(payload,"colorloop",9) && (len == 9))
		hard_RGB_ctrl_sensor.configure(SENSORS_ACTIVE,10);
	else if(!memcmp(payload,"fire",4) && (len == 4))
		hard_RGB_ctrl_sensor.configure(SENSORS_ACTIVE,11);
	else if(!memcmp(payload,"rapid_red",9) && (len == 9))
		hard_RGB_ctrl_sensor.configure(SENSORS_ACTIVE,12);

	process_post(PROCESS_BROADCAST, Trig_sensors, NULL);
}
//////////////////////////////////////////////////////////////////////////////////
void pub_light_soft_switch_handler(uint8_t *payload, uint16_t len)
{
	len -= 2;
	if(len>1)
		return;

	if(!memcmp(payload,"ON",2))
		soft_RGB_ctrl_sensor.configure(SENSORS_ACTIVE,7);
	else if(!memcmp(payload,"OFF",3))
		soft_RGB_ctrl_sensor.configure(SENSORS_ACTIVE,8);

	process_post(PROCESS_BROADCAST, Trig_sensors, NULL);
}

void pub_light_soft_brightness_handler(uint8_t *payload, uint16_t len)
{
	int brightness = strtol((char *)payload,NULL,10);
	RGB_soft_t tmp;

	if((brightness < 0) ||  (brightness > 256))
		return;

	tmp.all = ((RGB_soft_t *) soft_RGB_ctrl_sensor.value(SENSOR_ERROR))->all;
	tmp.led.brightness = brightness;
	soft_RGB_ctrl_sensor.value((int) &tmp.all);
}

void pub_light_soft_rgb_handler(uint8_t *payload, uint16_t len)
{
	uint16_t color[3];
	uint8_t color_index = 0;
	char chr;
	RGB_soft_t tmp;

	memset(color,0,sizeof(color));

	while(*payload)
	{
		chr = *payload++;
		if((chr < 0x30) || (chr > 0x39) || (chr == ',')){
			if(chr == ',')
				color_index++;
			continue;
		}
		color[color_index] *= 10;
		color[color_index] += chr-0x30;
	}

	if((color[0] > 256))	color[0] = 256;
	if((color[1] > 256))	color[1] = 256;
	if((color[2] > 256))	color[2] = 256;

	tmp.all = ((RGB_soft_t *) soft_RGB_ctrl_sensor.value(SENSOR_ERROR))->all;
	tmp.led.r = color[0];
	tmp.led.g = color[1];
	tmp.led.b = color[2];
	soft_RGB_ctrl_sensor.value((int) &tmp.all);
}

void pub_light_soft_effect_handler(uint8_t *payload, uint16_t len)
{
	if(!memcmp(payload,"colorloop",9) && (len == 9))
		soft_RGB_ctrl_sensor.configure(SENSORS_ACTIVE,10);
	else if(!memcmp(payload,"fire",4) && (len == 4))
		soft_RGB_ctrl_sensor.configure(SENSORS_ACTIVE,11);
	else if(!memcmp(payload,"rapid_red",9) && (len == 9))
		soft_RGB_ctrl_sensor.configure(SENSORS_ACTIVE,12);

	process_post(PROCESS_BROADCAST, Trig_sensors, NULL);
}
//////////////////////////////////////////////////////////////////////////////////


static void pub_handler(const char *topic, uint16_t topic_len,
		const uint8_t *chunk, uint16_t chunk_len)
{
	DBG("Pub Handler: topic='%s' (len=%u), chunk_len=%u\n", topic, topic_len,
			chunk_len);


	subscribe_ele = list_head(MQTT_subscribe_list);

	while (subscribe_ele != NULL){

		if(!memcmp((void *)subscribe_ele->topic,(void *)topic,topic_len)){
			if(subscribe_ele->data_handler /*&& (topic_len == strlen(subscribe_ele->topic))*/)
				subscribe_ele->data_handler((uint8_t *)chunk,chunk_len);
		}
		subscribe_ele = subscribe_ele->next;
	}

}
/*---------------------------------------------------------------------------*/
static void mqtt_event(struct mqtt_connection *m, mqtt_event_t event,
		void *data)
{
	switch (event)
	{
		case MQTT_EVENT_CONNECTED:
			PRINTF("APP - Application has a MQTT connection\n");
			timer_set(&connection_life, CONNECTION_STABLE_TIME);
			state = MQTT_CLIENT_STATE_CONNECTED;
			process_poll(&mqtt_client_process);
			break;

		case MQTT_EVENT_DISCONNECTED:
			PRINTF("APP - MQTT Disconnect. Reason %u\n", *((mqtt_event_t *)data));

			/* Do nothing if the disconnect was the result of an incoming config */
			if (state != MQTT_CLIENT_STATE_NEWCONFIG)
			{
				state = MQTT_CLIENT_STATE_DISCONNECTED;
				process_poll(&mqtt_client_process);
			}
			break;

		case MQTT_EVENT_PUBLISH:
			msg_ptr = data;

			/* Implement first_flag in publish message? */
			if (msg_ptr->first_chunk)
			{
				msg_ptr->first_chunk = 0;

				PRINTF("APP - Application received a publish on topic '%s'. Payload "
						"size is %i bytes. Content:\n\n",
						msg_ptr->topic, msg_ptr->payload_length);
			}

			pub_handler(msg_ptr->topic, strlen(msg_ptr->topic),
					msg_ptr->payload_chunk, msg_ptr->payload_length);
			break;

		case MQTT_EVENT_SUBACK:
			PRINTF("APP - Application is subscribed to topic successfully\n");
			break;

		case MQTT_EVENT_UNSUBACK:
			PRINTF("APP - Application is unsubscribed to topic successfully\n");
			break;

		case MQTT_EVENT_PUBACK:
			state = MQTT_CLIENT_STATE_PUBLISHING_DONE;
			process_post(PROCESS_BROADCAST, MQTT_publish_sensor_data_done_event, NULL);
			//PRINTF("APP - Publishing complete.\n");
			break;

		default:
			 PRINTF("APP - Application got a unhandled MQTT event: %i\n", event); //DBG
			break;
	}
}
/*---------------------------------------------------------------------------*/
// TODO: this will only work for sensors. NEDAFIX.
static int construct_pub_topic(void)
{
	int len = snprintf(pub_topic, BUFFER_SIZE, "Hass/common/%s/%s/state",
			client_id, conf->Username);

	/* len < 0: Error. Len >= BUFFER_SIZE: Buffer too small */
	if (len < 0 || len >= BUFFER_SIZE)
	{
		printf("Pub Topic: %d, Buffer %d\n", len, BUFFER_SIZE);
		return 0;
	}

	return 1;
}


/*---------------------------------------------------------------------------*/
// TODO: for debug purpose this information is stored. Rewrite this to only
// happen when needed so only one buffer is used.
static int construct_configs(void)
{
	for (reading = MQTT_sensor_first(); reading != NULL; reading = reading->next)
	{
		if(reading->component_topic_config == NULL)
			continue;

		snprintf(reading->MQTT_config_ele.topic,sizeof(reading->MQTT_config_ele.topic),
							reading->component_topic_config,
							reading->descr,
							linkaddr_node_addr.u8[0], linkaddr_node_addr.u8[1],
							linkaddr_node_addr.u8[2], linkaddr_node_addr.u8[5],
							linkaddr_node_addr.u8[6], linkaddr_node_addr.u8[7]);

		switch(reading->HASS_class){
			case sensor_class:
				snprintf(reading->MQTT_config_ele.arg, sizeof(reading->MQTT_config_ele.arg),
									"{\"name\": \"%s %s\","
									"\"stat_t\": \"%s\","
									"\"unit_of_meas\": \"%s\","
									"\"exp_aft\":%d,"
									"\"val_tpl\":\"{{ value_json.%s}}\" }",
									conf->Username, reading->descr, 	//name
									pub_topic,  						//state_topic
									reading->units,						//unit_of_measurement
									(int)(conf->pub_interval/CLOCK_SECOND)*2 + 10,
									reading->descr); 					//value_template
				break;

			case switch_class:
				snprintf(reading->MQTT_config_ele.arg, sizeof(reading->MQTT_config_ele.arg),
									"{\"name\": \"%s %s\","
									"\"stat_t\": \"%s\","
									"\"val_tpl\":\"{{ value_json.%s}}\","
									"\"cmd_t\": \"Hass/switch/%s/%s/set\"}",
									conf->Username,reading->descr, 		//name
									pub_topic, 							//state_topic
									reading->descr,						//value_template
									client_id, reading->descr);  //command_topic
				break;

			case cover_class:
				break;

			case binary_sensor_class:

				snprintf(reading->MQTT_config_ele.arg, sizeof(reading->MQTT_config_ele.arg),
									"{\"name\": \"%s %s\","
									"\"stat_t\": \"%s\","
									"\"dev_cla\": \"%s\","
									"\"val_tpl\":\"{{ value_json.%s}}\"}",
									conf->Username,reading->descr, 		//name
									pub_topic, 							//state_topic
									DEVICE_CLASS_lookup[reading->device_class],
									reading->descr);						//value_template

				break;

			case light_class:
				snprintf(reading->MQTT_config_ele.arg, sizeof(reading->MQTT_config_ele.arg),
									"{\"name\":\"%s %s\","
									"\"stat_t\":\"%s\","
									"\"cmd_t\":\"Hass/light/%s/%s/set\","
									"\"bri_stat_t\":\"%s\","
									"\"bri_cmd_t\":\"Hass/light/%s/brightness/set\","  // NB: brightness hardcoded. must be reading->descr of brightness element
									"\"rgb_stat_t\":\"%s\","
									"\"rgb_cmd_t\":\"Hass/light/%s/color/set\","   // NB: rgb hardcoded. must be reading->descr of rgb element
									"\"fx_stat_t\":\"%s\","
									"\"fx_cmd_t\":\"Hass/light/%s/effect/set\","   // NB: effect hardcoded. must be reading->descr of effect element
									"\"stat_val_tpl\":\"{{value_json.switch}}\","
									"\"bri_val_tpl\":\"{{value_json.brightness}}\","
									"\"fx_val_tpl\":\"{{value_json.effect}}\","
									"\"fx_list\":[\"colorloop\",\"fire\",\"rapid_red\"],"
									"\"rgb_val_tpl\":\"{{value_json.color|join(',')}}\"}",
									conf->Username,reading->descr, 		//name
									pub_topic, //state_topic
									client_id, reading->descr,  //command_topic
									pub_topic,  //brightness status
									client_id,  //brightness set
									pub_topic,  //color status
									client_id,	//color set
									pub_topic,  //effect status
									client_id);	//effect set


				break;

			default: break;
		}


		list_add(MQTT_config_list, &reading->MQTT_config_ele);
	}


	//--------------------------------------------------------------------------------------------
	return 1;
}
//--------------------------------------------------------------------------------------------

static int construct_sub_topic(void)
{
	// Common commands ////////////////////////////////////////////////////////////
	snprintf(MQTT_COMMON_NO_SLEEP_sub_cmd.topic,
				sizeof(MQTT_COMMON_NO_SLEEP_sub_cmd.topic), "Hass/power/%s/%s/set",
				client_id, "Sleep");
	MQTT_COMMON_NO_SLEEP_sub_cmd.data_handler = pub_sleep_handler;
	list_add(MQTT_subscribe_list, &MQTT_COMMON_NO_SLEEP_sub_cmd);

	snprintf(MQTT_COMMON_RESET_sub_cmd.topic,
				sizeof(MQTT_COMMON_RESET_sub_cmd.topic), "Hass/power/%s/%s/set",
				client_id, "Reset");
	MQTT_COMMON_RESET_sub_cmd.data_handler = pub_reset_handler;
	list_add(MQTT_subscribe_list, &MQTT_COMMON_RESET_sub_cmd);

	snprintf(MQTT_COMMON_NAME_sub_cmd.topic,
				sizeof(MQTT_COMMON_NAME_sub_cmd.topic), "Hass/power/%s/%s/set",
				client_id, "Name");
	MQTT_COMMON_NAME_sub_cmd.data_handler = Name_change;
	list_add(MQTT_subscribe_list, &MQTT_COMMON_NAME_sub_cmd);


	//////////////////////////////////////////////////////////////////////////////

	for (reading = MQTT_sensor_first(); reading != NULL; reading = reading->next)
	{
		if(reading->component_topic_sub == NULL)
			continue;
					//ex: "Hass/light/%s/%s/set"
		snprintf(reading->MQTT_subscr_ele.topic,sizeof(reading->MQTT_subscr_ele.topic),
									reading->component_topic_sub,
									client_id, reading->descr);

		list_add(MQTT_subscribe_list, &reading->MQTT_subscr_ele);

	}


/*
	snprintf(MQTT_step_motor_sub_cmd.topic,
			sizeof(MQTT_step_motor_sub_cmd.topic), "Hass/%s/%s/%s/%s/set",
			"cover", client_id, conf->Username, "Step-Position");
	list_add(MQTT_subscribe_list, &MQTT_step_motor_sub_cmd);
//--------------------------------------------------------------------------------------------
	snprintf(MQTT_step_motor_sub_tilt.topic,
			sizeof(MQTT_step_motor_sub_tilt.topic), "Hass/%s/%s/%s/%s/tilt",
			"cover", client_id, conf->Username, "Step-Position");
	list_add(MQTT_subscribe_list, &MQTT_step_motor_sub_tilt);
//--------------------------------------------------------------------------------------------
	int len = snprintf(sub_topic, BUFFER_SIZE, "Hass/%s/%s/%s/set", "sensor",
			client_id, conf->Username);

	// len < 0: Error. Len >= BUFFER_SIZE: Buffer too small
	if (len < 0 || len >= BUFFER_SIZE)
	{
		printf("Sub Topic: %d, Buffer %d\n", len, BUFFER_SIZE);
		return 0;
	}
*/
	return 1;
}
/*---------------------------------------------------------------------------*/
static int construct_client_id(void)
{
	int len;
	/*
	 len = snprintf(client_id, BUFFER_SIZE,
			"d:%s:%s:%02x%02x%02x%02x%02x%02x", conf->Company, conf->Modul_type,
			linkaddr_node_addr.u8[0], linkaddr_node_addr.u8[1],
			linkaddr_node_addr.u8[2], linkaddr_node_addr.u8[5],
			linkaddr_node_addr.u8[6], linkaddr_node_addr.u8[7]); //TODO: use PD snr instead as id
	*/
	memcpy(client_id,(void *)device_certificate.crt.snr,device_certificate.crt.snlen);
	len = device_certificate.crt.snlen;
	/* len < 0: Error. Len >= BUFFER_SIZE: Buffer too small */
	if (len < 0 || len >= BUFFER_SIZE)
	{
		printf("Client ID: %d, Buffer %d\n", len, BUFFER_SIZE);
		return 0;
	}

	return 1;
}
/*---------------------------------------------------------------------------*/
static void update_config(void)
{
	if (construct_client_id() == 0)
	{
		/* Fatal error. Client ID larger than the buffer */
		state = MQTT_CLIENT_STATE_CONFIG_ERROR;
		return;
	}

	if (construct_sub_topic() == 0)
	{
		/* Fatal error. Topic larger than the buffer */
		state = MQTT_CLIENT_STATE_CONFIG_ERROR;
		return;
	}

	if (construct_pub_topic() == 0)
	{
		/* Fatal error. Topic larger than the buffer */
		state = MQTT_CLIENT_STATE_CONFIG_ERROR;
		return;
	}
	construct_configs();
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
static int init_config()
{
	/* Populate configuration with default values */
	memset(conf, 0, sizeof(mqtt_client_config_t));

	memcpy(conf->Company, DEFAULT_ORG_ID,
			sizeof(DEFAULT_ORG_ID));
	memcpy(conf->Modul_type, DEFAULT_TYPE_ID,
			sizeof(DEFAULT_TYPE_ID));
	memcpy(conf->Username, DEFAULT_EVENT_TYPE_ID, 7);
	memcpy(conf->broker_ip, broker_ip, strlen(broker_ip));

	conf->broker_port = DEFAULT_BROKER_PORT;
	conf->pub_interval = DEFAULT_PUBLISH_INTERVAL;

	return 1;
}
/*---------------------------------------------------------------------------*/

static void publish(void)
{
	int len;
	int remaining = APP_BUFFER_SIZE;



	seq_nr_value++;

	buf_ptr = app_buffer;

	len = snprintf(buf_ptr, remaining, "{"
			"\"Board\":\"%s\","
			"\"Node\":\"%s\","
			"\"Name\":\"%s\","
			"\"Seq #\":%d,"
			"\"Uptime [s]\":%lu"
			/*"\"Battery state\":%c,"
			"\"Unix [s]\":%lu"*/
	,device_certificate.crt.modul, SENSOR_STRING, conf->Username, seq_nr_value, clock_seconds()/*,RF231_bat_status()+0x30,clock_get_unix_time()*/);

	if (len < 0 || len >= remaining)
	{
		printf("Buffer too short. Have %d, need %d + \\0\n", remaining, len);
		return;
	}

	remaining -= len;
	buf_ptr += len;

	for (reading = MQTT_sensor_first(); reading != NULL; reading = reading->next)
	{
		if (reading->publish && reading->raw != SENSOR_ERROR)
		{
			len = snprintf(buf_ptr, remaining, ",\"%s\":%s", reading->descr, reading->converted);

			if ((len < 0) || (len >= remaining))
			{
				printf("Buffer too short. Have %d, need %d + \\0\n", remaining,	len);
				return;
			}
			remaining -= len;
			buf_ptr += len;
		}
	}

	len = snprintf(buf_ptr, remaining, "}");

	if (len < 0 || len >= remaining)
	{
		printf("Buffer too short. Have %d, need %d + \\0\n", remaining, len);
		return;
	}
	// There is no need to supply 12 mA into the radio when measuring the temperature.
	//NETSTACK_RADIO.on(); // yes there is. we are not alone
	mqtt_publish(&conn, NULL, pub_topic, (uint8_t *) app_buffer,
			strlen(app_buffer), MQTT_QOS_LEVEL_1, MQTT_RETAIN_OFF);

	DBG("APP - Publish!\n");
}
/*---------------------------------------------------------------------------*/
static void connect_to_broker(void)
{
	/* Connect to MQTT server */
	mqtt_status_t conn_attempt_result = mqtt_connect(&conn, conf->broker_ip,
			conf->broker_port, conf->pub_interval * 3);

	if (conn_attempt_result == MQTT_STATUS_OK)
	{
		state = MQTT_CLIENT_STATE_CONNECTING;
	}
	else
	{
		state = MQTT_CLIENT_STATE_CONFIG_ERROR;
	}
}
/*---------------------------------------------------------------------------*/
static MQTT_config_ele_t *hass_config;
static MQTT_config_ele_t *subscribe_topic;
static void state_machine(void)
{

	switch (state)
	{
	case MQTT_CLIENT_STATE_INIT:
		mqtt_register(&conn, &mqtt_client_process, client_id, mqtt_event,
		MQTT_CLIENT_MAX_SEGMENT_SIZE);

		hass_config = list_head(MQTT_config_list);
		subscribe_topic = list_head(MQTT_subscribe_list);

		if (strlen(conf->MQTT_user_Password) == 0
				|| strlen(conf->MQTT_user_name) == 0)
		{
			printf("User name or password is empty\n");
		}
		else
		{
			mqtt_set_username_password(&conn, conf->MQTT_user_name,
					conf->MQTT_user_Password);
		}

		conn.auto_reconnect = 0;
		connect_attempt = 1;

		memset(&def_route, 0, sizeof(def_route));

		state = MQTT_CLIENT_STATE_REGISTERED;
		DBG("Init\n");
		/* Continue */
	case MQTT_CLIENT_STATE_REGISTERED:
		if (uip_ds6_get_global(ADDR_PREFERRED) != NULL)
		{
			/* Registered and with a public IP. Connect */
			DBG("Registered. Connect attempt %u\n", connect_attempt);
			connect_to_broker();
		}
		// TODO: HOTFIX. I don't use retain flag for config, so
		// It must be sent when reconnection.


		etimer_set(&publish_periodic_timer,
				NET_CONNECT_PERIODIC);
		return;

	case MQTT_CLIENT_STATE_CONNECTING:
		/* Not connected yet. Wait */
		DBG("Connecting (%u)\n", connect_attempt);
		break;

	case MQTT_CLIENT_STATE_CONNECTED:

		state = MQTT_CLIENT_STATE_SUBSCRIBING;
		subscribe_ele = list_head(MQTT_subscribe_list);
		break;

	case MQTT_CLIENT_STATE_SUBSCRIBING:

		if (subscribe_topic != NULL)
		{
			if (mqtt_ready(&conn) && conn.out_buffer_sent)
			{
				mqtt_subscribe(&conn, NULL, subscribe_topic->topic,	MQTT_QOS_LEVEL_0);
				subscribe_topic = subscribe_topic->next;

				break;
			}
		}
		else
		{
			state = MQTT_CLIENT_STATE_PUBLISHING;
		}
		break;

	case MQTT_CLIENT_STATE_PUBLISHING:
	case MQTT_CLIENT_STATE_PUBLISHING_DONE:
		state = MQTT_CLIENT_STATE_PUBLISHING;
		/* If the timer expired, the connection is stable. */
		if (timer_expired(&connection_life))
		{
			/*
			 * Intentionally using 0 here instead of 1: We want RECONNECT_ATTEMPTS
			 * attempts if we disconnect after a successful connect
			 */
			connect_attempt = 0;
		}

		if (mqtt_ready(&conn) && conn.out_buffer_sent)
		{
			if(hass_config != NULL){
				mqtt_publish(&conn, NULL, hass_config->topic, (uint8_t *) hass_config->arg,
							strlen(hass_config->arg), MQTT_QOS_LEVEL_0, MQTT_RETAIN_ON);
				hass_config = hass_config->next;
				break;
			}
			/* Connected. Publish */
			publish();
			//etimer_set(&publish_periodic_timer, conf->pub_interval);
			DBG("Publishing\n");
			/* Return here so we don't end up rescheduling the timer */
			return;
		}
		else
		{
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
		if (connect_attempt < RECONNECT_ATTEMPTS ||
		RECONNECT_ATTEMPTS == RETRY_FOREVER)
		{
			/* Disconnect and backoff */
			clock_time_t interval;
			mqtt_disconnect(&conn);
			connect_attempt++;

			interval = connect_attempt < 3 ?
			RECONNECT_INTERVAL << connect_attempt :
												RECONNECT_INTERVAL << 3;

			DBG("Disconnected. Attempt %u in %lu ticks\n", connect_attempt, interval);

			etimer_set(&publish_periodic_timer, interval);

			state = MQTT_CLIENT_STATE_REGISTERED;
			return;
		}
		else
		{
			/* Max reconnect attempts reached. Enter error state */
			state = MQTT_CLIENT_STATE_ERROR;
			DBG("Aborting connection after %u attempts\n", connect_attempt - 1);
		}
		break;
	case MQTT_CLIENT_STATE_NEWCONFIG:
		/* Only update config after we have disconnected or in the case of an error */
		if (conn.state == MQTT_CONN_STATE_NOT_CONNECTED
				|| conn.state == MQTT_CONN_STATE_ERROR)
		{
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
		/*
		 * 'default' should never happen.
		 *
		 * If we enter here it's because of some error. Stop timers. The only thing
		 * that can bring us out is a new config event
		 */
		printf("Default case: State=0x%02x\n", state);
		state = MQTT_CLIENT_STATE_ERROR;
		return;
	}

	/* If we didn't return so far, reschedule ourselves */
	etimer_set(&publish_periodic_timer, STATE_MACHINE_PERIODIC);
}

uint8_t MQTT_init_config(void)
{
	conf = &web_demo_config.mqtt_config;
	init_config();
	return 0;
}

uint8_t MQTT_ready(void)
{
	return mqtt_ready(&conn);
}
extern int rtimer_arch_sleep(rtimer_clock_t howlong);
/*---------------------------------------------------------------------------*/
PROCESS_THREAD(mqtt_client_process, ev, data)
{

	PROCESS_BEGIN();
	static uint8_t sleep_counter = 1;

	printf("%s: starting\n",mqtt_client_process.name);

	MQTT_publish_sensor_data_done_event = process_alloc_event();

	update_config();
	/* Main loop */
	while (1)
	{

		PROCESS_YIELD();

		if ((ev == MQTT_publish_sensor_data_done_event) || (ev == PROCESS_EVENT_TIMER && data == &sleep_retry_timer) || (ev == PROCESS_EVENT_TIMER && data == &timeout_timer))
		{
			// If for some reason the chain breaks we need to just sleep on it.
			// Timeout_timer makes sure we get here 1s after wake up.
			etimer_stop(&timeout_timer);
			if(no_sleep_allowed || NTP_status() || !sleep_counter || (ev == PROCESS_EVENT_TIMER && data == &timeout_timer) || ((state!=MQTT_CLIENT_STATE_PUBLISHING)&& (state!=MQTT_CLIENT_STATE_PUBLISHING_DONE)))
			{

				if(sleep_counter){
					sleep_counter = 0;
					etimer_set(&sleep_retry_timer, conf->pub_interval);
					//PRINTF("MQTT: can't sleep\n");
					NETSTACK_RADIO.on(); // Just to be sure. otherwise we end in a radio silence mode. This only happens if we get an error in sensor measurement.
				}else{
					sleep_counter = 1;
					etimer_set(&timeout_timer, 5*CLOCK_SECOND);
					//PRINTF("MQTT: Trig from no sleep\n");
					process_post(PROCESS_BROADCAST, Trig_sensors, NULL);
				}

			}else
			{
				if(NETSTACK_RADIO.sleep() !=-1){

					rtimer_arch_sleep(conf->pub_interval/CLOCK_SECOND * RTIMER_ARCH_SECOND); // 54uA in wait mode (cpu + extern flash) + sensor
					// if sleeptime is 60 sec, and we are awake 100ms we can live on a battery with 2700mAh for
					// 60/60.1*54uA+0.1/60.1*22mA = 90.5uA avg  2700mA/90.5uA = 29829hr ~ 3.4 years
					NETSTACK_RADIO.on();
					// TODO: Need timing to indicate how long the radio is on.
					// A ping is about 47ms in avg. so if we assume 100ms on-time we get:
					// 60/120.1*54uA + 0.1/120.1*22mA  = 70.5uA avg  2700mA/70uA = 38571hr ~ 4.4 years
					//PRINTF("MQTT: Just woke up, trig\n");

					etimer_set(&timeout_timer, 5*CLOCK_SECOND); // We have 5 sec to complete sensor measurement and publish result. Otherwise we will treat it as nosleep.
					process_post(PROCESS_BROADCAST, Trig_sensors, NULL);
				}
				else
				{
					PRINTF("MQTT: Failed to put radio to sleep\n");
					etimer_set(&sleep_retry_timer, CLOCK_SECOND>>6); // retry in 15 ms
				}
			}
		}

		if (state == MQTT_CLIENT_STATE_ERROR)
		{
			connect_attempt = 1;
			state = MQTT_CLIENT_STATE_REGISTERED;
		}

		if ((ev == PROCESS_EVENT_TIMER && data == &publish_periodic_timer)
				|| ev == PROCESS_EVENT_POLL
				|| ev == MQTT_publish_sensor_data_event /*||
				 (ev == sensors_event)*/)
		{

			state_machine();
		}

		if (ev == httpd_simple_event_new_config)
		{
			/*
			 * Schedule next pass in a while. When HTTPD sends us this event, it is
			 * also in the process of sending the config page. Wait a little before
			 * reconnecting, so as to not cause congestion.
			 */
			etimer_set(&publish_periodic_timer, NEW_CONFIG_WAIT_INTERVAL);
		}

		if (ev == load_config_defaults)
		{
			init_config();
			connect_to_broker();
		}

	}

	PROCESS_END();
}
/*---------------------------------------------------------------------------*/
/**
 * @}
 */
