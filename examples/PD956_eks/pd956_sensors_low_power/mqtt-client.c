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

#include <string.h>
#include <strings.h>
/*---------------------------------------------------------------------------*/
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

typedef struct MQTT_config_ele
{
	struct MQTT_config_ele *next;
	char topic[BUFFER_SIZE];
	char arg[BUFFER_SIZE * 10];
} MQTT_config_ele_t;

typedef struct MQTT_sub_ele
{
	struct MQTT_sub_ele *next;
	char topic[BUFFER_SIZE];
} MQTT_sub_ele_t;

#if defined(NODE_STEP_MOTOR) || defined(NODE_4_ch_relay)
MQTT_config_ele_t MQTT_dht11_temperature_config;
MQTT_config_ele_t MQTT_dht11_humidity_config;
#endif
#ifdef NODE_HTU21D
MQTT_config_ele_t MQTT_htu21_temperature_config;
MQTT_config_ele_t MQTT_htu21_humidity_config;
MQTT_config_ele_t MQTT_htu21_interneltmp_config;
#endif
#ifdef NODE_STEP_MOTOR
MQTT_config_ele_t MQTT_step_motor_config;
MQTT_sub_ele_t MQTT_step_motor_sub_cmd;
MQTT_sub_ele_t MQTT_step_motor_sub_tilt;
#endif

#ifdef NODE_PRESSURE
MQTT_config_ele_t MQTT_bm280_temperature_config;
MQTT_config_ele_t MQTT_bm280_pressure_config;
MQTT_config_ele_t MQTT_htu21_interneltmp_config;
#endif
static MQTT_sub_ele_t *subscribe_ele;
MQTT_sub_ele_t MQTT_COMMON_NO_SLEEP_sub_cmd;
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
/* Parent RSSI functionality */
extern int def_rt_rssi;
/*---------------------------------------------------------------------------*/
const static cc26xx_web_demo_sensor_reading_t *reading;
/*---------------------------------------------------------------------------*/
mqtt_client_config_t *conf;
process_event_t MQTT_publish_sensor_data_done_event;
static volatile uint8_t no_sleep_allowed = 0;
/*---------------------------------------------------------------------------*/
PROCESS(mqtt_client_process, "CC26XX MQTT Client");
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

static void pub_handler(const char *topic, uint16_t topic_len,
		const uint8_t *chunk, uint16_t chunk_len)
{
	DBG("Pub Handler: topic='%s' (len=%u), chunk_len=%u\n", topic, topic_len,
			chunk_len);

	/* If we don't like the length, ignore */
	if (chunk_len != 4 || chunk_len != 5)
	{
		printf("Incorrect topic or chunk len. Ignored\n");
		return;
	}

	static MQTT_config_ele_t *sub_topic;
	sub_topic = list_head(MQTT_subscribe_list);

	while (sub_topic != NULL){
		if(!memcpy((void *)sub_topic->topic,(void *)topic,topic_len)){
			if(!memcpy(chunk,"wake",4))
				no_sleep_allowed = 1;
			else if(!memcpy((void *)chunk,"sleep",5))
				no_sleep_allowed = 0;
		}
		sub_topic = sub_topic->next;
	}

}
/*---------------------------------------------------------------------------*/
static void mqtt_event(struct mqtt_connection *m, mqtt_event_t event,
		void *data)
{
	switch (event)
	{
	case MQTT_EVENT_CONNECTED:
	{
		DBG("APP - Application has a MQTT connection\n");
		timer_set(&connection_life, CONNECTION_STABLE_TIME);
		state = MQTT_CLIENT_STATE_CONNECTED;
		process_poll(&mqtt_client_process);
		break;
	}
	case MQTT_EVENT_DISCONNECTED:
	{
		DBG("APP - MQTT Disconnect. Reason %u\n", *((mqtt_event_t *)data));

		/* Do nothing if the disconnect was the result of an incoming config */
		if (state != MQTT_CLIENT_STATE_NEWCONFIG)
		{
			state = MQTT_CLIENT_STATE_DISCONNECTED;
			process_poll(&mqtt_client_process);
		}
		break;
	}
	case MQTT_EVENT_PUBLISH:
	{
		msg_ptr = data;

		/* Implement first_flag in publish message? */
		if (msg_ptr->first_chunk)
		{
			msg_ptr->first_chunk = 0;

			DBG("APP - Application received a publish on topic '%s'. Payload "
					"size is %i bytes. Content:\n\n",
					msg_ptr->topic, msg_ptr->payload_length);
		}

		pub_handler(msg_ptr->topic, strlen(msg_ptr->topic),
				msg_ptr->payload_chunk, msg_ptr->payload_length);
		break;
	}
	case MQTT_EVENT_SUBACK:
	{
		DBG("APP - Application is subscribed to topic successfully\n");
		break;
	}
	case MQTT_EVENT_UNSUBACK:
	{
		DBG("APP - Application is unsubscribed to topic successfully\n");
		break;
	}
	case MQTT_EVENT_PUBACK:
	{
		state = MQTT_CLIENT_STATE_PUBLISHING_DONE;
		process_post(PROCESS_BROADCAST, MQTT_publish_sensor_data_done_event, NULL);
		DBG("APP - Publishing complete.\n");
		break;
	}
	default:
		DBG("APP - Application got a unhandled MQTT event: %i\n", event);
		break;
	}
}
/*---------------------------------------------------------------------------*/
static int construct_pub_topic(void)
{
	int len = snprintf(pub_topic, BUFFER_SIZE, "Hass/%s/%s/%s/state", "sensor",
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
static int construct_configs(void)
{
	//--------------------------------------------------------------------------------------------
	snprintf(MQTT_htu21_interneltmp_config.topic,
					sizeof(MQTT_htu21_interneltmp_config.topic),
					"Hass/%s/%s%02x%02x%02x%02x%02x%02x/config", "sensor","sensorItemp",
					linkaddr_node_addr.u8[0], linkaddr_node_addr.u8[1],
					linkaddr_node_addr.u8[2], linkaddr_node_addr.u8[5],
					linkaddr_node_addr.u8[6], linkaddr_node_addr.u8[7]);

	snprintf(MQTT_htu21_interneltmp_config.arg,
			sizeof(MQTT_htu21_interneltmp_config.arg),
			"{\"name\": \"%s Internal temperature\","
			"\"state_topic\": \"Hass/%s/%s/%s/state\","
			"\"unit_of_measurement\": \"°C\","
			"\"value_template\":\"{{ value_json.%s}}\" }",
			conf->Username,"sensor", client_id, conf->Username, "Internaltemperature"); // Last must be equal reading->descr

	list_add(MQTT_config_list, &MQTT_htu21_interneltmp_config);
	//--------------------------------------------------------------------------------------------
#ifdef NODE_HTU21D
	snprintf(MQTT_htu21_temperature_config.topic,
			sizeof(MQTT_htu21_temperature_config.topic),
			"Hass/%s/%s%02x%02x%02x%02x%02x%02x/config", "sensor","sensorTemperature",
			linkaddr_node_addr.u8[0], linkaddr_node_addr.u8[1],
			linkaddr_node_addr.u8[2], linkaddr_node_addr.u8[5],
			linkaddr_node_addr.u8[6], linkaddr_node_addr.u8[7]);


	snprintf(MQTT_htu21_temperature_config.arg,
			sizeof(MQTT_htu21_temperature_config.arg),
			"{\"name\": \"%s Temperature\","
			" \"state_topic\": \"Hass/%s/%s/%s/state\","
			" \"unit_of_measurement\": \"C\","
			" \"value_template\": \"{{ value_json.%s}}\" }",
			conf->Username, "sensor", client_id, conf->Username, "Temperature"); // Last must be equal reading->descr

	list_add(MQTT_config_list, &MQTT_htu21_temperature_config);
	//--------------------------------------------------------------------------------------------
	snprintf(MQTT_htu21_humidity_config.topic,
				sizeof(MQTT_htu21_temperature_config.topic),
				"Hass/%s/%s%02x%02x%02x%02x%02x%02x/config", "sensor","sensorHumidity",
				linkaddr_node_addr.u8[0], linkaddr_node_addr.u8[1],
				linkaddr_node_addr.u8[2], linkaddr_node_addr.u8[5],
				linkaddr_node_addr.u8[6], linkaddr_node_addr.u8[7]);

	snprintf(MQTT_htu21_humidity_config.arg,
			sizeof(MQTT_htu21_humidity_config.arg),
			"{\"name\": \"%s Humidity\","
			" \"state_topic\": \"Hass/%s/%s/%s/state\","
			" \"unit_of_measurement\": \"%%\","
			" \"value_template\": \"{{ value_json.%s}}\" }",
			conf->Username, "sensor", client_id, conf->Username, "Humidity"); // Last must be equal reading->descr

	list_add(MQTT_config_list, &MQTT_htu21_humidity_config);
#endif

#ifdef NODE_PRESSURE
	snprintf(MQTT_bm280_temperature_config.topic,
			sizeof(MQTT_bm280_temperature_config.topic),
			"Hass/%s/%s%02x%02x%02x%02x%02x%02x/config", "sensor","sensorTemperature",
			linkaddr_node_addr.u8[0], linkaddr_node_addr.u8[1],
			linkaddr_node_addr.u8[2], linkaddr_node_addr.u8[5],
			linkaddr_node_addr.u8[6], linkaddr_node_addr.u8[7]);


	snprintf(MQTT_bm280_temperature_config.arg,
			sizeof(MQTT_bm280_temperature_config.arg),
			"{\"name\": \"%s Temperature\","
			" \"state_topic\": \"Hass/%s/%s/%s/state\","
			" \"unit_of_measurement\": \"C\","
			" \"value_template\": \"{{ value_json.%s}}\" }",
			conf->Username, "sensor", client_id, conf->Username, "Temperature"); // Last must be equal reading->descr

	list_add(MQTT_config_list, &MQTT_bm280_temperature_config);
	//--------------------------------------------------------------------------------------------
	snprintf(MQTT_bm280_pressure_config.topic,
				sizeof(MQTT_bm280_pressure_config.topic),
				"Hass/%s/%s%02x%02x%02x%02x%02x%02x/config", "sensor","sensorPressure",
				linkaddr_node_addr.u8[0], linkaddr_node_addr.u8[1],
				linkaddr_node_addr.u8[2], linkaddr_node_addr.u8[5],
				linkaddr_node_addr.u8[6], linkaddr_node_addr.u8[7]);

	snprintf(MQTT_bm280_pressure_config.arg,
			sizeof(MQTT_bm280_pressure_config.arg),
			"{\"name\": \"%s Pressure\","
			" \"state_topic\": \"Hass/%s/%s/%s/state\","
			" \"unit_of_measurement\": \"hPa\","
			" \"value_template\": \"{{ value_json.%s}}\" }",
			conf->Username, "sensor", client_id, conf->Username, "Pressure"); // Last must be equal reading->descr

	list_add(MQTT_config_list, &MQTT_bm280_pressure_config);
#endif
	//--------------------------------------------------------------------------------------------
/*
	snprintf(MQTT_step_motor_config.topic, sizeof(MQTT_step_motor_config.topic),
			"Hass/%s/%s/%s/%s/config", "cover", client_id, conf->Username,
			"Step-Position");

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
					" }", conf->Username, "cover", client_id, conf->Username,
			"Step-Position", "cover", client_id, conf->Username,
			"Step-Position", "Step-Position", "cover", client_id,
			conf->Username, "Step-Position", "cover", client_id, conf->Username,
			"Step-Position"); // Last must be equal reading->descr

	list_add(MQTT_config_list, &MQTT_step_motor_config);*/
	return 1;
}
//--------------------------------------------------------------------------------------------

static int construct_sub_topic(void)
{
	snprintf(MQTT_COMMON_NO_SLEEP_sub_cmd.topic,
				sizeof(MQTT_COMMON_NO_SLEEP_sub_cmd.topic), "Hass/%s/%s/%s/set",
				client_id, conf->Username, "Sleep");
		list_add(MQTT_subscribe_list, &MQTT_COMMON_NO_SLEEP_sub_cmd);

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
	int len = snprintf(client_id, BUFFER_SIZE,
			"d:%s:%s:%02x%02x%02x%02x%02x%02x", conf->Company, conf->Modul_type,
			linkaddr_node_addr.u8[0], linkaddr_node_addr.u8[1],
			linkaddr_node_addr.u8[2], linkaddr_node_addr.u8[5],
			linkaddr_node_addr.u8[6], linkaddr_node_addr.u8[7]);

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

	construct_configs();

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

	memcpy(conf->Company, CC26XX_WEB_DEMO_DEFAULT_ORG_ID,
			sizeof(CC26XX_WEB_DEMO_DEFAULT_ORG_ID));
	memcpy(conf->Modul_type, CC26XX_WEB_DEMO_DEFAULT_TYPE_ID,
			sizeof(CC26XX_WEB_DEMO_DEFAULT_TYPE_ID));
	memcpy(conf->Username, CC26XX_WEB_DEMO_DEFAULT_EVENT_TYPE_ID, 7);
	memcpy(conf->broker_ip, broker_ip, strlen(broker_ip));

	conf->broker_port = CC26XX_WEB_DEMO_DEFAULT_BROKER_PORT;
	conf->pub_interval = CC26XX_WEB_DEMO_DEFAULT_PUBLISH_INTERVAL;

	return 1;
}
/*---------------------------------------------------------------------------*/

static void publish(void)
{
	int len;
	int remaining = APP_BUFFER_SIZE;
	// Could we move this, so the radio is only turned on when we are about to send.
	// There is no need to supply 12 mA into the radio when measuring the temperature.
	NETSTACK_RADIO.on();
	seq_nr_value++;

	buf_ptr = app_buffer;

	len = snprintf(buf_ptr, remaining, "{"
			"\"Board\":\"%s\","
			"\"Node\":\"%s\","
			"\"Name\":\"%s\","
			"\"Seq #\":%d,"
			"\"Uptime [s]\":%lu",
	BOARD_STRING, SENSOR_STRING, conf->Username, seq_nr_value, clock_seconds());

	if (len < 0 || len >= remaining)
	{
		printf("Buffer too short. Have %d, need %d + \\0\n", remaining, len);
		return;
	}

	remaining -= len;
	buf_ptr += len;

	for (reading = cc26xx_web_demo_sensor_first(); reading != NULL; reading =
			reading->next)
	{
		if (reading->publish && reading->raw != SENSOR_ERROR)
		{
			len = snprintf(buf_ptr, remaining, ",\"%s\":%s", reading->descr,
					reading->converted);

			if (len < 0 || len >= remaining)
			{
				printf("Buffer too short. Have %d, need %d + \\0\n", remaining,
						len);
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
		etimer_set(&publish_periodic_timer,
		CC26XX_WEB_DEMO_NET_CONNECT_PERIODIC);
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
							strlen(hass_config->arg), MQTT_QOS_LEVEL_0, MQTT_RETAIN_OFF);
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

		printf("MQTT Client Process\n");

		MQTT_publish_sensor_data_done_event = process_alloc_event();

		/*conf = &web_demo_config.mqtt_config;
		if (init_config() != 1)
		{
			PROCESS_EXIT();
		}*/

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
				if(no_sleep_allowed || !sleep_counter)
				{
					if(sleep_counter){
						sleep_counter = 0;
						etimer_set(&sleep_retry_timer, conf->pub_interval/CLOCK_SECOND);
					}else{
						sleep_counter = 1;
						etimer_set(&timeout_timer, CLOCK_SECOND);
						process_post(PROCESS_BROADCAST, Trig_sensors, NULL);
					}

				}
				if(NETSTACK_RADIO.sleep() !=-1){

					rtimer_arch_sleep(conf->pub_interval/CLOCK_SECOND * RTIMER_ARCH_SECOND); // 54uA in wait mode
					// if sleeptime is 60 sec, and we are awake 100ms we can live on a battery with 2700mAh for
					// 60/60.1*54uA+0.1/60.1*22mA = 90.5uA avg  2700mA/90.5uA = 29829hr ~ 3.4 years

					etimer_set(&timeout_timer, CLOCK_SECOND);
					process_post(PROCESS_BROADCAST, Trig_sensors, NULL);
				}
				else
				{
					etimer_set(&sleep_retry_timer, CLOCK_SECOND>>6);
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

			if (ev == cc26xx_web_demo_load_config_defaults)
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
