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
#include "httpd-simple.h"
#include "clock.h"

#include <string.h>
#include <strings.h>
#include <stdio.h>
#include <stdlib.h>

#define ADDRESS_CONVERSION_OK       1
#define ADDRESS_CONVERSION_ERROR    0

extern void
new_net_config(void);

// wget --post-data "effectOption=8" http://10.42.0.7/ -O /dev/null


static int effectOption_post_handler(char *key, int key_len, char *val, int val_len)
{
	int ret = HTTPD_SIMPLE_POST_HANDLER_UNKNOWN;

	if(key_len != strlen("effectOption")
			|| strncasecmp(key, "effectOption", strlen("effectOption")) != 0){
		/* Not ours */
		return HTTPD_SIMPLE_POST_HANDLER_UNKNOWN;
	}
	ret = strtoul(val, NULL, 10);

	if(ret < 7 || ret > 12){
		return HTTPD_SIMPLE_POST_HANDLER_ERROR;
	}

	  soft_RGB_ctrl_sensor.configure(SENSORS_ACTIVE,ret);


	return HTTPD_SIMPLE_POST_HANDLER_OK;
}

static int effectOption2_post_handler(char *key, int key_len, char *val, int val_len)
{
	int ret = HTTPD_SIMPLE_POST_HANDLER_UNKNOWN;

	if(key_len != strlen("effectOption2")
			|| strncasecmp(key, "effectOption2", strlen("effectOption2")) != 0){
		/* Not ours */
		return HTTPD_SIMPLE_POST_HANDLER_UNKNOWN;
	}
	ret = strtoul(val, NULL, 10);

	if(ret < 7 || ret > 12){
		return HTTPD_SIMPLE_POST_HANDLER_ERROR;
	}

	  soft_RGB2_ctrl_sensor.configure(SENSORS_ACTIVE,ret);


	return HTTPD_SIMPLE_POST_HANDLER_OK;
}


static int RGB_blue_post_handler(char *key, int key_len, char *val, int val_len)
{
	int ret = HTTPD_SIMPLE_POST_HANDLER_UNKNOWN;
	RGB_soft_t tmp;
	if(key_len != strlen("RGB_blue")
			|| strncasecmp(key, "RGB_blue", strlen("RGB_blue")) != 0){
		/* Not ours */
		return HTTPD_SIMPLE_POST_HANDLER_UNKNOWN;
	}

	ret = strtoul(val, NULL, 10);
	printf("B:%d\n", ret);
	if(ret < 0 || ret > 256){
		return HTTPD_SIMPLE_POST_HANDLER_ERROR;
	}

	tmp.all = ((RGB_soft_t *) soft_RGB_ctrl_sensor.value(SENSOR_ERROR))->all;
	tmp.led.b = ret;
	soft_RGB_ctrl_sensor.value((int) &tmp.all);

	return HTTPD_SIMPLE_POST_HANDLER_OK;
}

/*---------------------------------------------------------------------------*/

static int RGB_green_post_handler(char *key, int key_len, char *val,
		int val_len)
{
	int ret = HTTPD_SIMPLE_POST_HANDLER_UNKNOWN;
	RGB_soft_t tmp;
	if(key_len != strlen("RGB_green")
			|| strncasecmp(key, "RGB_green", strlen("RGB_green")) != 0){
		/* Not ours */
		return HTTPD_SIMPLE_POST_HANDLER_UNKNOWN;
	}

	ret = strtoul(val, NULL, 10);
	printf("G:%d\n", ret);
	if(ret < 0 || ret > 256){
		return HTTPD_SIMPLE_POST_HANDLER_ERROR;
	}

	tmp.all = ((RGB_soft_t *) soft_RGB_ctrl_sensor.value(SENSOR_ERROR))->all;
	tmp.led.g = ret;
	soft_RGB_ctrl_sensor.value((int) &tmp.all);

	return HTTPD_SIMPLE_POST_HANDLER_OK;
}

/*---------------------------------------------------------------------------*/
static int RGB_red_post_handler(char *key, int key_len, char *val, int val_len)
{
	int ret = HTTPD_SIMPLE_POST_HANDLER_UNKNOWN;
	RGB_soft_t tmp;
	if(key_len != strlen("RGB_red")
			|| strncasecmp(key, "RGB_red", strlen("RGB_red")) != 0){
		/* Not ours */
		return HTTPD_SIMPLE_POST_HANDLER_UNKNOWN;
	}

	ret = strtoul(val, NULL, 10);
	printf("R:%d\n", ret);
	if(ret < 0 || ret > 256){
		return HTTPD_SIMPLE_POST_HANDLER_ERROR;
	}

	tmp.all = ((RGB_soft_t *) soft_RGB_ctrl_sensor.value(SENSOR_ERROR))->all;
	tmp.led.r = ret;
	soft_RGB_ctrl_sensor.value((int) &tmp.all);

	return HTTPD_SIMPLE_POST_HANDLER_OK;
}

/*---------------------------------------------------------------------------*/
static int RGB_brightness_post_handler(char *key, int key_len, char *val,
		int val_len)
{
	int ret = HTTPD_SIMPLE_POST_HANDLER_UNKNOWN;
	RGB_soft_t tmp;
	if(key_len != strlen("RGB_brightness")
			|| strncasecmp(key, "RGB_brightness", strlen("RGB_brightness"))
					!= 0){
		/* Not ours */
		return HTTPD_SIMPLE_POST_HANDLER_UNKNOWN;
	}

	ret = strtoul(val, NULL, 10);
	printf("I:%d\n", ret);
	if(ret < 0 || ret > 256){
		return HTTPD_SIMPLE_POST_HANDLER_ERROR;
	}

	tmp.all = ((RGB_soft_t *) soft_RGB_ctrl_sensor.value(SENSOR_ERROR))->all;
	tmp.led.brightness = ret;
	soft_RGB_ctrl_sensor.value((int) &tmp.all);

	return HTTPD_SIMPLE_POST_HANDLER_OK;
}

//==================================================================================
static int RGB2_blue_post_handler(char *key, int key_len, char *val, int val_len)
{
	int ret = HTTPD_SIMPLE_POST_HANDLER_UNKNOWN;
	RGB2_soft_t tmp;
	if(key_len != strlen("RGB2_blue")
			|| strncasecmp(key, "RGB2_blue", strlen("RGB2_blue")) != 0){
		/* Not ours */
		return HTTPD_SIMPLE_POST_HANDLER_UNKNOWN;
	}

	ret = strtoul(val, NULL, 10);
	printf("B:%d\n", ret);
	if(ret < 0 || ret > 256){
		return HTTPD_SIMPLE_POST_HANDLER_ERROR;
	}

	tmp.all = ((RGB2_soft_t *) soft_RGB2_ctrl_sensor.value(SENSOR_ERROR))->all;
	tmp.led.b = ret;
	soft_RGB2_ctrl_sensor.value((int) &tmp.all);

	return HTTPD_SIMPLE_POST_HANDLER_OK;
}

/*---------------------------------------------------------------------------*/

static int RGB2_green_post_handler(char *key, int key_len, char *val,
		int val_len)
{
	int ret = HTTPD_SIMPLE_POST_HANDLER_UNKNOWN;
	RGB2_soft_t tmp;
	if(key_len != strlen("RGB2_green")
			|| strncasecmp(key, "RGB2_green", strlen("RGB2_green")) != 0){
		/* Not ours */
		return HTTPD_SIMPLE_POST_HANDLER_UNKNOWN;
	}

	ret = strtoul(val, NULL, 10);
	printf("G:%d\n", ret);
	if(ret < 0 || ret > 256){
		return HTTPD_SIMPLE_POST_HANDLER_ERROR;
	}

	tmp.all = ((RGB2_soft_t *) soft_RGB2_ctrl_sensor.value(SENSOR_ERROR))->all;
	tmp.led.g = ret;
	soft_RGB2_ctrl_sensor.value((int) &tmp.all);

	return HTTPD_SIMPLE_POST_HANDLER_OK;
}

/*---------------------------------------------------------------------------*/
static int RGB2_red_post_handler(char *key, int key_len, char *val, int val_len)
{
	int ret = HTTPD_SIMPLE_POST_HANDLER_UNKNOWN;
	RGB2_soft_t tmp;
	if(key_len != strlen("RGB2_red")
			|| strncasecmp(key, "RGB2_red", strlen("RGB2_red")) != 0){
		/* Not ours */
		return HTTPD_SIMPLE_POST_HANDLER_UNKNOWN;
	}

	ret = strtoul(val, NULL, 10);
	printf("R:%d\n", ret);
	if(ret < 0 || ret > 256){
		return HTTPD_SIMPLE_POST_HANDLER_ERROR;
	}

	tmp.all = ((RGB2_soft_t *) soft_RGB2_ctrl_sensor.value(SENSOR_ERROR))->all;
	tmp.led.r = ret;
	soft_RGB2_ctrl_sensor.value((int) &tmp.all);

	return HTTPD_SIMPLE_POST_HANDLER_OK;
}

/*---------------------------------------------------------------------------*/
static int RGB2_brightness_post_handler(char *key, int key_len, char *val,
		int val_len)
{
	int ret = HTTPD_SIMPLE_POST_HANDLER_UNKNOWN;
	RGB2_soft_t tmp;
	if(key_len != strlen("RGB2_brightness")
			|| strncasecmp(key, "RGB2_brightness", strlen("RGB2_brightness"))
					!= 0){
		/* Not ours */
		return HTTPD_SIMPLE_POST_HANDLER_UNKNOWN;
	}

	ret = strtoul(val, NULL, 10);
	printf("I:%d\n", ret);
	if(ret < 0 || ret > 256){
		return HTTPD_SIMPLE_POST_HANDLER_ERROR;
	}

	tmp.all = ((RGB2_soft_t *) soft_RGB2_ctrl_sensor.value(SENSOR_ERROR))->all;
	tmp.led.brightness = ret;
	soft_RGB2_ctrl_sensor.value((int) &tmp.all);

	return HTTPD_SIMPLE_POST_HANDLER_OK;
}
//=================================================================

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



/*---------------------------------------------------------------------------*/
HTTPD_SIMPLE_POST_HANDLER(effectOption,effectOption_post_handler);
HTTPD_SIMPLE_POST_HANDLER(effectOption2,effectOption2_post_handler);

HTTPD_SIMPLE_POST_HANDLER(RGB_blue,RGB_blue_post_handler);
HTTPD_SIMPLE_POST_HANDLER(RGB_green,RGB_green_post_handler);
HTTPD_SIMPLE_POST_HANDLER(RGB_red,RGB_red_post_handler);
HTTPD_SIMPLE_POST_HANDLER(RGB_brightness,RGB_brightness_post_handler);

HTTPD_SIMPLE_POST_HANDLER(RGB2_blue,RGB2_blue_post_handler);
HTTPD_SIMPLE_POST_HANDLER(RGB2_green,RGB2_green_post_handler);
HTTPD_SIMPLE_POST_HANDLER(RGB2_red,RGB2_red_post_handler);
HTTPD_SIMPLE_POST_HANDLER(RGB2_brightness,RGB2_brightness_post_handler);

#ifdef NODE_STEP_MOTOR
HTTPD_SIMPLE_POST_HANDLER(Step_motor_position,step_motor_position_post_handler);
#endif
#ifdef NODE_4_ch_relay
HTTPD_SIMPLE_POST_HANDLER(Relay,Relay_post_handler);
#endif

/*---------------------------------------------------------------------------*/

/*---------------------------------------------------------------------------*/
HTTPD_SIMPLE_POST_HANDLER(timestamp,timestamp_post_handler);
HTTPD_SIMPLE_POST_HANDLER(timezone,timezone_post_handler);


void
register_http_post_handlers(void)
{
	httpd_simple_register_post_handler(&RGB_blue_handler);
	httpd_simple_register_post_handler(&RGB_green_handler);
	httpd_simple_register_post_handler(&RGB_red_handler);
	httpd_simple_register_post_handler(&RGB_brightness_handler);
	httpd_simple_register_post_handler(&effectOption_handler);

	httpd_simple_register_post_handler(&RGB2_blue_handler);
	httpd_simple_register_post_handler(&RGB2_green_handler);
	httpd_simple_register_post_handler(&RGB2_red_handler);
	httpd_simple_register_post_handler(&RGB2_brightness_handler);
	httpd_simple_register_post_handler(&effectOption2_handler);
#ifdef NODE_STEP_MOTOR
	httpd_simple_register_post_handler(&Step_motor_position_handler);
#endif
#ifdef NODE_4_ch_relay
	httpd_simple_register_post_handler(&Relay_handler);
#endif



	httpd_simple_register_post_handler(&timestamp_handler);
	httpd_simple_register_post_handler(&timezone_handler);
}
