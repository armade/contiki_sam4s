

/*---------------------------------------------------------------------------*/
#ifndef BOARD_PERIPHERALS_H_
#define BOARD_PERIPHERALS_H_
/*---------------------------------------------------------------------------*/
#include "ADC_temp.h"
#include "Soft_rgb.h"
#include "Hard_rgb.h"
#include "lm73.h"
#include "button-sensor.h"
#include "dht11.h"
#include "step_motor.h"
#include "4ch_relay.h"
#include "bmp-280-sensor.h"
#include "htu21d-sensor.h"
#include "GPS-sensor.h"
#include "PIR-sensor.h"

//#define NODE_4_ch_relay
//#define NODE_LIGHT
//#define NODE_HARD_LIGHT
//#define NODE_STEP_MOTOR
//#define NODE_BMP280
//#define NODE_HTU21D
//#define NODE_GPS
//#define NODE_DHT11
//#define NODE_PIR_SR501

#ifdef NODE_PIR_SR501
#define SENSOR_STRING 		"PIR_SR501"
#endif

#ifdef NODE_DHT11
#define SENSOR_STRING 		"DHT11"
#endif

#ifdef NODE_LIGHT
#define SENSOR_STRING 		"Light"
#endif

#ifdef NODE_HARD_LIGHT
#define SENSOR_STRING 		"Light (hard)"
#endif

#ifdef NODE_STEP_MOTOR
#define SENSOR_STRING 		"Step motor"
#endif

#ifdef NODE_4_ch_relay
#define SENSOR_STRING 		"4ch relay"
#endif

#ifdef NODE_BMP280
#define SENSOR_STRING 		"BMP280"
#endif

#ifdef NODE_HTU21D
#define SENSOR_STRING 		"HTU21D"
#endif

#ifdef NODE_GPS
#define SENSOR_STRING 		"GPS_mc1010"
#endif
/*---------------------------------------------------------------------------*/
#endif /* BOARD_PERIPHERALS_H_ */
/*---------------------------------------------------------------------------*/

#define SENSOR_STATUS_DISABLED     0
#define SENSOR_STATUS_INITIALISED  1
#define SENSOR_STATUS_NOT_READY    2
#define SENSOR_STATUS_READY        3
