/*
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
 *
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
 *
 */

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
#include "1ch_relay.h"
#include "bmp-280-sensor.h"
#include "htu21d-sensor.h"
#include "GPS-sensor.h"
#include "PIR_SR501_sensor.h"
#include "christmas_light.h"

//#define NODE_4_ch_relay
//#define NODE_1_ch_relay
//#define NODE_LIGHT
//#define NODE_HARD_LIGHT
//#define NODE_STEP_MOTOR
//#define NODE_BMP280
//#define NODE_HTU21D
//#define NODE_GPS
//#define NODE_DHT11
//#define NODE_PIR_SR501
//#define NODE_christmas_light

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

#ifdef NODE_1_ch_relay
#define SENSOR_STRING 		"1ch relay"
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

#ifdef NODE_christmas_light
#define SENSOR_STRING 		"christmas light"
#endif
/*---------------------------------------------------------------------------*/
#endif /* BOARD_PERIPHERALS_H_ */
/*---------------------------------------------------------------------------*/

#define SENSOR_STATUS_DISABLED     0
#define SENSOR_STATUS_INITIALISED  1
#define SENSOR_STATUS_NOT_READY    2
#define SENSOR_STATUS_READY        3
