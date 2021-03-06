/*
 * Copyright � 2019, Peter Mikkelsen
 *
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
#include "contiki-conf.h"
#include "platform-conf.h"
#include "compiler.h"
#include "lib/sensors.h"
#include "Soft_rgb.h"
#include <stdint.h>
#include "csprng.h"
#include "board-peripherals.h"
#include "drivers/pmc.h"

/*---------------------------------------------------------------------------*/
#define DEBUG 0
#if DEBUG
#define PRINTF(...) printf(__VA_ARGS__)
#else
#define PRINTF(...)
#endif

#define RGB_R_GPIO            (PIO_PC11)
#define RGB_G_GPIO            (PIO_PC10)
#define RGB_B_GPIO            (PIO_PC9)
Pio *RGB2_base = (Pio *)PIOC;

struct ctimer RGB2_effect_timer;
static unsigned effect_state;
static RGB2_soft_t RGB_tmp;
static void RGB_COLORLOOP_RUN(void *data);
static void RGB_RANDOM_RUN(void *data);
static void RGB_RAPID_RED_RUN(void *data);

static volatile RGB2_soft_t RGB; // True output
static volatile RGB2_soft_t RGB_reload; // True reload output
static RGB2_soft_t user_set; // User set value (not modified by brightness)

static uint16_t counter=256;


static int Sensor_status = SENSOR_STATUS_DISABLED;

/*---------------------------------------------------------------------------*/
/*
   |\   |\
   | \  | \
   |  \ |  \
   |   \|   \
     ___  ___
  ___|  |_|  |
*/
void TC3_Handler(void) // waist of resources but we have plenty timers in same70 and this is easier
{
	RGB2_TIMER.TC_SR;

	if(counter == 0)
	{
		counter = 256;
		RGB.all = RGB_reload.all;
		RGB2_base->PIO_SODR = RGB_R_GPIO | RGB_G_GPIO | RGB_B_GPIO;
	}
	else
	{
		if(counter == RGB.led.r)	RGB2_base->PIO_CODR = RGB_R_GPIO;
		if(counter == RGB.led.g)	RGB2_base->PIO_CODR = RGB_G_GPIO;
		if(counter == RGB.led.b)	RGB2_base->PIO_CODR = RGB_B_GPIO;
		counter--;
	}

}

#define max_in  256 // Top end of INPUT range
#define max_out 256 // Top end of OUTPUT range

// exp return (int)(pow(input / max_in, gamma_var) * max_out + 0.5);
// To simplify the expression the gamma value is chosen to 3
static uint16_t gamma_corr(int input)
{
	uint64_t accu;
	uint16_t ret;

	accu = (uint64_t)input*input*input*max_out; // 33 bit max
	//accu /= (max_in*max_in*max_in);
	accu >>= 23;  // 1/(255^3)
	accu++;// The last bit is 0.5 if we add 0.5 to 0.5 we get 1 :)

	ret = (accu>>1);
	return ret;
}
/*---------------------------------------------------------------------------*/
/**
 * \brief Set value (0-256)
 */
static int
value_soft_RGB2(uint16_t R,uint16_t G,uint16_t B, uint16_t brightness)
{
	static RGB2_soft_t tmp;

	if((brightness>256))		brightness = 256;
	if((R > 256))				R=256;
	if((G > 256))				G=256;
	if((B > 256))				B=256;

	tmp.led.r = (gamma_corr(R)*brightness)>>8; // divide by 256
	tmp.led.g = (gamma_corr(G)*brightness)>>8;
	tmp.led.b = (gamma_corr(B)*brightness)>>8;
	tmp.led.brightness = brightness;

	RGB_reload.all = tmp.all;

	return 1;
}

int
soft_RGB2_value(int value)
{
	RGB2_soft_t *temp = (RGB2_soft_t *)value;
	if(value != SENSOR_ERROR){
		effect_state = 255;
		user_set.all = temp->all;
		value_soft_RGB2(temp->led.r,temp->led.g,temp->led.b,temp->led.brightness);
		sensors_changed(&soft_RGB_ctrl_sensor);
	}
	return (int)&user_set.all;
}
/*---------------------------------------------------------------------------*/
int
soft_RGB2_init(void)
{
	pmc_enable_periph_clk(RGB2_TIMER_ID);
	RGB2_TIMER.TC_CMR= 3 + (2<<13); //1=MCK/128   2<<13=up rc compare
	RGB2_TIMER.TC_RC=30; // 150MHz/(128*30*256) = 152 HZ

	//RGB2_TIMER.TC_CCR=1;
	//RGB2_TIMER.TC_CCR=4;
	RGB2_TIMER.TC_IER=1<<4; //CPCS
	NVIC_ClearPendingIRQ(RGB2_TIMER_IRQ);
	NVIC_SetPriority((IRQn_Type) RGB2_TIMER_ID, 5); //level 0 is the highest interrupt priority (0-15)
	NVIC_EnableIRQ(RGB2_TIMER_IRQ);


	// Enable PIO to controle the pin
	RGB2_base->PIO_PER  = RGB_R_GPIO | RGB_G_GPIO | RGB_B_GPIO;
	// high and Output
	RGB2_base->PIO_SODR = RGB_R_GPIO | RGB_G_GPIO | RGB_B_GPIO;
	RGB2_base->PIO_OER  = RGB_R_GPIO | RGB_G_GPIO | RGB_B_GPIO;
	return 1;
}
/*---------------------------------------------------------------------------*/
/**
 * \brief Configuration function for the sensor.
 *
 * \param type Activate, enable or disable the sensor. See below
 * \param enable
 *
 * When type == SENSORS_HW_INIT we turn on the hardware
 * When type == SENSORS_ACTIVE and enable==1 we enable the sensor
 * When type == SENSORS_ACTIVE and enable==0 we disable the sensor
 */
static int
soft_RGB2_configure(int type, int enable)
{
	switch(type) {

		case SENSORS_HW_INIT:
			user_set.led.brightness = 255;
			user_set.led.r = 255;
			user_set.led.g = 255;
			user_set.led.b = 255;

			RGB_reload.all = user_set.all;
			soft_RGB2_init();
			Sensor_status = SENSOR_STATUS_INITIALISED;

			break;

		case SENSORS_ACTIVE:
			if(Sensor_status == SENSOR_STATUS_DISABLED)
				return Sensor_status;

			 if(enable == 1) {
				 sensors_changed(&soft_RGB_ctrl_sensor);
			 }else if(enable == 7){
				 //effect_state = 255;
				 counter = 0;
				 value_soft_RGB2(255,255,255,255);
				 RGB2_TIMER.TC_CCR=1;
				 RGB2_TIMER.TC_CCR=4;
				 Sensor_status = SENSOR_STATUS_READY;
			 } else if(enable == 8){
				 effect_state = 255;
				 RGB2_TIMER.TC_CCR=2;
				 RGB2_base->PIO_OER =  RGB_R_GPIO | RGB_G_GPIO | RGB_B_GPIO;
				 RGB2_base->PIO_SODR = RGB_R_GPIO | RGB_G_GPIO | RGB_B_GPIO;
				 Sensor_status = SENSOR_STATUS_INITIALISED;
			 } else if(enable == 10){
				 if((Sensor_status&0xfff) == SENSOR_STATUS_READY){
					 effect_state = 0;
					 RGB_tmp.led = (leds2_t){0,0,256,256};
					 RGB_COLORLOOP_RUN(NULL);
					 Sensor_status &= ~(0xf<<12);
					 Sensor_status |= (1<<12);
					 sensors_changed(&soft_RGB_ctrl_sensor);
				 }
			 } else if(enable == 11){
				 if((Sensor_status&0xfff) == SENSOR_STATUS_READY){
					 effect_state = 0;
					 RGB_tmp.led = (leds2_t){256,256,256,256};
					 RGB_RANDOM_RUN(NULL);
					 Sensor_status &= ~(0xf<<12);
					 Sensor_status |= (2<<12);
					 sensors_changed(&soft_RGB_ctrl_sensor);
				 }
			 }else if(enable == 12){
				 if((Sensor_status&0xfff) == SENSOR_STATUS_READY){
					 effect_state = 0;
					 RGB_tmp.led = (leds2_t){256,0,0,256};
					 RGB_RAPID_RED_RUN(NULL);
					 Sensor_status &= ~(0xf<<12);
					 Sensor_status |= (3<<12);
					 sensors_changed(&soft_RGB_ctrl_sensor);
				 }
			 }
			 break;
	}
	return Sensor_status;
}
/*---------------------------------------------------------------------------*/
static int
soft_RGB2_status(int type)
{
	return Sensor_status;
}
/*---------------------------------------------------------------------------*/
SENSORS_SENSOR(soft_RGB2_ctrl_sensor, "RGB2", soft_RGB2_value, soft_RGB2_configure, soft_RGB2_status);
/*---------------------------------------------------------------------------*/




static void
RGB_COLORLOOP_RUN(void *data)
{
	clock_time_t next = 16;

	switch(effect_state){

		case 0:
			RGB_tmp.led.b--;
			RGB_tmp.led.r++;
			if(RGB_tmp.led.r == 256)	effect_state++;
			break;

		case 1:
			RGB_tmp.led.r--;
			RGB_tmp.led.g++;
			if(RGB_tmp.led.g == 256)	effect_state++;
			break;

		case 2:
			RGB_tmp.led.g--;
			RGB_tmp.led.b++;
			if(RGB_tmp.led.b == 256)	effect_state = 0;
			break;

		case 255:// exit
			return;
	}

	value_soft_RGB2(RGB_tmp.led.r,RGB_tmp.led.g,RGB_tmp.led.b,RGB_tmp.led.brightness);
	ctimer_set(&RGB2_effect_timer, next, RGB_COLORLOOP_RUN, NULL);
}

static void
RGB_RANDOM_RUN(void *data)
{
	clock_time_t next;
	uint16_t rnd[2];

	if(effect_state == 255)// exit
		return;

	csprng_get((unsigned char *)&rnd[0],4);
	RGB_tmp.led.brightness = (rnd[0] & 127)+128;

	next = rnd[1]&0xff;

	if(next < 5)
		next = 5;

	// NB: user can't see the value update on the PWM signal. It would just confuse them.
	value_soft_RGB2(256,170,0,RGB_tmp.led.brightness);
	ctimer_set(&RGB2_effect_timer, next, RGB_RANDOM_RUN, NULL);
}

static void
RGB_RAPID_RED_RUN(void *data)
{
	clock_time_t next = 50;

	if(effect_state == 255)// exit
		return;

	RGB_tmp.led.brightness ^= 256;

	// NB: user can't see the value update on the PWM signal. It would just confuse them.
	value_soft_RGB2(256,0,0,RGB_tmp.led.brightness);
	ctimer_set(&RGB2_effect_timer, next, RGB_RAPID_RED_RUN, NULL);
}

/*---------------------------------------------------------------------------*/

