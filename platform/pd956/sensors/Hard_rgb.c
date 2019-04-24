/*
 * Copyright © 2019, Peter Mikkelsen
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
#include "Hard_rgb.h"
#include "gpio.h"
#include <stdint.h>
#include "csprng.h"


#include "board-peripherals.h"
#ifdef NODE_HARD_LIGHT
/*---------------------------------------------------------------------------*/
#define DEBUG 0
#if DEBUG
#define PRINTF(...) printf(__VA_ARGS__)
#else
#define PRINTF(...)
#endif

struct ctimer RGB_effect_timer;
unsigned effect_state;
static RGB_hard_t RGB_tmp; // Used doing effect
static void RGB_COLORLOOP_RUN(void *data);
static void RGB_RANDOM_RUN(void *data);
static void RGB_RAPID_RED_RUN(void *data);

static RGB_hard_t hard_user_set; // User set value (not modified by brightness or gamma)
static int Sensor_status = SENSOR_STATUS_DISABLED;



// NB: max_in is hardcoded into function for optimizing flow.
//     Just change the number of shifts.
#define max_in  4096 // Top end of INPUT range
#define max_out 4096 // Top end of OUTPUT range

// exp: (int)(pow(input / max_in, gamma_var) * max_out + 0.5);
// To simplify the expression the gamma value is chosen to 3 (normally 2.8)
int gamma_corr(int input)
{
	volatile uint64_t accu = (uint64_t)input*input*input*max_out; // 49 bit max
	accu >>= 35;  // 1/(4096^3)   4096 = 2^12   12*3=36  shift 35 down to preserve the last digit.
	accu++; // The last bit is 0.5 if we add 0.5 to 0.5 we get 1 :)

	return (int) (accu>>1);
}

/**
 * \brief Set value (0-4096) (0-4096) (0-4096) (0-256)
 * NB: only to be used here
 */
static int
value_hard_RGB(uint16_t R,uint16_t G,uint16_t B, uint16_t brightness)
{
	if((brightness>256))		brightness = 256;
	if((R > 4096))				R=4096;
	if((G > 4096))				G=4096;
	if((B > 4096))				B=4096;

	PWM->PWM_CH_NUM[1].PWM_CDTYUPD = (uint32_t)(gamma_corr(R)*brightness)>>8; // divide by 256
	PWM->PWM_CH_NUM[2].PWM_CDTYUPD = (uint32_t)(gamma_corr(G)*brightness)>>8; // divide by 256
	PWM->PWM_CH_NUM[3].PWM_CDTYUPD = (uint32_t)(gamma_corr(B)*brightness)>>8; // divide by 256

	return 1;
}

//NB: value must be an address to the variable that holds the RGB values
static int
hard_RGB_value(int value_addr)
{
	RGB_hard_t *temp = (RGB_hard_t *)value_addr;
	if(value_addr != SENSOR_ERROR){
		effect_state = 255;
		hard_user_set.all = temp->all;
		value_hard_RGB(hard_user_set.led.r,hard_user_set.led.g,hard_user_set.led.b,hard_user_set.led.brightness);
		sensors_changed(&hard_RGB_ctrl_sensor);
	}
	return (int)&hard_user_set.all;
}
/*---------------------------------------------------------------------------*/
int
hard_RGB_init(void)
{
	uint32_t clk_config;
	pmc_enable_periph_clk(ID_PWM);
	PWM->PWM_DIS = (1<<0) | (1<<1) | (1<<2) | (1<<3);
	PWM->PWM_OOV=0; // overwrite to low
	PWM->PWM_OS = (1<<3) | (1<<2) | (1<<17); // Set all PWM (high and low) to low level

	REG_CCFG_SYSIO = PIO_PB4;

	pio_set_peripheral(PIOA, PIO_PERIPH_B, PIO_PA20); 	// pwm1L -> R
	pio_set_peripheral(PIOA, PIO_PERIPH_B, PIO_PA7); 	// pwm3H -> B
	pio_set_peripheral(PIOB, PIO_PERIPH_B, PIO_PB4);	// pwm2H -> G

	// PWM clock = 3.75MHz
#if !LOW_CLOCK //120Mhz
	clk_config = (5<<0); //CLKA no divide mck/32
#else //30Mhz
	clk_config = (3<<0); //CLKA no divide mck/8
#endif

	// 3.75MHz/4096 = 915.5Hz => more that enough
	PWM->PWM_CH_NUM[1].PWM_CPRD = 4096;
	PWM->PWM_CH_NUM[1].PWM_CDTY = 0;
	PWM->PWM_CH_NUM[1].PWM_CMR = clk_config;

	PWM->PWM_CH_NUM[2].PWM_CPRD = 4096;
	PWM->PWM_CH_NUM[2].PWM_CDTY = 0;
	PWM->PWM_CH_NUM[2].PWM_CMR = (1<<17) | clk_config;

	PWM->PWM_CH_NUM[3].PWM_CPRD = 4096;
	PWM->PWM_CH_NUM[3].PWM_CDTY = 0;
	PWM->PWM_CH_NUM[3].PWM_CMR = (1<<17) | clk_config;

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
hard_RGB_configure(int type, int enable)
{
	switch(type)
	{
		case SENSORS_HW_INIT:
			hard_RGB_init();
			Sensor_status = SENSOR_STATUS_INITIALISED;
			hard_user_set.led.brightness = 256;
			hard_user_set.led.r = 4096;
			hard_user_set.led.g = 4096;
			hard_user_set.led.b = 4096;
			break;

		case SENSORS_ACTIVE:
			if(Sensor_status == SENSOR_STATUS_DISABLED)
			return Sensor_status;

			if(enable==1){
				//Nothing to enable so we signal done
				sensors_changed(&hard_RGB_ctrl_sensor);
				//RGB_TEST(NULL);
			} else if(enable == 7){ // ON command
				//effect_state = 255;
				PWM->PWM_ENA = (1<<1) | (1<<2) | (1<<3);
				PWM->PWM_OSC = (1<<3) | (1<<2) | (1<<17);// Remove overwrite (0 to pwm)
				value_hard_RGB(hard_user_set.led.r,hard_user_set.led.g,hard_user_set.led.b,hard_user_set.led.brightness);
				Sensor_status = SENSOR_STATUS_READY;
				sensors_changed(&hard_RGB_ctrl_sensor);
			} else if(enable == 8){ // OFF command
				effect_state = 255;
				PWM->PWM_DIS = (1<<1) | (1<<2) | (1<<3);
				PWM->PWM_OSS = (1<<3) | (1<<2) | (1<<17);// Apply overwrite (pwm to 0)
				Sensor_status = SENSOR_STATUS_INITIALISED;
				sensors_changed(&hard_RGB_ctrl_sensor);
			} else if(enable == 10){
				if(Sensor_status == SENSOR_STATUS_READY){
					effect_state = 0;
					RGB_tmp.led = (leds_hard_t){0,0,4096,256};
					RGB_COLORLOOP_RUN(NULL);
					Sensor_status &= ~(0xf<<12);
					Sensor_status |= (1<<12);
					sensors_changed(&hard_RGB_ctrl_sensor);
				}
			} else if(enable == 11){
				if(Sensor_status == SENSOR_STATUS_READY){
					effect_state = 0;
					RGB_tmp.led = (leds_hard_t){4096,4096,4096,256};
					RGB_RANDOM_RUN(NULL);
					Sensor_status &= ~(0xf<<12);
					Sensor_status |= (2<<12);
					sensors_changed(&hard_RGB_ctrl_sensor);
				}
			}else if(enable == 12){
				if((Sensor_status&0xfff) == SENSOR_STATUS_READY){
					 effect_state = 0;
					 RGB_tmp.led = (leds_hard_t){4096,4096,4096,256};
					 RGB_RAPID_RED_RUN(NULL);
					 Sensor_status &= ~(0xf<<12);
					 Sensor_status |= (3<<12);
					 sensors_changed(&hard_RGB_ctrl_sensor);
				}
			}
			break;
	}
	return Sensor_status;
}
/*---------------------------------------------------------------------------*/
static int
hard_RGB_status(int type)
{
	return Sensor_status;
}
/*---------------------------------------------------------------------------*/
SENSORS_SENSOR(hard_RGB_ctrl_sensor, "RGB", hard_RGB_value, hard_RGB_configure, hard_RGB_status);
/*---------------------------------------------------------------------------*/



// 4 ms run on 4096 = 16 sec pr color
// 4 ms run on 4096/4 = 4 sec pr color
static void
RGB_COLORLOOP_RUN(void *data)
{
	clock_time_t next = (4 * 1000) / CLOCK_SECOND;

	switch(effect_state)
	{
		case 0:
			RGB_tmp.led.b-=4;
			RGB_tmp.led.r += 4;
			if(RGB_tmp.led.r == 4096) effect_state++;
			break;

		case 1:
			RGB_tmp.led.r-=4;
			RGB_tmp.led.g += 4;
			if(RGB_tmp.led.g == 4096) effect_state++;
			break;

		case 2:
			RGB_tmp.led.g-=4;
			RGB_tmp.led.b += 4;
			if(RGB_tmp.led.b == 4096) effect_state = 0;
			break;

		case 255: // exit
			return;
	}
	// NB: user can't see the value update on the PWM signal. It would just confuse them.
	value_hard_RGB(RGB_tmp.led.r,RGB_tmp.led.g,RGB_tmp.led.b,RGB_tmp.led.brightness);
	ctimer_set(&RGB_effect_timer, next, RGB_COLORLOOP_RUN, NULL);
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

	next = (next * 1000) / CLOCK_SECOND;

	// NB: user can't see the value update on the PWM signal. It would just confuse them.
	value_hard_RGB(4096,2720,0,RGB_tmp.led.brightness);
	ctimer_set(&RGB_effect_timer, next, RGB_RANDOM_RUN, NULL);
}

static void
RGB_RAPID_RED_RUN(void *data)
{
	clock_time_t next = (50 * 1000) / CLOCK_SECOND;

	if(effect_state == 255)// exit
		return;

	RGB_tmp.led.brightness ^= 256;

	// NB: user can't see the value update on the PWM signal. It would just confuse them.
	value_hard_RGB(256,0,0,RGB_tmp.led.brightness);
	ctimer_set(&RGB_effect_timer, next, RGB_RAPID_RED_RUN, NULL);
}

/*---------------------------------------------------------------------------*/
#endif
