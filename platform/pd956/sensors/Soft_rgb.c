
#include "contiki-conf.h"
#include "platform-conf.h"
#include "compiler.h"
#include "lib/sensors.h"
#include "Soft_rgb.h"
#include <stdint.h>
#include "csprng.h"
#include "board-peripherals.h"
#ifdef NODE_LIGHT
/*---------------------------------------------------------------------------*/
#define DEBUG 0
#if DEBUG
#define PRINTF(...) printf(__VA_ARGS__)
#else
#define PRINTF(...)
#endif

#define RGB_R_GPIO            (PIO_PA8)
#define RGB_G_GPIO            (PIO_PA9)
#define RGB_B_GPIO            (PIO_PA10)
Pio *RGB_base = (Pio *)PIOA;

struct ctimer RGB_effect_timer;
unsigned effect_state = 43;
static void RGB_COLORLOOP_RUN(void *data);

volatile RGB_t RGB; // True output
volatile RGB_t RGB_reload; // True reload output
RGB_t user_set; // User set value (not modified by brightness)

uint16_t counter=256;


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
void TC2_Handler(void)
{
	RGB_TIMER.TC_SR;

	if(counter == 0)
	{
		counter = 256;
		RGB.all = RGB_reload.all;
		RGB_base->PIO_CODR = RGB_R_GPIO | RGB_G_GPIO | RGB_B_GPIO;
	}
	else
	{
		if(counter == RGB.led.r)	RGB_base->PIO_SODR = RGB_R_GPIO;
		if(counter == RGB.led.g)	RGB_base->PIO_SODR = RGB_G_GPIO;
		if(counter == RGB.led.b)	RGB_base->PIO_SODR = RGB_B_GPIO;
	}
	counter--;
}

#define max_in  256 // Top end of INPUT range
#define max_out 256 // Top end of OUTPUT range

// exp return (int)(pow(input / max_in, gamma_var) * max_out + 0.5);
// To simplify the expression the gamma value is chosen to 3
int gamma_corr(int input)
{
	uint64_t accu = input*input*input*max_out; // 33 bit max
	//accu /= (max_in*max_in*max_in);
	accu >>= 23;  // 1/(255^3)
	accu++;// The last bit is 0.5 if we add 0.5 to 0.5 we get 1 :)

	return (int) (accu>>1);
}
/*---------------------------------------------------------------------------*/
/**
 * \brief Set value (0-255)
 */
static int
value_RGB(uint16_t R,uint16_t G,uint16_t B, uint16_t brightness)
{
	RGB_t tmp;

	tmp.led.r = (gamma_corr(R)*brightness)>>8; // divide by 256
	tmp.led.g = (gamma_corr(G)*brightness)>>8;
	tmp.led.b = (gamma_corr(B)*brightness)>>8;
	tmp.led.brightness = brightness;

	RGB_reload.all = tmp.all;

	return 1;
}

int
RGB_set(int value)
{
	RGB_t *temp = (RGB_t *)&value;
	if(value != SENSOR_ERROR){
		effect_state = 255;
		user_set.all = temp->all;
		value_RGB(temp->led.r,temp->led.g,temp->led.b,temp->led.brightness);
		sensors_changed(&soft_RGB_ctrl_sensor);
	}
	return user_set.all;
}
/*---------------------------------------------------------------------------*/
int
configure_RGB(void)
{
	pmc_enable_periph_clk(RGB_TIMER_ID);
#if !LOW_CLOCK //120Mhz
	RGB_TIMER.TC_CMR= 2 + (2<<13); //2=MCK/32   2<<13=up rc compare
	RGB_TIMER.TC_RC=30; //30 er 125kHz,  (125kHz/255 ~ 500Hz)
#else //30Mhz
	RGB_TIMER.TC_CMR= 1 + (2<<13); //1=MCK/8   2<<13=up rc compare
	RGB_TIMER.TC_RC=30; //30 er 125kHz,  (125kHz/255 ~ 500Hz)
#endif

	//RGB_TIMER.TC_CCR=1;
	//RGB_TIMER.TC_CCR=4;
	RGB_TIMER.TC_IER=1<<4; //CPCS
	NVIC_ClearPendingIRQ(RGB_TIMER_IRQ);
	NVIC_SetPriority((IRQn_Type) RGB_TIMER_ID, 5); //level 0 is the highest interrupt priority (0-15)
	NVIC_EnableIRQ(RGB_TIMER_IRQ);


	pio_set_input(PIOA,PIO_PA3 | PIO_PA28,0); // NB: pa3 and pa28 is connected to pa9 and pa10.

	// Enable PIO to controle the pin
	RGB_base->PIO_PER  = RGB_R_GPIO | RGB_G_GPIO | RGB_B_GPIO;
	// Output and low
	RGB_base->PIO_OER  = RGB_R_GPIO | RGB_G_GPIO | RGB_B_GPIO;
	RGB_base->PIO_CODR = RGB_R_GPIO | RGB_G_GPIO | RGB_B_GPIO;

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
RGB_init(int type, int enable)
{
	switch(type) {

		case SENSORS_HW_INIT:
			user_set.led.brightness = 255;
			user_set.led.r = 255;
			user_set.led.g = 255;
			user_set.led.b = 255;

			RGB_reload.all = user_set.all;
			configure_RGB();
			Sensor_status = SENSOR_STATUS_INITIALISED;

			break;

		case SENSORS_ACTIVE:
			if(Sensor_status == SENSOR_STATUS_DISABLED)
				return SENSOR_STATUS_DISABLED;

			 if(enable == 1) {
				 sensors_changed(&hard_RGB_ctrl_sensor);
			 }else if(enable == 7){
				 effect_state = 255;
				 counter = 0;
				 RGB_TIMER.TC_CCR=1;
				 RGB_TIMER.TC_CCR=4;
				 Sensor_status = SENSOR_STATUS_READY;
			 } else if(enable == 8){
				 effect_state = 255;
				 RGB_TIMER.TC_CCR=2;
				 RGB_base->PIO_OER =  RGB_R_GPIO | RGB_G_GPIO | RGB_B_GPIO;
				 RGB_base->PIO_CODR = RGB_R_GPIO | RGB_G_GPIO | RGB_B_GPIO;
				 Sensor_status = SENSOR_STATUS_INITIALISED;
			 } else if(enable == 10){
				 if(Sensor_status == SENSOR_STATUS_READY){
					 effect_state = 43; // First run indicator
					 RGB_COLORLOOP_RUN(NULL);
					 Sensor_status |= (1<<12);
					 sensors_changed(&hard_RGB_ctrl_sensor);
				 }
			 } else if(enable == 11){
				 if(Sensor_status == SENSOR_STATUS_READY){
					 effect_state = 43; // First run indicator
					 RGB_RANDOM_RUN(NULL);
					 Sensor_status |= (2<<12);
					 sensors_changed(&hard_RGB_ctrl_sensor);
				 }
			 }
			 break;
	}
	return Sensor_status;
}
/*---------------------------------------------------------------------------*/
static int
RGB_status(int type)
{
	return Sensor_status;
}
/*---------------------------------------------------------------------------*/
SENSORS_SENSOR(soft_RGB_ctrl_sensor, "RGB", RGB_set, RGB_init, RGB_status);
/*---------------------------------------------------------------------------*/


RGB_t RGB_tmp;

static void
RGB_COLORLOOP_RUN(void *data)
{
	clock_time_t next = 16;
	if(effect_state == 43){
		effect_state = 0;
		RGB_tmp.led.r = 0;
		RGB_tmp.led.g = 0;
		RGB_tmp.led.b = 0;
		RGB_tmp.led.brightness = 256;
	}
	witch(effect_state){

			case 0:
				if(RGB_tmp.led.b) 			RGB_tmp.led.b--;
				RGB_tmp.led.r++;
				if(RGB_tmp.led.r == 255)	effect_state++;
				break;

			case 1:
				if(RGB_tmp.led.r) 			RGB_tmp.led.r--;
				RGB_tmp.led.g++;
				if(RGB_tmp.led.g == 255)	effect_state++;
				break;

			case 2:
				if(RGB_tmp.led.g) 			RGB_tmp.led.g--;
				RGB_tmp.led.b++;
				if(RGB_tmp.led.b == 255)	effect_state = 0;
				break;

			case 255:// exit
				return;
		}

	value_RGB(RGB_tmp.led.r,RGB_tmp.led.g,RGB_tmp.led.b,RGB_tmp.led.brightness);

	ctimer_set(&RGB_effect_timer, next, RGB_COLORLOOP_RUN, NULL);
}
static void
RGB_RANDOM_RUN(void *data)
{
	clock_time_t next;
	uint16_t rnd[2];
	if(effect_state == 43){
		effect_state = 0;
		RGB_tmp.led.brightness = 256;
	}

	csprng_get((unsigned char *)&rnd[0],4);
	RGB_tmp.led.r = rnd[0];
	RGB_tmp.led.g = rnd[0];
	RGB_tmp.led.b = rnd[0];

	next = rnd[1]&127;

	if(next < 5)
		next = 5;

	if(effect_state == 255)// exit
		return;
	// NB: user can't see the value update on the PWM signal. It would just confuse them.
	value_RGB(RGB_tmp.led.r,RGB_tmp.led.g,RGB_tmp.led.b,RGB_tmp.led.brightness);
	ctimer_set(&RGB_effect_timer, next, RGB_RANDOM_RUN, NULL);
}

#endif
/*---------------------------------------------------------------------------*/
