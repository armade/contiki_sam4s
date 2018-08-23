
#include "contiki-conf.h"
#include "platform-conf.h"
#include "compiler.h"
#include "lib/sensors.h"
#include "Soft_rgb.h"
#include <stdint.h>

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



volatile RGB_t RGB = {0}; // True output
volatile RGB_t RGB_reload = {0}; // True reload output
RGB_t user_set; // User set value (not modified by brightness)

uint8_t counter=0;

#define SENSOR_STATUS_DISABLED     0
#define SENSOR_STATUS_INITIALISED  1
#define SENSOR_STATUS_NOT_READY    2
#define SENSOR_STATUS_READY        3

static int Sensor_status = SENSOR_STATUS_DISABLED;

/*---------------------------------------------------------------------------*/


void TC2_Handler(void)
{
	RGB_TIMER.TC_SR;

	if(counter == 0)
	{
		RGB.all = RGB_reload.all;
		// If zero don't bother turning them on
		if(0 != RGB.led.r)		RGB_base->PIO_SODR = RGB_R_GPIO;
		else 					RGB_base->PIO_CODR = RGB_R_GPIO;

		if(0 != RGB.led.g)		RGB_base->PIO_SODR = RGB_G_GPIO;
		else					RGB_base->PIO_CODR = RGB_G_GPIO;

		if(0 != RGB.led.b)		RGB_base->PIO_SODR = RGB_B_GPIO;
		else					RGB_base->PIO_CODR = RGB_B_GPIO;
	}
	else
	{
		if(counter == RGB.led.r)	RGB_base->PIO_CODR = RGB_R_GPIO;
		if(counter == RGB.led.g)	RGB_base->PIO_CODR = RGB_G_GPIO;
		if(counter == RGB.led.b)	RGB_base->PIO_CODR = RGB_B_GPIO;
	}
	counter++;

}
/*---------------------------------------------------------------------------*/
/**
 * \brief Set value (0-255)
 */
static int
value_RGB(uint8_t R,uint8_t G,uint8_t B, uint8_t brightness)
{

	if(brightness>128)
		brightness = 128;

	RGB_reload.led.r = (R*brightness)>>7; // divide by 128
	RGB_reload.led.g = (G*brightness)>>7;
	RGB_reload.led.b = (B*brightness)>>7;
	RGB_reload.led.brightness = brightness;

	sensors_changed(&soft_RGB_ctrl_sensor);
	return 1;
}

int
RGB_set(int value)
{
	RGB_t *temp = (RGB_t *)&value;
	if(value != SENSOR_ERROR){
		user_set.all = temp->all;
		value_RGB(temp->led.r,temp->led.g,temp->led.b,temp->led.brightness);
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
	RGB_TIMER.TC_RC=147; //147 er 25.510kHz, 39.2us (25.510kHz/255 ~ 100Hz)
#else //30Mhz
	RGB_TIMER.TC_CMR= 1 + (2<<13); //1=MCK/8   2<<13=up rc compare
	RGB_TIMER.TC_RC=147; //375 er 25.510kHz, 39.2us (25.510kHz/255 ~ 100Hz)
#endif

	//RGB_TIMER.TC_CCR=1;
	//RGB_TIMER.TC_CCR=4;
	RGB_TIMER.TC_IER=1<<4; //CPCS
	NVIC_ClearPendingIRQ(RGB_TIMER_IRQ);
	NVIC_SetPriority((IRQn_Type) RGB_TIMER_ID, 0);
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
			configure_RGB();
			Sensor_status = SENSOR_STATUS_INITIALISED;
			//RGB_TEST(RGB_base);
			break;

		case SENSORS_ACTIVE:
			if(Sensor_status == SENSOR_STATUS_DISABLED)
				return SENSOR_STATUS_DISABLED;

			 if(enable) {
				 counter = 0;
				 RGB_TIMER.TC_CCR=1;
				 RGB_TIMER.TC_CCR=4;
				 Sensor_status = SENSOR_STATUS_READY;
			 } else {
				 RGB_TIMER.TC_CCR=2;
				 RGB_base->PIO_OER =  RGB_R_GPIO | RGB_G_GPIO | RGB_B_GPIO;
				 RGB_base->PIO_CODR = RGB_R_GPIO | RGB_G_GPIO | RGB_B_GPIO;
				 Sensor_status = SENSOR_STATUS_INITIALISED;
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
#if 0
struct ctimer RGB_timer;
volatile unsigned first = 1;
volatile unsigned test = (1<<0) | (128<<8) | (128<<16);

unsigned state = 0;
RGB_t RGB_tmp;

static void
RGB_TEST(void *data)
{
	clock_time_t next = 40;
	if(first)
	{
		RGB_init(SENSORS_ACTIVE,1);
		first = 0;
		RGB_tmp.led.brightness = 128;
	}
	switch(state)
	{
		case 0:
			RGB_tmp.led.r++;
			if(RGB_tmp.led.r == 255){
				state++;
				RGB_tmp.led.r = 0;
			}
			break;

		case 1:
			RGB_tmp.led.g++;
			if(RGB_tmp.led.g == 255){
				state++;
				RGB_tmp.led.g = 0;
			}
			break;

		case 2:
			RGB_tmp.led.b++;
			if(RGB_tmp.led.b == 255){
				state = 0;
				RGB_tmp.led.b = 0;
			}
			break;
	}
	RGB_set(RGB_tmp.all);
	ctimer_set(&RGB_timer, next, RGB_TEST, NULL);
}
#endif

#endif
/*---------------------------------------------------------------------------*/
