#include "contiki.h"
#include "leds.h"
#include "same70.h"
#include "compiler.h"
#include "gpio.h"
#include "drivers/pmc.h"

#define LEDS_GPIO_PIN_MASK   LEDS_ALL
/*---------------------------------------------------------------------------*/
void leds_arch_init(void)
{
	uint32_t clk_config;
	pmc_enable_periph_clk(ID_PWM0);

	PWM0->PWM_DIS = (1<<0) | (1<<1) | (1<<2) | (1<<3);
	PWM0->PWM_OOV = (1<<1) | (1<<2); // overwrite to high on ch 1 and ch 2 and 0 on ch 0
	PWM0->PWM_OS = (1<<1) | (1<<2) | (1<<0); // Output override value

	pio_set_peripheral(PIOA, PIO_PERIPH_B, PIO_PA23); 	// pwm0H -> R
	pio_set_peripheral(PIOA, PIO_PERIPH_B, PIO_PA24); 	// pwm1H -> B
	pio_set_peripheral(PIOA, PIO_PERIPH_B, PIO_PA25);	// pwm2H -> G

	// PWM clock = 585938Hz/2
	clk_config = (9<<0); //CLKA no divide mck/512

	// 585938Hz/2/4096 = 72Hz => more that enough
	PWM0->PWM_CH_NUM[0].PWM_CPRD = 4096;
	PWM0->PWM_CH_NUM[0].PWM_CDTY = 200;
	PWM0->PWM_CH_NUM[0].PWM_CMR = (1<<17) |clk_config;

	PWM0->PWM_CH_NUM[1].PWM_CPRD = 4096;
	PWM0->PWM_CH_NUM[1].PWM_CDTY = 96;
	PWM0->PWM_CH_NUM[1].PWM_CMR =  clk_config;

	PWM0->PWM_CH_NUM[2].PWM_CPRD = 4096;
	PWM0->PWM_CH_NUM[2].PWM_CDTY = 96;
	PWM0->PWM_CH_NUM[2].PWM_CMR = clk_config;

	PWM0->PWM_ENA = (1<<0) | (1<<1) | (1<<2);
}
/*---------------------------------------------------------------------------*/
unsigned char leds_arch_get(void)
{
	unsigned char leds = 0;
	unsigned char leds_status;

	leds_status = (~PWM0->PWM_OS) & 0b111;
	if(leds_status & (1<<2))
		leds |= LEDS_GREEN;

	if(leds_status & (1<<1))
		leds |= LEDS_BLUE;

	if(leds_status & (1<<0))
		leds |= LEDS_RED;

	return leds;
}
/*---------------------------------------------------------------------------*/
void leds_arch_set(unsigned char leds)
{
	if(leds & LEDS_GREEN)
		PWM0->PWM_OSC = (1<<2);// Remove overwrite (1 to pwm)
	else
		PWM0->PWM_OSS = (1<<2);

	if(leds & LEDS_BLUE)
		PWM0->PWM_OSC = (1<<1);// Remove overwrite (1 to pwm)
	else
		PWM0->PWM_OSS = (1<<1);

	if(leds & LEDS_RED)
		PWM0->PWM_OSC = (1<<0);// Remove overwrite (0 to pwm)
	else
		PWM0->PWM_OSS = (1<<0);
}
/*---------------------------------------------------------------------------*/


