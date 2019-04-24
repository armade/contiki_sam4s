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
#include "Neo_Pixel_Ring.h"
#include "usart.h"
#include "gpio.h"
#include "usart_spi.h"
#include <stdint.h>

#include "board-peripherals.h"
//#ifdef NODE_NEO_PIXEL_RING
/*---------------------------------------------------------------------------*/
#define DEBUG 0
#if DEBUG
#define PRINTF(...) printf(__VA_ARGS__)
#else
#define PRINTF(...)
#endif


static int Sensor_status = SENSOR_STATUS_DISABLED;

/*---------------------------------------------------------------------------*/
#define NR_OF_LEDS 12

/*---------------------------------------------------------------------------*/
/**
 * Composition of 24 bit data
 * _________________________________________________________________________
 * |g7|g6|g5|g4|g3|g2|g1|g0|r7|r6|r5|r4|r3|r2|r1|r0|b7|b6|b5|b4|b3|b2|b1|b0|
 *
 * 			__  t1
 * 0 code  |t0|_____|    t0 = 350ns +-150ns    t1 = 800ns +- 150ns
 *
 * 			_____ t1
 * 1 code  |t0   |__|     t0 = 700ns +-150ns    t1 = 600ns +- 150ns
 *
 *  			t0
 * ret code  |______|    t0 > 50us
 */


uint8_t LED_chain[NR_OF_LEDS][3];

static void
Neo_Pixel_Ring_sync()
{
	// TODO: Setup PDC
	uint8_t led_cou,i;
	uint32_t val;

	for(led_cou = 0; led_cou < NR_OF_LEDS; led_cou++)
	{
		val = (LED_chain[led_cou][0]<<16) | (LED_chain[led_cou][1]<<8) | (LED_chain[led_cou][2]<<0);

		for(i=24;i>0;i--){
			if((val>>i)&1)
				usart_spi_write_single(USART0,0x1C); // write 1  (0xE0>>3)
			else
				usart_spi_write_single(USART0,0x10); // write 0  (0x80>>3)
		}
	}
	// The last bit is alway 0 -- Can't figure out
	// if the last bit must have a high transition.
	// if not just ensure no one calls sync again within 50 us.
}


int
Neo_Pixel_Ring_set(int value)
{
	Led_element_t *LED = (Led_element_t *)&value;

	if(LED->led_nr == 255)
		Neo_Pixel_Ring_sync();
	else if(LED->led_nr < NR_OF_LEDS){
		LED_chain[LED->led_nr][0] = LED->green;
		LED_chain[LED->led_nr][1] = LED->red;
		LED_chain[LED->led_nr][2] = LED->blue;
	}
	return 0;
}
/*---------------------------------------------------------------------------*/
// Perhaps!!!.
// 120MHz/18 = 6666666.66667 MHz
// 1/6666666.66667 MHz = 150ns
// 8/6666666.66667 MHz = 1200ns
// 0b11000000 = 0  ~ 300ns
// 0b11111000 = 1  ~ 750ns

// now!!!.
// 120MHz/30 = 4000000 MHz
// 1/4000000 MHz = 250ns
// 5/4000000 MHz = 1250ns
// 0b10000 = 0  ~ 500ns
// 0b11100 = 1  ~ 750ns
int
configure_Neo_Pixel_Ring(void)
{
	usart_spi_opt_t opt;

	// Enabling the peripheral clock
	sysclk_enable_peripheral_clock(ID_USART0);
	pio_set_peripheral(PIOA, PIO_PERIPH_A, PIO_PA6);

	// Initialize UART
	opt.baudrate = 4000000;
	opt.char_length = US_MR_CHRL_5_BIT;
	opt.spi_mode = 0;
	opt.channel_mode = US_MR_CHMODE_NORMAL;

	/* Initialize the USART module as SPI master. */
	usart_init_spi_master(USART0, &opt, sysclk_get_peripheral_hz());
	usart_enable_tx(USART0);

	return 1;
}
#if 0

/* Pdc transfer buffer */
uint8_t USART_tx_buf[BUFFER_SIZE];
/* PDC data packet for transfer */
pdc_packet_t USART_packet;
/* Pointer to USART PDC register base */
Pdc *USART_pdc;

/* Get pointer to USART PDC register base */
	USART_pdc = usart_get_pdc_base(USART0);

	/* Initialize PDC data packet for transfer */
	USART_packet.ul_addr = (uint32_t) USART_tx_buf;
	USART_packet.ul_size = BUFFER_SIZE;

	/* Configure PDC for data transmit */
	pdc_tx_init(USART_pdc, &USART_packet, NULL);

	/* Enable PDC transfers */
	pdc_enable_transfer(USART_pdc, PERIPH_PTCR_TXTEN);

	/* Enable USART IRQ */
	usart_enable_interrupt(CONSOLE_UART, US_IER_TXBUFF);

	/* Enable USART interrupt */
	NVIC_EnableIRQ(CONSOLE_USART_IRQn);
#endif


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
Neo_Pixel_Ring_init(int type, int enable)
{
	switch(type) {

		case SENSORS_HW_INIT:
			configure_Neo_Pixel_Ring();
			Sensor_status = SENSOR_STATUS_INITIALISED;

			break;

		case SENSORS_ACTIVE:
			if(Sensor_status == SENSOR_STATUS_DISABLED)
				return SENSOR_STATUS_DISABLED;

			 if(enable) {
				 // start
				 Sensor_status = SENSOR_STATUS_READY;
			 } else {
				 // stop
				 Sensor_status = SENSOR_STATUS_INITIALISED;
			 }
			break;
	}
	return Sensor_status;
}
/*---------------------------------------------------------------------------*/
static int
Neo_Pixel_Ring_status(int type)
{
	return Sensor_status;
}
/*---------------------------------------------------------------------------*/
SENSORS_SENSOR(
		Neo_Pixel_Ring_sensor,
		"Neo_Pixel_Ring",
		Neo_Pixel_Ring_set,
		Neo_Pixel_Ring_init,
		Neo_Pixel_Ring_status);
/*---------------------------------------------------------------------------*/


//#endif
/*---------------------------------------------------------------------------*/
