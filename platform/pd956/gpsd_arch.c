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
#include "slip.h"

#include "compiler.h"
#include "udi_cdc.h"
#include "uart.h"
#include "sam4s.h"
#include "gpio.h"
extern void gpsd_put_char(uint8_t c);

struct ctimer gps_config_timer;
#ifdef NODE_GPS

//https://github.com/f5eng/mt3339-utils

static uint8_t cmd_index = 0;
static const char init_cmd[]={
	"$PMTK301,2*2E\r\n"// Set DGPS mode to SBAS
	"$PMTK313,1*2E\r\n"// Set SBAS Enabled
	//"$PMTK314,0,2,2,2,0,0,0,0,0,0,0,0,0,0,0,0,0,5,0*2F\r\n"// Set NMEA Sentence Output: GLL,RMC,VTG,GGA,GSA,GSV...........ZDA
	"$PMTK314,0,1,1,1,0,0,0,0,0,0,0,0,0,0,0,0,0,1,0*28\r\n"
	"$PMTK319,1*24\r\n"// Set SBAS 'Integrity' Mode
	"$PMTK225,0*2B\r\n"// Disable Periodic Mode
	"$PMTK286,1*23\r\n"// Enable AIC Mode
	"$PMTK869,1,1*35\r\n"// Enable EASY Mode
	"$PMTK386,0.27*08\r\n" //Set static threshold to 0.27[m/s] (1 [km/hr])
};
/*
	18 NMEA_SEN_MCHN, // PMTKCHN interval – GPS channel status ---- info here >> http://snoob-community.wikispaces.com/PMTKCHN
	   decoding >>  $PMTKCHN,aabbc,...*XX\r\n
			 aa => SVid: Space Vehicle Id
			   bb => SNR: Signal to Noise Ratio
				 c => Status:
					  • 0 == Idle
					  • 1 == Searching
					  • 2 == Tracking
*/

static volatile char *config_init = (volatile char *)&init_cmd[0];


void UART1_Handler(void)
{
	if (uart_is_rx_ready(UART1)){

		uint32_t rx_data = 0;
		if (uart_read(UART1, (uint8_t *)&rx_data) == 0)
		{
			gpsd_put_char(rx_data);
		}
	}
	if(UART1->UART_IMR & US_CSR_TXEMPTY){

		UART1->UART_THR = init_cmd[++cmd_index];
		uart_write(UART1,*config_init++);

		if(cmd_index == sizeof(init_cmd)){
			uart_disable_interrupt(UART1,US_IDR_TXEMPTY);
		}
	}
}

void
gps_config_init(void *ptr)
{
	UART1->UART_THR = init_cmd[++cmd_index];
	uart_enable_interrupt(UART1, US_IER_TXEMPTY);
}

void gpsd_arch_init(void)
{

	const sam_uart_opt_t uart_settings = {
		.ul_mck			= sysclk_get_peripheral_hz(),
		.ul_baudrate   	= 9600,
		.ul_mode		= US_MR_CHMODE_NORMAL|US_MR_PAR_NO
	};

	// Enabling the peripheral clock
	sysclk_enable_peripheral_clock(ID_UART1);
	pio_set_peripheral(PIOB, PIO_PERIPH_A, PIO_PB2);
	pio_set_peripheral(PIOB, PIO_PERIPH_A, PIO_PB3);

	// Initialize UART
	uart_init(UART1,&uart_settings);
	uart_reset((Uart *)UART1);
	uart_enable((Uart *)UART1);

	NVIC_EnableIRQ((IRQn_Type)ID_UART1);

	// Enable UART IRQ
	uart_enable_interrupt(UART1, US_IER_RXRDY);

	ctimer_set(&gps_config_timer,2* CLOCK_SECOND, gps_config_init, NULL);
}
#endif
