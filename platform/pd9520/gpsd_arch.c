/*
 * slip_arch.c
 *
 *  Created on: 5. jul. 2017
 *      Author: pm
 */
#include "slip.h"

#include "compiler.h"
#include "udi_cdc.h"
#include "drivers/uart.h"
#include "drivers/sysclk.h"
#include "same70.h"
#include "gpio.h"
extern void gpsd_put_char(uint8_t c);

struct ctimer gps_config_timer;
#ifdef NODE_GPS

//https://github.com/f5eng/mt3339-utils

static uint8_t cmd_index = 0;
static const char init_cmd[]={
	"$PMTK301,2*2E\r\n"// Set DGPS mode to SBAS
	"$PMTK313,1*2E\r\n"// Set SBAS Enabled
	"$PMTK314,0,2,2,2,0,0,0,0,0,0,0,0,0,0,0,0,0,5,0*2F\r\n"// Set NMEA Sentence Output: GLL,RMC,VTG,GGA,GSA,GSV...........ZDA
	//"$PMTK314,0,1,1,1,0,0,0,0,0,0,0,0,0,0,0,0,0,1,0*28\r\n"
	"$PMTK319,1*24\r\n"// Set SBAS 'Integrity' Mode
	"$PMTK225,0*2B\r\n"// Disable Periodic Mode
	"$PMTK286,1*23\r\n"// Enable AIC Mode
	"$PMTK869,1,1*35\r\n"// Enable EASY Mode
	"$PMTK386,0.27*08\r\n" //Set static threshold to 0.27[m/s] (1 [km/hr])
};
/*
	18 NMEA_SEN_MCHN, // PMTKCHN interval � GPS channel status ---- info here >> http://snoob-community.wikispaces.com/PMTKCHN
	   decoding >>  $PMTKCHN,aabbc,...*XX\r\n
			 aa => SVid: Space Vehicle Id
			   bb => SNR: Signal to Noise Ratio
				 c => Status:
					  � 0 == Idle
					  � 1 == Searching
					  � 2 == Tracking
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
	pio_set_peripheral(PIOA, PIO_PERIPH_C, PIO_PA5);
	pio_set_peripheral(PIOA, PIO_PERIPH_C, PIO_PA6);

	// Initialize UART
	uart_init(UART1,&uart_settings);
	uart_reset((Uart *)UART1);
	uart_enable((Uart *)UART1);

	NVIC_EnableIRQ((IRQn_Type)ID_UART1);
	NVIC_SetPriority((IRQn_Type) ID_UART1, 12);//level 0 is the highest interrupt priority (0-15)

	// Enable UART IRQ
	uart_enable_interrupt(UART1, US_IER_RXRDY);

	ctimer_set(&gps_config_timer,2* CLOCK_SECOND, gps_config_init, NULL);
}
#endif
