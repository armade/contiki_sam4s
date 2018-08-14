/*
 * slip_arch.c
 *
 *  Created on: 5. jul. 2017
 *      Author: pm
 */
#include "slip.h"

#include "compiler.h"
#include "udi_cdc.h"
#include "uart.h"
#include "sam4s.h"
#include "gpio.h"
extern void gpsd_put_char(uint8_t c);


#ifdef NODE_GPS
									    //GLL,RMC,VTG,GGA,GSA,GSV			ZDA
static const uint8_t PMTK314[] = "$PMTK314,0,2,2,2,0,0,0,0,0,0,0,0,0,0,0,0,0,5,0*2F\r\n";
// this means only RMC,VTG,GGA and ZDA output. ZDA is only send every 5 locks and all other is send ever 2 locks.
volatile uint8_t *config = (volatile uint8_t *)PMTK314;


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
		if(*config)
			uart_write(UART1,*config++);
		else
			uart_disable_interrupt(UART1,US_IDR_TXEMPTY);
	}
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
	uart_enable_interrupt(UART1, US_IER_RXRDY | US_IER_TXEMPTY);

	uart_write(UART1,*config++);

}
#endif
