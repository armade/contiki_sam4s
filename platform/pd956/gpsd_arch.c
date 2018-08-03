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
extern void gpsd_put_char(uint8_t c);

#ifdef NODE_GPS

void UART1_Handler(void)
{
	if (uart_is_rx_ready(UART1))
	{
		uint32_t rx_data = 0;
		if (uart_read(UART1, (uint8_t *)&rx_data) == 0)
		{
			gpsd_put_char(rx_data);
		}
	}
}

void gpsd_arch_init(void)
{
	const sam_uart_opt_t uart_settings = {
		.ul_mck			= sysclk_get_peripheral_hz(),
		.ul_baudrate   	= 9600,
		.ul_mode		= US_MR_CHMODE_NORMAL
	};

	// Enabling the peripheral clock
	sysclk_enable_peripheral_clock(ID_UART1);

	// Initialize UART
	uart_init(UART1,&uart_settings);

	// Enable UART IRQ
	uart_enable_interrupt(UART1, US_IER_RXRDY);

}
#endif
