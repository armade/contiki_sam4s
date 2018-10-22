/**
 * \file
 *
 * \brief User Interface
 *
 * Copyright (c) 2016 Atmel Corporation. All rights reserved.
 *
 * \asf_license_start
 *
 * \page License
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * 3. The name of Atmel may not be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * 4. This software may only be redistributed and used in connection with an
 *    Atmel microcontroller product.
 *
 * THIS SOFTWARE IS PROVIDED BY ATMEL "AS IS" AND ANY EXPRESS OR IMPLIED
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT ARE
 * EXPRESSLY AND SPECIFICALLY DISCLAIMED. IN NO EVENT SHALL ATMEL BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 * STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 * \asf_license_stop
 *
 */
/*
 * Support and FAQ: visit <a href="http://www.atmel.com/design-support/">Atmel Support</a>
 */

#include "conf_usb_host.h"
#include <same70.h>
#include "stdio.h"

/**
 * \name Main user interface functions
 * @{
 */
void ui_init(void)
{
	printf("%s\n",__func__);
	/* Initialize LEDs */
	//LED_Off(LED0);
	//LED_Off(LED1);
}

void ui_usb_mode_change(bool b_host_mode)
{
	printf("%s\n",__func__);
	(void)b_host_mode;
	ui_init();
}
/*! @} */

/**
 * \name Host mode user interface functions
 * @{
 */

/*! Status of device enumeration */
static uhc_enum_status_t ui_enum_status=UHC_ENUM_DISCONNECT;
/*! Blink frequency depending on device speed */
static uint16_t ui_device_speed_blink;

void ui_usb_vbus_change(bool b_vbus_present)
{
	printf("%s\n",__func__);
	/*if (b_vbus_present) {
		LED_On(LED1);
	} else {
		LED_Off(LED1);
	}*/
}

void ui_usb_vbus_error(void)
{
	printf("%s\n",__func__);
}
extern volatile char enumeration_complete;
void ui_usb_connection_event(uhc_device_t *dev, bool b_present)
{
	printf("%s\n",__func__);
	(void)dev;
	if (b_present) {
		//LED_On(LED1);
		enumeration_complete=1;
	} else {
		//LED_Off(LED1);
		ui_enum_status = UHC_ENUM_DISCONNECT;
		enumeration_complete=0;
	}
	//LED_Off(LED0);
}

void ui_usb_enum_event(uhc_device_t *dev, uhc_enum_status_t status)
{
	printf("%s\n",__func__);
	ui_enum_status = status;
	switch (dev->speed) {
	case UHD_SPEED_HIGH:
		ui_device_speed_blink = 250;
		printf("UHD_SPEED_HIGH\n");
		break;
	case UHD_SPEED_FULL:
		ui_device_speed_blink = 500;
		printf("UHD_SPEED_FULL\n");
		break;
	case UHD_SPEED_LOW:
	default:
		ui_device_speed_blink = 1000;
		printf("UHD_SPEED_LOW\n");
		break;
	}
	if (ui_enum_status == UHC_ENUM_SUCCESS) {
		/* USB Device CDC connected
		   Open and configure UART and USB CDC ports */
		usb_cdc_line_coding_t cfg = {
			.dwDTERate   = CPU_TO_LE32(115200),
			.bCharFormat = CDC_STOP_BITS_1,
			.bParityType = CDC_PAR_NONE,
			.bDataBits   = 8,
		};
		//uart_open();
		//uart_config(&cfg);
		uhi_cdc_open(0, &cfg);
	}
}

void ui_usb_wakeup_event(void)
{
	printf("%s\n",__func__);
}

void ui_usb_sof_event(void)
{
	static uint16_t counter_sof = 0;

	//printf("%s\n",__func__);

	if (ui_enum_status == UHC_ENUM_SUCCESS) {
		/* Display device enumerated and in active mode */
		if (++counter_sof > ui_device_speed_blink) {
			counter_sof = 0;
			//LED_Toggle(LED0);
		}
	}
}

void ui_com_rx_start(void)
{
	printf("%s\n",__func__);
	//LED_On(LED1);
}

void ui_com_rx_stop(void)
{
	printf("%s\n",__func__);
	//LED_Off(LED1);
}

void ui_com_tx_start(void)
{
	printf("%s\n",__func__);
	//LED_On(LED1);
}

void ui_com_tx_stop(void)
{
	printf("%s\n",__func__);
	//LED_Off(LED1);
}

void ui_com_error(void)
{
	printf("%s\n",__func__);
}

void ui_com_overflow(void)
{
	printf("%s\n",__func__);
}

/*! @} */

/**
 * \defgroup UI User Interface
 *
 * Human interface on SAMV71-Xplained-Ultra:
 * - Led 1 is on when USB OTG cable is plugged in and Vbus is present
 * - Led 1 is continuously on when a device is connected
 * - Led 0 blinks when USB host has checked and enabled CDC interface
 *   - The blink is slow (1s) with low speed device
 *   - The blink is normal (0.5s) with full speed device
 *   - The blink is fast (0.25s) with high speed device
 * - Led 1 is on during data transfer between CDC and UART
 */