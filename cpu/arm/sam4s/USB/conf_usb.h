/**
 * \file
 *
 * \brief USB configuration file for CDC application
 *
 * Copyright (c) 2009-2016 Atmel Corporation. All rights reserved.
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

#ifndef _CONF_USB_H_
#define _CONF_USB_H_

//! \brief Serial number to be passed to USB stack
//extern uint8_t cdc_serial_number[];
/**
 * USB Device Configuration
 * @{
 */
#define UDD_NO_SLEEP_MGR 1
//! Device definition (mandatory)
#define  USB_DEVICE_VENDOR_ID             0x1B8B //USB_VID_ATMEL
# define  USB_DEVICE_PRODUCT_ID           0xA956 //USB_PID_ATMEL_ASF_CDC

#define  USB_DEVICE_MAJOR_VERSION         1
#define  USB_DEVICE_MINOR_VERSION         0
#define  USB_DEVICE_POWER                 100 // Consumption on Vbus line (mA)
#define  USB_DEVICE_ATTR                  \
	(USB_CONFIG_ATTR_SELF_POWERED)
// (USB_CONFIG_ATTR_BUS_POWERED)
//	(USB_CONFIG_ATTR_REMOTE_WAKEUP|USB_CONFIG_ATTR_SELF_POWERED)
//	(USB_CONFIG_ATTR_REMOTE_WAKEUP|USB_CONFIG_ATTR_BUS_POWERED)

//! USB Device string definitions (Optional)
#define  USB_DEVICE_MANUFACTURE_NAME      "PROCES-DATA A/S"
#define  USB_DEVICE_PRODUCT_NAME          "PD956 - Hardware secure module"
#define  USB_DEVICE_SERIAL_NAME
#define  USB_DEVICE_GET_SERIAL_NAME_POINTER cdc_serial_number
#define  USB_DEVICE_GET_SERIAL_NAME_LENGTH  10


/**
 * Device speeds support
 * Low speed not supported by CDC
 * @{
 */
//! To authorize the High speed
//@}
//static void dummy_func(void){}

/**
 * USB Device Callbacks definitions (Optional)
 * @{
 */
#define  UDC_VBUS_EVENT(b_vbus_high)
//#define  UDC_SOF_EVENT() dummy_func
//#define  UDC_SUSPEND_EVENT() dummy_func
//#define  UDC_RESUME_EVENT() dummy_func
//! Mandatory when USB_DEVICE_ATTR authorizes remote wakeup feature
// #define  UDC_REMOTEWAKEUP_ENABLE()        user_callback_remotewakeup_enable()
// extern void user_callback_remotewakeup_enable(void);
// #define  UDC_REMOTEWAKEUP_DISABLE()       user_callback_remotewakeup_disable()
// extern void user_callback_remotewakeup_disable(void);
#ifdef USB_DEVICE_LPM_SUPPORT
#define  UDC_SUSPEND_LPM_EVENT()          dummy_func()
#define  UDC_REMOTEWAKEUP_LPM_ENABLE()    dummy_func()
#define  UDC_REMOTEWAKEUP_LPM_DISABLE()   dummy_func()
#endif
//! When a extra string descriptor must be supported
//! other than manufacturer, product and serial string
// #define  UDC_GET_EXTRA_STRING()
//@}

//@}


/**
 * USB Interface Configuration
 * @{
 */
/**
 * Configuration of CDC interface
 * @{
 */

#if ((USB_DEVICE_PRODUCT_ID == 0xA956) || USB_DEVICE_PRODUCT_ID == 0xE956)
//! Define two USB communication ports
#define  UDI_CDC_PORT_NB 2
#else
#define  UDI_CDC_PORT_NB 1
#endif

extern void rx_poll(void);
extern volatile char enumeration_complete;
__attribute__((used))
static char main_cdc_enable(char port)
{
	enumeration_complete = 1;

	return 1;
}
__attribute__((used))
static void main_cdc_disable(char port)
{
	enumeration_complete = 0;
}

//! Interface callback definition
#define  UDI_CDC_ENABLE_EXT(port) main_cdc_enable(port)
#define  UDI_CDC_DISABLE_EXT(port) main_cdc_disable(port)
#define  UDI_CDC_RX_NOTIFY(port) rx_poll()
#define  UDI_CDC_TX_EMPTY_NOTIFY(port)
#define  UDI_CDC_SET_CODING_EXT(port,cfg)
#define  UDI_CDC_SET_DTR_EXT(port,set)
#define  UDI_CDC_SET_RTS_EXT(port,set)

//! Define it when the transfer CDC Device to Host is a low rate (<512000 bauds)
//! to reduce CDC buffers size
#define  UDI_CDC_LOW_RATE

//! Default configuration of communication port
#define  UDI_CDC_DEFAULT_RATE             115200
#define  UDI_CDC_DEFAULT_STOPBITS         CDC_STOP_BITS_1
#define  UDI_CDC_DEFAULT_PARITY           CDC_PAR_NONE
#define  UDI_CDC_DEFAULT_DATABITS         8
//@}
//@}


/**
 * USB Device Driver Configuration
 * @{
 */
//@}

#include "udi_cdc_conf.h"

#endif // _CONF_USB_H_
