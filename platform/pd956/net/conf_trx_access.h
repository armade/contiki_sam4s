/**
 * \file *********************************************************************
 *
 * \brief Common TRX Access Configuration
 *
 * Copyright (c) 2013-2016 Atmel Corporation. All rights reserved.
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
 */

#ifndef CONF_TRX_ACCESS_H_INCLUDED
#define CONF_TRX_ACCESS_H_INCLUDED


#define LOW 0
#define HIGH 1


#include <pio.h>
#include "pio_handler.h"

#define PIN_USART1_SCK    		{PIO_PA23A_SCK1, PIOA, ID_PIOA, PIO_PERIPH_A, PIO_DEFAULT}
#define PIN_USART1_SCK_IDX        (PIO_PA23_IDX)
#define PIN_USART1_SCK_FLAGS      (PIO_PERIPH_A | PIO_DEFAULT)


/** SPI MISO pin definition. */
#define SPI_MISO_GPIO             (PIO_PA12_IDX)
#define SPI_MISO_FLAGS       (PIO_PERIPH_A | PIO_DEFAULT)
/** SPI MOSI pin definition. */
#define SPI_MOSI_GPIO             (PIO_PA13_IDX)
#define SPI_MOSI_FLAGS       (PIO_PERIPH_A | PIO_DEFAULT)
/** SPI SPCK pin definition. */
#define SPI_SPCK_GPIO             (PIO_PA14_IDX)
#define SPI_SPCK_FLAGS       (PIO_PERIPH_A | PIO_DEFAULT)
/** SPI chip select 0 pin definition. (Only one configuration is possible) */
#define SPI_NPCS0_GPIO            (PIO_PA11_IDX)
#define SPI_NPCS0_FLAGS           (PIO_PERIPH_A | PIO_DEFAULT)


#define AT86RFX_SPI                  USART1
#define AT86RFX_RST_PIN              IOPORT_CREATE_PIN(PIOA, 25)
#define AT86RFX_IRQ_PIN              IOPORT_CREATE_PIN(PIOA, 15)
#define AT86RFX_SLP_PIN              IOPORT_CREATE_PIN(PIOA, 1)
#define AT86RFX_SPI_CS               IOPORT_CREATE_PIN(PIOA, 24)
#define AT86RFX_SPI_CS_PIN           IOPORT_CREATE_PIN(PIOA, 24)
#define AT86RFX_SPI_CS_FLAGS         SPI_NPCS0_FLAGS
#define AT86RFX_SPI_MOSI             IOPORT_CREATE_PIN(PIOA, 22)
#define AT86RFX_SPI_MISO             IOPORT_CREATE_PIN(PIOA, 21)
#define AT86RFX_SPI_SCK              IOPORT_CREATE_PIN(PIOA, 23)
#define AT86RFX_CSD     		     0
#define AT86RFX_CPS 	             0

#define AT86RFX_SPI_BAUDRATE         (6000000)

#define AT86RFX_INTC_INIT()         pio_configure_pin(AT86RFX_IRQ_PIN,	\
		PIO_TYPE_PIO_INPUT); \
	ioport_set_pin_sense_mode(AT86RFX_IRQ_PIN, IOPORT_SENSE_RISING); \
	pmc_enable_periph_clk(ID_PIOA);	\
	pio_set_debounce_filter(PIOA, PIO_PA15, 10); \
	pio_handler_set(PIOA, ID_PIOA, PIO_PA15, PIO_IT_HIGH_LEVEL, at86rfx_isr); \
	NVIC_EnableIRQ((IRQn_Type)ID_PIOA); \
	NVIC_SetPriority((IRQn_Type) ID_PIOA, 2);\
	pio_enable_interrupt(PIOA, PIO_PA15);
	//level 0 is the highest interrupt priority (0-15)
#define AT86RFX_ISR()               void at86rfx_isr(uint32_t a, uint32_t b)

/** Enables the transceiver main interrupt. */
#define ENABLE_TRX_IRQ()            pio_enable_pin_interrupt(AT86RFX_IRQ_PIN)

/** Disables the transceiver main interrupt. */
#define DISABLE_TRX_IRQ()           pio_disable_pin_interrupt(AT86RFX_IRQ_PIN)

/** Clears the transceiver main interrupt. */
#define CLEAR_TRX_IRQ()             NVIC_ClearPendingIRQ(PIOA_IRQn);

/*
 * This macro saves the trx interrupt status and disables the trx interrupt.
 */
#define ENTER_TRX_REGION()          pio_disable_pin_interrupt(AT86RFX_IRQ_PIN);

/*
 *  This macro restores the transceiver interrupt status
 */
#define LEAVE_TRX_REGION()         pio_enable_pin_interrupt(AT86RFX_IRQ_PIN)


#endif /* CONF_TRX_ACCESS_H_INCLUDED */
