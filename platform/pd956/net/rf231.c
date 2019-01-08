/*
 * Copyright (c) 2013, Thingsquare, http://www.thingsquare.com/.
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
 /**
* Copyright (c) 2015 Atmel Corporation and 2012 – 2013, Thingsquare, http://www.thingsquare.com/. All rights reserved. 
*  
* Redistribution and use in source and binary forms, with or without 
* modification, are permitted provided that the following conditions are met:
* 
* 1. Redistributions of source code must retain the above copyright notice, this
* list of conditions and the following disclaimer.
* 
* 2. Redistributions in binary form must reproduce the above copyright notice, 
* this list of conditions and the following disclaimer in the documentation 
* and/or other materials provided with the distribution.
* 
* 3. Neither the name of Atmel nor the name of Thingsquare nor the names of its contributors may be used to endorse or promote products derived 
* from this software without specific prior written permission.  
* 
* 4. This software may only be redistributed and used in connection with an 
* Atmel microcontroller or Atmel wireless product.
* 
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" 
* AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE 
* IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE 
* ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE 
* LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR 
* CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE 
* GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) 
* HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, 
* STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY 
* OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
* 
* 
* 
*/

#include <stdlib.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <pio.h>
#include <ioport.h>
#include "contiki.h"
#include "leds.h"
#include "rtimer.h"
#include "netstack.h"
#include "net/packetbuf.h"
#include "rf231-const.h"
#include "rf231-config.h"
#include "rf231-arch.h"
#include "trx_access.h"
#include "contiki-conf.h"
#include "platform-conf.h"
//#include "system_interrupt.h"
#include "sam4s.h"
#include "rf231.h"
#include "csprng.h"
//#define RF231_STATUS()                    RF231_status()
/*---------------------------------------------------------------------------*/
PROCESS(RF231_radio_process, "RF231 radio driver");
static int  on(void);
static int  off(void);
#if !NULLRDC_CONF_802154_AUTOACK_HW
static void radiocore_hard_recovery(void);
#endif
static void rf_generate_random_seed(void);
static uint8_t flag_transmit = 0;
#if NULLRDC_CONF_802154_AUTOACK_HW
static uint8_t ack_status = 0;
#endif
static volatile int radio_is_on = 0;
static volatile int pending_frame = 0;
static volatile int sleep_on = 0;

/*---------------------------------------------------------------------------*/
int RF231_init(void);
int RF231_prepare(const void *payload, unsigned short payload_len);
int RF231_transmit(unsigned short payload_len);
int RF231_send(const void *data, unsigned short len);
int RF231_read(void *buf, unsigned short bufsize);
int RF231_channel_clear(void);
int RF231_receiving_packet(void);
int RF231_pending_packet(void);
int RF231_on(void);
int RF231_off(void);
int RF231_sleep(void);

/*---------------------------------------------------------------------------*/
static radio_result_t
get_value(radio_param_t param, radio_value_t *value)
{
  return RADIO_RESULT_NOT_SUPPORTED;
}
/*---------------------------------------------------------------------------*/
static radio_result_t
set_value(radio_param_t param, radio_value_t value)
{
  return RADIO_RESULT_NOT_SUPPORTED;
}
/*---------------------------------------------------------------------------*/
static radio_result_t
get_object(radio_param_t param, void *dest, size_t size)
{
  return RADIO_RESULT_NOT_SUPPORTED;
}
/*---------------------------------------------------------------------------*/
static radio_result_t
set_object(radio_param_t param, const void *src, size_t size)
{
  return RADIO_RESULT_NOT_SUPPORTED;
}

const struct radio_driver RF231_radio_driver =
{
	RF231_init,
	RF231_prepare,
	RF231_transmit,
	RF231_send,
	RF231_read,
	RF231_channel_clear,
	RF231_receiving_packet,
	RF231_pending_packet,
	RF231_on,
	RF231_off,
	get_value,
	set_value,
	get_object,
	set_object,
	RF231_sleep
};


/*---------------------------------------------------------------------------*/
/* convenience macros */
//#define RF231_STATUS()                    RF231_arch_status()
#define RF231_COMMAND(c)                  trx_reg_write(RF231_REG_TRX_STATE, c)

/* each frame has a footer consisting of LQI, ED, RX_STATUS added by the radio */
#define FOOTER_LEN                        3   /* bytes */
#define MAX_PACKET_LEN                    127 /* bytes, excluding the length (first) byte */

/* when transmitting, time to allow previous transmission to end before drop */
#define PREV_TX_TIMEOUT                   (10 * RTIMER_SECOND/1000)
/*---------------------------------------------------------------------------*/
#define _DEBUG_                 0
#define DEBUG_PRINTDATA       0    /* print frames to/from the radio; requires DEBUG == 1 */
#if _DEBUG_
#define PRINTF(...)       printf(__VA_ARGS__)
#else
#define PRINTF(...)
#endif

uint8_t bat_state_index = 5;
uint8_t bat_states[5] = {
		0b11, 	// 1.85V
		0b111,	// 2.05V
		0b1111,	// 2.45V
		0b10011,// 2.775V
		0b10111 // 3.075V
};

#define BUSYWAIT_UNTIL(cond, max_time)                                  \
  do {                                                                  \
    static rtimer_clock_t t0;                                                  \
    t0 = RTIMER_NOW();                                                  \
    while(!(cond) && RTIMER_CLOCK_LT(RTIMER_NOW(), t0 + (max_time)));   \
  } while(0)
/*---------------------------------------------------------------------------*/
/**
 * \brief      Get radio channel
 * \return     The radio channel
 */
int
rf_get_channel(void)
{
	uint8_t channel;
  channel=trx_reg_read(RF231_REG_PHY_CC_CCA) & PHY_CC_CCA_CHANNEL;
  //printf("RF231 channel%d\n",channel);
  return (int)channel;
}
/*---------------------------------------------------------------------------*/
/**
 * \brief      Set radio channel
 * \param ch   The radio channel
 * \retval -1  Fail: channel number out of bounds
 * \retval 0   Success
 */
int
rf_set_channel(uint8_t ch)
{
  uint8_t temp;
  PRINTF("RF231: setting channel %u\n", ch);
  if(ch > 26 || ch < 11) {
    return -1;
  }

  /* read-modify-write to conserve other settings */
  temp = trx_reg_read(RF231_REG_PHY_CC_CCA);
  temp &=~ PHY_CC_CCA_CHANNEL;
  temp |= ch;
  trx_reg_write(RF231_REG_PHY_CC_CCA, temp);
  return 0;
}

unsigned char rf_radio_Is_Busy(void)
{
    unsigned char state;

    state = RF231_status();
    return (state == STATE_BUSY_RX_AACK ||
            state == STATE_BUSY_TX_ARET ||
            state == STATE_BUSY_TX ||
            state == STATE_BUSY_RX ||
            state == STATE_BUSY_RX_AACK_NOCLK);
}
/*---------------------------------------------------------------------------*/
/**
 * \brief      Get transmission power
 * \return     The transmission power
 */
int
RF231_get_txp(void)
{
  PRINTF("RF231: get txp\n");
  return trx_reg_read(RF231_REG_PHY_TX_PWR_CONF) & PHY_TX_PWR_TXP;
}
/*---------------------------------------------------------------------------*/
/**
 * \brief      Set transmission power
 * \param txp  The transmission power
 * \retval -1  Fail: transmission power out of bounds
 * \retval 0   Success
 */
int
RF231_set_txp(uint8_t txp)
{
  PRINTF("RF231: setting txp %u\n", txp);
  if(txp > TXP_17M) {
    /* undefined */
    return -1;
  }

  trx_reg_write(RF231_REG_PHY_TX_PWR_CONF, txp);
  return 0;
}

unsigned char radioSetCSMASeed(void)
{
	uint8_t seed[2];

	csprng_get(seed,2);

	trx_reg_write(RF231_REG_CSMA_SEED_0,seed[0]);
	trx_bit_write(RF231_REG_CSMA_SEED_1, 0x03, 0, seed[1]);

	return 0;
}
volatile uint16_t batvoltage;
/*---------------------------------------------------------------------------*/
/**
 * \brief      Init the radio
 * \return     Returns success/fail
 * \retval 0   Success
 */
int
RF231_init(void)
{
	PRINTF("RF231: init.\n");
	uint16_t EEPROM_PAN;
	uint8_t EEPROM_channel;

	// reset will put us into TRX_OFF state
	// reset the radio core
	PhyReset();
	trx_spi_init();
	// Clear IRQ status
	trx_reg_read(RF231_REG_IRQ_STATUS);

	trx_reg_write(RF231_REG_TRX_STATE, TRXCMD_TRX_OFF);
	BUSYWAIT_UNTIL(0, 1 * RTIMER_SECOND / 1000);

	batvoltage = RF231_bat_volt();
	if(batvoltage>3075)	{ bat_state_index = 5;}else
	if(batvoltage>2775)	{ bat_state_index = 4;}else
	if(batvoltage>2450)	{ bat_state_index = 3;}else
	if(batvoltage>2050)	{ bat_state_index = 2;}else
	if(batvoltage>1850)	{ bat_state_index = 1;}else
	 					{ bat_state_index = 0;}

	trx_reg_write(RF231_REG_BATMON, bat_states[bat_state_index]);

	trx_irq_init((FUNC_PTR)RF231_interrupt_poll);
	ENABLE_TRX_IRQ();

	get_eeprom(channel,EEPROM_channel);

	/* Configure the radio using the default values except these. */
	trx_reg_write(RF231_REG_TRX_CTRL_0,		RF231_REG_TRX_CTRL_0_CONF); // No clock at pin 17 (CLKM), pin set to logic low
	trx_reg_write(RF231_REG_TRX_CTRL_1,		RF231_REG_TRX_CTRL_1_CONF); // TX auto crc on
	trx_reg_write(RF231_REG_TRX_CTRL_2,		RF231_REG_TRX_CTRL_2_CONF); // RX safe mode on, 250 kbps
	if(EEPROM_channel >= 11 && EEPROM_channel <=26)
		trx_reg_write(RF231_REG_PHY_CC_CCA,		EEPROM_channel); // Channel and CCA mode
	else
		trx_reg_write(RF231_REG_PHY_CC_CCA,		RF231_REG_PHY_CC_CCA_CONF); // Channel and CCA mode
	trx_reg_write(RF231_REG_PHY_TX_PWR, 	RF231_REG_PHY_TX_PWR_CONF); // TX power
	trx_reg_write(RF231_REG_IRQ_MASK,		RF231_REG_IRQ_MASK_CONF);
	// Set ack times shorter
	//trx_bit_write(SR_AACK_ACK_TIME, 1);

	get_eeprom(PANID,EEPROM_PAN);
	if(EEPROM_PAN == 0xffff)
		SetPanId(IEEE802154_CONF_PANID);
	else
		SetPanId(EEPROM_PAN);

	rf_generate_random_seed();

#if HW_CSMA_FRAME_RETRIES
	trx_bit_write(SR_MAX_FRAME_RETRIES, 5);
	trx_bit_write(SR_MAX_CSMA_RETRIES, 4);
	radioSetCSMASeed();
#else  
	trx_bit_write(SR_MAX_FRAME_RETRIES, 0);
	trx_bit_write(SR_MAX_CSMA_RETRIES, 7);
#endif  


	/* start the radio process */
	process_start(&RF231_radio_process, NULL);
	return 0;
}

/*
 * \brief Generates a 16-bit random number used as initial seed for srand()
 *
 */
static void rf_generate_random_seed(void)
{
	uint32_t seed = 0;
	uint8_t cur_random_val = 0;
	uint8_t i;

	/*
	 * We need to disable TRX IRQs while generating random values in RX_ON,
	 * we do not want to receive frames at this point of time at all.
	 */
	ENTER_TRX_REGION();

	do
	{
		trx_reg_write(RF231_REG_TRX_STATE, TRXCMD_TRX_OFF);
		
	} while (TRXCMD_TRX_OFF != RF231_status());

	do
	{
		/* Ensure that PLL has locked and receive mode is reached. */
		trx_reg_write(RF231_REG_TRX_STATE, TRXCMD_PLL_ON);
		
	} while (TRXCMD_PLL_ON != RF231_status());
	do
	{
		trx_reg_write(RF231_REG_TRX_STATE, TRXCMD_RX_ON);
		
	} while (TRXCMD_RX_ON != RF231_status());

	/* Ensure that register bit RX_PDT_DIS is set to 0. */
	trx_bit_write(SR_RX_PDT_DIS, RX_ENABLE);

	/*
	 * The 32-bit random value is generated from various 2-bit random
	 * values.
	 */
	for (i = 0; i < 16; i++) {
		/* Now we can safely read the 2-bit random number. */
		cur_random_val = trx_bit_read(SR_RND_VALUE);
		seed = seed << 2;
		seed |= cur_random_val;
		BUSYWAIT_UNTIL(0, 1 * RTIMER_SECOND / 1000);
		//delay_us(1); /* wait that the random value gets updated */
	}

	do
	{
		/* Ensure that PLL has locked and receive mode is reached. */
		trx_reg_write(RF231_REG_TRX_STATE, TRXCMD_TRX_OFF);
	} while (TRXCMD_TRX_OFF != RF231_status());
	/*
	 * Now we need to clear potential pending TRX IRQs and
	 * enable the TRX IRQs again.
	 */
	trx_reg_read(RF231_REG_IRQ_STATUS);
	trx_irq_flag_clr();
	LEAVE_TRX_REGION();

	/* Set the seed for the random number generator. */
	srand(seed);
}

/*---------------------------------------------------------------------------*/
/**
 * \brief      prepare a frame and the radio for immediate transmission 
 * \param payload         Pointer to data to copy/send
 * \param payload_len     length of data to copy
 * \return     Returns success/fail, refer to radio.h for explanation
 */
int
RF231_prepare(const void *payload, unsigned short payload_len)
{
#if DEBUG_PRINTDATA
  int i;
#endif  /* DEBUG_PRINTDATA */
  uint8_t templen;
  volatile uint8_t radio_status;
  uint8_t data[130];

#if USE_HW_FCS_CHECK
  /* Add length of the FCS (2 bytes) */
  templen = payload_len + 2;
#else   /* USE_HW_FCS_CHECK */
  /* FCS is assumed to already be included in the payload */
  templen = payload_len;
#endif  /* USE_HW_FCS_CHECK */
 
data[0] = templen;
memcpy(&data[1],payload,templen);

#if DEBUG_PRINTDATA
  PRINTF("RF231 prepare (%u/%u): 0x", payload_len, templen);
  for(i = 0; i < templen; i++) {
    PRINTF("%02x", *(uint8_t *)(payload + i));
  }
  PRINTF("\n");
#endif  /* DEBUG_PRINTDATA */
   
  PRINTF("RF231: prepare %u\n", payload_len);
  if(payload_len > MAX_PACKET_LEN) {
    PRINTF("RF231: error, frame too large to tx\n");
    return RADIO_TX_ERR;
  }

  BUSYWAIT_UNTIL(!rf_radio_Is_Busy(), 10 * RTIMER_SECOND/1000);
	trx_reg_write(RF231_REG_TRX_STATE, STATE_PLL_ON);
  /* check that the FIFO is clear to access */
  radio_status=RF231_status();
  #if NULLRDC_CONF_802154_AUTOACK_HW
  if(radio_status == STATE_BUSY_RX_AACK || radio_status == STATE_BUSY_TX_ARET) {
	  PRINTF("RF231: TRX buffer unavailable: prep when %s\n", radio_status == STATE_BUSY_RX_AACK ? "rx" : "tx");
  #else
   if(radio_status == STATE_BUSY_RX || radio_status == STATE_BUSY_TX) {
	   PRINTF("RF231: TRX buffer unavailable: prep when %s\n", radio_status == STATE_BUSY_RX? "rx" : "tx");
  #endif
    
    return RADIO_TX_ERR;
  }

  /* Write packet to TX FIFO. */
  PRINTF("RF231 len = %u\n", payload_len);
  trx_frame_write((uint8_t *)data, templen+1);
  return RADIO_TX_OK;
}
/*---------------------------------------------------------------------------*/
/**
 * \brief      Transmit a frame already put in the radio with 'prepare'
 * \param payload_len    Length of the frame to send
 * \return     Returns success/fail, refer to radio.h for explanation
 */
int
RF231_transmit(unsigned short payload_len)
{
  static uint8_t status_now;
  PRINTF("RF231: tx %u\n", payload_len);

  /* prepare for TX */
  
  status_now = RF231_status();
   //status_now = trx_reg_read(RF231_REG_TRX_RPC);
  #if NULLRDC_CONF_802154_AUTOACK_HW
  if(status_now == STATE_BUSY_RX_AACK || status_now == STATE_BUSY_TX_ARET) {
  #else
  if(status_now == STATE_BUSY_RX || status_now == STATE_BUSY_TX) {
  #endif
    PRINTF("RF231: collision, was receiving 0x%02X\n",status_now);
    /* NOTE: to avoid loops */
    return RADIO_TX_ERR;;
    // return RADIO_TX_COLLISION;
  }
  if(status_now != STATE_PLL_ON) {
    /* prepare for going to state TX, should take max 80 us */
    //RF231_COMMAND(TRXCMD_PLL_ON);
	trx_reg_write(RF231_REG_TRX_STATE, STATE_PLL_ON);
   // BUSYWAIT_UNTIL(trx_reg_read(RF231_REG_TRX_STATUS) == STATE_PLL_ON, 1 * RTIMER_SECOND/1000);
   //delay_ms(10);
   //status_now = trx_reg_read(RF231_REG_TRX_STATE);
   do 
   {
	   status_now = RF231_status();
   } while (status_now == STATE_TRANSITION);
  }

  if(RF231_status() != STATE_PLL_ON) {
    /* failed moving into PLL_ON state, gracefully try to recover */
    PRINTF("RF231: failed going to PLLON\n");
    RF231_COMMAND(TRXCMD_PLL_ON);   /* try again */
	static uint8_t state;
	state = RF231_status();
    if(state != STATE_PLL_ON) {
      /* give up and signal big fail (should perhaps reset radio core instead?) */
      PRINTF("RF231: graceful recovery (in tx) failed, giving up. State: 0x%02X\n", RF231_status());
      return RADIO_TX_ERR;
    }
  }

  /* perform transmission */
  ENERGEST_OFF(ENERGEST_TYPE_LISTEN);
  ENERGEST_ON(ENERGEST_TYPE_TRANSMIT);
#if NULLRDC_CONF_802154_AUTOACK_HW
  RF231_COMMAND(TRXCMD_TX_ARET_ON);
#endif
  RF231_COMMAND(TRXCMD_TX_START);
   flag_transmit=1;
   //delay_ms(5);
  //printf("RTIMER value %d",RTIMER_NOW());

#if !NULLRDC_CONF_802154_AUTOACK_HW
    BUSYWAIT_UNTIL(RF231_status() == STATE_BUSY_TX, RTIMER_SECOND/2000);
   // printf("RTIMER value1 %d",RTIMER_NOW());
   // printf("\r\nSTATE_BUSY_TX");
  BUSYWAIT_UNTIL(RF231_status() != STATE_BUSY_TX, 10 * RTIMER_SECOND/1000);
  // printf("RTIMER value2 %d",RTIMER_NOW());
#endif

  ENERGEST_OFF(ENERGEST_TYPE_TRANSMIT);
  ENERGEST_ON(ENERGEST_TYPE_LISTEN);

#if !NULLRDC_CONF_802154_AUTOACK_HW
   if(RF231_status() != STATE_PLL_ON) {
    // something has failed 
    PRINTF("RF231: radio fatal err after tx\n");
    radiocore_hard_recovery();
    return RADIO_TX_ERR;
  }
  RF231_COMMAND(TRXCMD_RX_ON);
#else
	BUSYWAIT_UNTIL(ack_status == 1, 10 * RTIMER_SECOND/1000);
	if((ack_status))
	{
	//	printf("\r\nrf231 sent\r\n ");
		ack_status=0;
		PRINTF("\nACK received\n");
		return RADIO_TX_OK;
	}
	else
	{
		PRINTF("\nNOACK received\n");
		return RADIO_TX_NOACK;
	}
	
#endif

  PRINTF("RF231: tx ok\n");
  return RADIO_TX_OK;
}


  int hal_frame_write(uint8_t *write_buffer, uint8_t length);
/*---------------------------------------------------------------------------*/
/**
 * \brief      Send data: first prepares, then transmits
 * \param payload         Pointer to data to copy/send
 * \param payload_len     length of data to copy
 * \return     Returns success/fail, refer to radio.h for explanation
 */
int RF231_send(const void *payload, unsigned short payload_len)
{
	PRINTF("RF231: send %u\n", payload_len);
	if(RF231_prepare(payload, payload_len) == RADIO_TX_ERR){
		return RADIO_TX_ERR;
	}
	return RF231_transmit(payload_len);
}


/*---------------------------------------------------------------------------*/
/**
 * \brief      read a received frame out of the radio buffer 
 * \param buf         pointer to where to copy received data
 * \param bufsize     Maximum size we can copy into bufsize
 * \return     Returns length of data read (> 0) if successful
 * \retval -1  Failed, was transmitting so FIFO is invalid
 * \retval -2  Failed, rx timed out (stuck in rx?)
 * \retval -3  Failed, too large frame for buffer
 * \retval -4  Failed, CRC/FCS failed (if USE_HW_FCS_CHECK is true)
 */
int
RF231_read(void *buf, unsigned short bufsize)
{
//  uint8_t radio_state;
  uint8_t ed;       /* frame metadata */
  uint8_t frame_len = 0;
  uint8_t len = 0;
  int rssi;
#if DEBUG_PRINTDATA
  uint8_t tempreadlen;
#endif  /* DEBUG_PRINTDATA */

  if(pending_frame == 0) {
    return 0;
  }
  pending_frame = 0;

  /* get length of data in FIFO */
  trx_frame_read(&frame_len, 1);
#if DEBUG_PRINTDATA
  tempreadlen = frame_len;
#endif  /* DEBUG_PRINTDATA */
  if(frame_len == 1) {
    frame_len = 0;
  }

  len = frame_len;
#if USE_HW_FCS_CHECK
  /* FCS has already been stripped */
  len = frame_len - 2;
#endif  /* USE_HW_FCS_CHECK */

  if(frame_len == 0) {
    return 0;
  }
  if(len > bufsize) {
    /* too large frame for the buffer, drop */
    PRINTF("RF231: too large frame for buffer, dropping (%u > %u).\n", frame_len, bufsize);
    return -3;
  }
  PRINTF("RF231 read %u B\n", frame_len);

  /* read out the data into the buffer, disregarding the length and metadata bytes */
  // Note RF231: No length so data is at addr 0
  trx_sram_read(0,(uint8_t *)buf, len);
#if DEBUG_PRINTDATA
  {
    int k;
    PRINTF("RF231: Read frame (%u/%u): ", tempreadlen, frame_len);
    for(k = 0; k < frame_len; k++) {
      PRINTF("%02x", *((uint8_t *)buf + k));
    }
    PRINTF("\n");
  }
#endif  /* DEBUG_PRINTDATA */

  /* 
   * Energy level during reception, ranges from 0x00 to 0x53 (=83d) with a
   * resolution of 1dB and accuracy of +/- 5dB. 0xFF means invalid measurement.
   * 0x00 means <= RSSI(base_val), which is -91dBm (typ). See datasheet 12.7.
   * Ergo, real RSSI is (ed-91) dBm or less.
   */
  #define RSSI_OFFSET       (91)
  ed = trx_reg_read(RF231_REG_PHY_ED_LEVEL);
  rssi = (int) ed - RSSI_OFFSET;
  packetbuf_set_attr(PACKETBUF_ATTR_RSSI, rssi);

/*
#if USE_HW_FCS_CHECK
  {
    uint8_t crc_ok;   / * frame metadata * /
    crc_ok = RF231_arch_read_reg(RF231_REG_PHY_RSSI) & PHY_RSSI_CRC_VALID;
    if(crc_ok == 0) {
      / * CRC/FCS fail, drop * /
      PRINTF("RF231: CRC/FCS fail, dropping.\n");
      return -4;
    }
  }
#endif  / * USE_HW_FCS_CHECK * /*/

  return len;
}
/*---------------------------------------------------------------------------*/
/**
 * \brief      perform a clear channel assessment 
 * \retval >0  Channel is clear
 * \retval 0   Channel is not clear
 */
int
RF231_channel_clear(void)
{
  uint8_t regsave;
  int was_off = 0;
  
  if(RF231_status() != STATE_RX_ON) {
    /* CCA can only be performed in RX state */
    was_off = 1;
    RF231_COMMAND(TRXCMD_RX_ON);
  }
  BUSYWAIT_UNTIL(0, 1 * RTIMER_SECOND / 1000);
   //delay_us(200);
  /* request a CCA, storing the channel number (set with the same reg) */
  regsave = trx_reg_read(RF231_REG_PHY_CC_CCA);
  regsave |= PHY_CC_CCA_DO_CCA | PHY_CC_CCA_MODE_CS_OR_ED;
  trx_reg_write(RF231_REG_PHY_CC_CCA, regsave);
  
  BUSYWAIT_UNTIL(trx_reg_read(RF231_REG_TRX_STATUS) & TRX_CCA_DONE,
      RTIMER_SECOND / 1000);
  //regsave = RF231_status();
  regsave = trx_reg_read(RF231_REG_TRX_STATUS);
  /* return to previous state */
  if(was_off) {
    RF231_COMMAND(TRXCMD_TRX_OFF);
  }
  #if NULLRDC_CONF_802154_AUTOACK_HW 
  else{
	  RF231_COMMAND(TRXCMD_RX_AACK_ON);
  }
  #endif

  /* check CCA */
  if((regsave & TRX_CCA_DONE) && (regsave & TRX_CCA_STATUS)) {
    PRINTF("RF231: CCA 1\n");
    return 1;
  }
  PRINTF("RF231: CCA 0\n");
  return 0;
}
/*---------------------------------------------------------------------------*/
/**
 * \brief      check whether we are currently receiving a frame 
 * \retval >0  we are currently receiving a frame 
 * \retval 0   we are not currently receiving a frame 
 */
int
RF231_receiving_packet(void)
{ 
  uint8_t trx_state;
  trx_state=RF231_status();
  #if NULLRDC_CONF_802154_AUTOACK_HW
  if(trx_state == STATE_BUSY_RX_AACK) {
  #else 
  if(trx_state == STATE_BUSY_RX) {
  #endif
  
    PRINTF("RF231: Receiving frame\n");
    return 1;
  }
  PRINTF("RF231: not Receiving frame\n");
  return 0;
}
/*---------------------------------------------------------------------------*/
/**
 * \brief      check whether we have a frame awaiting processing 
 * \retval >0  we have a frame awaiting processing 
 * \retval 0   we have not a frame awaiting processing 
 */
int
RF231_pending_packet(void)
{
  PRINTF("RF231: Frame %spending\n", pending_frame ? "" : "not ");
  return pending_frame;
}
/*---------------------------------------------------------------------------*/
/**
 * \brief      switch the radio on to listen (rx) mode 
 * \retval 0   Success
 */
int
RF231_on(void)
{
  PRINTF("RF231: on\n");

  return on();
}
/*---------------------------------------------------------------------------*/
/**
 * \brief      switch the radio off 
 * \retval 0   Success
 */
int
RF231_off(void)
{
  PRINTF("RF231: off\n");
  return off();
}
void SetIEEEAddr(uint8_t *ieee_addr)
{
	uint8_t *ptr_to_reg = ieee_addr;

	trx_reg_write((RF231_REG_IEEE_ADDR_7), *ptr_to_reg++);
	trx_reg_write((RF231_REG_IEEE_ADDR_6), *ptr_to_reg++);
	trx_reg_write((RF231_REG_IEEE_ADDR_5), *ptr_to_reg++);
	trx_reg_write((RF231_REG_IEEE_ADDR_4), *ptr_to_reg++);
	trx_reg_write((RF231_REG_IEEE_ADDR_3), *ptr_to_reg++);
	trx_reg_write((RF231_REG_IEEE_ADDR_2), *ptr_to_reg++);
	trx_reg_write((RF231_REG_IEEE_ADDR_1), *ptr_to_reg++);
	trx_reg_write((RF231_REG_IEEE_ADDR_0), *ptr_to_reg);
}

void SetPanId(uint16_t panId)
{
	uint8_t *d = (uint8_t *)&panId;

	trx_reg_write(0x22, d[0]);
	trx_reg_write(0x23, d[1]);
}

void SetShortAddr(uint16_t addr)
{
	uint8_t *d = (uint8_t *)&addr;

	trx_reg_write(0x20, d[0]);
	trx_reg_write(0x21, d[1]);
	trx_reg_write(0x2d, d[0] + d[1]);
}

/*---------------------------------------------------------------------------*/
/* switch the radio on */
int
on(void)
{
  /* Check whether radio is in sleep */
  if(sleep_on)
  {
     /* Wake the radio. It'll move to TRX_OFF state */
	
  	 wake_from_sleep();
  	BUSYWAIT_UNTIL(0, (1 * RTIMER_SECOND / 1000)+1);
	 //delay_ms(1);
	 //printf("\r\nWake from sleep %d",RF231_get_channel());
	 sleep_on = 0;
  }
  uint8_t state_now = RF231_status();
  if(state_now != STATE_PLL_ON && state_now != STATE_TRX_OFF 
#if NULLRDC_CONF_802154_AUTOACK_HW
  && state_now != STATE_TX_ARET_ON
#endif
  ) {
    /* fail, we need the radio transceiver to be in either of those states */
    return -1;
  }

  /* go to RX_ON state */
  ENERGEST_ON(ENERGEST_TYPE_LISTEN);
  #if NULLRDC_CONF_802154_AUTOACK_HW
  RF231_COMMAND(TRXCMD_RX_AACK_ON);
  #else
  RF231_COMMAND(TRXCMD_RX_ON);
  #endif
  radio_is_on = 1;
  return 0;
}
/*---------------------------------------------------------------------------*/
/* switch the radio off */
int
off(void)
{ 
	volatile int status = RF231_status();

	if(status == STATE_TRX_OFF)
		return 0;

	/* turn off the radio transceiver */
	ENERGEST_OFF(ENERGEST_TYPE_LISTEN);
	RF231_COMMAND(TRXCMD_FORCE_TRX_OFF);
	do
	{
		status = RF231_status();
	} while (status == STATE_TRANSITION);
	if(status == STATE_TRX_OFF){
		radio_is_on = 0;
		return 0;
	}
	else
		return -1;
}
/*---------------------------------------------------------------------------*/
/* Put the Radio in sleep mode */

int 
RF231_sleep(void)
{
	int status = -1;
	/* Check whether we're already sleeping */
	if (!sleep_on) {
		/* Turn off the Radio */
		status = RF231_off();
		/* Set the SLP_PIN to high */
		if(status == 0) {
			sleep_on = 1;
			goto_sleep();
		}
	}
	
	return status;
}
/*---------------------------------------------------------------------------*/
/* used for indicating that the interrupt wasn't able to read SPI and must be serviced */
static volatile int interrupt_callback_wants_poll = 0;
/* used as a blocking semaphore to indicate that we are currently servicing an interrupt */
static volatile int interrupt_callback_in_progress = 0;

/**
 * \brief      Radio RF231 process, infinitely awaits a poll, then checks radio
 *             state and handles received data.
 */
PROCESS_THREAD(RF231_radio_process, ev, data)
{
  int len;
  PROCESS_BEGIN();
  PRINTF("RF231: started.\n");

  while(1) {
    PROCESS_YIELD_UNTIL(ev == PROCESS_EVENT_POLL);
    PRINTF("RF231: polled.\n");

    if(interrupt_callback_wants_poll) {
      RF231_interrupt_poll();
    }

    packetbuf_clear();
    // packetbuf_set_attr(PACKETBUF_ATTR_TIMESTAMP, last_packet_timestamp);
    len = RF231_read(packetbuf_dataptr(), PACKETBUF_SIZE);
    if(len > 0) {
      packetbuf_set_datalen(len);
      NETSTACK_RDC.input();
    } else {
      PRINTF("RF231: error while reading: %d\n", len);
    }
  }
  PROCESS_END();
}


/*---------------------------------------------------------------------------*/
/**
 * \brief      RF231 radio process poll function, to be called from the
 *             interrupt handler to poll the radio process
 * \retval 0   success
 */
int
RF231_interrupt_poll(void)
{  
	volatile uint8_t irq_source;
	 /* handle IRQ source (for what IRQs are enabled, see RF231-config.h) */
	 irq_source = trx_reg_read(RF231_REG_IRQ_STATUS);
	 if(irq_source & IRQ_TRX_DONE) {
		 
		 if(flag_transmit==1)
		 {
			 flag_transmit=0;
			 interrupt_callback_in_progress = 0;
			 #if NULLRDC_CONF_802154_AUTOACK_HW
			//printf("Status %x",trx_reg_read(RF231_REG_TRX_STATE) & TRX_STATE_TRAC_STATUS);
			if(!(trx_reg_read(RF231_REG_TRX_STATE) & TRX_STATE_TRAC_STATUS))
			ack_status = 1;
			 RF231_COMMAND(TRXCMD_RX_AACK_ON);
			 #endif
			 return 0;
		 }
  
  if( interrupt_callback_in_progress) {
    /* we cannot read out info from radio now, return here later (through a poll) */
    interrupt_callback_wants_poll = 1;
    process_poll(&RF231_radio_process);
    PRINTF("RF231: irq but busy, returns later.\n");
    return 0;
  }

  interrupt_callback_wants_poll = 0;
  interrupt_callback_in_progress = 1;

 
    /* we have started receiving a frame, len can be read */
    pending_frame = 1;
	//delay_cycles_ms(1);
    process_poll(&RF231_radio_process);
  }

	 if(irq_source & IRQ_BAT_LOW) {
	     /* Battery low */
		 bat_state_index--;
		 if(bat_state_index)
			 trx_reg_write(RF231_REG_BATMON, bat_states[bat_state_index]);
		 else
			 trx_reg_write(RF231_REG_IRQ_MASK,RF231_REG_IRQ_MASK_NO_BAT_CONF);
	   }

#if 0
  /* Note, these are not currently in use but here for completeness. */
  if(irq_source & IRQ_TRX_DONE) {
    /* End of transmitted or received frame.  */
  }
  if(irq_source & IRQ_TRXBUF_ACCESS_VIOLATION) {
    /* 
     * Access violation on the shared TX/RX FIFO. Possible causes:
     *  - buffer underrun while transmitting, ie not enough data in FIFO to tx
     *  - reading too fast from FIFO during rx, ie not enough data received yet
     *  - haven't read last rxed frame when next rx starts, but first is possible
     *    to read out, with possible corruption - check FCS
     *  - writing frames larger than 127 B to FIFO (len byte)
     */
    PRINTF("RF231-arch: access violation.\n");
  }
  if(irq_source & IRQ_BAT_LOW) {
    /* Battery low */
  }
  if(irq_source & IRQ_RX_ADDRESS_MATCH) {
    /* receiving frame address match */
  }
  if(irq_source & IRQ_CCA_ED_DONE) {
    /* CCA/ED done */
  }
  if(irq_source & IRQ_PLL_UNLOCK) {
    /* PLL unlock */
  }
  if(irq_source & IRQ_PLL_LOCK) {
    /* PLL lock */
  }
#endif

  interrupt_callback_in_progress = 0;
  return 0;
}

#if !NULLRDC_CONF_802154_AUTOACK_HW
/*---------------------------------------------------------------------------*/
/* 
 * Hard, brute reset of radio core and re-init due to it being in unknown,
 * unexpected, or locked state from which we cannot recover in the usual places.
 * Does a full reset and re-init.
 */
static void
radiocore_hard_recovery(void)
{
  RF231_init();
}
#endif


void
goto_sleep(void)
{
	TRX_SLP_TR_HIGH();
}
void
wake_from_sleep(void)
{
	TRX_SLP_TR_LOW();
}

uint8_t RF231_status(void)
{
	return (trx_reg_read(RF231_REG_TRX_STATUS) & TRX_STATUS);
}
/*---------------------------------------------------------------------------*/
// Don't call from Pon and sleep mode
/*
 * This function uses the comparator inside the chip.
 * EKS: voltage is 3.3V
 * we start by asking if the voltage is above or below 2.45V
 * The voltage is above so we set BATMON_HR = 1 and BATMON_VTH=0b0111 (3.075V)
 * We then ask if the voltage is above or below 3.075V
 * The voltage is above so we set BATMON_HR = 1 and BATMON_VTH=0b1011 (3.375V)
 * We then ask if the voltage is above or below 3.375V
 * The voltage is now below so we keep the bit clear and setBATMON_HR = 1 and BATMON_VTH=0b1001 (3.225V)
 * We then ask if the voltage is above or below 3.225V
 * The voltage is above so we set BATMON_HR = 1 and BATMON_VTH=0b1010 (3.300V)
 *
 * The last bit is kept clear to floor the result and be pessimistic.
 */
uint16_t RF231_bat_volt(void)
{
	volatile uint8_t reg;
	uint8_t i;

	reg = 0xf; //BATMON_VTH = 2.45

	for (i = 4; i > 0 ; i--){
		trx_reg_write(RF231_REG_BATMON, reg);
		reg = trx_reg_read(RF231_REG_BATMON);
		if(reg & (1 << 5)) //The battery voltage is above the threshold
			reg ^= (3 << (i - 1)); // set the bit again and clear the next
		else//The battery voltage is below the threshold.
			reg &= ~(1 << (i - 1)); // clear the next
	}

	if(reg & (1 << 4)) //BATMON_HR = 1
		return (reg & 0xf) * 75 + 2550;
	else
		return (reg & 0xf) * 50 + 1700;
}

// More efficient then ask for exact voltage
uint8_t RF231_bat_status(void)
{
	return bat_state_index;
}
