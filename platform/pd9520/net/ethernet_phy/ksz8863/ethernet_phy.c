 /**
 * \file
 *
 * \brief API driver for KSZ8061RNB PHY component.
 *
 * Copyright (c) 2015-2016 Atmel Corporation. All rights reserved.
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
#include <stdio.h>
#include "same70.h"
#include "drivers/spi_master.h"
#include "ethernet_phy.h"
#include "gpio.h"
#include "gmac.h"
//#include "conf_eth.h"
#include "same70.h"
#define ETH_PHY_MODE                                  GMAC_PHY_RMII
/// @cond 0
/**INDENT-OFF**/
#ifdef __cplusplus
extern "C" {
#endif

#define PIN_GMAC_RESET_MASK   PIO_PD11
#define PIN_GMAC_RESET_PIO    PIOD
#define PIN_GMAC_INT_MASK     PIO_PA18
#define PIN_GMAC_SIGDET_PIO   PIOA
#define PIN_GMAC_SIGDET_MASK  PIO_PA29
#define PIN_GMAC_INT_PIO      PIOA
#define PIN_GMAC_PERIPH       PIO_PERIPH_A
#define PIN_GMAC_PIO          PIOD
#define PIN_GMAC_MASK         (PIO_PD0A_GTXCK | PIO_PD1A_GTXEN | PIO_PD2A_GTX0 | \
						       PIO_PD3A_GTX1 | PIO_PD4A_GRXDV | PIO_PD5A_GRX0 |  \
						       PIO_PD6A_GRX1 | PIO_PD7A_GRXER )

//static const Pin pinsTX_ENABLE[] = {{ PIO_PD10, PIOD, ID_PIOD, PIO_OUTPUT_1, PIO_DEFAULT}}; //Transmit enable, pin low => enable
/**INDENT-ON**/
/// @endcond

/**
 * \defgroup ksz8061rnbva_ethernet_phy_group PHY component (KSZ8061RNB)
 *
 * Driver for the ksz8061rnb component. This driver provides access to the main
 * features of the PHY.
 *
 * \section dependencies Dependencies
 * This driver depends on the following modules:
 * - \ref gmac_group Ethernet Media Access Controller (GMAC) module.
 *
 * @{
 */

/* Max PHY number */
#define ETH_PHY_MAX_ADDR   31

/* Ethernet PHY operation max retry count */
#define ETH_PHY_RETRY_MAX 1000000

/* Ethernet PHY operation timeout */
#define ETH_PHY_TIMEOUT 10

/**
 * \brief Find a valid PHY Address ( from addrStart to 31 ).
 *
 * \param p_gmac   Pointer to the GMAC instance.
 * \param uc_phy_addr PHY address.
 * \param uc_start_addr Start address of the PHY to be searched.
 *
 * \return 0xFF when no valid PHY address is found.
 */
static uint8_t ethernet_phy_find_valid(Gmac *p_gmac, uint8_t uc_phy_addr,
		uint8_t uc_start_addr)
{
	uint32_t ul_value = 0;
	uint8_t uc_rc = 0;
	uint8_t uc_cnt;
	uint8_t uc_phy_address = uc_phy_addr;

    uc_rc = uc_phy_address;
	/* Check the current PHY address */
	gmac_phy_read(p_gmac, uc_phy_addr, MII_PHYID1, &ul_value);

	/* Find another one */
	if (ul_value != MII_OUI_MSB) {
		uc_rc = 0xFF;
		for (uc_cnt = uc_start_addr; uc_cnt <= ETH_PHY_MAX_ADDR; uc_cnt++) {
			uc_phy_address = (uc_phy_address + 1) & 0x1F;
			gmac_phy_read(p_gmac, uc_phy_address, MII_PHYID1, &ul_value);
			if (ul_value == MII_OUI_MSB) {
				uc_rc = uc_phy_address;
				//printf("PHY ADDR: 0x%x\n",uc_phy_address);
				break;
			}
		}
	}


	if (uc_rc != 0xFF) {
		gmac_phy_read(p_gmac, uc_phy_address, MII_BMSR, &ul_value);
	}
	return uc_rc;
}

extern void spi3_init(void);
/**
 * \brief Perform a HW initialization to the PHY and set up clocks.
 *
 * This should be called only once to initialize the PHY pre-settings.
 * The PHY address is the reset status of CRS, RXD[3:0] (the emacPins' pullups).
 * The COL pin is used to select MII mode on reset (pulled up for Reduced MII).
 * The RXDV pin is used to select test mode on reset (pulled up for test mode).
 * The above pins should be predefined for corresponding settings in resetPins.
 * The GMAC peripheral pins are configured after the reset is done.
 *
 * \param p_gmac   Pointer to the GMAC instance.
 * \param uc_phy_addr PHY address.
 * \param ul_mck GMAC MCK.
 *
 * Return GMAC_OK if successfully, GMAC_TIMEOUT if timeout.
 */

uint8_t ethernet_phy_init(Gmac *p_gmac, uint8_t uc_phy_addr, uint32_t mck)
{
	uint8_t uc_rc;
	uint8_t uc_phy;

	pio_set_output(PIN_GMAC_RESET_PIO, PIN_GMAC_RESET_MASK, 1,  false, true);
	pio_set_input(PIN_GMAC_INT_PIO, PIN_GMAC_INT_MASK, PIO_PULLUP);
	//pio_set_input(PIN_GMAC_SIGDET_PIO, PIN_GMAC_SIGDET_MASK, PIO_DEFAULT);
	pio_set_peripheral(PIN_GMAC_PIO, PIN_GMAC_PERIPH, PIN_GMAC_MASK);
	pio_set_output(PIOD, PIO_PD10, 0,  false, false);
	//PIO_Configure(pinsTX_ENABLE, 1);

	spi3_init();
	ethernet_phy_reset(GMAC,uc_phy_addr);

	/* Configure GMAC runtime clock */
	/*uc_rc = gmac_set_mdc_clock(p_gmac, mck);
	if (uc_rc != GMAC_OK) {
		return 0;
	}*/

	/* Check PHY Address */
	uc_phy = ethernet_phy_find_valid(p_gmac, uc_phy_addr, 0);
	if (uc_phy == 0xFF) {
		return 0;
	}
	if (uc_phy != uc_phy_addr) {
		ethernet_phy_reset(p_gmac, uc_phy_addr);
	}


	return uc_rc;
}


/**
 * \brief Get the Link & speed settings, and automatically set up the GMAC with the
 * settings.
 *
 * \param p_gmac   Pointer to the GMAC instance.
 * \param uc_phy_addr PHY address.
 * \param uc_apply_setting_flag Set to 0 to not apply the PHY configurations, else to apply.
 *
 * Return GMAC_OK if successfully, GMAC_TIMEOUT if timeout.
 */
uint8_t ethernet_phy_set_link(Gmac *p_gmac, uint8_t uc_phy_addr,
		uint8_t uc_apply_setting_flag)
{
	uint32_t ul_stat1;
	uint32_t ul_stat2;
	uint8_t uc_phy_address, uc_speed, uc_fd;
	uint8_t uc_rc = GMAC_OK;


	uc_phy_address = uc_phy_addr;
	uc_rc = gmac_phy_read(p_gmac, uc_phy_address, MII_BMSR, &ul_stat1);
	if (uc_rc != GMAC_OK) {
		/* Disable PHY management and start the GMAC transfer */
		printf("gmac_phy_read (%d)ERROR!\n",uc_rc);
		return uc_rc;
	}

	if (uc_apply_setting_flag == 0) {
		/* Disable PHY management and start the GMAC transfer */
		printf("uc_apply_setting_flag!\n");
		return uc_rc;
	}
	/* Read advertisement */
	uc_rc = gmac_phy_read(p_gmac, uc_phy_address, MII_PCR1, &ul_stat2);
	if (uc_rc != GMAC_OK) {
		/* Disable PHY management and start the GMAC transfer */
		printf("gmac_phy_read2 (%d)ERROR!\n",uc_rc);
		return uc_rc;
	}else
	if ((ul_stat1 & MII_100BASE_TX_FD) && (ul_stat2 & MII_OMI_100BASE_TX_FD)) {
		/* Set GMAC for 100BaseTX and Full Duplex */
		printf("100BaseTX and Full Duplex\n");
		uc_speed = true;
		uc_fd = true;
	}else
	if ((ul_stat1 & MII_10BASE_T_FD) && (ul_stat2 & MII_OMI_10BASE_T_FD)) {
		/* Set MII for 10BaseT and Full Duplex */
		printf("10BaseT and Full Duplex\n");
		uc_speed = false;
		uc_fd = true;
	}else
	if ((ul_stat1 & MII_100BASE_TX_HD) && (ul_stat2 & MII_OMI_100BASE_TX_HD)) {
		/* Set MII for 100BaseTX and Half Duplex */
		printf("100BaseTX and Half Duplex\n");
		uc_speed = true;
		uc_fd = false;
	}else
	if ((ul_stat1 & MII_10BASE_T_HD) && (ul_stat2 & MII_OMI_10BASE_T_HD)) {
		/* Set MII for 10BaseT and Half Duplex */
		printf("10BaseT and Half Duplex\n");
		uc_speed = false;
		uc_fd = false;
	}
	gmac_set_speed(p_gmac, uc_speed);
	gmac_enable_full_duplex(p_gmac, uc_fd);
	return GMAC_OK;
}


/**
 * \brief Issue an auto negotiation of the PHY.
 *
 * \param p_gmac   Pointer to the GMAC instance.
 * \param uc_phy_addr PHY address.
 *
 * Return GMAC_OK if successfully, GMAC_TIMEOUT if timeout.
 */
uint8_t ethernet_phy_auto_negotiate1(Gmac *p_gmac, uint8_t uc_phy_addr)
{
	uint32_t ul_value;
	uint32_t ul_phy_anar;


	uint8_t uc_rc;


	/* Set up control register */
	uc_rc = gmac_phy_read(p_gmac, uc_phy_addr, MII_BMCR, &ul_value);
	if (uc_rc != GMAC_OK) {
		return uc_rc;
	}

	ul_value &= ~(uint32_t)MII_AUTONEG; /* Remove auto-negotiation enable */
	ul_value &= ~(uint32_t)(MII_LOOPBACK | MII_POWER_DOWN);
	ul_value |= (uint32_t)MII_ISOLATE; /* Electrically isolate PHY */
	uc_rc = gmac_phy_write(p_gmac, uc_phy_addr, MII_BMCR, ul_value);
	if (uc_rc != GMAC_OK) {
		return uc_rc;
	}

	/* 
	 * Set the Auto_negotiation Advertisement Register.
	 * MII advertising for Next page.
	 * 100BaseTxFD and HD, 10BaseTFD and HD, IEEE 802.3.
	 */
	ul_phy_anar = MII_100TX_FDX | MII_100TX_HDX | MII_10_FDX | MII_10_HDX |
			MII_AN_IEEE_802_3;
	uc_rc = gmac_phy_write(p_gmac, uc_phy_addr, MII_ANAR, ul_phy_anar);
	if (uc_rc != GMAC_OK) {
		return uc_rc;
	}

	/* Read & modify control register */
	uc_rc = gmac_phy_read(p_gmac, uc_phy_addr, MII_BMCR, &ul_value);
	if (uc_rc != GMAC_OK) {
		return uc_rc;
	}

	ul_value |= MII_SPEED_SELECT | MII_AUTONEG | MII_DUPLEX_MODE;
	uc_rc = gmac_phy_write(p_gmac, uc_phy_addr, MII_BMCR, ul_value);
	if (uc_rc != GMAC_OK) {
		return uc_rc;
	}

	/* Restart auto negotiation */
	ul_value |= (uint32_t)MII_RESTART_AUTONEG;
	ul_value &= ~(uint32_t)MII_ISOLATE;
	uc_rc = gmac_phy_write(p_gmac, uc_phy_addr, MII_BMCR, ul_value);
	if (uc_rc != GMAC_OK) {
		return uc_rc;
	}
	return GMAC_OK;
}
uint8_t ethernet_phy_auto_negotiate2(Gmac *p_gmac, uint8_t uc_phy_addr)
{
	uint8_t uc_rc;
	uint32_t ul_value;
	/* Check if auto negotiation is completed */

	uc_rc = gmac_phy_read(p_gmac, uc_phy_addr, MII_BMSR, &ul_value);
	if (uc_rc != GMAC_OK) {
		return uc_rc;
	}
	/* Done successfully */
	if (ul_value & MII_AUTONEG_COMP) {
		return GMAC_OK;
	}

	return GMAC_TIMEOUT;


}
uint8_t ethernet_phy_auto_negotiate3(Gmac *p_gmac, uint8_t uc_phy_addr)
{
	uint8_t uc_rc;
	uint32_t ul_phy_anar;
	uint32_t ul_phy_analpar;
	uint8_t uc_speed = 0;
	uint8_t uc_fd=0;

	ul_phy_anar = MII_100TX_FDX | MII_100TX_HDX | MII_10_FDX | MII_10_HDX |
				MII_AN_IEEE_802_3;

	/* Get the auto negotiate link partner base page */
	uc_rc = gmac_phy_read(p_gmac, uc_phy_addr, MII_ANLPAR, &ul_phy_analpar);
	if (uc_rc != GMAC_OK) {
		return uc_rc;
	}


	/* Set up the GMAC link speed */
	if ((ul_phy_anar & ul_phy_analpar) & MII_100TX_FDX) {
		printf(" Set MII for 100BaseTX and Full Duplex");
		uc_speed = true;
		uc_fd = true;
	} else if ((ul_phy_anar & ul_phy_analpar) & MII_10_FDX) {
		printf(" Set MII for 10BaseT and Full Duplex ");
		uc_speed = false;
		uc_fd = true;
	} else if ((ul_phy_anar & ul_phy_analpar) & MII_100TX_HDX) {
		printf(" Set MII for 100BaseTX and half Duplex ");
		uc_speed = true;
		uc_fd = false;
	} else if ((ul_phy_anar & ul_phy_analpar) & MII_10_HDX) {
		printf(" Set MII for 10BaseT and half Duplex ");
		uc_speed = false;
		uc_fd = false;
	}

	gmac_set_speed(p_gmac, uc_speed);
	gmac_enable_full_duplex(p_gmac, uc_fd);

	/* Select Media Independent Interface type */
	//gmac_select_mii_mode(p_gmac, ETH_PHY_MODE);

	gmac_enable_transmit(GMAC, true);
	gmac_enable_receive(GMAC, true);

	return uc_rc;
}
extern int KSZ8863_reset(void);
/**
 * \brief Issue a SW reset to reset all registers of the PHY.
 *
 * \param p_gmac   Pointer to the GMAC instance.
 * \param uc_phy_addr PHY address.
 *
 * \Return GMAC_OK if successfully, GMAC_TIMEOUT if timeout.
 */
uint8_t ethernet_phy_reset(Gmac *p_gmac, uint8_t uc_phy_addr)
{
	KSZ8863_reset();

	return GMAC_OK;
}

uint32_t ethernet_phy_Basic_Status_Register(Gmac *p_gmac, uint8_t uc_phy_addr)
{
	uint32_t ul_value;

	gmac_phy_read(p_gmac, uc_phy_addr, MII_BMSR, &ul_value);

	return ul_value;
}



/// @cond 0
/**INDENT-OFF**/
#ifdef __cplusplus
}
#endif
/**INDENT-ON**/
/// @endcond

/**
 * \}
 */
