#include <stdio.h>
#include "net/ip/uip.h"
#include "ip64.h"
#include "ip64-eth.h"
#include "same70.h"
#include "gmac.h"
#include "gmac_raw_2.h"
#include "drivers/sysclk.h"
#include "ethernet_phy.h"

#include "platform-conf.h"

PROCESS(ksz8863_process, "KSZ8863 IP64 driver");

void rx_input(uint32_t ul_status);

/** The GMAC driver instance */
static gmac_device_t gs_gmac_dev;

void GMAC_Handler(void)
{
	gmac_handler(&gs_gmac_dev, GMAC_QUE_0);
}

uint8_t gs_uc_mac_address[8];


static void
init(void)
{

	volatile uint32_t ul_delay;
	gmac_options_t gmac_option;

	get_eeprom(eepromMacAddress[0], gs_uc_mac_address[0]);
	get_eeprom(eepromMacAddress[1], gs_uc_mac_address[1]);
	get_eeprom(eepromMacAddress[2], gs_uc_mac_address[2]);
	get_eeprom(eepromMacAddress[3], gs_uc_mac_address[3]);
	get_eeprom(eepromMacAddress[4], gs_uc_mac_address[4]);
	get_eeprom(eepromMacAddress[5], gs_uc_mac_address[5]);

	/* Display MAC & IP settings */
	printf("-- MAC %x:%x:%x:%x:%x:%x\n",
			gs_uc_mac_address[0], gs_uc_mac_address[1], gs_uc_mac_address[2],
			gs_uc_mac_address[3], gs_uc_mac_address[4], gs_uc_mac_address[5]);



	/* Wait for PHY to be ready (CAT811: Max400ms) */
	ul_delay = sysclk_get_cpu_hz() / 1000 / 3 * 400;
	while (ul_delay--);

	/* Enable GMAC clock */
	pmc_enable_periph_clk(ID_GMAC);

	/* Fill in GMAC options */
	gmac_option.uc_copy_all_frame = 0;
	gmac_option.uc_no_boardcast = 0;

	memcpy(gmac_option.uc_mac_addr, gs_uc_mac_address, sizeof(gs_uc_mac_address));
	memcpy(ip64_eth_addr.addr, gs_uc_mac_address, sizeof(gs_uc_mac_address));
	gs_gmac_dev.p_hw = GMAC;

	/* Init GMAC driver structure */
	gmac_dev_init(GMAC, &gs_gmac_dev, &gmac_option);

	/* Enable Interrupt */
	NVIC_EnableIRQ(GMAC_IRQn);

	gmac_dev_set_rx_callback(&gs_gmac_dev, GMAC_QUE_0,rx_input);

	/* Init MAC PHY driver */
	if (ethernet_phy_init(GMAC, 0, sysclk_get_cpu_hz())
					!= GMAC_OK) {
		printf("PHY Initialize ERROR!\n");
		return;
	}

	process_start(&ksz8863_process, NULL);

}

void rx_input(uint32_t ul_status)
{
	process_poll(&ksz8863_process);
}

PROCESS_THREAD(ksz8863_process, ev, data)
{
  uint32_t ul_frm_size;
  PROCESS_BEGIN();

  /* Auto Negotiate, work in RMII mode */
  	if (ethernet_phy_auto_negotiate1(GMAC, 1) != GMAC_OK) {
  		printf("Auto Negotiate ERROR!\r");
  	}

  	if(ethernet_phy_auto_negotiate2(GMAC, 1) != GMAC_OK){
  		PROCESS_YIELD();
  	}
  	ethernet_phy_auto_negotiate3(GMAC, 1);
  /* Establish ethernet link */
  	if(ethernet_phy_set_link(GMAC, 1, 1) != GMAC_OK) {
  		printf("Set link ERROR!\n");
  	}

  	printf("-- Link detected. \n");

  while(1) {
	  PROCESS_YIELD_UNTIL(ev == PROCESS_EVENT_POLL);

    gmac_dev_read(&gs_gmac_dev, GMAC_QUE_0, (uint8_t *) ip64_packet_buffer,
			ip64_packet_buffer_maxlen, &ul_frm_size);

	if (ul_frm_size > 0) {
		/* Handle input frame */
		IP64_INPUT(ip64_packet_buffer, ul_frm_size);
		process_poll(&ksz8863_process);
	}
  }

  PROCESS_END();
}

/*---------------------------------------------------------------------------*/

static int
output(uint8_t *packet, uint16_t len)
{
	gmac_dev_write(&gs_gmac_dev, GMAC_QUE_0, packet, len, NULL);
  return len;
}


/*---------------------------------------------------------------------------*/
const struct ip64_driver ksz8863_ip64_driver = {
  init,
  output
};
/*---------------------------------------------------------------------------*/
