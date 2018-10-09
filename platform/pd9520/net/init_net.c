#include <stdio.h>
#include "net/ip/uip.h"
#include "ip64.h"
#include "ip64-eth.h"
#include "same70.h"
#include "gmac.h"
#include "gmac_raw_2.h"
#include "drivers/sysclk.h"
#include "ethernet_phy.h"

#include <gpio.h>
#include "pio_handler.h"

#include "platform-conf.h"

PROCESS(ksz8863_process, "KSZ8863 IP64 driver");
PROCESS(ksz8863_link_process, "KSZ8863 link");

void rx_input(uint32_t ul_status);
void phy_pin_irq(uint32_t a, uint32_t b);

volatile uint8_t link_status = 0;

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
	//ul_delay = sysclk_get_cpu_hz() / 1000 / 3 * 400;
	//while (ul_delay--);

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
	process_start(&ksz8863_link_process, NULL);
	process_start(&ksz8863_process, NULL);

	pio_set_input(PIOA,PIO_PA18,PIO_PULLUP);
		pio_handler_set(PIOA, ID_PIOA, PIO_PA18, PIO_IT_FALL_EDGE, phy_pin_irq);
		NVIC_SetPriority((IRQn_Type) ID_PIOA, 12);//level 0 is the highest interrupt priority (0-15)
		NVIC_EnableIRQ((IRQn_Type)ID_PIOA);
		pio_enable_interrupt(PIOA, PIO_PA18);

}

void rx_input(uint32_t ul_status)
{
	process_poll(&ksz8863_process);
	//printf("r\n");
}

PROCESS_THREAD(ksz8863_process, ev, data)
{
  uint32_t ul_frm_size;
  PROCESS_BEGIN();
#if 1
  /* Auto Negotiate, work in RMII mode */
  	if (ethernet_phy_auto_negotiate1(GMAC, 1) != GMAC_OK) {
  		printf("Auto Negotiate ERROR!\r");
  	}

  	if(ethernet_phy_auto_negotiate2(GMAC, 1) != GMAC_OK){
  		PROCESS_YIELD();
  	}
  	ethernet_phy_auto_negotiate3(GMAC, 1);
   /*Establish ethernet link */
  	ul_frm_size = ethernet_phy_set_link(GMAC, 1, 1);
  	if(ul_frm_size != GMAC_OK) {
  		printf("Set link ERROR (%d)!\n",ul_frm_size);
  	}

  	printf("-- Link detected. \n");
#endif
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
	if(link_status){
		gmac_dev_write(&gs_gmac_dev, GMAC_QUE_0, packet, len, NULL);
		return len;
	}
	else
		return 0;
}


/*---------------------------------------------------------------------------*/
const struct ip64_driver ksz8863_ip64_driver = {
  init,
  output
};
/*---------------------------------------------------------------------------*/

void phy_pin_irq(uint32_t a, uint32_t b)
{
	process_poll(&ksz8863_link_process);
}
extern  int KSZ8863_ack_interrupt(void);
extern int KSZ8863_config_intr(void);
PROCESS_THREAD(ksz8863_link_process, ev, data)
{
  uint32_t ul_value;
  uint32_t ul_frm_size;
  PROCESS_BEGIN();

  KSZ8863_config_intr();

  while(1) {
	  KSZ8863_ack_interrupt();
	  PROCESS_YIELD_UNTIL(ev == PROCESS_EVENT_POLL);

	  ul_value = ethernet_phy_Basic_Status_Register(GMAC, 1);
	  if(ul_value & MII_LINK_STATUS) // link up
	  {
		  gmac_dev_reset(&gs_gmac_dev, GMAC_QUE_0);
		  // Auto Negotiate, work in RMII mode
		  	if (ethernet_phy_auto_negotiate1(GMAC, 1) != GMAC_OK) {
		  		printf("Auto Negotiate ERROR!\r");
		  	}

		  	while(ethernet_phy_auto_negotiate2(GMAC, 1) != GMAC_OK){
		  		PROCESS_YIELD();
		  	}
		  	//ethernet_phy_auto_negotiate3(GMAC, 1);
		   //Establish ethernet link
		  	ul_frm_size = ethernet_phy_set_link(GMAC, 1, 1);
		  	if(ul_frm_size != GMAC_OK) {
		  		printf("Set link ERROR (%d)!\n",ul_frm_size);
		  	}
		  	link_status = 1;
		  	printf("-- Link up. \n");
	  }
	  else{
		  link_status = 0;
		  printf("-- Link down. \n");
	  }
  }

  PROCESS_END();
}
