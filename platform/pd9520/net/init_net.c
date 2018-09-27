#include "same70.h"
#include "gmac.h"

void GMAC_Handler(void)
{
	gmac_handler(&gs_gmac_dev, GMAC_QUE_0);
}

#define printf(...)

int main(void)
{
	uint32_t ul_frm_size;
	volatile uint32_t ul_delay;
	gmac_options_t gmac_option;

	/* Display MAC & IP settings */
	printf("-- MAC %x:%x:%x:%x:%x:%x\n\r",
			gs_uc_mac_address[0], gs_uc_mac_address[1], gs_uc_mac_address[2],
			gs_uc_mac_address[3], gs_uc_mac_address[4], gs_uc_mac_address[5]);

	printf("-- IP  %d.%d.%d.%d\n\r", gs_uc_ip_address[0], gs_uc_ip_address[1],
			gs_uc_ip_address[2], gs_uc_ip_address[3]);

	/* Wait for PHY to be ready (CAT811: Max400ms) */
	ul_delay = sysclk_get_cpu_hz() / 1000 / 3 * 400;
	while (ul_delay--);

	/* Enable GMAC clock */
	pmc_enable_periph_clk(ID_GMAC);

	/* Fill in GMAC options */
	gmac_option.uc_copy_all_frame = 0;
	gmac_option.uc_no_boardcast = 0;

	memcpy(gmac_option.uc_mac_addr, gs_uc_mac_address, sizeof(gs_uc_mac_address));

	gs_gmac_dev.p_hw = GMAC;

	/* Init GMAC driver structure */
	gmac_dev_init(GMAC, &gs_gmac_dev, &gmac_option);

	/* Enable Interrupt */
	NVIC_EnableIRQ(GMAC_IRQn);

	/* Init MAC PHY driver */
	if (ethernet_phy_init(GMAC, BOARD_GMAC_PHY_ADDR, sysclk_get_cpu_hz())
					!= GMAC_OK) {
		puts("PHY Initialize ERROR!\r");
		return -1;
	}

	/* Auto Negotiate, work in RMII mode */
	if (ethernet_phy_auto_negotiate(GMAC, BOARD_GMAC_PHY_ADDR) != GMAC_OK) {

		puts("Auto Negotiate ERROR!\r");
		return -1;
	}

	/* Establish ethernet link */
	while (ethernet_phy_set_link(GMAC, BOARD_GMAC_PHY_ADDR, 1) != GMAC_OK) {
		puts("Set link ERROR!\r");
		return -1;
	}

	puts("-- Link detected. \r");

	while (1) {
		/* Process packets */
#if (SAM4E)
		if (GMAC_OK != gmac_dev_read(&gs_gmac_dev, (uint8_t *) gs_uc_eth_buffer,
						sizeof(gs_uc_eth_buffer), &ul_frm_size)) {
#else
		if (GMAC_OK != gmac_dev_read(&gs_gmac_dev, GMAC_QUE_0, (uint8_t *) gs_uc_eth_buffer,
						sizeof(gs_uc_eth_buffer), &ul_frm_size)) {
#endif
			continue;
		}

		if (ul_frm_size > 0) {
			/* Handle input frame */
			gmac_process_eth_packet((uint8_t *) gs_uc_eth_buffer, ul_frm_size);
		}
	}
}
