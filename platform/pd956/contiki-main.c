#include <stdint.h>
#include <stdio.h>
#include <sys/process.h>
#include <sys/procinit.h>
#include <etimer.h>
#include <sys/autostart.h>
#include <clock.h>
#include "sys/rtimer.h"
#include "sys/ctimer.h"
#include "net/netstack.h"
#include "contiki-net.h"
#include "conf_trx_access.h"
#include "sysclk.h"
#include "gpio.h"
#include "node-id.h"
#include "rf231.h"
#include "dev/slip.h"
#include "udc.h"
#include "sleepmgr.h"
#include "lib/sensors.h"
#include "board-peripherals.h"
#include "i2csoft.h"


#include "FLASH_driver.h"
#include "platform-conf.h"
#include "compiler.h"
#include "flash_efc.h"
#include "csprng.h"
#include "uECC.h"
#include "sha256.h"
//extern rtimer_arch_sleep(rtimer_clock_t howlong);

void board_init(void);
void enable_cache(void);
void init_node_mac(void);
void set_linkaddr(void);
void Setup_EEPROM(void);
void Set_time(void);

char cdc_serial_number[sizeof("99999998PD") - 1];

volatile devicecert_t device_certificate = (devicecert_t) {
	.private_key = "Replace point"
};

uint8_t sleepmgr_locks[SLEEPMGR_NR_OF_MODES];
//rtimer_clock_t static rtime_now,rtime_old;

int main()
{
	uint8_t version_var;
	uint8_t hash[32] = {0};
	uint8_t masterpublic_key_eeprom[64];

	sysclk_init();
	board_init();

	eeprom_init();

	// Hotfix to program eeprom. Must be removed
	get_eeprom(version, version_var);
	if(version_var != 0x48)
		Setup_EEPROM();

	enable_cache();
	get_eeprom(masterpublic_key,masterpublic_key_eeprom);
	/*
	 * we only start the firmware if the public signer key can
	 * be used to verify the devices public certificate.
	 * Otherwise we have a bad certificate.
	 */

	sha2_sha256( (uint8_t *)&device_certificate.crt.payloadfield_size_control, sizeof(device_certificate.crt.payloadfield_size_control),hash);

	if (!uECC_verify((void *)&masterpublic_key_eeprom, hash, sizeof(hash), (void *)&device_certificate.crt.signature, uECC_secp256r1())) {
		printf("uECC_verify() failed\n");
		while(1);
	}

	// START
//#if defined(NODE_HTU21D) || defined(NODE_BMP280) || defined(NODE_LM73)
//	SoftI2CInit();
//#endif
	flash_init_df();

	// Don't start USB on endnodes
#if !LOW_CLOCK //120Mhz
	memcpy(cdc_serial_number,(void *)&device_certificate.crt.snr,device_certificate.crt.snlen);
	udc_start();
#endif
	printf("Initialising\n");

	clock_init();
	rtimer_init();
	ctimer_init();

	process_init();
	process_start(&etimer_process, NULL);

	node_id_restore();
	memcpy(&uip_lladdr.addr, &node_mac, sizeof(linkaddr_t));
	set_linkaddr();

	queuebuf_init();
	netstack_init();
	SetIEEEAddr(node_mac);

	Load_time_from_RTC();
	csprng_start();

	process_start(&sensors_process, NULL);

#if NETSTACK_CONF_WITH_IPV6 || NETSTACK_CONF_WITH_IPV4
	process_start(&tcpip_process, NULL);
#endif

	autostart_start(autostart_processes);
	printf("Processes running\n");

	// Power down between tasks, when cpu is in idle
	sleepmgr_init();
	sleepmgr_lock_mode(SLEEPMGR_SLEEP_WFI);
	while(1){
		while(process_run());
		//rtime_old = rtimer_arch_now();
		//sleepmgr_enter_sleep();
		//rtime_now = rtimer_arch_now();
		//clock_adjust_ticks((rtime_now-rtime_old)*CLOCK_CONF_SECOND/RTIMER_ARCH_SECOND);
		// This will stop the clock.
		// Not worth it since we power down
		// when needed.
		//sleepmgr_enter_sleep();
	}
	return 0;
}

void Setup_EEPROM(void)
{
	tEepromContents EEPROM;
	uint32_t addr[2];

	eeprom_read(0, (void *) &EEPROM, sizeof(tEepromContents));

	EEPROM.PANID = IEEE802154_CONF_PANID;
	EEPROM.channel = 26;
	EEPROM.version = 0x48;

	flash_read_unique_id(EEPROM.Flash_unique_id, 4);

	addr[0] = __builtin_bswap32(EEPROM.Flash_unique_id[2]);
	addr[1] = __builtin_bswap32(EEPROM.Flash_unique_id[3]);

	memcpy(EEPROM.eepromMacAddress,addr,8);

	memcpy((void *)&EEPROM.masterpublic_key,(void *)&device_certificate.masterpublic_key,sizeof(device_certificate.masterpublic_key));

	eeprom_write(0, (void *) &EEPROM, sizeof(tEepromContents));
}

void set_linkaddr(void)
{

	linkaddr_t addr;
	memset(&addr, 0, LINKADDR_SIZE);
#if NETSTACK_CONF_WITH_IPV6
	memcpy(addr.u8, node_mac, sizeof(addr.u8));
#else
	int i;
	if(node_id == 0){
		for (i = 0; i < LINKADDR_SIZE ; ++i){
			addr.u8[i] = node_mac[LINKADDR_SIZE - 1 - i];
		}
	} else{
		addr.u8[0] = node_id & 0xff;
		addr.u8[1] = node_id >> 8;
	}
#endif
	linkaddr_set_node_addr(&addr);
#if DEBUG
	PRINTF("Link-layer address: ");
	for(i = 0; i < sizeof(addr.u8) - 1; i++){
		PRINTF("%d.", addr.u8[i]);
	}
	PRINTF("%d\n", addr.u8[i]);
#endif
}

/** SPI MISO pin definition. */
#define SPI_MISO_GPIO        	(PIO_PA12_IDX)
#define SPI_MISO_FLAGS       	(PIO_PERIPH_A | PIO_DEFAULT)
/** SPI MOSI pin definition. */
#define SPI_MOSI_GPIO   		(PIO_PA13_IDX)
#define SPI_MOSI_FLAGS       	(PIO_PERIPH_A | PIO_DEFAULT)
/** SPI SPCK pin definition. */
#define SPI_SPCK_GPIO			(PIO_PA14_IDX)
#define SPI_SPCK_FLAGS       	(PIO_PERIPH_A | PIO_DEFAULT)

/** SPI chip select 0 pin definition. (Only one configuration is possible) */
#define SPI_NPCS0_GPIO        	(PIO_PA11_IDX)
#define SPI_NPCS0_FLAGS       	(PIO_PERIPH_A | PIO_DEFAULT)

#define clkm_pin IOPORT_CREATE_PIN(PIOA, 26)

void board_init(void)
{
	/* Disable the watchdog */
	WDT->WDT_MR = WDT_MR_WDDIS;
	//wdt_init(WDT, WDT_MR_WDRSTEN|WDT_MR_WDDBGHLT|WDT_MR_WDIDLEHLT, 0xfff, 0xfff);
	ioport_init();

	/* Configure all unused PIOs as outputs and high to save power */
	pio_set_output(PIOA,
			(1 << 0)  | (1 << 2)  | (1 << 3)  | (1 << 4)  |
			(1 << 5)  | (1 << 6)  | (1 << 8)  | (1 << 9)  |
			(1 << 10) | (1 << 16) |	(1 << 17) | (1 << 18) |
			(1 << 19) | (1 << 26) |	(1 << 27) | (1 << 28) |
			(1 << 29) | (1 << 30) | (1 << 31),
			0, 0, 0);
/********************************************************************************/
	// Please verify that this does not influence on sleep mode current.
	// By setting these pins low hardlight does not startup
	// lighting all diodes. This problem must also be fixed in hardware!!!
	// NB: these pins are normally not used so this should not be a problem.
	/*pio_set_output(PIOA,
			(1 << 7) | (1 << 20),
			0, 0, 0);

	pio_set_output(PIOB,
			(1 << 4),
			0, 0, 0);*/
/********************************************************************************/
	pio_set_output(PIOB,
			(1 << 0) | (1 << 1)  | (1 << 2)  | (1 << 3) | (1 << 4) |
			(1 << 5) | (1 << 13) | (1 << 14),
			1, 0, 1);

#ifdef CONF_BOARD_UART_CONSOLE
	/* Configure UART pins */
	SETUP_CONSOLE(DEBUG_UART);
#endif
	/*------------------------------------------------------------------------------*/
	// SPI FLASH
	// Configure SPI pins
	gpio_configure_pin(SPI_MISO_GPIO, SPI_MISO_FLAGS);
	gpio_configure_pin(SPI_MOSI_GPIO, SPI_MOSI_FLAGS);
	gpio_configure_pin(SPI_SPCK_GPIO, SPI_SPCK_FLAGS);
	gpio_configure_pin(SPI_NPCS0_GPIO, SPI_NPCS0_FLAGS);
	/*------------------------------------------------------------------------------*/
	// SPI USART RADIO
	gpio_configure_pin(PIN_USART1_SCK_IDX, PIN_USART1_SCK_FLAGS);
	gpio_configure_pin(AT86RFX_SPI_MISO, SPI_MISO_FLAGS);
	gpio_configure_pin(AT86RFX_SPI_MOSI, SPI_MOSI_FLAGS);
	gpio_configure_pin(AT86RFX_SPI_SCK, SPI_SPCK_FLAGS);
	gpio_configure_pin(AT86RFX_SPI_CS_PIN, AT86RFX_SPI_CS_FLAGS);

	// Initialize TRX_RST and SLP_TR as GPIO.
	gpio_configure_pin(AT86RFX_RST_PIN, PIO_TYPE_PIO_OUTPUT_1);
	gpio_configure_pin(AT86RFX_SLP_PIN, PIO_TYPE_PIO_OUTPUT_1);
	/*------------------------------------------------------------------------------*/
}

void enable_cache(void)
{
// I'm currently operating on two cpu's. One with cache and one without.
	if((CHIPID->CHIPID_CIDR &0xFFFFFFFE) == 0x29970EE0){
		printf("I - Found cache. Enabling it.\n\r");
		// Enable the CMCC module. (cache)

		CMCC->CMCC_MCFG = 2;//CMCC_DHIT_COUNT_MODE;
		CMCC->CMCC_MEN |= CMCC_MEN_MENABLE;

		CMCC->CMCC_CTRL |= CMCC_CTRL_CEN;
	}
}
