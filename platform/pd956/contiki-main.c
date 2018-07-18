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
void init_node_mac(void);
void set_linkaddr(void);
void Setup_EEPROM(void);
void Set_time(void);

char cdc_serial_number[sizeof("99999998PD") - 1];

volatile devicecert_t device_certificate = (devicecert_t) {
	.private_key = "Replace point"
};

uint8_t sleepmgr_locks[SLEEPMGR_NR_OF_MODES];


int main()
{
	uint8_t version_var;
	crt_t eeprom_crt;
	uint8_t hash[32] = {0};
	uint32_t FLASH_id[4];
	uint32_t FLASH_id_eeprom[4];

	sysclk_init();
	board_init();

	SHA256_CTX CTX;

	eeprom_init();


	// Hotfix to program eeprom. Must be removed
	get_eeprom(version, version_var);
	if(version_var != 0x45)
		Setup_EEPROM();

	// Verify that the code is running on the processor it was programmed for.
	flash_read_unique_id(FLASH_id, 4);
	get_eeprom(Flash_unique_id, FLASH_id_eeprom);
	if(memcmp(FLASH_id_eeprom,FLASH_id,sizeof(FLASH_id)))
		while(1);
	/*
	 * we only start the firmware if the public signer key that has been
	 * provided with this firmware, can be used to verify the devices
	 * public certificate.
	 */
	get_eeprom(devicecert, eeprom_crt);

	sha2_sha256_init(&CTX);
	sha2_sha256_update(&CTX, (uint8_t *)&eeprom_crt.payloadfield_size_control, sizeof(eeprom_crt.payloadfield_size_control));
	sha2_sha256_final(&CTX, hash);

	if (!uECC_verify((void *)&device_certificate.masterpublic_key, hash, sizeof(hash), eeprom_crt.signature, uECC_secp256r1())) {
		printf("uECC_verify() failed\n");
		while(1);
	}

	// START
	SoftI2CInit();
	flash_init_df();

	// Don't start USB on endnodes
#if !LOW_CLOCK //120Mhz
	memcpy(cdc_serial_number,eeprom_crt.snr,eeprom_crt.snlen);
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

	csprng_start();
	Load_time_from_RTC();
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
		sleepmgr_enter_sleep();
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
	EEPROM.version = 0x45;

	flash_read_unique_id(EEPROM.Flash_unique_id, 4);

	addr[0] = __builtin_bswap32(EEPROM.Flash_unique_id[2]);
	addr[1] = __builtin_bswap32(EEPROM.Flash_unique_id[3]);

	memcpy(EEPROM.eepromMacAddress,addr,8);

	memcpy((void *)&EEPROM.devicecert,(void *)&device_certificate.crt,sizeof(device_certificate.crt));

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

	// I'm currently operating on two cpu's. One with cache and one without.
	/*if((CHIPID->CHIPID_CIDR &0xFFFFFFFE) == 0x29970EE0){
		printf("I - Found cache. Enabling it.\n\r");
		// Enable the CMCC module. (cache)

		CMCC->CMCC_MCFG = 2;//CMCC_DHIT_COUNT_MODE;
		CMCC->CMCC_MEN |= CMCC_MEN_MENABLE;

		CMCC->CMCC_CTRL |= CMCC_CTRL_CEN;
	}*/
	//wdt_init(WDT, WDT_MR_WDRSTEN|WDT_MR_WDDBGHLT|WDT_MR_WDIDLEHLT, 0xfff, 0xfff);
	ioport_init();

	/* Configure all unused PIOs as outputs and low to save power */
	pio_set_output(PIOA,
			(1 << 0) | (1 << 2) | (1 << 3)  | (1 << 4)  | (1 << 5)  | (1 << 6)
					| (1 << 7)  | (1 << 8)  | (1 << 9)  | (1 << 10) | (1 << 16)
					| (1 << 17) | (1 << 18) | (1 << 19) | (1 << 20) | (1 << 26)
					| (1 << 27) | (1 << 28) | (1 << 29) | (1 << 30) | (1 << 31),
			0, 0, 0);
	pio_set_output(PIOB,
			(1 << 0) | (1 << 1) | (1 << 2)  | (1 << 3)  | (1 << 4)  | (1 << 5)
					| (1 << 13) | (1 << 14),
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
	pio_configure_pin(AT86RFX_RST_PIN, PIO_TYPE_PIO_OUTPUT_1);
	//ioport_set_pin_level(AT86RFX_RST_PIN, IOPORT_PIN_LEVEL_HIGH);
	pio_configure_pin(AT86RFX_SLP_PIN, PIO_TYPE_PIO_OUTPUT_1);
	//ioport_set_pin_level(AT86RFX_SLP_PIN, IOPORT_PIN_LEVEL_HIGH);
	/*------------------------------------------------------------------------------*/
}

