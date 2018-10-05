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
#include "sysclk.h"
#include "gpio.h"
#include "node-id.h"
#include "dev/slip.h"
#include "udc.h"
#include "sleepmgr.h"
#include "lib/sensors.h"
#include "board-peripherals.h"
#include "i2csoft.h"
#include "ioport.h"
#include "leds.h"


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
extern int net_init_eth(void);
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
	//SetIEEEAddr(node_mac);

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

void initSWO(void)
{
	uint32_t SWOSpeed = 6000000; //6000kbps, default for JLinkSWOViewer
    uint32_t SWOPrescaler = (150000000 / SWOSpeed) - 1; // SWOSpeed in Hz, note that F_CPU is expected to be 150000000 in this case

	PMC->PMC_PCK[3] = 4 ;
	PMC->PMC_SCER = PMC_SCDR_PCK3;
	CoreDebug->DEMCR = (1<<24); //Set the TRCENA bit to 1 into the Debug Exception and Monitor Register (0xE000EDFC) to enable the use of trace and debug blocks

	TPI->SPPR = 2;//Write 0x2 into the Selected Pin Protocol Register. Select the Serial Wire output – NRZ
	TPI->FFCR = 0x100; //Write 0x100 into the Formatter and Flush Control Register
	TPI->ACPR = SWOPrescaler;//Set the suitable clock prescaler value into the Async Clock Prescaler Register to scale the baud rate of the asynchronous output (this can be done automatically by the debugging tool).

	ITM->LAR = 0xC5ACCE55; //Enable the write accesses into the ITM registers by writing “0xC5ACCE55” into the Lock Access Register
	ITM->TCR = 0x10009;// Write 0x00010015 into the Trace Control register
	//ITM->TPR = ITM_TPR_PRIVMASK_Msk; // ITM Trace Privilege Register
	ITM->TER |= 1;//Write 0x1 into the Trace Enable register: Enable the Stimulus port 0
}

void board_init(void)
{
	/* Disable the watchdog */
	WDT->WDT_MR = WDT_MR_WDDIS;
	//wdt_init(WDT, WDT_MR_WDRSTEN|WDT_MR_WDDBGHLT|WDT_MR_WDIDLEHLT, 0xfff, 0xfff);
	ioport_init();
	leds_init();
	leds_arch_set(LEDS_GREEN);

	initSWO();

	/*------------------------------------------------------------------------------*/
}

void enable_cache(void)
{
	//SCB_EnableICache();
	//SCB_EnableDCache();
}
