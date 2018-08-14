
#include "contiki.h"
#include "contiki-net.h"
#include "uip.h"
#include "net/rpl/rpl.h"
#include "sys/node-id.h"
#include "servreg-hack.h"
#include <string.h>

#include "FLASH_driver.h"

#define DATASIZE		256
#define UDP_PORT        9898
#define SERVICE_ID      190

#define FLASH_FIRMWARE_OFFSET	4096

struct msg {
  uint16_t streamno;
  uint16_t seqno;
  uint8_t buf[DATASIZE];
};

static struct simple_udp_connection unicast_connection;
static uint8_t flash_is_sleeping = 1;
static uint16_t seqno_end;
static uint16_t streamno_current;
/*---------------------------------------------------------------------------*/
PROCESS(udpstream_process, "UDP Stream Process");
AUTOSTART_PROCESSES(&udpstream_process);

/*---------------------------------------------------------------------------*/
static void
receiver(struct simple_udp_connection *c,
         const uip_ipaddr_t *sender_addr,
         uint16_t sender_port,
         const uip_ipaddr_t *receiver_addr,
         uint16_t receiver_port,
         const uint8_t *data,
         uint16_t datalen)
{
	struct msg *diagram = (struct msg *)data;

	if(flash_is_sleeping){
		// TODO: apply UUID to packet so we know it's for us
		if(!memcmp(diagram->buf,"Upgrade",sizeof("Upgrade")-1)){

			seqno_end = diagram->seqno; // seqno indicate the size of the firmware in seqno but only in the first frame.
			if(seqno_end > 512) // 512 diagrams *256 bytes = 131072 = 128kB
				return;

			streamno_current = diagram->streamno;

			flash_leave_deep_sleep();
			flash_is_sleeping = 0;

			memset((char *)diagram->buf,0,DATASIZE);
			diagram->buf[0] = 'R';
			simple_udp_sendto(&unicast_connection, &diagram->buf, sizeof("R"), sender_addr);
			flash_erase_df(FLASH_FIRMWARE_OFFSET+0*64*1024,erase_64k_block);
			flash_erase_df(FLASH_FIRMWARE_OFFSET+1*64*1024,erase_64k_block);

		}
		// TODO: Make it possible to ask for firmware version
	}
	else{
		//TODO: write into correct location, and let bootloader erase the flash area.
		// And size check. Let bootloader verify signature. If signature is valid then move
		// firmware to other location.
		// Add timeout to this after debug.
		// Disable sleep.

		if((streamno_current == diagram->streamno) && (seqno_end >= diagram->seqno)){
			flash_write_df(diagram->seqno+FLASH_FIRMWARE_OFFSET, diagram->buf, datalen);
			memset((char *)diagram->buf,0,DATASIZE);
			diagram->buf[0] = 'P';

			while(diagram->seqno > 99){
				diagram->buf[3]++;
				diagram->seqno-=100;
			}
			while(diagram->seqno > 9){
				diagram->buf[2]++;
				diagram->seqno-=10;
			}
			diagram->buf[1] = diagram->seqno + 0x30;
			diagram->buf[2] += 0x30;
			diagram->buf[3] += 0x30;

			simple_udp_sendto(&unicast_connection, &diagram->buf, sizeof("Pxxx"), sender_addr);
		}else{
			flash_enter_deep_sleep();
			flash_is_sleeping = 1;
			memset((char *)diagram->buf,0,DATASIZE);
			diagram->buf[0] = 'S';
			simple_udp_sendto(&unicast_connection, &diagram->buf, sizeof("S"), sender_addr);
		}
	}

}

/*---------------------------------------------------------------------------*/
PROCESS_THREAD(udpstream_process, ev, data)
{
	//static uip_ds6_addr_t *ip_addr;

	PROCESS_BEGIN();

	/* Start service registration */
	servreg_hack_init();

	PROCESS_WAIT_UNTIL(uip_ds6_get_global(ADDR_PREFERRED));



    /* The sink creates a dag and waits for UDP datagrams */
    //servreg_hack_register(SERVICE_ID, &ip_addr->ipaddr);
    simple_udp_register(&unicast_connection, UDP_PORT,
                        NULL, UDP_PORT, receiver);
    while(1) {
    	PROCESS_WAIT_EVENT();
    }


  PROCESS_END();
}
/*---------------------------------------------------------------------------*/
