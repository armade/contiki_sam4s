#include "node-id.h"
#include "contiki-conf.h"
#include "platform-conf.h"
#include "flash_efc.h"

unsigned short node_id = 0;
unsigned char node_mac[8];
/*---------------------------------------------------------------------------*/
#define _DEBUG_ 0
#if _DEBUG_
#include <stdio.h>
#define PRINTF(...)       printf(__VA_ARGS__)
#else
#define PRINTF(...)
#endif
/*---------------------------------------------------------------------------*/

void node_id_restore(void)
{
	if (node_id != 0)
	{
		node_mac[6] = node_id >> 8;
		node_mac[7] = node_id & 0xff;
		return;
	}

	get_eeprom(eepromMacAddress[0], node_mac[0]);
	get_eeprom(eepromMacAddress[1], node_mac[1]);
	get_eeprom(eepromMacAddress[2], node_mac[2]);
	get_eeprom(eepromMacAddress[3], node_mac[3]);
	get_eeprom(eepromMacAddress[4], node_mac[4]);
	get_eeprom(eepromMacAddress[5], node_mac[5]);
	get_eeprom(eepromMacAddress[6], node_mac[6]);
	get_eeprom(eepromMacAddress[7], node_mac[7]);

	/* set node ID */
	node_id = node_mac[7] | (node_mac[6] << 2);

}
/*---------------------------------------------------------------------------*/
void node_id_burn(unsigned short id)
{
	/* This is not implemented - burning is currently done from the command line */
	PRINTF("node-id: node_id_burn() not implemented. See 'make help'\n");
}
/*---------------------------------------------------------------------------*/
