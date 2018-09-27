
#ifndef INCLUDED_HL_SPIFLASH_H
#define INCLUDED_HL_SPIFLASH_H


#include "hw_spiflash.h"

#ifndef sf_flashchip_info
 extern unsigned char sf_flashchip_info[SF_MAXFLASHCHIPS];
#endif

#define CHIP_NOTPRESENT 	0
#define CHIP_AT25DF641  	1
#define CHIP_N25Q256    	2
#define CHIP_N25Q128    	3
#define CHIP_N25Q64     	4
#define CHIP_MX25L25645G	5
#define CHIP_MT25QL256ABA   6

unsigned sf_abit(int chipno);
unsigned sf_chipsize(int chipno);
unsigned sf_totalsize(void);
unsigned sf_map(unsigned a);


void sf_sread_start(unsigned a);
unsigned char sf_sread_end(void);

unsigned char sf_sread(void);

int sf_flashinit(void);

unsigned sf_is_erased_n(unsigned a, unsigned bytes);
unsigned sf_is_erased_4k(unsigned a);
int sf_erase4k(unsigned eraseaddr);
int sf_erase64k(unsigned eraseaddr);
int sf_write(unsigned dst, unsigned char *src, unsigned bytes);
int sf_getflashid_64byte(unsigned char *buf64);

void sm_write_start(unsigned addr);
void sm_write(unsigned d);
void sm_write_end(void);

void sm_read_start(unsigned addr);
unsigned sm_read(void);
void sm_read_end(void);

void sf_deepsleep_enter(void);
void sf_deepsleep_leave(void);

#endif
