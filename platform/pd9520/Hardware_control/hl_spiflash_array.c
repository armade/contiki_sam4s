#include <stdio.h>
#include "hl_spiflash.h"
#include "SPI1.h"

unsigned char sf_select;
#define sf_array_select_read() (sf_select)
#define sf_array_select_write(x) do { sf_select=(x); } while (0)

#define sf_array_enable()	SPI1_select(Seriel_flash_1+sf_select)
#define sf_array_disable() 	SPI1_select(none)
#define sf_array_rw(x)		SPI_Write(0,x)
#define sf_array_r()		SPI_Read()


#define DEBUG 1
#if DEBUG
#define PRINTF(...) printf(__VA_ARGS__)
#else
#define PRINTF(...)
#endif


unsigned char sf_array_flashchip_info[7];

#define chiptype() sf_array_flashchip_info[sf_array_select_read()]


void sf_array_cmd_s(unsigned char cmd)
{
	sf_array_enable();
	sf_array_rw(cmd); //read status
}

unsigned sf_array_cmd_1b(unsigned char cmd)
{
	unsigned d;
	sf_array_cmd_s(cmd);
	d=sf_array_r();
	sf_array_disable();
	return d;
}

void sf_array_cmd0(unsigned char cmd)
{
	sf_array_cmd_s(cmd);
	sf_array_disable();
}

void sf_array_writeenable(void)
{
	sf_array_cmd0(0x06);
}

void sf_array_deepsleep_enter(void)
{
	sf_array_cmd0(0xB9);
}

void sf_array_deepsleep_leave(void)
{
	sf_array_cmd0(0xAB);
	sf_tshortdelay();
}

void sf_array_writeprotect(void)
{
	sf_array_cmd0(0x04); //write disable
}


unsigned sf_array_abit(int chipno)
{
	if (chipno>=7) return 0;
	switch (sf_array_flashchip_info[chipno]) {
		case CHIP_AT25DF641: return 23;
		case CHIP_N25Q256:   return 25;
		case CHIP_N25Q128:   return 24;
		case CHIP_N25Q64:    return 23;
		case CHIP_MX25L25645G: return 25;
		case CHIP_MT25QL256ABA:   return 25;
		default: return 0;
	}
}

unsigned sf_array_chipsize(int chipno)
{
	unsigned bit=sf_array_abit(chipno);
	if (!bit) return 0;
	return 1<<bit;
}

unsigned sf_array_totalsize(void)
{
	unsigned i,tot;
	tot=0;
	for (i=0; i<7; ++i) tot+=sf_array_chipsize(i);
	return tot;
}

unsigned sf_array_map(unsigned a)
{
	unsigned s,i;
	for (i=0; i<7; ++i) {
		s=sf_array_chipsize(i);
		if (a<s) {
			sf_array_select_write(i);
			return a;
		}
		a-=s;
	}
	return 0;
}




#define IAP32 0x80

void sf_array_iap_r(unsigned char ins, unsigned a, unsigned pad)
{
	switch (chiptype()) {
//		case CHIP_AT25DF641:
//		case CHIP_N25Q128:
//		case CHIP_N25Q64:
//			break;
		case CHIP_N25Q256:
		case CHIP_MX25L25645G:
//		case CHIP_MT25QL256ABA:
			sf_array_writeenable();
			sf_array_enable();
			sf_array_rw(0xC5);
			sf_array_rw(a>>24);
			break;
	}
	sf_array_writeprotect();
	sf_array_enable();
	sf_array_rw(ins); //array read max 75MHz, 1 padding byte
	if (pad&IAP32) {
		pad-=IAP32;
		sf_array_rw(a>>24);
	}
	sf_array_rw(a>>16);
	sf_array_rw(a>> 8);
	sf_array_rw(a    );
	while (pad) {
	  sf_array_rw(0);
	  --pad;
	}
}

void sf_array_iap_w(unsigned char ins, unsigned a, unsigned pad)
{
	switch (chiptype()) {
//		case CHIP_AT25DF641:
//		case CHIP_N25Q128:
//		case CHIP_N25Q64:
//			break;
		case CHIP_N25Q256:
		case CHIP_MX25L25645G:
  //		case CHIP_MT25QL256ABA:
			sf_array_writeenable();
			sf_array_enable();
			sf_array_rw(0xC5);
			sf_array_rw(a>>24);
			break;
	}
	sf_array_writeenable();
	sf_array_enable();
	sf_array_rw(ins); //array read max 75MHz, 1 padding byte
	if (pad&IAP32) {
		pad-=IAP32;
		sf_array_rw(a>>24);
	}
	sf_array_rw(a>>16);
	sf_array_rw(a>> 8);
	sf_array_rw(a    );
	while (pad) {
	  sf_array_rw(0);
	  --pad;
	}
}

void sf_array_iap_e(void)
{
	switch (chiptype()) {
//		case CHIP_AT25DF641:
//		case CHIP_N25Q128:
//		case CHIP_N25Q64:
//			break;
		case CHIP_N25Q256:
		case CHIP_MX25L25645G:
//		case CHIP_MT25QL256ABA:
			sf_array_writeenable();
			sf_array_enable();
			sf_array_rw(0xC5);
			sf_array_rw(0);
			break;
	}
	sf_array_writeprotect();
}

#ifdef SPIDMAREAD
void sf_array_read_dma(unsigned char *dst, unsigned a, unsigned n)
{
	switch (chiptype()) {
		case CHIP_MT25QL256ABA:
			sf_array_iap_r(0x0C,a,1 | IAP32); //4 byte fast read med 32bit adresse
			break;
		default:
			sf_array_iap_r(0x0B,a,1);
	}
	hw_spi_dmaread(dst,n);
	sf_array_disable();
}
#endif

void sf_array_sread_start(unsigned a)
{
	switch (chiptype()) {
		case CHIP_MT25QL256ABA:
			sf_array_iap_r(0x0C,a,1 | IAP32); //4 byte fast read med 32bit adresse
			break;

		default:
			sf_array_iap_r(0x0B,a,1);
	}
	sf_array_startstreamread();
}

unsigned char sf_array_sread_end(void)
{
	unsigned d;
	d=sf_array_endstreamread();
	sf_array_iap_e();
	return d;
}

unsigned sf_array_status(void)
{
	return sf_array_cmd_1b(0x05);
}

unsigned sf_array_status_atmel(void)
{
	unsigned d;
	sf_array_cmd_s(0x05); //read status
	d=sf_array_r();
	d|=sf_array_r()<<8; //2nd byte is in high bit
	sf_array_disable();
	return d;
}

void sf_array_waitready(void)
{
	do {
		sf_waiting();
	} while (sf_array_status()&1);
}

static unsigned read_this_chipid(void)
{
	unsigned m,i;
	sf_array_cmd_s(0x9F); //read ID
	m=0;
	for (i=0; i<4; ++i) m=(m<<8) | sf_array_r();
	sf_array_disable();
	return m;
}

static unsigned char read_discovery_byte(unsigned a)
{
	sf_array_cmd_s(0x5A); //flash discovery
	sf_array_rw(a>>16);
	sf_array_rw(a>>8 );
	sf_array_rw(a    );
	sf_array_r(); //dummy
	a=sf_array_r();
	sf_array_disable();
	return a;
}

int sf_array_flashinit(void)
{
	unsigned m,i,good,retrying;

	PRINTF("init before hw\n");
	SPI1_init();
	PRINTF("init after hw\n");

#ifdef MRAM_INITIAL_AB_CMD
	sm_enable();
	sm_rw(0xAB); //MRAM resume from power down
	sm_disable();
#endif
	PRINTF("init after mram\n");

	good=0;
	sf_tlongdelay();
	for (i=0; i<7; ++i) {
		retrying=0;
retry_recovery:
		PRINTF("select %d\n",i);
		sf_array_select_write(i);
		sf_array_flashchip_info[i]=CHIP_NOTPRESENT;


		switch(retrying) {
			case 0:
				break;
			case 1:
				PRINTF("Wake from sleep\n");
				sf_tlongdelay();
				sf_array_cmd0(0xAB); //resume from power down
				sf_tlongdelay();
				break;
			default:
				continue; //skip
		}

		//read id
		m=read_this_chipid();
		PRINTF("read id: 0x%x\n",m);
		if ((m&0xFFFF0000U)==0x1F480000U) { //AT25DF641
			PRINTF("AT25DF641\n");
			good|=1<<i;
			sf_array_flashchip_info[i]=CHIP_AT25DF641; //8MB
rewait1:	sf_array_waitready();
			if (sf_array_status_atmel() & (3<<9)) {
				PRINTF("resume\n");
				sf_array_cmd0(0xD0); //program/erase resume
				sf_tlongdelay();
				goto rewait1;
			}
		} else
		if ((m==0x20BA1910U) || (m==0x20BA1810U) || (m==0x20BA1710U)) { //N25Q 256 128 64
			good|=1<<i;
			if (m == 0x20BA1910U) m=CHIP_N25Q256; //32MB
			else
			if (m == 0x20BA1810U) m=CHIP_N25Q128; //16MB
			else
			if (m == 0x20BA1710U) m=CHIP_N25Q64; //8MB
			else
				m=0; //unreachable kode

			if (m==CHIP_N25Q256) {
				//BAD: MT25QL256 has same idcode as N25Q256, but different flash discovery version
				if ((read_discovery_byte(4)!=0) || (read_discovery_byte(5)!=1)) {
					m=CHIP_MT25QL256ABA;
				}
			}
			PRINTF("N25Q type: 0x%x\n",m);
			// MT25QL128ABA is identically with CHIP_N25Q128
			// MT25QL64ABA is identically with CHIP_N25Q64

			sf_array_flashchip_info[i]=m;
rewait2:	sf_array_waitready();
			if (sf_array_cmd_1b(0x70) & ((1<<2)|(1<<6))) { //micron flags register
				PRINTF("resume\n");
				sf_array_cmd0(0x7A); //program/erase resume
				sf_tlongdelay();
				goto rewait2;
			}
			if (m==CHIP_N25Q256) sf_array_cmd0(0xE9); //EXIT 4 BYTE MODE
		} else
		if ((m&0xFFFFFF00U)==0xC2201900U) { //MX25L25645G 32MB
			good|=1<<i;
			sf_array_flashchip_info[i]=CHIP_MX25L25645G;
			PRINTF("MX25L\n");
rewait3:	sf_array_waitready();
			if (sf_array_cmd_1b(0x2B) & (3<<2)) { //program or erase suspend?
				PRINTF("resume\n");
				sf_array_cmd0(0x30); //program/erase resume
				goto rewait3;
			}
			sf_array_cmd0(0xC1); //EXIT SECURE OTP
			sf_array_cmd0(0xE9); //EXIT 4 BYTE MODE
		} else {
			PRINTF("not found: %d",retrying);
			//Nothing found
			++retrying;
			goto retry_recovery;
		}

	}
	return !good; //returns 1 on error
}

int sf_array_unprotect_sector(unsigned saddr)
{
	unsigned a;
	//read status
	switch (chiptype()) {
		case CHIP_AT25DF641:
			a=sf_array_status();
			//unprotect if protected
			if ((a&0x0C)!=0x00) { //bit 3,2 not 00, some sectors locked, need to unlock
				if (a&0x80) { //bit 7 set, sector protection registers are locked.
					sf_array_cmd_s(0x01); //write status 1
					sf_array_rw(0x00); //unlock sector protection register
					sf_array_waitready(); //hmm...
				}
//				sf_array_writeenable();
				sf_array_iap_w(0x39,saddr,0); //unprotect sector
				sf_array_iap_r(0x3C,saddr,0); //read protection status
				a=sf_array_r();
				sf_array_disable();
				if (a) return -1;
			}
			return 0;
		case CHIP_N25Q256:
		case CHIP_N25Q128:
		case CHIP_N25Q64:
			a=sf_array_status();
			//unprotect if protected
			if (a&0x5C) { //one or more block bits are set.
				sf_array_cmd_s(0x01); //write status 1
				sf_array_rw(a&0x20); //Keep top/bottom bit
				sf_array_waitready();
			}
			sf_array_iap_w(0xE8,saddr,0);
			if (sf_array_r()&1) {
				sf_array_iap_w(0xE5,saddr,1); //unprotect sector, OBS: 1 zerobyte
				sf_array_waitready();
				sf_array_iap_e();
			}
			return 0;
		case CHIP_MT25QL256ABA:
			a=sf_array_status();
			//unprotect if protected
			if (a&0x5C) { //one or more block bits are set.
				sf_array_cmd_s(0x01); //write status 1
				sf_array_rw(a&0x20); //keep top/bottom bit
				sf_array_waitready();
			}
			sf_array_iap_w(0xE0,saddr,0 | IAP32);
			if (sf_array_r()&1) {
				sf_array_iap_w(0xE1,saddr,1 | IAP32); //unprotect sector, OBS: 1 zerobyte
				sf_array_waitready();
				sf_array_iap_e();
			}
			return 0;
		case CHIP_MX25L25645G:
			//no support for unlocking, it works with factory defaults.
			return 0;
		default:
			return -1;
	}
}

unsigned sf_array_is_erased_n(unsigned a, unsigned bytes)
{
	unsigned n;
	sf_array_sread_start(a);
	for (n=0; n<bytes; ++n) {
		if (sf_array_sread()!=0xff) {
			sf_array_sread_end();
			return 0;
		}
	}
	sf_array_sread_end();
	return 1;
}

unsigned sf_array_is_erased_4k(unsigned a)
{
	return sf_array_is_erased_n(a,4096);
}

int sf_array_erase4k(unsigned eraseaddr)
{
	sf_array_unprotect_sector(eraseaddr);
	switch (chiptype()) {
		case CHIP_MT25QL256ABA:
			sf_array_iap_w(0x21,eraseaddr,0 | IAP32);
			break;

		default:
			sf_array_iap_w(0x20,eraseaddr,0);
	}
	sf_array_disable();
	//wait
	sf_array_waitready();
	sf_array_iap_e();
	return 0;
}

int sf_array_erase64k(unsigned eraseaddr)
{
	sf_array_unprotect_sector(eraseaddr);
	switch (chiptype()) {
		case CHIP_MT25QL256ABA:
			sf_array_iap_w(0xDC,eraseaddr,0 | IAP32);
			break;

		default:
			sf_array_iap_w(0xD8,eraseaddr,0);
	}
	sf_array_disable();
	//wait
	sf_array_waitready();
	sf_array_iap_e();
	return 0;
}

int sf_array_write(unsigned dst, unsigned char *src, unsigned bytes)
{
	//int a=
	sf_array_unprotect_sector(dst);
	//a=0;
	do {
		switch (chiptype()) {
			case CHIP_MT25QL256ABA:
				sf_array_iap_w(0x12,dst,0 | IAP32);
				break;

			default:
				sf_array_iap_w(0x02,dst,0);
		}
		//transfer page
		do {
			sf_array_rw(*src++);
			++dst;
			--bytes;
		} while ((dst&0xFF) && (bytes));
		sf_array_disable();
		sf_array_waitready();
	} while (bytes);
	sf_array_disable();
	sf_array_iap_e();
	return 0;
}

int sf_array_getflashid_64byte(unsigned char *buf64)
{
	int i;

	sf_array_map(0);
	switch (chiptype()) {
		case CHIP_AT25DF641:
			sf_array_iap_r(0x77,64,2);
			for (i=0; i<64; ++i) *buf64++=sf_array_r();
			sf_array_disable();
			return 1;

		case CHIP_N25Q256:
		case CHIP_N25Q128:
		case CHIP_N25Q64:
		case CHIP_MT25QL256ABA:
			sf_array_cmd_s(0x9F); //read ID
			for (i=0; i<20; ++i) *buf64++=sf_array_r();
			for (   ; i<64; ++i) *buf64++=0x52; //filler for N25Q = 0x52
			sf_array_disable();
			return 1;
		case CHIP_MX25L25645G:
			sf_array_cmd0(0xB1); //enter otp
			sf_array_iap_r(0x0B,0,1);
			for (i=0; i<16; ++i) *buf64++=sf_array_r();
			for (   ; i<64; ++i) *buf64++=0x53; //filler for MX25 = 0x53
			sf_array_cmd0(0xC1); //exit otp
		default:
			return 0;
	}
}




