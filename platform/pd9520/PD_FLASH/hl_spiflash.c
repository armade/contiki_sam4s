
#include "hl_spiflash.h"


#define FLASHTRACE(s)
#define FLASHTRACE1(s,v)

#define FLASHTRACE(s)
#define FLASHTRACE1(s,v)


#ifndef sf_flashchip_info
  unsigned char sf_flashchip_info[SF_MAXFLASHCHIPS];
#endif
#define chiptype() sf_flashchip_info[sf_select_read()]


void sf_cmd_s(unsigned char cmd)
{
	sf_enable();
	sf_rw(cmd); //read status
}

unsigned sf_cmd_1b(unsigned char cmd)
{
	unsigned d;
	sf_cmd_s(cmd);
	d=sf_r();
	sf_disable();
	return d;
}

void sf_cmd0(unsigned char cmd)
{
	sf_cmd_s(cmd);
	sf_disable();
}

void sf_writeenable(void)
{
	sf_cmd0(0x06);
}

void sf_deepsleep_enter(void)
{
	sf_cmd0(0xB9);
}

void sf_deepsleep_leave(void)
{
	sf_cmd0(0xAB);
	sf_tshortdelay();
}

void sf_writeprotect(void)
{
	sf_cmd0(0x04); //write disable
}


unsigned sf_abit(int chipno)
{
	if (chipno>=SF_MAXFLASHCHIPS) return 0;
	switch (sf_flashchip_info[chipno]) {
		case CHIP_AT25DF641: return 23;
		case CHIP_N25Q256:   return 25;
		case CHIP_N25Q128:   return 24;
		case CHIP_N25Q64:    return 23;
		case CHIP_MX25L25645G: return 25;
		case CHIP_MT25QL256ABA:   return 25;
		default: return 0;
	}
}

unsigned sf_chipsize(int chipno)
{
	unsigned bit=sf_abit(chipno);
	if (!bit) return 0;
	return 1<<bit;
}

unsigned sf_totalsize(void)
{
	unsigned i,tot;
	tot=0;
	for (i=0; i<SF_MAXFLASHCHIPS; ++i) tot+=sf_chipsize(i);
	return tot;
}

unsigned sf_map(unsigned a)
{
	unsigned s,i;
	for (i=0; i<SF_MAXFLASHCHIPS; ++i) {
		s=sf_chipsize(i);
		if (a<s) {
			sf_select_write(i);
			return a;
		}
		a-=s;
	}
	return 0;
}




#define IAP32 0x80

void sf_iap_r(unsigned char ins, unsigned a, unsigned pad)
{
	switch (chiptype()) {
//		case CHIP_AT25DF641:
//		case CHIP_N25Q128:
//		case CHIP_N25Q64:
//			break;
		case CHIP_N25Q256:
		case CHIP_MX25L25645G:
//		case CHIP_MT25QL256ABA:
			sf_writeenable();
			sf_enable();
			sf_rw(0xC5);
			sf_rw(a>>24);
			break;
	}
	sf_writeprotect();
	sf_enable();
	sf_rw(ins); //array read max 75MHz, 1 padding byte
	if (pad&IAP32) {
		pad-=IAP32;
		sf_rw(a>>24);
	}
	sf_rw(a>>16);
	sf_rw(a>> 8);
	sf_rw(a    );
	while (pad) {
	  sf_rw(0);
	  --pad;
	}
}

void sf_iap_w(unsigned char ins, unsigned a, unsigned pad)
{
	switch (chiptype()) {
//		case CHIP_AT25DF641:
//		case CHIP_N25Q128:
//		case CHIP_N25Q64:
//			break;
		case CHIP_N25Q256:
		case CHIP_MX25L25645G:
  //		case CHIP_MT25QL256ABA:
			sf_writeenable();
			sf_enable();
			sf_rw(0xC5);
			sf_rw(a>>24);
			break;
	}
	sf_writeenable();
	sf_enable();
	sf_rw(ins); //array read max 75MHz, 1 padding byte
	if (pad&IAP32) {
		pad-=IAP32;
		sf_rw(a>>24);
	}
	sf_rw(a>>16);
	sf_rw(a>> 8);
	sf_rw(a    );
	while (pad) {
	  sf_rw(0);
	  --pad;
	}
}

void sf_iap_e(void)
{
	switch (chiptype()) {
//		case CHIP_AT25DF641:
//		case CHIP_N25Q128:
//		case CHIP_N25Q64:
//			break;
		case CHIP_N25Q256:
		case CHIP_MX25L25645G:
//		case CHIP_MT25QL256ABA:
			sf_writeenable();
			sf_enable();
			sf_rw(0xC5);
			sf_rw(0);
			break;
	}
	sf_writeprotect();
}

#ifdef SPIDMAREAD
void sf_read_dma(unsigned char *dst, unsigned a, unsigned n)
{
	switch (chiptype()) {
		case CHIP_MT25QL256ABA:
			sf_iap_r(0x0C,a,1 | IAP32); //4 byte fast read med 32bit adresse
			break;
		default:
			sf_iap_r(0x0B,a,1);
	}
	hw_spi_dmaread(dst,n);
	sf_disable();
}
#endif

void sf_sread_start(unsigned a)
{
	switch (chiptype()) {
		case CHIP_MT25QL256ABA:
			sf_iap_r(0x0C,a,1 | IAP32); //4 byte fast read med 32bit adresse
			break;

		default:
			sf_iap_r(0x0B,a,1);
	}
	sf_startstreamread();
}

unsigned char sf_sread_end(void)
{
	unsigned d;
	d=sf_endstreamread();
	sf_iap_e();
	return d;
}

unsigned sf_status(void)
{
	return sf_cmd_1b(0x05);
}

unsigned sf_status_atmel(void)
{
	unsigned d;
	sf_cmd_s(0x05); //read status
	d=sf_r();
	d|=sf_r()<<8; //2nd byte is in high bit
	sf_disable();
	return d;
}

void sf_waitready(void)
{
	do {
		sf_waiting();
	} while (sf_status()&1);
}

static unsigned read_this_chipid(void)
{
	unsigned m,i;
	sf_cmd_s(0x9F); //read ID
	m=0;
	for (i=0; i<4; ++i) m=(m<<8) | sf_r();
	sf_disable();
	return m;
}

static unsigned char read_discovery_byte(unsigned a)
{
	sf_cmd_s(0x5A); //flash discovery
	sf_rw(a>>16);
	sf_rw(a>>8 );
	sf_rw(a    );
	sf_r(); //dummy
	a=sf_r();
	sf_disable();
	return a;
}

int sf_flashinit(void)
{
	unsigned m,i,good,retrying;

	FLASHTRACE("init before hw");
	hw_spi_init();
	FLASHTRACE("init after hw");

#ifdef MRAM_INITIAL_AB_CMD
	sm_enable();
	sm_rw(0xAB); //MRAM resume from power down
	sm_disable();
#endif
	FLASHTRACE("init after mram");

	good=0;
	sf_tlongdelay();
	for (i=0; i<SF_MAXFLASHCHIPS; ++i) {
		retrying=0;
retry_recovery:
		FLASHTRACE1("select",i);
		sf_select_write(i);
		sf_flashchip_info[i]=CHIP_NOTPRESENT;


		switch(retrying) {
			case 0:
				break;
			case 1:
				FLASHTRACE("resume pd");
				sf_tlongdelay();
				sf_cmd0(0xAB); //resume from power down
				sf_tlongdelay();
				break;
			default:
				continue; //skip
		}

		//read id
		m=read_this_chipid();
		FLASHTRACE1("read id",m);
		if ((m&0xFFFF0000U)==0x1F480000U) { //AT25DF641
			FLASHTRACE("AT25DF641");
			good|=1<<i;
			sf_flashchip_info[i]=CHIP_AT25DF641; //8MB
rewait1:	sf_waitready();
			if (sf_status_atmel() & (3<<9)) {
				FLASHTRACE("resume");
				sf_cmd0(0xD0); //program/erase resume
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
			FLASHTRACE1("N25Q type ",m);
			// MT25QL128ABA is identically with CHIP_N25Q128
			// MT25QL64ABA is identically with CHIP_N25Q64

			sf_flashchip_info[i]=m;
rewait2:	sf_waitready();
			if (sf_cmd_1b(0x70) & ((1<<2)|(1<<6))) { //micron flags register
				FLASHTRACE("resume");
				sf_cmd0(0x7A); //program/erase resume
				sf_tlongdelay();
				goto rewait2;
			}
			if (m==CHIP_N25Q256) sf_cmd0(0xE9); //EXIT 4 BYTE MODE
		} else
		if ((m&0xFFFFFF00U)==0xC2201900U) { //MX25L25645G 32MB
			good|=1<<i;
			sf_flashchip_info[i]=CHIP_MX25L25645G;
			FLASHTRACE("MX25L");
rewait3:	sf_waitready();
			if (sf_cmd_1b(0x2B) & (3<<2)) { //program or erase suspend?
				FLASHTRACE("resume");
				sf_cmd0(0x30); //program/erase resume
				goto rewait3;
			}
			sf_cmd0(0xC1); //EXIT SECURE OTP
			sf_cmd0(0xE9); //EXIT 4 BYTE MODE
		} else {
			FLASHTRACE1("not found",retrying);
			//Nothing found
			++retrying;
			goto retry_recovery;
		}

	}
	return !good; //returns 1 on error
}

int sf_unprotect_sector(unsigned saddr)
{
	unsigned a;
	//read status
	switch (chiptype()) {
		case CHIP_AT25DF641:
			a=sf_status();
			//unprotect if protected
			if ((a&0x0C)!=0x00) { //bit 3,2 not 00, some sectors locked, need to unlock
				if (a&0x80) { //bit 7 set, sector protection registers are locked.
					sf_cmd_s(0x01); //write status 1
					sf_rw(0x00); //unlock sector protection register
					sf_waitready(); //hmm...
				}
//				sf_writeenable();
				sf_iap_w(0x39,saddr,0); //unprotect sector
				sf_iap_r(0x3C,saddr,0); //read protection status
				a=sf_r();
				sf_disable();
				if (a) return -1;
			}
			return 0;
		case CHIP_N25Q256:
		case CHIP_N25Q128:
		case CHIP_N25Q64:
			a=sf_status();
			//unprotect if protected
			if (a&0x5C) { //one or more block bits are set.
				sf_cmd_s(0x01); //write status 1
				sf_rw(a&0x20); //Keep top/bottom bit
				sf_waitready();
			}
			sf_iap_w(0xE8,saddr,0);
			if (sf_r()&1) {
				sf_iap_w(0xE5,saddr,1); //unprotect sector, OBS: 1 zerobyte
				sf_waitready();
				sf_iap_e();
			}
			return 0;
		case CHIP_MT25QL256ABA:
			a=sf_status();
			//unprotect if protected
			if (a&0x5C) { //one or more block bits are set.
				sf_cmd_s(0x01); //write status 1
				sf_rw(a&0x20); //keep top/bottom bit
				sf_waitready();
			}
			sf_iap_w(0xE0,saddr,0 | IAP32);
			if (sf_r()&1) {
				sf_iap_w(0xE1,saddr,1 | IAP32); //unprotect sector, OBS: 1 zerobyte
				sf_waitready();
				sf_iap_e();
			}
			return 0;
		case CHIP_MX25L25645G:
			//no support for unlocking, it works with factory defaults.
			return 0;
		default:
			return -1;
	}
}

unsigned sf_is_erased_n(unsigned a, unsigned bytes)
{
	unsigned n;
	sf_sread_start(a);
	for (n=0; n<bytes; ++n) {
		if (sf_sread()!=0xff) {
			sf_sread_end();
			return 0;
		}
	}
	sf_sread_end();
	return 1;
}

unsigned sf_is_erased_4k(unsigned a)
{
	return sf_is_erased_n(a,4096);
}

int sf_erase4k(unsigned eraseaddr)
{
	sf_unprotect_sector(eraseaddr);
	switch (chiptype()) {
		case CHIP_MT25QL256ABA:
			sf_iap_w(0x21,eraseaddr,0 | IAP32);
			break;

		default:
			sf_iap_w(0x20,eraseaddr,0);
	}
	sf_disable();
	//wait
	sf_waitready();
	sf_iap_e();
	return 0;
}

int sf_erase64k(unsigned eraseaddr)
{
	sf_unprotect_sector(eraseaddr);
	switch (chiptype()) {
		case CHIP_MT25QL256ABA:
			sf_iap_w(0xDC,eraseaddr,0 | IAP32);
			break;

		default:
			sf_iap_w(0xD8,eraseaddr,0);
	}
	sf_disable();
	//wait
	sf_waitready();
	sf_iap_e();
	return 0;
}

int sf_write(unsigned dst, unsigned char *src, unsigned bytes)
{
	//int a=
	sf_unprotect_sector(dst);
	//a=0;
	do {
		switch (chiptype()) {
			case CHIP_MT25QL256ABA:
				sf_iap_w(0x12,dst,0 | IAP32);
				break;

			default:
				sf_iap_w(0x02,dst,0);
		}
		//transfer page
		do {
			sf_rw(*src++);
			++dst;
			--bytes;
		} while ((dst&0xFF) && (bytes));
		sf_disable();
		sf_waitready();
	} while (bytes);
	sf_disable();
	sf_iap_e();
	return 0;
}

int sf_getflashid_64byte(unsigned char *buf64)
{
	int i;

	sf_map(0);
	switch (chiptype()) {
		case CHIP_AT25DF641:
			sf_iap_r(0x77,64,2);
			for (i=0; i<64; ++i) *buf64++=sf_r();
			sf_disable();
			return 1;

		case CHIP_N25Q256:
		case CHIP_N25Q128:
		case CHIP_N25Q64:
		case CHIP_MT25QL256ABA:
			sf_cmd_s(0x9F); //read ID
			for (i=0; i<20; ++i) *buf64++=sf_r();
			for (   ; i<64; ++i) *buf64++=0x52; //filler for N25Q = 0x52
			sf_disable();
			return 1;
		case CHIP_MX25L25645G:
			sf_cmd0(0xB1); //enter otp
			sf_iap_r(0x0B,0,1);
			for (i=0; i<16; ++i) *buf64++=sf_r();
			for (   ; i<64; ++i) *buf64++=0x53; //filler for MX25 = 0x53
			sf_cmd0(0xC1); //exit otp
		default:
			return 0;
	}
}




