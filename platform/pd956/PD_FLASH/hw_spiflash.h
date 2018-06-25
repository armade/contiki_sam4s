

#ifndef INCLUDED_HW_SPIFLASH_H
#define INCLUDED_HW_SPIFLASH_H

//1..N
#define SF_MAXFLASHCHIPS 1

//MRAM valg: (undef hvis ingen MRAM)
#define MRAM_SIZE_IN_BYTES 32768



void hw_spi_init(void);
void sf_waiting(void); //called when waiting

void sf_tlongdelay(void); //min 400us
void sf_tshortdelay(void);

#if SF_MAXFLASHCHIPS!=1
	extern unsigned char sf_select;
	#define sf_select_read() (sf_select)
	#define sf_select_write(x) do { sf_select=(x); } while (0)
#else
	#define sf_select_read() (0)
	#define sf_select_write(x) do { } while (0)
#endif

void sf_enable(void);
void sf_disable(void);
unsigned char sf_rw(unsigned char data);
unsigned char sf_r(void);
void sf_startstreamread(void);
unsigned char sf_endstreamread(void);

void sm_enable(void);
void sm_disable(void);
unsigned char sm_rw(unsigned char data);



#endif
