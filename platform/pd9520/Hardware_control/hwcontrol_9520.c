/*
 * hwcontrol_9520.c
 *
 * TODO for COPP:
 *  Definer ENTER/EXIT
 *  Definer delay_microseconds(unsigned us) og delay_nanoseconds(unsigned ns)
 *
 *
 *
 *
 */

/*

Serietermineringer i USB fjernes (det virker ikke med dem i)

spgen-spændingsdeleren har en 10k i bunden i den ene side.
Det skal være 10k/10k til de 47 ohm, og 1k/1k til stel.
Ellers virker ADC ikke.

Lightlink sender opamp har inputben vendt forkert for begge kanaler.
Det skal vendes rigtigt for at det virker.

TIC kondensator skal være 4.7nF NP0, ikke 1nF

Fremtidig: buffer opamp LMC6482 på TIMING signal, værdi...
Isrc er 10mA. Med 1n er det 10V/us.
Da Vmax er ca. 2V, fullscale tid 1us, skal den vaere 5nF. 4.7nF.
Ileak 1uA i 4.7nF er 200V/s, 0.2mV/us
det er ok..



Floating Generator, fremtidig version:
Det er kun lige at opampen i float generatoren kan drive fetten.
Tilføj en -5V chargepump, og forsyn opampens negative side derfra.
Check hvad der sker mht. adc input!!

MUX, fremtidig version:
MUX23 signalet, lige ved siden af I/O22 signalet, bindes ikke til GND,
men i stedet til SPGEN.
Det vil tillade selvtest af FASTIN uden at signalet også ses på et I/O.


*/

#include "chip.h"
//#include "at25flash.h"
#include "hwcontrol_9520.h"
#include "SPI1.h"
#define COPP


	extern void critical_leave(unsigned old);
	extern unsigned critical_enter(void);
 #define ENTER unsigned saveflag = critical_enter();
 #define EXIT critical_leave(saveflag);

//#define delay_microseconds(us) PD_1007_Sleep(us)
/*void delay_microseconds(unsigned us)
{
	unsigned dt,r;
	RESET_CYCLE_COUNTER();
	r = us*300;
	do{
		if(us>1){
			PD_1007_Sleep(us-1); // Be nice to the system
			us=0;
		}
		GET_CYCLE_COUNTER(dt);
	}while (dt < r);
}*/
void delay_microseconds(unsigned us)
{
	PD_1007_Sleep(us);
}

/*void delay_nanoseconds(unsigned ns)
{
	unsigned dt,r;
	RESET_CYCLE_COUNTER();
	r = ns/3; // The MCU has hw integer div. (1 clk) - but the result is a increase in delay by 33.3% for not using floating point
	do{
		GET_CYCLE_COUNTER(dt);
	}while (dt < r);
	
}*/
void delay_nanoseconds(unsigned ns)
{
	unsigned time;
	
	time = ns/1000;
	
	if(time == 0)
		time = 1;
	
	PD_1007_Sleep(time);
}
 
#endif

				   

const Pin psu_enable_pin={ 1<<31 , PIOC, ID_PIOC, PIO_OUTPUT_0 , PIO_DEFAULT};
const Pin curgen20_disable_pin={ 1<<4 , PIOB, ID_PIOB, PIO_OUTPUT_1 , PIO_DEFAULT};
const
Pin tcxo_pins[ 3]= {  { 1<<3  , PIOA, ID_PIOA, PIO_PERIPH_A , PIO_DEFAULT}, //TWD0
				      { 1<<4  , PIOA, ID_PIOA, PIO_PERIPH_A , PIO_DEFAULT}, //TWCK0
				      { 1<<5  , PIOC, ID_PIOC, PIO_PERIPH_B , PIO_DEFAULT}}; //PPS TIOA1
const
Pin pulsgen_pin= { 1<<4  , PIOE, ID_PIOE, PIO_PERIPH_B , PIO_DEFAULT};

const
Pin multiplexer_pins[11]= {  { 1<<16 , PIOC, ID_PIOC, PIO_OUTPUT_0 , PIO_DEFAULT}, //enable 1
						     { 1<<15 , PIOC, ID_PIOC, PIO_OUTPUT_0 , PIO_DEFAULT}, //enable 2
						     { 1<<14 , PIOC, ID_PIOC, PIO_OUTPUT_0 , PIO_DEFAULT}, //enable 3
						     { 1<<13 , PIOC, ID_PIOC, PIO_OUTPUT_0 , PIO_DEFAULT}, //enable 4
						     { 1<<12 , PIOC, ID_PIOC, PIO_OUTPUT_0 , PIO_DEFAULT}, //enable 5
						     { 1<<22 , PIOC, ID_PIOC, PIO_OUTPUT_0 , PIO_DEFAULT}, //a0
						     { 1<<21 , PIOC, ID_PIOC, PIO_OUTPUT_0 , PIO_DEFAULT}, //a1
						     { 1<<20 , PIOC, ID_PIOC, PIO_OUTPUT_0 , PIO_DEFAULT}, //a2
						     { 1<<19 , PIOC, ID_PIOC, PIO_OUTPUT_0 , PIO_DEFAULT}, //a3
						     { 1<<18 , PIOC, ID_PIOC, PIO_OUTPUT_0 , PIO_DEFAULT},  //a4
						     { 1<<17 , PIOC, ID_PIOC, PIO_OUTPUT_0 , PIO_DEFAULT} }; //a5
const
Pin programmer_pins[6]= { { 1<<0  , PIOC, ID_PIOC, PIO_INPUT , PIO_PULLUP},
                          { 1<<1  , PIOC, ID_PIOC, PIO_INPUT , PIO_PULLUP},
                          { 1<<2  , PIOC, ID_PIOC, PIO_INPUT , PIO_PULLUP},
                          { 1<<3  , PIOC, ID_PIOC, PIO_INPUT , PIO_PULLUP},
                          { 1<<4  , PIOC, ID_PIOC, PIO_INPUT , PIO_PULLUP},
                          { 1<<19 , PIOD, ID_PIOD, PIO_OUTPUT_0 , PIO_DEFAULT} };



#define SPI3_MISO_BIT 23
#define SPI3_MOSI_BIT 24
#define SPI3_SCK_BIT 25
#define SPI3_CS1_BIT 26
#define SPI3_CS2_BIT 28
#define SPI3_CS3_BIT 29
#define SPI3PORT PIOD
static Pin spi3[6]={ { 1<<SPI3_MISO_BIT , PIOD, ID_PIOD, PIO_INPUT   , PIO_DEFAULT},
                     { 1<<SPI3_MOSI_BIT , PIOD, ID_PIOD, PIO_OUTPUT_1, PIO_DEFAULT},
                     { 1<<SPI3_SCK_BIT , PIOD, ID_PIOD, PIO_OUTPUT_1, PIO_DEFAULT},
                     { 1<<SPI3_CS1_BIT , PIOD, ID_PIOD, PIO_OUTPUT_1, PIO_DEFAULT},
                     { 1<<SPI3_CS2_BIT , PIOD, ID_PIOD, PIO_OUTPUT_1, PIO_DEFAULT},
                     { 1<<SPI3_CS3_BIT , PIOD, ID_PIOD, PIO_OUTPUT_1, PIO_DEFAULT} };


//////////////////////////////////////////////////////////////////////////
/////////   SPI 1 lowlevel  //////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////

static unsigned char spi1_selected;
volatile unsigned spi1_currentmuxmask;



void spi1_delay(void)
{
	delay_nanoseconds(1000);
}

void spi1_unselect(void)
{
	if (spi1_selected) {
		SPI1_select(none);
		spi1_selected=0;
		//spi1_delay(); //idle
	}
}

void spi1_select_adc(void) //ud paa falling edge, ind paa rising edge,
{
	//SPI1_select(none);
	SPI1_select(ADC);
	//spi1_delay();
	spi1_selected=4;
}

void spi1_dac24(unsigned d)
{
	//unsigned bit;
	unsigned char d_uint[3];
	
	d_uint[0] = d>>16;
	d_uint[1] = d>>8;
	d_uint[2] = d>>0;
	
	//SPI1_select(none);
	SPI1_select(DAC);
	spi1_selected=5;
	//spi1_delay();
	SPI1_write_buffer_wait(d_uint,3);
	/*for (bit=0; bit<24; ++bit) {  //observeret: 10MHz data rate = 15 bus cycles, det er 2x antal eksplicitte ops.
		//data setup
		if (d&0x800000)
			SPI1PORT->PIO_SODR = 1<<SPI1_MOSI_BIT;
		else
			SPI1PORT->PIO_CODR = 1<<SPI1_MOSI_BIT;
		d<<=1;
		spi1_delay();
		//falling edge
		SPI1PORT->PIO_CODR = 1<<SPI1_SCK_BIT; //4 busops = 27ns
		spi1_delay();
		//rising edge
		SPI1PORT->PIO_SODR = 1<<SPI1_SCK_BIT; //3 busops = 20ns
	}*/
	//spi1_delay();
	SPI1_select(none);
	spi1_selected=0;
	//spi1_delay();
}


void spi1_muxmask(unsigned d)
{
	//unsigned bit;
	unsigned char d_uint[3];
	spi1_currentmuxmask=d;
	//SPI1_select(none);
	SPI1_select(MUX);
	spi1_selected=6;
	//spi1_delay();
	
	d_uint[0] = d>>8;
	d_uint[1] = d>>0;
	
	SPI1_write_buffer_wait(d_uint,2);
	/*for (bit=0; bit<16; ++bit) {
		//data setup
		if (d&0x8000)
			SPI1PORT->PIO_SODR = 1<<SPI1_MOSI_BIT;
		else
			SPI1PORT->PIO_CODR = 1<<SPI1_MOSI_BIT;
		d<<=1;
		//falling edge
		SPI1PORT->PIO_CODR = 1<<SPI1_SCK_BIT;
		spi1_delay();
		//rising edge
		SPI1PORT->PIO_SODR = 1<<SPI1_SCK_BIT;
		spi1_delay();
	}*/
	SPI1_select(none);
	spi1_selected=0;
	//spi1_delay();
}

unsigned spi1_adcx_int(unsigned d, unsigned n)
{
	unsigned out=0;
	uint8_t result=0;
	int bits_signed = n;
	
	if(!(n == 32 || n == 24 || n == 16 || n == 8))
		return -1;

	do{
		out<<=8;
		SPI1_read_buffer_wait(&result, 1,	(d>>(bits_signed-8))&0xff);
		out |= result;
		bits_signed-=8;
	}while(bits_signed);
	
	
	/*while (n) {
		--n;
		if (d&(1<<n))
			SPI1PORT->PIO_SODR = 1<<SPI1_MOSI_BIT;
		else
			SPI1PORT->PIO_CODR = 1<<SPI1_MOSI_BIT;
		SPI1PORT->PIO_CODR = 1<<SPI1_SCK_BIT;
		spi1_delay();
		//sample before rising edge
		if (SPI1PORT->PIO_PDSR&(1<<SPI1_MISO_BIT)) out|=1<<n;
		SPI1PORT->PIO_SODR = 1<<SPI1_SCK_BIT;
		spi1_delay();
	}*/
	return out;
}

//////////////////////////////////////////////////////////////////////////
/////////   SPI 3 lowlevel  //////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////

static unsigned char spi3_selected;

void spi3_init(void)
{
	PIO_Configure( spi3, 6 ) ;
}

void spi3_delay(void) //min 100ns. Det er 15.
{
	int i;
	for (i=0; i<4; ++i) {
		SPI3PORT->PIO_SODR=0;
		SPI3PORT->PIO_SODR=0;
		SPI3PORT->PIO_SODR=0;
		SPI3PORT->PIO_SODR=0;
	}
}

void spi3_unselect(void)
{
	if (spi3_selected) {
		SPI3PORT->PIO_SODR = (1<<SPI3_CS1_BIT)|(1<<SPI3_CS2_BIT)|(1<<SPI3_CS3_BIT);
		spi3_selected=0;
		spi3_delay(); //idle
	}
}

void spi3_select_ethsw(void) //ud paa falling edge, ind paa rising edge
{
	spi3_unselect();
	SPI3PORT->PIO_CODR = 1<<SPI3_CS1_BIT;
	spi3_delay();
	spi3_selected=1;
}
void spi3_select_adc(void) //ud paa falling edge, ind paa rising edge,
{
	spi3_unselect();
	SPI3PORT->PIO_CODR = 1<<SPI3_CS2_BIT;
	spi3_delay();
	spi3_selected=2;
}


void spi3_dac24(unsigned d)
{
	unsigned bit;
	//dac er hurtig, men speciel: Data ind paa falling edge.
	//1 busoperation er 6.6ns.

	spi3_unselect();
	SPI3PORT->PIO_CODR = 1<<SPI3_CS3_BIT;
	spi3_selected=3;

	for (bit=0; bit<24; ++bit) {  //observeret: 10MHz data rate = 15 bus cycles, det er 2x antal eksplicitte ops.
		//data setup
		if (d&0x800000)
			SPI3PORT->PIO_SODR = 1<<SPI3_MOSI_BIT;
		else
			SPI3PORT->PIO_CODR = 1<<SPI3_MOSI_BIT;
		d<<=1;
		SPI3PORT->PIO_CODR=0;
		SPI3PORT->PIO_CODR=0;

		//falling edge
		SPI3PORT->PIO_CODR = 1<<SPI3_SCK_BIT; //4 busops = 27ns
		SPI3PORT->PIO_CODR=0;
		SPI3PORT->PIO_CODR=0;
		//rising edge
		SPI3PORT->PIO_SODR = 1<<SPI3_SCK_BIT; //3 busops = 20ns
	}
	SPI3PORT->PIO_CODR=0; //tail
	SPI3PORT->PIO_CODR=0;
	SPI3PORT->PIO_SODR = (1<<SPI3_CS1_BIT)|(1<<SPI3_CS2_BIT)|(1<<SPI3_CS3_BIT);
	SPI3PORT->PIO_CODR=0; //tail
	SPI3PORT->PIO_CODR=0;
	spi3_selected=0;
}

unsigned spi3_adcx_int(unsigned d, unsigned n)
{
	unsigned out=0;
	while (n) {
		--n;
		if (d&(1<<n))
			SPI3PORT->PIO_SODR = 1<<SPI3_MOSI_BIT;
		else
			SPI3PORT->PIO_CODR = 1<<SPI3_MOSI_BIT;
		SPI3PORT->PIO_CODR = 1<<SPI3_SCK_BIT;
		spi3_delay();
		//sample before rising edge
		if (SPI3PORT->PIO_PDSR&(1<<SPI3_MISO_BIT)) out|=1<<n;
		SPI3PORT->PIO_SODR = 1<<SPI3_SCK_BIT;
		spi3_delay();
	}
	return out;
}

unsigned spi3_ethsw_int(unsigned d)
{
	unsigned out=0;
	unsigned n=8;
	while (n) {
		--n;
		if (d&(1<<n))
			SPI3PORT->PIO_SODR = 1<<SPI3_MOSI_BIT;
		else
			SPI3PORT->PIO_CODR = 1<<SPI3_MOSI_BIT;
		SPI3PORT->PIO_CODR = 1<<SPI3_SCK_BIT;
		spi3_delay();
		//sample before rising edge
		if (SPI3PORT->PIO_PDSR&(1<<SPI3_MISO_BIT)) out|=1<<n;
		SPI3PORT->PIO_SODR = 1<<SPI3_SCK_BIT;
		spi3_delay();
	}
	return out;
}

//////////////////////////////////////////////////////////////////////////
/////////   abstraktioner   //////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////

void lowdac_cmd(unsigned cmd)
{
	ENTER
	spi3_dac24(cmd); //inkluderer chipselect
	EXIT
}

void lowdac_set(unsigned chan, unsigned val)
{
	ENTER
	spi3_dac24((3<<19) | ((chan&7)<<16) | (val&0xFFFF) ); //write and update - inkluderer chipselect
	EXIT
}

void psu_off(void)
{
	ENTER
	PIO_Clear(&psu_enable_pin);
	lowdac_set(3, 65535); //Vgen DAC max off
	EXIT
}

void psu_on_dac(unsigned outval) //slew rate pos = 60V/ms = 3,96A
{
	ENTER
	lowdac_set(3, 65535-outval); //vender om.
	PIO_Set(&psu_enable_pin);
	EXIT
}

void gen20mA_off(void)
{
	ENTER
	PIO_Set(&curgen20_disable_pin);
	lowdac_set(2, 0);
	EXIT
}

void gen20mA_on_dac(unsigned dac)
{
	ENTER
	PIO_Clear(&curgen20_disable_pin);
	lowdac_set(2, dac);
	EXIT
}
unsigned Current_DAC_position;
void spgen_set_dac(unsigned dac)
{
	ENTER
	Current_DAC_position = dac;
	lowdac_set(1, dac);
	EXIT
}

void fastin_dac(unsigned dac)
{
	ENTER
	lowdac_set(0, dac);
	EXIT
}

void muxadc_write(unsigned rs, unsigned w, int bit)
{
	ENTER
	spi3_select_adc();
	spi3_adcx_int(  rs<<3 ,8);
	spi3_adcx_int(w,bit);
	spi3_unselect();
	EXIT
}

unsigned muxadc_read(unsigned rs, int bit)
{
	unsigned r;
	ENTER
	spi3_select_adc();
	spi3_adcx_int(  (rs<<3) |0x40 ,8);
	r=spi3_adcx_int(0,bit);
	spi3_unselect();
	EXIT
	return r;
}

unsigned muxadc_status(void)
{
	return muxadc_read(0,8);
}

unsigned muxadc_datau(void)
{
	return muxadc_read(3,24);
}

signed muxadc_datas(void)
{
	return muxadc_read(3,24) - 0x800000;
}

void ethsw_writeb(unsigned a, unsigned b)
{
	ENTER
	spi3_select_ethsw();
	spi3_switch_int(2);
	spi3_switch_int(a);
	spi3_switch_int(b);
	spi3_unselect();
	EXIT
}

unsigned ethsw_readb(unsigned a)
{
	unsigned r;
	ENTER
	spi3_select_ethsw();
	spi3_ethsw_int(3);
	spi3_ethsw_int(a);
	r=spi3_ethsw_int(0);
	spi3_unselect();
	EXIT
	return r;
}

void ethsw_write(unsigned a, unsigned char *p, unsigned n)
{
	ENTER
	spi3_select_ethsw();
	spi3_ethsw_int(2);
	spi3_ethsw_int(a);
	while (n) {
		spi3_switch_int(*p++);
		--n;
	}
	spi3_unselect();
	EXIT
}

void ethsw_read(unsigned char *p, unsigned a, unsigned n)
{
	ENTER
	spi3_select_ethsw();
	spi3_ethsw_int(3);
	spi3_ethsw_int(a);
	while (n) {
		*p++=spi3_switch_int(0);
		--n;
	}
	spi3_unselect();
	EXIT
}

int ethsw_checkalive(void)
{
	return ethsw_readb(0)==0x88;
}

void isodac_cmd(unsigned cmd)
{
	ENTER
	spi1_dac24(cmd); //inkluderer chipselect
	EXIT
}

void isodac_set(unsigned chan, unsigned val)
{
	ENTER
	spi1_dac24((3<<19) | ((chan&7)<<16) | (val&0xFFFF) ); //write and update - inkluderer chipselect
	EXIT
}

void highside_set_dac(int iosignal, int dac) // +/- 65535 ved at kombinere to dacs
{
	unsigned m;
	if ((iosignal>=1)||(iosignal<=16)) {
		//connect generator - nr. 15 er intern selvtest
		m=1<<(iosignal-1);
		ENTER
		if (m!=spi1_currentmuxmask) spi1_muxmask(m);
		isodac_set(0, (65535+dac) >>1  ); //neg input paa opamp
		isodac_set(1, (65535-dac) >>1  ); //pos input paa opamp
		//-1    32767 32768
		// 0    32767 32767
		// 1    32768 32767
		// 2    32768 32766
		EXIT
		return;
	}
	//turn off generator
	ENTER
	isodac_set(0,     0); //neg input paa opamp
	isodac_set(1, 65535); //pos input paa opamp
	spi1_muxmask(0); //disconnect
	EXIT
}

void highside_off(void)
{
	highside_set_dac(16,-65000);
}

void isoadc_write(unsigned rs, unsigned w, int bit)
{
	ENTER
	spi1_select_adc();
	spi1_adcx_int(  rs<<3 ,8);
	spi1_adcx_int(w,bit);
	spi1_unselect();
	EXIT
}

unsigned isoadc_read(unsigned rs, int bit)
{
	unsigned r;
	ENTER
	spi1_select_adc();
	spi1_adcx_int(  (rs<<3) |0x40 ,8);
	r=spi1_adcx_int(0,bit);
	spi1_unselect();
	EXIT
	return r;
}

unsigned isoadc_status(void)
{
	return isoadc_read(0,8);
}

unsigned isoadc_datau(void)
{
	return isoadc_read(3,24);
}

signed isoadc_datas(void)
{
	return isoadc_read(3,24) - 0x800000;
}

//////////////////////////////////////////////////////////////////////////
///////////  multiplexer kontrol lowside /////////////////////////////////
//////////////////////////////////////////////////////////////////////////

static unsigned char multiplexer_a_current_selection;
static unsigned char multiplexer_b_current_selection;

static inline void myPIO_PutBit(const Pin *pin, unsigned b)
{
	b ?	(pin->pio->PIO_SODR = pin->mask) : (pin->pio->PIO_CODR = pin->mask);
}

void multiplexer_a_select_int(int iosignal) //den brede mux
{
	int i;

	//anti blafre logik
	if (iosignal==multiplexer_a_current_selection) return;
	ENTER
	multiplexer_a_current_selection=iosignal;

	//konverter for normale signaler
	i=iosignal-1;
	if (i>=15) ++i; //iosignal16 er skubbet 1 opad
	//og for specielle signaler
	if ((iosignal<1)||(iosignal>22)) i=24; //tristate for alle ugyldige vaerdier, undtagen:
	if (iosignal==108) i=15; //internal gnd 1
	if (iosignal==109) i=23; //internal gnd 2 - fremtidigt noget andet for bedre selvtest

	//disable
	PIO_Clear(multiplexer_pins+0);
	PIO_Clear(multiplexer_pins+1);
	PIO_Clear(multiplexer_pins+2);
	PIO_Get(multiplexer_pins+2); //sync
	delay_nanoseconds(350);
	//configure
	myPIO_PutBit(multiplexer_pins+5, i&1);
	myPIO_PutBit(multiplexer_pins+6, i&2);
	myPIO_PutBit(multiplexer_pins+7, i&4);
	PIO_Get(multiplexer_pins+2); //sync
	delay_nanoseconds(350);
	//reenable
	if (i<8) PIO_Set(multiplexer_pins+0);
	else
		if (i<16) PIO_Set(multiplexer_pins+1);
		else
			if (i<24) PIO_Set(multiplexer_pins+2);
	EXIT
}

void multiplexer_a_select(int iosignal) //den brede mux
{
	unsigned b = multiplexer_b_current_selection;
	if ((b==103)||(b==104)||(b==105))  //farligt signal er valgt på den anden multiplekser
		if (iosignal)                  //og dette bliver farligt..
			multiplexer_b_off();       //anti-braende-af.

	multiplexer_a_select_int(iosignal);
}

void multiplexer_a_off(void)
{
	multiplexer_a_select_int(0);
}

void multiplexer_a_select_gnd(void)
{
	multiplexer_a_select(108);
}

void multiplexer_b_select_int(int iosignal) //den smalle mux
{
	int i;

	//anti blafre logik
	if (iosignal==multiplexer_b_current_selection) return;

	ENTER
	multiplexer_b_current_selection=iosignal;

	//konverter for normale signaler
	i=iosignal-1;
	if ((iosignal<1)||(iosignal>8)) i=16; //tristate for alle ugyldige vaerdier, undtagen:
	if ((iosignal>=100)&&(iosignal<=107)) { //signalnumre 100..107 er interne generatorer
		i=iosignal-100;
		i+=8;
	}

	//disable
	PIO_Clear(multiplexer_pins+3);
	PIO_Clear(multiplexer_pins+4);
	PIO_Get(multiplexer_pins+4); //sync
	delay_nanoseconds(350);
	//configure
	myPIO_PutBit(multiplexer_pins+8, i&1);
	myPIO_PutBit(multiplexer_pins+9, i&2);
	myPIO_PutBit(multiplexer_pins+10, i&4);
	PIO_Get(multiplexer_pins+2); //sync
	delay_nanoseconds(350);
	//reenable
	if (i<8) PIO_Set(multiplexer_pins+4); //enable5: den med IO 1..8
	else
		if (i<16) PIO_Set(multiplexer_pins+3); //enable4: interne signaler
	EXIT
}


void multiplexer_ab_select_levelselftest_io1(void) //very special...
{
	int i;

	ENTER
	multiplexer_b_current_selection=80; //80 er et internt ID for denne mode.
	//special:
	//  konfigurer til signal 0 (fastinput) samtidig med IO1 på B mux
	i=0;
	//disable
	PIO_Clear(multiplexer_pins+3);
	PIO_Clear(multiplexer_pins+4);
	PIO_Get(multiplexer_pins+4); //sync
	delay_nanoseconds(350);
	//configure
	myPIO_PutBit(multiplexer_pins+8, i&1);
	myPIO_PutBit(multiplexer_pins+9, i&2);
	myPIO_PutBit(multiplexer_pins+10, i&4);
	PIO_Get(multiplexer_pins+2); //sync
	delay_nanoseconds(350);
	//reenable BEGGE multiplexere
	PIO_Set(multiplexer_pins+4); //enable5: den med IO 1..8
	PIO_Set(multiplexer_pins+3); //enable4: interne signaler
	EXIT
	//vi har nu SPGEN ud paa IO1, og FASTIN på COMMON
	multiplexer_a_select(1); // IO1 (SPGEN) på common.
}




void multiplexer_b_select(int iosignal) //den smalle mux
{
	if (iosignal>8) iosignal=0;
	multiplexer_b_select_int(iosignal);
}

void multiplexer_b_select_fastin(void)
{
	multiplexer_b_select_int(100);
}
void multiplexer_b_select_spgen(void)
{
	multiplexer_b_select_int(101);
}
void multiplexer_b_select_pulsgen(void)
{
	multiplexer_b_select_int(102);
}
void multiplexer_b_select_gnd(void)
{
	if (multiplexer_a_current_selection) multiplexer_a_off(); //anti-braende-af.
	multiplexer_b_select_int(103);
}
void multiplexer_b_select_psu_hard(void)
{
	if (multiplexer_a_current_selection) multiplexer_a_off(); //anti-braende-af.
	multiplexer_b_select_int(105);
}
void multiplexer_b_select_psu_6k81(void)
{
	multiplexer_b_select_int(106);
}
void multiplexer_b_select_gnd_6k81(void)
{
	multiplexer_b_select_int(107);
}

void multiplexer_b_off(void)
{
	multiplexer_b_select_int(0);
}



void init_9520(int mck)
{
    spi3_init();
    spi1_init();
	PIO_Configure( &psu_enable_pin, 1 ) ;
	MATRIX->CCFG_SYSIO |= 1<<4; //enable PB4, no TDI  //KKP 20160222
	PIO_Configure( &curgen20_disable_pin, 1 ) ;
	delay_microseconds(5);
	spi3_dac24(0x280001); //power on reset
	spi1_dac24(0x280001); //power on reset
	//medens dac'er arbejder kan vi lige resette adc'en
	spi3_select_adc();
	spi3_adcx_int(0xFFFFFFFFU,32); //reset
	spi3_unselect();
	spi1_select_adc();
	spi1_adcx_int(0xFFFFFFFFU,32); //reset
	spi1_unselect();
	//og tilbage til dac'erne
	delay_microseconds(100);
	spi3_dac24(0x20000F); //power down register, all channels active
	spi1_dac24(0x20000F); //power down register, all channels active
	delay_microseconds(100);
	spi3_dac24(0x380001); //reference on
	spi1_dac24(0x380001); //reference on
	delay_microseconds(400); //vi skal bruge mindst 500 til ADC'en, så her er god tid.
	lowdac_set(0, 13107); //comparator threshold 5V = 0,5V DAC = 0.2V
	lowdac_set(1, 20047); //Vstim DAC = 10V = 0.778V  =20047
	lowdac_set(2,     0); //Cur DAC = 0mA
	lowdac_set(3, 65536); //Vgen DAC max off
	isodac_set(0,     0); //neg input paa opamp
	isodac_set(1, 65535); //pos input paa opamp
	isodac_set(2, 32768); //unused
	isodac_set(3, 32768); //unused

	//multiplexer initialiseres til off
	PIO_Configure( multiplexer_pins, 11 ) ;
	multiplexer_a_current_selection=255;
	multiplexer_b_current_selection=255;
	//og den paa iso siden ogsaa
	spi1_muxmask(0);

	//GPS uart1 init, 9600N81
	PMC_EnablePeripheral(ID_UART1);
	NVIC_EnableIRQ(ID_UART1);
	UART1->UART_CR = UART_CR_RSTRX | UART_CR_RSTTX | UART_CR_RXDIS | UART_CR_TXDIS | UART_CR_RSTSTA;
	UART1->UART_IDR = 0xFFFFFFFF;
	UART1->UART_MR = UART_MR_FILTER_ENABLED | UART_MR_PAR_NO;
	UART1->UART_BRGR = ((mck+4800) / 9600) / 16;
	UART1->UART_CR = UART_CR_TXEN | UART_CR_RXEN;

	//TCXO, XIN32=PA7 TIOA6=PC5 - Dette giver en 2048Hz capture rate med 9155.273 counterticks/capture - og vi kan interrupte pr 2.
	PMC_EnablePeripheral(ID_TC2); //TIOA6
	TC2->TC_CHANNEL[0].TC_CCR =  TC_CCR_CLKDIS;
	TC2->TC_CHANNEL[0].TC_IDR =  0xFFFFFFFF;
	TC2->TC_CHANNEL[0].TC_SR;
	TC2->TC_CHANNEL[0].TC_CMR = 1 | TC_CMR_LDRA_RISING | TC_CMR_LDRB_RISING | TC_CMR_SBSMPLR_SIXTEENTH ; //E70:1=mck div 8 capture mode, TIOA6 rising /16
	TC2->TC_CHANNEL[0].TC_CCR = TC_CCR_CLKEN | TC_CCR_SWTRG;
	PIO_Configure( tcxo_pins, 3 );
	
	init_RTC();
	
/*	//pe4 TIMEROUT TIOB10 - virker ikke, skal fixes - og det skal ikke vaere her.
	PMC_EnablePeripheral(ID_TC3); //TIOA9..11
	TC3->TC_CHANNEL[1].TC_CCR =  TC_CCR_CLKDIS;
	TC3->TC_CHANNEL[1].TC_IDR =  0xFFFFFFFF;
	TC3->TC_CHANNEL[1].TC_SR;      //18.75MHz, 1875 for 10kHz
	TC3->TC_CHANNEL[1].TC_RC=1875;
	TC3->TC_CHANNEL[1].TC_RB=1875/2;
	TC3->TC_CHANNEL[1].TC_CMR = 1 | TC_CMR_WAVE | TC_CMR_WAVSEL_UP_RC | TC_CMR_BCPB_SET | TC_CMR_BCPC_CLEAR;
	TC3->TC_CHANNEL[1].TC_CCR = TC_CCR_CLKEN | TC_CCR_SWTRG;
	PIO_Configure( &pulsgen_pin, 1 ) ;*/


	//tilbage til ADC init. Der er gået 600 us.
	//Start referencen - brug intern AVCC maaling som init. 16ms maaletid.
	isoadc_write( 2, (2<<6) |8, 16 ); //internal ref no buf, null
	isoadc_write( 1, 0x2003, 16 ); //single conversion, 16Hz, 50Hz reject
	muxadc_write( 2, (2<<6) | (1<<4) |8, 16 ); //internal ref, buffered, null
	muxadc_write( 1, 0x2003, 16 ); //single conversion, 16Hz, 50Hz reject
	do {} while (isoadc_status()&128);
	do {} while (muxadc_status()&128);
	muxadc_datau();
	isoadc_datau();
	//referencen er startet, DAC'erne har fået spænding på deres referencekondensatorer,
	//og alt forventes at være faldet til ro.

	//highside i alm-off tilstand.
	highside_off();
	multiplexer_b_off();
	multiplexer_a_off();

	PIO_Configure( programmer_pins, 6 ) ;
}

selftestdata_t selftestdata;

static int i_selftest_iso_igen_read(unsigned ch)
{
	isoadc_write( 2, (2<<6) | ch, 16 ); //internal ref no buf, opamp voltage
	isoadc_write( 1, 0x2004, 16 ); //single conversion, fast
	do {PD_1007_Sleep(10);} while (isoadc_status()&128);
	return isoadc_datas();
}

int isogen_selftest(void)
{
	int u1,u2,u3,u4;

	////////////step 1: zero
	highside_off();
	delay_microseconds(10000);
	isoadc_write( 2, (2<<6) | (1<<4) |7|(1<<12), 16 ); //internal ref, buffered, avdd monitor, unipolar
	isoadc_write( 1, 0x2004, 16 ); //single conversion, fast
	do {PD_1007_Sleep(10);} while (isoadc_status()&128);
	u1=isoadc_datau() / 2390; //AVCC: 2389916 counts pr volt, u er millivolt VCC
	u2=i_selftest_iso_igen_read(2) /1247;   //1246913 cts pr volt, u er millivolt OpAmp
	u3=i_selftest_iso_igen_read(0) /716;   //716975 cts/A
	selftestdata.isogen_data[0]=u1;
	selftestdata.isogen_data[1]=u2;
	selftestdata.isogen_data[2]=u3;
	//test AVCC
	if (u1<4500) return -1;
	if (u1>5500) return -2;
	//opamp skal vaere hoej/disabled
	if (u2<3400) return -3; //
	if (u2>5500) return -4; //umulighed.
	//shunt skal vaere stroemloes
	if (u3<-2) return -5; //
	if (u3> 2) return -6; // eq. 20mA fejl

	/////////////step 2: 372mA og FET test
	highside_set_dac(16,20000); //nominelt 372 mA
	delay_microseconds(10000);
	isoadc_write( 2, (2<<6) | (1<<4) |7|(1<<12), 16 ); //internal ref, buffered, avdd monitor, unipolar
	isoadc_write( 1, 0x2004, 16 ); //single conversion, fast
	do {PD_1007_Sleep(10);} while (isoadc_status()&128);
	u1=isoadc_datau() / 2390; //AVCC: 2389916 counts pr volt, u er millivolt VCC
	u2=i_selftest_iso_igen_read(2) /1247;   //1246913 cts pr volt, u er millivolt OpAmp
	u3=i_selftest_iso_igen_read(0) /716;   //716975 cts/A
	spi1_muxmask(0);
	u4=i_selftest_iso_igen_read(0) /716;   //716975 cts/A
	highside_off();
	selftestdata.isogen_data[3]=u1;
	selftestdata.isogen_data[4]=u2;
	selftestdata.isogen_data[5]=u3;
	selftestdata.isogen_data[6]=u4;
	//test AVCC
	if (u1<4300) return -7;
	if (u1>5500) return -8;
	//opamp maa ikke vaere saturated
	if (u2<400) return -9; //
	//shunt skal vaere 372 mA +/- tolerancen, 10% for test, og 4mA mere for offset
	if (u3<330) return -10; //
	if (u3>413) return -11; //
	//test fet kontrol: med det slukket skal stroemmen falde (til 0 faktisk, men 50 er en ok testgraense)
	if (u4>50) return -12; //

	/////////////step 3: 1A
	highside_set_dac(16,53739); //nominelt 1A
	delay_microseconds(10000);
	isoadc_write( 2, (2<<6) | (1<<4) |7|(1<<12), 16 ); //internal ref, buffered, avdd monitor, unipolar
	isoadc_write( 1, 0x2004, 16 ); //single conversion, fast
	do {PD_1007_Sleep(10);} while (isoadc_status()&128);
	u1=isoadc_datau() / 2390; //AVCC: 2389916 counts pr volt, u er millivolt VCC
	u2=i_selftest_iso_igen_read(2) /1247;   //1246913 cts pr volt, u er millivolt OpAmp
	u3=i_selftest_iso_igen_read(0) /716;   //716975 cts/A
	highside_off();
	selftestdata.isogen_data[7]=u1;
	selftestdata.isogen_data[8]=u2;
	selftestdata.isogen_data[9]=u3;
	//test AVCC
	if (u1<4300) return -13;
	if (u1>5500) return -14;
	//opamp maa ikke vaere saturated
	if (u2<400) return -15; //saturated
	if (u2>3500) return -16; //umulig hoej gatespaending
	//shunt skal vaere 1000 mA +/- tolerancen, 10% for test
	if (u3<900) return -17; //
	if (u3>1100) return -18; //

	////////////step 4: recovery fra off
	highside_set_dac(16,-60000); 
	delay_microseconds(10000);
	highside_set_dac(16,20000); //nominelt 372 mA
	u1=i_selftest_iso_igen_read(0);   //716975 cts/A
	u2=i_selftest_iso_igen_read(0);   //716975 cts/A
	u3=i_selftest_iso_igen_read(0);   //716975 cts/A
	highside_off();
	selftestdata.isogen_data[10]=u1;
	selftestdata.isogen_data[11]=u2;
	selftestdata.isogen_data[12]=u3;

	return 0;
}

static int i_gen20mA_read(unsigned ch)
{
	muxadc_write( 2, (2<<6) | ch, 16 ); //internal ref no buf, opamp voltage
	muxadc_write( 1, 0x2009, 16 ); //single conversion, normal speed
	do {PD_1007_Sleep(10);} while (muxadc_status()&128);
	return muxadc_datas();
}

static int i_spgen_read(unsigned ch)
{
	muxadc_write( 2, (2<<6) | ch, 16 ); //internal ref no buf, opamp voltage
	muxadc_write( 1, 0x2004, 16 ); //single conversion, normal speed
	do {PD_1007_Sleep(10);} while (muxadc_status()&128);
	return muxadc_datas();
}

int gen20ma_selftest(void)
{
	signed s1,s2,s3,s4,s5,s6;

	highside_off();
	multiplexer_a_select(0); //none
	spgen_set_dac(0);
	delay_microseconds(1000);

	//multiplexer_b_select_spgen(); --kkp: nuvaerende spgen kan ikke altid drive 20mA
	//spgen_set_dac(16000);
	multiplexer_b_select_psu_hard();
	psu_on_dac(1966* 10); //33.33V span, 1966/V, ca. 10V

	//generator: 1.25V i 40.2 ohm er 31,1mA. 2108 counts per mA
	//adc: 29.1mA FS, 288223 counts per mA
	gen20mA_on_dac(2108 * 4);
	delay_microseconds(10000);
	s1=i_gen20mA_read(0)/288;
	gen20mA_on_dac(2108 * 14);
	delay_microseconds(10000);
	s2=i_gen20mA_read(0)/288;
	gen20mA_on_dac(2108 * 24);
	delay_microseconds(10000);
	s3=i_gen20mA_read(0)/288;
	gen20mA_off();
	delay_microseconds(10000);
	s4=i_gen20mA_read(0)/288;
	selftestdata.gen20mA_data[0]=s1;
	selftestdata.gen20mA_data[1]=s2;
	selftestdata.gen20mA_data[2]=s3;
	selftestdata.gen20mA_data[3]=s4;
	multiplexer_b_select(0);
	psu_off();

	//KKP 20160222
	multiplexer_b_select_spgen(); //obs - kan ikke altid drive 20mA.
	spgen_set_dac(2040* 1); //1V
	delay_microseconds(10000);
	s5=(i_gen20mA_read(0)*968)/279;
	selftestdata.gen20mA_data[6]=i_spgen_read(2)/227;

	spgen_set_dac(2040* 21); //22V
	delay_microseconds(10000);
	s6=(i_gen20mA_read(0)*968)/279;
	selftestdata.gen20mA_data[7]=i_spgen_read(2)/227;

	//med 20V delta skal man se 10uA.
	//s5,s6 er i nA
	selftestdata.gen20mA_data[4]=s5;
	selftestdata.gen20mA_data[5]=s6;
	multiplexer_b_select(0);

	if (s1<3000) return -1;
	if (s1>5000) return -2;
	if (s2<13000) return -3;
	if (s2>15000) return -4;
	if (s3<23000) return -5;
	if (s3>25000) return -6;
	if (s4<-500) return -7;
	if (s4>500) return -8;

	s6-=s5;
	if (s6<9000) return -9;
	if (s6>11000) return -10;

	return 0;
}


int spgen_selftest(void)
{
	int i,act,e,err;

	highside_off();
	gen20mA_off();
	multiplexer_a_select(0); //none
	delay_microseconds(1000);
	multiplexer_b_select_spgen();

	err=0;
	//ramp test
	//dac: 32.125 span, 2040 cts pr V
	//adc: 31.5 deler, 36.855 span, 227611 cts per volt
	for (i=0; i<32; ++i) {
		spgen_set_dac(2040*i);
		delay_microseconds(1000);
		act=i_spgen_read(2)/227; 
		selftestdata.spgen_data1[i]=act;
		e=act-1000*i;
		if (i<20) {
			if (e<-2000) ++err;
			if (e>2000) ++err;
		}
	}

	//load test. 47 ohm delt med 11, opfoerer sig som 4,7 ohm. Dvs 4,7mV/mA = 30634/mA
	spgen_set_dac(2040 * 8);
	delay_microseconds(10000);
	selftestdata.spgen_data2[0]=i_spgen_read(4)/30;
	for (i=1; i<4; ++i) {
		gen20mA_on_dac(2108 *i *10);     //2108 pr mA, 10mA step
		delay_microseconds(10000);
		selftestdata.spgen_data2[i]=i_spgen_read(4)/30;

	}
	gen20mA_off();
	//cmrr test
	spgen_set_dac(2040 * 4);
	delay_microseconds(10000);
	selftestdata.spgen_data2[4]=i_spgen_read(4)/30;
	spgen_set_dac(2040 * 24);
	delay_microseconds(10000);
	selftestdata.spgen_data2[5]=i_spgen_read(4)/30;

	//check stroem maaling.
	for (i=0; i<4; ++i) {
		e=selftestdata.spgen_data2[i]-10000*i;
		if (e<-5000) ++err;
		if (e>5000) ++err;
	}
	//CMRR test
	e=selftestdata.spgen_data2[4]-selftestdata.spgen_data2[5];
	if (e<-2000) ++err;
	if (e>2000) ++err;

	spgen_set_dac(0);
	multiplexer_b_select(0);
	if (err) return -1;
	return 0;
}

int psu_selftest(void)
{
	int i,e,err,act1,act2;

	highside_off();
	gen20mA_off();
	multiplexer_a_select(0); //none
	delay_microseconds(1000);
	multiplexer_b_select_psu_hard(); //PSU+ direkte

	psu_on_dac(0);  //24mA, 66uF, 360V/s slewrate
	gen20mA_on_dac(2108 * 24); //4mA load, 13 ohm on, 50mV fejl, det er fint
	delay_microseconds(100000);

	gen20mA_on_dac(2108 * 4); //4mA load, 13 ohm on, 50mV fejl, det er fint
	err=0;
	//adc for multiplexer scanner: 31.5 deler, 36.855 span, 227611 cts per volt
	//adc for PSU sp. maaling: 31 deler, 36,27 span, 231282 cts per volt

	for (i=0; i<32; ++i) {
		psu_on_dac(1966*i); //33.33V span, 1966/V
		delay_microseconds(10000);
		act1=i_spgen_read(2)/227; //maalt via mux
		act2=i_spgen_read(5)/231; //maalt via Vsense
		selftestdata.psu_data1[i]=act1;
		selftestdata.psu_data2[i]=act2;

		if (i<20) {
			e=act1-i*1000;
			if (e<0) e=-e;
			if (e>5000) ++err;

			e=act2-i*1000;
			if (e<0) e=-e;
			if (e>5000) ++err;
		}
	}
	psu_on_dac(0);
	//24mA, 66uF, 360V/s slewrate
	gen20mA_on_dac(2108 * 24);
	delay_microseconds(100000);
	gen20mA_on_dac(2108 * 4); //4mA load, 13 ohm on, 50mV fejl, det er fint
	selftestdata.psu_data1[32]=i_spgen_read(2)/227; //maalt via mux
	selftestdata.psu_data2[32]=i_spgen_read(5)/231; //maalt via Vsense
	gen20mA_off();

	psu_off();
	multiplexer_b_select(0);
	if (err) return -1;
	return 0;
}


int selftest_all(void)
{
	delay_microseconds(1000000); // We need 1 sec settling time before selftest.
	psu_off();
	multiplexer_b_select(0);
	multiplexer_a_select(0);
	selftestdata.isogen=isogen_selftest();
	selftestdata.gen20ma=gen20ma_selftest();
	selftestdata.spgen=spgen_selftest();
	selftestdata.psu=psu_selftest();
	selftestdata.ethsw=-!ethsw_checkalive(); //-1 ved fejl

	selftestdata.cpld=initcpld9520();

	if (selftestdata.isogen) return 0;
	if (selftestdata.gen20ma) return 0;
	if (selftestdata.spgen) return 0;
	if (selftestdata.psu) return 0;
	if (selftestdata.ethsw) return 0;
	return 1;
}



/*void print_state(void)
{
	print_loud("state: muxA: %d muxB: %d spi1_mux: %d\n",
	multiplexer_a_current_selection,
	multiplexer_b_current_selection,
	spi1_currentmuxmask);
}*/

////////////////////////////////////////////////////////////////////////

// x*1.17/2^23*31.1 = x*4.338E-6 
// Reading AIN6 on IC09
int Get_PSU_voltage_V(void)
{
	return i_gen20mA_read(5);
}

void Set_PSU_voltage_V(int voltage_V)
{
	if(voltage_V > 0)
		psu_on_dac(voltage_V); //33.33V span, 1966/V
	else{
		psu_off();
	}
}
////////////////////////////////////////////////////////////////////////

void Route_io_to_analog_measurement(int io, int calib)
{
	if(io < 1 || io > 22)
		return;
	
	multiplexer_b_off();
	
	if(!calib) {
		highside_off();
	
		ENTER
		spi1_muxmask(0); //disconnect
		EXIT
	}
	multiplexer_a_select(io);
}

// Reading AIN1 on IC09
int Get_analog_current_A(void)
{
	return i_gen20mA_read(0);
}

// SPI3 - DAC3
void Set_analog_current_A(int current)
{
	gen20mA_on_dac(current);
}
////////////////////////////////////////////////////////////////////////
int Route_io_to_voltage_generator(int io, int calib)
{
	if(!calib)
		highside_off();
	gen20mA_off();
	delay_microseconds(1000);
	multiplexer_b_select_spgen();
	multiplexer_a_select(io);
	
	return 0;
}

int Route_io_to_voltage_generator_through_mux_b(int io, int calib)
{
	if(io < 1 || io > 8)
		return 1;
	
	multiplexer_b_select(io);
	return 0;
}

void Set_analog_voltage_v(int voltage)
{
	spgen_set_dac(voltage);	
}

int Get_analog_voltage(void)
{
	return i_spgen_read(2);//227611.0;
}

int Get_current_in_voltage_generator(void)
{
	return i_spgen_read(4); // 1,17V/2^23 / 47.5ohm
}
////////////////////////////////////////////////////////////////////////
int Get_digital_current(void)
{
	return i_selftest_iso_igen_read(0);   //716975 cts/A
}

int Set_digital_current(int current)
{
	if(current > 65535 || current < 0)
		return 0;
	ENTER
	isodac_set(0, (65535+current) >>1  ); //neg input paa opamp
	isodac_set(1, (65535-current) >>1  ); //pos input paa opamp
	//-1    32767 32768
	// 0    32767 32767
	// 1    32768 32767
	// 2    32768 32766
	EXIT
	return 0;
}

int Route_io_to_digital_current_measure(int io, int calib)
{
	unsigned m;
	multiplexer_b_off();
	multiplexer_a_select(0);
	gen20mA_off();
	if ((io>=1)||(io<=15)) {
		//connect generator - nr. 15 er intern selvtest
		m=1<<(io-1);
		ENTER
		if (m!=spi1_currentmuxmask) spi1_muxmask(m);
		EXIT
		//print_state();
		return 0;
	}
	ENTER
	spi1_muxmask(0); //disconnect
	EXIT
	return 0;
}

// Reading AIN2 on IC09
int Get_DUT_current_consumtion_A(void)
{
	return i_gen20mA_read(1);
}

int Route_io_to_fast_in(int io, int calib)
{
	if(!calib)
		highside_off();
	gen20mA_off();
	delay_microseconds(1000);
	multiplexer_b_select_fastin();
	multiplexer_a_select(io);
	fastin_dac(1<<15);
	return 0;
}
/////////////////////////////////////////////////////////////////////

int Set_tx_light_intensity(int tx1, int tx2, int x3PNET)
{
	if(tx1 > 31)
		tx1 = 31;
	else if(tx1 < 0)
		tx1 = 0;
	
	if(tx2 > 31)
		tx2 = 31;
	else if(tx2 < 0)
		tx2 = 0;
	
	lightlink_setconfig(tx1,1,tx2,1,x3PNET);
	
	return 1;
}
//============================================================================================================
// Self Calibrating functions
#define ctl1	11
#define ctl2	10
#define IOA		1
#define IOB		2

int Selfcalibration_measure_isence(void)
{
	multiplexer_b_off();
	multiplexer_a_select(0);
	highside_off();
	ENTER
	spi1_muxmask(0); //disconnect the relays
	EXIT
	PD_1007_Sleep(100000); // 100ms settling time
	
	highside_set_dac(ctl1,250*53739); 
	return 0;
}

int Selfcalibration_measure_voltage(void)
{
	multiplexer_b_off();
	highside_off();
	ENTER
	spi1_muxmask(0); //disconnect the relays
	EXIT
	PD_1007_Sleep(100000); // 100ms settling time
	
	highside_set_dac(ctl2,0.250*53739);
	return 0;
}

int Selfcalibration_measure_current(void)
{
	multiplexer_b_off();
	
	ENTER
	spi1_muxmask(0); //disconnect the relays
	EXIT
	highside_off();
	return 0;
}

int Selfcalibration_PSU_on_io(int io)
{
	multiplexer_b_select_int(105);
	multiplexer_a_select_int(io);
	return 0;
}

int Selfcalibration_common_mode(void)
{
	int m1,m2;
	float tt = 0;
	unsigned i;
	
	multiplexer_b_off();

	for(i=0;i<10;i++)
	{
		spgen_set_dac(10000);
		PD_1007_Sleep(100000); // 100ms settling time
		m1=i_spgen_read(4);
		
		spgen_set_dac(50000);
		PD_1007_Sleep(100000); // 100ms settling time
		m2=i_spgen_read(4);
		
		tt += (m2-m1)/(50000.0-10000.0);
	}
	tt /= 10;
	//printk("test m1=%d,\tm2=%d,\tcc=%f\n",m1,m2,tt);
	
	return tt*1000000;
}

int route_spgen_to_current_in(void)
{
	multiplexer_a_select(0); // off
	multiplexer_b_select_spgen();
	return 0;
}

int Selfcalibration_measure_own_supply_voltage(void)
{
	multiplexer_b_off();
	highside_off();
	ENTER
	spi1_muxmask(0); //disconnect the relays
	EXIT
	PD_1007_Sleep(100000); // 100ms settling time
	
	highside_set_dac(ctl2,0.250*53739);
	multiplexer_b_select_int(102);
	multiplexer_a_select_int(2);
	return 0;
}

