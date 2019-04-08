/*
 * hwcontrol_9520.h
 *
 *  Created on: Feb 11, 2016
 *      Author: Kasper
 */

#ifndef HWCONTROL_9520_H_
#define HWCONTROL_9520_H_


//disse er lowlevel uden indbygget beskyttelse

void spi1_init(void);
void spi1_delay(void);
void spi1_unselect(void);
void spi1_select_adc(void);
void spi1_dac24(unsigned d);
void spi1_muxmask(unsigned d);
unsigned spi1_adcx_int(unsigned d, unsigned n);

void spi3_init(void);
void spi3_delay(void);
void spi3_unselect(void);
void spi3_select_ethsw(void);
void spi3_select_adc(void);
void spi3_dac24(unsigned d);
unsigned spi3_adcx_int(unsigned d, unsigned n);
unsigned spi3_switch_int(unsigned d);


/////// alle herunder er med intern mutex beskyttelse

void lowdac_cmd(unsigned cmd);
void lowdac_set(unsigned chan, unsigned val);
void isodac_cmd(unsigned cmd);
void isodac_set(unsigned chan, unsigned val);

void psu_off(void);
void psu_on_dac(unsigned outval);

void gen20mA_off(void);
void gen20mA_on_dac(unsigned dac);

void spgen_set_dac(unsigned dac);

void highside_set_dac(int iosignal, int dac);
void highside_off(void);

void muxadc_write(unsigned rs, unsigned w, int bit);
unsigned muxadc_read(unsigned rs, int bit);
unsigned muxadc_status(void);
unsigned muxadc_datau(void);
signed muxadc_datas(void); 

void isoadc_write(unsigned rs, unsigned w, int bit);
unsigned isoadc_read(unsigned rs, int bit);
unsigned isoadc_status(void);
unsigned isoadc_datau(void);
signed isoadc_datas(void);

void ethsw_writeb(unsigned a, unsigned b);
unsigned ethsw_readb(unsigned a);
void ethsw_write(unsigned a, unsigned char *p, unsigned n);
void ethsw_read(unsigned char *p, unsigned a, unsigned n);
int ethsw_checkalive(void);

//void multiplexer_a_select_int(int iosignal); //brug ikke denne til andet end service/selvtest - kig i koden. FARLIG!
void multiplexer_a_select(int iosignal); //1..22  0=off
void multiplexer_a_off(void);
void multiplexer_a_select_gnd(void);

//void multiplexer_b_select_int(int iosignal); //brug ikke denne til andet end service/selvtest - kig i koden. FARLIG!
void multiplexer_b_select(int iosignal); //1..8  0=off
void multiplexer_b_off(void);

void multiplexer_b_select_fastin(void);
void multiplexer_b_select_spgen(void);
void multiplexer_b_select_pulsgen(void);
void multiplexer_b_select_gnd(void);
void multiplexer_b_select_psu_hard(void);
void multiplexer_b_select_psu_6k81(void);
void multiplexer_b_select_gnd_6k81(void);


void set_test_led(unsigned c);

void init_9520(int mck);


// PM - translation
float Get_PSU_voltage_V(void);
void Set_PSU_voltage_V(float voltage_V);
void Route_io_to_analog_measurement(int io, int calib);
int Get_analog_current_A(void);
void Set_analog_current_A(int current);
int Route_io_to_voltage_generator(int io, int calib);
int Route_io_to_voltage_generator_through_mux_b(int io, int calib);
int Get_digital_current(void);
void Set_analog_voltage_v(int voltage);
int Get_analog_voltage(void);
int Get_current_in_voltage_generator(void);
int Set_digital_current(int current);
int Route_io_to_digital_current_measure(int io, int calib);
float Get_DUT_current_consumtion_A(void);
int Route_io_to_fast_in(int io, int calib);

int Selfcalibration_measure_isence(void);
int Selfcalibration_measure_voltage(void);
int Selfcalibration_measure_current(void);
int Selfcalibration_PSU_on_io(int io);
int Selfcalibration_common_mode(void);
int route_spgen_to_current_in(void);
int Selfcalibration_measure_own_supply_voltage(void);

int Set_tx_light_intensity(int tx1, int tx2, int x3PNET);

unsigned Read_RTC(unsigned char reg);
void Write_RTC(unsigned char reg, unsigned char val);
float Get_temperature_RTC(void);
float Get_temperature_lm73(void);

/*
typedef struct {
	// 0x00
	unsigned seconds : 4;
	unsigned seconds10 :3;
	unsigned dummy1 :1;
	// 0x01
	unsigned minutes :4;
	unsigned minutes10 : 3;
	unsigned dummy2 : 1;
	//0x02
	unsigned hour : 4;
	unsigned hour10 : 1;
	unsigned hour20_am_pm : 1;
	unsigned clock_format : 1;
	unsigned dummy3 : 1;
	//0x03
	unsigned wday : 3;
	unsigned dummy4 : 5;
	//0x04
	unsigned date : 4;
	unsigned date10 : 2;
	unsigned dymmy5 : 2;
	//0x05
	unsigned month : 4;
	unsigned month10 : 1;
	unsigned dummy6 : 2;
	unsigned century : 1; 	// NB: only information in datasheet is that this is toggled when year goes from 99 to 00.
							// The rtc is only valid until 2100, and the default value is 0. So i use this bit to
							// validate time. My definition: if century=1 means 2000-2099 and that the rct is valid.
	//0x07
	unsigned year : 4;
	unsigned year10 : 4;
	
} RTC_time_t;

struct s_tm {
	int tm_sec;//seconds after the minute: 0 - 59
	int tm_min;//minutes after the hour: 0 - 59
	int tm_hour;//hours since midnight: 0 - 23
	int tm_mday;//day of the month: 1 - 31
	int tm_mon;//months since January: 0 - 11
	int tm_year;//years since 1900
	int tm_wday;//days since Sunday: 0 - 6
	int tm_yday;//days since Jan 1st: 0 - 365
	//int tm_isdst;//daylight savings flag: -1 unknown, 0 not in DST, 1 in DST
};

unsigned char weekday[8][4]={
	"inv\0",
	"mon\0",
	"tue\0",
	"wed\0",
	"thu\0",
	"fri\0",
	"sat\0",
	"sun\0"
};
*/
/*unsigned long long rtctime_offset(void);
void external_RTC_get_time(struct s_tm *ptr);
void external_RTC_set_time(struct s_tm *tvrtc);
void internal_RTC_get_time(struct s_tm *tvrtc);
void internal_RTC_set_time(struct s_tm *tvrtc);*/

typedef struct {
	int isogen,gen20ma,spgen,psu,ethsw,cpld;//result
	int isogen_data[13];
	int gen20mA_data[8]; //KKP 20160222
	short int spgen_data1[32];
	int spgen_data2[6];
	short int psu_data1[33];
	short int psu_data2[33];
} selftestdata_t;

extern selftestdata_t selftestdata;

extern unsigned Current_DAC_position;


int isogen_selftest(void);
int gen20ma_selftest(void);
int spgen_selftest(void);
int psu_selftest(void);

int selftest_all(void);

#endif /* HWCONTROL_9520_H_ */
