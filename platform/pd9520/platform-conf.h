#ifndef __PLATFORM_CONF_H__
#define __PLATFORM_CONF_H__
#include "stdint.h"
#include "sha256.h"
#include "uECC.h"


#define F_CPU 300000000
#define M_CPU 150000000


#define XMEM_ERASE_UNIT_SIZE (64 * 1024L)
#define CFS_XMEM_CONF_OFFSET    (2 * XMEM_ERASE_UNIT_SIZE)
#define CFS_XMEM_CONF_SIZE      (1 * XMEM_ERASE_UNIT_SIZE)

// ID_TC0 is used by flash
#define FLASH_TIMER_ID	ID_TC0

#define RGB_TIMER 		TC1->TC_CHANNEL[1]
#define RGB_TIMER_ID 	ID_TC4
#define RGB_TIMER_IRQ	TC4_IRQn

#define RGB2_TIMER 		TC1->TC_CHANNEL[0]
#define RGB2_TIMER_ID 	ID_TC3
#define RGB2_TIMER_IRQ	TC3_IRQn

// NB: move so it's the same as RGB - can't have both.
#define step_TIMER 		TC1->TC_CHANNEL[1]
#define step_TIMER_ID 	ID_TC4
#define step_TIMER_IRQ	TC4_IRQn

/*#define RngTimer 		TC0->TC_CHANNEL[1]
#define RngTimerID 		ID_TC1
#define RngTimerIRQ		TC1_IRQn*/

#define BOOTFLAGS		GPBR->SYS_GPBR[0]
#define CLOCK_FLAGS		GPBR->SYS_GPBR[1]

typedef struct {
		uint8_t stranum;
		uint8_t RTC_valid;
		uint16_t timezone;
}clock_gpbr_t;


//PA6 	btn			-	step1	-	relay1
//PA8 	RGB (r)		-	step2	- 	relay2
//PA9 	RGB (g)		-	step3	-	relay3
//PA10 	RGB (b)		-	step4	-	relay4
//PB6 	I2C (sda)
//PB7 	I2C (sdl)
//PB2 	lm73 (al) 	- 	dht11	I2C_debug (sdl)
//PB3							I2C_debug (sda)

#include "dev/eeprom.h"
typedef struct{
    double a;     ///< The slope of the reading versus ADC input
    double b;     ///< The offset of the reading
} tCalFactors;

typedef struct {
	unsigned char public_key[64];
	union {
	   struct {
			uint16_t typeBE;
			uint8_t snlen;
			unsigned char snr[20];
			unsigned char modul[9];
	   };
	   unsigned char payloadfield_size_control[32];
	};
	unsigned char signature[64];
}crt_t;

typedef struct {
	 unsigned char private_key[32];
	 unsigned char masterpublic_key[64];
	 crt_t crt; //public cert
} devicecert_t;

extern volatile devicecert_t device_certificate;

typedef struct {
    uint8_t eepromMacAddress[8];//8      ///< The node's unique IEEE MAC address
    tCalFactors calFactors; //32     ///< The node's calibration data
    uint16_t PANID; //40
    uint8_t channel; //48
    uint8_t version;	//56
    uint32_t Flash_unique_id[4];//72
    uint8_t masterpublic_key[64]; //136
    uint16_t timezone;
} tEepromContents; // Max 512 bytes


#define  get_eeprom(x,b)    	eeprom_read(offsetof(tEepromContents, x), \
                                       (uint8_t*)&b, \
										sizeof(typeof(((tEepromContents*)0)->x)))

#define  set_eeprom(x,b)   		eeprom_write(offsetof(tEepromContents, x), \
										(uint8_t*)&b, \
										sizeof(typeof(((tEepromContents*)0)->x)))


typedef struct { //cal for 1 analog 20mA IO
	float adc_multiplier_amps;
	float adc_zerooffset_amps;
} anaio_cal_20mA_2_t; //8 byte


typedef struct{
	float PSU_read_mul;
	float PSU_read_zero;
	float PSU_set_mul;
	float PSU_set_zero;
	float Analog_curr_in_mul;
	float Analog_curr_in_zero;
	float Analog_curr_out_mul;
	float Analog_curr_out_zero;
	float Analog_volt_set_mul;
	float Analog_volt_set_zero;
	float Analog_volt_read_mul;
	float Analog_volt_read_zero;
	float Analog_volt_current_read_mul;
	float Analog_volt_current_read_zero;
	float Digital_curr_read_mul;
	float Digital_curr_read_zero;
	float Digital_curr_set_mul;
	float Digital_curr_set_zero;
	float DUT_ana_mul;
	float DUT_ana_zero;
	float Commenmode_error_voltage_current;
}multiio_cal_t;

typedef struct { //cal for modulet selv
	char snr[24];
	char hwguid[24];
	unsigned s0,s1,s2,s3;
	char sig[128];
} module_cal_1_t; //192

typedef struct {
	float clocktrim;
	float supply_ad_to_volt;
	float tdiode_tref;
	float tdiode_vref;
} device_cal_data_t;

typedef struct { //cal for alle IO
	unsigned cpu_wdtword, cpu_resvd1, cpu_resvd2, cpu_resvd3; //reserve 4 words at start, cpu uses first.
	union {
		struct {
			unsigned i_version;
			module_cal_1_t     module; //192 byte
			multiio_cal_t multiio_cal;
			device_cal_data_t	device_calib;//16*1	=  16 bytes;
			anaio_cal_20mA_2_t ana[1]; //8 byte
			anaio_cal_20mA_2_t	light[2];

			unsigned char macaddr[6];
			unsigned char dummy[2];
			unsigned tag;
			unsigned checksumfix;
		};
		unsigned ul[128-4];
	};
} internal_cpu_userpage_9520_1_t;

extern volatile internal_cpu_userpage_9520_1_t hwio_cal_userpage_internal;

#endif /* __PLATFORM_CONF_H__ */
