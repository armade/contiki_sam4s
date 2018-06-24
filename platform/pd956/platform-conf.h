#ifndef __PLATFORM_CONF_H__
#define __PLATFORM_CONF_H__
#include "stdint.h"

#ifndef LOW_CLOCK
#define LOW_CLOCK 1
#endif

#if LOW_CLOCK
/* The frequency of the main processor */
#define F_CPU 30000000 		//2.0mA max + 4mA ADC + 12-14mA radio ~ 18-20mA
#else
/* The frequency of the main processor */
#define F_CPU 120000000  	//6.9mA max  + 4mA ADC + 12-14mA radio ~ 25-27mA
#endif


// timer 0 is on timestamp dig2 (Not implemented yet) TIOA0

#define RGB_TIMER 		TC0->TC_CHANNEL[2]
#define RGB_TIMER_ID 	ID_TC2
#define RGB_TIMER_IRQ	TC2_IRQn

// NB: move so it's the same as RGB - can't have both.
#define step_TIMER 		TC0->TC_CHANNEL[2]
#define step_TIMER_ID 	ID_TC2
#define step_TIMER_IRQ	TC2_IRQn

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
    uint8_t eepromMacAddress[8];//8      ///< The node's unique IEEE MAC address
    tCalFactors calFactors; //32     ///< The node's calibration data
    uint16_t dataSeconds; //34            ///< The number of seconds between data readings
    uint8_t name[20]; //54
    uint16_t PANID; //56
    uint8_t channel; //57
    uint8_t version;	//58
    uint8_t PD_snr[20]; //78
    uint8_t User_friendly_name[30]; //108
} tEepromContents; // Max 512 bytes

#define  get_eeprom(x,b)    	eeprom_read(offsetof(tEepromContents, x), \
                                       (uint8_t*)&b, \
										sizeof(typeof(((tEepromContents*)0)->x)))

#define  set_eeprom(x,b)   		eeprom_write(offsetof(tEepromContents, x), \
										(uint8_t*)&b, \
										sizeof(typeof(((tEepromContents*)0)->x)))


#endif /* __PLATFORM_CONF_H__ */
