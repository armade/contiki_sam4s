#include "contiki.h"
#include "board-peripherals.h"
#include "lib/sensors.h"

//#include <string.h>



/** \brief Exports a global symbol to be used by the sensor API */
SENSORS(
		&SAM4S_ADC_TS_sensor
#ifdef NODE_LIGHT
		,&soft_RGB_ctrl_sensor
#endif

#ifdef NODE_HARD_LIGHT
		,&hard_RGB_ctrl_sensor
#endif

#ifdef NODE_DHT11
		,&dht11_sensor
#endif

#ifdef NODE_STEP_MOTOR
		,&step_sensor
#endif

#ifdef NODE_4_ch_relay
		,&ch4_relay_PD956
#endif

#ifdef NODE_BMP280
		,&bmp_280_sensor
#endif

#ifdef NODE_HTU21D
		,&HTU21D_sensor
#endif

#ifdef NODE_GPS
		,&GPS_sensor
#endif

#ifdef NODE_ROUTER

#endif
);
