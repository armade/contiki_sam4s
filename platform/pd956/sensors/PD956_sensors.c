#include "contiki.h"
#include "board-peripherals.h"
#include "lib/sensors.h"

//#include <string.h>



/** \brief Exports a global symbol to be used by the sensor API */

#ifdef NODE_LIGHT
SENSORS(
		&SAM4S_ADC_TS_sensor,
		&button_sensor_PD956,
		&RGB_sensor
);
#endif

#ifdef NODE_HARD_LIGHT
SENSORS(
		&SAM4S_ADC_TS_sensor,
		&hard_RGB_sensor
);
#endif

#ifdef NODE_STEP_MOTOR
SENSORS(
		&SAM4S_ADC_TS_sensor,
		&dht11_sensor,
		&step_sensor
);
#endif

#ifdef NODE_4_ch_relay
SENSORS(
		&SAM4S_ADC_TS_sensor,
		&dht11_sensor,
		&ch4_relay_PD956
);
#endif

#ifdef NODE_PRESSURE
SENSORS(
		&SAM4S_ADC_TS_sensor,
		&bmp_280_sensor
);
#endif

#ifdef NODE_HTU21D
SENSORS(
		&SAM4S_ADC_TS_sensor,
		&HTU21D_sensor
);
#endif
