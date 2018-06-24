#ifndef lm73_h
#define lm73_h

#define boot

#define LM73_ADDRESS 	0b1001101
#define LM73			(1<<2)

void lm73_set_resolution(uint16_t res);
uint16_t lm73_get_resolution(void);
void lm73_set_mode_power_down(void);
void lm73_oneshot(void);

void lm73_T_high_set(uint16_t temp);
uint16_t lm73_T_high_get(void);

/*---------------------------------------------------------------------------*/
#define LM73_SENSOR_TYPE_POWERDOWN    			1
#define LM73_SENSOR_TYPE_CONTINUESLY    		2
#define LM73_SENSOR_TYPE_CONTINUESLY_alarm    	3
/*---------------------------------------------------------------------------*/
extern const struct sensors_sensor LM73_sensor;
/*---------------------------------------------------------------------------*/

#endif
