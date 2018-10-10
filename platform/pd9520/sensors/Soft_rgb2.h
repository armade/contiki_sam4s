#ifndef Soft_rgb2_H_
#define Soft_rgb2_H_

/*---------------------------------------------------------------------------*/
typedef struct{
	uint16_t r;
	uint16_t g;
	uint16_t b;
	uint16_t brightness;
}leds2_t;
typedef union{
	leds2_t led;
	uint64_t all;
}RGB2_soft_t;
/*---------------------------------------------------------------------------*/
extern const struct sensors_sensor soft_RGB2_ctrl_sensor;
/*---------------------------------------------------------------------------*/
#endif
