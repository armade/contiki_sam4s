#ifndef Soft_rgb_H_
#define Soft_rgb_H_
/*---------------------------------------------------------------------------*/
int configure_RGB(void);
//int value_RGB(uint8_t R,uint8_t G,uint8_t B);
/*---------------------------------------------------------------------------*/
typedef struct{
	uint16_t r;
	uint16_t g;
	uint16_t b;
	uint16_t brightness;
}leds_t;
typedef union{
	leds_t led;
	uint64_t all;
}RGB_t;
/*---------------------------------------------------------------------------*/
extern const struct sensors_sensor soft_RGB_ctrl_sensor;
/*---------------------------------------------------------------------------*/
#endif
