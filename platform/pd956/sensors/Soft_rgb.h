#ifndef Soft_rgb_H_
#define Soft_rgb_H_
/*---------------------------------------------------------------------------*/
int configure_RGB(void);
//int value_RGB(uint8_t R,uint8_t G,uint8_t B);
/*---------------------------------------------------------------------------*/
typedef struct{
	uint8_t r;
	uint8_t g;
	uint8_t b;
	uint8_t brightness;
}leds_t;
typedef union{
	leds_t led;
	uint32_t all;
}RGB_t;
/*---------------------------------------------------------------------------*/
extern const struct sensors_sensor soft_RGB_ctrl_sensor;
/*---------------------------------------------------------------------------*/
#endif
