#ifndef hard_rgb_H_
#define hard_rgb_H_
/*---------------------------------------------------------------------------*/
int configure_RGB(void);
//int value_RGB(uint8_t R,uint8_t G,uint8_t B);
/*---------------------------------------------------------------------------*/
typedef struct{
	uint16_t r;
	uint16_t g;
	uint16_t b;
	uint16_t brightness;
}leds_hard_t;
typedef union{
	leds_hard_t led;
	uint64_t all;
}RGB_hard_t;
/*---------------------------------------------------------------------------*/
extern const struct sensors_sensor hard_RGB_sensor;
/*---------------------------------------------------------------------------*/
#endif
