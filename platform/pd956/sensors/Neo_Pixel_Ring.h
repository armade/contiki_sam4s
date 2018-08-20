#ifndef Neo_Pixel_Ring_H_
#define Neo_Pixel_Ring_H_
/*---------------------------------------------------------------------------*/
typedef struct{
	uint8_t green;
	uint8_t red;
	uint8_t blue;
	uint8_t led_nr;
}Led_element_t;

static const Led_element_t sync_leds = {
		.green = 0,
		.red = 0,
		.blue = 0,
		.led_nr = 255,
};
/*---------------------------------------------------------------------------*/
extern const struct sensors_sensor Neo_Pixel_Ring_sensor;
/*---------------------------------------------------------------------------*/
#endif
