#ifndef christmas_light_h
#define christmas_light_h

#define STATUS_STATE	1
#define STATUS_TIME		2

#define STATUS_CH1		10


#define ch1_STATUS_MIN		STATUS_CH1
#define ch1_STATUS_MAX		STATUS_CH1

#define CH1_RELAY_ON	14
#define CH1_RELAY_OFF	15


#define ch1_RELAY_MIN	CH1_RELAY_ON
#define ch1_RELAY_MAX	CH1_RELAY_OFF
/*---------------------------------------------------------------------------*/
extern const struct sensors_sensor christmas_light;
/*---------------------------------------------------------------------------*/

#endif
