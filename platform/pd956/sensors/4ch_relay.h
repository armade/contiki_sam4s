#ifndef relay4_sensor_h
#define relay4_sensor_h

#define STATUS_STATE	1
#define STATUS_TIME		2

#define STATUS_CH1		10
#define STATUS_CH2		11
#define STATUS_CH3		12
#define STATUS_CH4		13

#define STATUS_MIN		STATUS_CH1
#define STATUS_MAX		STATUS_CH4

#define CH1_RELAY_ON	14
#define CH1_RELAY_OFF	15
#define CH2_RELAY_ON	16
#define CH2_RELAY_OFF	17
#define CH3_RELAY_ON	18
#define CH3_RELAY_OFF	19
#define CH4_RELAY_ON	20
#define CH4_RELAY_OFF	21

#define CH_RELAY_MIN	CH1_RELAY_ON
#define CH_RELAY_MAX	CH4_RELAY_OFF
/*---------------------------------------------------------------------------*/
extern const struct sensors_sensor ch4_relay_PD956;
/*---------------------------------------------------------------------------*/

#endif
