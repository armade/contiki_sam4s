DEFINES+=PROJECT_CONF_H=\"project-conf.h\"
CFLAGS += -D NODE_PIR_SR501=1  -DLOW_CLOCK=1
#define NODE_4_ch_relay
#define NODE_1_ch_relay
#define NODE_LIGHT
#define NODE_HARD_LIGHT
#define NODE_STEP_MOTOR
#define NODE_BMP280
#define NODE_HTU21D
#define NODE_GPS
#define NODE_DHT11
#define NODE_PIR_SR501


# dedicated i2c board 20346522


all: pd956_sensor_low_power

ifeq ($(UUID),)
 UUID = 12345678PD
endif

PROJECT_SOURCEFILES += mqtt-client.c
PROJECT_SOURCEFILES += httpd-simple.c
PROJECT_SOURCEFILES += udp-stream.c
PROJECT_SOURCEFILES += httpd_post_handlers.c

CONTIKI_WITH_IPV6 = 1

APPS += mqtt servreg-hack ntpd gpsd ftp

CONTIKI=../../..
include $(CONTIKI)/Makefile.include
