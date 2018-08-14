DEFINES+=PROJECT_CONF_H=\"project-conf.h\"
CFLAGS += -DNODE_GPS=1  -DLOW_CLOCK=0

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
