DEFINES+=PROJECT_CONF_H=\"project-conf.h\"

all: pd956_sensor_low_power


PROJECT_SOURCEFILES += mqtt-client.c
PROJECT_SOURCEFILES += httpd-simple.c
PROJECT_SOURCEFILES += udp-stream.c
PROJECT_SOURCEFILES += httpd_post_handlers.c

CONTIKI_WITH_IPV6 = 1

# REST Engine shall use Erbium CoAP implementation
APPS += mqtt servreg-hack ntpd

CONTIKI=../../..
include $(CONTIKI)/Makefile.include
