CONTIKI_PROJECT = hello-world
all: $(CONTIKI_PROJECT)


ifeq ($(UUID),)
 UUID = 12345678PD
endif

CFLAGS += -D NODE_GPS=1 -D NODE_ROUTER=1
APPS += mqtt servreg-hack ntpd gpsd ftp

PROJECT_SOURCEFILES += httpd-simple.c slip-net.c
PROJECT_SOURCEFILES += httpd_post_handlers.c
CONTIKI_WITH_IPV6 = 1
CONTIKI = ../../..
include $(CONTIKI)/Makefile.include
