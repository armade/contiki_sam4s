CONTIKI_PROJECT = hello-world
all: $(CONTIKI_PROJECT)


ifeq ($(UUID),)
 UUID = 12345678PD
endif

CFLAGS += -D NODE_GPS=1
APPS += mqtt servreg-hack ntpd gpsd ftp weather

PROJECT_SOURCEFILES += httpd-simple.c
PROJECT_SOURCEFILES += httpd_post_handlers.c

CONTIKI = ../../..
include $(CONTIKI)/Makefile.include
