CONTIKI_PROJECT = hello-world
all: $(CONTIKI_PROJECT)

CFLAGS += -D NODE_GPS=1
APPS += mqtt servreg-hack ntpd gpsd ftp

PROJECT_SOURCEFILES += httpd-simple.c
PROJECT_SOURCEFILES += httpd_post_handlers.c

CONTIKI = ../../..
include $(CONTIKI)/Makefile.include
