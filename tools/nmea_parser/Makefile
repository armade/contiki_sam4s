CC=gcc
CFLAGS=-c -Wall -MD
LDFLAGS=

SOURCES=$(shell find . -maxdepth 1 -name "*.c") 
OBJECTS=$(SOURCES:.c=.o)
EXECUTABLE=nmea_tester.exe
OPTIMIZE = -O2 -g
DEPS := $(OBJECTS:.o=.d)


all: $(OBJECTS) $(EXECUTABLE)
-include $(DEPS)

$(EXECUTABLE): $(OBJECTS)
	@echo "ld $@" 
	@$(CC) $(LDFLAGS) $(OBJECTS) $(INCLUDE) $(OPTIMIZE) -o $@

%.o: %.c
	@echo "cc $@"
	@$(CC) $(CFLAGS) $(INCLUDE) $(OPTIMIZE) $< -o $@
	
clean: 
	rm -f $(OBJECTS) $(EXECUTABLE) $(DEPS)
	@echo Clean done
	

