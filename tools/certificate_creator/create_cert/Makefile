CC=gcc
CFLAGS=-c -Wall -MD
LDFLAGS=

SOURCES=$(shell find . ../sha3/ ../sha256/ ../xtea/ ../micro-ecc-master/ -maxdepth 1 -name "*.c") 
INCLUDE = -I ../sha3/ -I ../sha256/ -I ../xtea/ -I ../micro-ecc-master/
OBJECTS=$(SOURCES:.c=.o)
EXECUTABLE=make_certificate.exe
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
	
run:
	./$(EXECUTABLE) 123456789 border-router.bin
	
