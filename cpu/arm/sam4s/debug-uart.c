#include "compiler.h"
#include "udi_cdc.h"
#include "debug-uart.h"
#include <string.h>

#define USB_PORT	1

char volatile enumeration_complete;
#define log_print

void
dbg_putchar(const char ch)
{
	// Potential deadlock if tx buffers become full
	//if(enumeration_complete)
	//	udi_cdc_multi_putc(USB_PORT, ch);

#ifdef log_print
	if(ch != '\n'){
		put_char_into_log(ch);
	} else{
		put_char_into_log('<');
		put_char_into_log('b');
		put_char_into_log('r');
		put_char_into_log('>');
	}
#endif
}

unsigned int
dbg_send_bytes(const unsigned char *seq, unsigned int len)
{
	// Potential deadlock if tx buffers become full
	//if(enumeration_complete)
	//	len = udi_cdc_multi_write_buf(USB_PORT, seq, len);
#ifdef log_print
	unsigned int size = len;
	while(size--)
		if(*seq != '\n')
			put_char_into_log(*seq++);
		else{

			put_char_into_log('<');
			put_char_into_log('b');
			put_char_into_log('r');
			put_char_into_log('>');
		}
#endif
  return len;
}

#ifdef log_print
uint8_t debug_log[4096] = {0};
uint16_t debug_log_head=0, debug_log_full=0;

void put_char_into_log(unsigned char c)
{
	debug_log[debug_log_head] = c;
	debug_log_head = (debug_log_head+1)&0xfff;
	if(debug_log_head == 0)
		debug_log_full = 1;
}

uint16_t debug_log_fill_level(void)
{
	if(debug_log_full)
		return 4096;
	else
		return debug_log_head;
}

uint16_t Get_debug_log(unsigned char *buf)
{
	uint16_t current_entry = debug_log_head;
	uint16_t last_chunck_size = 4096 - current_entry;

	if(debug_log_fill_level()==4096){
		memcpy(buf,&debug_log[current_entry],last_chunck_size);
		memcpy(&buf[last_chunck_size],debug_log,current_entry);

		return 4096;
	}
	else{
		memcpy(buf,debug_log,debug_log_head);
		return debug_log_head;
	}
}
#endif
