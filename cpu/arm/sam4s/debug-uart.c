#include "compiler.h"
#include "udi_cdc.h"

#define USB_PORT	1

char volatile enumeration_complete;

void
dbg_putchar(const char ch)
{
	// Potential deadlock if tx buffers become full
	//if(enumeration_complete)
		udi_cdc_multi_putc(USB_PORT, ch);
}

unsigned int
dbg_send_bytes(const unsigned char *seq, unsigned int len)
{
	// Potential deadlock if tx buffers become full
	//if(enumeration_complete)
		len = udi_cdc_multi_write_buf(USB_PORT, seq, len);

  return len;
}


