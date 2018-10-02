#include "compiler.h"
#include "udi_cdc.h"

#define USB_PORT	1

char volatile enumeration_complete;

void
dbg_putchar(const char ch)
{
	while (ITM->PORT[0].u32 == 0UL) { __NOP(); }
		ITM->PORT[0].u8 = (uint8_t)ch;
	// Potential deadlock if tx buffers become full
	//if(enumeration_complete)
	//	udi_cdc_multi_putc(USB_PORT, ch);
}

unsigned int
dbg_send_bytes(const unsigned char *seq, unsigned int len)
{
	unsigned int size = len;
	while(size--)
		dbg_putchar(*seq++);
	// Potential deadlock if tx buffers become full
	//if(enumeration_complete)
	//	len = udi_cdc_multi_write_buf(USB_PORT, seq, len);

  return len;
}


