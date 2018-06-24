#ifndef DEBUG_UART_H_USB_SAM4S__
#define DEBUG_UART_H_USB_SAM4S__



unsigned int
dbg_send_bytes(const unsigned char *seq, unsigned int len);


void
dbg_putchar(const char ch);


#endif /* DEBUG_UART_H_USB_SAM4S__ */
