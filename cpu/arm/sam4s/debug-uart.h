#ifndef DEBUG_UART_H_USB_SAM4S__
#define DEBUG_UART_H_USB_SAM4S__



unsigned int
dbg_send_bytes(const unsigned char *seq, unsigned int len);


void
dbg_putchar(const char ch);


void put_char_into_log(unsigned char c);
uint16_t debug_log_fill_level(void);
uint16_t Get_debug_log(unsigned char *buf);


#endif /* DEBUG_UART_H_USB_SAM4S__ */
