#include <stdio.h>
#include <debug-uart.h>
#include <string.h>

int
puts(const char *str)
{
  dbg_send_bytes((unsigned char*)str, strlen(str));

  dbg_putchar('\n');
  dbg_putchar('\r');
  return 0;
}
