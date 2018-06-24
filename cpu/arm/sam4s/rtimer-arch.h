#ifndef __RTIMER_ARCH_H__
#define __RTIMER_ARCH_H__

#include "contiki-conf.h"

#define RTIMER_ARCH_SECOND (32768>>2)

void rtimer_arch_init(void);
void rtimer_arch_schedule(rtimer_clock_t t);
rtimer_clock_t rtimer_arch_now(void);

#endif /* __RTIMER_ARCH_H__ */
