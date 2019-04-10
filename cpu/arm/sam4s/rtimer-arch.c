#include "contiki.h"
#include "contiki-conf.h"
#include "rtimer.h"
#include "pmc.h"
#include "rtimer-arch.h"
#include "include/sam4s.h"
#include "sleepmgr.h"
#include "clock.h"
#include "rtt.h"

//#define RTIMER_COMPENSATION_TICKS       0
#define RTIMER_SYNC_VALUE 				10
static rtimer_clock_t offset = 0;
volatile uint32_t sleep_count;
// eclipse helper
#ifndef rtimer_clock_t
typedef u32_t rtimer_clock_t;
#endif

volatile uint8_t status=0;
void RTT_Handler(void)
{
	//rtt_disable_interrupt(RTT, RTT_MR_ALMIEN);
	/* Get RTT status */
	RTT->RTT_SR;
	status=0;
	ENERGEST_ON(ENERGEST_TYPE_IRQ);
	rtimer_run_next();
	ENERGEST_OFF(ENERGEST_TYPE_IRQ);
}


void
rtimer_arch_init(void)
{
	rtt_sel_source(RTT, 0);
	rtt_init(RTT, 4);

	NVIC_DisableIRQ(RTT_IRQn);
	NVIC_ClearPendingIRQ(RTT_IRQn);
	pmc_set_fast_startup_input(PMC_FSMR_RTTAL);
	NVIC_SetPriority(RTT_IRQn, 1); //level 0 is the highest interrupt priority (0-15)
	NVIC_EnableIRQ(RTT_IRQn);
	rtt_enable_interrupt(RTT, RTT_MR_ALMIEN);
}

void
rtimer_arch_set(rtimer_clock_t t)
{
	offset = t - RTT->RTT_VR;
}

rtimer_clock_t
rtimer_arch_now(void)
{
	return RTT->RTT_VR + offset;
}

void
rtimer_arch_schedule(rtimer_clock_t t)
{
	uint32_t expiry = t + RTIMER_SYNC_VALUE;

	if(RTIMER_CLOCK_LT(expiry, rtimer_arch_now())) {
		/* too soon, run now instead */
		rtimer_run_next();
		return;
	}

	rtt_write_alarm_time(RTT, expiry);
	status=1;
}

void
rtimer_adjust_ticks(clock_time_t howlong)
{
	// Adjust other timer ticks
	clock_adjust_ticks(howlong);
	rtimer_run_next();
}
//#if RDC_CONF_MCU_SLEEP

static void disable_brownout_detector(void)
{
	uint32_t ul_mr = SUPC->SUPC_MR & (~(SUPC_MR_KEY_Msk | SUPC_MR_BODDIS));
	SUPC->SUPC_MR = SUPC_MR_KEY_PASSWD | ul_mr | SUPC_MR_BODDIS;
}

/**
 * \brief Enable the assertion of core reset signal when a brownout detection occurs.
 *
 * \param p_supc Pointer to a SUPC instance.
 */
void enable_brownout_reset(void)
{
	uint32_t ul_mr = SUPC->SUPC_MR & (~(SUPC_MR_KEY_Msk | SUPC_MR_BODRSTEN));
	SUPC->SUPC_MR = SUPC_MR_KEY_PASSWD | ul_mr | SUPC_MR_BODRSTEN;
}


// Hotfix: If the binary sensor should be able to wake up the device
// 			we don't know how long we have been sleep. This is an
//			experiment to compensate for this.
static volatile rtimer_clock_t sleep_start, sleep_dif;

void
rtimer_arch_sleep(rtimer_clock_t howlong)
{
	uint8_t enable_TSON=0;
	/*Step 1:  Calculate the sleep count from the ticks received */
	sleep_count = howlong *CLOCK_CONF_SECOND;
	/*Step 2: Enable RTC and set the calculated time period */
	RTT->RTT_SR;
	rtt_write_alarm_time(RTT, howlong+rtimer_arch_now());
	/* Step 3: Enable IRQ, since this function caled in IRQ context and other intterupts will be disabled by now */
	//cpu_irq_enable();
	/* Step 4: Set the sleep mode and put the MCU to sleep */


    ENERGEST_OFF(ENERGEST_TYPE_CPU);
    ENERGEST_ON(ENERGEST_TYPE_LPM);
    disable_brownout_detector();
    if(ADC->ADC_ACR & ADC_ACR_TSON){
    	ADC->ADC_ACR &= ~ADC_ACR_TSON;
    	enable_TSON = 1;
    }
    sleep_start = rtimer_arch_now();
    sleepmgr_sleep(SLEEPMGR_WAIT);
    sleep_dif = rtimer_arch_now() - sleep_start;
    enable_brownout_reset();
    if(enable_TSON)
    	ADC->ADC_ACR |= ADC_ACR_TSON;
    ENERGEST_OFF(ENERGEST_TYPE_LPM);
    ENERGEST_ON(ENERGEST_TYPE_CPU);

	/* Step 5: Once woke, wake Transceiver up */
	//wake_from_sleep(); // TRX Wakeup
	/* Step 6: Adjust the timer ticks - both rtimer and etimer */
    // divide by rtimer to get rticks => time and time * CLOCK_CONF_SEC => clock ticks
    //rtimer_adjust_ticks(sleep_count/RTIMER_ARCH_SECOND);
	rtimer_adjust_ticks((sleep_dif *CLOCK_CONF_SECOND)/RTIMER_ARCH_SECOND);
    return;
}

//#endif
