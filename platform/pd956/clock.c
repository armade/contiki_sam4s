#include "contiki.h"
#include "platform-conf.h"
#include "include/sam4s.h"
#include "pmc.h"
#include "clock.h"
#include "rtc.h"

static volatile clock_time_t ticks;
static volatile clock_time_t offset = 0;
clock_gpbr_t *clock_gpbr = (clock_gpbr_t *)&CLOCK_FLAGS;
// NB: only 32 bit access for write
#define Set_GPBR_val(reg,x) 	volatile clock_gpbr_t tmp = *clock_gpbr;\
								tmp.reg = x;\
								*clock_gpbr = tmp;

/* sleepseconds is the number of seconds sleeping since startup, available globally */
clock_time_t sleepseconds;			// Holds whole seconds
clock_time_t sleepticks = 0; 	// Holds ticks (less then 1 sec)


static void Store_time_to_RTC(void *data);
void calc_daylight_saving(tm_t *tm);

void SysTick_Handler(void)
{
	ticks++;

	if (etimer_pending())
		etimer_request_poll();
}

void clock_init(void)
{
	ticks = 0;
#if !LOW_CLOCK
	SysTick_Config(120000);
#else
	SysTick_Config(30000);
#endif
}

clock_time_t clock_time(void)
{
	return ticks;
}
/*---------------------------------------------------------------------------*/
clock_time_t clock_seconds(void)
{
	return ticks / CLOCK_SECOND;
}
/*---------------------------------------------------------------------------*/
/**
 * Adjust the system current clock time.
 * \param dt   How many ticks to add
 *
 * Typically used to add ticks after an MCU sleep
 * clock_seconds will increment if necessary to reflect the tick addition.
 * Leap ticks or seconds can (rarely) be introduced if the ISR is not blocked.
 */
void clock_adjust_ticks(clock_time_t howmany)
{
	ticks += howmany;
	sleepticks += howmany;
	while (sleepticks >= CLOCK_SECOND)
	{
		sleepticks -= CLOCK_SECOND;
		sleepseconds++;
	}
}

// blocks the CPU for a specified number of clock ticks
void clock_wait(clock_time_t i)
{
	clock_time_t start;

	start = clock_time();
	while ((clock_time() - start) < i)
		;
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// E X T R A :   U N I X   T I M E
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


// Soft implementation of time.
// Ticks is ms and represent time since reset.
// Apply a time offset and we have a clock.
void clock_set_unix_time(clock_time_t time, uint8_t hw_save)
{
	offset = time - clock_seconds();
	if(hw_save)
		Store_time_to_RTC(NULL);
}
/*
 * unix time in UTC.
 */
clock_time_t clock_get_unix_time(void)
{
	if (offset == 0)
		return 0;
	else
		return offset + clock_seconds();
}

void clock_set_unix_timezone(clock_time_t zone)
{
	//Set_GPBR_val(timezone,zone);
	set_eeprom(timezone, zone);
}
/*
 * unix time as local time UTC + timezone
 */
clock_time_t clock_get_unix_localtime(void)
{
	uint16_t zone;

	get_eeprom(timezone, zone);
	return clock_get_unix_time() + zone;
}

/// Interface to RTC so we can store/load time


#define _BASE_DOW          4                    /* 01-01-70 was a Thursday */
//#define _LEAP_YEAR_ADJUST  17L                  /* Leap years 1900 - 1970 */


static const unsigned short days[4][12] = {
	{ 0, 31, 59, 90, 120, 151, 181, 212, 243, 273, 304, 334 },
	{ 365, 393, 424, 454, 485, 515, 546, 577, 607, 638, 668, 699 },
	{ 730, 759, 790, 820, 851, 881, 912, 943, 973, 1004, 1034, 1065 },
	{ 1096, 1125, 1156, 1186, 1217, 1247, 1278, 1309, 1339, 1370, 1400, 1431 },
};


clock_time_t RtctoUnix(tm_t *tb)
{
    unsigned int second = tb->tm_sec;  // 0-59
    unsigned int minute = tb->tm_min;  // 0-59
    unsigned int hour   = tb->tm_hour;    // 0-23
    unsigned int day    = tb->tm_mday-1;   // 0-30
    unsigned int month  = tb->tm_mon-1; // 0-11
    unsigned int year   = tb->tm_year-1970;    // 0-99
    //return (((year/4*(365*4+1)+days[year%4][month]+day)*24+hour)*60+minute)*60+second;
    return (((year/4*(365*4+1)+days[year&3][month]+day)*24+hour)*60+minute)*60+second;
}

void UnixtoRTC(tm_t *tb, clock_time_t Unix_epoch)
{
	unsigned int years;
	unsigned int year;
	unsigned int month;

	tb->tm_sec = Unix_epoch % 60;
	Unix_epoch /= 60;
	tb->tm_min = Unix_epoch % 60;
	Unix_epoch /= 60;
	tb->tm_hour = Unix_epoch % 24;
	Unix_epoch /= 24;

	tb->tm_wday = ((Unix_epoch + _BASE_DOW) % 7);
	if (tb->tm_wday == 0)
		tb->tm_wday = 7; // RTC range from 1-7 (normally 0-6) 0 = sunday => 7 = sunday

	years = Unix_epoch / (365 * 4 + 1) * 4;
	Unix_epoch %= 365 * 4 + 1;

	for (year = 3; year > 0; year--){
		if (Unix_epoch >= days[year][0])
			break;
	}

	for (month = 11; month > 0; month--){
		if (Unix_epoch >= days[year][month])
			break;
	}

	tb->tm_year = years + year + 1970;
	tb->tm_mon = month + 1; // month in RTC is 1-12 and tm is 0-11
	tb->tm_mday = Unix_epoch - days[year][month] + 1;
}

//struct ctimer rtc_timer;
struct ctimer stranum_timer;

void Load_time_from_RTC(void)
{
	tm_t timer;
	volatile clock_time_t Unix_time;
	int32_t ul_time;
	uint16_t zone;

	get_eeprom(timezone, zone);
	//Set_GPBR_val(timezone,zone);

	do{
		ul_time = RTC->RTC_TIMR;
		rtc_get_date(RTC, &timer.tm_year, &timer.tm_mon, &timer.tm_mday, &timer.tm_wday);
		rtc_get_time(RTC, &timer.tm_hour, &timer.tm_min, &timer.tm_sec);
	} while ((ul_time != RTC->RTC_TIMR));

	Unix_time = RtctoUnix(&timer);

	//Unix_time = 1533018106;
	//UnixtoRTC(&timer, Unix_time);

	if(clock_gpbr->RTC_valid == 0xA7){
		clock_quality(clock_gpbr->stranum+1);
		calc_daylight_saving(&timer);
	}else
		clock_quality(UNSYNC_TIME);

	clock_set_unix_time(Unix_time,0);
	//ctimer_set(&rtc_timer, 1UL * 60UL * 60UL * CLOCK_SECOND, Store_time_to_RTC,
	//		NULL); // 1 hr interval
}

#define const_bcd2bin(x)	(((x) & 0x0f) + ((x) >> 4) * 10)
#define const_bin2bcd(x)	((((x) / 10) << 4) + (x) % 10)
// In order to prevent rollover when setting the time, we set both at the same time.
// This is not supported in the framework.
int rtc_settime(Rtc* pRtc, tm_t *tm)
{
	if((pRtc->RTC_SR & RTC_SR_SEC) != RTC_SR_SEC)
		return -1;
	/* Stop Time/Calendar from counting */

	pRtc->RTC_CR |= RTC_CR_UPDCAL | RTC_CR_UPDTIM;

	while ((pRtc->RTC_SR & RTC_SR_ACKUPD) != RTC_SR_ACKUPD);
	pRtc->RTC_SCCR = RTC_SCCR_ACKCLR;

	pRtc->RTC_TIMR =  const_bin2bcd(tm->tm_sec) << 0
					| const_bin2bcd(tm->tm_min) << 8
					| const_bin2bcd(tm->tm_hour) << 16;


	pRtc->RTC_CALR =
			  const_bin2bcd((tm->tm_year ) / 100)	/* century */
			| const_bin2bcd(tm->tm_year % 100) << 8	/* year */
			| const_bin2bcd(tm->tm_mon) << 16		/* month*/
			| const_bin2bcd(tm->tm_wday) << 21	/* day of the week [0-6], Sunday=0 */
			| const_bin2bcd(tm->tm_mday) << 24;

	/* Restart Time/Calendar */
	pRtc->RTC_SCCR |= RTC_SCCR_SECCLR;
	pRtc->RTC_CR &= (uint32_t)(~(RTC_CR_UPDCAL|RTC_CR_UPDTIM)) ;
	pRtc->RTC_SCCR |= RTC_SCCR_SECCLR; /* clear SECENV in SCCR */

	calc_daylight_saving(tm);

	return 0;
}

static void Store_time_to_RTC(void *data)
{
	clock_time_t Unix_time = clock_get_unix_time();
	tm_t timer;

	if (Unix_time == 0)
		return;

	UnixtoRTC(&timer, Unix_time);

	rtc_settime(RTC,&timer);

	Set_GPBR_val(RTC_valid,0xA7);
	// We are using the internal rc 32kHz. This is really bad so update it every 6 hour
	// from the more accurate systimer.
	// TODO: When we sleep we run on slow clock the most of the time making this pointless.
	//ctimer_set(&rtc_timer, 6 * 60 * 60 * CLOCK_SECOND, Store_time_to_RTC, NULL); // 6 hr interval

}

static void Increment_stranum(void *data)
{
	if(clock_gpbr->stranum < UNSYNC_TIME){
		Set_GPBR_val(stranum,tmp.stranum+1);
		if(clock_gpbr->stranum < UNSYNC_TIME) // if we reach absolute rock bottom there is no need to call this again.
			ctimer_set(&stranum_timer, 1 * 60 * 60 * CLOCK_SECOND, Increment_stranum, NULL); // 1 hr interval
	}
}

int clock_quality(int stranum_new)
{
	if(stranum_new == -1)
		return clock_gpbr->stranum;
	else if((stranum_new < 0) || (stranum_new > 15) )
		stranum_new = 15;

	Set_GPBR_val(stranum,stranum_new);
	if(clock_gpbr->stranum < 15)
		ctimer_set(&stranum_timer, 1 * 60 * 60 * CLOCK_SECOND, Increment_stranum, NULL); // 1 hr interval
	return 1;
}

void calc_daylight_saving(tm_t *tm)
{
	uint16_t daylight_saving = 0;

	if((tm->tm_mon<3) || (tm->tm_mon>10)){
		daylight_saving = 0; //normal time
	}
	if((tm->tm_mon>3) && (tm->tm_mon<10)){
		daylight_saving = 1;// summer time
	}
	if(tm->tm_mon == 3)	{
		if(tm->tm_wday == 7){ 			// sunday
			if((tm->tm_mday + 7) > 31) 	// last sunday
				if(tm->tm_hour >= 1) 	// Change at 0100 UTC
					daylight_saving = 1;
		}
		daylight_saving = 0;
	}
	if(tm->tm_mon == 10)	{
		if(tm->tm_wday == 7){ 			// sunday
			if((tm->tm_mday + 7) > 31) 	// last sunday
				if(tm->tm_hour >= 1)	// Change at 0100 UTC
					daylight_saving = 0;
		}
		daylight_saving = 1;
	}

	if(daylight_saving == 1){
		// set rtc alarm on last sunday in oct
		clock_set_unix_timezone(2*60*60);
	}
	else{
		// set rtc alarm on last sunday in marts
		clock_set_unix_timezone(1*60*60);
	}
}
