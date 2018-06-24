#include "contiki.h"
#include "platform-conf.h"
#include "include/sam4s.h"
#include "pmc.h"
#include "clock.h"
#include "rtc.h"

static volatile clock_time_t ticks;
static volatile clock_time_t offset = 0;
/* sleepseconds is the number of seconds sleeping since startup, available globally */
clock_time_t sleepseconds;			// Holds whole seconds
clock_time_t sleepticks = 0; 	// Holds ticks (less then 1 sec)

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
// Ticks is ms an represent time since reset.
// Apply a time offset and we have a clock.
void clock_set_unix_time(clock_time_t time)
{
	offset = time - clock_seconds();
}

clock_time_t clock_get_unix_time(void)
{
	if (offset == 0)
		return 0;
	else
		return offset + clock_seconds();
}

/// Interface to RTC so we can store/load time

/* Number of days in 4 years: (365 * 4) + 1 */
//#define DAYSIN4YEARS 1461                       /* Start of 'trusted' time - 00:00:00 01/09/1999 (Wed, September) */
#define _DAY_SEC           (24L * 60L * 60L)    /* secs in a day */
//#define _YEAR_SEC          (365L * _DAY_SEC)    /* secs in a year */
//#define _FOUR_YEAR_SEC     (1461L * _DAY_SEC)   /* secs in a 4 year interval */
//#define _DEC_SEC           315532800L           /* secs in 1970-1979 */
#define _BASE_YEAR         70L                  /* 1970 is the base year */
#define _BASE_DOW          4                    /* 01-01-70 was a Thursday */
#define _LEAP_YEAR_ADJUST  17L                  /* Leap years 1900 - 1970 */

static const short int _days[] =
{ -1, 30, 58, 89, 119, 150, 180, 211, 242, 272, 303, 333, 364 };

static unsigned short days[4][12] =
{
{ 0, 31, 60, 91, 121, 152, 182, 213, 244, 274, 305, 335 },
{ 366, 397, 425, 456, 486, 517, 547, 578, 609, 639, 670, 700 },
{ 731, 762, 790, 821, 851, 882, 912, 943, 974, 1004, 1035, 1065 },
{ 1096, 1127, 1155, 1186, 1216, 1247, 1277, 1308, 1339, 1369, 1400, 1430 }, };

clock_time_t RtctoUnix(tm_t *tb)
{
	clock_time_t tmptm1, tmptm2, tmptm3;
	int month = tb->tm_mon - 1;
	tmptm1 = tb->tm_year - 1900;

	// Adjust month value so it is in the range 0 - 11.
	if ((month < 0) || (month > 11))
	{
		tmptm1 += (month / 12);
		if ((month %= 12) < 0)
		{
			month += 12;
			tmptm1--;
		}
	}
	/* Calculate days elapsed minus one, in the given year, to the given month. Check for leap year and adjust if necessary. */
	tmptm2 = _days[month];
	if (!(tmptm1 & 3) && (month > 1))
		tmptm2++;
	/* Calculate elapsed days since base date (midnight, 1/1/70, UTC)
	 * 365 days for each elapsed year since 1970, plus one more day for
	 * each elapsed leap year. no danger of overflow because of the range
	 * check (above) on tmptm1.*/
	tmptm3 = (tmptm1 - _BASE_YEAR) * 365UL
			+ ((tmptm1 - 1UL) >> 2)- _LEAP_YEAR_ADJUST;
	/* elapsed days to current month (still no possible overflow)     */
	tmptm3 += tmptm2;
	/* elapsed days to current date */
	tmptm1 = tmptm3 + (tmptm2 = (long) (tb->tm_mday));
	/* Calculate elapsed hours since base date */
	tmptm2 = tmptm1 * 24UL;
	tmptm1 = tmptm2 + (tmptm3 = (long) tb->tm_hour);
	/* Calculate elapsed minutes since base date     */
	tmptm2 = tmptm1 * 60UL;
	tmptm1 = tmptm2 + (tmptm3 = (long) tb->tm_min);
	/** Calculate elapsed seconds since base date     */
	tmptm2 = tmptm1 * 60UL;
	tmptm1 = tmptm2 + (tmptm3 = (long) tb->tm_sec);
	/***** HERE: tmptm1 holds number of elapsed seconds *****/
	return tmptm1; //
}

void UnixtoRTC(tm_t *tb, clock_time_t Unix_epoch)
{
	tb->tm_sec = Unix_epoch % 60;
	Unix_epoch /= 60;
	tb->tm_min = Unix_epoch % 60;
	Unix_epoch /= 60;
	tb->tm_hour = Unix_epoch % 24;
	Unix_epoch /= 24;

	tb->tm_wday = ((Unix_epoch + _BASE_DOW) % 7);
	if (tb->tm_wday == 0)
		tb->tm_wday = 7; // RTC range from 1-7 (normally 0-6) 0 = sun => 7 = sun

	unsigned int years = Unix_epoch / (365 * 4 + 1) * 4;
	Unix_epoch %= 365 * 4 + 1;

	unsigned int year;
	for (year = 3; year > 0; year--)
	{
		if (Unix_epoch >= days[year][0])
			break;
	}

	unsigned int month;
	for (month = 11; month > 0; month--)
	{
		if (Unix_epoch >= days[year][month])
			break;
	}

	tb->tm_year = years + year + 1970;
	tb->tm_mon = month + 1;
	tb->tm_mday = Unix_epoch - days[year][month] + 1;
}

struct ctimer rtc_timer;
static void Store_time_to_RTC(void *data);

void Load_time_from_RTC(void)
{
	tm_t timer;
	clock_time_t Unix_time;

	do
	{
		rtc_get_date(RTC, &timer.tm_year, &timer.tm_mon, &timer.tm_mday,
				&timer.tm_wday);
		rtc_get_time(RTC, &timer.tm_hour, &timer.tm_min, &timer.tm_sec);
	} while ((timer.tm_sec == 0)); // if sec is zero we have a possible overflow. Just do it again.

	Unix_time = RtctoUnix(&timer);
	//Unix_time = 1512309780;
	//UnixtoRTC(&timer, Unix_time);

	clock_set_unix_time(Unix_time);

	ctimer_set(&rtc_timer, 1UL * 60UL * 60UL * CLOCK_SECOND, Store_time_to_RTC,
			NULL); // 1 hr interval
}
static void Store_time_to_RTC(void *data)
{
	clock_time_t Unix_time = clock_get_unix_time();
	tm_t timer;

	if (Unix_time == 0)
		return;

	UnixtoRTC(&timer, Unix_time);
	// wait 3 sec, and try again.
	// This is a problem if vi are setting the time doing a day change.
	// Then we face the possibility of missing a day.(and so on)
	if (timer.tm_sec > 57)
	{
		ctimer_set(&rtc_timer, 3 * CLOCK_SECOND, Store_time_to_RTC, NULL); // 3s
		return;
	}

	Unix_time = Unix_time - clock_get_unix_time();
	if (Unix_time)
	{
		timer.tm_sec += Unix_time;
		// The next code should never be run. If it does then UnixtoRTC() is more then 3 sec in execution.
		if (timer.tm_sec > 59)
		{
			while (timer.tm_sec > 59)
			{
				timer.tm_min++;
				timer.tm_sec -= 59;
			}
		}
	}

	rtc_set_time(RTC, timer.tm_hour, timer.tm_min, timer.tm_sec);
	rtc_set_date(RTC, timer.tm_year, timer.tm_mon, timer.tm_mday,
			timer.tm_wday);

	// We are using the internal rc 32kHz. This is really bad so update it every 6 hour
	// from the more accurate systimer.
	ctimer_set(&rtc_timer, 6 * 60 * 60 * CLOCK_SECOND, Store_time_to_RTC, NULL); // 6 hr interval

}
