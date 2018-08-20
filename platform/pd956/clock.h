/*
 * clock.h
 *
 *  Created on: 8. aug. 2017
 *      Author: pm
 */

#ifndef PLATFORM_PD956_CLOCK_H_
#define PLATFORM_PD956_CLOCK_H_

typedef struct {
	int32_t tm_sec;/**seconds after the minute: 0 - 59*/
	int32_t tm_min;/**minutes after the hour: 0 - 59*/
	int32_t tm_hour;/**hours since midnight: 0 - 23*/
	int32_t tm_mday;/**day of the month: 1 - 31*/
	int32_t tm_mon;/**months since January: 0 - 11*/
	int32_t tm_year;/**years since 1900*/
	int32_t tm_wday;/**weekday: 1 - 7, monday-sunday*/
	//uint32_t tm_yday;/**days since Jan 1st: 0 - 365*/
	//int tm_isdst;/**daylight savings flag: -1 unknown, 0 not in DST, 1 in DST*/
} tm_t;
clock_time_t RtctoUnix( tm_t *tb);
void UnixtoRTC(tm_t *tb, clock_time_t Unix_epoch);
void Load_time_from_RTC(void);


void clock_adjust_ticks(clock_time_t howmany);

void clock_set_unix_time(clock_time_t time, uint8_t hw_save);

clock_time_t clock_get_unix_time(void);

void clock_set_unix_timezone(clock_time_t zone);
clock_time_t clock_get_unix_localtime(void);


#define READ_STRANUM	-1
#define GPS_TIME		1
#define PC_TIME			6
#define RTC_TIME		8
#define UNSYNC_TIME		16

int clock_quality(int stranum_new);

#endif /* PLATFORM_PD956_CLOCK_H_ */
