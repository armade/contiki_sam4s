/*
 * SNTP implementation for Contiki
 *
 * Copyright (C) 2011 Anuj Sehgal <s.anuj@jacobs-university.de>
 *
 * This program is part of free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
 *
 */

#include "contiki.h"
#include "contiki-lib.h"
#include "contiki-net.h"
#include "minmea.h"
#include "clock.h"

#include <string.h>

#define _DEBUG_                 1
#if _DEBUG_
#define PRINTF(...)       printf(__VA_ARGS__)
#else
#define PRINTF(...)
#endif

process_event_t nmea_event;
PROCESS(gpsd_process, "gpsd");
AUTOSTART_PROCESSES(&gpsd_process);

struct minmea_sentence_rmc frame_rmc;
struct minmea_sentence_gga frame_gga;
struct minmea_sentence_gst frame_gst;
struct minmea_sentence_gsv frame_gsv;
struct minmea_sentence_vtg frame_vtg;
struct minmea_sentence_zda frame_zda;

#define INDENT_SPACES "  "

volatile float lat_f,long_f,spd_m_s;
tm_t time;

int parse_sentence(char *line)
{
	char debug_buf[6];
	snprintf(debug_buf,sizeof(debug_buf),line+1);
	printf("%s\n\r",debug_buf);

	switch (minmea_sentence_id(line, false)) {
		case MINMEA_SENTENCE_RMC: {

			if (minmea_parse_rmc(&frame_rmc, line)) {
				/*PRINTF(INDENT_SPACES "$xxRMC: raw coordinates and speed: (%d/%d,%d/%d) %d/%d\n",
						frame_rmc.latitude.value, frame_rmc.latitude.scale,
						frame_rmc.longitude.value, frame_rmc.longitude.scale,
						frame_rmc.speed.value, frame_rmc.speed.scale);
				PRINTF(INDENT_SPACES "$xxRMC fixed-point coordinates and speed scaled to three decimal places: (%d,%d) %d\n",
						minmea_rescale(&frame_rmc.latitude, 1000),
						minmea_rescale(&frame_rmc.longitude, 1000),
						minmea_rescale(&frame_rmc.speed, 1000));*/


				lat_f = minmea_tocoord(&frame_rmc.latitude);
				long_f = minmea_tocoord(&frame_rmc.longitude);
				spd_m_s = minmea_tofloat(&frame_rmc.speed) * 0.514444444f; //m/s

				PRINTF(INDENT_SPACES "$xxRMC floating point degree coordinates and speed: (%f,%f) %f\n",
						lat_f,
						long_f,
						spd_m_s);

				time.tm_hour = frame_rmc.time.hours;
				time.tm_min =  frame_rmc.time.minutes;
				time.tm_sec =  frame_rmc.time.seconds;
				time.tm_mon =  frame_rmc.date.month;
				time.tm_mday = frame_rmc.date.day;
				time.tm_year = frame_rmc.date.year;

				clock_time_t unixtime = RtctoUnix(&time);


				clock_set_unix_time(unixtime,1);
				clock_quality(GPS_TIME);

				asm volatile("NOP");
				asm volatile("NOP");
				asm volatile("NOP");
			}
			else {
				PRINTF(INDENT_SPACES "$xxVTG sentence could not be parsed\n");
			}
		} break;

		case MINMEA_SENTENCE_GGA: {

			if (minmea_parse_gga(&frame_gga, line)) {
				lat_f = minmea_tocoord(&frame_gga.latitude);
				long_f = minmea_tocoord(&frame_gga.longitude);


				asm volatile("NOP");
				asm volatile("NOP");
				asm volatile("NOP");

			}
			else {
				PRINTF(INDENT_SPACES "$xxVTG sentence could not be parsed\n");
			}
		} break;

		case MINMEA_SENTENCE_GST: {

			if (minmea_parse_gst(&frame_gst, line)) {

			}
			else {
				PRINTF(INDENT_SPACES "$xxVTG sentence could not be parsed\n");
			}
		} break;

		case MINMEA_SENTENCE_GSV: {

			if (minmea_parse_gsv(&frame_gsv, line)) {
				/*PRINTF(INDENT_SPACES "$xxGSV: message %d of %d\n", frame_gsv.msg_nr, frame_gsv.total_msgs);
				PRINTF(INDENT_SPACES "$xxGSV: sattelites in view: %d\n", frame_gsv.total_sats);
				for (int i = 0; i < 4; i++)
					PRINTF(INDENT_SPACES "$xxGSV: sat nr %d, elevation: %d, azimuth: %d, snr: %d dbm\n",
						frame_gsv.sats[i].nr,
						frame_gsv.sats[i].elevation,
						frame_gsv.sats[i].azimuth,
						frame_gsv.sats[i].snr);*/
			}
			else {
				PRINTF(INDENT_SPACES "$xxVTG sentence could not be parsed\n");
			}
		} break;

		case MINMEA_SENTENCE_VTG: {

		   if (minmea_parse_vtg(&frame_vtg, line)) {

		   }
		   else {
				PRINTF(INDENT_SPACES "$xxVTG sentence could not be parsed\n");
		   }
		} break;

		case MINMEA_SENTENCE_ZDA: {

			if (minmea_parse_zda(&frame_zda, line)) {
				tm_t time;
				clock_time_t timezone_offset;

				if( frame_zda.time.hours == -1)
			 		break;

				time.tm_hour = frame_zda.time.hours;
				time.tm_min =  frame_zda.time.minutes;
				time.tm_sec =  frame_zda.time.seconds;
				time.tm_mon =  frame_zda.date.month;
				time.tm_mday = frame_zda.date.day;
				time.tm_year = frame_zda.date.year;

				clock_time_t unixtime = RtctoUnix(&time);
				timezone_offset = frame_zda.hour_offset*60*60;
				timezone_offset += frame_zda.minute_offset*60;
				//Fields 5 and 6 together yield the total offset.
				//For example, if field 5 is -5 and field 6 is +15,
				//local time is 5 hours and 15 minutes earlier than GMT
				if(frame_zda.hour_offset<0)
					timezone_offset *= -1;

				clock_set_unix_timezone(timezone_offset);
				clock_set_unix_time(unixtime,1);
				clock_quality(GPS_TIME);

				PRINTF(INDENT_SPACES "$xxZDA: %d:%d:%d %02d.%02d.%d UTC%+03d:%02d\n",
					   frame_zda.time.hours,
					   frame_zda.time.minutes,
					   frame_zda.time.seconds,
					   frame_zda.date.day,
					   frame_zda.date.month,
					   frame_zda.date.year,
					   frame_zda.hour_offset,
					   frame_zda.minute_offset);
			}
			else {
				PRINTF(INDENT_SPACES "$xxZDA sentence is not parsed\n");
			}
		} break;

		case MINMEA_INVALID: {
			PRINTF(INDENT_SPACES "$xxxxx sentence is not valid\n");
		} break;

		default: {
			PRINTF(INDENT_SPACES "$xxxxx sentence is not parsed\n");
		} break;
	}
	// release buffer
	*line = '\0';
    return 0;
}

uint8_t gpsd_index = 0;
uint8_t buf_nr = 0;
// NB this is good for debugging not for code size.
// TODO: rewrite
volatile uint8_t buf[8][128];
uint8_t start_delimiter_seen=0;

void gpsd_put_char(uint8_t c)
{
	if(c == '$'){
		gpsd_index = 0;
	}
	if(start_delimiter_seen)
		buf[buf_nr][gpsd_index++] = c;
	else{
		if(c == '$'){
			if(buf[buf_nr][0] == '\0'){
				buf[buf_nr][gpsd_index++] = c;
				start_delimiter_seen = 1;
			}
		}
	}

	if((c=='\n') && start_delimiter_seen)
	{
		buf[buf_nr][gpsd_index++] = '\0';
		process_post(PROCESS_BROADCAST, nmea_event,  (uint8_t *) buf[buf_nr]);
		buf_nr = (buf_nr+1) & 7;
		gpsd_index = 0;
		start_delimiter_seen = 0;
	}
	gpsd_index &= 0x7F;
}


extern void gpsd_arch_init(void);
PROCESS_THREAD(gpsd_process, ev, data)
{
	PROCESS_BEGIN();
	PRINTF("gpsd process started\n");
	nmea_event = process_alloc_event();
	gpsd_arch_init();

	while(1){
		PROCESS_YIELD();
		if(ev == nmea_event){
			PRINTF("%s",(char *)data);
			parse_sentence((char *)data);
		}
	}

	PROCESS_END();
}
/*---------------------------------------------------------------------------*/
