/*
 * gpsd.c
 *
 *  Created on: 13/08/2018
 *      Author: pm
 */


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


#include <string.h>
#include "minmea.h"

struct minmea_sentence_rmc frame_rmc;
struct minmea_sentence_gga frame_gga;
struct minmea_sentence_gst frame_gst;
struct minmea_sentence_gsv frame_gsv;
struct minmea_sentence_vtg frame_vtg;
struct minmea_sentence_zda frame_zda;

#define INDENT_SPACES "  "

int parse_sentence(char *line)
{
	uint8_t debug_buf[6];
	snprintf(debug_buf,sizeof(debug_buf),line+1);
	printf("%s\n\r",debug_buf);

	switch (minmea_sentence_id(line, false)) {
		case MINMEA_SENTENCE_RMC: {

			if (minmea_parse_rmc(&frame_rmc, line)) {
				printf(INDENT_SPACES "$xxRMC: raw coordinates and speed: (%d/%d,%d/%d) %d/%d\n",
						frame_rmc.latitude.value, frame_rmc.latitude.scale,
						frame_rmc.longitude.value, frame_rmc.longitude.scale,
						frame_rmc.speed.value, frame_rmc.speed.scale);
				printf(INDENT_SPACES "$xxRMC fixed-point coordinates and speed scaled to three decimal places: (%d,%d) %d\n",
						minmea_rescale(&frame_rmc.latitude, 1000),
						minmea_rescale(&frame_rmc.longitude, 1000),
						minmea_rescale(&frame_rmc.speed, 1000));
				printf(INDENT_SPACES "$xxRMC floating point degree coordinates and speed: (%f,%f) %f\n",
						minmea_tocoord(&frame_rmc.latitude),
						minmea_tocoord(&frame_rmc.longitude),
						minmea_tofloat(&frame_rmc.speed));

				printf(INDENT_SPACES "$xxRMC double point degree coordinates and speed: (%lf,%f) %lf\n",
						minmea_tocoord_double(&frame_rmc.latitude),
						minmea_tocoord_double(&frame_rmc.longitude),
						minmea_tofloat(&frame_rmc.speed));


				printf(INDENT_SPACES "$xxZDA: %d:%d:%d %02d.%02d.%d \n",
						frame_rmc.time.hours,
						frame_rmc.time.minutes,
						frame_rmc.time.seconds,
						frame_rmc.date.day,
						frame_rmc.date.month,
						frame_rmc.date.year);
			}
			else {
				printf(INDENT_SPACES "$xxRMC sentence is not parsed\n");
			}
		} break;

		case MINMEA_SENTENCE_GGA: {

			if (minmea_parse_gga(&frame_gga, line)) {

				printf(INDENT_SPACES "$xxRMC floating point degree coordinates and speed: (%f,%f)\n",
						minmea_tocoord(&frame_gga.latitude),
						minmea_tocoord(&frame_gga.longitude));

				printf(INDENT_SPACES "$xxRMC double point degree coordinates and speed: (%f,%f)\n",
						minmea_tocoord_double(&frame_gga.latitude),
						minmea_tocoord_double(&frame_gga.longitude));



			}
			else {
				printf(INDENT_SPACES "$xxGGA sentence is not parsed\n");
			}
		} break;

		case MINMEA_SENTENCE_GST: {

			if (minmea_parse_gst(&frame_gst, line)) {

			}
			else {
				printf(INDENT_SPACES "$xxGST sentence is not parsed\n");
			}
		} break;

		case MINMEA_SENTENCE_GSV: {

			if (minmea_parse_gsv(&frame_gsv, line)) {
				/*printf(INDENT_SPACES "$xxGSV: message %d of %d\n", frame_gsv.msg_nr, frame_gsv.total_msgs);
				printf(INDENT_SPACES "$xxGSV: sattelites in view: %d\n", frame_gsv.total_sats);
				for (int i = 0; i < 4; i++)
					printf(INDENT_SPACES "$xxGSV: sat nr %d, elevation: %d, azimuth: %d, snr: %d dbm\n",
						frame_gsv.sats[i].nr,
						frame_gsv.sats[i].elevation,
						frame_gsv.sats[i].azimuth,
						frame_gsv.sats[i].snr);*/
			}
			else {
				printf(INDENT_SPACES "$xxGSV sentence is not parsed\n");
			}
		} break;

		case MINMEA_SENTENCE_VTG: {

		   if (minmea_parse_vtg(&frame_vtg, line)) {

		   }
		   else {
				printf(INDENT_SPACES "$xxVTG sentence is not parsed\n");
		   }
		} break;

		case MINMEA_SENTENCE_ZDA: {

			if (minmea_parse_zda(&frame_zda, line)) {


				printf(INDENT_SPACES "$xxZDA: %d:%d:%d %02d.%02d.%d UTC%+03d:%02d\n",
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
				printf(INDENT_SPACES "$xxZDA sentence is not parsed\n");
			}
		} break;

		case MINMEA_SENTENCE_GLL:{

		}break;

		case MINMEA_SENTENCE_GSA:{

		}break;

		case MINMEA_INVALID: {
			printf(INDENT_SPACES "$xxxxx sentence is not valid\n");
		} break;

		default: {
			printf(INDENT_SPACES "$xxxxx sentence is not parsed (%s)\n",line);
		} break;
	}

    return 0;
}

uint8_t gpsd_index = 0;
uint8_t buf_nr = 0;
char buf[2][128];
uint8_t start_delimiter_seen=0;

void pull_event(char * data);

void gpsd_put_char(uint8_t c)
{
	if(start_delimiter_seen)
		buf[buf_nr][gpsd_index++] = c;
	else{
		if(c == '$'){
			buf[buf_nr][gpsd_index++] = c;
			start_delimiter_seen = 1;
		}
	}

	if((c=='\n') && (buf[buf_nr][gpsd_index-2] == '\r') && start_delimiter_seen)
	{
		buf[buf_nr][gpsd_index++] = '\0';
		pull_event(buf[buf_nr]);//process_post(PROCESS_BROADCAST, nmea_event, buf[buf_nr]);
		buf_nr = (buf_nr+1) & 1;
		gpsd_index = 0;
		start_delimiter_seen = 0;
	}
	gpsd_index &= 0x7F;
}


/*extern void gpsd_arch_init(void);
PROCESS_THREAD(gpsd_process, ev, data)
{
	PROCESS_BEGIN();
	printf("gpsd process started\n");
	nmea_event = process_alloc_event();
	gpsd_arch_init();

	while(1){
		PROCESS_YIELD();
		if(ev == nmea_event){
			parse_sentence((char *)data);
		}
	}

	PROCESS_END();
}*/

void pull_event(char * data)
{
	parse_sentence(data);
}
/*---------------------------------------------------------------------------*/


