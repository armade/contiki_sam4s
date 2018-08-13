#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>

#include "gpsd.h"

int main(int argc, char **argv)
{
	FILE *NMEA_fp;
	char c;
	/*if (argc != 2) { // Normal error handling
		printf("usage: %s [UUID]  \n", argv[0]);
		return 1;
	}*/

	NMEA_fp = fopen("nmea.txt","r");

	 do{ // read one line
	          c = fgetc(NMEA_fp);
	          gpsd_put_char(c);
	 }while(c != EOF);

	 fclose(NMEA_fp);
}
