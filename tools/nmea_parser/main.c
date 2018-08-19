#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <math.h>


#include "gpsd.h"

double getDistance(double lat1, double lon1, double lat2, double lon2) ;
double dist2(double lat1,double lon1,double lat2,double lon2);
double addDistance(double lat, double lon,double distance, double bearing) ;
double CalculateEndingGlobalCoordinates( double Lat, double Lon, double s_Bearing, double distance);

int main(int argc, char **argv)
{
	FILE *NMEA_fp;
	int c;
	/*if (argc != 2) { // Normal error handling
		printf("usage: %s [UUID]  \n", argv[0]);
		return 1;
	}*/

	NMEA_fp = fopen("nmea.txt","r");

	if(NMEA_fp == NULL)
		printf("Failed to open file\n\r");
	else{
		printf("open file\n\r");
		 do{ // read one line
				  c = fgetc(NMEA_fp);
				  gpsd_put_char(c);
		 }while(c != EOF);

	 volatile double ret, ret2, dif;
	 ret = getDistance(56.183527, 9.535453,56.183753, 9.535186);
	 ret2 = dist2(56.183527, 9.535453,56.183753, 9.535186);
	 dif = ret - ret2;

	 CalculateEndingGlobalCoordinates(56.183527,9.535453,30.125472,-33.352331);

	 asm volatile ("NOP");

	 fclose(NMEA_fp);
	}
}


double toRadians(double degree) {
    return degree * M_PI / 180;
}

double toDegree(double rad) {
    return rad * 180 / M_PI;
}

double getDistance(double lat1, double lon1, double lat2, double lon2) {
    double a = 6378137.0;   		//  (6378137.0 meters in WGS-84)
    double f = 1 / 298.257223563;	// 	(1/298.257223563 in WGS-84)
    double b = (1 - f) * a; 		//  (6356752.314245 meters in WGS-84)
    double U1 = atan((1 - f) * tan(toRadians(lat1)));
    double U2 = atan((1 - f) * tan(toRadians(lat2)));
    double sinU1 = sin(U1), cosU1 = cos(U1);
    double sinU2 = sin(U2), cosU2 = cos(U2);
    double L = toRadians(lon2 - lon1);

    double sinLambda;
    double cosLambda;
    double sinSigma;
    double cosSigma;
    double sigma;
    double cosSqAlpha;
    double cos2SigmaM;

    double lambda = L, prevLambda;
    int iterationLimit = 100;

    /*
     * Iteratively evaluate the following equations until
     * lambda converges has converged to the desired degree of accuracy
     * (corresponds to approximately 0.06mm)
     */
    do {
        sinLambda = sin(lambda);
        cosLambda = cos(lambda);;
        sinSigma = sqrt(pow(cosU2 * sinLambda, 2) + pow(cosU1 * sinU2 - sinU1 * cosU2 * cosLambda, 2));

        if (sinSigma == 0)  // coincident points
            return 0;

        cosSigma = sinU1 * sinU2 + cosU1 * cosU2 * cosLambda;
        sigma = atan(sinSigma / cosSigma);
        double sinAlpha = cosU1 * cosU2 * sinLambda / sinSigma;
        cosSqAlpha = 1 - pow(sinAlpha, 2);

        if (cosSqAlpha == 0)    // equatorial line: cosSqAlpha = 0
            cos2SigmaM = 0;
        else
            cos2SigmaM = cosSigma - 2 * sinU1 * sinU2 / cosSqAlpha;

        double C = f / 16 * cosSqAlpha * (4 + f * (4 - 3 * cosSqAlpha));
        prevLambda = lambda;
        lambda = L + (1 - C) * f * sinAlpha * (sigma + C * sinSigma * (cos2SigmaM + C * cosSigma * (-1 + 2 * pow(cos2SigmaM, 2))));
    } while ((abs(lambda - prevLambda) > 1e-12) && (--iterationLimit > 0));

    if (iterationLimit == 0)    // formula failed to converge
        return 0;

    double uSq = cosSqAlpha * (pow(a, 2) - pow(b, 2)) / pow(b, 2);
    double A = 1 + uSq / 16384 * (4096 + uSq * (-768 + uSq * (320 - 175 * uSq)));
    double B = uSq / 1024 * (256 + uSq * (-128 + uSq * (74 - 47 * uSq)));
    double deltaSigma = B * sinSigma * (cos2SigmaM + B / 4 * (cosSigma * (-1 + 2 * pow(cos2SigmaM, 2)) - B / 6 * cos2SigmaM * (-3 + 4 * pow(sinSigma, 2)) * (-3 + 4 * pow(cos2SigmaM, 2))));

    double s = b * A * (sigma - deltaSigma);    // in the same units as a and b

    // bearing (direction) in radius
    // degree = radius * 180 / pi
    // North is 0
    double revAz = atan2(cosU1 * sinLambda, -sinU1 * cosU2 + cosU1 * sinU2 * cosLambda);

    revAz = toDegree(revAz);
    printf("result : %f (%f)\n\r",s,revAz);   // 2.21113 radians

    return s;
}


double dist2(double lat1,double lon1,double lat2,double lon2)
{
	double a = cos(toRadians(90-lat1));
	double b = cos(toRadians(90-lat2));
	double c = sin(toRadians(90-lat1));
	double d = sin(toRadians(90-lat2));
	double e = cos(toRadians(lon1-lon2));
	double diff = acos(a*b + c*d*e) * 6371 * 1000;// [m]

	printf("result (dist2) : %f\n\r",diff);

	return diff;
}


double CalculateEndingGlobalCoordinates( double Lat, double Lon, double distance ,double s_Bearing)
{
	double Latitude = toRadians(Lat);
	double Longitude = toRadians(Lon);
	double startBearing = toRadians(s_Bearing);
	double a = 6378137.0;   		//  (6378137.0 meters in WGS-84)
		double f = 1 / 298.257223563;	// 	(1/298.257223563 in WGS-84)
		double b = (1 - f) * a; 		//  (6356752.314245 meters in WGS-84)
      double aSquared = a * a;
      double bSquared = b * b;
      double phi1 = Latitude;
      double alpha1 = startBearing;
      double cosAlpha1 = cos(alpha1);
      double sinAlpha1 = sin(alpha1);
      double s = distance;
      double tanU1 = (1.0 - f) * tan(phi1);
      double cosU1 = 1.0 / sqrt( 1.0 + tanU1 * tanU1 );
      double sinU1 = tanU1 * cosU1;

      // eq. 1
      double sigma1 = atan2(tanU1, cosAlpha1);

      // eq. 2
      double sinAlpha = cosU1 * sinAlpha1;

      double sin2Alpha = sinAlpha * sinAlpha;
      double cos2Alpha = 1 - sin2Alpha;
      double uSquared = cos2Alpha * (aSquared - bSquared) / bSquared;

      // eq. 3
      double A = 1 + (uSquared / 16384) * (4096 + uSquared * (-768 + uSquared * (320 - 175 * uSquared)));

      // eq. 4
      double B = (uSquared / 1024) * (256 + uSquared * (-128 + uSquared * (74 - 47 * uSquared)));

      // iterate until there is a negligible change in sigma
      double deltaSigma;
      double sOverbA = s / (b * A);
      double sigma = sOverbA;
      double sinSigma;
      double prevSigma = sOverbA;
      double sigmaM2;
      double cosSigmaM2;
      double cos2SigmaM2;
      int iterationLimit = 100;

      do
      {
    	  prevSigma = sigma;
        // eq. 5
        sigmaM2 = 2.0*sigma1 + sigma;
        cosSigmaM2 = cos(sigmaM2);
        cos2SigmaM2 = cosSigmaM2 * cosSigmaM2;
        sinSigma = sin(sigma);
        double cosSignma = cos(sigma);

        // eq. 6
        deltaSigma = B * sinSigma * (cosSigmaM2 + (B / 4.0) * (cosSignma * (-1 + 2 * cos2SigmaM2)
            - (B / 6.0) * cosSigmaM2 * (-3 + 4 * sinSigma * sinSigma) * (-3 + 4 * cos2SigmaM2)));

        // eq. 7
        sigma = sOverbA + deltaSigma;

        // break after converging to tolerance
      } while ((abs(sigma - prevSigma) > 1e-12) && (--iterationLimit > 0));

      if (iterationLimit == 0)    // formula failed to converge
             return 0;

      sigmaM2 = 2.0*sigma1 + sigma;
      cosSigmaM2 = cos(sigmaM2);
      cos2SigmaM2 = cosSigmaM2 * cosSigmaM2;

      double cosSigma = cos(sigma);
      sinSigma = sin(sigma);

      // eq. 8
      double phi2 = atan2( sinU1 * cosSigma + cosU1 * sinSigma * cosAlpha1,
                               (1.0-f) * sqrt( sin2Alpha + pow(sinU1 * sinSigma - cosU1 * cosSigma * cosAlpha1, 2.0)));

      // eq. 9
      // This fixes the pole crossing defect spotted by Matt Feemster.  When a path
      // passes a pole and essentially crosses a line of latitude twice - once in
      // each direction - the longitude calculation got messed up.  Using Atan2
      // instead of Atan fixes the defect.  The change is in the next 3 lines.
      //double tanLambda = sinSigma * sinAlpha1 / (cosU1 * cosSigma - sinU1*sinSigma*cosAlpha1);
      //double lambda = Math.Atan(tanLambda);
      double lambda = atan2(sinSigma * sinAlpha1, cosU1 * cosSigma - sinU1 * sinSigma * cosAlpha1);

      // eq. 10
      double C = (f / 16) * cos2Alpha * (4 + f * (4 - 3 * cos2Alpha));

      // eq. 11
      double L = lambda - (1 - C) * f * sinAlpha * (sigma + C * sinSigma * (cosSigmaM2 + C * cosSigma * (-1 + 2 * cos2SigmaM2)));

      // eq. 12
      double alpha2 = atan2(sinAlpha, -sinU1 * sinSigma + cosU1 * cosSigma * cosAlpha1);

      // build result
      double latitude_new;
      double longitude_new;

      latitude_new = phi2;
      longitude_new = Longitude + L;

      double endBearing = alpha2;

      printf("New point (%f,%f)\n\r",toDegree(latitude_new),toDegree(longitude_new));

      return 1.0;
    }
