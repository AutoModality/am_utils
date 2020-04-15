/*
 * gps_util.cpp
 *
 *  Created on: Dec 3, 2015
 *      Author: ubuntu
 */

#include <vb_util_lib/gps_util.h>

//const double M_PI = 3.141592653589793;

using namespace std;

double toRadians(double deg) { return (deg / 180.0) * M_PI; }

double normalizePI(double ang) {
    while (ang > M_PI) { ang -= M_PI; }
    while (ang < -M_PI) { ang += M_PI; }

    return ang;
}

double normalize2PI(double ang) {
    while (ang > (2*M_PI)) { ang -= (2*M_PI); }
    while (ang < 0) { ang += (2*M_PI); }

    return ang;
}

double latLonDeg_to_Dist(double lat1, double lon1, double lat2, double lon2) {
    double delLat = toRadians(lat2-lat1);
    double delLon = toRadians(lon2-lon1);
    lat1 = toRadians(lat1);
    lat2 = toRadians(lat2);

    double a = (sin(delLat/2) * sin(delLat/2)) +
            (cos(lat1) * cos(lat2) *
            sin(delLon/2) * sin(delLon/2));
    double c = 2 * atan2(sqrt(a), sqrt(1-a));

    return EARTH_RADIUS * c;
}
//φ1 lat1
//var y = Math.sin(λ2-λ1) * Math.cos(φ2);
//var x = Math.cos(φ1)*Math.sin(φ2) -
//        Math.sin(φ1)*Math.cos(φ2)*Math.cos(λ2-λ1);
//var brng = Math.atan2(y, x).toDegrees();
//
//var y = Math.sin(lon2-lon1) * Math.cos(lat2);
//var x = Math.cos(lat1)*Math.sin(lat2) -
//        Math.sin(lat1)*Math.cos(lat2)*Math.cos(lon2-lon1);
//var brng = Math.atan2(y, x).toDegrees();
double latLonDeg_to_Bearing(double lat1, double lon1, double lat2, double lon2) {
    lat1 = toRadians(lat1);
    lat2 = toRadians(lat2);
    lon1 = toRadians(lon1);
    lon2 = toRadians(lon2);

    double Bx = (cos(lat1) * sin(lat2)) -
            (sin(lat1) * cos(lat2) * cos(lon2-lon1));
    double By = cos(lat2) * sin(lon2-lon1);
    double lon3 = atan2(By, Bx);
    lon3 = normalizePI(lon3); // normalise to -180..+180°
    return lon3;
}

void newGPSCoord(double lat, double lon, double bearing, double distance,
        double *new_lat, double *new_lon) {
    *new_lat = asin( sin(lat)*cos(distance/EARTH_RADIUS) +
                        cos(lat)*sin(distance/EARTH_RADIUS)*cos(bearing) );
    *new_lon = lon + atan2(sin(bearing)*sin(distance/EARTH_RADIUS)*cos(lat),
                             cos(distance/EARTH_RADIUS)-sin(lat)*sin(*new_lat));
    return;
}


