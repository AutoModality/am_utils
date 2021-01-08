/*
 * gps_util.h
 *
 *  Created on: Dec 3, 2015
 *      Author: ubuntu
 */

#ifndef VISBOX_PACKAGES_COMMON_UTILS_UTIL_LIB_INCLUDE_VB_UTIL_LIB_GPS_UTIL_H_
#define VISBOX_PACKAGES_COMMON_UTILS_UTIL_LIB_INCLUDE_VB_UTIL_LIB_GPS_UTIL_H_

#include <cmath>

#define EARTH_RADIUS_M 6371000  // meters

double latLonDeg_to_Dist(double lat1, double lon1, double lat2, double lon2);

double latLonDeg_to_Bearing(double lat1, double lon1, double lat2, double lon2);

void newGPSCoord(double lat, double lon, double bearing, double distance,
        double *new_lat, double *new_lon);

#endif /* VISBOX_PACKAGES_COMMON_UTILS_UTIL_LIB_INCLUDE_VB_UTIL_LIB_GPS_UTIL_H_ */
