/*
 * gis_util.h
 *
 *  Created on: Nov 3, 2015
 *      Author: ubuntu
 */

#ifndef VISBOX_PACKAGES_COMMON_UTILS_UTIL_LIB_INCLUDE_VISBOX_UTIL_LIB_GIS_UTIL_H_
#define VISBOX_PACKAGES_COMMON_UTILS_UTIL_LIB_INCLUDE_VISBOX_UTIL_LIB_GIS_UTIL_H_

#include <stdio.h>
#include <uuid_msgs/UniqueID.h>
#include <string>
#include <map>

enum class FOR {world, vehicle, target};
extern std::map<std::string, FOR> string_to_FOR;
extern std::map<FOR, std::string> FOR_to_string;

enum class GISEntityType {point=0, line, area};
extern std::map<std::string, GISEntityType> string_to_GISEntityType;
extern std::map<GISEntityType, std::string> GISEntityType_to_string;

enum class CoordType {GPS_COORD=0, FRU_COORD, FLU_COORD, ENU_COORD, XYZ_COORD};
extern std::map<std::string, CoordType> string_to_CoordType;
extern std::map<CoordType, std::string> CoordType_to_string;

struct GPSCoord {
    double latitude;
    double longitude;
    double altitude;
};

struct XYZCoord {
    double x;
    double y;
    double z;
};

struct FRUCoord {
    double front;
    double right;
    double up;
};

struct ENUCoord {
    double east;
    double north;
    double up;
};

struct FLUCoord {
    double front;
    double left;
    double up;
};

struct GISCoordinate {
    double error;	// error region radius around point in meters
    std::string uuid;
    CoordType type;
    union {
        GPSCoord gps;
        FRUCoord fru;
        FLUCoord flu;
        ENUCoord enu;
        XYZCoord xyz;
    } coord;
};

void printGISCoordinate(GISCoordinate& c);

#endif /* VISBOX_PACKAGES_COMMON_UTILS_UTIL_LIB_INCLUDE_VISBOX_UTIL_LIB_GIS_UTIL_H_ */
