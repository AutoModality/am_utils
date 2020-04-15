/*
 * gis_util.cpp
 *
 *  Created on: Sep 23, 2017
 *      Author: ubuntu
 */

#include <vb_util_lib/gis_util.h>

//enum class FOR {world, vehicle, target};
std::map<std::string, FOR> string_to_FOR = {
        {"world", FOR::world},
        {"vehicle", FOR::vehicle},
        {"target", FOR::target}};
std::map<FOR, std::string> FOR_to_string = {
        {FOR::world, "world"},
        {FOR::vehicle, "vehicle"},
        {FOR::target, "target"}};

//enum class GISEntityType {point=0, line, area};
std::map<std::string, GISEntityType> string_to_GISEntityType = {
        {"point", GISEntityType::point},
        {"line", GISEntityType::line},
        {"area", GISEntityType::area}};
std::map<GISEntityType, std::string> GISEntityType_to_string = {
        {GISEntityType::point, "point"},
        {GISEntityType::line, "line"},
        {GISEntityType::area, "area"}};

//enum class CoordType {GPS_COORD=0, FRU_COORD, FLU_COORD, ENU_COORD, XYZ_COORD};
std::map<std::string, CoordType> string_to_CoordType = {
        {"GPS_COORD", CoordType::GPS_COORD},
        {"FRU_COORD", CoordType::FRU_COORD},
        {"FLU_COORD", CoordType::FLU_COORD},
        {"ENU_COORD", CoordType::ENU_COORD},
        {"XYZ_COORD", CoordType::XYZ_COORD}};
std::map<CoordType, std::string> CoordType_to_string = {
        {CoordType::GPS_COORD, "GPS_COORD"},
        {CoordType::FRU_COORD, "FRU_COORD"},
        {CoordType::FLU_COORD, "FLU_COORD"},
        {CoordType::ENU_COORD, "ENU_COORD"},
        {CoordType::XYZ_COORD, "XYZ_COORD"}};

void printGISCoordinate(GISCoordinate& c) {
    switch (c.type)
    {
        case CoordType::GPS_COORD:
            printf("%s lat[%0.6f] lon[%0.6f] alt[%0.2f]", CoordType_to_string[c.type].c_str(),
                   c.coord.gps.latitude, c.coord.gps.longitude, c.coord.gps.altitude);
            break;
        case CoordType::FRU_COORD:
            printf("%s F[%0.2f] R[%0.2f] U[%0.2f]", CoordType_to_string[c.type].c_str(),
                   c.coord.fru.front, c.coord.fru.right, c.coord.fru.up);
            break;
        case CoordType::FLU_COORD:
            printf("%s F[%0.2f] L[%0.2f] U[%0.2f]", CoordType_to_string[c.type].c_str(),
                   c.coord.flu.front, c.coord.flu.left, c.coord.flu.up);
            break;
        case CoordType::ENU_COORD:
            printf("%s E[%0.2f] N[%0.6f] U[%0.2f]", CoordType_to_string[c.type].c_str(),
                   c.coord.enu.east, c.coord.enu.north, c.coord.enu.up);
            break;
        case CoordType::XYZ_COORD:
            printf("%s X[%0.2f] Y[%0.2f] Z[%0.2f]", CoordType_to_string[c.type].c_str(),
                   c.coord.xyz.x, c.coord.xyz.y, c.coord.xyz.z);
            break;
    }
}
