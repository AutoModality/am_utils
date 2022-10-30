/*
 * gis_entity.cpp
 *
 *  Created on: Nov 3, 2015
 *      Author: ubuntu
 */

#include <vb_util_lib/gis_entity.h>
#include <iostream>


GISEntity::GISEntity() {
}

GISEntity::GISEntity(brain_box_msgs::msg::GISEntity* msg) {
    parseGISEntityMsg(msg);
}

GISEntity::~GISEntity() {
	// TODO Auto-generated destructor stub
}

GISCoordinate GISEntity::locationCoordMsg_to_GISCoordinate(
		brain_box_msgs::msg::LocationCoord* msg) {
	GISCoordinate c;
	c.error = msg->error;
	c.uuid = msg->id;

	switch (msg->frame)
	{
	case brain_box_msgs::msg::LocationCoord::WGS_FRAME:
		c.coord.gps.latitude = msg->coord[brain_box_msgs::msg::LocationCoord::LATITUDE];
		c.coord.gps.longitude = msg->coord[brain_box_msgs::msg::LocationCoord::LONGITUDE];
		c.coord.gps.altitude = msg->coord[brain_box_msgs::msg::LocationCoord::ALT_ABS];
		c.type = CoordType::GPS_COORD;
	break;
	case brain_box_msgs::msg::LocationCoord::FRU_FRAME:
		c.coord.fru.front = msg->coord[brain_box_msgs::msg::LocationCoord::FRONT];
		c.coord.fru.right = msg->coord[brain_box_msgs::msg::LocationCoord::RIGHT];
		c.coord.fru.up = msg->coord[brain_box_msgs::msg::LocationCoord::UP];
		c.type = CoordType::FRU_COORD;
	break;
	case brain_box_msgs::msg::LocationCoord::ENU_FRAME:
		c.coord.enu.east = msg->coord[brain_box_msgs::msg::LocationCoord::EAST];
		c.coord.enu.north = msg->coord[brain_box_msgs::msg::LocationCoord::NORTH];
		c.coord.enu.up = msg->coord[brain_box_msgs::msg::LocationCoord::UP];
		c.type = CoordType::ENU_COORD;
	break;
	}

	return c;
}

brain_box_msgs::msg::LocationCoord GISEntity::gisCoordinate_to_LocationCoordMsg(
		GISCoordinate* c) {
	brain_box_msgs::msg::LocationCoord msg;
	msg.error = c->error;
	msg.id = c->uuid;

	switch (c->type)
	{
	case CoordType::GPS_COORD:
		msg.coord[brain_box_msgs::msg::LocationCoord::LATITUDE] = c->coord.gps.latitude;
		msg.coord[brain_box_msgs::msg::LocationCoord::LONGITUDE] = c->coord.gps.longitude;
		msg.coord[brain_box_msgs::msg::LocationCoord::ALT_ABS] = c->coord.gps.altitude;
		msg.frame = brain_box_msgs::msg::LocationCoord::WGS_FRAME;
		break;
	case CoordType::FRU_COORD:
		msg.coord[brain_box_msgs::msg::LocationCoord::FRONT] = c->coord.fru.front;
		msg.coord[brain_box_msgs::msg::LocationCoord::RIGHT] = c->coord.fru.right;
		msg.coord[brain_box_msgs::msg::LocationCoord::UP] = c->coord.fru.up;
		msg.frame = brain_box_msgs::msg::LocationCoord::FRU_FRAME;
		break;
	case CoordType::ENU_COORD:
		msg.coord[brain_box_msgs::msg::LocationCoord::EAST] = c->coord.enu.east;
		msg.coord[brain_box_msgs::msg::LocationCoord::NORTH] = c->coord.enu.north;
		msg.coord[brain_box_msgs::msg::LocationCoord::UP] = c->coord.enu.up;
		msg.frame = brain_box_msgs::msg::LocationCoord::ENU_FRAME;
		break;
	}

	return msg;
}

void GISEntity::getGISEntityMsg(brain_box_msgs::msg::GISEntity* msg) {
	msg->id = uuid_;

	switch (type_)
	{
	case GISEntityType::area:
		msg->type = brain_box_msgs::msg::GISEntity::AREA;
		break;
	case GISEntityType::line:
		msg->type = brain_box_msgs::msg::GISEntity::LINE;
		break;
	case GISEntityType::point:
	default:
		msg->type = brain_box_msgs::msg::GISEntity::POINT;
		break;
	}

	for(int i = 0; i < coords_.size(); i++)
	{
		msg->points.push_back(gisCoordinate_to_LocationCoordMsg(&(coords_.at(i))));
	}

	return;

}

bool GISEntity::loadGISCoordinate(Json::Value& fp, GISCoordinate* fc) {
    fc->uuid = fp.get("id", "").asString();
    fc->error = fp.get("error", 0.0).asDouble();
    std::string t = fp.get("type", "GPS").asString();
    if (!(t=="GPS" || t=="ENU" || t=="FRU"))
    {
        std::cerr << "Invalid coordinate type " << t << std::endl;
    }
    fc->type = string_to_CoordType[t];

    if (fc->error <= 0.0)
    {
        printf("Warning: error region <= 0 : [%f]\n", fc->error);
    }

    Json::Value c = fp["coord"];

    if (c.size() != 3)
    {
    	std::cerr << "Missing or invalid number of coordinates [coord]" << std::endl;
        return false;
    }

    double x = c[0].asDouble();
    double y = c[1].asDouble();
    double z = c[2].asDouble();
    switch (fc->type)
    {
    case CoordType::ENU_COORD:
        fc->coord.enu.east = x;
        fc->coord.enu.north = y;
        fc->coord.enu.up = z;
        break;
    case CoordType::FRU_COORD:
        fc->coord.fru.front = x;
        fc->coord.fru.right = y;
        fc->coord.fru.up = z;
        break;
    case CoordType::GPS_COORD:
        fc->coord.gps.latitude = x;
        fc->coord.gps.longitude = y;
        fc->coord.gps.altitude = z;
        break;
    }

//    ROS_INFO("      Loaded coordinate id:%s, error:%0.2fm, type:%s [%0.10f, %0.10f, %0.4f]",
//            fc->uuid.c_str(), fc->error, coordType_to_String(fc->type).c_str(), x, y, z);

    return true;
}

bool GISEntity::loadJSON(Json::Value& fp) {
	uuid_ = fp.get("id", "").asString();
	std::string ty = fp.get("type", "point").asString();
	if (!(ty=="point" || ty=="line" || ty=="area"))
	{
		std::cerr << "Invalid GISEntity type : " << ty << std::endl;
		return false;
	}

	type_ = string_to_GISEntityType[ty];
//	ROS_INFO("    Loading GIS Entity: %s [type=%s]", uuid_.c_str(), gisEntityType_to_String(type_).c_str());

	// Parse out all the coordinates
	for (Json::Value::iterator it = fp["coords"].begin(); it != fp["coords"].end(); ++it)
	{
		GISCoordinate fc;
		if (!loadGISCoordinate(*it, &fc))
		{
		    return false;
		}
		coords_.push_back(fc);
	}
	if (coords_.size() == 0 || (type_ == GISEntityType::point && coords_.size() != 1))
	{
        perror("Missing or invalid number of coordinates");
        return false;
	}

	return true;
}

void GISEntity::parseGISEntityMsg(brain_box_msgs::msg::GISEntity* msg) {
	uuid_ = msg->id;
	switch (msg->type)
	{
	case brain_box_msgs::msg::GISEntity::AREA:
		type_ = GISEntityType::area;
		break;
	case brain_box_msgs::msg::GISEntity::LINE:
		type_ = GISEntityType::line;
		break;
	case brain_box_msgs::msg::GISEntity::POINT:
	default:
		type_ = GISEntityType::point;
		break;
	}

	for(int i = 0; i < msg->points.size(); i++)
	{
		coords_.push_back(locationCoordMsg_to_GISCoordinate(&msg->points.at(i)));
	}

	return;
}

void GISEntity::print(std::string prefix) {
    printf("%s GISEntity  uuid:%s, type:%s\n", prefix.c_str(), uuid_.c_str(),
           GISEntityType_to_string[type_].c_str());
    for (int i = 0; i < coords_.size(); i++)
    {
        GISCoordinate c = coords_.at(i);
        switch (c.type)
        {
        case CoordType::GPS_COORD:
            printf("%s    %d. type:GPS_COORD  error: %f  %f, %f, %f\n", prefix.c_str(), i,
                    c.error, c.coord.gps.latitude, c.coord.gps.longitude, c.coord.gps.altitude);
            break;
        case CoordType::FRU_COORD:
            printf("%s    %d. type:FRU_COORD  error: %f, %f, %f %f\n", prefix.c_str(), i,
                    c.error, c.coord.fru.front, c.coord.fru.right, c.coord.fru.up);
            break;
        case CoordType::ENU_COORD:
            printf("%s    %d. type:ENU_COORD  error: %f, %f, %f %f\n", prefix.c_str(), i,
                    c.error, c.coord.enu.east, c.coord.enu.north, c.coord.enu.up);
            break;
        }
    }
    return;
}
