/*
 * gis_entity.h
 *
 *  Created on: Nov 3, 2015
 *      Author: ubuntu
 */

#ifndef VISBOX_PACKAGES_COMMON_UTILS_UTIL_LIB_SRC_GIS_ENTITY_H_
#define VISBOX_PACKAGES_COMMON_UTILS_UTIL_LIB_SRC_GIS_ENTITY_H_

#include <vector>
#include <brain_box_msgs/msg/location_coord.hpp>
#include <brain_box_msgs/msg/gis_entity.hpp>

#include <vb_util_lib/gis_util.h>
#include <vb_util_lib/json.h>

class GISEntity {
public:
	GISEntity();
	GISEntity(brain_box_msgs::msg::GISEntity* msg);
	virtual ~GISEntity();

	static GISCoordinate locationCoordMsg_to_GISCoordinate(brain_box_msgs::msg::LocationCoord* msg);
	static brain_box_msgs::msg::LocationCoord gisCoordinate_to_LocationCoordMsg(GISCoordinate* c);

	void getGISEntityMsg(brain_box_msgs::msg::GISEntity* msg);

	/**
	 * Load JSON representation
	 */
	bool loadJSON(Json::Value& fp);

	/**
	 * General purpose routine to load GISCoordinates
	 */
	static bool loadGISCoordinate(Json::Value& fp, GISCoordinate* c);

	std::vector<GISCoordinate> getCoords() const {
		return coords_;
	}

	void print(std::string prefix = "");

private:
	std::string uuid_ = "";
	GISEntityType type_ = GISEntityType::point;
	std::vector<GISCoordinate> coords_;

	void parseGISEntityMsg(brain_box_msgs::msg::GISEntity* msg);
};

#endif /* VISBOX_PACKAGES_COMMON_UTILS_UTIL_LIB_SRC_GIS_ENTITY_H_ */
