/*
 * gps_location.cpp
 *
 *  Created on: Nov 9, 2015
 *      Author: ubuntu
 */

#include "../../../utils/util_lib/include/vb_util_lib/gps_location.h"

GPSLocation::GPSLocation() {
	// TODO Auto-generated constructor stub

}

GPSLocation::~GPSLocation() {
	// TODO Auto-generated destructor stub
}

void GPSLocation::updateLocation(const sensor_msgs::NavSatFix::ConstPtr& msg) {
	latitude_ = msg->latitude;
	longitude_ = msg->longitude;
	altitude_ = msg->altitude;
	status_ = msg->status;
}
