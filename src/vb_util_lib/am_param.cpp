/*
 * am_aram.cpp
 *
 *  Created on: Oct 28, 2015
 *      Author: ubuntu
 */

#include "vb_util_lib/am_param.h"

AMParam::AMParam() {
	// TODO Auto-generated constructor stub

}

AMParam::~AMParam() {
	// TODO Auto-generated destructor stub
}

double AMParam::getParam(std::string prefix, double default_val)
{
	double param = default_val;
	if (ros::param::get(prefix, param))
	{
		ROS_DEBUG("%s = %f", prefix.c_str(), param);
	}
	else
	{
		ROS_WARN(" NO PARAMETER: %s, defaulting to %f", prefix.c_str(), param);
	}

	return param;
}

void AMParam::setParam(std::string prefix, double param)
{
	ros::param::set(prefix, param);
	ROS_DEBUG("%s = %f", prefix.c_str(), param);
}


