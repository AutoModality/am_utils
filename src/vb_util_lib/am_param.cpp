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

double AMParam::getParam(rclcpp::Node::SharedPtr node, std::string prefix, double default_val)
{
	double param = 0.0;

	if (!node->get_parameter_or(prefix, param, default_val))
	{
		RCLCPP_DEBUG(node->get_logger(), "%s = %f", prefix.c_str(), param);
	}
	else
	{
		RCLCPP_DEBUG(node->get_logger(), " NO PARAMETER: %s, defaulting to %f", prefix.c_str(), param);
	}

	return param;
}

void AMParam::setParam(rclcpp::Node::SharedPtr node, std::string prefix, double param)
{
	node->declare_parameter(prefix, param);
	RCLCPP_DEBUG(node->get_logger(), "%s = %f", prefix.c_str(), param);
}


