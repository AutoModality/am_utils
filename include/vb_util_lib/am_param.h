/*
 * am_param.h
 *
 *  Created on: Oct 28, 2015
 *      Author: ubuntu
 */

#ifndef AM_PARAM_H_
#define AM_PARAM_H_

#include <stdio.h>
#include <termios.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/time.h>
#include <string.h>
#include <iostream>
#include <am_utils/am_ros2_utility.h>

class AMParam {
public:
	AMParam();
	virtual ~AMParam();

	static double getParam(std::string prefix, double default_val = 0.0) {
		return getParam(am::Node::node, prefix, default_val);
	}
	static double getParam(std::shared_ptr<rclcpp::Node> node, std::string prefix, double default_val = 0.0);
	
	static void setParam(std::string prefix, double param) {
		return setParam(am::Node::node, prefix, param);
	}
	static void setParam(std::shared_ptr<rclcpp::Node> node, std::string prefix, double param);
};

#endif /* AM_PARAM_H_ */
