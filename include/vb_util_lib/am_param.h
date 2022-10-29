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
#include <rclcpp/rclcpp.hpp>

class AMParam {
public:
	AMParam();
	virtual ~AMParam();

	static double getParam(rclcpp::Node::SharedPtr node, std::string prefix, double default_val = 0.0);
	static void setParam(rclcpp::Node::SharedPtr node, std::string prefix, double param);
};

#endif /* AM_PARAM_H_ */
