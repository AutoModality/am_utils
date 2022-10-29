/*
 * am_bag_utils.cpp
 *
 *  Created on: Sep 3, 2015
 *      Author: dan
 */

#ifndef BRAIN_BOX_MSGS_INCLUDE_BRAIN_BOX_MSGS_AM_BAG_UTILS_CPP_
#define BRAIN_BOX_MSGS_INCLUDE_BRAIN_BOX_MSGS_AM_BAG_UTILS_CPP_

#include <rclcpp/rclcpp.hpp>

namespace brain_box {

class AMBagUtils {
public:
	static std::string getFileName();
};

} /* namespace brain_box */

#endif /* BRAIN_BOX_MSGS_INCLUDE_BRAIN_BOX_MSGS_AM_BAG_UTILS_CPP_ */
