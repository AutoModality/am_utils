/*
 * dji_challenge_util.h
 *
 *  Created on: Apr 27, 2016
 *      Author: ubuntu
 */

#ifndef VISBOX_PACKAGES_COMMON_UTILS_UTIL_LIB_INCLUDE_VB_UTIL_LIB_DJI_CHALLENGE_UTIL_H_
#define VISBOX_PACKAGES_COMMON_UTILS_UTIL_LIB_INCLUDE_VB_UTIL_LIB_DJI_CHALLENGE_UTIL_H_

#include <brain_box_msgs/VisionCmnd.h>


class DJIChallengeUtil
{
public:
	enum Mode {SURVIVOR, LANDING};

	static brain_box_msgs::VisionCmnd selectMode(Mode mode);
};

#endif /* VISBOX_PACKAGES_COMMON_UTILS_UTIL_LIB_INCLUDE_VB_UTIL_LIB_DJI_CHALLENGE_UTIL_H_ */
