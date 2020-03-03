/*
 * fcu_mode_type.h
 *
 *  Created on: Nov 25, 2015
 *      Author: ubuntu
 */

#ifndef VISBOX_PACKAGES_COMMON_MESSAGES_INCLUDE_BRAIN_BOX_MSGS_FCU_MODE_TYPE_H_
#define VISBOX_PACKAGES_COMMON_MESSAGES_INCLUDE_BRAIN_BOX_MSGS_FCU_MODE_TYPE_H_

#include <string>
#include <map>

enum class FCU_MODE {
	MANUAL=0,
	ACRO=1,
	ALTCTL=2,
	POSCTL=3,
	OFFBOARD=4,
	STABILIZED=5,
	AUTO_MISSION=6,
	AUTO_LOITER=7,
	AUTO_RTL=8,
	AUTO_LAND=9,
	AUTO_RTGS=10,
	AUTO_READY=11,
	AUTO_TAKEOFF=12,
	UNKNOWN=13,
};

extern std::map<FCU_MODE, std::string> mode_2_px4;
extern std::map<std::string, FCU_MODE> px4_2_mode;

#define MODE_TO_PX4(m) mode_2_px4[m]
#define PX4_TO_MODE(p) px4_2_mode[p]
#define PX4_TO_MODE_MSG(p) (static_cast<unsigned int>(px4_2_mode[p]))

#endif /* VISBOX_PACKAGES_COMMON_MESSAGES_INCLUDE_BRAIN_BOX_MSGS_FCU_MODE_TYPE_H_ */
