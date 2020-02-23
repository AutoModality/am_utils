/*
 * mission_cmd_type.h
 *
 *  Created on: Nov 24, 2015
 *      Author: ubuntu
 */

#ifndef VISBOX_PACKAGES_COMMON_MESSAGES_INCLUDE_BRAIN_BOX_MSGS_MISSION_CMD_TYPE_H_
#define VISBOX_PACKAGES_COMMON_MESSAGES_INCLUDE_BRAIN_BOX_MSGS_MISSION_CMD_TYPE_H_

#include <string>

/**
 * Constants used for commands in VxFlightPlan.srv
 */

enum class MISSION_CMD {EXECUTE=0,
	RESUME=1,
	ABORT=2,
	PAUSE=3
};

extern std::string MISSION_CMD_KEY[];

#define MISSION_KEY(k) MISSION_CMD_KEY[static_cast<int>(k)]

#endif /* VISBOX_PACKAGES_COMMON_MESSAGES_INCLUDE_BRAIN_BOX_MSGS_MISSION_CMD_TYPE_H_ */
