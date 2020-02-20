/*
 * flightplan_type.h
 *
 *  Created on: Nov 24, 2015
 *      Author: ubuntu
 */

#ifndef VISBOX_PACKAGES_COMMON_MESSAGES_INCLUDE_BRAIN_BOX_MSGS_FLIGHTPLAN_TYPE_H_
#define VISBOX_PACKAGES_COMMON_MESSAGES_INCLUDE_BRAIN_BOX_MSGS_FLIGHTPLAN_TYPE_H_

#include <string>

// From the VxFlightPlan.srv file:
//# key="FP_LOAD" value="id"	load the enclosed flight plan and give it id
//# key="FP_LOAD_JSON_FILE value="file name"  load json flight plan from file
//# key="FP_LOAD_PX4_FILE value="file name"  load PX4 flight plan from file
//# key="FP_DELETE" value="id" delete flight plan with id
//# key="FP_EXECUTE" value="id"			execute flight plan with id
//# key="FP_ABORT" value=""  			abort currently executing flight plan
//# key="FP_PAUSE" value=""				pause currently executing flight plan

/**
 * Defines for the commands used in VxFlightPlan service requests
 */

enum class FP_CMD { FP_LOAD=0,
	FP_LOAD_JSON_FILE,
	FP_LOAD_PX4_FILE,
	FP_DELETE,
    FP_LOAD_FCU,
    FP_START,
	FP_ABORT,
	FP_PAUSE,
};

extern std::string FP_CMD_KEY[];

#define FP_KEY(k) FP_CMD_KEY[static_cast<int>(k)]


#endif /* VISBOX_PACKAGES_COMMON_MESSAGES_INCLUDE_BRAIN_BOX_MSGS_FLIGHTPLAN_TYPE_H_ */
