/*
 * fcu_mode_type.cpp
 *
 *  Created on: Nov 25, 2015
 *      Author: ubuntu
 */

#include <am_utils/fcu_mode_type.h>
/*
 * Mappings between the FCU_MODE enums in fcu_mode_type.h and string representations
 */

//std::string mode_2_px4x[] {
//	"MANUAL",
//	"ACRO",
//	"ALTCTL",
//	"POSCTL",
//	"OFFBOARD",
//	"STABILIZED",
//	"AUTO.MISSION",
//	"AUTO.LOITER",
//	"AUTO.RTL",
//	"AUTO.LAND",
//	"AUTO.RTGS",
//	"AUTO.READY",
//	"AUTO.TAKEOFF",
//	"UNKNOWN",
//};
//
/**
 * mapping between the FCU_MODE enum and the PX4 string representation.
 */
std::map<FCU_MODE, std::string> mode_2_px4 = {
		{FCU_MODE::MANUAL, "MANUAL"},
		{FCU_MODE::ACRO, "ACRO"},
		{FCU_MODE::ALTCTL, "ALTCTL"},
		{FCU_MODE::POSCTL, "POSCTL"},
		{FCU_MODE::OFFBOARD, "OFFBOARD"},
		{FCU_MODE::STABILIZED, "STABILIZED"},
		{FCU_MODE::AUTO_MISSION, "AUTO.MISSION"},
		{FCU_MODE::AUTO_LOITER, "AUTO.LOITER"},
		{FCU_MODE::AUTO_RTL, "AUTO.RTL"},
		{FCU_MODE::AUTO_LAND, "AUTO.LAND"},
		{FCU_MODE::AUTO_RTGS, "AUTO.RTGS"},
		{FCU_MODE::AUTO_READY, "AUTO.READY"},
		{FCU_MODE::AUTO_TAKEOFF, "AUTO.TAKEOFF"},
		{FCU_MODE::UNKNOWN, "UNKNOWN"},
};

std::map<std::string, FCU_MODE> px4_2_mode = {
		{"MANUAL", FCU_MODE::MANUAL},
		{"ACRO", FCU_MODE::ACRO},
		{"ALTCTL", FCU_MODE::ALTCTL},
		{"POSCTL", FCU_MODE::POSCTL},
		{"OFFBOARD", FCU_MODE::OFFBOARD},
		{"STABILIZED", FCU_MODE::STABILIZED},
		{"AUTO.MISSION", FCU_MODE::AUTO_MISSION},
		{"AUTO.LOITER", FCU_MODE::AUTO_LOITER},
		{"AUTO.RTL", FCU_MODE::AUTO_RTL},
		{"AUTO.LAND", FCU_MODE::AUTO_LAND},
		{"AUTO.RTGS", FCU_MODE::AUTO_RTGS},
		{"AUTO.READY", FCU_MODE::AUTO_READY},
		{"AUTO.TAKEOFF", FCU_MODE::AUTO_TAKEOFF},
		{"UNKNOWN", FCU_MODE::UNKNOWN},
};


