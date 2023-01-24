/*
 * GSJSONUtils.cpp
 *
 *  Created on: Oct 8, 2014
 *      Author: dan
 */
#include <iostream>
#include <string>
#include <ctime>
#include <stdio.h>
#include <string.h>
#include <boost/lexical_cast.hpp>

#include <ros/ros.h>

#include <vb_util_lib/time_util.h>
#include <vb_util_lib/json_utils.h>

#include <vb_util_lib/gs_json_utils.h>

void GSJSONUtils::headerToJson(
		Json::Value &value,
		const brain_box_msgs::GSHeader &header
)
{
    value["destination"] = header.destination;
    value["version"] = header.version;
}

void GSJSONUtils::jsonToHeader(
		const Json::Value &value,
		brain_box_msgs::GSHeader &header)
{
	header.destination = value["destination"].asString();
	header.version = value["version"].asUInt();
}

void GSJSONUtils::jsonToMSDKStatus(
		const Json::Value &value,
		brain_box_msgs::GSMSDKStatus &msg)
{
	jsonToHeader(value, msg.gs_header);

	msg.compass_error = value["compass_error"].asBool();
}

void GSJSONUtils::jsonToCommand(
		const Json::Value &value,
		brain_box_msgs::GSCommand &msg)
{
	jsonToHeader(value, msg.gs_header);

	msg.id = value["id"].asUInt();
	msg.value = value["value"].asString();
}

void GSJSONUtils::gsPNStatusToJson(
		Json::Value &value,
		const brain_box_msgs::GSPNStatus &msg)
{
	headerToJson(value, msg.gs_header);

	value["names"]["plan"] = msg.flight_plan;
	value["names"]["trajectory"] = msg.trajectory;
	value["names"]["segment"] = msg.segment;
	value["names"]["feature"] = msg.feature;
	value["telemetry"]["x"] = msg.x;
	value["telemetry"]["x_feature_fix"] = msg.x_feature_fix;
	value["telemetry"]["y"] = msg.y;
	value["telemetry"]["y_feature_fix"] = msg.y_feature_fix;
	value["telemetry"]["z"] = msg.z;
	value["telemetry"]["z_feature_fix"] = msg.z_feature_fix;
	value["telemetry"]["yaw"] = msg.yaw;
	value["telemetry"]["yaw_feature_fix"] = msg.yaw_feature_fix;
	value["am_status"]["id"] = msg.id;
	value["am_status"]["message"] = msg.message;
	value["am_status"]["code"] = msg.code;
	value["am_status"]["detail"] = msg.detail;
	value["am_status"]["color1"] = msg.color1;
	value["am_status"]["color2"] = msg.color2;
	value["am_status"]["attribute"] = msg.attribute;
}

void GSJSONUtils::gsAMStatusToJson(
		Json::Value &value,
		const brain_box_msgs::GSAMStatus &msg)
{
	headerToJson(value, msg.gs_header);

	value["am_status"]["code"] = msg.code;
}

