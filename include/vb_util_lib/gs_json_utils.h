/*
 * AMJSONUtils.h
 *
 *  Created on: Oct 8, 2014
 *      Author: dan
 */

#ifndef VB_UTIL_LIB_SRC_GS_JSON_UTILS_H_
#define VB_UTIL_LIB_SRC_GS_JSON_UTILS_H_

#include <boost/date_time/posix_time/posix_time.hpp>

#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/Float64.h>
#include <rosgraph_msgs/Log.h>

#include <brain_box_msgs/GSCommand.h>
#include <brain_box_msgs/GSHeader.h>
#include <brain_box_msgs/GSMSDKStatus.h>
#include <brain_box_msgs/GSPNStatus.h>
#include <brain_box_msgs/GSAMStatus.h>
#include <vb_util_lib/json.h>

using namespace std;

class GSJSONUtils {
public:
    static void jsonToHeader(const Json::Value &value, brain_box_msgs::GSHeader &header);
	static void jsonToMSDKStatus(const Json::Value &value, brain_box_msgs::GSMSDKStatus &msg);
	static void jsonToCommand(const Json::Value &value, brain_box_msgs::GSCommand &msg);
    static void gsPNStatusToJson(Json::Value &value, const brain_box_msgs::GSPNStatus &msg);
    static void gsAMStatusToJson(Json::Value &value, const brain_box_msgs::GSAMStatus &msg);

private:
    static void headerToJson(Json::Value &value, const brain_box_msgs::GSHeader &header);
};
#endif /* VB_UTIL_LIB_SRC_GS_JSON_UTILS_H_ */
