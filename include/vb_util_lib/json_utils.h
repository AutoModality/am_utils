/*
 * AMJSONUtils.h
 *
 *  Created on: Oct 8, 2014
 *      Author: dan
 */

#ifndef AMJSONUTILS_H_
#define AMJSONUTILS_H_

#include <boost/date_time/posix_time/posix_time.hpp>

#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/Float64.h>
#include <nav_msgs/Odometry.h>
#include <rosgraph_msgs/Log.h>

#include <brain_box_msgs/C2Status.h>
#include <brain_box_msgs/CommsStatus.h>
#include <brain_box_msgs/DepthStatus.h>
#include <brain_box_msgs/FTPStatus.h>
#include <brain_box_msgs/TargetPose.h>
#include <brain_box_msgs/TargetPoseArray.h>
#include <brain_box_msgs/VisionStatus.h>
#include <brain_box_msgs/VideoStatus.h>
#include <brain_box_msgs/SuperStatus.h>
#include <brain_box_msgs/SPStatus.h>
#include <brain_box_msgs/VisLogStatus.h>
#include <brain_box_msgs/BBVxCommand.h>
#include <brain_box_msgs/VisionCmnd.h>
#include <brain_box_msgs/VideoCmnd.h>
#include <brain_box_msgs/OperatorMsg.h>
#include <brain_box_msgs/VxMissionExecCmd.h>
#include <brain_box_msgs/VxState.h>
#include <brain_box_msgs/PIDParam.h>
#include <brain_box_msgs/LogControl.h>

#include <vb_util_lib/json.h>
#include <vb_util_lib/ftp_request.h>
#include <vb_util_lib/ftp_response.h>
#include <vb_util_lib/message.h>

using namespace std;

class AMJSONUtils {
public:
	enum AmMessageType {AM_MSG_TYPE_JSON};

	static bool isJSONMember(Json::Value value, const char *memberName);
	static bool isJSONString(Json::Value value, const char *memberName);
	static bool isJSONInt(Json::Value value, const char *memberName);
	static bool isJSONBool(Json::Value value, const char *memberName);

    static bool jsonToAMMessageHead(Json::Value &value, AMMessage &msg);
	static void jsonToCtrlVxCmnd(Json::Value &value, brain_box_msgs::BBVxCommand &msg);
	static void jsonToCtrlVxPid_param(Json::Value &value, brain_box_msgs::PIDParam &msg);
	static void jsonToCtrlVideoCmnd(Json::Value &value, brain_box_msgs::VideoCmnd &msg);
	static void jsonToCtrlVisionCmnd(Json::Value &value, brain_box_msgs::VisionCmnd &msg);
	static void jsonToRemoteCtrlVxMission(Json::Value &value, brain_box_msgs::VxMissionExecCmd &msg);
	static void jsonToLogControl(Json::Value &value, brain_box_msgs::LogControl &msg);
	static void jsonToOdometry(Json::Value &value, nav_msgs::Odometry &msg);

    static void poseStampedToJson(Json::Value &value, std::string vehicleId, uint16_t sequence, boost::posix_time::ptime timestamp,
    		std::string messageName, std::string sender, const geometry_msgs::PoseStamped &msg);
    static void bbPoseToJson(Json::Value &value, const brain_box_msgs::TargetPose &msg);
    static void throttleToJson(Json::Value &value, std::string vehicleId, uint16_t sequence, boost::posix_time::ptime timestamp,
    		std::string messageName, std::string sender, const std_msgs::Float64 &msg);

	static bool stringToFTPRequest(std::string jsonString, FTPRequest &request);
	static void ftpResponseToString(FTPResponse response, std::string &jsonString);

    static Json::Value statusC2ToJson(std::string vehicleId, uint16_t sequence, boost::posix_time::ptime timestamp,
    		std::string sender, const brain_box_msgs::C2Status &msg);
    static Json::Value statusCommsToJson(std::string vehicleId, uint16_t sequence, boost::posix_time::ptime timestamp,
    		std::string sender, const brain_box_msgs::CommsStatus &msg);
    static Json::Value statusDepthToJson(std::string vehicleId, uint16_t sequence, boost::posix_time::ptime timestamp,
    		std::string sender, const brain_box_msgs::DepthStatus &msg);
    static Json::Value statusFTPToJson(std::string vehicleId, uint16_t sequence, boost::posix_time::ptime timestamp,
    		std::string sender, const brain_box_msgs::FTPStatus &msg);
    static Json::Value statusSuperToJson(std::string vehicleId, uint16_t sequence, boost::posix_time::ptime timestamp,
    		std::string sender, const brain_box_msgs::SuperStatus &msg);
    static Json::Value statusVideoToJson(std::string vehicleId, uint16_t sequence, boost::posix_time::ptime timestamp,
    		std::string sender, const brain_box_msgs::VideoStatus &msg);
    static Json::Value statusVisionToJson(std::string vehicleId, uint16_t sequence, boost::posix_time::ptime timestamp,
    		std::string sender, const brain_box_msgs::VisionStatus &msg);
    static Json::Value statusVisLogToJson(std::string vehicleId, uint16_t sequence, boost::posix_time::ptime timestamp,
    		std::string sender, const brain_box_msgs::VisLogStatus &msg);

    static Json::Value heartbeatToJson(std::string vehicleId, uint16_t sequence, boost::posix_time::ptime timestamp,
    		std::string sender);
    static Json::Value snsrCamTrgtActToJson(std::string vehicleId, uint16_t sequence, boost::posix_time::ptime timestamp,
    		std::string sender, const brain_box_msgs::TargetPoseArray &msg);
    static Json::Value ctrlVxMissionMessageToJson(std::string vehicleId, uint16_t sequence, boost::posix_time::ptime timestamp,
    		std::string sender, const brain_box_msgs::OperatorMsg &msg);
    static Json::Value rosoutToJson(std::string vehicleId, uint16_t sequence, boost::posix_time::ptime timestamp,
    		std::string sender, const rosgraph_msgs::Log &msg);
    static Json::Value vstateSummaryToJson(std::string vehicleId, uint16_t sequence, boost::posix_time::ptime timestamp,
    		std::string sender, const brain_box_msgs::VxState &msg);

private:
    static void headerToJson(Json::Value &value, std::string vehicleId, uint16_t sequence, boost::posix_time::ptime timestamp,
    		std::string messageName, std::string sende, std::string type);
};
#endif /* AMJSONUTILS_H_ */
