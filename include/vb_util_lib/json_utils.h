/*
 * AMJSONUtils.h
 *
 *  Created on: Oct 8, 2014
 *      Author: dan
 */

#ifndef AMJSONUTILS_H_
#define AMJSONUTILS_H_

#include <boost/date_time/posix_time/posix_time.hpp>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <std_msgs/msg/float64.hpp>

#include <brain_box_msgs/msg/c2_status.hpp>
#include <brain_box_msgs/msg/comms_status.hpp>
#include <brain_box_msgs/msg/depth_status.hpp>
#include <brain_box_msgs/msg/ftp_status.hpp>
#include <brain_box_msgs/msg/target_pose.hpp>
#include <brain_box_msgs/msg/target_pose_array.hpp>
#include <brain_box_msgs/msg/vision_status.hpp>
#include <brain_box_msgs/msg/video_status.hpp>
#include <brain_box_msgs/msg/super_status.hpp>
#include <brain_box_msgs/msg/sp_status.hpp>
#include <brain_box_msgs/msg/vis_log_status.hpp>
#include <brain_box_msgs/msg/bb_vx_command.hpp>
#include <brain_box_msgs/msg/vision_cmnd.hpp>
#include <brain_box_msgs/msg/video_cmnd.hpp>
#include <brain_box_msgs/msg/operator_msg.hpp>
#include <brain_box_msgs/msg/vx_mission_exec_cmd.hpp>
#include <brain_box_msgs/msg/vx_state.hpp>
#include <brain_box_msgs/msg/pid_param.hpp>
#include <brain_box_msgs/msg/log_control.hpp>
#include <rcl_interfaces/msg/log.hpp>

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
	static void jsonToCtrlVxCmnd(Json::Value &value, brain_box_msgs::msg::BBVxCommand &msg);
	static void jsonToCtrlVxPid_param(Json::Value &value, brain_box_msgs::msg::PIDParam &msg);
	static void jsonToCtrlVideoCmnd(Json::Value &value, brain_box_msgs::msg::VideoCmnd &msg);
	static void jsonToCtrlVisionCmnd(Json::Value &value, brain_box_msgs::msg::VisionCmnd &msg);
	static void jsonToRemoteCtrlVxMission(Json::Value &value, brain_box_msgs::msg::VxMissionExecCmd &msg);
	static void jsonToLogControl(Json::Value &value, brain_box_msgs::msg::LogControl &msg);

    static void poseStampedToJson(Json::Value &value, std::string vehicleId, uint16_t sequence, boost::posix_time::ptime timestamp,
    		std::string messageName, std::string sender, const geometry_msgs::msg::PoseStamped &msg);
    static void bbPoseToJson(Json::Value &value, const brain_box_msgs::msg::TargetPose &msg);
    static void throttleToJson(Json::Value &value, std::string vehicleId, uint16_t sequence, boost::posix_time::ptime timestamp,
    		std::string messageName, std::string sender, const std_msgs::msg::Float64 &msg);

	static bool stringToFTPRequest(std::string jsonString, FTPRequest &request);
	static void ftpResponseToString(FTPResponse response, std::string &jsonString);

    static Json::Value statusC2ToJson(std::string vehicleId, uint16_t sequence, boost::posix_time::ptime timestamp,
    		std::string sender, const brain_box_msgs::msg::C2Status &msg);
    static Json::Value statusCommsToJson(std::string vehicleId, uint16_t sequence, boost::posix_time::ptime timestamp,
    		std::string sender, const brain_box_msgs::msg::CommsStatus &msg);
    static Json::Value statusDepthToJson(std::string vehicleId, uint16_t sequence, boost::posix_time::ptime timestamp,
    		std::string sender, const brain_box_msgs::msg::DepthStatus &msg);
    static Json::Value statusFTPToJson(std::string vehicleId, uint16_t sequence, boost::posix_time::ptime timestamp,
    		std::string sender, const brain_box_msgs::msg::FTPStatus &msg);
    static Json::Value statusSuperToJson(std::string vehicleId, uint16_t sequence, boost::posix_time::ptime timestamp,
    		std::string sender, const brain_box_msgs::msg::SuperStatus &msg);
    static Json::Value statusVideoToJson(std::string vehicleId, uint16_t sequence, boost::posix_time::ptime timestamp,
    		std::string sender, const brain_box_msgs::msg::VideoStatus &msg);
    static Json::Value statusVisionToJson(std::string vehicleId, uint16_t sequence, boost::posix_time::ptime timestamp,
    		std::string sender, const brain_box_msgs::msg::VisionStatus &msg);
    static Json::Value statusVisLogToJson(std::string vehicleId, uint16_t sequence, boost::posix_time::ptime timestamp,
    		std::string sender, const brain_box_msgs::msg::VisLogStatus &msg);

    static Json::Value heartbeatToJson(std::string vehicleId, uint16_t sequence, boost::posix_time::ptime timestamp,
    		std::string sender);
    static Json::Value snsrCamTrgtActToJson(std::string vehicleId, uint16_t sequence, boost::posix_time::ptime timestamp,
    		std::string sender, const brain_box_msgs::msg::TargetPoseArray &msg);
    static Json::Value ctrlVxMissionMessageToJson(std::string vehicleId, uint16_t sequence, boost::posix_time::ptime timestamp,
    		std::string sender, const brain_box_msgs::msg::OperatorMsg &msg);
    static Json::Value rosoutToJson(std::string vehicleId, uint16_t sequence, boost::posix_time::ptime timestamp,
    		std::string sender, const rcl_interfaces::msg::Log &msg);
    static Json::Value vstateSummaryToJson(std::string vehicleId, uint16_t sequence, boost::posix_time::ptime timestamp,
    		std::string sender, const brain_box_msgs::msg::VxState &msg);

private:
    static void headerToJson(Json::Value &value, std::string vehicleId, uint16_t sequence, boost::posix_time::ptime timestamp,
    		std::string messageName, std::string sende, std::string type);
};
#endif /* AMJSONUTILS_H_ */
