/*
 * am_message.cpp
 *
 *  Created on: May 3, 2015
 *      Author: dan
 */
#include <vb_util_lib/message.h>


const std::string AMMessage::NAME_CTRL_VIDEO_CMND = "CtrlVideoCmnd";
const std::string AMMessage::NAME_CTRL_VISION_CMND = "CtrlVisionCmnd";
const std::string AMMessage::NAME_CTRL_VX_CMND = "CtrlVxCmnd";
const std::string AMMessage::NAME_CTRL_VX_PID_PARAM = "CtrlVxPid_param";
const std::string AMMessage::NAME_CTRL_VX_POS_SETPT = "CtrlCtrlPosSetpt";
const std::string AMMessage::NAME_HEARTBEAT = "Heartbeat";
const std::string AMMessage::NAME_SNSR_CAM_TRGT_ACT = "SnsrCamTrgtAct";
const std::string AMMessage::NAME_STATUS_C2 = "StatusC2";
const std::string AMMessage::NAME_STATUS_FTP = "StatusFTP";
const std::string AMMessage::NAME_STATUS_SUPER = "StatusSuper";
const std::string AMMessage::NAME_STATUS_VIDEO = "StatusVideo";
const std::string AMMessage::NAME_STATUS_VISION = "StatusVision";
const std::string AMMessage::NAME_STATUS_VISLOG = "StatusVisLog";
const std::string AMMessage::NAME_CTRL_VX_MISSION_MESSAGE = "CtrlVxMissionMessage";
const std::string AMMessage::NAME_REMOTE_CTRL_VX_MISSION = "RemoteCtrlVxMission";
const std::string AMMessage::NAME_ROSOUT = "Rosout";
const std::string AMMessage::NAME_VSTATE_SUMMARY = "VStateSummary";
const std::string AMMessage::NAME_STATUS_COMMS = "StatusComms";
const std::string AMMessage::NAME_STATUS_DEPTH = "StatusDepth";
const std::string AMMessage::NAME_LOG_CONTROL = "LogControl";
const std::string AMMessage::NAME_ODOMETRY = "Odometry";
const std::string AMMessage::NAME_VX_FEATURE_POSE = "VxFeaturePose";

const std::string AMMessage::TYPE_AUTOMOD = "A";
const std::string AMMessage::TYPE_MAVLINK = "M";
const std::string AMMessage::TYPE_MAVLINK_RAW = "R";
const std::string AMMessage::TYPE_ROS2 = "2";

AMMessage::AMMessage() {
}

AMMessage::~AMMessage() {
}

