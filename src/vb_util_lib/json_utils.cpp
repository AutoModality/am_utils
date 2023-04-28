/*
 * AMJSONUtils.cpp
 *
 *  Created on: Oct 8, 2014
 *      Author: dan
 */
#include <iostream>
#include <string>
#include <ctime>
#include <stdio.h>
#include <string.h>
// #include <boost/lexical_cast.hpp>

#include <rclcpp/rclcpp.hpp>

#include <am_utils/mission_cmd_type.h>
#include <am_utils/message_util.h>

#include <vb_util_lib/time_util.h>
#include <vb_util_lib/json_utils.h>
#include <rcl_interfaces/msg/log.hpp>
#include <vb_util_lib/utility.h>
// #include <vb_util_lib/bag_logger.h>

bool AMJSONUtils::isJSONMember(Json::Value value, const char *memberName)
{
	if(!value.isMember(memberName))
	{
		cerr << "config_file missing member " << memberName << endl;
		return false;
	}
	return true;
}

bool AMJSONUtils::isJSONString(Json::Value value, const char *memberName)
{
	if(!isJSONMember(value, memberName))
	{
		return false;
	}
	if(!value[memberName].isString())
	{
		cerr << memberName << " is not a string" << endl;
		return false;
	}
	return true;
}

bool AMJSONUtils::isJSONInt(Json::Value value, const char *memberName)
{
	if(!isJSONMember(value, memberName))
	{
		return false;
	}
	if(!value[memberName].isInt())
	{
		cerr << memberName << " is not an int" << endl;
		return false;
	}
	return true;
}

bool AMJSONUtils::isJSONBool(Json::Value value, const char *memberName)
{
	if(!isJSONMember(value, memberName))
	{
		return false;
	}
	if(!value[memberName].isBool())
	{
		cerr << memberName << " is not a bool" << endl;
		return false;
	}
	return true;
}

string messageIDString(int messageID)
{
    switch(messageID)
    {
    case 0:
        return "HEARTBEAT";
    case 1:
        return "SYS_STATUS";
    case 2:
        return "SYSTEM_TIME";
    case 4:
        return "PING";
    case 5:
        return "CHANGE_OPERATOR_CONTROL";
    case 6:
        return "CHANGE_OPERATOR_CONTROL_ACK";
    case 7:
        return "AUTH_KEY";
    case 11:
        return "SET_MODE";
    case 20:
        return "PARAM_REQUEST_READ";
    case 21:
        return "PARAM_REQUEST_LIST";
    case 22:
        return "PARAM_VALUE";
    case 23:
        return "PARAM_SET";
    case 24:
        return "GPS_RAW_INT";
    case 25:
        return "GPS_STATUS";
    case 26:
        return "SCALED_IMU";
    case 27:
        return "RAW_IMU";
    case 28:
        return "RAW_PRESSURE";
    case 29:
        return "SCALED_PRESSURE";
    case 30:
        return "ATTITUDE";
    case 31:
        return "ATTITUDE_QUATERNION";
    case 32:
        return "LOCAL_POSITION_NED";
    case 33:
        return "GLOBAL_POSITION_INT";
    case 34:
        return "RC_CHANNELS_SCALED";
    case 35:
        return "RC_CHANNELS_RAW";
    case 36:
        return "SERVO_OUTPUT_RAW";
    case 37:
        return "MISSION_REQUEST_PARTIAL_LIST";
    case 38:
        return "MISSION_WRITE_PARTIAL_LIST";
    case 39:
        return "MISSION_ITEM";
    case 40:
        return "MISSION_REQUEST";
    case 41:
        return "MISSION_SET_CURRENT";
    case 42:
        return "MISSION_CURRENT";
    case 43:
        return "MISSION_REQUEST_LIST";
    case 44:
        return "MISSION_COUNT";
    case 45:
        return "MISSION_CLEAR_ALL";
    case 46:
        return "MISSION_ITEM_REACHED";
    case 47:
        return "MISSION_ACK";
    case 48:
        return "SET_GPS_GLOBAL_ORIGIN";
    case 49:
        return "GPS_GLOBAL_ORIGIN";
    case 54:
        return "SAFETY_SET_ALLOWED_AREA";
    case 55:
        return "SAFETY_ALLOWED_AREA";
    case 61:
        return "ATTITUDE_QUATERNION_COV";
    case 62:
        return "NAV_CONTROLLER_OUTPUT";
    case 63:
        return "GLOBAL_POSITION_INT_COV";
    case 64:
        return "LOCAL_POSITION_NED_COV";
    case 65:
        return "RC_CHANNELS";
    case 66:
        return "REQUEST_DATA_STREAM";
    case 67:
        return "DATA_STREAM";
    case 69:
        return "MANUAL_CONTROL";
    case 70:
        return "RC_CHANNELS_OVERRIDE";
    case 73:
        return "MISSION_ITEM_INT";
    case 74:
        return "VFR_HUD";
    case 75:
        return "COMMAND_INT";
    case 76:
        return "COMMAND_LONG";
    case 77:
        return "COMMAND_ACK";
    case 81:
        return "MANUAL_SETPOINT";
    case 82:
        return "SET_ATTITUDE_TARGET";
    case 83:
        return "ATTITUDE_TARGET";
    case 84:
        return "SET_POSITION_TARGET_LOCAL_NED";
    case 85:
        return "POSITION_TARGET_LOCAL_NED";
    case 86:
        return "SET_POSITION_TARGET_GLOBAL_INT";
    case 87:
        return "POSITION_TARGET_GLOBAL_INT";
    case 89:
        return "LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET";
    case 90:
        return "HIL_STATE";
    case 91:
        return "HIL_CONTROLS";
    case 92:
        return "HIL_RC_INPUTS_RAW";
    case 100:
        return "OPTICAL_FLOW";
    case 101:
        return "GLOBAL_VISION_POSITION_ESTIMATE";
    case 102:
        return "VISION_POSITION_ESTIMATE";
    case 103:
        return "VISION_SPEED_ESTIMATE";
    case 104:
        return "VICON_POSITION_ESTIMATE";
    case 105:
        return "HIGHRES_IMU";
    case 106:
        return "OMNIDIRECTIONAL_FLOW";
    case 107:
        return "HIL_SENSOR";
    case 108:
        return "SIM_STATE";
    case 109:
        return "RADIO_STATUS";
    case 110:
        return "FILE_TRANSFER_PROTOCOL";
    case 111:
    	return "TIMESYNC";
    case 113:
        return "HIL_GPS";
    case 114:
        return "HIL_OPTICAL_FLOW";
    case 115:
        return "HIL_STATE_QUATERNION";
    case 116:
        return "SCALED_IMU2";
    case 117:
        return "LOG_REQUEST_LIST";
    case 118:
        return "LOG_ENTRY";
    case 119:
        return "LOG_REQUEST_DATA";
    case 120:
        return "LOG_DATA";
    case 121:
        return "LOG_ERASE";
    case 122:
        return "LOG_REQUEST_END";
    case 123:
        return "GPS_INJECT_DATA";
    case 124:
        return "GPS2_RAW";
    case 125:
        return "POWER_STATUS";
    case 126:
        return "SERIAL_CONTROL";
    case 127:
        return "GPS_RTK";
    case 128:
        return "GPS2_RTK";
    case 130:
        return "DATA_TRANSMISSION_HANDSHAKE";
    case 131:
        return "ENCAPSULATED_DATA";
    case 132:
        return "DISTANCE_SENSOR";
    case 133:
        return "TERRAIN_REQUEST";
    case 134:
        return "TERRAIN_DATA";
    case 135:
        return "TERRAIN_CHECK";
    case 136:
        return "TERRAIN_REPORT";
    case 147:
        return "BATTERY_STATUS";
    case 148:
        return "AUTOPILOT_VERSION";
    case 151:
        return "SET_CAM_SHUTTER";
    case 152:
        return "IMAGE_TRIGGERED";
    case 153:
        return "IMAGE_TRIGGER_CONTROL";
    case 154:
        return "IMAGE_AVAILABLE";
    case 160:
        return "SET_POSITION_CONTROL_OFFSET";
    case 170:
        return "POSITION_CONTROL_SETPOINT";
    case 171:
        return "MARKER";
    case 172:
        return "RAW_AUX";
    case 180:
        return "WATCHDOG_HEARTBEAT";
    case 181:
        return "WATCHDOG_PROCESS_INFO";
    case 182:
        return "WATCHDOG_PROCESS_STATUS";
    case 183:
        return "WATCHDOG_COMMAND";
    case 190:
        return "PATTERN_DETECTED";
    case 191:
        return "POINT_OF_INTEREST";
    case 192:
        return "POINT_OF_INTEREST_CONNECTION";
    case 195:
        return "BRIEF_FEATURE";
    case 200:
        return "ATTITUDE_CONTROL";
    case 205:
        return "DETECTION_STATS";
    case 206:
        return "ONBOARD_HEALTH";
    case 248:
        return "V2_EXTENSION";
    case 249:
        return "MEMORY_VECT";
    case 250:
        return "DEBUG_VECT";
    case 251:
        return "NAMED_VALUE_FLOAT";
    case 252:
        return "NAMED_VALUE_INT";
    case 253:
        return "STATUSTEXT";
    case 254:
        return "DEBUG";
    default:
        return ""; // boost::lexical_cast<std::string>(messageID);


    }
}

string componentIDString(int componentID)
{
    switch(componentID)
    {
    default:
        return ""; // boost::lexical_cast<std::string>(componentID);
    }
}

int componentID(const char *componentID)
{
    return atoi(componentID);
}

string createDataString(uint8_t *bytes, int len)
{
	string s;
	char tmp[3];
    for(int i = 0; i < len; i++)
    {
        sprintf(tmp, "%02X", bytes[i]);
        s += tmp;
    }
    return s;
}

int parseDataString(const char *raw, uint8_t *bytes)
{
	int len = strlen(raw) / 2;
	uint8_t byte;
    for(int i = 0, j = 0; i < len; i++, j += 2)
    {
    	unsigned int byte;
        sscanf(&raw[j], "%02X", &byte);
        bytes[i] = byte;
    }
    return len;
}

void AMJSONUtils::headerToJson(
		Json::Value &value,
		std::string vehicleId,
		uint16_t sequence,
		// boost::posix_time::ptime timestamp,
		int64_t timestamp,
		std::string messageName,
		std::string sender,
		std::string type)
{
    value["head"]["veh"] = vehicleId;
    value["head"]["seq"] = sequence;

    // TODO: make this local time (with timezone offset) for easier reading
    // value["head"]["time"] = TimeUtil::printTime(timestamp);
    value["head"]["time"] = (Json::Value::Int64)timestamp;

    value["head"]["name"] = messageName;

    // remove leading '/' from node name
    const char *from = sender.c_str();
    if(from[0] == '/')
    {
    	from++;
    }
    value["head"]["from"] = from;
    value["head"]["type"] = type;
}

void AMJSONUtils::jsonToCtrlVxCmnd(
		Json::Value &value,
		brain_box_msgs::msg::BBVxCommand &msg)
{
	if(value["data"]["command"] == "COMMAND_START")
	{
		msg.command = brain_box_msgs::msg::BBVxCommand::COMMAND_START;
	}
	else if(value["data"]["command"] == "COMMAND_STOP")
	{
		msg.command = brain_box_msgs::msg::BBVxCommand::COMMAND_STOP;
	}
	else if(value["data"]["command"] == "COMMAND_TRACK_TARGET")
	{
		msg.command = brain_box_msgs::msg::BBVxCommand::COMMAND_TRACK_TARGET;
		msg.target_id = value["data"]["target_id"].asString();
		if(value["data"]["frame_of_reference"] == "FOR_TARGET")
		{
			msg.frame_of_reference = brain_box_msgs::msg::BBVxCommand::FOR_TARGET;
		}
		else if(value["data"]["frame_of_reference"] == "FOR_GLOBAL")
		{
			msg.frame_of_reference = brain_box_msgs::msg::BBVxCommand::FOR_GLOBAL;
		}
		else
		{
			msg.frame_of_reference = brain_box_msgs::msg::BBVxCommand::FOR_VEHICLE;
		}
		msg.kinematics.position.x = value["data"]["kinematics"]["position"]["x"].asDouble();
		msg.kinematics.position.y = value["data"]["kinematics"]["position"]["y"].asDouble();
		msg.kinematics.position.z = value["data"]["kinematics"]["position"]["z"].asDouble();
	}
	else
	{
		std::cout << YELLOW << "bad vx command: " << value["data"]["command"] << COLOR_RESET << std::endl;
	}
}

void AMJSONUtils::jsonToCtrlVxPid_param(
		Json::Value &value,
		brain_box_msgs::msg::PIDParam &msg)
{
	if(value["data"]["axis"] == "X")
	{
		msg.axis = 0;
	}
	else if(value["data"]["axis"] == "Y")
	{
		msg.axis = 1;
	}
	else if(value["data"]["axis"] == "Z")
	{
		msg.axis = 2;
	}
	else if(value["data"]["axis"] == "ALL")
	{
		msg.axis = 3;
	}
	msg.pid_i = value["data"]["i"].asDouble();
	msg.pid_d = value["data"]["d"].asDouble();
	msg.pid_p = value["data"]["p"].asDouble();
	msg.i_max = value["data"]["iMax"].asDouble();
	msg.i_min = value["data"]["iMin"].asDouble();
	msg.max_error = value["data"]["maxError"].asDouble();
	msg.max_set_point = value["data"]["maxSetpoint"].asDouble();
	msg.land_error = value["data"]["landError"].asDouble();
	msg.loiter_time = value["data"]["loiterTime"].asDouble();
	msg.land_speed = value["data"]["landSpeed"].asDouble();
}

void AMJSONUtils::jsonToCtrlVideoCmnd(
		Json::Value &value,
		brain_box_msgs::msg::VideoCmnd &msg)
{
	if(value["data"]["command"] == "COMMAND_RECORD_STILLS_START")
	{
		msg.command = brain_box_msgs::msg::VideoCmnd::COMMAND_RECORD_STILLS_START;
	}
	else if(value["data"]["command"] == "COMMAND_RECORD_STILLS_STOP")
	{
		msg.command = brain_box_msgs::msg::VideoCmnd::COMMAND_RECORD_STILLS_STOP;
	}
	else if(value["data"]["command"] == "COMMAND_STREAM_VIDEO_START")
	{
		msg.command = brain_box_msgs::msg::VideoCmnd::COMMAND_STREAM_VIDEO_START;
	}
	else if(value["data"]["command"] == "COMMAND_STREAM_VIDEO_STOP")
	{
		msg.command = brain_box_msgs::msg::VideoCmnd::COMMAND_STREAM_VIDEO_STOP;
	}
	else if(value["data"]["command"] == "COMMAND_RECORD_VIDEO_START")
	{
		msg.command = brain_box_msgs::msg::VideoCmnd::COMMAND_RECORD_VIDEO_START;
	}
	else if(value["data"]["command"] == "COMMAND_RECORD_VIDEO_STOP")
	{
		msg.command = brain_box_msgs::msg::VideoCmnd::COMMAND_RECORD_VIDEO_STOP;
	}
	else
	{
		std::cout << YELLOW << "bad vision command: " << value["data"]["command"] << COLOR_RESET << std::endl;
	}
}

void AMJSONUtils::jsonToCtrlVisionCmnd(
		Json::Value &value,
		brain_box_msgs::msg::VisionCmnd &msg)
{
	msg.marker_configs.clear();
    for (const Json::Value& json_marker_config : value["marker_configs"])
    {
    	brain_box_msgs::msg::MarkerConfig marker_config;
		marker_config.type.type = json_marker_config["type"]["type"].asString();
		marker_config.image_pixels_k = json_marker_config["image_pixels_k"].asUInt();
	    for (const Json::Value& json_family : json_marker_config["families"])
	    {
	    	brain_box_msgs::msg::MarkerConfigFamily marker_config_family;
	    	marker_config_family.family_name = json_family["family_name"].asString();
	    	marker_config_family.marker_size_m = json_family["marker_size_m"].asFloat();
	    	marker_config.families.push_back(marker_config_family);
	    }
	    msg.marker_configs.push_back(marker_config);
    }
}

void AMJSONUtils::jsonToRemoteCtrlVxMission(
		Json::Value &value,
		brain_box_msgs::msg::VxMissionExecCmd &msg)
{
	// TODO: figure way to do reverse mapping in mission_cmd_type.cpp
	if(value["data"]["command"]["key"] == "EXECUTE")
	{
		msg.command.key = MISSION_KEY(MISSION_CMD::EXECUTE);
	}
	else if(value["data"]["command"]["key"] == "RESUME")
	{
		msg.command.key = MISSION_KEY(MISSION_CMD::RESUME);
	}
	else if(value["data"]["command"]["key"] == "ABORT")
	{
		msg.command.key = MISSION_KEY(MISSION_CMD::ABORT);
	}
	else if(value["data"]["command"]["key"] == "PAUSE")
	{
		msg.command.key = MISSION_KEY(MISSION_CMD::PAUSE);
	}
	else
	{
		std::cout << YELLOW << "bad mission command: " << value["data"]["command"]["key"] << COLOR_RESET << std::endl;
	}
	msg.command.value = value["data"]["command"]["value"].asString();

}

void AMJSONUtils::jsonToLogControl(
		Json::Value &value,
		brain_box_msgs::msg::LogControl &msg)
{
	// TODO: figure way to do reverse mapping in mission_cmd_type.cpp
	msg.enable = value["data"]["enable"].asBool();

	if(value["data"]["level"] == "OFF")
	{
//		msg.level = BagLogger::BagLoggerLevel::OFF;
	}
	else if(value["data"]["level"] == "NORM")
	{
//			msg.level = BagLogger::BagLoggerLevel::NORM;
	}
	else if(value["data"]["level"] == "FINE")
	{
//			msg.level = BagLogger::BagLoggerLevel::FINE;
	}
	else if(value["data"]["level"] == "EXTRA")
	{
//			msg.level = BagLogger::BagLoggerLevel::EXTRA;
	}
	else if(value["data"]["level"] == "ALL")
	{
//			msg.level = BagLogger::BagLoggerLevel::ALL;
	}

	msg.name = value["data"]["name"].asString();
}

bool AMJSONUtils::jsonToAMMessageHead(Json::Value &value, AMMessage &msg)
{
	AMMsgHead *msgHead = msg.getHead();
	Json::Value valueHead = value["head"];
	msgHead->setVeh(valueHead["veh"].asString());
	msgHead->setSeq(valueHead["seq"].asInt());
	// std::string timeString = valueHead["time"].asString();
	// timeString = timeString.substr(0, timeString.length() - 1);
	// if(timeString.length() < 11)
	// {
	// 	return false;
	// }
	// timeString.replace(10, 1, " ");
	// boost::posix_time::ptime time;
	// try
	// {
	// 	time = boost::posix_time::time_from_string(timeString);
	// }
	// catch(...)
	// {
	// 	std::cout << YELLOW << "bad date in message: " << timeString << COLOR_RESET << std::endl;
	// }
	// msgHead->setTime(time);
	msgHead->setName(valueHead["name"].asString());
	msgHead->setFrom(valueHead["from"].asString());
	msgHead->setType(valueHead["type"].asString());
	return true;
}


// void AMJSONUtils::poseStampedToJson(
// 		Json::Value &value,
// 		std::string vehicleId,
// 		uint16_t sequence,
// 		boost::posix_time::ptime timestamp,
// 		std::string messageName,
// 		std::string sender,
// 		const geometry_msgs::msg::PoseStamped &msg)
// {

// 	// headerToJson(value, vehicleId, sequence, timestamp, messageName, sender, AMMessage::TYPE_AUTOMOD);

//     value["data"]["position"]["x"] = msg.pose.position.x;
//     value["data"]["position"]["y"] = msg.pose.position.y;
//     value["data"]["position"]["z"] = msg.pose.position.z;
//     value["data"]["orientation"]["x"] = msg.pose.orientation.x;
//     value["data"]["orientation"]["y"] = msg.pose.orientation.y;
//     value["data"]["orientation"]["z"] = msg.pose.orientation.z;
//     value["data"]["orientation"]["w"] = msg.pose.orientation.w;
// }

// void AMJSONUtils::bbPoseToJson(
// 		Json::Value &value,
// 		const brain_box_msgs::msg::TargetPose &msg)
// {
//     value["confidence"] = msg.confidence;
//     value["pose"]["position"]["x"] = msg.pose.position.x;
//     value["pose"]["position"]["y"] = msg.pose.position.y;
//     value["pose"]["position"]["z"] = msg.pose.position.z;
//     value["pose"]["orientation"]["x"] = msg.pose.orientation.x;
//     value["pose"]["orientation"]["y"] = msg.pose.orientation.y;
//     value["pose"]["orientation"]["z"] = msg.pose.orientation.z;
//     value["pose"]["orientation"]["w"] = msg.pose.orientation.w;
// //    value["latency"]["capture_stamp"] = printTime(msg.latency.capture_stamp.toBoost());
// //    value["latency"]["transfer_stamp"] = printTime(msg.latency.transfer_stamp.toBoost());
// //    value["latency"]["driver_stamp"] = printTime(msg.latency.driver_stamp.toBoost());
// //    value["latency"]["read_stamp"] = printTime(msg.latency.read_stamp.toBoost());
// //    value["latency"]["image_pipeline_stamp0"] = printTime(msg.latency.image_pipeline_stamp0.toBoost());
// //    value["latency"]["image_pipeline_stamp1"] = printTime(msg.latency.image_pipeline_stamp1.toBoost());
// //    value["latency"]["image_pipeline_stamp2"] = printTime(msg.latency.image_pipeline_stamp2.toBoost());
// //    value["latency"]["image_pipeline_stamp3"] = printTime(msg.latency.image_pipeline_stamp3.toBoost());
// //    value["latency"]["image_pipeline_stamp4"] = printTime(msg.latency.image_pipeline_stamp4.toBoost());
// //    value["latency"]["smart_pilot_stamp0"] = printTime(msg.latency.smart_pilot_stamp0.toBoost());
// //    value["latency"]["smart_pilot_stamp1"] = printTime(msg.latency.smart_pilot_stamp1.toBoost());
// //    value["latency"]["mavros_stamp"] = printTime(msg.latency.mavros_stamp.toBoost());
//     value["target_id"] = msg.target_id;
// 	if(msg.outline.size() == 0)
// 	{
// 		value["outline"] = Json::Value(Json::arrayValue);
// 	}
// 	else
// 	{
// 		Json::Value& outlineArray = value["outline"];
// 		for(int i = 0; i < msg.outline.size(); i++)
// 		{
// 			Json::Value outlineValue;
// 			outlineValue["x"] = msg.outline.at(i).x;
// 			outlineValue["y"] = msg.outline.at(i).y;
// 			outlineArray.append(outlineValue);
// 		}
// 	}
// }

// void AMJSONUtils::throttleToJson(
// 		Json::Value &value,
// 		std::string vehicleId,
// 		uint16_t sequence,
// 		boost::posix_time::ptime timestamp,
// 		std::string messageName,
// 		std::string sender,
// 		const std_msgs::msg::Float64 &msg)
// {
// 	// headerToJson(value, vehicleId, sequence, timestamp, messageName, sender, AMMessage::TYPE_AUTOMOD);

//     value["data"]["throttle"] = msg.data;
// }


bool AMJSONUtils::stringToFTPRequest(std::string jsonString, FTPRequest &request)
{
	Json::Value value;
	Json::Reader reader;
	if(!reader.parse( jsonString, value ))
	{
		return false;
	}

	request.setUuid(value["uuid"].asString());

	if(value["command"] == "DIRECTORY")
	{
		request.setCommand(FTPRequest::Command::DIRECTORY);
	}
	else if(value["command"] == "SEND")
	{
		request.setCommand(FTPRequest::Command::SEND);

	}
	else if(value["command"] == "RECEIVE")
	{
		request.setCommand(FTPRequest::Command::RECEIVE);
	}
	request.setVxId(value["vxID"].asString());
	return true;
}

void AMJSONUtils::ftpResponseToString(FTPResponse response, std::string &jsonString)
{
	Json::Value value;
	value.clear();

    value["uuid"] = response.getUuid();
    switch(response.getType())
    {
    case FTPResponse::MessageType::VXID:
    	value["type"] = "VXID";
    	break;
    case FTPResponse::MessageType::DIRECTORY_LISTING:
    default:
    	value["type"] = "DIRECTORY_LISTING";
    	break;
    }
    value["vxID"] = response.getVxId();
    value["date"] = TimeUtil::printTime(response.getDate());
    int i = 0;
    for(std::vector<FileSpec>::const_iterator it = response.getFileSpecs().begin();
    	it != response.getFileSpecs().end(); it++, i++)
    {
    	switch(it->getDir())
    	{
    	case FileSpec::PARTS:
    		value["fileSpecs"][i]["dir"] = "PARTS";
    		break;
    	case FileSpec::LOGS:
    		value["fileSpecs"][i]["dir"] = "LOGS";
    		break;
    	case FileSpec::PHOTOS:
    		value["fileSpecs"][i]["dir"] = "PHOTOS";
    		break;
    	case FileSpec::VIDEOS:
    		value["fileSpecs"][i]["dir"] = "VIDEOS";
    		break;
    	}
    	switch(it->getType())
    	{
    	case FileSpec::FLIGHT_PLAN:
    		value["fileSpecs"][i]["type"] = "FLIGHT_PLAN";
    		break;
    	case FileSpec::LOG:
    		value["fileSpecs"][i]["type"] = "LOG";
    		break;
    	case FileSpec::MISSION_SPEC:
    		value["fileSpecs"][i]["type"] = "MISSION_SPEC";
    		break;
    	case FileSpec::PHOTO:
    		value["fileSpecs"][i]["type"] = "PHOTO";
    		break;
    	case FileSpec::SCRIPT:
    		value["fileSpecs"][i]["type"] = "SCRIPT";
    		break;
    	case FileSpec::PART:
    		value["fileSpecs"][i]["type"] = "PART";
    		break;
    	case FileSpec::VIDEO:
    		value["fileSpecs"][i]["type"] = "VIDEO";
    		break;
    	case FileSpec::FILE:
    		value["fileSpecs"][i]["type"] = "FILE";
    		break;
    	case FileSpec::DIRECTORY:
    		value["fileSpecs"][i]["type"] = "DIRECTORY";
    		break;
    	}
    	value["fileSpecs"][i]["name"] = it->getName();
    	value["fileSpecs"][i]["md5"] = it->getMd5();
    	value["fileSpecs"][i]["date"] = TimeUtil::printTime(it->getDate());
    	value["fileSpecs"][i]["size"] = it->getSize();
    }

    Json::FastWriter writer;
	jsonString = writer.write( value );
}


std::string spStateIntToString(int state)
{
	 switch(state)
	 {
	 case brain_box_msgs::msg::SPStatus::STATE_LANDED:
		   return "LANDED";
	 case brain_box_msgs::msg::SPStatus::STATE_TAKING_OFF:
		 return "TAKING_OFF";
	 case brain_box_msgs::msg::SPStatus::STATE_LOITER:
		   return "LOITER";
	 case brain_box_msgs::msg::SPStatus::STATE_TRACKING:
		   return "TRACKING";
	 case brain_box_msgs::msg::SPStatus::STATE_LANDING:
		   return "LANDING";
	 }
	 return "";
}

// "header:
//   stamp:
//     sec: 0
//     nanosec: 0
//   frame_id: ''
// child_frame_id: ''
// pose:
//   pose:
//     position:
//       x: 0.0
//       y: 0.0
//       z: 0.0
//     orientation:
//       x: 0.0
//       y: 0.0
//       z: 0.0
//       w: 1.0
//   covariance:
//   - 0.0
//   - 0.0
//   - 0.0
//   - 0.0
//   - 0.0
//   - 0.0
//   - 0.0
//   - 0.0
//   - 0.0"

static const uint64_t NANOSECS_IN_SEC = 1000000000L;

Json::Value AMJSONUtils::odometryToJson(
		std::string vehicleId,
		uint16_t sequence,
		int64_t timestamp,
		std::string sender,
		const nav_msgs::msg::Odometry::SharedPtr msg)
{
	 Json::Value value;
	 value.clear();

	headerToJson(value, vehicleId, sequence, timestamp, AMMessage::NAME_ODOMETRY, sender, AMMessage::TYPE_ROS2);

    // uint64_t seconds = timestamp / NANOSECS_IN_SEC;
    // uint64_t nanoseconds = timestamp - seconds * NANOSECS_IN_SEC;
	value["data"]["head"]["stamp"]["sec"] = msg->header.stamp.sec;
	value["data"]["head"]["stamp"]["nsec"] = msg->header.stamp.nanosec;
	value["data"]["head"]["frame"] = msg->header.frame_id;
	value["data"]["child"] = msg->child_frame_id;

	value["data"]["pose"]["pose"]["pos"]["x"] = msg->pose.pose.position.x;
	value["data"]["pose"]["pose"]["pos"]["y"] = msg->pose.pose.position.y;
	value["data"]["pose"]["pose"]["pos"]["z"] = msg->pose.pose.position.z;
	value["data"]["pose"]["pose"]["orien"]["x"] = msg->pose.pose.orientation.x;
	value["data"]["pose"]["pose"]["orien"]["y"] = msg->pose.pose.orientation.y;
	value["data"]["pose"]["pose"]["orien"]["z"] = msg->pose.pose.orientation.z;
	value["data"]["pose"]["pose"]["orien"]["w"] = msg->pose.pose.orientation.w;
	Json::Value& poseCovarianceArray = value["data"]["pose"]["covar"];
	for(int i = 0; i < 36; i++)
	{
		Json::Value elementValue = msg->pose.covariance[i];
		poseCovarianceArray.append(elementValue);
	}

	value["data"]["twist"]["twist"]["lin"]["x"] = msg->twist.twist.linear.x;
	value["data"]["twist"]["twist"]["lin"]["y"] = msg->twist.twist.linear.y;
	value["data"]["twist"]["twist"]["lin"]["z"] = msg->twist.twist.linear.z;
	value["data"]["twist"]["twist"]["ang"]["x"] = msg->twist.twist.angular.x;
	value["data"]["twist"]["twist"]["ang"]["y"] = msg->twist.twist.angular.y;
	value["data"]["twist"]["twist"]["ang"]["z"] = msg->twist.twist.angular.z;
	Json::Value& twistCovarianceArray = value["data"]["twist"]["covar"];
	for(int i = 0; i < 36; i++)
	{
		Json::Value elementValue = msg->twist.covariance[i];
		twistCovarianceArray.append(elementValue);
	}

	 return value;
}

// header:
//   stamp:
//     sec: 1682638776
//     nanosec: 775860224
//   frame_id: Asset_ENU
// feature_id: Asset_ENU
// position:
//   x: -0.09929207898676087
//   y: -0.039304064592709076
//   z: 0.13110816089508914
// yaw_angle: -0.033249409100654805
// perceived: true
// feature_end: false
// confidence:
// - 1.0
// - 1.0
// - 1.0
// - 1.0

Json::Value AMJSONUtils::vxFeaturePoseToJson(
		std::string vehicleId,
		uint16_t sequence,
		int64_t timestamp,
		std::string sender,
		const brain_box_msgs::msg::VxFeaturePose::SharedPtr msg)
{
	 Json::Value value;
	 value.clear();

	headerToJson(value, vehicleId, sequence, timestamp, AMMessage::NAME_VX_FEATURE_POSE, sender, AMMessage::TYPE_ROS2);

    // uint64_t seconds = timestamp / NANOSECS_IN_SEC;
    // uint64_t nanoseconds = timestamp - seconds * NANOSECS_IN_SEC;
	value["data"]["head"]["stamp"]["sec"] = msg->header.stamp.sec;
	value["data"]["head"]["stamp"]["nsec"] = msg->header.stamp.nanosec;
	value["data"]["head"]["frame"] = msg->header.frame_id;
	value["data"]["feature_id"] = msg->feature_id;
	value["data"]["position"]["x"] = msg->position.x;
	value["data"]["position"]["y"] = msg->position.y;
	value["data"]["position"]["z"] = msg->position.z;
	value["data"]["yaw_angle"] = msg->yaw_angle;
	value["data"]["perceived"] = msg->perceived;
	value["data"]["feature_end"] = msg->feature_end;
	Json::Value& poseCovarianceArray = value["data"]["confidence"];
	for(int i = 0; i < 4; i++)
	{
		Json::Value elementValue = msg->confidence[i];
		poseCovarianceArray.append(elementValue);
	}

	 return value;
}

// Json::Value AMJSONUtils::statusVisionToJson(
// 		std::string vehicleId,
// 		uint16_t sequence,
// 		boost::posix_time::ptime timestamp,
// 		std::string sender,
// 		const brain_box_msgs::msg::VisionStatus &msg)
// {
// 	Json::Value value;
// 	value.clear();

// 	// headerToJson(value, vehicleId, sequence, timestamp,
// 	// 	AMMessage::NAME_STATUS_VISION, sender, AMMessage::TYPE_AUTOMOD);

// 	value["data"]["status"]["summary"] = MessageUtil::statusIntToString(msg.status.summary);
// 	value["data"]["fps_ave"] = msg.fps_ave;
// 	value["data"]["fps_min"] = msg.fps_min;
// 	value["data"]["fps_max"] = msg.fps_max;
// 	value["data"]["latency_ave_ms"] = msg.latency_ave_ms;
// 	value["data"]["latency_min_ms"] = msg.latency_min_ms;
// 	value["data"]["latency_max_ms"] = msg.latency_max_ms;
// 	value["data"]["native_image_width"] = msg.native_image_width;
// 	value["data"]["native_image_width"] = msg.native_image_width;
// 	Json::Value& poseArray = value["data"]["marker_statuses"];
// 	for(brain_box_msgs::msg::MarkerStatus marker_status: msg.marker_statuses)
// 	{
// 		Json::Value elementValue;
// 		if(marker_status.type.type == brain_box_msgs::msg::MarkerType::TYPE_APRILTAGS)
// 		{
// 			elementValue["type"]["type"] = "APRILTAGS";
// 		}
// 		else if(marker_status.type.type == brain_box_msgs::msg::MarkerType::TYPE_ARUCO)
// 		{
// 			elementValue["type"]["type"] = "ARUCO";
// 		}
// 		elementValue["family_name"] = marker_status.family_name;
// 		elementValue["marker_size_m"] = marker_status.marker_size_m;
// 		elementValue["image_pixels_k"] = marker_status.image_pixels_k;
// 		elementValue["image_scale"] = marker_status.image_scale;
// 		elementValue["image_width"] = marker_status.image_width;
// 		elementValue["image_height"] = marker_status.image_height;
// 		poseArray.append(elementValue);
// 	}

// 	return value;
// }

// std::string operatorMsgTypeToString(int type)
// {
// 	 switch(type)
// 	 {
// 	 case brain_box_msgs::msg::OperatorMsg::INFO:
// 		   return "INFO";
// 	 case brain_box_msgs::msg::OperatorMsg::WARNING:
// 		 return "WARNING";
// 	 case brain_box_msgs::msg::OperatorMsg::OP_ERROR:
// 		   return "ERROR";
// 	 case brain_box_msgs::msg::OperatorMsg::CRITICAL:
// 		   return "CRITICAL";
// 	 case brain_box_msgs::msg::OperatorMsg::SYSTEM:
// 		   return "SYSTEM";
// 	 }
// 	 return "";
// }

// Json::Value AMJSONUtils::ctrlVxMissionMessageToJson(
// 		std::string vehicleId,
// 		uint16_t sequence,
// 		boost::posix_time::ptime timestamp,
// 		std::string sender,
// 		const brain_box_msgs::msg::OperatorMsg &msg)
// {
// 	 Json::Value value;
// 	 value.clear();

// 	//  headerToJson(value, vehicleId, sequence, timestamp, AMMessage::NAME_CTRL_VX_MISSION_MESSAGE, sender, AMMessage::TYPE_AUTOMOD);

// 	 value["data"]["type"] = operatorMsgTypeToString(msg.type);
// 	 value["data"]["message"] = msg.message;

// 	 return value;
// }

// std::string logLevelToString(int type)
// {
// 	 switch(type)
// 	 {
// 	 case rcl_interfaces::msg::Log::DEBUG:
// 		   return "DEBUG";
// 	 case rcl_interfaces::msg::Log::INFO:
// 		   return "INFO";
// 	 case rcl_interfaces::msg::Log::WARN:
// 		   return "WARN";
// 	 case rcl_interfaces::msg::Log::ERROR:
// 		   return "ERROR";
// 	 case rcl_interfaces::msg::Log::FATAL:
// 		   return "FATAL";
// 	 }
// 	 return "";
// }

// Json::Value AMJSONUtils::rosoutToJson(
// 		std::string vehicleId,
// 		uint16_t sequence,
// 		boost::posix_time::ptime timestamp,
// 		std::string sender,
// 		const rcl_interfaces::msg::Log &msg)
// {
// 	 Json::Value value;
// 	 value.clear();

// 	//  headerToJson(value, vehicleId, sequence, timestamp, AMMessage::NAME_ROSOUT, sender, AMMessage::TYPE_AUTOMOD);

// 	 if(msg.level >= rcl_interfaces::msg::Log::INFO)
// 	 {
// 		 value["data"]["level"] = logLevelToString(msg.level);
// 		 value["data"]["msg"] = msg.msg;
// 	 }

// 	 return value;
// }

// Json::Value AMJSONUtils::vstateSummaryToJson(
// 		std::string vehicleId,
// 		uint16_t sequence,
// 		boost::posix_time::ptime timestamp,
// 		std::string sender,
// 		const brain_box_msgs::msg::VxState &msg)
// {
// 	 Json::Value value;
// 	 value.clear();

// 	//  headerToJson(value, vehicleId, sequence, timestamp, AMMessage::NAME_VSTATE_SUMMARY, sender, AMMessage::TYPE_AUTOMOD);

// 	 value["data"]["gps_latitude"] = msg.gps_latitude;
// 	 value["data"]["gps_longitude"] = msg.gps_longitude;
// 	 value["data"]["gps_altitude"] = msg.gps_altitude;
// 	 value["data"]["hdop_m"] = msg.hdop_m;
// 	 value["data"]["vdop_m"] = msg.vdop_m;
// 	 value["data"]["alt_ato_m"] = msg.alt_ato_m;
// 	 value["data"]["height_agl_m"] = msg.height_agl_m;
// 	 value["data"]["heading_deg"] = msg.heading_deg;
// 	 value["data"]["pitch_deg"] = msg.pitch_deg;
// 	 value["data"]["roll_deg"] = msg.roll_deg;
// 	 value["data"]["gimbal_yaw_deg"] = msg.gimbal_yaw_deg;
// 	 value["data"]["gimbal_pitch_deg"] = msg.gimbal_pitch_deg;
// 	 value["data"]["gimbal_roll_deg"] = msg.gimbal_roll_deg;
// 	 value["data"]["h_speed_mps"] = msg.h_speed_mps;
// 	 value["data"]["v_speed_mps"] = msg.v_speed_mps;
// 	 value["data"]["log_level"] = msg.log_level;
// 	 value["data"]["log_duration"] = msg.log_duration;
// 	 value["data"]["fp_feature_id"] = msg.fp_feature_id;
// 	 value["data"]["fp_in_view"] = msg.fp_in_view == 0 ? false : true;
// 	 value["data"]["fp_x"] = msg.fp_x;
// 	 value["data"]["fp_y"] = msg.fp_y;
// 	 value["data"]["fp_z"] = msg.fp_z;

// 	 return value;
// }

// Json::Value AMJSONUtils::statusVideoToJson(
// 		std::string vehicleId,
// 		uint16_t sequence,
// 		boost::posix_time::ptime timestamp,
// 		std::string sender,
// 		const brain_box_msgs::msg::VideoStatus &msg)
// {
// 	 Json::Value value;
// 	 value.clear();

// 	//  headerToJson(value, vehicleId, sequence, timestamp, AMMessage::NAME_STATUS_VIDEO, sender, AMMessage::TYPE_AUTOMOD);

//     value["data"]["status"]["summary"] = MessageUtil::statusIntToString(msg.status.summary);
//     value["data"]["fps_ave"] = msg.fps_ave;
//     value["data"]["fps_min"] = msg.fps_min;
//     value["data"]["fps_max"] = msg.fps_max;
//     value["data"]["stream_bitrate"] = msg.stream_bitrate;
//     value["data"]["stream_fps"] = msg.stream_fps;
//     value["data"]["stream_width"] = msg.stream_width;
//     value["data"]["stream_height"] = msg.stream_height;
//     value["data"]["recording_stills"] = msg.recording_stills == 0 ? false : true;
//     value["data"]["recording_video"] = msg.recording_video == 0 ? false : true;
//     value["data"]["streaming_video"] = msg.streaming_video == 0 ? false : true;

//     return value;
// }

// Json::Value AMJSONUtils::statusSuperToJson(
// 		std::string vehicleId,
// 		uint16_t sequence,
// 		boost::posix_time::ptime timestamp,
// 		std::string sender,
// 		const brain_box_msgs::msg::SuperStatus &msg)
// {
// 	Json::Value value;
// 	value.clear();

// 	// headerToJson(value, vehicleId, sequence, timestamp, AMMessage::NAME_STATUS_SUPER, sender, AMMessage::TYPE_AUTOMOD);

// 	// value["data"]["status"]["summary"] = MessageUtil::statusIntToString(msg.status.summary);
// 	// value["data"]["status"]["lastContact"] = TimeUtil::printTime(msg.status.last_contact.toBoost());

// 	// value["data"]["c2_status"]["summary"] = MessageUtil::statusIntToString(msg.c2_status.summary);
// 	// value["data"]["c2_status"]["lastContact"] = TimeUtil::printTime(msg.c2_status.last_contact.toBoost());
// 	// value["data"]["comms_status"]["summary"] = MessageUtil::statusIntToString(msg.comms_status.summary);
// 	// value["data"]["comms_status"]["lastContact"] = TimeUtil::printTime(msg.comms_status.last_contact.toBoost());
// 	// value["data"]["corr_status"]["summary"] = MessageUtil::statusIntToString(msg.corr_status.summary);
// 	// value["data"]["corr_status"]["lastContact"] = TimeUtil::printTime(msg.corr_status.last_contact.toBoost());
// 	// value["data"]["lidar_status"]["summary"] = MessageUtil::statusIntToString(msg.lidar_status.summary);
// 	// value["data"]["lidar_status"]["lastContact"] = TimeUtil::printTime(msg.lidar_status.last_contact.toBoost());
// 	// value["data"]["ss1_status"]["summary"] = MessageUtil::statusIntToString(msg.ss1_status.summary);
// 	// value["data"]["ss1_status"]["lastContact"] = TimeUtil::printTime(msg.ss1_status.last_contact.toBoost());
// 	// value["data"]["ss2_status"]["summary"] = MessageUtil::statusIntToString(msg.ss2_status.summary);
// 	// value["data"]["ss2_status"]["lastContact"] = TimeUtil::printTime(msg.ss2_status.last_contact.toBoost());
// 	// value["data"]["vislog_status"]["summary"] = MessageUtil::statusIntToString(msg.vislog_status.summary);
// 	// value["data"]["vislog_status"]["lastContact"] = TimeUtil::printTime(msg.vislog_status.last_contact.toBoost());


// 	value["data"]["cpu_percent_total"] = msg.cpu_percent_total;
// 	Json::Value& poseArray = value["data"]["cpu_percent_cores"];
// 	for(uint8_t cpu_percent: msg.cpu_percent_cores)
// 	{
// 		Json::Value elementValue = cpu_percent;
// 		poseArray.append(elementValue);
// 	}

// 	return value;
// }

// Json::Value AMJSONUtils::statusC2ToJson(
// 		std::string vehicleId,
// 		uint16_t sequence,
// 		boost::posix_time::ptime timestamp,
// 		std::string sender,
// 		const brain_box_msgs::msg::C2Status &msg)
// {
// 	 Json::Value value;
// 	 value.clear();

// 	//  headerToJson(value, vehicleId, sequence, timestamp, AMMessage::NAME_STATUS_C2, sender, AMMessage::TYPE_AUTOMOD);

// 	 value["data"]["status"]["summary"] = MessageUtil::statusIntToString(msg.status.summary);

//      return value;
// }

// Json::Value AMJSONUtils::statusVisLogToJson(
// 		std::string vehicleId,
// 		uint16_t sequence,
// 		boost::posix_time::ptime timestamp,
// 		std::string sender,
// 		const brain_box_msgs::msg::VisLogStatus &msg)
// {
// 	 Json::Value value;
// 	 value.clear();

// 	//  headerToJson(value, vehicleId, sequence, timestamp, AMMessage::NAME_STATUS_VISLOG, sender, AMMessage::TYPE_AUTOMOD);

// 	 value["data"]["status"]["summary"] = MessageUtil::statusIntToString(msg.status.summary);
// 	 value["data"]["enabled"] = msg.enabled == 0 ? false : true;

// 	 switch(msg.level)
// 	 {
// //	 case BagLogger::BagLoggerLevel::NORM:
// //		value["data"]["level"] = "NORM";
// //		break;
// //	 case BagLogger::BagLoggerLevel::FINE:
// //		value["data"]["level"] = "FINE";
// //		break;
// //	 case BagLogger::BagLoggerLevel::EXTRA:
// //		value["data"]["level"] = "EXTRA";
// //		break;
// //	 case BagLogger::BagLoggerLevel::ALL:
// //		value["data"]["level"] = "ALL";
// //		break;
// //	 default:
// //		value["data"]["level"] = "OFF";
// //		break;
// 	 }

//      return value;
// }

// Json::Value AMJSONUtils::statusFTPToJson(
// 		std::string vehicleId,
// 		uint16_t sequence,
// 		boost::posix_time::ptime timestamp,
// 		std::string sender,
// 		const brain_box_msgs::msg::FTPStatus &msg)
// {
// 	 Json::Value value;
// 	 value.clear();

// 	//  headerToJson(value, vehicleId, sequence, timestamp, AMMessage::NAME_STATUS_FTP, sender, AMMessage::TYPE_AUTOMOD);

// 	 value["data"]["status"]["summary"] = MessageUtil::statusIntToString(msg.status.summary);
// 	 value["data"]["opens"]["summary"] = msg.opens;
// 	 value["data"]["failures"]["summary"] = msg.failures;
// 	 value["data"]["timeouts"]["summary"] = msg.timeouts;
// 	 value["data"]["closes"]["summary"] = msg.closes;
// 	 value["data"]["messages"]["summary"] = msg.messages;

//      return value;
// }

// Json::Value AMJSONUtils::statusDepthToJson(
// 		std::string vehicleId,
// 		uint16_t sequence,
// 		boost::posix_time::ptime timestamp,
// 		std::string sender,
// 		const brain_box_msgs::msg::DepthStatus &msg)
// {
// 	Json::Value value;
// 	value.clear();

// 	// headerToJson(value, vehicleId, sequence, timestamp, AMMessage::NAME_STATUS_DEPTH, sender, AMMessage::TYPE_AUTOMOD);

// 	value["data"]["status"]["summary"] = MessageUtil::statusIntToString(msg.status.summary);
// 	value["data"]["simulation"] = msg.simulation;
// 	value["data"]["enabled"] = msg.enabled;
// 	value["data"]["odometry_z_m"] = msg.odometry_z_m;
// 	value["data"]["agl_m"] = msg.agl_m;
// 	Json::Value& poseArray = value["data"]["guidance_statuses"];
// 	for(brain_box_msgs::msg::GuidanceStatus guidance_status: msg.guidance_statuses)
// 	{
// 		Json::Value elementValue;
// 		elementValue["name"] = guidance_status.name;
// 		elementValue["active"] = guidance_status.active;
// 		elementValue["range_min_m"] = guidance_status.range_min_m;
// 		elementValue["range_max_m"] = guidance_status.range_max_m;
// 		elementValue["publish_pc"] = guidance_status.publish_pc;
// 		elementValue["publish_img_left"] = guidance_status.publish_img_left;
// 		elementValue["publish_img_right"] = guidance_status.publish_img_right;
// 		elementValue["publish_depth"] = guidance_status.publish_depth;
// 		elementValue["publish_dist"] = guidance_status.publish_dist;
// 		elementValue["use_for_agl"] = guidance_status.use_for_agl;
// 		elementValue["agl_top_m"] = guidance_status.agl_top_m;
// 		elementValue["agl_bot_m"] = guidance_status.agl_bot_m;
// 		elementValue["fps_ave"] = guidance_status.fps_ave;
// 		elementValue["fps_min"] = guidance_status.fps_min;
// 		elementValue["fps_max"] = guidance_status.fps_max;
// 		poseArray.append(elementValue);
// 	}
// 	return value;
// }

// Json::Value AMJSONUtils::statusCommsToJson(
// 		std::string vehicleId,
// 		uint16_t sequence,
// 		boost::posix_time::ptime timestamp,
// 		std::string sender,
// 		const brain_box_msgs::msg::CommsStatus &msg)
// {
// 	 Json::Value value;
// 	 value.clear();

// 	//  headerToJson(value, vehicleId, sequence, timestamp, AMMessage::NAME_STATUS_COMMS, sender, AMMessage::TYPE_AUTOMOD);

// 	 value["data"]["status"]["summary"] = MessageUtil::statusIntToString(msg.status.summary);
// 	 value["data"]["logging"] = msg.logging;

//      return value;
// }

Json::Value AMJSONUtils::heartbeatToJson(
		std::string vehicleId,
		uint16_t sequence,
		int64_t timestamp,
		std::string sender)
{
	Json::Value value;
	value.clear();

	headerToJson(value, vehicleId, sequence, timestamp, AMMessage::NAME_HEARTBEAT, sender, AMMessage::TYPE_AUTOMOD);

	return value;
}

// Json::Value AMJSONUtils::snsrCamTrgtActToJson(std::string vehicleId, uint16_t sequence, boost::posix_time::ptime timestamp,
// 		std::string sender, const brain_box_msgs::msg::TargetPoseArray &msg)
// {
// 	Json::Value value;
// 	value.clear();

// 	// headerToJson(value, vehicleId, sequence, timestamp, AMMessage::NAME_SNSR_CAM_TRGT_ACT, sender, AMMessage::TYPE_AUTOMOD);
// 	if(msg.poses.size() == 0)
// 	{
// 		value["data"]["poses"] = Json::Value(Json::arrayValue);
// 	}
// 	else
// 	{
// 		Json::Value& poseArray = value["data"]["poses"];
// 		for(int i = 0; i < msg.poses.size(); i++)
// 		{
// 			Json::Value elementValue;
// 			bbPoseToJson(elementValue, msg.poses[i]);
// 			poseArray.append(elementValue);
// 		}
// 	}
// 	return value;
// }

