/*
 * am_message.h
 *
 *  Created on: Apr 27, 2015
 *      Author: dan
 */

#ifndef BRAIN_BOX_SRC_AM_MESSAGE_H_
#define BRAIN_BOX_SRC_AM_MESSAGE_H_

#include <vb_util_lib/msg_head.h>
#include <vb_util_lib/msg_data.h>

class AMMessage {
public:
	AMMessage();
	virtual ~AMMessage();

	static const std::string NAME_CTRL_VIDEO_CMND;
	static const std::string NAME_CTRL_VISION_CMND;
	static const std::string NAME_CTRL_VX_CMND;
	static const std::string NAME_CTRL_VX_PID_PARAM;
	static const std::string NAME_CTRL_VX_POS_SETPT;
	static const std::string NAME_HEARTBEAT;
	static const std::string NAME_SNSR_CAM_TRGT_ACT;
	static const std::string NAME_STATUS_C2;
	static const std::string NAME_STATUS_FTP;
	static const std::string NAME_STATUS_SUPER;
	static const std::string NAME_STATUS_VIDEO;
	static const std::string NAME_STATUS_VISION;
	static const std::string NAME_STATUS_VISLOG;
	static const std::string NAME_CTRL_VX_MISSION_MESSAGE;
	static const std::string NAME_REMOTE_CTRL_VX_MISSION;
	static const std::string NAME_ROSOUT;
	static const std::string NAME_VSTATE_SUMMARY;
	static const std::string NAME_STATUS_COMMS;
	static const std::string NAME_STATUS_DEPTH;
	static const std::string NAME_LOG_CONTROL;
	static const std::string NAME_ODOMETRY;
	static const std::string NAME_VX_FEATURE_POSE;
	static const std::string NAME_RC_IN;
	static const std::string NAME_IMU;

	static const std::string TYPE_AUTOMOD;
	static const std::string TYPE_MAVLINK;
	static const std::string TYPE_MAVLINK_RAW;
	static const std::string TYPE_ROS2;

	inline AMMsgHead* getHead() {
		return &head_;
	}

	inline void setHead(AMMsgHead &head) {
		head_ = head;
	}

	AMMsgData* getData() {
		return &data_;
	}

	inline void setData(AMMsgData &data) {
		data_ = data;
	}

private:
	AMMsgHead head_;
	AMMsgData data_;
};

#endif /* BRAIN_BOX_SRC_AM_MESSAGE_H_ */
