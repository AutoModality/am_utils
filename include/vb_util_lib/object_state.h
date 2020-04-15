/*
 * ObjectPose.h
 * Contains state information about objects
 */

#ifndef BRAIN_BOX_SRC_AM_SP_OBJECT_STATE_H_
#define BRAIN_BOX_SRC_AM_SP_OBJECT_STATE_H_

#include "math.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/TwistStamped.h"
#include "geometry_msgs/Quaternion.h"

#include "vb_util_lib/control_util.h"
#include "vb_util_lib/kinematics.h"
#include "vb_util_lib/kinematics_list.h"

class ObjectState {
public:
	ObjectState(ros::NodeHandle *n);
	virtual ~ObjectState();

	// Current actual object kinematics
	Kinematics vehicle_current_ENU;
	// Current object kinematics last commanded
	Kinematics vehicle_command_ENU;
	// Desired object kinematics
	Kinematics vehicle_setpoint_ENU;

	// Current target state
	Kinematics target_current_ENU;
	// Target setpoint
	Kinematics target_setpoint_ENU;
	// Target origin
	geometry_msgs::Pose target_origin_ENU;

	// List of targets
	KinematicsList target_list_RFU;

	void calculateTargetENU();

	void setTargetOrigin();
	geometry_msgs::Pose getTargetCurrentENUWRTOrigin();

	/*
	 * Routines for logging state data
	 */
	void publishRPY(ros::Publisher topic, ros::Time t,
			double roll, double pitch, double yaw, double throttle);
	void dumpDebug();

	Kinematics& getTargetCurrentRFU() {
		return target_current_RFU;
	}

protected:
	ros::NodeHandle *rosNode;

private:
	// Current target state
	Kinematics target_current_RFU;

	void initROS(ros::NodeHandle *n);

	ros::Publisher currentAttPub;
	ros::Publisher commandAttPub;
	ros::Publisher setpointPub;

};

#endif /* BRAIN_BOX_SRC_AM_SP_OBJECT_STATE_H_ */
