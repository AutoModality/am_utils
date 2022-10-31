/*
 * ObjectPose.h
 * Contains state information about objects
 */

#ifndef BRAIN_BOX_SRC_AM_SP_OBJECT_STATE_H_
#define BRAIN_BOX_SRC_AM_SP_OBJECT_STATE_H_

#include <math.h>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <geometry_msgs/msg/quaternion.hpp>

#include <vb_util_lib/control_util.h>
#include <vb_util_lib/kinematics.h>
#include <vb_util_lib/kinematics_list.h>
#include <rclcpp/rclcpp.hpp>

class ObjectState {
public:
	ObjectState(rclcpp::Node::SharedPtr n);
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
	geometry_msgs::msg::Pose target_origin_ENU;

	// List of targets
	KinematicsList target_list_RFU;

	void calculateTargetENU();

	void setTargetOrigin();
	geometry_msgs::msg::Pose getTargetCurrentENUWRTOrigin();

	/*
	 * Routines for logging state data
	 */
	void publishRPY(rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr topic, rclcpp::Time t,
			double roll, double pitch, double yaw, double throttle);
	void dumpDebug();

	Kinematics& getTargetCurrentRFU() {
		return target_current_RFU;
	}

protected:
	rclcpp::Node::SharedPtr rosNode;

private:
	// Current target state
	Kinematics target_current_RFU;

	void initROS(rclcpp::Node::SharedPtr n);

	rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr currentAttPub;
	rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr commandAttPub;
	rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr setpointPub;

};

#endif /* BRAIN_BOX_SRC_AM_SP_OBJECT_STATE_H_ */
