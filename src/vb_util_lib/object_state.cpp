/*
 * ObjectPose.cpp
 *
 *  Created on: Mar 31, 2015
 *      Author: ubuntu
 */

#include <vb_util_lib/object_state.h>

ObjectState::ObjectState(rclcpp::Node::SharedPtr n):
vehicle_current_ENU(n),vehicle_command_ENU(n),vehicle_setpoint_ENU(n),
target_current_ENU(n), target_setpoint_ENU(n), target_list_RFU(n), target_current_RFU(n)
{
	// TODO Auto-generated constructor stub
	initROS(n);
}

ObjectState::~ObjectState() {
	// TODO Auto-generated destructor stub
}

geometry_msgs::msg::Pose ObjectState::getTargetCurrentENUWRTOrigin() {
	geometry_msgs::msg::Pose p;
	target_current_ENU.getPose(p);
	p.position.x -= target_origin_ENU.position.x;
	p.position.y -= target_origin_ENU.position.y;
	p.position.z -= target_origin_ENU.position.z;

	return p;
}

void ObjectState::setTargetOrigin() {
	target_current_ENU.getPose(target_origin_ENU);
}

/*
 * Initialize all the necessary ROS handles and publish/subscribe to topics
 */
void ObjectState::initROS(rclcpp::Node::SharedPtr n)
{
	rosNode = n;

	/*
	 * Subscribe to the necessary topics
	 */

	/*
	 * Advertise topics to be published
	 */
	currentAttPub = rosNode->create_publisher<geometry_msgs::msg::PoseStamped>("/brain_box/attitude/current", 1000);
	setpointPub = rosNode->create_publisher<geometry_msgs::msg::PoseStamped>("/brain_box/setpoint/current", 1000);
	commandAttPub = rosNode->create_publisher<geometry_msgs::msg::PoseStamped>("/brain_box/attitude/command", 1000);

	RCLCPP_INFO(rosNode->get_logger(), "^^^ ROS ON OBJECT STATE INITIALIZED ^^^");

	return;
}

void ObjectState::publishRPY(rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr topic, rclcpp::Time t
		, double roll, double pitch,
		double yaw, double throttle) {
	geometry_msgs::msg::PoseStamped pose;
	pose.header.stamp = t;
	pose.pose.orientation.x = roll;
	pose.pose.orientation.y = pitch;
	pose.pose.orientation.z = yaw;
	pose.pose.orientation.w = throttle;

	topic->publish(pose);
}

void ObjectState::dumpDebug() {
//	publishRPY(currentAttPub, t, );
//	publishRPY(commandAttPub, t, )
	rclcpp::Time t = rosNode->get_clock()->now();
	publishRPY(commandAttPub, t, vehicle_command_ENU.getAttitude().roll, vehicle_command_ENU.getAttitude().pitch, vehicle_command_ENU.getAttitude().yaw, vehicle_command_ENU.getAttitude().throttle);
	publishRPY(currentAttPub, t, vehicle_current_ENU.getAttitude().roll, vehicle_current_ENU.getAttitude().pitch, vehicle_current_ENU.getAttitude().yaw, vehicle_current_ENU.getAttitude().throttle);
	geometry_msgs::msg::PoseStamped p;
	target_current_RFU.getPose(p.pose);
	p.header.stamp = t;
	setpointPub->publish(p);
}

void ObjectState::calculateTargetENU() {
	//variables associated with the camera view
	double Xc, Yc, Dc, Ac;
	// Variables associated with the current vehicle state (i.e. yaw)
	double Ay;
	// Calculated Variables associated with the calculated target position in ENU
	double Xt, Yt, At;

	Xc = target_current_RFU.position().x;
	Yc = target_current_RFU.position().y;
	Dc = sqrt((Xc*Xc) + (Yc*Yc));
	Ay = vehicle_current_ENU.getAttitude().yaw + PI_2;

	if (Dc > 0)
	{
		Ac = asin(-Xc/Dc);
		At = Ay + Ac;
		Xt = Dc * cos(At);
		Yt = Dc * sin(At);
	}
	else
	{
		Xt = 0;
		Yt = 0;
	}

	// update the target ENU state
	target_current_ENU.setPose(target_current_ENU.getFrameId(), target_current_RFU.getUpdateTime(), Xt, Yt, target_current_RFU.position().z,
			target_current_RFU.getOrientation().w(), target_current_RFU.getOrientation().x(),
			target_current_RFU.getOrientation().y(), target_current_RFU.getOrientation().z());
//	target_current_ENU.setPosition(target_current_RFU.getUpdateTime(), Xt, Yt, target_current_RFU.position().z);
//	target_current_ENU.setAttitude(target_current_RFU.getAttitude());
//	targetCurrentENU.publishTransform();
//	printf("Xc:%f, Yc:%f, Dc:%f, Xt:%f, Yt:%f, YAW:%f, Ay:%f, Ac:%f, At:%f\n", Xc, Yc, Dc, Xt, Yt, vehicle_current_ENU.attitude.yaw, Ay, Ac, At);
//			targetCurrentLocal.position.x,
//			targetCurrentLocal.position.y,
//			targetCurrentLocal.position.z,
//			vehicleCurrent.attitude.yaw);
//	printf("target ENU position = x,y,z %f, %f, %f\n",
//			targetCurrentENU.position.x,
//			targetCurrentENU.position.y,
//			targetCurrentENU.position.z);
}

