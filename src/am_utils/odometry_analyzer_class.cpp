/*
 * odometry_analyser.cpp
 *
 *  Created on: Mar 22, 2023
 *      Author: alireza
 */

#include <am_utils/odometry_analyzer_class.h>
#include <vb_util_lib/frame_names.h>
#include <vb_util_lib/utility.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace am
{
/*
 *
 * USE THIS CONSTRUCTOR IF YOU WANT TO USE
 * THE INTERNAL SUBSCRIBER
 *
 */
OdometryAnalyzer::OdometryAnalyzer(const std::string odom_topic,
		double x_ref, double y_ref, double yaw_ref): odom_topic_(odom_topic)
{
	tf_ = std::make_shared<am::Transformer>();

	setGroundTruth(x_ref, y_ref, yaw_ref);
	odom_sub_ = am::Node::node->create_subscription<nav_msgs::msg::Odometry>(odom_topic, 1,
			std::bind(&OdometryAnalyzer::odomCB, this, std::placeholders::_1));
}

/*
 *
 * USE THIS CONSTRUCTOR IF YOU WANT TO MANUALLY
 * LOAD THE ODOMETRY
 *
 */
OdometryAnalyzer::OdometryAnalyzer(double x_ref, double y_ref, double yaw_ref)
{
	tf_ = std::make_shared<am::Transformer>();
	setGroundTruth(x_ref, y_ref, yaw_ref);
}

OdometryAnalyzer::~OdometryAnalyzer()
{


}

void OdometryAnalyzer::setGroundTruth(double x, double y, double yaw)
{
	x_ref_ = x;
	y_ref_ = y;
	yaw_ref_ = yaw;
}

void OdometryAnalyzer::insert(const nav_msgs::msg::Odometry &odom)
{
	geometry_msgs::msg::Pose pose;
	pose.orientation = odom.pose.pose.orientation;
	pose.position = odom.pose.pose.position;

	geometry_msgs::msg::TransformStamped transform;
	while(!tf_->getTransform(Asset_Frame, odom.header.frame_id, transform, 1.0, false))
	{
		ROS_INFO(YELLOW "Cannot find a transform between Asset_Frame and %s" COLOR_RESET, odom.header.frame_id.c_str());
	}
	tf2::doTransform(pose, pose, transform);

	insertAssetFramePose(pose);

}

void OdometryAnalyzer::insert(const nav_msgs::msg::Odometry::SharedPtr odom)
{
	insert(*odom);
}

void OdometryAnalyzer::odomCB(const nav_msgs::msg::Odometry::SharedPtr msg)
{
	insert(msg);
}

void OdometryAnalyzer::insertAssetFramePose(const geometry_msgs::msg::Pose &pose)
{
	double yaw = am::Rotate::getYaw(pose.orientation);


	x_.insert(pose.position.x);
	y_.insert(pose.position.y);
	yaw_.insert(yaw);

	x_diff_.insert(x_ref_ - pose.position.x);
	y_diff_.insert(y_ref_ - pose.position.y);
	yaw_diff_.insert(yaw_ref_ - yaw);
}

am::Axis OdometryAnalyzer::getX()
{
	x_.findSD();
	return x_;
}

am::Axis OdometryAnalyzer::getY()
{
	y_.findSD();
	return y_;
}

am::Axis OdometryAnalyzer::getYaw()
{
	yaw_.findSD();
	return yaw_;
}
am::Axis OdometryAnalyzer::getXDiff()
{
	x_diff_.findSD();
	return x_diff_;
}

am::Axis OdometryAnalyzer::getYDiff()
{
	y_diff_.findSD();
	return y_diff_;
}

am::Axis OdometryAnalyzer::getYawDiff()
{
	yaw_diff_.findSD();
	return yaw_diff_;
}
}



