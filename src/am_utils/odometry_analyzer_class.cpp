/*
 * odometry_analyser.cpp
 *
 *  Created on: Mar 22, 2023
 *      Author: alireza
 */

#include <am_percept_utils/odometry_analyzer_class.h>


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
	double yaw = am::Rotate::getYaw(odom.pose.pose.orientation);


	x_.insert(odom.pose.pose.position.x);
	y_.insert(odom.pose.pose.position.y);
	yaw_.insert(yaw);

	x_diff_.insert(x_ref_ - odom.pose.pose.position.x);
	y_diff_.insert(y_ref_ - odom.pose.pose.position.y);
	yaw_diff_.insert(yaw_ref_ - yaw);
}

void OdometryAnalyzer::insert(const nav_msgs::msg::Odometry::SharedPtr odom)
{
	double yaw = am::Rotate::getYaw(odom->pose.pose.orientation);


	x_.insert(odom->pose.pose.position.x);
	y_.insert(odom->pose.pose.position.y);
	yaw_.insert(yaw);

	x_diff_.insert(x_ref_ - odom->pose.pose.position.x);
	y_diff_.insert(y_ref_ - odom->pose.pose.position.y);
	yaw_diff_.insert(yaw_ref_ - yaw);
}

void OdometryAnalyzer::odomCB(const nav_msgs::msg::Odometry::SharedPtr msg)
{
	insert(*msg);
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



