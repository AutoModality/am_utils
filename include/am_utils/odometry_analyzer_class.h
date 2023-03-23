/*
 * odometry_analyzer_class.h
 *
 *  Created on: Mar 22, 2023
 *      Author: alireza
 */

#ifndef INCLUDE_AM_PERCEPT_UTILS_ODOMETRY_ANALYZER_CLASS_H_
#define INCLUDE_AM_PERCEPT_UTILS_ODOMETRY_ANALYZER_CLASS_H_

#include <am_utils/am_ros2_utility.h>
#include <nav_msgs/msg/odometry.hpp>
#include <vb_util_lib/rotate.h>


namespace am
{

struct Axis
{
	std::vector<double> components;

	double sum {0.0};
	double mean {0.0};
	int min_idx {0};
	int max_idx {0};
	double sd {0.0};
	double delta_min_max {0.0};

	void insert(double a)
	{
		//data analysis
		mean = (components.size() * mean + a)/(components.size() + 1);
		sum+=a;

		//first insert a to the back of the array
		//the index of the a is now components.size() - 1
		components.push_back(a);
		if(a <= components[min_idx])
		{
			min_idx = components.size() - 1;
		}
		else if(a >= components[max_idx])
		{
			max_idx = components.size() - 1;
		}

		delta_min_max = abs(components[max_idx] - components[min_idx]);
	}

	void findSD()
	{
		for(int a : components)
		{
			sd += pow(a - mean,2);
		}
		sd = sqrt(sd/components.size());
	}

	void print(const std::string &msg)
	{
		ROS_INFO("%s: cnt[%d] mean[%f] min[%f] max[%f] dmm[%f] sd[%f]", msg.c_str(), (int)components.size(), mean, components[min_idx],
				components[max_idx], delta_min_max, sd);
	}
};


class OdometryAnalyzer
{
public:
	OdometryAnalyzer(const std::string odom_topic,
			double x_ref, double y_ref, double yaw_ref);

	OdometryAnalyzer(double x_ref, double y_ref, double yaw_ref);

	~OdometryAnalyzer();

	void insert(const nav_msgs::msg::Odometry &odom);

	void insert(const nav_msgs::msg::Odometry::SharedPtr odom);

	am::Axis getX();
	am::Axis getY();
	am::Axis getYaw();
	am::Axis getXDiff();
	am::Axis getYDiff();
	am::Axis getYawDiff();

	void setGroundTruth(double x, double y, double yaw);



private:
	std::string odom_topic_;

	rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;

	double x_ref_ {0.0};
	double y_ref_ {0.0};
	double yaw_ref_ {0.0};

	am::Axis x_;
	am::Axis y_;
	am::Axis yaw_;
	am::Axis x_diff_;
	am::Axis y_diff_;
	am::Axis yaw_diff_;


	void odomCB(const nav_msgs::msg::Odometry::SharedPtr msg);

};

}



#endif /* INCLUDE_AM_PERCEPT_UTILS_ODOMETRY_ANALYZER_CLASS_H_ */
