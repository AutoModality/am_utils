/*
 * am_ros2_utility.cpp
 *
 *  Created on: Dec 7, 2022
 *      Author: alireza
 */

#include <am_utils/am_ros2_utility.h>
#include <iostream>


namespace am
{

/*
 * Set Covariances according to the following map
 * X 0 0 0 0 0
 * 0 Y 0 0 0 0
 * 0 0 Z 0 0 0
 * 0 0 0 R 0 0
 * 0 0 0 0 P 0
 * 0 0 0 0 0 Y
 */
bool setCovariances(const std::vector<double> &src, nav_msgs::msg::Odometry &odom)
{
	if(src.size() != 6)
	{
		return false;
	}

	for(int i = 0; i < src.size(); i++)
	{
		odom.pose.covariance[i * 7] = src[i];
	}

	return true;
}


void getParam(const std::string& param_name, std::vector<int>& param, const std::vector<int>& initial_value)
{
	am::declare_parameter<std::vector<int>>(param_name, param);

	rclcpp::Parameter temp = am::Node::node->get_parameter(param_name);

	am::Node::node->get_parameter(param_name, temp);

	std::vector<int64_t> res = temp.as_integer_array();

	for(const int64_t &i : res)
	{
		param.push_back(static_cast<int>(i));
	}

}

void getParam(const std::string& param_name, std::vector<uint8_t>& param, const std::vector<uint8_t>& initial_value)
{
	am::declare_parameter<std::vector<uint8_t>>(param_name, param);

	rclcpp::Parameter temp = am::Node::node->get_parameter(param_name);

	param = temp.as_byte_array();

}

void getParam(const std::string& param_name, std::vector<bool>& param, const std::vector<bool>& initial_value)
{
	am::declare_parameter<std::vector<bool>>(param_name, param);

	rclcpp::Parameter temp = am::Node::node->get_parameter(param_name);

	param = temp.as_bool_array();

}

void getParam(const std::string& param_name, std::vector<double>& param, const std::vector<double>& initial_value)
{
	am::declare_parameter<std::vector<double>>(param_name, param);

	rclcpp::Parameter temp = am::Node::node->get_parameter(param_name);

	param = temp.as_double_array();

}

void getParam(const std::string& param_name, std::vector<std::string>& param, const std::vector<std::string>& initial_value)
{
	am::declare_parameter<std::vector<std::string>>(param_name, param);

	rclcpp::Parameter temp = am::Node::node->get_parameter(param_name);

	param = temp.as_string_array();

}

std::string vectToStr(const std::vector<std::string> &vect)
{
	std::string res = "";
	for(int i = 0; i < vect.size(); i++)
	{
		res += vect[i];
		if(i < vect.size() - 1)
		{
			res += ", ";
		}
	}

	return res;
}

std::string getPackagePath(const std::string &package_name)
{
	return ament_index_cpp::get_package_share_directory(package_name);
}

}
