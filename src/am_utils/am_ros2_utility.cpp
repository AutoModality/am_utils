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



void getParam(rclcpp::Node::SharedPtr node, const std::string& param_name, std::vector<int>& param,
		const std::vector<int>& initial_value)
{
	am::declare_parameter<std::vector<int>>(node, param_name, param);

	rclcpp::Parameter temp = node->get_parameter(param_name);

	node->get_parameter(param_name, temp);

	std::vector<int64_t> res = temp.as_integer_array();

	for(const int64_t &i : res)
	{
		param.push_back(static_cast<int>(i));
	}

}

void getParam(rclcpp::Node::SharedPtr node, const std::string& param_name, std::vector<uint8_t>& param,
		const std::vector<uint8_t>& initial_value)
{
	am::declare_parameter<std::vector<uint8_t>>(node, param_name, param);

	rclcpp::Parameter temp = node->get_parameter(param_name);

	param = temp.as_byte_array();

}

void getParam(rclcpp::Node::SharedPtr node, const std::string& param_name, std::vector<bool>& param,
		const std::vector<bool>& initial_value)
{
	am::declare_parameter<std::vector<bool>>(node, param_name, param);

	rclcpp::Parameter temp = node->get_parameter(param_name);

	param = temp.as_bool_array();

}

void getParam(rclcpp::Node::SharedPtr node, const std::string& param_name, std::vector<double>& param,
		const std::vector<double>& initial_value)
{
	am::declare_parameter<std::vector<double>>(node, param_name, param);

	rclcpp::Parameter temp = node->get_parameter(param_name);

	param = temp.as_double_array();

}

void getParam(rclcpp::Node::SharedPtr node, const std::string& param_name, std::vector<std::string>& param,
		const std::vector<std::string>& initial_value)
{
	am::declare_parameter<std::vector<std::string>>(node, param_name, param);

	rclcpp::Parameter temp = node->get_parameter(param_name);

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
}
