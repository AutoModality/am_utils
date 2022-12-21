/*
 * am_ros2_utilities.h
 *
 *  Created on: Oct 31, 2022
 *      Author: alireza
 */

#ifndef INCLUDE_AM_UTILS_AM_ROS2_UTILITY_H_
#define INCLUDE_AM_UTILS_AM_ROS2_UTILITY_H_

#include <chrono>
#include <rclcpp/rclcpp.hpp>

namespace am
{

//toDuration() can be used whenever a float or double has to be converted to std::chrono::duration datatype
//This function is needed when you are create a timer
template <typename T>
std::chrono::duration<T, std::ratio<1>> toDuration(T seconds)
{
    return std::chrono::duration<T, std::ratio<1>>(seconds);
}

template<class T>
void getParam(rclcpp::Node::SharedPtr node, const std::string &param_name, T &param,
		const T &initial_value, bool with_declaration=true)
{
	if(with_declaration)
	{
		node->declare_parameter(param_name, initial_value);
	}

	node->get_parameter_or(param_name, param, initial_value);
}

//a function that returns the string representation of a given ros2 message,
template<class MessageT>
std::string getMessageName()
{
	return rosidl_generator_traits::name<MessageT>();
}

template<class T>
T getRandom(T min, T max, bool reset = false)
{
	if(reset)
	{
		srand((unsigned) time(0));
	}

    return (min + 1) + (((T) rand()) / (T) RAND_MAX) * (max - (min + 1));
}

std::vector<std::string> toVector(const std::string &raw_str, char del);

std::string vectToStr(const std::vector<std::string> &vect);



}



#endif /* INCLUDE_AM_UTILS_AM_ROS2_UTILITY_H_ */
