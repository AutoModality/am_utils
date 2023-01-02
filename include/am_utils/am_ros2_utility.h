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
//
// Simple class used to create a global node handle singleton
//
class Node
{
public:
  Node() {}

  virtual ~Node() {}

  static std::shared_ptr<rclcpp::Node> node;
};

// toDuration() can be used whenever a float or double has to be converted to std::chrono::duration datatype
// This function is needed when you are create a timer
template <typename T>
std::chrono::duration<T, std::ratio<1>> toDuration(T seconds)
{
  return std::chrono::duration<T, std::ratio<1>>(seconds);
}

template <class T>
void getParam(rclcpp::Node::SharedPtr node, const std::string& param_name, T& param, const T& initial_value,
			  bool with_declaration = true)
{
  if (with_declaration)
  {
	node->declare_parameter(param_name, initial_value);
  }

  node->get_parameter_or(param_name, param, initial_value);
}

template <class T>
void getParam(const std::string& param_name, T& param, const T& initial_value)
{
  am::Node::node->get_parameter_or(param_name, param, initial_value);
}

// a function that returns the string representation of a given ros2 message,
template <class MessageT>
std::string getMessageName()
{
  return rosidl_generator_traits::name<MessageT>();
}

template <class T>
T getRandom(T min, T max, bool reset = false)
{
  if (reset)
  {
	srand((unsigned)time(0));
  }

  return (min + 1) + (((T)rand()) / (T)RAND_MAX) * (max - (min + 1));
}

std::vector<std::string> toVector(const std::string& raw_str, char del);

std::string vectToStr(const std::vector<std::string>& vect);

inline rclcpp::Time ClockNow() {return am::Node::node->now();}; 

}  // namespace am

// Defines to convert ROS1 outputs to ROS2
#define ROS_INFO(...) RCLCPP_INFO(am::Node::node->get_logger(), __VA_ARGS__)
#define ROS_INFO_THROTTLE(duration, ...) RCLCPP_INFO_THROTTLE(am::Node::node->get_logger(), *am::Node::node->get_clock(), (long)((duration) * 1000.0), __VA_ARGS__)
#define ROS_INFO_STREAM(...) RCLCPP_INFO_STREAM(am::Node::node->get_logger(), __VA_ARGS__)
#define ROS_INFO_STREAM_THROTTLE(duration, ...) RCLCPP_INFO_STREAM_THROTTLE(am::Node::node->get_logger(), *am::Node::node->get_clock(), (long)((duration) * 1000.0), __VA_ARGS__)

#define ROS_WARN(...) RCLCPP_WARN(am::Node::node->get_logger(), __VA_ARGS__)
#define ROS_WARN_THROTTLE(duration, ...) RCLCPP_WARN_THROTTLE(am::Node::node->get_logger(), *am::Node::node->get_clock(), (long)((duration) * 1000.0), __VA_ARGS__)
#define ROS_WARN_STREAM(...) RCLCPP_WARN_STREAM(am::Node::node->get_logger(), __VA_ARGS__)
#define ROS_WARN_STREAM_THROTTLE(duration, ...) RCLCPP_WARN_STREAM_THROTTLE(am::Node::node->get_logger(), *am::Node::node->get_clock(), (long)((duration) * 1000.0), __VA_ARGS__)

#define ROS_ERROR(...) RCLCPP_ERROR(am::Node::node->get_logger(), __VA_ARGS__)
#define ROS_ERROR_STREAM(...) RCLCPP_ERROR_STREAM(am::Node::node->get_logger(), __VA_ARGS__)
#define ROS_ERROR_THROTTLE(duration, ...) RCLCPP_ERROR_THROTTLE(am::Node::node->get_logger(), *am::Node::node->get_clock(), (long)((duration) * 1000.0), __VA_ARGS__)
#define ROS_ERROR_STREAM_THROTTLE(duration, ...) RCLCPP_ERROR_STREAM_THROTTLE(am::Node::node->get_logger(), *am::Node::node->get_clock(), (long)((duration) * 1000.0), __VA_ARGS__)

#define ROS_DEBUG(...) RCLCPP_DEBUG(am::Node::node->get_logger(), __VA_ARGS__)
#define ROS_DEBUG_STREAM(...) RCLCPP_DEBUG_STREAM(am::Node::node->get_logger(), __VA_ARGS__)
#define ROS_DEBUG_THROTTLE(duration, ...) RCLCPP_DEBUG_THROTTLE(am::Node::node->get_logger(), *am::Node::node->get_clock(), (long)((duration) * 1000.0), __VA_ARGS__)
#define ROS_DEBUG_STREAM_THROTTLE(duration, ...) RCLCPP_DEBUG_STREAM_THROTTLE(am::Node::node->get_logger(), *am::Node::node->get_clock(), (long)((duration) * 1000.0), __VA_ARGS__)

#endif /* INCLUDE_AM_UTILS_AM_ROS2_UTILITY_H_ */
