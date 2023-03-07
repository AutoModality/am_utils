
/*
 * am_ros2_utilities.h
 *
 *  Created on: Oct 31, 2022
 *      Author: alireza
 */

#ifndef INCLUDE_AM_UTILS_AM_ROS2_UTILITY_H_
#define INCLUDE_AM_UTILS_AM_ROS2_UTILITY_H_

#define BOOST_BIND_NO_PLACEHOLDERS

#include <chrono>
#include <rclcpp/rclcpp.hpp>
#include <any>
#include <vb_util_lib/utility.h>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <super_lib/am_life_cycle.h>
#include <nav_msgs/msg/odometry.hpp>

// using namespace std::chrono_literals;
using std::placeholders::_1;

namespace am
{
//
// Simple class used to create a global node handle singleton
//
class Node
{
public:
  Node() {}
  Node(std::shared_ptr<am::AMLifeCycle> n) {
    node = n;
  }

  virtual ~Node() {}

  static std::shared_ptr<am::AMLifeCycle> node;
};

std::string getPackagePath(const std::string &package_name);


// toDuration() can be used whenever a float or double has to be converted to std::chrono::duration datatype
// This function is needed when you are create a timer
template <typename T>
std::chrono::duration<T, std::ratio<1>> toDuration(T seconds)
{
  return std::chrono::duration<T, std::ratio<1>>(seconds);
}

//declare parameter is for all datatypes.
//this function is called by all instances of getParam
//and therefore, you do not need to call this function
template<class T>
void declare_parameter(const std::string &param_name, const T &param)
{
	try
	{
		am::Node::node->declare_parameter(param_name, param);
	}
	catch(rclcpp::exceptions::ParameterAlreadyDeclaredException &e)
	{
		;
	}
}

bool setCovariances(const std::vector<double> &src, nav_msgs::msg::Odometry &odom);

//get parameter for vector of integers
void getParam(const std::string& param_name, std::vector<int>& param, const std::vector<int>& initial_value);

//get parameter for vector of bytes
void getParam(const std::string& param_name, std::vector<uint8_t>& param, const std::vector<uint8_t>& initial_value);

//get parameter for vector of booleans
void getParam(const std::string& param_name, std::vector<bool>& param, const std::vector<bool>& initial_value);

//get parameter for vector of doubles
void getParam(const std::string& param_name, std::vector<double>& param, const std::vector<double>& initial_value);

//get parameter for vector of strings
void getParam(const std::string& param_name, std::vector<std::string>& param, const std::vector<std::string>& initial_value);

//get parameters for all basic datatypes except arrays
template <class T>
bool getParam(const std::string& param_name, T& param, const T& initial_value)
{
	am::declare_parameter(param_name, initial_value);
	return am::Node::node->get_parameter_or(param_name, param, initial_value);
}

// Compatible with getParam with different method name
template <class T>
void getParameter(const std::string& param_name, T& param, const T& initial_value)
{
  am::getParam(param_name, param, initial_value);
}

// a function that returns the string representation of a given ros2 message,
template <class MessageT>
std::string getMessageName()
{
  return rosidl_generator_traits::name<MessageT>();
}

//returns a random number between two min and max limit
template <class T>
T getRandom(T min, T max, bool reset = false)
{
  if (reset)
  {
	srand((unsigned)time(0));
  }

  return (min + 1) + (((T)rand()) / (T)RAND_MAX) * (max - (min + 1));
}

//this will split a string to array of strings with the given key
template <class T>
std::vector<T> toVector(const std::string& raw_str, char del)
{
	std::string line;
	std::vector<T> vec;
	std::stringstream ss(raw_str);
	while(std::getline(ss, line, del)) {
		vec.push_back(std::any_cast<T>(line));
	}
	return vec;
}

//this will convert a vector to a string
std::string vectToStr(const std::vector<std::string>& vect);

//inline rclcpp::Time ClockNow() {return am::Node::node->now();};
inline rclcpp::Time ClockNow() {return am::Node::node->get_clock()->now();};
inline rclcpp::Time clockNow() {return am::Node::node->get_clock()->now();};
inline rclcpp::Clock::SharedPtr getClock() {return am::Node::node->get_clock();};
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
