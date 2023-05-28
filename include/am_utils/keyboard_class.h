/*
 * keyboard_class.h
 *
 *  Created on: Aug 28, 2020
 *      Author: ubuntu
 */

#ifndef AM_UTILS_INCLUDE_AM_UTILS_KEYBOARD_CLASS_H_
#define AM_UTILS_INCLUDE_AM_UTILS_KEYBOARD_CLASS_H_

#include <std_msgs/msg/string.hpp>

//for keyboard accessing
#include <stdio.h>
#include <stdlib.h>
#include <cstdlib>
#include <sys/time.h>
#include <sys/types.h>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>

#include <am_utils/am_ros2_utility.h>

#include <sstream>

namespace am
{
class KeyboardClass
{
public:

	/**
	 * Constructor with ros handler
	 * @param nh
	 */
	KeyboardClass();

	KeyboardClass(const std::string &pub_topic);

	/**
	 * destructor
	 */
	~KeyboardClass();

	/**
	 * A function to read the key pressed. Use this function within the timer
	 * @return
	 */
	std::string getKey();

private:
	/**
	 * timer reads input at 100 Hz
	 */
	double timer_duration_ = {0.01};

	/**
	 * ros timer handler
	 */
	rclcpp::TimerBase::SharedPtr keyboard_read_timer_;

	/**
	 * ros publisher handler
	 */
	rclcpp::Publisher<std_msgs::msg::String>::SharedPtr key_pub_;

	/**
	 * timer callback, used for the timer
	 * @param event
	 */
	void keyboardReadCB();

	/**
	 * a function to setup all the necessary components
	 */
	void init();

	/**
	 * Variables for keyboards
	 */
	struct termios oldSettings, newSettings;
};
}



#endif /* AM_UTILS_INCLUDE_AM_UTILS_KEYBOARD_CLASS_H_ */
