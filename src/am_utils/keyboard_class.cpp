/*
 * keyboard_class.cpp
 *
 *  Created on: Aug 28, 2020
 *      Author: ubuntu
 */

#include <am_utils/keyboard_class.h>
#include <am_utils/am_ros2_utility.h>

namespace am
{

KeyboardClass::KeyboardClass()
{
	init();
}

/**
 * Constructor with ros handler
 * @param nh
 */
KeyboardClass::KeyboardClass(const std::string &pub_topic)
{
	init();

	key_pub_ = am::Node::node->create_publisher<std_msgs::msg::String>(pub_topic, 1);

	keyboard_read_timer_ = am::Node::node->create_wall_timer(am::toDuration(timer_duration_),
			std::bind(&KeyboardClass::keyboardReadCB, this));
}

/**
 * destructor
 */
KeyboardClass::~KeyboardClass()
{
	tcsetattr( fileno( stdin ), TCSANOW, &oldSettings );
}

/**
 * a function to setup all the necessary components
 */
void KeyboardClass::init()
{
	tcgetattr( fileno( stdin ), &oldSettings );
	newSettings = oldSettings;
	newSettings.c_lflag &= (~ICANON & ~ECHO);
	tcsetattr( fileno( stdin ), TCSANOW, &newSettings );
}

/**
 * A function to read the key pressed. Use this function within the timer
 * @return
 */
std::string KeyboardClass::getKey()
{
	fd_set set;
	struct timeval tv;

	tv.tv_sec = 10;
	tv.tv_usec = 0;

	FD_ZERO( &set );
	FD_SET( fileno( stdin ), &set );

	int res = select( fileno( stdin )+1, &set, NULL, NULL, &tv );

	if( res > 0 )
	{

		//printf( "Input available\n" );
		char l;
		read( fileno( stdin ), &l, 1 );
		std::stringstream ss;
		ss << l;
		return ss.str();
	}

	return "";
}

/**
 * timer callback, used for the timer
 * @param event
 */
void KeyboardClass::keyboardReadCB()
{
	std_msgs::msg::String c;
	c.data = getKey();
	if(c.data == "")
	{
		return;
	}
	key_pub_->publish(c);
}
}
