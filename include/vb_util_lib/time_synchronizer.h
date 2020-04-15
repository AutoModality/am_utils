/*
 * time_synchronizer.h
 *
 *  Created on: March 17, 2018
 *      Author: Automodality Inc.
 */

#ifndef VISBOX_PACKAGES_COMMON_UTILS_UTIL_LIB_INCLUDE_TIME_SYNCHRONIZER_H_
#define VISBOX_PACKAGES_COMMON_UTILS_UTIL_LIB_INCLUDE_TIME_SYNCHRONIZER_H_

#include <stdio.h>
#include <stdlib.h>
#include <errno.h>
#include <fcntl.h>
#include <string.h>
#include <unistd.h>

#define BUFFER_LENGTH 256               ///< The buffer length (crude but fine)

#include <ros/ros.h>
#include <brain_box_msgs/StampedInterruptTime.h>

class TimeSynchronizer
{
public:
	TimeSynchronizer();

	~TimeSynchronizer();

	void timerSynchronizerPollCallback(const ros::TimerEvent& event);

	int getStatusInterrupt(void);
	int setStatusInterrupt(bool status_interrupt);
	double getTimeInterrupt(void);
	int setTimeInterrupt(double time_interrupt);
	int getNumInterrupt(void);
	int setNumInterrupt(int num_interrupts);
	double getPollRate(void);
	int setPollRate(double poll_rate);

private:
	brain_box_msgs::StampedInterruptTime stamped_interrupt_time_; // message of sampling rate of lw20 altimeter
	ros::NodeHandle n_;
	ros::Publisher time_synchronizer_pub_;
	ros::Timer timer_synchronizer_poll_;
	int fd_;
	double time_interrupt_;
	int num_interrupts_;
	bool new_interrupt_;
	double poll_rate_;
	int gpio_num_;
};


#endif /* VISBOX_PACKAGES_COMMON_UTILS_UTIL_LIB_INCLUDE_TIME_SYNCHRONIZER_H_ */
