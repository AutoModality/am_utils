/*
 * time_synchronizer.cpp
 *
 *  Created on: March 17, 2018
 *      Author: AutoModality
 */

#include <vb_util_lib/time_synchronizer.h>
#include <vb_util_lib/frame_names.h>

//
// Constructors
//

TimeSynchronizer::TimeSynchronizer():
	n_("~")
{
	bool debounce;
	double poll_rate;

	ros::param::param<int>("~gpio_num", gpio_num_, 422);
	ros::param::param<bool>("~debounce", debounce, false);
	ros::param::param<double>("~poll_rate", poll_rate, 1.0);

	poll_rate_ = poll_rate;

	time_synchronizer_pub_ = n_.advertise<brain_box_msgs::StampedInterruptTime>("/interrupt_time", 120);

	if(poll_rate < 0.001)
	{// if the rate is too slow, just poll once
		timer_synchronizer_poll_ = n_.createTimer(ros::Duration(1.0/poll_rate), &TimeSynchronizer::timerSynchronizerPollCallback, this, true);
	}
	else
	{
		timer_synchronizer_poll_ = n_.createTimer(ros::Duration(1.0/poll_rate), &TimeSynchronizer::timerSynchronizerPollCallback, this);
	}

	//printf("Instance of time_synchronizer.");
	// set if need debounce, depend on how interrupt signal behavior.
	int ret;
	int fd;
	char gpio_path[BUFFER_LENGTH];
	char string_send[BUFFER_LENGTH];
	sprintf(gpio_path, "/sys/ts_lkm/gpio%d/isDebounce", gpio_num_);
	sprintf(string_send, "%d", debounce);
	ROS_INFO_STREAM("Set debounce to: " << debounce);
	fd = open(gpio_path, O_WRONLY);
	if(fd < 0)
	{
		perror("Failed to open isDebounce.");
		return;
	}
	ret = write(fd, string_send, sizeof(string_send));
	if (ret < 0)
	{
		perror("Failed to set debounce.");
	}
	close(fd);
}

//
// Destructor
//

TimeSynchronizer::~TimeSynchronizer()
{
}

//
// call back functions
//

void TimeSynchronizer::timerSynchronizerPollCallback(const ros::TimerEvent& event)
{
	// Check if the interrupt is a new one.
	// If it is a new one, then get the time and clear the newInterrupt in the kernel module.
	// The newInterrupt variable in kernel module is set in the interrupt handler when new interrupt comes in.
	// This suppose when read the time, no new interrupt will come in.
	switch (getStatusInterrupt())
	{
		case 1:
			char time_interrupt_str[BUFFER_LENGTH];
			sprintf(time_interrupt_str, "%.9f", getTimeInterrupt());
			ROS_INFO_STREAM("Got new interrupt. The interrupt time is: " << time_interrupt_str);

			stamped_interrupt_time_.header.frame_id = body_FLU;
			stamped_interrupt_time_.header.stamp = ros::Time(time_interrupt_);
			stamped_interrupt_time_.data = time_interrupt_;
			stamped_interrupt_time_.count = getNumInterrupt();

			time_synchronizer_pub_.publish(stamped_interrupt_time_);
			break;

		case 0:
			ROS_INFO("There is no new interrupt!");
			break;

		default:
			ROS_INFO("Failed to get status of interrupt!");
	}
}

int TimeSynchronizer::getStatusInterrupt(void)
{
	// Is there a new interrupt?
	int ret;
	int fd;
	char gpio_path[BUFFER_LENGTH];
	char receive[BUFFER_LENGTH];
	sprintf(gpio_path, "/sys/ts_lkm/gpio%d/newInterrupt", gpio_num_);
	fd = open(gpio_path, O_RDONLY);
	if(fd < 0)
	{
		perror("Failed to open newInterrupt.");
		return -1;
	}
	ret = read(fd, receive, BUFFER_LENGTH);
	close(fd);
	if (ret < 0)
	{
		perror("Failed to read the status of the interrupt.");
		return -1;
	}
	if(receive[0] == '1')
	{
		new_interrupt_ = true;
		return 1;
	}
	else if (receive[0] = '0')
	{
		new_interrupt_ = false;
		return 0;
	}
	else return -1;
}

// This function is to clear the newInterrupt in the ts_lkm when get interrupt time.
int TimeSynchronizer::setStatusInterrupt(bool status_interrupt)
{
	// set status of interrupt, used mostly for clear the status.
	int ret;
	int fd;
	char gpio_path[BUFFER_LENGTH];
	char string_send[BUFFER_LENGTH];
	sprintf(gpio_path, "/sys/ts_lkm/gpio%d/newInterrupt", gpio_num_);
	sprintf(string_send, "%d", status_interrupt);
	ROS_INFO_STREAM("Set status of interrupt to: " << status_interrupt);
	fd = open(gpio_path, O_WRONLY);
	if(fd < 0)
	{
		perror("Failed to open newInterrupt.");
		return -1;
	}
	ret = write(fd, string_send, sizeof(string_send));
	close(fd);
	if (ret < 0)
	{
		perror("Failed to set status of interrupt.");
		return -1;
	}
	new_interrupt_ = status_interrupt;

	return 0;
}

double TimeSynchronizer::getTimeInterrupt(void)
{
	// Get the newest interrupt time
	int ret;
	int fd;
	char gpio_path[BUFFER_LENGTH];
	char receive[BUFFER_LENGTH];
	double time_interrupt;
	sprintf(gpio_path, "/sys/ts_lkm/gpio%d/timeInterrupt", gpio_num_);
	ROS_INFO("Get interrupt time.");
	fd = open(gpio_path, O_RDONLY);
	if(fd < 0)
	{
		perror("Failed to open timeInterrupt.");
		return -1;
	}
	ret = read(fd, receive, BUFFER_LENGTH);
	close(fd);
	if (ret < 0)
	{
		perror("Failed to read interrupt time.");
		return -1;
	}

	setStatusInterrupt(false); // Clear newInterrupt

	sscanf(receive, "%lf", &time_interrupt);
	time_interrupt_ = time_interrupt;

	return time_interrupt;
}

int TimeSynchronizer::setTimeInterrupt(double time_interrupt)
{
	// Set interrupt time, mostly will be used for reset
	int ret;
	int fd;
	char gpio_path[BUFFER_LENGTH];
	char string_send[BUFFER_LENGTH];
	sprintf(gpio_path, "/sys/ts_lkm/gpio%d/timeInterrupt", gpio_num_);
	sprintf(string_send, "%.9f", time_interrupt);
	ROS_INFO_STREAM("Set interrupt time to: " << time_interrupt);
	fd = open(gpio_path, O_WRONLY);
	if(fd < 0)
	{
		perror("Failed to open timeInterrupt.");
		return -1;
	}
	ret = write(fd, string_send, sizeof(string_send));
	close(fd);
	if (ret < 0)
	{
		perror("Failed to set interrupt time.");
		return -1;
	}

	time_interrupt_ = time_interrupt;

	return 0;
}

int TimeSynchronizer::getNumInterrupt(void)
{
	// Get the number of interrupts
	int ret;
	int fd;
	char gpio_path[BUFFER_LENGTH];
	char receive[BUFFER_LENGTH];
	int num_interrupts;
	sprintf(gpio_path, "/sys/ts_lkm/gpio%d/numberInterrupts", gpio_num_);
	fd_ = open(gpio_path, O_RDONLY);
	if(fd_ < 0)
	{
		perror("Failed to open numberInterrupt.");
		return -1;
	}
	ret = read(fd_, receive, BUFFER_LENGTH);
	close(fd_);
	if (ret < 0)
	{
		perror("Failed to read the number of interrupts.");
		return -1;
	}

	sscanf(receive, "%d", &num_interrupts);
	num_interrupts_ = num_interrupts;
	ROS_INFO_STREAM("Get number of interrupts: " << num_interrupts);

	return num_interrupts;
}

int TimeSynchronizer::setNumInterrupt(int num_interrupts)
{
	// Set number of interrupts, mostly will be used for reset
	int ret;
	int fd;
	char gpio_path[BUFFER_LENGTH];
	char string_send[BUFFER_LENGTH];
	sprintf(gpio_path, "/sys/ts_lkm/gpio%d/numberInterrupts", gpio_num_);
	sprintf(string_send, "%d", num_interrupts);
	ROS_INFO_STREAM("Set number of interrupts to: " << num_interrupts);
	fd = open(gpio_path, O_WRONLY);
	if(fd < 0)
	{
		perror("Failed to open numberInterrupts.");
		return -1;
	}
	ret = write(fd, string_send, sizeof(string_send));
	close(fd);
	if (ret < 0)
	{
		perror("Failed to set number of interrupts.");
		return -1;
	}

	num_interrupts_ = num_interrupts;

	return 0;
}

double TimeSynchronizer::getPollRate(void)
{
	return poll_rate_;
}

int TimeSynchronizer::setPollRate(double poll_rate)
{
	poll_rate_ = poll_rate;
	timer_synchronizer_poll_.setPeriod(ros::Duration(poll_rate));

	return 1;
}
