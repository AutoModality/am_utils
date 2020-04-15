/*
 * rc_controls.cpp
 *
 *  Created on: May 4, 2017
 *      Author: ubuntu
 */

#include <vb_util_lib/rc_controls.h>
#include <vb_util_lib/topics.h>

using namespace am;

//tk DJI Tuning
#define PX4_THROTTLE 2
#define PX4_YAW 3
#define PX4_ROLL 0
#define PX4_PITCH 1
#define PX4_GEAR 7
#define PX4_MODE 4
#define PX4_ARM 6

#define PX4_MAX_VALUE 1987
#define PX4_MIN_VALUE 998

#define ACSL_THROTTLE 2
#define ACSL_YAW 3
#define ACSL_ROLL 0
#define ACSL_PITCH 1
#define ACSL_GEAR 8
#define ACSL_MODE 6
#define ACSL_ARM 9

#define ACSL_THROTTLE_SIGN -1
#define ACSL_YAW_SIGN 1
#define ACSL_ROLL_SIGN 1
#define ACSL_PITCH_SIGN -1

#define PX4_THROTTLE_SIGN 1
#define PX4_YAW_SIGN 1
#define PX4_ROLL_SIGN 1
#define PX4_PITCH_SIGN 1

#define ACSL_MAX_VALUE 2020
#define ACSL_MIN_VALUE 1020

#define DEADBAND 25

RC_Controls::RC_Controls() {
}

RC_Controls::~RC_Controls() {
    // TODO Auto-generated destructor stub
}

RC_Controls::RC_Controls(ros::NodeHandle& n, RC_TYPE type) : rc_type_(type) {
	setSubscriber(n);
}

void RC_Controls::setSubscriber(ros::NodeHandle& n) {
	// Initialize the min and max constants
	n.param("/RC/max_value", MAX_VALUE_, MAX_VALUE_);
	n.param("/RC/min_value", MIN_VALUE_, MIN_VALUE_);

	std::string rc_type;
	n.param<std::string>("/RC/rc_type", rc_type, "DJI");
	if (rc_type == "DJI")
	{
		rc_type_ = RC_TYPE::DJI;
		ROS_INFO("RC: rc_type[DJI]");
	}
	else if (rc_type == "PX4")
	{
		rc_type_ = RC_TYPE::PX4;
		ROS_INFO("RC: rc_type[PX4]");
	}
	else if (rc_type == "ACSL")
	{
		rc_type_ = RC_TYPE::ACSL;
		ROS_INFO("RC: rc_type[ACSL]");
	}
	else
	{
		ROS_ERROR("RC: Illegal RC type parameter[%s], defaulting to DJI", rc_type.c_str());
		rc_type_ = RC_TYPE::DJI;
	}


	switch (rc_type_)
	{
	case RC_TYPE::DJI:
	    break;

	case RC_TYPE::ACSL:
        MAX_VALUE_ = ACSL_MAX_VALUE;
        MIN_VALUE_ = ACSL_MIN_VALUE;
		throttle_idx_ = ACSL_THROTTLE;
		yaw_idx_ = ACSL_YAW;
		roll_idx_ = ACSL_ROLL;
		pitch_idx_ = ACSL_PITCH;
		gear_idx_ = ACSL_GEAR;
		mode_idx_ = ACSL_MODE;
		arm_idx_ =  ACSL_ARM;

		throttle_sign = ACSL_THROTTLE_SIGN;
		yaw_sign = ACSL_YAW_SIGN;
		roll_sign = ACSL_ROLL_SIGN;
		pitch_sign = ACSL_PITCH_SIGN;

	    break;

	case RC_TYPE::PX4:
	default:
        MAX_VALUE_ = 1987; //2020;
        MIN_VALUE_ = 998; //1020;
		throttle_idx_ = PX4_THROTTLE;
		yaw_idx_ = PX4_YAW;
		roll_idx_ = PX4_ROLL;
		pitch_idx_ = PX4_PITCH;
		gear_idx_ = PX4_GEAR;
		mode_idx_ = PX4_MODE;
		arm_idx_ =  PX4_ARM;

		throttle_sign = PX4_THROTTLE_SIGN;
		yaw_sign = PX4_YAW_SIGN;
		roll_sign = PX4_ROLL_SIGN;
		pitch_sign = PX4_PITCH_SIGN;

		break;
	}

	RANGE_ = ((MAX_VALUE_ - MIN_VALUE_)/2);
	ZERO_VALUE_ =  (MIN_VALUE_ + RANGE_);
	UPPER_VALUE_ = (MAX_VALUE_ - (RANGE_/2));
	LOWER_VALUE_ = (MIN_VALUE_ + (RANGE_/2));

	dji_rc_channels_sub_ = n.subscribe<sensor_msgs::Joy>(am_topics::DJI_SDK_RC, 10,
			&RC_Controls::djiRemoteCB, this);

	px4_rc_channels_sub_ = n.subscribe<mavros_msgs::RCIn>(am_topics::MAVROS_RC_IN, 10,
			&RC_Controls::px4RemoteCB, this);

	stamp = ros::Time::now();
}

//===================================================
// DJI specific RC stuff
//===================================================

void RC_Controls::setDJIControls(const sensor_msgs::Joy::ConstPtr& msg) {

	stamp = msg->header.stamp;

	// Set the raw values;
	throttle_ = msg->axes[3];
	yaw_ = msg->axes[2];
	pitch_ = msg->axes[1];
	roll_ = msg->axes[0];
	gear_ = msg->axes[5];
	mode_ = msg->axes[4];

	setDJIStickValue(yaw_, yaw);
	setDJIStickValue(pitch_, pitch);
	setDJIStickValue(roll_, roll);
	setDJIStickValue(throttle_, throttle);

	setDJIModeSwitch(mode_, mode);
	setDJIGearSwitch(gear_, gear);
//    printf("XXX RC: roll[%0.2f], pitch[%0.2f], yaw[%0.2f], throttle[%0.2f], gear[%0.2f], mode[%0.2f]\n",
//          roll_, pitch_, yaw_, throttle_, gear_, mode_);
}

bool RC_Controls::setDJIModeSwitch(double mode_switch, RC_MODE& state) {
	bool change_state = false;

	if (mode_switch > 10)
	{
		if (state != RC_MODE::POS)
		{
			ROS_INFO("RC: CHANGING TO POS MODE");
			change_state = true;
		}
		state = RC_MODE::POS;
	}
	else if (mode_switch == 0)
	{
		if (state != RC_MODE::ALT)
		{
			ROS_INFO("RC: CHANGING TO ALT MODE");
			change_state = true;
		}
		state = RC_MODE::ALT;
	}
	else
	{
		if (state != RC_MODE::ATT)
		{
			ROS_INFO("RC: CHANGING TO ATT MODE");
			change_state = true;
		}
		state = RC_MODE::ATT;
	}

	return change_state;
}

bool RC_Controls::setDJIGearSwitch(double gear_switch, RC_GEAR& state) {
	bool change = false;
	if (gear_switch < -6000)
	{
		if (state != RC_GEAR::UP)
		{
			ROS_INFO("RC: CHANGING GEAR TO UP");
			change = true;
		}
		state = RC_GEAR::UP;
	}
	else
	{
		// gear switch down
		if (state != RC_GEAR::DOWN)
		{
			ROS_INFO("RC: CHANGING GEAR TO DOWN");
			change = true;
		}
		state = RC_GEAR::DOWN;
	}

	return change;
}

void RC_Controls::djiRemoteCB(const sensor_msgs::Joy::ConstPtr&  rc_msg) {
	setDJIControls(rc_msg);

    if (callback_ != NULL)
    {
        callback_(this);
    }
}

//===================================================
// PX4 specific RC stuff
//===================================================

//#define PX4_RANGE ((PX4_MAX_VALUE-PX4_MIN_VALUE)/2)
//#define PX4_ZERO_VALUE (PX4_MIN_VALUE + PX4_RANGE)
//#define PX4_UPPER_VALUE (PX4_MAX_VALUE - (PX4_RANGE/2))
//#define PX4_LOWER_VALUE (PX4_MIN_VALUE + (PX4_RANGE/2))

void RC_Controls::setPX4Controls(const mavros_msgs::RCIn::ConstPtr&  msg) {

	stamp = msg->header.stamp;

	// Set the raw values;
	throttle_ = msg->channels[throttle_idx_];
	yaw_ = msg->channels[yaw_idx_];
	pitch_ = msg->channels[pitch_idx_];
	roll_ = msg->channels[roll_idx_];
	gear_ = msg->channels[gear_idx_];
	mode_ = msg->channels[mode_idx_];
	armed_ = msg->channels[arm_idx_];

	setPX4StickValue(yaw_, yaw, yaw_sign);
	setPX4StickValue(pitch_, pitch, pitch_sign);
	setPX4StickValue(roll_, roll, roll_sign);
	setPX4StickValue(throttle_, throttle, throttle_sign);

	setPX4ModeSwitch(mode_, mode);
	setPX4GearSwitch(gear_, gear);
	setPX4ArmSwitch(armed_, armed_switch);
     //ROS_INFO_THROTTLE(2, "XXX RC: roll[%0.2f], pitch[%0.2f], yaw[%0.2f], throttle[%0.2f], gear[%d], mode[%d] armed[%d]\n",
     //      roll, pitch, yaw, throttle, static_cast<int>(gear), static_cast<int>(mode), armed_switch);
}

void RC_Controls::setPX4StickValue(double input, double& state, double polarity) {
	double diff = input - ZERO_VALUE_;

	if (fabs(diff) < DEADBAND)
	{
		diff = 0.0;
	}

	state = diff / RANGE_;

	if (state > 1.0)
	{
		state = 1.0;
	}
	else if (state < -1.0)
	{
		state = -1.0;
	}

	state *= polarity;
}

bool RC_Controls::setPX4ModeSwitch(double mode_switch, RC_MODE& state) {
	bool change_state = false;

	if (mode_switch > UPPER_VALUE_)
	{
		if (state != RC_MODE::POS)
		{
			ROS_INFO("RC: CHANGING TO POS MODE");
			change_state = true;
		}
		state = RC_MODE::POS;
	}
	else if (mode_switch > LOWER_VALUE_)
	{
		if (state != RC_MODE::ALT)
		{
			ROS_INFO("RC: CHANGING TO ALT MODE");
			change_state = true;
		}
		state = RC_MODE::ALT;
	}
	else
	{
		if (state != RC_MODE::ATT)
		{
			ROS_INFO("RC: CHANGING TO ATT MODE");
			change_state = true;
		}
		state = RC_MODE::ATT;
	}

	return change_state;
}

bool RC_Controls::setPX4GearSwitch(double gear_switch, RC_GEAR& state) {
	bool change = false;
	if (gear_switch > UPPER_VALUE_)
	{
		if (state != RC_GEAR::UP)
		{
			ROS_INFO("RC: CHANGING GEAR TO UP");
			change = true;
		}
		state = RC_GEAR::UP;
	}
	else
	{
		// gear switch down
		if (state != RC_GEAR::DOWN)
		{
			ROS_INFO("RC: CHANGING GEAR TO DOWN");
			change = true;
		}
		state = RC_GEAR::DOWN;
	}

	return change;
}

bool RC_Controls::setPX4ArmSwitch(double arm_switch, bool& state) {
	bool change = false;
	if (arm_switch > UPPER_VALUE_)
	{
		if (!state)
		{
			ROS_INFO("RC: SET ARM TO TRUE");
			change = true;
		}
		state = true;
	}
	else
	{
		// gear switch down
		if (state)
		{
			ROS_INFO("RC: SET ARM TO FALSE");
			change = true;
		}
		state = false;
	}

	return change;
}

void RC_Controls::px4RemoteCB(const mavros_msgs::RCIn::ConstPtr&  rc_msg) {
//	ROS_INFO_STREAM(">>>>RC got message " << *rc_msg << "\n");
	setPX4Controls(rc_msg);

    if (callback_ != NULL)
    {
        callback_(this);
    }
}

