/*
 * rc_controls.h
 *
 *  Created on: May 4, 2017
 *      Author: ubuntu
 */

#ifndef VISBOX_PACKAGES_COMMON_UTILS_UTIL_LIB_INCLUDE_VB_UTIL_LIB_RC_CONTROLS_H_
#define VISBOX_PACKAGES_COMMON_UTILS_UTIL_LIB_INCLUDE_VB_UTIL_LIB_RC_CONTROLS_H_

#include <stdio.h>
#include "rc_mapper.h"
//#include <dji_sdk/RCChannels.h>
#include <am_utils/am_ros2_utility.h>
#include <sensor_msgs/msg/joy.hpp>
#include <mavros_msgs/msg/rc_in.hpp>

#define MAX_DJI_CONTROL 10000.0

#define ENGAGED_THRESHOLD 0.95

enum class RC_MODE 
{
    ATT = 0, 
    ALT, 
    POS, 
    UNKNOWN
};
enum class RC_GEAR 
{
    DOWN = 0, 
    UP, 
    UNKNOWN
};
enum class RC_TYPE 
{
    PX4 = rc_mapper::eRCT_PX4, 
    DJI = rc_mapper::eRCT_DJI, 
    ACSL = rc_mapper::eRCT_ACSL,
    UNKNOWN = rc_mapper::eRCT_Unknown
};

class RC_Controls 
{
public:
    RC_Controls() {};
    // RC_Controls(rclcpp::Node::SharedPtr node);
    RC_Controls(rclcpp::Node::SharedPtr node, RC_TYPE type = RC_TYPE::UNKNOWN);

    virtual ~RC_Controls();

    void setSubscriber(rclcpp::Node::SharedPtr node);

    /**
     * Install a callback handler.
     * Use the following sytax to call this function:
     *   setCallback(std::bind(&MyClass::Callback, this, std::placeholders::_1));
     */
    void setCallback(std::function<void(RC_Controls*)> callback) {
        callback_ = callback;
    }

    RC_MODE mode {RC_MODE::UNKNOWN};
    RC_GEAR gear {RC_GEAR::UNKNOWN};

    bool armed_switch {false};

    // This is a special mode we can engage and disengage with the congtroler sticks
    //both up and in engage and both up and out disengage
    bool user_function_engaged {false};

    // All stick values are between -1.0 and 1.0
    double throttle {0.0};
    double yaw {0.0};
    double roll {0.0};
    double pitch {0.0};

    rclcpp::Time stamp;
    rc_mapper* rc_mapper_;

//    const double MAX_DJI_CONTROL {10000.0};
//
//    RC_Controls& RC_Controls::operator=(const RC_Controls&)
//
//    RC_Controls& operator= ( const RC_Controls & ) = default;

private:

    rclcpp::Node::SharedPtr node_;

    // RC Controller raw values
    double mode_ {0.0};
    double gear_ {0.0};
    double throttle_ {0.0};
    double yaw_ {0.0};
    double roll_ {0.0};
    double pitch_ {0.0};
    double armed_ {0.0};

    unsigned throttle_idx_;
    unsigned yaw_idx_;
    unsigned roll_idx_;
    unsigned pitch_idx_;
    unsigned gear_idx_;
    unsigned mode_idx_;
    unsigned arm_idx_;

    int MAX_VALUE_ {2000};
	int MIN_VALUE_ {1000};

	int throttle_sign_ {1};
	int yaw_sign_ {1};
	int roll_sign_ {1};
	int pitch_sign_ {1};

	unsigned int RANGE_;
	unsigned int ZERO_VALUE_;
	unsigned int UPPER_VALUE_;
	unsigned int LOWER_VALUE_;

	RC_TYPE rc_type_ {RC_TYPE::DJI};

    /**
     * Callback function pointer
     */
    std::function<void(RC_Controls*)> callback_ {NULL};

    // DJI callback functions
    void djiRemoteCB(const sensor_msgs::msg::Joy::SharedPtr msg);
    void djiPsdkRemoteCB(const sensor_msgs::msg::Joy::SharedPtr msg);
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr dji_rc_channels_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr dji_psdk_rc_channels_sub_;
    void setDJIControls(const sensor_msgs::msg::Joy::SharedPtr  msg);
    void setDJIPsdkControls(const sensor_msgs::msg::Joy::SharedPtr  msg);
    bool setDJIModeSwitch(double mode_switch, RC_MODE& state);
    bool setDJIGearSwitch(double gear_switch, RC_GEAR& state);
    void setDJIStickValue(double input, double& state) { state = input; }

    // Logitech callback functions
    void logitechRemoteCB(const sensor_msgs::msg::Joy::SharedPtr  msg);
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr logitech_rc_channels_sub_;
    void setLogitechControls(const sensor_msgs::msg::Joy::SharedPtr msg);
    bool setLogitechModeSwitch(double mode_switch, RC_MODE& state);
    bool setLogitechGearSwitch(double gear_switch, RC_GEAR& state);
    void setLogitechStickValue(double input, double& state) { state = input; }

    // PX4 callback functions
    void px4RemoteCB(const mavros_msgs::msg::RCIn::SharedPtr  msg);
    rclcpp::Subscription<mavros_msgs::msg::RCIn>::SharedPtr  px4_rc_channels_sub_;
    void setPX4Controls(const mavros_msgs::msg::RCIn::SharedPtr  msg);
    bool setPX4ModeSwitch(double mode_switch, RC_MODE& state);
    bool setPX4GearSwitch(double gear_switch, RC_GEAR& state);
    bool setPX4ArmSwitch(double arm_switch, bool& state);
    void setPX4StickValue(double input, double& state, double polarity);

    // Routine to set the engaged state of the controller
    bool setEngagedState();
};

#endif /* VISBOX_PACKAGES_COMMON_UTILS_UTIL_LIB_INCLUDE_VB_UTIL_LIB_RC_CONTROLS_H_ */
