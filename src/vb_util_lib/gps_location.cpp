/*
 * gps_location.cpp
 *
 *  Created on: Nov 9, 2015
 *      Author: ubuntu
 */

#include <vb_util_lib/gps_location.h>
#include <functional>

// using namespace std::placeholders;
using std::placeholders::_1;

GPSLocation::GPSLocation() {
	// TODO Auto-generated constructor stub

}

GPSLocation::~GPSLocation() {
	// TODO Auto-generated destructor stub
}

void GPSLocation::initROS(rclcpp::Node::SharedPtr node) {
    if (pub_on_change_ || pub_rate_ > 0.0)
    {
        publisher_ = true;
        pub_nh_ = node_->create_publisher<brain_box_msgs::msg::GPSLocation>(topic_.c_str(), 10);
    }
    else
    {
        publisher_ = false;
        sub_nh_ = node_->create_subscription<brain_box_msgs::msg::GPSLocation>(topic_, 10, std::bind(&GPSLocation::updateCB, this, _1));
    }

    if (pub_rate_ > 0)
    {
        //pub_rate_
        pub_timer_ = node_->create_wall_timer(500ms, std::bind(&GPSLocation::publishTimerCB, this));
    }

//3.3        dji_sub_nh_ = n->subscribe((am_topics::DJI_SDK_GLOBAL_POSITION, 1000,
//                &GPSLocation::djiUpdateCB, this);
    dji_sub_nh_ = node_->create_subscription<sensor_msgs::msg::NavSatFix>(am_topics::DJI_SDK_GPS_POSITION, 10, std::bind(&GPSLocation::djiUpdateCB, this, _1));

    ros_init_ = true;
}

void GPSLocation::updateLocation(const sensor_msgs::msg::NavSatFix::SharedPtr& msg) {
	latitude_ = msg->latitude;
	longitude_ = msg->longitude;
	altitude_ = msg->altitude;
	status_ = msg->status;
}
