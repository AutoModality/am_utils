/*
 * gps_location.h
 *
 *  Created on: Nov 9, 2015
 *      Author: ubuntu
 */

#ifndef VISBOX_PACKAGES_COMMON_UTILS_UTIL_LIB_SRC_GPS_LOCATION_H_
#define VISBOX_PACKAGES_COMMON_UTILS_UTIL_LIB_SRC_GPS_LOCATION_H_

#define BOOST_BIND_NO_PLACEHOLDERS

#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <brain_box_msgs/msg/gps_location.hpp>
#include <vb_util_lib/topics.h>
//3.3#include <dji_sdk/GlobalPosition.h>
#include <rclcpp/rclcpp.hpp>

using namespace am;

/**
 * Wrapper method for the global GPS location.
 * Can use this class either as a publisher or a subscriber for the GPS Location.
 * Note that if a publisher of the GPS Location then you should explicitly call the
 * doUpdate method after changing any of the attributes so that it will get published
 * to the appropriate topic.
 */
using namespace std::chrono_literals;

class GPSLocation {
public:
    GPSLocation();
    GPSLocation(rclcpp::Node::SharedPtr node, std::string topic=am::am_topics::CTRL_VX_GPS_LOCATION,
            bool pub_on_change=false, double pub_rate=0.0) : node_(node),
        topic_(topic), pub_on_change_(pub_on_change), pub_rate_(pub_rate) {
        initROS(node_);
    }
	virtual ~GPSLocation();

    void initROS(rclcpp::Node::SharedPtr node);

	double getAltitude() const {
		return altitude_;
	}

    void setAltitude(double altitude) {
        altitude_ = altitude;
    }

//    void updateAltitude(double altitude) {
//        altitude_ = altitude;
//        checkUpdate();
//    }
//
	double getLatitude() const {
		return latitude_;
	}

	void setLatitude(double latitude) {
		latitude_ = latitude;
	}

	double getLongitude() const {
		return longitude_;
	}

	void setLongitude(double longitude) {
		longitude_ = longitude;
	}

	const sensor_msgs::msg::NavSatStatus& getStatus() const {
		return status_;
	}

	void setStatus(const sensor_msgs::msg::NavSatStatus& status) {
		status_ = status;
	}

    double getHeight() const {
        return height_;
    }

    void setHeight(double height) {
        height_ = height;
    }

    unsigned int getHealth() const {
        return health_;
    }

    void setHealth(unsigned int health) {
        this->health_ = health;
    }

    void setGPSLocation(const brain_box_msgs::msg::GPSLocation& msg) {
        latitude_ = msg.latitude;
        longitude_ = msg.longitude;
        altitude_ = msg.altitude;
        height_ = msg.height;
        health_ = msg.health;
    }

    void setGPSLocation(const sensor_msgs::msg::NavSatFix::SharedPtr& msg) {
        latitude_ = msg->latitude;
        longitude_ = msg->longitude;
        altitude_ = msg->altitude;
        status_ = msg->status;
    }

/*3.3    void setGPSLocation(const dji_sdk::GlobalPosition::ConstPtr& msg) {
        latitude_ = msg->latitude;
        longitude_ = msg->longitude;
        altitude_ = msg->altitude;
        height_ = msg->height;
        health_ = msg->health;
    }
*/
    void sendGPSLocation() {
        updated_ = true;

        if (!ros_init_ || !publisher_)
        {
            return;
        }
        brain_box_msgs::msg::GPSLocation ms;
        ms.latitude = latitude_;
        ms.longitude = longitude_;
        ms.altitude = altitude_;
        ms.height = height_;
        ms.health = health_;
        ms.header.stamp = node_->get_clock()->now();

        pub_nh_->publish(ms);
    }

    /**
     * Install a callback handler.
     * Use the following sytax to call this function:
     *   setCallback(std::bind(&MyClass::Callback, this, std::placeholders::_1));
     */
    void setCallback(std::function<void(GPSLocation*)> callback)
    {
        callback_ = callback;
    }

    /**
     * Check for update condition and publish status message if necessary
     */
    void doUpdate() {
      //        ROS_INFO("FCU STATUS: checkUpdate IN");
        updated_ = true;
        if (pub_on_change_)
        {
            sendGPSLocation();
        }
        if (callback_ != NULL)
        {
            callback_(this);
        }
    }

    void print() {
        printf("GPS - lat:%f  lon:%f  alt:%f  hgt:%f  hlth:%d\n",
                latitude_, longitude_, altitude_, height_, health_);
        return;
    }

    void updateLocation(const sensor_msgs::msg::NavSatFix::SharedPtr& msg);

private:

    rclcpp::Node::SharedPtr node_;

    /**
     * Latitude in degrees
     */
	double latitude_ = 0.0;
    /**
     * Longitude in degrees
     */
	double longitude_ = 0.0;
	/**
	 * Altitude above sea level
	 */
	double altitude_ = 0.0;
	/**
	 * Height above ground
	 */
	double height_ = 0.0;
	/**
	 * Health of the GPS 0-5
	 */
	unsigned int health_ {0};

	sensor_msgs::msg::NavSatStatus status_;

	// The following support the wrapper function

	/**
	 * topic to publish updates on
	 */
	std::string topic_ {""};

   /**
     * Updated flag
     */
    bool updated_ {false};

    /**
     * Publish on update flag
     */
    bool pub_on_change_ = false;

    /**
     * Rate to publish status if not 0
     */
    double pub_rate_ = 0.0;

    /**
     * True if this instance is publishing versus consuming the data
     */
    bool publisher_ {false};

    /**
     * Callback function pointer
     */
    std::function<void(GPSLocation*)> callback_ {NULL};

    bool ros_init_ {false};
    rclcpp::Publisher<brain_box_msgs::msg::GPSLocation>::SharedPtr pub_nh_;
    rclcpp::Subscription<brain_box_msgs::msg::GPSLocation>::SharedPtr sub_nh_;
    rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr dji_sub_nh_;
    rclcpp::TimerBase::SharedPtr pub_timer_;

    /**
     * Timer callback for publishing stats messages
     */
    void publishTimerCB() {
        sendGPSLocation();
    }

    /**
     * Callback to handle updates to DJI GPS location
     */
    void djiUpdateCB(const sensor_msgs::msg::NavSatFix::SharedPtr msg) {
        updated_ = true;
        setGPSLocation(msg);
        if (callback_ != NULL)
        {
            callback_(this);
        }
//print();
    }

    /**
     * Callback to handle updates to GPS location
     */
    void updateCB(const brain_box_msgs::msg::GPSLocation::SharedPtr msg) {
        updated_ = true;
        setGPSLocation(*msg);
        if (callback_ != NULL)
        {
            callback_(this);
        }
    }

};

#endif /* VISBOX_PACKAGES_COMMON_UTILS_UTIL_LIB_SRC_GPS_LOCATION_H_ */
