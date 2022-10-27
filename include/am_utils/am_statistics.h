/*
 * am_statistics.h
 *
 *  Created on: Jun 20, 2018
 *      Author: ubuntu
 */

#ifndef AM_UTILS_INCLUDE_AM_UTILS_AM_STATISTICS_H_
#define AM_UTILS_INCLUDE_AM_UTILS_AM_STATISTICS_H_

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/vector3_stamped.hpp>
#include <iostream>
#include <string>
#include <brain_box_msgs/msg/latency.hpp>
#include <builtin_interfaces/msg/time.hpp>

namespace am
{

class Statistics
{
public:
    Statistics();
    virtual ~Statistics();

    void reset();

    void setVector(geometry_msgs::msg::Vector3Stamped& vec);
    void setVector(geometry_msgs::msg::Vector3Stamped& vec, builtin_interfaces::msg::Time t);
    void setLatency(brain_box_msgs::msg::Latency& latency);

    void addData(double data_point);

    std::ostream& operator<<(std::ostream& os);

    void print(std::string prefix);

    double minimum {0.0};
    double maximum {0.0};
    double average {0.0};
    double current {0.0};
    unsigned long cnt {0};
};

} /* namespace am */

#endif /* AM_UTILS_INCLUDE_AM_UTILS_AM_STATISTICS_H_ */
