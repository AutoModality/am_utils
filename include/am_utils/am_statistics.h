/*
 * am_statistics.h
 *
 *  Created on: Jun 20, 2018
 *      Author: ubuntu
 */

#ifndef AM_UTILS_INCLUDE_AM_UTILS_AM_STATISTICS_H_
#define AM_UTILS_INCLUDE_AM_UTILS_AM_STATISTICS_H_

#include <ros/ros.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <iostream>
#include <string>
#include <am_utils/Latency.h>

namespace am
{

class Statistics
{
public:
    Statistics();
    virtual ~Statistics();

    void reset();

    void setVector(geometry_msgs::Vector3Stamped& vec);
    void setVector(geometry_msgs::Vector3Stamped& vec, ros::Time t);
    void setLatency(am_utils::Latency& latency);

    void addData(double data_point);

    std::ostream& operator<<(std::ostream& os);

    void print(std::string prefix);

    double minimum;
    double maximum;
    double average;
    double current;
    unsigned long cnt;
};

} /* namespace am */

#endif /* AM_UTILS_INCLUDE_AM_UTILS_AM_STATISTICS_H_ */
