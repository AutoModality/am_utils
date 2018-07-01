/*
 * am_statistics.cpp
 *
 *  Created on: Jun 20, 2018
 *      Author: ubuntu
 */

#include <am_utils/am_statistics.h>

namespace am
{

Statistics::Statistics() {
    // TODO Auto-generated constructor stub

}

Statistics::~Statistics() {
    // TODO Auto-generated destructor stub
}

void Statistics::reset() {
    minimum = 999999999.0;
    maximum = -999999999.0;
    cnt = 0;
}

void Statistics::setVector(geometry_msgs::Vector3Stamped& vec) {
    vec.vector.x = minimum;
    vec.vector.y = maximum;
    vec.vector.z = average;
}

void Statistics::setVector(geometry_msgs::Vector3Stamped& vec, ros::Time t) {
    setVector(vec);
    vec.header.stamp = t;
}

void Statistics::addData(double data_point) {
    double total = average * cnt;
    total += data_point;
    cnt++;
    average = total / (double)cnt;

    if (data_point < minimum)
    {
        minimum = data_point;
    }

    if (data_point > maximum)
    {
        maximum = data_point;
    }

    current = data_point;
}

std::ostream& Statistics::operator <<(std::ostream& os) {
    os << "min[" << minimum << "]  max[" << maximum << "] avg[" << average;
    return os;
}

void Statistics::setLatency(am_utils::Latency& latency) {
    latency.cur_delay = current;
    latency.avg_delay = average;
    latency.max_delay = maximum;
    latency.min_delay = minimum;
}

void Statistics::print(std::string prefix) {
    printf("%scurrent[%0.4f] min[%0.4f] max[%0.4f] avg[%0.4f]", prefix.c_str(), current, minimum, maximum, average);
}

} /* namespace am */
