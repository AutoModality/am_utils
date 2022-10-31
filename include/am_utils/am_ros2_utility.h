/*
 * am_ros2_utilities.h
 *
 *  Created on: Oct 31, 2022
 *      Author: alireza
 */

#ifndef INCLUDE_AM_UTILS_AM_ROS2_UTILITY_H_
#define INCLUDE_AM_UTILS_AM_ROS2_UTILITY_H_

#include <chrono>

namespace am
{

//toDuration() can be used whenever a float or double has to be converted to std::chrono::duration datatype
//This function is needed when you are create a timer
template <typename T>
std::chrono::duration<T, std::ratio<1>> toDuration(T seconds)
{
    return std::chrono::duration<T, std::ratio<1>>(seconds);
}



}



#endif /* INCLUDE_AM_UTILS_AM_ROS2_UTILITY_H_ */
