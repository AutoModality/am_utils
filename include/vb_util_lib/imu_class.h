/*
 * ImuInfuser.h
 *
 *  Created on: May 8, 2018
 *      Author: Alireza Janani
 */

#ifndef VISBOX_PACKAGES_COMMON_UTILS_UTIL_LIB_INCLUDE_VB_UTIL_LIB_IMU_CLASS_H_
#define VISBOX_PACKAGES_COMMON_UTILS_UTIL_LIB_INCLUDE_VB_UTIL_LIB_IMU_CLASS_H_

#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <vector>
#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <vb_util_lib/rotate.h>
#include <vb_util_lib/utility.h>
#include <vb_util_lib/topics.h>
#include <am_utils/point_types.h>
#include <pcl/point_types.h>
#include <am_utils/am_pcl_utils.h>

#include <math.h>
#include <stdio.h>

namespace am
{
  enum AXIS
  {
    ROLL,
    PITCH,
    YAW,
    ROLL_PITCH,
    ROLL_YAW,
    PITCH_YAW,
    RPY
  };
  class ImuClass
  {
  public:
	/*
	 * @brief default constructor. the user needs to subscribe to imu topics
	 */
    ImuClass();

    /*
     * @brief constructor
     */
    ImuClass(int imu_pot_size);

    /*
     * @brief constructor for customized imu source
     */
    ImuClass(rclcpp::Node::SharedPtr node, const std::string &imu_topic, int imu_pot_size = 400);

    /*
     * @brief destructor
     */
    ~ImuClass();

    void addImu(const sensor_msgs::msg::Imu &imu);
    int getImu(std::vector<sensor_msgs::msg::Imu> &res, const rclcpp::Time &t, double time_difference);
    int getImu(sensor_msgs::msg::Imu &res, const rclcpp::Time &t);
    int getImu(sensor_msgs::msg::Imu &res);
    int transform(geometry_msgs::msg::Point32 &p, const rclcpp::Time &t, int infuse_axis = AXIS::ROLL_PITCH);
    int transform(nav_msgs::msg::Odometry &odom, const rclcpp::Time &t, std::vector<double> &rpy, int infuse_axis = AXIS::RPY);
    int transform(float &x, float &y, float &z, const rclcpp::Time &t, int infuse_axis = AXIS::ROLL_PITCH);
    int transform(int &x, int &y, int &z, const rclcpp::Time &t, int infuse_axis = AXIS::ROLL_PITCH);
    int transform(pcl::PointCloud<pcl::PointXYZIR> &pc, const rclcpp::Time &t, double time_difference, int infuse_axis);
    int transform(pcl::PointCloud<pcl::PointXYZ> &pc, const rclcpp::Time &t, double time_difference, int infuse_axis);
    void setup(int imu_pot_size);
    static void normalize(sensor_msgs::msg::Imu &imu);
  private:
    rclcpp::Node::SharedPtr node_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
    int imu_pot_size_;
	int imu_cnt_;
    std::vector<sensor_msgs::msg::Imu> imu_pot_;
    void applyAxis(sensor_msgs::msg::Imu &src, int axis, double &roll, double &pitch, double &yaw);
    void imuCB(const sensor_msgs::msg::Imu::SharedPtr imu_msg);
  };
}



#endif /* VISBOX_PACKAGES_COMMON_UTILS_UTIL_LIB_INCLUDE_VB_UTIL_LIB_IMU_CLASS_H_ */
