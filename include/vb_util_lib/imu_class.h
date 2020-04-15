/*
 * ImuInfuser.h
 *
 *  Created on: May 8, 2018
 *      Author: Alireza Janani
 */

#ifndef VISBOX_PACKAGES_COMMON_UTILS_UTIL_LIB_INCLUDE_VB_UTIL_LIB_IMU_CLASS_H_
#define VISBOX_PACKAGES_COMMON_UTILS_UTIL_LIB_INCLUDE_VB_UTIL_LIB_IMU_CLASS_H_

#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <opencv2/opencv.hpp>
#include <vector>
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud.h>
#include <nav_msgs/Odometry.h>
#include <cv_bridge/cv_bridge.h>

#include <vb_util_lib/rotate.h>
#include <vb_util_lib/utility.h>
#include <vb_util_lib/topics.h>

#include <math.h>
#include <stdio.h>
#include <tf/tf.h>
#include <ros/ros.h>

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
    ImuClass();
    ImuClass(int imu_pot_size);
    ImuClass(ros::NodeHandle &nh, int imu_pot_size = 400);
    ~ImuClass();

    void addImu(const sensor_msgs::Imu &imu);
    int getImu(std::vector<sensor_msgs::Imu> &res, const ros::Time &t, double time_difference);
    int getImu(sensor_msgs::Imu &res, const ros::Time &t);
    int getImu(sensor_msgs::Imu &res);
    int transform(sensor_msgs::PointCloud &pc, const ros::Time &t, double time_difference, int infuse_axis = AXIS::RPY);
    int transform(geometry_msgs::Point32 &p, const ros::Time &t, int infuse_axis = AXIS::RPY);
    int transform(cv::Point &p, const ros::Time &t, int infuse_axis = AXIS::RPY);
    int transform(nav_msgs::Odometry &odom, const ros::Time &t, std::vector<double> &rpy, int infuse_axis = AXIS::RPY);
    int transform(sensor_msgs::PointCloud &pc, const ros::Time &t, int infuse_axis = AXIS::RPY);
    int transform(float &x, float &y, float &z, const ros::Time &t, int infuse_axis = AXIS::RPY);
    int transform(int &x, int &y, int &z, const ros::Time &t, int infuse_axis = AXIS::RPY);
    int transform(cv::Mat &img, const ros::Time &t);
    int transform(const sensor_msgs::PointCloud2 &src, sensor_msgs::PointCloud2 &res, const ros::Time &t);
    void setup(int imu_pot_size);
    static cv::Mat getRotation(sensor_msgs::Imu &imu, bool doNormalization = false);
    static void normalize(sensor_msgs::Imu &imu);
  private:
    ros::NodeHandle nh_;
    ros::Subscriber dji_imu_sub_;
    ros::Subscriber	px4_imu_sub_;
    int imu_pot_size_;
	int imu_cnt_;
    std::vector<sensor_msgs::Imu> imu_pot_;
    void applyAxis(sensor_msgs::Imu &src, int axis, double &roll, double &pitch, double &yaw);
    void imuCB(const sensor_msgs::Imu::ConstPtr &imu_msg);
  };
}



#endif /* VISBOX_PACKAGES_COMMON_UTILS_UTIL_LIB_INCLUDE_VB_UTIL_LIB_IMU_CLASS_H_ */
