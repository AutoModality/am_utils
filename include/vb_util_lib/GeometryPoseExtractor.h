/*
 * GeometryPoseExtractor.h
 *
 *  Created on: Apr 16, 2018
 *      Author: ubuntu
 */

#ifndef VISBOX_PACKAGES_COMMON_UTILS_UTIL_LIB_INCLUDE_VB_UTIL_LIB_GEOMETRYPOSEEXTRACTOR_H_
#define VISBOX_PACKAGES_COMMON_UTILS_UTIL_LIB_INCLUDE_VB_UTIL_LIB_GEOMETRYPOSEEXTRACTOR_H_

#include<geometry_msgs/msg/pose_stamped.hpp>
#include<geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include<nav_msgs/msg/odometry.hpp>
#include<string>
#include<vector>



class GeometryPoseExtractor
{
public:

  GeometryPoseExtractor(std::vector<geometry_msgs::msg::PoseStamped> &msgs);
  GeometryPoseExtractor(std::vector<geometry_msgs::msg::PoseWithCovarianceStamped> &msgs);
  GeometryPoseExtractor(std::vector<nav_msgs::msg::Odometry> &msgs);

  GeometryPoseExtractor();
  ~GeometryPoseExtractor();
  std::vector<geometry_msgs::msg::PoseStamped> &getPoses();

private:
  std::vector<geometry_msgs::msg::PoseStamped> poses;
};



#endif /* VISBOX_PACKAGES_COMMON_UTILS_UTIL_LIB_INCLUDE_VB_UTIL_LIB_GEOMETRYPOSEEXTRACTOR_H_ */
