/*
 * GeometryPoseExtractor.h
 *
 *  Created on: Apr 16, 2018
 *      Author: ubuntu
 */

#ifndef VISBOX_PACKAGES_COMMON_UTILS_UTIL_LIB_INCLUDE_VB_UTIL_LIB_GEOMETRYPOSEEXTRACTOR_H_
#define VISBOX_PACKAGES_COMMON_UTILS_UTIL_LIB_INCLUDE_VB_UTIL_LIB_GEOMETRYPOSEEXTRACTOR_H_

#include<geometry_msgs/PoseStamped.h>
#include<geometry_msgs/PoseWithCovarianceStamped.h>
#include<nav_msgs/Odometry.h>
#include<string>
#include<vector>



class GeometryPoseExtractor
{
public:

  GeometryPoseExtractor(std::vector<geometry_msgs::PoseStamped> &msgs);
  GeometryPoseExtractor(std::vector<geometry_msgs::PoseWithCovarianceStamped> &msgs);
  GeometryPoseExtractor(std::vector<nav_msgs::Odometry> &msgs);

  GeometryPoseExtractor();
  ~GeometryPoseExtractor();
  std::vector<geometry_msgs::PoseStamped> &getPoses();

private:
  std::vector<geometry_msgs::PoseStamped> poses;
};



#endif /* VISBOX_PACKAGES_COMMON_UTILS_UTIL_LIB_INCLUDE_VB_UTIL_LIB_GEOMETRYPOSEEXTRACTOR_H_ */
