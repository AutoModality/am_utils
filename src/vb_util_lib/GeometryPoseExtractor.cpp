/*
 * GeometryPoseExtractor.cpp
 *
 *  Created on: Apr 16, 2018
 *      Author: Alireza Janani
 *
 */
#include<ros/ros.h>
#include<geometry_msgs/PoseStamped.h>
#include<geometry_msgs/PoseWithCovarianceStamped.h>
#include<nav_msgs/Odometry.h>
#include<string>
#include<vector>
#include<vb_util_lib/GeometryPoseExtractor.h>


GeometryPoseExtractor::GeometryPoseExtractor(std::vector<geometry_msgs::PoseStamped> &msgs)
{
  poses = msgs;
}
GeometryPoseExtractor::GeometryPoseExtractor(std::vector<geometry_msgs::PoseWithCovarianceStamped> &msgs)
{
  poses.resize(msgs.size());
  int idx = -1;
  for(auto msg : msgs)
  {
    ++idx;
    poses[idx].header = msg.header;
    poses[idx].pose = msg.pose.pose;
  }
}
GeometryPoseExtractor::GeometryPoseExtractor(std::vector<nav_msgs::Odometry> &msgs)
{
  poses.resize(msgs.size());
  int idx = -1;
  for(auto msg : msgs)
  {
    ++idx;
    poses[idx].header = msg.header;
    poses[idx].pose = msg.pose.pose;
  }
}
std::vector<geometry_msgs::PoseStamped> &GeometryPoseExtractor::getPoses()
{
  return poses;
}
