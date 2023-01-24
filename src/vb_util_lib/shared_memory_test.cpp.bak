/*
 * shared_pointer_test.cpp
 *
 *  Created on: Aug 7, 2018
 *      Author: Alireza Janani
 */

#include <ros/ros.h>
#include <opencv2/opencv.hpp>
#include <vb_util_lib/shared_memory_master.h>
#include <vb_util_lib/shared_memory_client.h>
#include <string>

std::string segment_id_ = "test";
std::string img_location_ = "";
am::SharedMemoryMaster *spm = nullptr;
am::SharedMemoryClient *spc = nullptr;


int main(int argc, char** argv)
{
  ros::init(argc, argv, "shared_pointer_test");

  cv::Mat img = cv::imread(img_location_.c_str());

  spm = new am::SharedMemoryMaster<cv::Mat>(segment_id_, sizeof(img));
  spc = new am::SharedMemoryClient<cv::Mat>(segment_id_, sizeof(img));



  ros::spin();
  return 0;
}


