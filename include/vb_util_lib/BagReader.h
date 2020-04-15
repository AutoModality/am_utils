/*
 * bag_reader.h
 *
 *  Created on: Apr 12, 2018
 *      Author: Alireza Janani
 */

#ifndef VISBOX_PACKAGES_COMMON_UTILS_UTIL_LIB_INCLUDE_VB_UTIL_LIB_BAGREADER_H_
#define VISBOX_PACKAGES_COMMON_UTILS_UTIL_LIB_INCLUDE_VB_UTIL_LIB_BAGREADER_H_

#include<rosbag/bag.h>
#include<rosbag/view.h>
#include<vb_util_lib/console_color.h>
#include<vector>
#include<string>
#include<ros/ros.h>
#include<iostream>
#include<fstream>

template<typename T>
class BagReader : public T
{
public:
  BagReader()
  {
    bag_file_location = "";
    topic = "";
    bag = new rosbag::Bag();
  }
  BagReader(const std::string &bag_file_location_, const std::string &topic_)
  {
    bag_file_location = bag_file_location_;
    topic = topic_;
    bag = new rosbag::Bag();
    readBagFile();
  }
  ~BagReader()
  {
    bag->close();
    delete bag;
  }
  void setBagLocation(const std::string &bag_file_location_)
  {
    bag_file_location = bag_file_location_;
    readBagFile();
  }
  std::vector<T> &getMessages()
  {
    return msgs;
  }

  static inline bool exists(const std::string &str)
  {
    std::ifstream f(str.c_str());
    return f.good();
  }
private:
  rosbag::Bag *bag;
  std::string bag_file_location;
  std::string topic;
  std::vector<T> msgs;

  bool bagIsOpen = false;


  void readBagFile()
  {
    if(!BagReader::exists(bag_file_location))
    {
      ROS_ERROR(RED "The file does not exist." COLOR_RESET);
      return;
    }
    ROS_INFO("Opening bag file at %s ...", bag_file_location.c_str());
    bag->open(bag_file_location, rosbag::bagmode::Read);
    /*if (bag->isOpen())
    {
      ROS_INFO("Opening %s is completed, Reading %s topic",bag_file_location.c_str(), topic.c_str());
    }*/
    ROS_INFO("Opening %s is completed, Reading %s topic",bag_file_location.c_str(), topic.c_str());
    rosbag::View view(*bag, rosbag::TopicQuery(topic));
    long cnt = 0;
    for(rosbag::MessageInstance const m : view)
    {
      cnt++;
      msgs.push_back(*m.instantiate<T>());
    }
    ROS_INFO("Operation is completed, entries processed [%ld]", cnt);
    bag->close();
  }


};


#endif /* VISBOX_PACKAGES_COMMON_UTILS_UTIL_LIB_INCLUDE_VB_UTIL_LIB_BAGREADER_H_ */
