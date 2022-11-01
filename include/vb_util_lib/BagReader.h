/*
 * bag_reader.h
 *
 *  Created on: Apr 12, 2018
 *      Author: Alireza Janani
 */

#ifndef VISBOX_PACKAGES_COMMON_UTILS_UTIL_LIB_INCLUDE_VB_UTIL_LIB_BAGREADER_H_
#define VISBOX_PACKAGES_COMMON_UTILS_UTIL_LIB_INCLUDE_VB_UTIL_LIB_BAGREADER_H_

#include <rclcpp/serialization.hpp>
#include <rclcpp/serialized_message.hpp>
#include <rosbag2_cpp/readers/sequential_reader.hpp>
#include <rosbag2_cpp/converter_interfaces/serialization_format_converter.hpp>
#include <rosbag2_cpp/storage_options.hpp>
#include <rosbag2_cpp/converter_options.hpp>
#include <vb_util_lib/console_color.h>
#include <vector>
#include <string>
#include <rclcpp/rclcpp.hpp>
#include <iostream>
#include <fstream>
#include <boost/filesystem.hpp>


template<typename T>
class BagReader : public T
{
public:
  BagReader(rclcpp::Node::SharedPtr node): node_(node)
  {
    bag_file_location_ = "";
    topic_ = "";
    //bag = rosbag::Bag();
  }
  BagReader(rclcpp::Node::SharedPtr node,
		  const std::string &bag_file_location, const std::string &topic): node_(node)
  {
	bag_file_location_ = bag_file_location;
    topic_ = topic;

    storage_options_.uri = bag_file_location_;
    storage_options_.storage_id = "sqlite3";

    converter_options_.input_serialization_format = "cdr";
    converter_options_.output_serialization_format = "cdr";
    //bag = new rosbag::Bag();
    readBagFile();
  }
  ~BagReader()
  {
    //bag->close();
    //delete bag;
  }
  void setBagLocation(const std::string &bag_file_location)
  {
    bag_file_location_ = bag_file_location;
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
  rclcpp::Node::SharedPtr node_;

  rosbag2_cpp::readers::SequentialReader bag_reader_;
  rosbag2_cpp::ConverterOptions converter_options_;
  rosbag2_cpp::StorageOptions storage_options_;
  std::string bag_file_location_;
  std::string topic_;
  std::vector<T> msgs;

  bool bagIsOpen = false;


  void readBagFile()
  {
	if(!boost::filesystem::exists(bag_file_location_))
    {
      RCLCPP_ERROR(node_->get_logger(), " The file does not exist." );
      return;
    }
	RCLCPP_INFO(node_->get_logger(), "Opening bag file at %s ...", bag_file_location_.c_str());
	bag_reader_.open(storage_options_, converter_options_);
    /*if (bag->isOpen())
    {
      ROS_INFO("Opening %s is completed, Reading %s topic",bag_file_location.c_str(), topic.c_str());
    }*/
    RCLCPP_INFO(node_->get_logger(), "Opening %s is completed, Reading %s topic",bag_file_location_.c_str(), topic_.c_str());


    long cnt = 0;
    std::shared_ptr<rosbag2_storage::SerializedBagMessage> msg;
    while(bag_reader_.has_next())
    {
    	msg = bag_reader_.read_next();
    	if(msg->topic_name == topic_)
    	{
    		T final_msg;

    		rclcpp::Serialization<T> serialization;
    		rclcpp::SerializedMessage extracted_serialized_msg(*msg->serialized_data);
    		serialization.deserialize_message(&extracted_serialized_msg, &final_msg);

    		msgs.push_back(final_msg);
    	}

    }


    RCLCPP_INFO(node_->get_logger(), "Operation is completed, entries processed [%ld]", cnt);
  }


};


#endif /* VISBOX_PACKAGES_COMMON_UTILS_UTIL_LIB_INCLUDE_VB_UTIL_LIB_BAGREADER_H_ */
