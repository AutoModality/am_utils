/*
 * odometry_gps_validator.cpp
 *
 *  Created on: Apr 19, 2023
 *      Author: alireza
 *
 *      This node subscribes to multiple odometry topics and a single GPS topic (as ground truth)
 *      and record the odometry paths, GPS paths (in Asset_Frame), and their delta in x and y
 */

#include <am_utils/am_ros2_utility.h>
#include <boost/filesystem.hpp>

#include <nav_msgs/msg/odometry.hpp>
#include <mavros_msgs/msg/gpsraw.hpp>

#include <nav_msgs/msg/path.hpp>

#include <vb_util_lib/am_geometry.h>

#include <am_utils/keyboard_class.h>
#include <rosbag2_cpp/writer.hpp>
#include <fcntl.h>
#include <unistd.h>
#include <linux/input.h>

std::string NODE_NAME_ = "odometry_gps_validator";

std::shared_ptr<am::AMLifeCycle> am::Node::node;


namespace am
{
class OdomGPSValidator
{
public:
	OdomGPSValidator() : kc_()
	{
		getParams();

		event_timer_ = am::Node::node->create_wall_timer(am::toDuration(0.01), std::bind(&OdomGPSValidator::timerCB,this));

		odom_sub_ = am::Node::node->create_subscription<nav_msgs::msg::Odometry>(odom_topic_, 1, std::bind(&OdomGPSValidator::odomCB, this,
				std::placeholders::_1));

		gps_sub_ = am::Node::node->create_subscription<mavros_msgs::msg::GPSRAW>(gps_topic_, 1, std::bind(&OdomGPSValidator::gpsCB, this,
				std::placeholders::_1));




	}
	~OdomGPSValidator()
	{
		writer_->get_implementation_handle().close();
	}
private:

	std::string odom_topic_;

	std::string gps_topic_;

	double gps_reference_latitude_ = 0.0;

	double gps_reference_longitude_ = 0.0;

	std::string bag_file_output_path_;

	nav_msgs::msg::Path odom_path_;

	nav_msgs::msg::Path gps_path_;

	rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;

	rclcpp::Subscription<mavros_msgs::msg::GPSRAW>::SharedPtr gps_sub_;

	rclcpp::TimerBase::SharedPtr event_timer_;

	am::KeyboardClass kc_;
	std::unique_ptr<rosbag2_cpp::Writer> writer_ = std::make_unique<rosbag2_cpp::Writer>();

	void timerCB()
	{
		//std::string c = kc_.getKey();

		//ROS_INFO("KEY PRESSED: %s", c.c_str());
		//if(c == "e" || c == "r" || c == "s")
		{
			//start logging


			writer_->write(odom_path_, odom_topic_, am::ClockNow());

			writer_->write(gps_path_, gps_topic_, am::ClockNow());


		}
	}

	void gpsCB(const mavros_msgs::msg::GPSRAW::SharedPtr raw_msg)
	{
		if(raw_msg->h_acc <= 14)
		{
			double x, y;
			//calculateXY(raw_msg->lat / 10000000.0, raw_msg->lon / 10000000.0, x, y);
			AM::latLon_to_XY(gps_reference_latitude_, gps_reference_longitude_, 0.0,
					raw_msg->lat / 10000000.0, raw_msg->lon / 10000000.0, x,y);
			geometry_msgs::msg::PoseStamped ps;
			ps.header = raw_msg->header;
			ps.header.frame_id = "map";
			ps.pose.position.x = x;
			ps.pose.position.y = y;

			gps_path_.header = raw_msg->header;
			gps_path_.poses.push_back(ps);
		}
	}

	void odomCB(const nav_msgs::msg::Odometry::SharedPtr msg)
	{
		geometry_msgs::msg::PoseStamped ps;
		ps.header = msg->header;
		ps.header.frame_id = "map";
		ps.pose = msg->pose.pose;
		odom_path_.poses.push_back(ps);

	}

	void getParams()
	{
		am::getParam<std::string>("odom_topic", odom_topic_, odom_topic_);
		ROS_INFO(GREEN "odom_topic: %s" COLOR_RESET, odom_topic_.c_str());
		if(odom_topic_ == "")
		{
			std::runtime_error(RED "Odom topic is left empty, exiting" COLOR_RESET);
			exit(0);
		}

		am::getParam<double>("gps_reference_latitude", gps_reference_latitude_, gps_reference_latitude_);
		ROS_INFO(GREEN "gps_reference_latitude: %f" COLOR_RESET, gps_reference_latitude_);

		am::getParam<double>("gps_reference_longitude", gps_reference_longitude_, gps_reference_longitude_);
		ROS_INFO(GREEN "gps_reference_longitude: %f" COLOR_RESET, gps_reference_longitude_);
		if(gps_reference_longitude_ == 0.0 || gps_reference_latitude_ == 0.0)
		{
			std::runtime_error("GPS reference is invalid");
			exit(0);
		}

		am::getParam<std::string>("gps_topic", gps_topic_, gps_topic_);
		ROS_INFO(GREEN "gps_topic: %s" COLOR_RESET, gps_topic_.c_str());
		if(gps_topic_ == "")
		{
			std::runtime_error(RED "GPS topic is left empty, exiting" COLOR_RESET);
			exit(0);
		}

		am::getParam<std::string>("bag_file_output_path", bag_file_output_path_, bag_file_output_path_);
		ROS_INFO(GREEN "bag_file_output_path: %s" COLOR_RESET, bag_file_output_path_.c_str());

		//if the path does not exist, create a path recursively
		//if(!boost::filesystem::exists(bag_file_output_path_))
		//{
		//	boost::filesystem::create_directories(bag_file_output_path_);
		//}
		writer_->open(bag_file_output_path_);

	}

};

}


int main(int argc, char** argv)
{
	rclcpp::init(argc, argv);

	am::Node::node = std::make_shared<am::AMLifeCycle>(NODE_NAME_);

	am::OdomGPSValidator ogv;

	rclcpp::spin(am::Node::node);

	rclcpp::shutdown();
	return 0;

}
