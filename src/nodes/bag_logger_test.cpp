/*
 * bag_logger_test.cpp
 *
 *  Created on: Dec 20, 2022
 *      Author: alireza
 */

#include <rclcpp/rclcpp.hpp>
#include <am_utils/am_ros2_utility.h>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <vb_util_lib/bag_logger.h>
#include <brain_box_msgs/msg/log_control.hpp>

class TestClass
{
public:
	TestClass(rclcpp::Node::SharedPtr node): node_(node), bag_logger_()
	{
		bag_logger_.add_topic("/points", "geometry_msgs::msg::PointStamped");

		pub_ = node_->create_publisher<geometry_msgs::msg::PointStamped>("/points",1);

		points_sub_ = node_->create_subscription<geometry_msgs::msg::PointStamped>("/points",1,
				std::bind(&TestClass::pointCB, this, std::placeholders::_1));

		log_ctrl_sub_ = node_->create_subscription<brain_box_msgs::msg::LogControl>
		("/ctrl/log_control",1,std::bind(&TestClass::logCtrlCB, this, std::placeholders::_1));

		timer_ = node_->create_wall_timer(am::toDuration(0.25), std::bind(&TestClass::timerCB, this));
	}

	~TestClass()
	{

	}
private:

	rclcpp::Node::SharedPtr node_;

	rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr pub_;

	rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr points_sub_;

	rclcpp::Subscription<brain_box_msgs::msg::LogControl>::SharedPtr log_ctrl_sub_;

	rclcpp::TimerBase::SharedPtr timer_;

	void pointCB(const geometry_msgs::msg::PointStamped::SharedPtr msg)
	{
		bag_logger_.LOG(msg, "/points", "geometry_msgs::msg::PointStamped", msg->header.stamp, 2);
	}

	void logCtrlCB(const brain_box_msgs::msg::LogControl::SharedPtr msg)
	{
		if(msg->enable)
		{
			bag_logger_.startLogging("LG", msg->level);
			RCLCPP_INFO(node_->get_logger(), "%s: Logging is started", node_->get_name());
		}
		else
		{
			bag_logger_.stopLogging();
			RCLCPP_INFO(node_->get_logger(), "%s: Logging is stopped", node_->get_name());
		}
	}

	BagLogger bag_logger_;

	void timerCB()
	{
		static double val = 0.0;

		geometry_msgs::msg::PointStamped p;
		p.header.frame_id = "";
		p.header.stamp = node_->now();
		p.point.x = am::getRandom<double>(-10.0, 10.0);
		p.point.y = am::getRandom<double>(-20.0, 20.0);
		p.point.z = am::getRandom<double>(0.0, 100.0);

		pub_->publish(p);
	}
};


int main(int argc, char* argv[])
{
	rclcpp::init(argc, argv);

	std::shared_ptr<rclcpp::Node> node = std::make_shared<rclcpp::Node>("bag_logger_tester");

	std::shared_ptr<TestClass> tc = std::make_shared<TestClass>(node);

	rclcpp::spin(node);

	rclcpp::shutdown();

	return 0;

}
