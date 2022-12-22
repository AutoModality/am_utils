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
	TestClass(rclcpp::Node::SharedPtr node): node_(node)
	{

		pub_ = node_->create_publisher<geometry_msgs::msg::PointStamped>("/points",1);

		points_sub_ = node_->create_subscription<geometry_msgs::msg::PointStamped>("/points",1,
				std::bind(&TestClass::pointCB, this, std::placeholders::_1));

		log_ctrl_pub_ = node_->create_publisher<brain_box_msgs::msg::LogControl>("/ctrl/log_control",1);

		log_ctrl_sub_ = node_->create_subscription<brain_box_msgs::msg::LogControl>
		("/ctrl/log_control",1,std::bind(&TestClass::logCtrlCB, this, std::placeholders::_1));

		timer_ = node_->create_wall_timer(am::toDuration(0.25), std::bind(&TestClass::timerCB, this));
	}

	~TestClass()
	{
		delete BagLogger::instance();
	}
private:

	rclcpp::Node::SharedPtr node_;

	rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr pub_;

	rclcpp::Publisher<brain_box_msgs::msg::LogControl>::SharedPtr log_ctrl_pub_;

	rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr points_sub_;

	rclcpp::Subscription<brain_box_msgs::msg::LogControl>::SharedPtr log_ctrl_sub_;

	rclcpp::TimerBase::SharedPtr timer_;

	void pointCB(const geometry_msgs::msg::PointStamped::SharedPtr msg)
	{
		LOG_MSG(*msg, "/points", "geometry_msgs/msg/PointStamped", msg->header.stamp, 1);
	}

	void logCtrlCB(const brain_box_msgs::msg::LogControl::SharedPtr msg)
	{
		if(msg->enable)
		{
			BagLogger::instance()->startLogging("LG", 3);
			RCLCPP_INFO(node_->get_logger(), "%s: Logging is started", node_->get_name());
		}
		else
		{
			BagLogger::instance()->stopLogging();
			RCLCPP_INFO(node_->get_logger(), "%s: Logging is stopped", node_->get_name());
		}
	}

	void timerCB()
	{
		static double val = 0.0;
		static bool logging_is_started_ = false;
		static rclcpp::Time start_t = node_->now();
		static bool enable_logging_ = true;

		geometry_msgs::msg::PointStamped p;
		p.header.frame_id = "";
		p.header.stamp = node_->now();
		p.point.x = am::getRandom<double>(-10.0, 10.0);
		p.point.y = am::getRandom<double>(-20.0, 20.0);
		p.point.z = am::getRandom<double>(0.0, 100.0);

		pub_->publish(p);

		if(enable_logging_)
		{
			if(!logging_is_started_)
			{
				if((node_->now() - start_t).seconds() >= 5.0)
				{
					brain_box_msgs::msg::LogControl log_ctrl_msg;
					log_ctrl_msg.enable = true;
					log_ctrl_msg.level = 3;
					log_ctrl_pub_->publish(log_ctrl_msg);

					start_t = node_->now();
					logging_is_started_ = true;
				}
			}
			else
			{
				if((node_->now() - start_t).seconds() >= 5.0)
				{
					brain_box_msgs::msg::LogControl log_ctrl_msg;
					log_ctrl_msg.enable = false;
					log_ctrl_msg.level = 3;
					log_ctrl_pub_->publish(log_ctrl_msg);
					enable_logging_ = false;
					start_t = node_->now();
				}
			}
		}



		if(!enable_logging_)
		{
			if((node_->now() - start_t).seconds() >= 5.0)
			{
				enable_logging_ = true;
				logging_is_started_ = false;
				start_t = node_->now();
			}
		}

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
