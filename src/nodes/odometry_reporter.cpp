/*
 * odometry_reporter.cpp
 *
 *  Created on: Apr 5, 2023
 *      Author: alireza
 */

#include <am_utils/am_ros2_utility.h>
#include <am_utils/odometry_analyzer_class.h>
#include <boost/filesystem.hpp>
#include <fstream>
#include <iostream>

std::string NODE_NAME_ = "odometry_reporter";

std::shared_ptr<am::AMLifeCycle> am::Node::node;

class OdometryReporter
{
public:
	OdometryReporter()
	{
		getParams();

		for(const std::string &topic : topics_)
		{
			std::shared_ptr<am::OdometryAnalyzer> oa = std::make_shared<am::OdometryAnalyzer>(topic,
					ground_truth_[0], ground_truth_[1], ground_truth_[2], false);

			odom_analyzers_.push_back(oa);
		}

		odom_sub_ = am::Node::node->create_subscription<nav_msgs::msg::Odometry>("/odometry/filtered",1,
				std::bind(&OdometryReporter::odomCB, this, std::placeholders::_1));

		write("topic, count, dx_min, dx_max, dx_mm, dx_mean, dx_sd, dy_min, dy_max, dy_mm, dy_mean, dy_sd, dyaw_min, dyaw_max, dyaw_mm, dyaw_mean, dyaw_sd\n", false);


		timer_ = am::Node::node->create_wall_timer(am::toDuration(0.1), std::bind(&OdometryReporter::dataCollectCB, this));
		timer_->cancel();
	}

	~OdometryReporter()
	{
	}

	void generateReport()
	{
		ROS_INFO(CYAN "GENERATING REPORT" COLOR_RESET);
	}
private:

	std::string output_path_ {""};
	std::string location_nickname_{""};
	std::string report_file_name_{""};
	std::vector<std::string> topics_;
	std::vector<double> ground_truth_;
	std::vector<bool> axis_list_;
	int test_id_ = 1;
	double record_time_ {20.0};
	double wait_time_ {10.0};

	rclcpp::TimerBase::SharedPtr timer_;

	rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;

	std::vector<std::shared_ptr<am::OdometryAnalyzer>> odom_analyzers_;


	void odomCB(const nav_msgs::msg::Odometry::SharedPtr msg)
	{
		static bool timer_is_running = false;

		if(!timer_is_running)
		{
			timer_is_running = true;
			timer_->reset();
		}
	}

	void write(const std::string &line, bool append = true)
	{
		std::ofstream output;
		if(append)
		{
			output.open(output_path_+"/"+report_file_name_, std::ios::app);
		}
		else
		{
			output.open(output_path_+"/"+report_file_name_);
		}

		output << line << std::endl;

		output.close();
	}

	void dataCollectCB()
	{
		static bool wait_time_is_over = false;

		static bool record_time_is_over = false;

		static rclcpp::Time start = am::ClockNow();

		if(!wait_time_is_over)
		{
			if(abs((am::ClockNow() - start).seconds()) >= wait_time_)
			{
				wait_time_is_over = true;
				start = am::ClockNow();
				ROS_INFO(CYAN "WAIT TIME IS OVER" COLOR_RESET);
			}
		}
		else if(!record_time_is_over)
		{

			//collect data here by enabling the data collectors
			for(int i = 0; i < odom_analyzers_.size(); i++)
			{
				odom_analyzers_[i]->enable();
			}

			//check if the test time is over
			if(abs((am::ClockNow() - start).seconds()) >= record_time_)
			{
				record_time_is_over = true;
				//record your data here
				//Stop the data collection
				for(int i = 0; i < odom_analyzers_.size(); i++)
				{
					odom_analyzers_[i]->disable();
					insert(odom_analyzers_[i]);
				}
			}
		}
		else if(record_time_is_over && wait_time_is_over)
		{
			ROS_INFO(CYAN "Done with recording. Exiting" COLOR_RESET);
			rclcpp::shutdown();
			exit(0);
		}

	}


	void insert(std::shared_ptr<am::OdometryAnalyzer> oa)
	{
		if(!oa)
		{
			return;
		}


		am::Axis dx = oa->getXDiff();
		//dx.print(oa->getTopic());
		am::Axis dy = oa->getYDiff();
		am::Axis dyaw = oa->getYawDiff();

		std::string message = oa->getTopic() + "," + std::to_string(dx.components.size()) + "," + std::to_string(dx.getMin()) + "," + std::to_string(dx.getMax()) + "," + std::to_string(dx.getMM())
				+ "," + std::to_string(dx.getMean()) + "," + std::to_string(dx.getSD())+
				"," + std::to_string(dy.getMin()) + "," + std::to_string(dy.getMax()) + "," + std::to_string(dy.getMM())
				+ "," + std::to_string(dy.getMean()) + "," + std::to_string(dy.getSD()) +
				"," + std::to_string(dyaw.getMin()) + "," + std::to_string(dyaw.getMax()) + "," + std::to_string(dyaw.getMM())
				+ "," + std::to_string(dyaw.getMean()) + "," + std::to_string(dyaw.getSD()) +
				+ "\n";
		write(message);
	}

	void getParams()
	{
		am::getParam<std::string>("output_path",output_path_, output_path_);
		ROS_INFO(CYAN "output_path: %s" COLOR_RESET, output_path_.c_str());
		//if the path does not exist, create a path recursively
		if(!boost::filesystem::exists(output_path_))
		{
			boost::filesystem::create_directories(output_path_);
		}

		am::getParam<std::string>("location_nickname", location_nickname_, location_nickname_);
		ROS_INFO(CYAN "location_nickname: %s" COLOR_RESET, location_nickname_.c_str());

		am::getParam("topics", topics_, topics_);
		if(topics_.size() < 1)
		{
			std::runtime_error(RED "There must be at least on topic" COLOR_RESET);
		}
		ROS_INFO(CYAN "topics: %s" COLOR_RESET, am::vectToStr(topics_).c_str());

		am::getParam("ground_truth", ground_truth_, ground_truth_);
		if(ground_truth_.size() < 4)
		{
			std::runtime_error(RED "Ground truth size must have the size of 4" COLOR_RESET);
		}
		ROS_INFO(CYAN "ground truth: %s" COLOR_RESET, am::vectToStr<double>(ground_truth_).c_str());

		report_file_name_ = location_nickname_+"_"+std::to_string(test_id_) + ".csv";

		am::getParam("axis_list", axis_list_, axis_list_);
		if(axis_list_.size() < 4)
		{
			std::runtime_error(RED "Axis list size must have the size of 4" COLOR_RESET);
		}
		ROS_INFO(CYAN "axis_list: %s" COLOR_RESET, am::vectToStr<bool>(axis_list_).c_str());


		am::getParam<int>("test_id", test_id_, test_id_);
		ROS_INFO(CYAN "test_id: %d" COLOR_RESET, test_id_);

		am::getParam<double>("wait_time", wait_time_, wait_time_);
		ROS_INFO(CYAN "wait_time: %f" COLOR_RESET, wait_time_);

		am::getParam<double>("record_time", record_time_, record_time_);
		ROS_INFO(CYAN "record_time: %f" COLOR_RESET, record_time_);
	}
};


int main(int argc, char** argv)
{
	rclcpp::init(argc, argv);

	am::Node::node = std::make_shared<am::AMLifeCycle>(NODE_NAME_);

	OdometryReporter odom_reporter;

	rclcpp::spin(am::Node::node);

	rclcpp::shutdown();
	return 0;
}


