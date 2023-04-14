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
					ground_truth_[0], ground_truth_[1], ground_truth_[3], false);

			odom_analyzers_.push_back(oa);
		}

		odom_sub_ = am::Node::node->create_subscription<nav_msgs::msg::Odometry>("/odometry/filtered",1,
				std::bind(&OdometryReporter::odomCB, this, std::placeholders::_1));

		write("odometry_source, topic, count, dx_min, dx_max, dx_mm, dx_mean, dx_sd, dy_min, dy_max, dy_mm, dy_mean, dy_sd, dyaw_min, dyaw_max, dyaw_mm, dyaw_mean, dyaw_sd\n", false);


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
	std::vector<std::string> odom_sources_;
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
					insert(odom_analyzers_[i], odom_sources_[i]);
				}

				//insert the reference points
				std::string reference_point = "reference_point, x=" +std::to_string(ground_truth_[0]) + ", y=" +
						std::to_string(ground_truth_[1]) + ",yaw=" + std::to_string(ground_truth_[3])+ "\n";
				write(reference_point);
			}
		}
		else if(record_time_is_over && wait_time_is_over)
		{
			ROS_INFO(CYAN "Done with recording. Exiting" COLOR_RESET);
			rclcpp::shutdown();
			exit(0);
		}

	}


	void insert(std::shared_ptr<am::OdometryAnalyzer> oa, const std::string &topic_info)
	{
		if(!oa)
		{
			return;
		}
		std::vector<bool> axis = {true, true, true, true};
		if(oa->getTopic() == "/ekf/odom/yh")
		{
			axis[0] = false;
			axis[2] = false;
		}
		else if(oa->getTopic() == "/ekf/odom/xyh")
		{
			axis[2] = false;
		}

		am::Axis dx = oa->getXDiff();
		am::Axis dy = oa->getYDiff();
		am::Axis dyaw = oa->getYawDiff();

		std::string message = topic_info +
				"," + oa->getTopic() +
				"," + std::to_string(dx.components.size()) +
				"," + getLine(dx, axis[0]) +
				"," + getLine(dy, axis[1]) +
				"," + getLine(dyaw, axis[3], true) + "\n";
		write(message);
	}

	std::string getLine(am::Axis &a, bool valid, bool heading = false)
	{
		std::string res = "";

		//this means that there are no valid data
		if(!valid)
		{
			res = "0.0, 0.0, 0.0, 0.0, 0.0";
		}
		else
		{
			double mm = a.getMM();
			double max_v = a.getMax();
			double min_v = a.getMin();
			double mean_v = a.getMean();
			double sd_v = a.getMean();
			if(heading)
			{
				mm = am::Rotate::wrap_pi(mm);
				max_v = am::Rotate::wrap_pi(max_v);
				min_v = am::Rotate::wrap_pi(min_v);
				mean_v = am::Rotate::wrap_pi(mean_v);
				sd_v = am::Rotate::wrap_pi(sd_v);
			}



			res = std::to_string(min_v) + "," + std::to_string(max_v) + "," + std::to_string(mm)
						+ "," + std::to_string(mean_v) + "," + std::to_string(sd_v);
		}

		return res;
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


		am::getParam("odom_sources", odom_sources_, odom_sources_);
		if(odom_sources_.size() < 1)
		{
			std::runtime_error(RED "There must be at least on odom_source" COLOR_RESET);
		}
		ROS_INFO(CYAN "odom_sources: %s" COLOR_RESET, am::vectToStr(odom_sources_).c_str());

		am::getParam("ground_truth", ground_truth_, ground_truth_);
		if(ground_truth_.size() < 4)
		{
			std::runtime_error(RED "Ground truth size must have the size of 4" COLOR_RESET);
		}
		ground_truth_[3] = ground_truth_[3] * M_PI/180.0;
		ROS_INFO(CYAN "ground truth: %s" COLOR_RESET, am::vectToStr<double>(ground_truth_).c_str());

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


		report_file_name_ = location_nickname_+"_"+std::to_string(test_id_) + ".csv";

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


