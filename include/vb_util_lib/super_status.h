#ifndef _SUPER_STATUS_H_
#define _SUPER_STATUS_H_

#include <rclcpp/rclcpp.hpp>
#include <brain_box_msgs/msg/node_status.hpp>
#include <unistd.h>

void initNode(rclcpp::Node::SharedPtr node);


class SuperStatus
{
private:
	static SuperStatus *ss_instance_;
	rclcpp::Node::SharedPtr node_;
	rclcpp::Publisher<brain_box_msgs::msg::NodeStatus>::SharedPtr status_pub_;
	std::string node_name;
	bool isNodeSet = false;
	
public:
	SuperStatus()
	{
//		status_pub_ = nh_.advertise<brain_box_msgs::NodeStatus>("/process/status", 100);
	}
	
	SuperStatus(rclcpp::Node::SharedPtr node)
	{
		ss_instance_ = this;
		ss_instance_->node_ = node;
		ss_instance_->node_name;
		ss_instance_->status_pub_ = node_->create_publisher<brain_box_msgs::msg::NodeStatus>("/process/status", 100);
		isNodeSet = true;
	}
	~SuperStatus(){};

	rclcpp::Node::SharedPtr getNode()
	{
		return node_;
	}

	static SuperStatus *instance() 
	{
	    if (!ss_instance_)
	    {
	        ss_instance_ = new SuperStatus;
	    }
	    return ss_instance_;
	}
	
	static void init(rclcpp::Node::SharedPtr n)
	{
		ss_instance_ = new SuperStatus(n);
	}
	static int sendStatus(rclcpp::Node::SharedPtr node, const std::string &status, std::string value)
	{
		if (!ss_instance_->isNodeSet)
		{
			return -1;
		}
		//ROS_INFO("Started");
		brain_box_msgs::msg::NodeStatus ns;
//		ROS_INFO("ns");
		ns.node_name = node->get_name();
		//ROS_INFO("node name: %s", ns.node_name.c_str());
		ns.status = status;
		ns.value = value;
		ns.process_id = (int)getpid();
		ss_instance_->status_pub_->publish(ns);
		//ROS_INFO("published");
		return 0;
	}
};

#endif 
