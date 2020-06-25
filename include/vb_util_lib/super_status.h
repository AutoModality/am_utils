#ifndef _SUPER_STATUS_H_
#define _SUPER_STATUS_H_

#include <ros/ros.h>
#include <brain_box_msgs/NodeStatus.h>
#include <unistd.h>

void initNode(ros::NodeHandle& n);


class SuperStatus
{
private:
	static SuperStatus *ss_instance_;
	ros::NodeHandle nh_;
	ros::Publisher status_pub_;
	std::string node_name;
	bool isNodeSet = false;
	
public:
	SuperStatus()
	{
//		status_pub_ = nh_.advertise<brain_box_msgs::NodeStatus>("/process/status", 100);
	}
	
	SuperStatus(ros::NodeHandle &n)
	{
		ss_instance_ = this;
		ss_instance_->nh_ = n;
		ss_instance_->node_name;
		ss_instance_->status_pub_ = nh_.advertise<brain_box_msgs::NodeStatus>("/process/status", 100);
		isNodeSet = true;
	}
	~SuperStatus(){};
	static SuperStatus *instance() 
	{
	    if (!ss_instance_)
	    {
	        ss_instance_ = new SuperStatus;
	    }
	    return ss_instance_;
	}
	
	static void init(ros::NodeHandle &n)
	{
		ss_instance_ = new SuperStatus(n);
	}
	static int sendStatus(const std::string &status, std::string value)
	{
	        if (!ss_instance_->isNodeSet)
	        {
	            return -1;
	        }
		//ROS_INFO("Started");
		brain_box_msgs::NodeStatus ns;
//		ROS_INFO("ns");
		ns.node_name = ros::this_node::getName();
		//ROS_INFO("node name: %s", ns.node_name.c_str());
		ns.status = status;
		ns.value = value;
		ns.process_id = (int)getpid();
		ss_instance_->status_pub_.publish(ns);
		//ROS_INFO("published");
		return 0;
	}
};

#endif 
