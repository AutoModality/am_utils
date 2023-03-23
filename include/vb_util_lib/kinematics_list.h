/*
 * am_sp_kinematics_list.h
 *
 *  Created on: Jun 3, 2015
 *      Author: ubuntu
 */

#ifndef BRAIN_BOX_SRC_AM_SP_KINEMATICS_LIST_H_
#define BRAIN_BOX_SRC_AM_SP_KINEMATICS_LIST_H_

#include "string.h"
#include <iostream>

#include <am_utils/am_ros2_utility.h>

#include <brain_box_msgs/msg/target_pose.hpp>
#include <brain_box_msgs/msg/target_pose_array.hpp>

#include <vb_util_lib/kinematics.h>

class KinematicsList {
public:
	KinematicsList(rclcpp::Node::SharedPtr node = am::Node::node);
	virtual ~KinematicsList();

	Kinematics* updateList(const brain_box_msgs::msg::TargetPoseArray::SharedPtr msg);
	Kinematics* findNode(std::string id);
	Kinematics* trackNode(std::string id);
	Kinematics* updateNode(const brain_box_msgs::msg::TargetPose* pose);
	void addNode(Kinematics* node);
	bool isFocus(std::string id) {return (focus_node == NULL ? false : (id == focus_node->id));}
	bool isFocus(const Kinematics* n) {return (n == NULL ? false : isFocus(n->getObjectId()));}
	unsigned int cleanList(rclcpp::Time age);
	void print();
	void print(std::string s);

	unsigned int getCnt() const {
		return cnt;
	}

	Kinematics* getFocus() {
		return (focus_node == NULL ? NULL : focus_node->node);
	}

protected:
	bool deleteNode(std::string id);

private:
	class KinematicsNode {
	public:
		KinematicsNode(rclcpp::Node::SharedPtr n, Kinematics* p);
		virtual ~KinematicsNode();

		rclcpp::Node::SharedPtr nh_;

		Kinematics* node {NULL};
		std::string id {NULL};
		KinematicsNode* next {NULL};
		KinematicsNode* prev {NULL};
		bool clean {true};
	};



	KinematicsNode* findNodePtr(std::string id);
	void deleteNode(KinematicsNode* p);

	// The head of the linked list
	KinematicsNode* head {NULL};

	// Node currently under focus
	KinematicsNode*	focus_node {NULL};

	// Number of elements in linked list
	unsigned int cnt = 0;

	// Last update time
	rclcpp::Time last_update;

	rclcpp::Node::SharedPtr node_;

};

#endif /* BRAIN_BOX_SRC_AM_SP_KINEMATICS_LIST_H_ */
