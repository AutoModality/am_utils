/*
 * pose_queue.h
 *
 *  Created on: Feb 9, 2021
 *      Author: ubuntu
 */

#ifndef AM_UTILS_INCLUDE_VB_UTIL_LIB_POSE_QUEUE_H_
#define AM_UTILS_INCLUDE_VB_UTIL_LIB_POSE_QUEUE_H_

#include <vector>

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>

class PoseQueue {
public:
	struct Pose {
		ros::Time stamp;
		double x;
		double y;
		double z;
		double roll;
		double pitch;
		double yaw;
	};

	PoseQueue();
	virtual ~PoseQueue();

	void addPose(ros::Time stamp, double X, double Y, double Z, double roll, double pitch, double yaw);

	bool getPose(ros::Time stamp, PoseQueue& pose, unsigned int& index);

	bool projectPose(PoseQueue::Pose& from_pose, PoseQueue::Pose& to_pose, unsigned int index);

private:
	// This is the index into poses_ where the next entry will be placed
	unsigned int next_entry_index_ {0};

	// This is the number of valid entries in poses_
	unsigned int pose_cnt_ {0};

	// Returns the next index to insert an entry into poses_
	unsigned int getNextIndex() { return next_entry_index_; }
	unsigned int getPrevIndex() { return next_entry_index_; }

	// Return the index into poses_ from the current_index.
	// Return false if there is none
	bool getPrevEntryIndex(unsigned int current_index, unsigned int& prev_index) {
		if (current_index == 0)
		{
			if (pose_cnt_ < poses_.size())
			{
				return false;
			}
			else
			{
				return true;
			}
		}
		return false;
	}

	bool getNextEntryIndex(unsigned int current_index, unsigned int& next_index) {
		return true;
	}

	std::vector<PoseQueue::Pose> poses_;
};

#endif /* AM_UTILS_INCLUDE_VB_UTIL_LIB_POSE_QUEUE_H_ */
