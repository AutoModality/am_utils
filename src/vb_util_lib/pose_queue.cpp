/*
 * pose_queue.cpp
 *
 *  Created on: Feb 9, 2021
 *      Author: ubuntu
 */

#include <vb_util_lib/pose_queue.h>

PoseQueue::PoseQueue() {
	poses_.reserve(100);
}

PoseQueue::~PoseQueue() {
	// TODO Auto-generated destructor stub
}

