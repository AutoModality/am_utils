/*
 * rviz_util.h
 *
 *  Created on: Jul 18, 2016
 *      Author: ubuntu
 */

#ifndef VISBOX_PACKAGES_COMMON_UTILS_UTIL_LIB_INCLUDE_VB_UTIL_LIB_ROW_MARKER_H_
#define VISBOX_PACKAGES_COMMON_UTILS_UTIL_LIB_INCLUDE_VB_UTIL_LIB_ROW_MARKER_H_

#include <vector>
#include <string.h>
#include <ros/ros.h>
#include <brain_box_msgs/RowView.h>

class RowMarker
{
public:
    RowMarker() {};
    RowMarker(ros::NodeHandle& n, std::string topic);

    void visualizeRow(int id, const brain_box_msgs::Row& row);

    void visualizeRows(const std::vector<brain_box_msgs::Row>& rows, bool clear=true);

private:
    ros::Publisher pub_;
    int id_;
};




#endif /* VISBOX_PACKAGES_COMMON_UTILS_UTIL_LIB_INCLUDE_VB_UTIL_LIB_ROW_MARKER_H_ */
