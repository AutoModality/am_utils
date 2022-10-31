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
#include <rclcpp/rclcpp.hpp>
#include <brain_box_msgs/msg/row_view.hpp>
#include <visualization_msgs/msg/marker.hpp>

class RowMarker
{
public:
    RowMarker(rclcpp::Node::SharedPtr node): node_(node)
    {};
    RowMarker(rclcpp::Node::SharedPtr n, std::string topic);

    void visualizeRow(int id, const brain_box_msgs::msg::Row& row);

    void visualizeRows(const std::vector<brain_box_msgs::msg::Row>& rows, bool clear=true);

private:
    rclcpp::Node::SharedPtr node_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr pub_;
    int id_ {0};
};




#endif /* VISBOX_PACKAGES_COMMON_UTILS_UTIL_LIB_INCLUDE_VB_UTIL_LIB_ROW_MARKER_H_ */
