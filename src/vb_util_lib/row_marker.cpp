/*
 * rviz_util.cpp
 *
 *  Created on: Jul 18, 2016
 *      Author: ubuntu
 */
#include <vb_util_lib/row_marker.h>
#include <visualization_msgs/msg/marker.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <tf2/transform_datatypes.h>
#include <tf2/LinearMath/Quaternion.h>
//#include <tf2/transform_broadcaster.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <vb_util_lib/frame_names.h>

RowMarker::RowMarker(rclcpp::Node::SharedPtr node, std::string topic): node_(node)
{
    pub_ = node_->create_publisher<visualization_msgs::msg::Marker>(topic, 2);
    id_ = 0;
}

void RowMarker::visualizeRow(int id, const brain_box_msgs::msg::Row& row) {
    visualization_msgs::msg::Marker marker;

    // Send the current location to RVIZ
    marker.header.frame_id = World_ENU;
    marker.header.stamp = node_->now();
    marker.ns = "row";
    marker.id = id;
    marker.type = visualization_msgs::msg::Marker::CUBE;
    marker.action = visualization_msgs::msg::Marker::ADD;

    // Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
    marker.pose.position.x = row.location[0];
    marker.pose.position.y = row.location[1];
    marker.pose.position.z = row.height / 2;
    // translate roll, pitch and yaw into a Quaternion
    tf2::Quaternion q;
    //        q.setRPY(3.14159, 0.0, 0.0);
    //        geometry_msgs::Quaternion qt;
    //        tf::quaternionTFToMsg(q, qt);
    //        printf("******* x[%0.7f] y[%0.7f] z[%0.7f] w[%0.7f] \n", qt.x, qt.y, qt.z, qt.w);
    q.setRPY(0.0, 0.0, row.orientation);
    geometry_msgs::msg::Quaternion odom_quat;
    odom_quat = tf2::toMsg(q);

    marker.pose.orientation.x = odom_quat.x;
    marker.pose.orientation.y = odom_quat.y;
    marker.pose.orientation.z = odom_quat.z;
    marker.pose.orientation.w = odom_quat.w;

    // Set the scale of the marker -- 1x1x1 here means 1m on a side
    marker.scale.x = row.length;
    marker.scale.y = row.width;
    marker.scale.z = row.height;

    // Set the color -- be sure to set alpha to something non-zero!
    marker.color.r = 0.0f;
    marker.color.g = 1.0f;
    marker.color.b = 0.0f;
    marker.color.a = 0.9;

    marker.lifetime = rclcpp::Duration(0);
    pub_->publish(marker);


    return;
}

void RowMarker::visualizeRows(const std::vector<brain_box_msgs::msg::Row>& rows, bool clear) {
//    rviz_visual_tools::RvizVisualTools rviz_interface("dummy_wall","/visualization_marker")
//    rviz_interface.deleteAllMarkers();

    int id = 0;
    for (auto it = rows.begin(); it != rows.end(); it++)
    {
        visualizeRow(id++, *it);
        // visualizeRow(id_++, *it); // use instead to leave turds
    }
    return;
}



