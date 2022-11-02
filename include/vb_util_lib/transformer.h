/*
 * transformer.h
 *
 *  Created on: May 18, 2018
 *      Author: ubuntu
 */

#ifndef VISBOX_PACKAGES_COMMON_UTILS_UTIL_LIB_INCLUDE_VB_UTIL_LIB_TRANSFORMER_H_
#define VISBOX_PACKAGES_COMMON_UTILS_UTIL_LIB_INCLUDE_VB_UTIL_LIB_TRANSFORMER_H_

#include <string>

#include <rclcpp/rclcpp.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <geometry_msgs/msg/quaternion.hpp>

#include <eigen3/Eigen/Geometry>
#include <eigen3/Eigen/StdVector>
#include <vb_util_lib/rotate.h>

namespace am
{
#define DEFAULT_TRANSFORM_WAIT 0.07

class Transformer
{
public:
    Transformer(rclcpp::Node::SharedPtr node);
    virtual ~Transformer();

    tf2_ros::Buffer& tfBuffer();

    /// Used to fetch the latest transform between two frames
    /// Return true if it works and false otherwise
    bool getTransform(const std::string target, const std::string source,
                      geometry_msgs::msg::TransformStamped& transform,  double wait = -1.0, bool debug = false);

    /// Used to fetch the latest transform between two frames
    /// Checks the initialized flag and will not call the lookup if it is true.
    /// Otherwise it attempts to lookup the transform and set the initialized flag appropriately.
    /// Return true if the transform is valid and false otherwise
    bool getTransform(bool& initialized_flag, const std::string target, const std::string source,
                      geometry_msgs::msg::TransformStamped& transform,  double wait = -1.0, bool debug = false);

    /// Used to rotate the "orientation" coordinates by the "rotation" coordinates
    void rotateOrientation(geometry_msgs::msg:: Quaternion& orientation, geometry_msgs::msg::Quaternion& rotation, bool debug = false);

    /// Used to rotate a vector from source frame to target frame.
    /// Returns false if failed,
    bool rotateVector(const std::string target_frame, const std::string source_frame,
    		Eigen::Vector3d& target_vector, const Eigen::Vector3d& source_vector, bool debug = false);

private:
    // transform buffer management. Use factory creation method to auto create the tfBuffer
    // later make this a static to save space and make more efficient
    tf2_ros::Buffer *tfBuffer_instance_ {NULL};
    tf2_ros::TransformListener *tfListener_instance_;

    rclcpp::Node::SharedPtr node_;
};

} /* namespace am */

#endif /* VISBOX_PACKAGES_COMMON_UTILS_UTIL_LIB_INCLUDE_VB_UTIL_LIB_TRANSFORMER_H_ */
