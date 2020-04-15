/*
 * transformer.cpp
 *
 *  Created on: May 18, 2018
 *      Author: ubuntu
 */

#include <vb_util_lib/transformer.h>

#include <tf/transform_datatypes.h>

namespace am
{

Transformer::Transformer() {
    // TODO Auto-generated constructor stub

}

Transformer::~Transformer() {
    // TODO Auto-generated destructor stub
}

tf2_ros::Buffer& Transformer::tfBuffer() {
    if (!tfBuffer_instance_)
    {
        tfBuffer_instance_ = new tf2_ros::Buffer;
        tfListener_instance_ = new tf2_ros::TransformListener(*tfBuffer_instance_);
    }
    return *tfBuffer_instance_;
}

bool Transformer::getTransform(const std::string target, const std::string source,
                  geometry_msgs::TransformStamped& transform,  double wait, bool debug) {
    if (wait < 0.0)
    {
        wait = DEFAULT_TRANSFORM_WAIT;
    }

    try
    {
        transform = tfBuffer().lookupTransform(target, source, ros::Time(0), ros::Duration(wait));
    }
    catch (tf2::TransformException &ex) {
        ROS_ERROR_STREAM("Transformer::getTransform TRANSFORM ERROR source[" << source << "], target[" << target << "]  ERROR: " << ex.what());
        return false;
    }

    if (debug)
    {
        ROS_INFO_STREAM("Transformer::getTransform TRANSFORM = \n" << transform);
    }

    return true;
}

bool Transformer::getTransform(bool& initialized_flag, const std::string target, const std::string source,
                               geometry_msgs::TransformStamped& transform, double wait, bool debug) {
    if (!initialized_flag)
    {
        if (!getTransform(target, source, transform, wait, debug))
        {
            return false;
        }
        initialized_flag = true;
    }

    return true;
}

void Transformer::rotateOrientation(geometry_msgs::Quaternion& orientation, geometry_msgs::Quaternion& rotation,
                                    bool debug) {
    tf::Quaternion q_orig;
    tf::Quaternion q_rot;
    tf::Quaternion q_new;

    tf::quaternionMsgToTF(orientation, q_orig);
    tf::quaternionMsgToTF(rotation, q_rot);

    q_new = q_rot*q_orig;  // Calculate the new orientation
    q_new.normalize();
    tf::quaternionTFToMsg(q_new, orientation);

    if (debug)
    {
        double orig_yaw = tf::getYaw(q_orig) * 180.0 / 3.14159;
        double rot_yaw = tf::getYaw(q_rot) * 180.0 / 3.14159;
        double new_yaw =  tf::getYaw(q_new) * 180.0 / 3.14159;
        ROS_INFO("Transformer::rotateOrientation orientation[%0.3f] rotation[%0.3f] new[%0.3f]\n", orig_yaw, rot_yaw, new_yaw);
    }
}

} /* namespace am */
