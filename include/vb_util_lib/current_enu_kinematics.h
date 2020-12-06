/*
 * CurrentENUKinematics.h
 *
 *  Created on: Feb 16, 2016
 *      Author: ubuntu
 */

#ifndef VISBOX_PACKAGES_COMMON_UTILS_UTIL_LIB_INCLUDE_CURRENTENUKINEMATICS_H_
#define VISBOX_PACKAGES_COMMON_UTILS_UTIL_LIB_INCLUDE_CURRENTENUKINEMATICS_H_

#include <random>
#include <vb_util_lib/kinematics.h>
#include <vb_util_lib/topics.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Float64.h>

#include <vb_util_lib/transformer.h>

class CurrentENUKinematics: public Kinematics {
public:
    CurrentENUKinematics();

    CurrentENUKinematics(ros::NodeHandle *n, std::string topic=am::am_topics::CTRL_VX_VEHICLE_CURRENTENU,
            bool pub_on_change=false, double pub_rate=0.0);

    virtual ~CurrentENUKinematics();

    /**
     * Overrides the normal Kinematics update to also publish the transform
     */
    void doUpdate(bool start_pub = false);

    void sendENUTransform();

    bool getPoseStamped(std::string frame, geometry_msgs::PoseStamped& pose);

    void updateODCB(const nav_msgs::Odometry::ConstPtr& msg);

    // Method to publish the position to the Odometry topic of choice
    void publishOD();

    // Method to enable the publishing of the odometry
    void publishOD_Enable(bool enable, std::string& topic, bool ned_frame);

private:
    ros::Subscriber od_sub_nh_;

    ros::Subscriber agh_sub_nh_;

    // Variables used to publish the odometry when it is changed
    ros::Publisher od_pub_;
    std::string od_pub_topic_ {""};
	bool od_pub_enable_ {false};
    bool od_pub_ned_ {false};

    double last_odometry_z_m_ {0};

    // Set to true if the drone transform should be published
    bool publish_transform_ {false};

    // This is the parent frame of the drone_ENU publishing should be either World_ENU or Asset_ENU
    std::string parent_frame_ {Asset_ENU};

    AM::Point xfer_offset_ {0.0, 0.0, 0.0};

    // error testing
    std::random_device odom_random_;  //Will be used to obtain a seed for the random number engine
    std::mt19937  odom_gen_; //Standard mersenne_twister_engine seeded with rd()
    geometry_msgs::Point odom_trans_error_m_;
    double odom_yaw_error_r_;
    double odom_dropout_prob_;
    double odom_hyperspace_prob_;
    int odom_hyperspace_count_;
    int odom_hyperspace_counter_ {0};
    geometry_msgs::Point odom_hyperspace_trans_offset_m_;
    double odom_hyperspace_yaw_offset_r_;
    std::normal_distribution<double> *odom_trans_error_x_nd_;
    std::normal_distribution<double> *odom_trans_error_y_nd_;
    std::normal_distribution<double> *odom_trans_error_z_nd_;
    std::normal_distribution<double> *odom_trans_error_yaw_nd_;
    std::uniform_real_distribution<double> odom_prob_ud_;

    tf2_ros::Buffer tfBuffer;

    tf2_ros::TransformBroadcaster* br;
    tf2_ros::StaticTransformBroadcaster* static_br;

    am::Transformer transformer_;
    bool transform_init_ {false};

    /**
     * Timer callback for doing updates
     */
    ros::Timer update_timer_;
    void updateTimerCB(const ros::TimerEvent& te) {
    	doUpdate(true);
    }

    void updateAGLCB(const std_msgs::Float64ConstPtr& height_agl);

    void addYaw(geometry_msgs::Quaternion &q, double yaw_add_r);
};

#endif /* VISBOX_PACKAGES_COMMON_UTILS_UTIL_LIB_INCLUDE_CURRENTENUKINEMATICS_H_ */
