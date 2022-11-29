/*
 * CurrentENUKinematics.cpp
 *
 *  Created on: Feb 16, 2016
 *      Author: ubuntu
 */

#include <vb_util_lib/current_enu_kinematics.h>
#include <vb_util_lib/am_geometry.h>
#include <am_utils/am_ros2_utility.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

using std::placeholders::_1;

/*CurrentENUKinematics::CurrentENUKinematics(rclcpp::Node::SharedPtr node) : node_(node), transformer_(node)
{
}*/

CurrentENUKinematics::CurrentENUKinematics(rclcpp::Node::SharedPtr node, std::string topic,
        bool pub_on_change, double pub_rate) : node_(node), transformer_(node),
            Kinematics(node, topic!="" ? topic : am::am_topics::CTRL_VX_VEHICLE_CURRENTENU,
            		Asset_ENU, pub_on_change, pub_rate), odom_gen_(odom_random_()),
            		odom_prob_ud_(0.0, 1.0) {

	//tfListener_ = new tf2_ros::TransformListener(tfBuffer);
	tfBuffer_ = std::make_shared<tf2_ros::Buffer>(node->get_clock());
	br_ = std::make_shared<tf2_ros::TransformBroadcaster>(node);
	static_br_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(node);

    if (isPublisher())
    {
        node_->get_parameter_or("/locator/publish_enu_transform", publish_transform_, true);
        RCLCPP_INFO(node_->get_logger(),"CurrentENUKinematics: publish_transform_[%s]", publish_transform_ ? "TRUE" : "FALSE");

        od_sub_nh_ = node_->create_subscription<nav_msgs::msg::Odometry>("odometry/Asset_ENU", 1000, std::bind(&CurrentENUKinematics::updateODCB, this, _1));

        if (parent_frame_ != Asset_ENU && parent_frame_ != World_ENU)
        {
        	RCLCPP_ERROR(node_->get_logger(),"CurrentENUKinematics: parent_frame[%s] invalid. Setting to %s", parent_frame_.c_str(), Asset_ENU);
        	parent_frame_ = Asset_ENU;
        }
        else
        {
        	RCLCPP_ERROR(node_->get_logger(), "CurrentENUKinematics: parent_frame[%s]", parent_frame_.c_str());
        }

        update_timer_ = node_->create_wall_timer(am::toDuration(0.1), std::bind(&CurrentENUKinematics::updateTimerCB, this));
        //update_timer_.stop();
        enable_timer_ = false;

        // odom error testing
        node_->get_parameter_or("/locator/odom_trans_error_x", odom_trans_error_m_.x, 0.0);
        RCLCPP_INFO_STREAM(node_->get_logger(), "CurrentENUKinematics: odom_trans_error_x: " << odom_trans_error_m_.x);
        odom_trans_error_x_nd_ = new std::normal_distribution<double>(0, odom_trans_error_m_.x);

        node_->get_parameter_or("/locator/odom_trans_error_y", odom_trans_error_m_.y, 0.0);
        RCLCPP_INFO_STREAM(node_->get_logger(),"CurrentENUKinematics: odom_trans_error_y:" << odom_trans_error_m_.y);
        odom_trans_error_y_nd_ = new std::normal_distribution<double>(0, odom_trans_error_m_.y);

        node_->get_parameter_or("/locator/odom_trans_error_z", odom_trans_error_m_.z, 0.0);
        RCLCPP_INFO_STREAM(node_->get_logger(),"CurrentENUKinematics: odom_trans_error_z:" << odom_trans_error_m_.z);
        odom_trans_error_z_nd_ = new std::normal_distribution<double>(0, odom_trans_error_m_.z);

        node_->get_parameter_or("/locator/odom_trans_error_yaw", odom_yaw_error_r_, 0.0);
        RCLCPP_INFO_STREAM(node_->get_logger(),"CurrentENUKinematics: odom_trans_error_yaw:" << odom_yaw_error_r_);
        odom_trans_error_yaw_nd_ = new std::normal_distribution<double>(0, odom_yaw_error_r_);

        node_->get_parameter_or("/locator/odom_dropout_prob", odom_dropout_prob_, 0.0);
        RCLCPP_INFO_STREAM(node_->get_logger(),"CurrentENUKinematics: odom_dropout_prob:" << odom_dropout_prob_);

        node_->get_parameter_or("/locator/odom_hyperspace_prob", odom_hyperspace_prob_, 0.0);
        RCLCPP_INFO_STREAM(node_->get_logger(),"CurrentENUKinematics: odom_hyperspace_prob:" << odom_hyperspace_prob_);

        node_->get_parameter_or("/locator/odom_hyperspace_count", odom_hyperspace_count_, -1);
        RCLCPP_INFO_STREAM(node_->get_logger(),"CurrentENUKinematics: odom_hyperspace_count:" << odom_hyperspace_count_);

        node_->get_parameter_or("/locator/odom_hyperspace_offset_x", odom_hyperspace_trans_offset_m_.x, 0.0);
        RCLCPP_INFO_STREAM(node_->get_logger(),"CurrentENUKinematics: odom_hyperspace_offset_x:" << odom_hyperspace_trans_offset_m_.x);

        node_->get_parameter_or("/locator/odom_hyperspace_offset_y", odom_hyperspace_trans_offset_m_.y, 0.0);
        RCLCPP_INFO_STREAM(node_->get_logger(),"CurrentENUKinematics: odom_hyperspace_offset_y:" << odom_hyperspace_trans_offset_m_.y);

        node_->get_parameter_or("/locator/odom_hyperspace_offset_z", odom_hyperspace_trans_offset_m_.z, 0.0);
        RCLCPP_INFO_STREAM(node_->get_logger(),"CurrentENUKinematics: odom_hyperspace_offset_z:" << odom_hyperspace_trans_offset_m_.z);

        node_->get_parameter_or("/locator/odom_hyperspace_offset_yaw", odom_hyperspace_yaw_offset_r_, 0.0);
        RCLCPP_INFO_STREAM(node_->get_logger(),"CurrentENUKinematics: odom_hyperspace_offset_yaw:" << odom_hyperspace_yaw_offset_r_);
}
}


CurrentENUKinematics::~CurrentENUKinematics() {
  delete odom_trans_error_z_nd_;
  delete odom_trans_error_y_nd_;
  delete odom_trans_error_x_nd_;
}

void CurrentENUKinematics::doUpdate(bool start_pub) {
	Kinematics::doUpdate();
	if (publish_transform_)
	{
		sendENUTransform();
	}
// Going to explicitely call this from timer in the locator
//	publishOD();

	if (start_pub)
	{
		enable_timer_ = true;
	}
	else
	{
		enable_timer_ = false;
	}
}

void CurrentENUKinematics::sendENUTransform() {
    geometry_msgs::msg::TransformStamped ts;

//    if (!transformer_(transform_init_, drone_ENU, parent_frame_, ts))
//    {
//    	ROS_ERROR("CurrentENUKinematics::sendENUTransform: Cant find transform target[%s] source[%s]", drone_ENU, parent_frame_.c_str());
//    	return;
//    }
//
    ts.header.stamp = getUpdateTime();
    ts.header.frame_id = parent_frame_;
    ts.child_frame_id = drone_ENU;

    ts.transform.translation.x = position().x;
    ts.transform.translation.y = position().y;
    ts.transform.translation.z = position().z;

    Eigen::Quaterniond level_orientation;
    AM::levelOrientation(getOrientation(), level_orientation);

    ts.transform.rotation.x = level_orientation.x();
    ts.transform.rotation.y = level_orientation.y();
    ts.transform.rotation.z = level_orientation.z();
    ts.transform.rotation.w = level_orientation.w();
    br_->sendTransform(ts);

    return;
}

bool CurrentENUKinematics::getPoseStamped(std::string frame, geometry_msgs::msg::PoseStamped& pose) {
    geometry_msgs::msg::TransformStamped ts;

    geometry_msgs::msg::PoseStamped ps;
    getPose(ps.pose);
    ps.header.frame_id = getFrameId();
    ps.header.stamp = getUpdateTime();

    if (frame == body_FLU || frame == body_Level_FLU)
    {
    	// Don't do any transform
    	pose = ps;
    }
    else
    {
    	if (!transformer_.getTransform(frame, parent_frame_, ts))
    	{
    		RCLCPP_ERROR(node_->get_logger(), "CurrentENUKinematics::getPoseStamped: Cant find transform target[%s] source[%s]", drone_ENU, parent_frame_.c_str());
    		return false;
    	}
        tf2::doTransform(ps, pose, ts);
//        ROS_INFO_STREAM("ZZZZZ FROM: \n" << ps << "\n   ---- TO:\n" << pose);
    }

    return true;
}

void CurrentENUKinematics::updateODCB(const nav_msgs::msg::Odometry::SharedPtr msg) {
    // LOG_MSG("/odometry/Asset_ENU", msg, 1); // TODO (dan) bag_logger
    setPose(msg->header, msg->pose.pose);

    setAngularVelocity(msg->twist.twist.angular.x,
                   msg->twist.twist.angular.y,
                   msg->twist.twist.angular.z);
    setLinearVelocity(msg->twist.twist.linear.x,
                   msg->twist.twist.linear.y,
                   msg->twist.twist.linear.z);
    doUpdate();

    return;
}

void CurrentENUKinematics::publishOD() {
	if (od_pub_enable_)
	{
		nav_msgs::msg::Odometry od;
		od.header.stamp = getUpdateTime();
		if (!od_pub_ned_)
		{
			od.pose.pose.position.x = position().x;
			od.pose.pose.position.y = position().y;
			od.pose.pose.position.z = position().z;
			od.pose.covariance[0] = 0.0;
			od.header.frame_id = "ENU";
		}
		else
		{
			od.pose.pose.position.x = position().y;
			od.pose.pose.position.y = position().x;
			od.pose.pose.position.z = -position().z;
			od.pose.covariance[0] = 0.0;
			od.header.frame_id = "NED";
			// Convert attitude to the NED frame
			double ned_yaw = AM::wrap_pi(-(getAttitude().yaw - PI_2));
			AM::RPY_to_Quaternion(od.pose.pose.orientation, getAttitude().pitch, getAttitude().roll, ned_yaw);
		}

        // LOG_MSG(od_pub_topic_, od, 2); // TODO (dan) bag_logger

		// drop message if random prob is less than threshold
		if(odom_prob_ud_(odom_gen_) >= odom_dropout_prob_)
		{
                  // add in random error
                  if(odom_trans_error_m_.x > 0.0)
                  {
                    od.pose.pose.position.x += (*odom_trans_error_x_nd_)(odom_gen_);
                  }
                  if(odom_trans_error_m_.y > 0.0)
                  {
                    od.pose.pose.position.y += (*odom_trans_error_y_nd_)(odom_gen_);
                  }
                  if(odom_trans_error_m_.z > 0.0)
                  {
                    od.pose.pose.position.z += (*odom_trans_error_z_nd_)(odom_gen_);
                  }
                  if(odom_yaw_error_r_ > 0.0)
                  {
                    addYaw(od.pose.pose.orientation, (*odom_trans_error_yaw_nd_)(odom_gen_));
                  }

                  // if a count is set
                  if(odom_hyperspace_count_ >= 0)
                  {
                    RCLCPP_INFO_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000, "CurrentENUKinematics:hyperspace counter: %d", odom_hyperspace_counter_);

                    // apply the offset just once when the counter matches desired count
                    if(++odom_hyperspace_counter_ == odom_hyperspace_count_)
                    {
                      RCLCPP_INFO(node_->get_logger(), "CurrentENUKinematics:hyperspacing on counter: %d", odom_hyperspace_counter_);
                      od.pose.pose.position.x += odom_hyperspace_trans_offset_m_.x;
                      od.pose.pose.position.y += odom_hyperspace_trans_offset_m_.y;
                      od.pose.pose.position.z += odom_hyperspace_trans_offset_m_.z;
                      addYaw(od.pose.pose.orientation, odom_hyperspace_yaw_offset_r_);
                    }
                  }
                  // else, add offset if random prob is less than threshold
                  else if(odom_prob_ud_(odom_gen_) < odom_hyperspace_prob_)
                  {
                    RCLCPP_INFO_STREAM_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000, "CurrentENUKinematics:hyperspacing on probability");
                    od.pose.pose.position.x += odom_hyperspace_trans_offset_m_.x;
                    od.pose.pose.position.y += odom_hyperspace_trans_offset_m_.y;
                    od.pose.pose.position.z += odom_hyperspace_trans_offset_m_.z;
                    addYaw(od.pose.pose.orientation, odom_hyperspace_yaw_offset_r_);
                  }
                  od_pub_->publish(od);
		}
		else
		{
                  RCLCPP_INFO_STREAM_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000, "CurrentENUKinematics:dropping message");
		}
	}
}

void CurrentENUKinematics::addYaw(geometry_msgs::msg::Quaternion &msg_q, double yaw_add_r)
{
  tf2::Quaternion tf2_q;
  tf2::convert(msg_q , tf2_q);
  tf2::Matrix3x3 m(tf2_q);
  double roll_r, pitch_r, yaw_r;
  m.getRPY(roll_r, pitch_r, yaw_r);
  yaw_r += yaw_add_r;
  tf2_q.setRPY(roll_r, pitch_r, yaw_r);
  msg_q = tf2::toMsg(tf2_q);
}

void CurrentENUKinematics::publishOD_Enable(bool enable, std::string& topic, bool ned_frame) {
	if (od_pub_enable_)
	{
		RCLCPP_INFO(node_->get_logger(), "CurrentENUKinematics:publishOD_Enable Shutting down existing publisher");
	    //od_pub_->shutdown();
	    od_pub_enable_ = false;
	}

	if (enable)
	{
		RCLCPP_INFO(node_->get_logger(),"CurrentENUKinematics:publishOD_Enable Enabling new publisher");
		od_pub_topic_ = topic;
		od_pub_ = nh_->create_publisher<nav_msgs::msg::Odometry>(od_pub_topic_, 5);
		od_pub_enable_ = true;
		od_pub_ned_ = ned_frame;
	}

	return;
}

void CurrentENUKinematics::updateAGLCB(const std_msgs::msg::Float64::SharedPtr height_agl) {
}
