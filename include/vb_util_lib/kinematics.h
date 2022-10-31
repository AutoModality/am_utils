/*
 * Kinematics.h
 *
 *  Created on: Apr 24, 2015
 *      Author: ubuntu
 */

#ifndef BRAIN_BOX_SRC_AM_SP_KINEMATICS_H_
#define BRAIN_BOX_SRC_AM_SP_KINEMATICS_H_

#include <stdio.h>
#include <termios.h>
#include <unistd.h>
#include <sys/types.h>
#include <string.h>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/header.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2/transform_datatypes.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <eigen3/Eigen/StdVector>
#include <eigen3/Eigen/Geometry>
#include <visualization_msgs/msg/marker.hpp>

#include <vb_util_lib/topics.h>
#include <vb_util_lib/control_util.h>
#include <vb_util_lib/am_geometry.h>
#include <vb_util_lib/transformer.h>
#include <vb_util_lib/frame_names.h>

/**
 * Wrapper method for vehicle Kinematics
 * Can use this class either as a publisher or a subscriber for the state information.
 * Note that if a publisher of the information then you should explicitly call the
 * doUpdate method after changing any of the attributes so that it will get published
 * to the appropriate topic.
 */
class Kinematics {
public:
	enum POSE_STATE {
		X = 0x01,
		Y = 0x02,
		Z = 0x04,
		XYZ = 0x07,
		ROLL = 0x10,
		PITCH = 0x20,
		YAW = 0x40,
		RPY = 0x70,
		ALL_POSE = 0x77,
		YAW_CORRECTED = 0x80,
		COMPLETE = 0xf7
	};

    Kinematics(rclcpp::Node::SharedPtr node);
    Kinematics(rclcpp::Node::SharedPtr node, std::string topic, std::string frame_id,
               bool pub_on_change=false, double pub_rate=0.0);
    Kinematics(rclcpp::Node::SharedPtr node, const geometry_msgs::msg::PoseStamped& p);
    Kinematics(rclcpp::Node::SharedPtr node, const geometry_msgs::msg::PoseStamped& p, const std::string id);

    virtual ~Kinematics();

    void initROS(rclcpp::Node::SharedPtr node);

    bool isInitializedState(unsigned int mask) const {
    	return (pose_initialized_ & mask) == mask;
    }
    /*
     * General setters. Will maintain consistency between the pose with quaternions
     * and the vehcile attitude which is RPY. Whenever you set one the other get calculated
     */
    void setPose(const geometry_msgs::msg::PoseStamped& p, int debug=-1) {
        setPose(p.header.frame_id, p.header.stamp, p.pose, debug);
    }
    void setPose(const geometry_msgs::msg::PoseWithCovarianceStamped& p, int debug=-1) {
        setPose(p.header.frame_id, p.header.stamp, p.pose.pose, debug);
    }
    void setPose(const std_msgs::msg::Header& h, const geometry_msgs::msg::Pose& p, int debug=-1) {
        setPose(h.frame_id, h.stamp, p, debug);
    }
	void setPose(const rclcpp::Time& update_time, double X, double Y, double Z,
			double Qw, double Qx, double Qy, double Qz, int debug=-1) {
		setPose(frame_id_, update_time, X, Y, Z, Qw, Qx, Qy, Qz, debug);
	}

	void setPose(const std::string& source_frame, const rclcpp::Time& update_time, double X, double Y, double Z,
			double Qw, double Qx, double Qy, double Qz, int debug=-1);

    void setOdometry(const nav_msgs::msg::Odometry::SharedPtr& msg, int debug=-1) {
    	setPose(msg->child_frame_id, msg->header.stamp, msg->pose.pose, debug);
        setLinearVelocity(msg->twist.twist.linear);
        setAngularVelocity(msg->twist.twist.angular);
    }
    void setOdometry(nav_msgs::msg::Odometry& msg, int debug=-1) {
    	setPose(msg.child_frame_id, msg.header.stamp, msg.pose.pose, debug);
        setLinearVelocity(msg.twist.twist.linear);
        setAngularVelocity(msg.twist.twist.angular);
    }

    void getPose(geometry_msgs::msg::Pose& p);
    // Get this position in the desired frame
    bool getPoseStamped(std::string& target_frame_id, geometry_msgs::msg::PoseStamped& ps);
    void getOdometry(nav_msgs::msg::Odometry& ms);

    /*
     * The following setters set the core attributes without regard to the frame of reference.
     * Therefore it assumes that the coordinates are in the same frame as this object.
     * Does not update the velocity.
     */
    void setPosition(const rclcpp::Time& update_time, const geometry_msgs::msg::Point& p, int debug=-1) {
    	setPosition(update_time, p.x, p.y, p.z, debug);
    }
    void setPosition(const rclcpp::Time& update_time, const AM::Point& p, int debug=-1) {
    	setPosition(update_time, p.x, p.y, p.z, debug);
    }
    void setPosition(const rclcpp::Time& update_time, double x, double y, double z, int debug=-1);
    void checkPosition(double x, double y, double z, int debug=-1);
    void setAltitude(const rclcpp::Time& update_time, double a) {
    	position_.z = a;
    	setUpdateTime(update_time, 0);
    }
    const AM::Point& position() const {
        return position_;
    }

    void setAttitude(const rclcpp::Time& update_time, double Qw, double Qx, double Qy, double Qz, bool yaw_corrected=true) {
    	setOrientation(Qw, Qx, Qy, Qz, yaw_corrected);
    	setUpdateTime(update_time, 1);
    }

//    void setAttitude(const VehicleAttitude& va, bool yaw_corrected=true);
//    void setAttitude(double roll, double pitch, double yaw, bool yaw_corrected=true, double throttle=0.0);
//    void setAttitude(const geometry_msgs::Quaternion& q, bool yaw_corrected=true);
//    void setAttitude(const Eigen::Quaterniond& q, bool yaw_corrected=true);
    void setYaw(double yaw, bool corrected=true);
    bool getYaw(double& yaw) {
    	yaw = attitude_.yaw;
    	// May want to change this to reflect that we want the yaw to be corrected
    	return isInitializedState(POSE_STATE::YAW);
    }
    void setOrientation(double w, double x, double y, double z, bool yaw_corrected=true);
    const VehicleAttitude& getAttitude() const {
        return attitude_;
    }
    const Eigen::Quaterniond& getOrientation() const {
        return orientation_;
    }

    void setVelocity(const geometry_msgs::msg::Vector3& linear, const geometry_msgs::msg::Vector3& angular);
    void setVelocity(const Eigen::Vector3d& linear, const Eigen::Vector3d& angular);

    void setAngularVelocity(const Eigen::Vector3d& vel) {
    	setAngularVelocity(vel[0], vel[1], vel[2]);
    }
    void setAngularVelocity(const geometry_msgs::msg::Vector3& vel) {
    	setAngularVelocity(vel.x, vel.y, vel.z);
    }
    void setAngularVelocity(double x, double y, double z);
    const Eigen::Vector3d& getAngularVelocity() const {
        return velAngular_;
    }
    void setLinearVelocity(const Eigen::Vector3d& vel) {
        setLinearVelocity(vel[0], vel[1], vel[2]);
    }
    void setLinearVelocity(const geometry_msgs::msg::Vector3& vel) {
        setLinearVelocity(vel.x, vel.y, vel.z);
    }
    void setLinearVelocity(double x, double y, double z);
    double getLinearVelocityMagnitude() {
        return velLinear_.norm();
    }
    const Eigen::Vector3d& getLinearVelocity() const {
        return velLinear_;
    }

    /**
     * Get the linear velocity with respect to some frame of reference
     */
    bool getLinearVelocity(std::string target_frame_id, Eigen::Vector3d& velocity);

    bool isPoseInitialized() const {
    	return isInitializedState(POSE_STATE::ALL_POSE);
    }
    bool isHeadingCorrected() const {
    	return isInitializedState(POSE_STATE::YAW_CORRECTED);
    }
    bool isPoseReady() const {
    	return isInitializedState(POSE_STATE::COMPLETE);
    }
    void resetInitialized() {
        update_cnt_ = 0;
        pose_initialized_ = 0;
    }

    rclcpp::Time getUpdateTime() const {
        return update_time_;
    }

    void setObjectId(std::string objectId) {
        object_id_ = objectId;
    }
    std::string getObjectId() const {
        return object_id_;
    }

    void test();
    void print(std::string prefix, bool loc_only=false);
    void publishMarker();

    static void convertRFUtoENU(Kinematics& RFU, double yaw, Kinematics* ENU);
    static void convertFLUtoENU(Kinematics& FLU, Kinematics& ref_ENU, Kinematics* ENU);
    static AM::Point convertRFUtoENUoff(double right, double front, double up, double yaw);
    static AM::Point convertFLUtoENUoff(double front, double left, double up, double yaw);
    AM::Point convertFLUtoENUoff(double front, double left, double up) {
        return convertFLUtoENUoff(front, left, up, getAttitude().yaw);
    }
    AM::Point convertRFUtoENUoff(double front, double left, double up) {
        return convertRFUtoENUoff(front, left, up, getAttitude().yaw);
    }

    void sendKinematics();

    /**
     * Install a callback handler.
     * Use the following sytax to call this function:
     *   setCallback(std::bind(&MyClass::Callback, this, std::placeholders::_1));
     */
    void setCallback(std::function<void(Kinematics*)> callback) {
        callback_ = callback;
    }

    /**
     * Check for update condition and publish Kinematics message if necessary
     */
    virtual void doUpdate();

    bool isUpdated() const {
        return updated_flag_;
    }

    bool isPublisher() const {
        return publisher_;
    }

    unsigned long getUpdateCnt() const {
        return update_cnt_;
    }

    void setLogEnabled(bool logEnabled) {
        log_enabled_ = logEnabled;
    }

    nav_msgs::msg::Odometry& getMsg() {
        return msg_;
    }

    const std::string& getFrameId() const {
        return frame_id_;
    }

    const AM::Point& getPosition() const {
        return position_;
    }

	bool isUpdateStale() const {
		return update_stale_;
	}

	void setSpeedFilter(double speedFilter) {
		speed_filter_ = speedFilter;
	}

protected:
    rclcpp::Node::SharedPtr nh_;

    void setUpdateTime(const rclcpp::Time& updateTime, int source) {
    	update_time_cnt_++;
    	if (updateTime.seconds() == update_time_.seconds())
    	{
            duplicate_time_cnt_++;
    		//RCLCPP_WARN_THROTTLE(nh_->get_logger(), nh_->get_clock(), 15,"Kinematics::setUpdateTime duplicated now[%f] source[%d] time_cnt[%lu] dup_time_cnt[%lu] update_cnt[%lu] new time[%f] same as previous time[%f]",
    				//nh_->get_clock()::now().seconds(), source, update_time_cnt_, duplicate_time_cnt_, update_cnt_, updateTime.toSec(), update_time_.toSec());
    	}
        update_time_ = updateTime;
    }

    void setUpdated(bool updated) {
        updated_flag_ = updated;
        if (updated_flag_)
        {
            update_cnt_++;
        }
    }

    /**
     * Callback function pointer
     */
    std::function<void(Kinematics*)> callback_ {NULL};

    void setInitialized(POSE_STATE state) {
    	pose_initialized_ |= state;
    }

private:
    // general purpose transformer
    am::Transformer transformer_;

    // This is the core methods to set the pose taking into account the frame of the data as compared to the frame
    //   of this kinematics object.
    void setPose(const std::string& source_frame, const rclcpp::Time& update_time, const geometry_msgs::msg::Pose& p, int debug=-1) {
    	setPose(source_frame, update_time, p.position.x, p.position.y, p.position.z,
    			p.orientation.w, p.orientation.x, p.orientation.y, p.orientation.z, debug);
    }

    /**
     * Used to set the attitude
     */
    void setAttitude(const VehicleAttitude& va, bool yaw_corrected=true, double throttle=0.0) {
    	setAttitude(va.roll, va.pitch, va.yaw, yaw_corrected, throttle);
    }
    void setAttitude(double roll, double pitch, double yaw, bool yaw_corrected=true, double throttle=0.0);

    void setAttitude(const geometry_msgs::msg::Quaternion& q, bool yaw_corrected=true) {
    	setOrientation(q.w, q.x, q.y, q.z, yaw_corrected);
    }
    void setAttitude(const Eigen::Quaterniond& q, bool yaw_corrected=true) {
    	setOrientation(q.w(), q.x(), q.y(), q.z(), yaw_corrected);
    }
    void setAttitude(double Qw, double Qx, double Qy, double Qz, bool yaw_corrected=true) {
    	setOrientation(Qw, Qx, Qy, Qz, yaw_corrected);
    }

    /**
     * Used to update the velocity given a new position
     */
    void updateVelocity(double pos_x, double pos_y, double pos_z, double delta_t, int debug=-1);

    /**
     *  Identifier for the object represented by these kinematics
     */
    std::string object_id_ {"foo"};

    /**
     * Current location
     */
    AM::Point position_;

    /**
     *  Current orientation
     */
    Eigen::Quaterniond orientation_;

    /**
     *  Current attitude in RPY
     */
    VehicleAttitude attitude_;

    /**
     *  Linear velocity
     */
    Eigen::Vector3d velLinear_;

    /**
     *  Cached linear velocity in a certain frame of reference
     */
    Eigen::Vector3d velLinearFrame_;

    /**
     * Speed filter constant
     */
    double speed_filter_ {1.0};

    /**
     *  Angular velocity
     */
    Eigen::Vector3d velAngular_;


    /**
     *  message caches
     */
    geometry_msgs::msg::Pose p_;
    geometry_msgs::msg::PoseStamped ps_;

    /**
     * Update time of the Kinematics
     */
    rclcpp::Time update_time_;
    /**
     * Incremented when the time is updated
     */
    unsigned long update_time_cnt_ {0};
    /**
     * Incremented when the incoming time is the same as that previously reported.
     */
    unsigned long duplicate_time_cnt_ {0};

    /**
     * frame id for the kinematics object specified in the constructor when the class is created
     */
    std::string frame_id_;

    /**
     * Method to initialize the attributes
     */
    void _init();

    /**
     * Bit Flag to determine if the attributes are initialized
     */
    unsigned int pose_initialized_ {0};
    unsigned long update_cnt_ {0};
    unsigned long last_update_cnt_ {0};

    // The following support the wrapper function

    /**
     * topic to publish or subscribe to updates
     */
    std::string topic_ {""};

    /**
     * Updated flag. Used to poll the class for updates
     */
    bool updated_flag_ {false};

    /**
     * Publish on update flag
     */
    bool pub_on_change_ = false;

    /**
     * Rate to publish status if not 0
     */
    double pub_rate_ = 0.0;

    /**
     * True if this instance is publishing versus consuming the data
     */
    bool publisher_ {false};

    bool ros_init_ {false};
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_nh_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_nh_;
    rclcpp::TimerBase::SharedPtr pub_timer_;

    /**
     * Watchdog related stuff
     */
    rclcpp::TimerBase::SharedPtr watchdog_timer_;
    bool enable_watchdog_ {true};
    bool update_stale_ {false};
    void watchdogTimerCB();

    /**
     * Timer callback for publishing stats messages
     */
    void publishTimerCB() {
        sendKinematics();
    }

    /**
     * Callback to handle updates to Kinematics
     */
    nav_msgs::msg::Odometry msg_;
    bool log_enabled_ {false};
    void updateCB(const nav_msgs::msg::Odometry::SharedPtr msg);

    // Debug flag
    bool debug_ {false};
    bool enable_velocity_debug_ {false};

    /**
     * RVIZ debug display
     */
    bool rviz_debug_ {false};
    unsigned long rviz_point_cnt {0};
    visualization_msgs::msg::Marker marker;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr rviz_uav_marker_pub_;
};

#endif /* BRAIN_BOX_SRC_AM_SP_KINEMATICS_H_ */
