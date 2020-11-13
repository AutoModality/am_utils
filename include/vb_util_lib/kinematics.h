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

#include <ros/ros.h>
#include <std_msgs/Header.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TwistStamped.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_datatypes.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
#include <Eigen/StdVector>
#include <Eigen/Geometry>
//#include <brain_box_msgs/VxKinematics.h>
#include <visualization_msgs/Marker.h>

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

    Kinematics();
    Kinematics(ros::NodeHandle *n, std::string topic, std::string frame_id,
               bool pub_on_change=false, double pub_rate=0.0);
    Kinematics(const geometry_msgs::PoseStamped& p);
    Kinematics(const geometry_msgs::PoseStamped& p, const std::string id);

    virtual ~Kinematics();

    void initROS(ros::NodeHandle *n);

    bool isInitializedState(unsigned int mask) const {
    	return (pose_initialized_ & mask) == mask;
    }
    /*
     * General setters. Will maintain consistency between the pose with quaternions
     * and the vehcile attitude which is RPY. Whenever you set one the other get calculated
     */
    void setPose(const geometry_msgs::PoseStamped& p, int debug=-1) {
        setPose(p.header.frame_id, p.header.stamp, p.pose, debug);
    }
    void setPose(const geometry_msgs::PoseWithCovarianceStamped& p, int debug=-1) {
        setPose(p.header.frame_id, p.header.stamp, p.pose.pose, debug);
    }
    void setPose(const std_msgs::Header& h, const geometry_msgs::Pose& p, int debug=-1) {
        setPose(h.frame_id, h.stamp, p, debug);
    }
    void setOdometry(const nav_msgs::Odometry::ConstPtr& msg, int debug=-1) {
    	setPose(msg->child_frame_id, msg->header.stamp, msg->pose.pose, debug);
        setLinearVelocity(msg->twist.twist.linear);
        setAngularVelocity(msg->twist.twist.angular);
    }
    void setOdometry(nav_msgs::Odometry& msg, int debug=-1) {
    	setPose(msg.child_frame_id, msg.header.stamp, msg.pose.pose, debug);
        setLinearVelocity(msg.twist.twist.linear);
        setAngularVelocity(msg.twist.twist.angular);
    }

    void getPose(geometry_msgs::Pose& p);
    // Get this position in the desired frame
    bool getPoseStamped(std::string& target_frame_id, geometry_msgs::PoseStamped& ps);
    void getOdometry(nav_msgs::Odometry& ms);

    /*
     * The following setters set the core attributes without regard to the frame of reference.
     * Therefore it assumes that the coordinates are in the same frame as this object.
     * Does not update the velocity.
     */
    void setPosition(const ros::Time& update_time, const geometry_msgs::Point& p, int debug=-1) {
    	setPosition(update_time, p.x, p.y, p.z, debug);
    }
    void setPosition(const ros::Time& update_time, const AM::Point& p, int debug=-1) {
    	setPosition(update_time, p.x, p.y, p.z, debug);
    }
    void setPosition(const ros::Time& update_time, double x, double y, double z, int debug=-1);
    void checkPosition(double x, double y, double z, int debug=-1);
    void setAltitude(double a) {
    	position_.z = a;

    }
    const AM::Point& position() const {
        return position_;
    }

    void setAttitude(const VehicleAttitude& va);
    void setAttitude(double roll, double pitch, double yaw, double throttle=0.0);
    void setAttitude(const geometry_msgs::Quaternion& q);
    void setAttitude(const Eigen::Quaterniond& q);
    void setYaw(double yaw, bool corrected=true);
    bool getYaw(double& yaw) {
    	yaw = attitude_.yaw;
    	// May want to change this to reflect that we want the yaw to be corrected
    	return isInitializedState(POSE_STATE::YAW);
    }
    void setOrientation(double w, double x, double y, double z);
    const VehicleAttitude& getAttitude() const {
        return attitude_;
    }
    const Eigen::Quaterniond& getOrientation() const {
        return orientation_;
    }

    void setVelocity(const geometry_msgs::Vector3& linear, const geometry_msgs::Vector3& angular);
    void setVelocity(const Eigen::Vector3d& linear, const Eigen::Vector3d& angular);

    void setAngularVelocity(const Eigen::Vector3d& vel) {
    	setAngularVelocity(vel[0], vel[1], vel[2]);
    }
    void setAngularVelocity(const geometry_msgs::Vector3& vel) {
    	setAngularVelocity(vel.x, vel.y, vel.z);
    }
    void setAngularVelocity(double x, double y, double z);
    const Eigen::Vector3d& getAngularVelocity() const {
        return velAngular_;
    }
    void setLinearVelocity(const Eigen::Vector3d& vel) {
        setLinearVelocity(vel[0], vel[1], vel[2]);
    }
    void setLinearVelocity(const geometry_msgs::Vector3& vel) {
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

    void setUpdateTime(const ros::Time& updateTime) {
        update_time_ = updateTime;
    }
    ros::Time getUpdateTime() const {
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

    nav_msgs::Odometry& getMsg() {
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
    ros::NodeHandle *nh_;

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

    // This is the core method to set the pose taking into account the frame of the data as compared to the frame
    //   of this kinematics object.
    void setPose(const std::string& source_frame, const ros::Time& update_time, const geometry_msgs::Pose& p, int debug=-1);

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
    geometry_msgs::Pose p_;
    geometry_msgs::PoseStamped ps_;

    /**
     * Update time of the Kinematics
     */
    ros::Time update_time_;

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
    ros::Publisher pub_nh_;
    ros::Subscriber sub_nh_;
    ros::Timer pub_timer_;

    /**
     * Watchdog related stuff
     */
    ros::Timer watchdog_timer_;
    bool enable_watchdog_ {true};
    bool update_stale_ {false};
    void watchdogTimerCB(const ros::TimerEvent& te);

    /**
     * Timer callback for publishing stats messages
     */
    void publishTimerCB(const ros::TimerEvent& te) {
        sendKinematics();
    }

    /**
     * Callback to handle updates to Kinematics
     */
    nav_msgs::Odometry msg_;
    bool log_enabled_ {false};
    void updateCB(const nav_msgs::Odometry::ConstPtr& msg);

    // Debug flag
    bool debug_ {false};
    bool enable_velocity_debug_ {false};

    /**
     * RVIZ debug display
     */
    bool rviz_debug_ {false};
    unsigned long rviz_point_cnt {0};
    visualization_msgs::Marker marker;
    ros::Publisher rviz_uav_marker_pub_;
};

#endif /* BRAIN_BOX_SRC_AM_SP_KINEMATICS_H_ */
