/*
 * latency_wrapper.h
 *
 *  Created on: Jun 24, 2018
 *      Author: ubuntu
 */

#ifndef AM_UTILS_INCLUDE_AM_UTILS_LATENCY_WRAPPER_H_
#define AM_UTILS_INCLUDE_AM_UTILS_LATENCY_WRAPPER_H_

// See the bottom of this file for an example of how to use this class

#include <functional>

#include <ros/ros.h>

#include <am_utils/Latency.h>
#include <am_utils/LatencyHeader.h>
#include <am_utils/Latency_Image.h>
#include <am_utils/Latency_PointCloud.h>
#include <am_utils/Latency_LaserScan.h>
#include <am_utils/Latency_PointCloud2.h>
#include <am_utils/Latency_PoseStamped.h>
#include <am_utils/Latency_PoseWithCovarianceStamped.h>
#include <am_utils/Latency_TwistStamped.h>
#include <am_utils/Latency_TwistWithCovarianceStamped.h>
#include <am_utils/Latency_Odometry.h>

#include <am_utils/am_statistics.h>

#define BIND_CB(cb) std::bind(&cb, this, std::placeholders::_1)

namespace am
{
// List of the various function calback types supported
typedef std::function<void(const geometry_msgs::PoseStamped&)> PoseStamped_Func;
typedef std::function<void(const geometry_msgs::PoseWithCovarianceStamped&)> PoseWithCovarianceStamped_Func;
typedef std::function<void(const geometry_msgs::TwistStamped&)> TwistStamped_Func;
typedef std::function<void(const geometry_msgs::TwistWithCovarianceStamped&)> TwistWithCovarianceStamped_Func;
typedef std::function<void(const nav_msgs::Odometry&)> Odometry_Func;
typedef std::function<void(const sensor_msgs::Image&)> Image_Func;
typedef std::function<void(const sensor_msgs::LaserScan&)> LaserScan_Func;
typedef std::function<void(const sensor_msgs::PointCloud&)> PointCloud_Func;
typedef std::function<void(const sensor_msgs::PointCloud2&)> PointCloud2_Func;


class LatencyWrapper
{
public:
    LatencyWrapper() {
        resetStats();
    };
    LatencyWrapper(ros::NodeHandle* n, unsigned char id, double max_epoch_time = 1.0);
    virtual ~LatencyWrapper();

    void setNodeHandle(ros::NodeHandle* n) {
        nh_ = n;
    }

    void setNodeId(unsigned char id) {
        node_id_ = id;
    }

    // Methods for each of the wrapper types supported

    // ===== PoseStamped
    void subscribe(std::string topic, PoseStamped_Func callback);
    PoseStamped_Func pose_stamped_cb_ {NULL};
    void poseStamped_CB(const am_utils::Latency_PoseStamped::ConstPtr& msg);

    // ===== PoseWithCovarianceStamped
    void subscribe(std::string topic, PoseWithCovarianceStamped_Func callback);
    PoseWithCovarianceStamped_Func pose_with_covariance_stamped_cb_ {NULL};
    void poseWithCovarianceStamped_CB(const am_utils::Latency_PoseWithCovarianceStamped::ConstPtr& msg);

    // ===== TwistStamped
    void subscribe(std::string topic, TwistStamped_Func callback);
    TwistStamped_Func twist_stamped_cb_ {NULL};
    void twistStamped_CB(const am_utils::Latency_TwistStamped::ConstPtr& msg);

    // ===== TwistWithCovarianceStamped
    void subscribe(std::string topic, TwistWithCovarianceStamped_Func callback);
    TwistWithCovarianceStamped_Func twist_with_covariance_stamped_cb_ {NULL};
    void twistWithCovarianceStamped_CB(const am_utils::Latency_TwistWithCovarianceStamped::ConstPtr& msg);

    // ===== Odometry
    void subscribe(std::string topic, Odometry_Func callback);
    Odometry_Func odometry_cb_ {NULL};
    void odometry_CB(const am_utils::Latency_Odometry::ConstPtr& msg);

    // ===== Image
    void subscribe(std::string topic, Image_Func callback);
    Image_Func image_cb_ {NULL};
    void image_CB(const am_utils::Latency_Image::ConstPtr& msg);

    // ===== LaserScan
    void subscribe(std::string topic, LaserScan_Func callback);
    LaserScan_Func laser_scan_cb_ {NULL};
    void laserScan_CB(const am_utils::Latency_LaserScan::ConstPtr& msg);

    // ===== PointCloud
    void subscribe(std::string topic, PointCloud_Func callback);
    PointCloud_Func point_cloud_cb_ {NULL};
    void pointCloud_CB(const am_utils::Latency_PointCloud::ConstPtr& msg);

    // ===== PointCloud2
    void subscribe(std::string topic, PointCloud2_Func callback);
    PointCloud2_Func point_cloud2_cb_ {NULL};
    void pointCloud2_CB(const am_utils::Latency_PointCloud2::ConstPtr& msg);


    // General templates

    // This template should be called with the Latency_XXX type
    template <typename LT>
    void advertise(std::string topic, uint32_t queue_size) {
        pub_topic_ = topic + "_latency";
        ROS_INFO("LATENCY: Advertising to topic[%s]", pub_topic_.c_str());
        pub_ = nh_->advertise<LT>(pub_topic_, queue_size);
    }

    // This template should be called with the Latency_XXX and the XXX type
    template <typename LT, typename T>
    void publish(const T& message) {
        LT m;
        ros::Time stamp;
        getLastMessageStamp(stamp);
        updateSendStats(stamp);
        m.header = header_;
        m.msg = message;
        pub_.publish(m);
    }

    template <typename LTP, typename LT, typename T>
    void publishPtr(const T& message) {
        LTP m(new LT);
        ros::Time stamp;
        getLastMessageStamp(stamp);
        updateSendStats(stamp);
        m->header = header_;
        m->msg = message;
        pub_.publish(m);
    }

    ros::Publisher& publisher() {
        return pub_;
    }

    void resetStats() {
        receive_stats_.reset();
        send_stats_.reset();
    }

    void printStats(std::string prefix);
    void printCurrentStats(std::string prefix);

    am_utils::LatencyHeader& getHeader() {
        return header_;
    }

    double getMaxEpochTime() const {
        return max_epoch_time_;
    }

    void setMaxEpochTime(double max_epoch_time) {
        max_epoch_time_ = max_epoch_time;
    }

    void setLastMessageStamp(const ros::Time& t) {
        last_msg_stamp_ = t;
        last_message_stamp_init_ = true;
    }

    void copyStats(const am_utils::LatencyHeader& header, const ros::Time& stamp);

private:
    unsigned char node_id_;

    // This is the current stats with respect to the timing of the messages.
    // Each latency wrapper tracks both the differences in time from the header time stamp of the original data.
    // This delay is tracked for both the received messages and the send messages and added to a vector of such delays
    //   each time the message is transmitted.
    // The rolling average is reset every so often as specified by the reset interval of the wrapper. This
    //   defaults to 1 sec.
    am::Statistics receive_stats_;
    ros::Time last_recv_epoch_;

    am::Statistics send_stats_;
    ros::Time last_send_epoch_;

    double max_epoch_time_ {1.0};

    // This stores the most recently received set of stats and is updated each time a new message is received.
    am_utils::LatencyHeader header_;
    ros::Time last_msg_stamp_;
    bool last_message_stamp_init_ {false};
    void getLastMessageStamp(ros::Time& t) {
        t = last_message_stamp_init_ ? last_msg_stamp_ : ros::Time::now();
    }

    void updateRecvStats(const am_utils::LatencyHeader& header, const ros::Time& stamp);
    void updateSendStats(const ros::Time& stamp);
    void updateStats(am::Statistics& stats, ros::Time& last_epoch_stamp, const ros::Time& stamp);

    ros::NodeHandle* nh_;

    std::string sub_topic_ {""};
    std::string pub_topic_ {""};

    ros::Publisher pub_;
    ros::Subscriber sub_;
};

#if 0
// This is a simple test class for the latency stuff.
// Note that the subscribe and the advertise are the same type,
//   but this is not a requirement.
// Also note that the subscribe and advertise topics are the same, but
//   in practice this would never the the case.
class LatencyWrapperTest {
public:
    LatencyWrapperTest(ros::NodeHandle *n) {
        // Initialize the wrapper
        lw_.setNodeHandle(n);
        lw_.setNodeId(am_utils::Latency::LOCATOR);

        // Subsribe to the desired topic. Note the callback defined below
        lw_.subscribe("/test_pose", BIND_CB(LatencyWrapperTest::poseStamped_CB));

        // Advertise to the desired topic with the specified data type
        lw_.advertise<am_utils::Latency_PoseStamped>("/test_pose");
        ps.header.stamp = ros::Time::now();
        timer = n->createTimer(ros::Duration(1.0), &LocatorTest::timerCB, this);
        timer.start();
    }

    ~LatencyWrapperTest() {
    }

private:
    // Instance of the Latency wrapper
    LatencyWrapper lw_;

    // Sample message that is being profiled
    geometry_msgs::PoseStamped ps;

    // Simple timer for sending the test message
    ros::Timer timer;
    void timerCB(const ros::TimerEvent& te) {
        ROS_INFO("LPT: Timer sending poseStamped");
        ps.pose.position.x = 1.1;
        ps.pose.position.y = 2.2;
        ps.pose.position.z = 3.3;
        // Publish the deisred message
        lw_.publish<am_utils::Latency_PoseStamped, geometry_msgs::PoseStamped>(ps);
    }

    // Callback for receiving the message
    // Note that the calback must use a reference to the message and not a const pointer.
    void poseStamped_CB(const geometry_msgs::PoseStamped& msg) {
        // Simply print the stats
        lw_.printStats("STATS");
    }
};
#endif

} /* namespace am */
#endif /* AM_UTILS_INCLUDE_AM_UTILS_LATENCY_WRAPPER_H_ */
