/*
 * latency_wrapper.cpp
 *
 *  Created on: Jun 24, 2018
 *      Author: ubuntu
 */

#include <am_utils/latency_wrapper.h>

namespace am
{
LatencyWrapper::LatencyWrapper(ros::NodeHandle* n, unsigned char id, double max_epoch_time)  :
    nh_(n), node_id_(id), max_epoch_time_(max_epoch_time) {
    resetStats();
}

LatencyWrapper::~LatencyWrapper() {
    // TODO Auto-generated destructor stub
}

void LatencyWrapper::updateRecvStats(const brain_box_msgs::LatencyHeader& header,
                                     const ros::Time& stamp) {
    // first copy string of existing stats
    header_ = header;
//    send_stats_updated_ = false;
    setLastMessageStamp(stamp);

    // Calculate the received time stats
    updateStats(receive_stats_, last_recv_epoch_, stamp);

    // Add them to the received header
    brain_box_msgs::Latency lt;
    receive_stats_.setLatency(lt);
    lt.send = false;
    lt.node_id = node_id_;
    header_.latency.push_back(lt);
}

void LatencyWrapper::printStats(std::string prefix) {
    printf("%s\n", prefix.c_str());
    int i = 0;
    for (std::vector<brain_box_msgs::Latency>::iterator it = header_.latency.begin();
            it != header_.latency.end(); it++)
    {
        printf("   %d: node_id[%u] send[%s] current[%0.4f] min[%0.4f]  max[%0.4f]  avg[%0.4f]\n", i++,
               (unsigned int)it->node_id, it->send ? "TRUE" : "FALSE",
                       it->cur_delay, it->min_delay, it->max_delay, it->avg_delay);
    }
}

void LatencyWrapper::printCurrentStats(std::string prefix) {
    printf("%s\n", prefix.c_str());
    send_stats_.print("    SEND ");
    receive_stats_.print("    RECEIVE ");
}

void LatencyWrapper::updateStats(am::Statistics& stats,
                                 ros::Time& last_epoch_stamp, const ros::Time& stamp)
{
    if (stats.cnt == 0)
    {
        last_epoch_stamp = ros::Time::now();
    }
    else
    {
        ros::Duration de = ros::Time::now() - last_epoch_stamp;
        if (de.toSec() > max_epoch_time_)
        {
            stats.cnt = 0;
            last_epoch_stamp = ros::Time::now();
        }
    }

    ros::Duration d = ros::Time::now() - stamp;
    stats.addData(d.toSec());
}

void LatencyWrapper::updateSendStats(const ros::Time& stamp) {
    // Calculate the stats
    updateStats(send_stats_, last_send_epoch_, stamp);

    // Determine if this is a new entry or simply updating and existing one.
    if (header_.latency.size() > 0 && header_.latency.back().send)
    {
        // Simply change the existing one
        send_stats_.setLatency(header_.latency.back());
    }
    else
    {
        // Add a new one to the header_
        brain_box_msgs::Latency lt;
        send_stats_.setLatency(lt);
        lt.send = true;
        lt.node_id = node_id_;
        header_.latency.push_back(lt);
//        send_stats_updated_ = true;
    }
}

//======= PoseStamped
void LatencyWrapper::subscribe(std::string topic, PoseStamped_Func callback) {
    sub_topic_ = topic + "_latency";
    ROS_INFO("LATENCY: SUBSCRIBING TO topic[%s]", sub_topic_.c_str());
    pose_stamped_cb_ = callback;
    sub_ = nh_->subscribe<brain_box_msgs::Latency_PoseStamped>(
            sub_topic_, 10, &LatencyWrapper::poseStamped_CB, this);
}
void LatencyWrapper::poseStamped_CB(
        const brain_box_msgs::Latency_PoseStamped::ConstPtr& msg) {
    // update the stats
    updateRecvStats(msg->header, msg->msg.header.stamp);

    // Call the callback
    pose_stamped_cb_(msg->msg);
}

//======= PoseWithCovarianceStamped
void LatencyWrapper::subscribe(std::string topic, PoseWithCovarianceStamped_Func callback) {
    sub_topic_ = topic + "_latency";
//    ROS_INFO("LATENCY: SUBSCRIBING TO topic[%s]", sub_topic_.c_str());
    pose_with_covariance_stamped_cb_ = callback;
    sub_ = nh_->subscribe<brain_box_msgs::Latency_PoseWithCovarianceStamped>(
            sub_topic_, 10, &LatencyWrapper::poseWithCovarianceStamped_CB, this);
}
void LatencyWrapper::poseWithCovarianceStamped_CB(
        const brain_box_msgs::Latency_PoseWithCovarianceStamped::ConstPtr& msg) {
    // update the stats
    updateRecvStats(msg->header, msg->msg.header.stamp);

    // Call the callback
    pose_with_covariance_stamped_cb_(msg->msg);
}

//======= TwistStamped
void LatencyWrapper::subscribe(std::string topic, TwistStamped_Func callback) {
    sub_topic_ = topic + "_latency";
    twist_stamped_cb_ = callback;
    sub_ = nh_->subscribe<brain_box_msgs::Latency_TwistStamped>(
            sub_topic_, 10, &LatencyWrapper::twistStamped_CB, this);
}
void LatencyWrapper::twistStamped_CB(
        const brain_box_msgs::Latency_TwistStamped::ConstPtr& msg) {
    // update the stats
    updateRecvStats(msg->header, msg->msg.header.stamp);

    // Call the callback
    twist_stamped_cb_(msg->msg);
}

//======= TwistWithCovarianceStamped
void LatencyWrapper::subscribe(std::string topic, TwistWithCovarianceStamped_Func callback) {
    sub_topic_ = topic + "_latency";
    twist_with_covariance_stamped_cb_ = callback;
    sub_ = nh_->subscribe<brain_box_msgs::Latency_TwistWithCovarianceStamped>(
            sub_topic_, 10, &LatencyWrapper::twistWithCovarianceStamped_CB, this);
}
void LatencyWrapper::twistWithCovarianceStamped_CB(
        const brain_box_msgs::Latency_TwistWithCovarianceStamped::ConstPtr& msg) {
    // update the stats
    updateRecvStats(msg->header, msg->msg.header.stamp);

    // Call the callback
    twist_with_covariance_stamped_cb_(msg->msg);
}

//======= Odometry
void LatencyWrapper::subscribe(std::string topic, Odometry_Func callback) {
    sub_topic_ = topic + "_latency";
    odometry_cb_ = callback;
    sub_ = nh_->subscribe<brain_box_msgs::Latency_Odometry>(
            sub_topic_, 10, &LatencyWrapper::odometry_CB, this);
}
void LatencyWrapper::odometry_CB(
        const brain_box_msgs::Latency_Odometry::ConstPtr& msg) {
    // update the stats
    updateRecvStats(msg->header, msg->msg.header.stamp);

    // Call the callback
    odometry_cb_(msg->msg);
}

//======= Image
void LatencyWrapper::subscribe(std::string topic, Image_Func callback) {
    sub_topic_ = topic + "_latency";
    image_cb_ = callback;
    sub_ = nh_->subscribe<brain_box_msgs::Latency_Image>(
            sub_topic_, 10, &LatencyWrapper::image_CB, this);
}
void LatencyWrapper::image_CB(
        const brain_box_msgs::Latency_Image::ConstPtr& msg) {
    // update the stats
    updateRecvStats(msg->header, msg->msg.header.stamp);

    // Call the callback
    image_cb_(msg->msg);
}

//======= LaserScan
void LatencyWrapper::subscribe(std::string topic, LaserScan_Func callback) {
    sub_topic_ = topic + "_latency";
    laser_scan_cb_ = callback;
    sub_ = nh_->subscribe<brain_box_msgs::Latency_LaserScan>(
            sub_topic_, 10, &LatencyWrapper::laserScan_CB, this);
}
void LatencyWrapper::laserScan_CB(
        const brain_box_msgs::Latency_LaserScan::ConstPtr& msg) {
    // update the stats
    updateRecvStats(msg->header, msg->msg.header.stamp);

    // Call the callback
    laser_scan_cb_(msg->msg);
}

//======= PointCloud
void LatencyWrapper::subscribe(std::string topic, PointCloud_Func callback) {
    sub_topic_ = topic + "_latency";
    point_cloud_cb_ = callback;
    sub_ = nh_->subscribe<brain_box_msgs::Latency_PointCloud>(
            sub_topic_, 10, &LatencyWrapper::pointCloud_CB, this);
}
void LatencyWrapper::pointCloud_CB(
        const brain_box_msgs::Latency_PointCloud::ConstPtr& msg) {
    // update the stats
    updateRecvStats(msg->header, msg->msg.header.stamp);

    // Call the callback
    point_cloud_cb_(msg->msg);
}

//======= PointCloud2
void LatencyWrapper::subscribe(std::string topic, PointCloud2_Func callback) {
    sub_topic_ = topic + "_latency";
    point_cloud2_cb_ = callback;
    sub_ = nh_->subscribe<brain_box_msgs::Latency_PointCloud2>(
            sub_topic_, 10, &LatencyWrapper::pointCloud2_CB, this);
}
void LatencyWrapper::pointCloud2_CB(
        const brain_box_msgs::Latency_PointCloud2::ConstPtr& msg) {
    // update the stats
    updateRecvStats(msg->header, msg->msg.header.stamp);

    // Call the callback
    point_cloud2_cb_(msg->msg);
}

} // namespace am

