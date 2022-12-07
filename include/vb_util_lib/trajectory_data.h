/*
 * TrajectoryData.h
 *
 *  Created on: Apr 16, 2018
 *      Author: ubuntu
 */

#ifndef VISBOX_PACKAGES_VISION_AM_VISION_LIB_INCLUDE_AM_VISION_LIB_TRAJECTORY_H_
#define VISBOX_PACKAGES_VISION_AM_VISION_LIB_INCLUDE_AM_VISION_LIB_TRAJECTORY_H_

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <builtin_interfaces/msg/time.hpp>
#include <string>
#include <vector>

#include <eigen3/Eigen/Geometry>

#include <vb_util_lib/BagReader.h>
#include <vb_util_lib/GeometryPoseExtractor.h>
#include <vb_util_lib/transformer.h>
#include <vb_util_lib/am_geometry.h>
#include <vb_util_lib/frame_names.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

enum class TRAJECTORY_MSG_TYPE
{
    GM_POSESTAMPED=0,
    GM_POSEWITHCOVERIANCESTAMPED,
    NM_ODOMETRY
};

// Used to store a variety of statistics about points in trajectories
struct PointStat : AM::PointStamp {
    double dist_err;
    double time_err;
};

struct TrajectoryPointStats {
    // set to false if this point is not within the time bounds of the truth trajectory
    bool valid_point;
    // The location of the point being compared to the truth trajectory
    AM::PointStamp location;
    // The point on the truth trajectory that is closest distance to the above location
    PointStat closest_distance;
    // The point on the truth trajectory that is closest in time to the above location
    PointStat closest_time;
};

struct TrajectoryStats {
    double closest_time_RMS;
    double closest_time_max;
    double closest_dist_RMS;
    double closest_dist_max;
    double avg_time_error;
    double max_time_error;
    double min_time_error;
};


class TrajectoryData
{
public:
    TrajectoryData(rclcpp::Node::SharedPtr node): node_(node), transformer_(node_){
    	coord_offset_[0] = 0.0;
    	coord_offset_[1] = 0.0;
    	coord_offset_[2] = 0.0;
    }
    TrajectoryData(rclcpp::Node::SharedPtr node, std::string &bag_file_location,
    		std::string &topic, TRAJECTORY_MSG_TYPE msg_type): node_(node), transformer_(node_)
    {
        setup(bag_file_location, topic, msg_type);
    }

    ~TrajectoryData() {
    }

    void setup(std::string &bag_file_location, std::string &topic, TRAJECTORY_MSG_TYPE msg_type) {
    	rclcpp::Time t = node_->now();
    	setup(bag_file_location, topic, msg_type, false, t);
    }

    void setup(std::string &bag_file_location, std::string &topic, TRAJECTORY_MSG_TYPE msg_type, bool trigger_enable, rclcpp::Time& trigger_time);

    void addPose(geometry_msgs::msg::PoseStamped& pose) {
        pose_msgs_.push_back(pose);
    }

    void addPose(const nav_msgs::msg::Odometry& pose);

    void addComparePose(geometry_msgs::msg::PoseStamped& pose) {
        compare_trajectory_.push_back(pose);
    }

    void addComparePose(const nav_msgs::msg::Odometry& pose);

    // Fetch a pose in this trajectory that has the same time stamp as specified by t
    bool getPose(rclcpp::Time &stamp, geometry_msgs::msg::PoseStamped& pose) {
        return getPose(node_, stamp, pose_msgs_, pose);
    }

    std::vector<geometry_msgs::msg::PoseStamped> &getPoses() {
        return pose_msgs_;
    }

    double getTimeSpan();

    // Calculate the statistics of this trajectory as compared to a truth trajectory
    void calculateStats() {
        printf("SSSSSSSSS\n");
        calculateStats(node_, pose_msgs_, compare_trajectory_, trajectory_stats_, stats_);
        printf("EEEEEEEE\n");
    }

    // This method will take a compare trajectory called the "truth" trajectory and calculate a variety of
    // statistics against it.
    // It assumes that the truth trajectory is a superset of the this trajectory.
    // The coord_offset coordinates will be added to each of the points in the truth trajectory.
    // Likewise the time_offset will be added to the time stamp of the truth trajectory.
    // Before any stats are calculated the truth trajectory will be transformed into the same frame
    //   as this trajectory.
    void compareTrajectory(TrajectoryData& truth, Eigen::Vector3d& coord_offset, double time_offset);

    // publish a path fo rthis trajectory that can be displyed in RVIZ
    void publishRvizPath(rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pub) {
    	rclcpp::Time tmp;
    	publishRvizPath(pub, false, tmp, tmp);
    }
    void publishRvizPath(rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pub, bool trim_time, rclcpp::Time& start, rclcpp::Time& end);

    void setOffsets(double x, double y, double z, double t) {
    	coord_offset_[0] = x;
    	coord_offset_[1] = y;
    	coord_offset_[2] = z;
    	time_offset_ = t;
    }

    void print(std::string prefix) {
        printTrajectory(prefix, "TRAJECTORY POINTS", pose_msgs_);
    }

    void printCompare(std::string prefix) {
        printTrajectory(prefix, "COMPARE TRAJECTORY POINTS", compare_trajectory_);
    }

    void printStats(std::string prefix) {
        printStats(prefix, stats_);
    }

    void getStartTime(rclcpp::Time& start) {
    	if (pose_msgs_.size() > 0)
    	{
    		start = pose_msgs_.front().header.stamp;
    	}
    }

    void getEndTime(rclcpp::Time& end) {
    	if (pose_msgs_.size() > 0)
    	{
    		end = pose_msgs_.back().header.stamp;
    	}
    }

    // General purpose method that can take any PoseStamped vector trajectory and return
    //   the pose on that trajectory that has the same time stamp as
    static bool getPose(rclcpp::Node::SharedPtr node, rclcpp::Time &stamp, std::vector<geometry_msgs::msg::PoseStamped>& trajectory,
                        geometry_msgs::msg::PoseStamped& pose);

    // General purpose method that can take any PoseStamped vector trajectory and return
    //   statistics about the point on that trajectory that is the closest.
    // If time_span != 0.0 then it restricts the search to only points that are within a certain time
    //   of pt.header.stamp
    static bool getPose(rclcpp::Node::SharedPtr node, geometry_msgs::msg::PoseStamped& pt, std::vector<geometry_msgs::msg::PoseStamped>& trajectory,
                        PointStat& pose, double time_span=0.0);

    // General purpose routine that will return the interpolated position between p1 and p2 based
    //   on the time stamp.
    static geometry_msgs::msg::PoseStamped interpolatePose(rclcpp::Time &stamp,
                           geometry_msgs::msg::PoseStamped& p1, geometry_msgs::msg::PoseStamped& p2);

    // Takes the trajectory vector and compares it to the truth trajectory to create a vector
    //   of statistics. For each element in the trajectory vector there will be an element in the
    //   stats vector.
    static void calculateStats(rclcpp::Node::SharedPtr node, std::vector<geometry_msgs::msg::PoseStamped>& trajectory,
                      std::vector<geometry_msgs::msg::PoseStamped>& truth,
                      std::vector<TrajectoryPointStats>& stats_vector, TrajectoryStats& stats);

    static void printStats(std::string prefix, TrajectoryStats& stats);

    void test(double x_off, double y_off, double z_off, double t_off);

private:

    rclcpp::Node::SharedPtr node_;

    am::Transformer transformer_;
    geometry_msgs::msg::TransformStamped compare_transform_;

    TrajectoryStats stats_;

    static void printPoint(std::string prefix, geometry_msgs::msg::PoseStamped& p);

    static void printPointStamp(std::string prefix, AM::PointStamp& p);

    static void printPointStat(std::string prefix, PointStat& p);

    static void printTrajectory(std::string prefix, std::string title,
                         std::vector<geometry_msgs::msg::PoseStamped>& traj_msgs);

    static void printTrajectoryPointStats(std::string prefix, TrajectoryPointStats& s);

    static void printAllTrajectoryPointStats(std::string prefix, std::vector<TrajectoryPointStats>& s);

    Eigen::Vector3d coord_offset_;
    double time_offset_ {0.0};

    std::vector<geometry_msgs::msg::PoseStamped> pose_msgs_;

    std::vector<geometry_msgs::msg::PoseStamped> compare_trajectory_;

    std::vector<TrajectoryPointStats> trajectory_stats_;
};

#endif /* VISBOX_PACKAGES_VISION_AM_VISION_LIB_INCLUDE_AM_VISION_LIB_TRAJECTORY_H_ */
