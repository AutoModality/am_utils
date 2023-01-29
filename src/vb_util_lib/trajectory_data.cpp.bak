#include <vb_util_lib/trajectory_data.h>
#include <nav_msgs/msg/path.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <am_utils/am_ros2_utility.h>


void TrajectoryData::setup(std::string &bag_file_location, std::string &topic, TRAJECTORY_MSG_TYPE msg_type,
		bool trigger_enable, rclcpp::Time& trigger_time)
{
    GeometryPoseExtractor *gpe;
    BagReader<geometry_msgs::msg::PoseStamped> *br1;
    BagReader<geometry_msgs::msg::PoseWithCovarianceStamped> *br2;
    BagReader<nav_msgs::msg::Odometry> *br3;

    switch (msg_type)
    {
        case TRAJECTORY_MSG_TYPE::GM_POSESTAMPED:
            br1 = new BagReader<geometry_msgs::msg::PoseStamped>(bag_file_location, topic);
            gpe = new GeometryPoseExtractor(br1->getMessages());
            break;
        case TRAJECTORY_MSG_TYPE::GM_POSEWITHCOVERIANCESTAMPED:
            br2 = new BagReader<geometry_msgs::msg::PoseWithCovarianceStamped>(bag_file_location, topic);
            gpe = new GeometryPoseExtractor(br2->getMessages());
            break;
        case TRAJECTORY_MSG_TYPE::NM_ODOMETRY:
            br3 = new BagReader<nav_msgs::msg::Odometry>(bag_file_location, topic);
            gpe = new GeometryPoseExtractor(br3->getMessages());
            break;
    }

    if (trigger_enable)
    {
    	int j = 0;
    	std::vector<geometry_msgs::msg::PoseStamped> pose_list = gpe->getPoses();
    	for (std::vector<geometry_msgs::msg::PoseStamped>::iterator i = pose_list.begin(); i != pose_list.end(); i++, j++)
    	{
    		if (rclcpp::Time(i->header.stamp) > trigger_time)
    		{
    			pose_msgs_.push_back(*i);
    		}
    	}
    	ROS_INFO("TrajectoryData::setup: loaded %d triggered messages", j);
    }
    else
    {
        pose_msgs_ = gpe->getPoses();
    }
}

bool TrajectoryData::getPose(geometry_msgs::msg::PoseStamped& pt, std::vector<geometry_msgs::msg::PoseStamped>& traj,
                             PointStat& pose, double time_span) {
    double t_sec = pt.header.stamp.sec;
    double closest_dist = 99999999999.0;
    bool found_one = false;
    Eigen::Vector3d p_search, p_traj;
    Eigen::Vector3d p1, p2;

    Point2Eigen(pt.pose.position, p_search);

    if (time_span == 0.0)
    {
        time_span = 9999999.0;
    }

    // Check to make sure the time specified falls within the time span of the specified trajectory
    if (traj.size() <= 1 ||
            traj[0].header.stamp.sec > t_sec ||
            traj[traj.size()-1].header.stamp.sec < t_sec)
    {
    	//ROS_INFO("closest distance");
    	//ROS_INFO_STREAM("traj.size: " << traj.size());
    	//ROS_INFO("traj[0].header.stamp.toSec(): %f", traj[0].header.stamp.toSec());
    	//ROS_INFO("traj[traj.size()-1].header.stamp.toSec(): %f", traj[traj.size()-1].header.stamp.toSec());
        ROS_WARN("TrajectoryData::getPose1 - specified time not within compare trajectory time span %0.2f   %0.2f   %0.2f",
        		traj[0].header.stamp.sec, t_sec, traj[traj.size()-1].header.stamp.sec);
    	//ROS_INFO("t_sec: %f", t_sec);
        return false;
    }

    for(int i = 1; i < traj.size(); i++)
    {
//printf("    checking #%d time[%0.4f]\n", i, traj[i].header.stamp.toSec());
        double time_diff = traj[i].header.stamp.sec - t_sec;
        if (time_diff > time_span)
        {
            // Exceeded the upper limit of the time span, stop searching
            break;
        }

        if(time_diff < -time_span)
        {
            // Below the lower time span slimit so ignore point
            continue;
        }

        // Get the point on the line segment
        Point2Eigen(traj[i-1].pose.position, p1);
        Point2Eigen(traj[i].pose.position, p2);
        AM::pointOnLineSegment(p_traj, p1, p2, p_search);
        Eigen::Vector3d delta = p_search - p_traj;
        double dist = delta.norm();
        if (dist < closest_dist)
        {
            found_one = true;
            closest_dist = dist;
            pose.location = p_traj;
            pose.dist_err = dist;
            // Interpolate times
            Eigen::Vector3d seg, sub_seg;
            seg = p2 - p1;
            double seg_len = seg.norm();
            sub_seg = p_traj - p1;
            double sub_seg_len = sub_seg.norm();
            double ratio = sub_seg_len / seg_len;
            double time_diff = traj[i].header.stamp.sec - traj[i-1].header.stamp.sec;
            pose.stamp = traj[i-1].header.stamp.sec + (ratio * time_diff);
            pose.time_err = t_sec - pose.stamp;
        }
    }

    return found_one;
}

bool TrajectoryData::getPose(rclcpp::Time &t, std::vector<geometry_msgs::msg::PoseStamped>& traj,
                                    geometry_msgs::msg::PoseStamped& pose) {
    double t_sec = t.seconds();
//printf("getPose: Looking for t_sec[%0.3f] \n", t_sec);

    // Check to make sure the time specified falls within the time span of the specified trajectory
    if (traj.size() <= 1 ||
            traj[0].header.stamp.sec > t_sec ||
            traj[traj.size()-1].header.stamp.sec < t_sec)
    {
    	//ROS_INFO("closest time");
    	//ROS_INFO_STREAM("traj.size: " << traj.size());
    	//ROS_INFO("traj[0].header.stamp.toSec(): %f", traj[0].header.stamp.toSec());
    	//ROS_INFO("traj[traj.size()-1].header.stamp.toSec(): %f", traj[traj.size()-1].header.stamp.toSec());
    	ROS_WARN("TrajectoryData::getPose2 - specified time not within compare trajectory time span %0.2f   %0.2f   %0.2d", traj[0].header.stamp.sec, t_sec, traj[traj.size()-1].header.stamp.sec);
    	//ROS_INFO("t_sec: %f", t_sec);
        return false;
    }

    for(int i = 0; i < traj.size(); i++)
    {
//printf("    checking #%d time[%0.4f]\n", i, traj[i].header.stamp.toSec());
        double time_diff = t_sec - traj[i].header.stamp.sec;
        if(time_diff < 0.0)
        {
//            printf("        BINGO interpolating between %d and %d \n", i-1, i);
//            if (i == 0) {printf("SOMEHTING SERIOUSLY WRONG WIHT TIMES\n"); return false;}

            pose = interpolatePose(t, traj[i-1], traj[i]);
            return true;
        }
    }

    pose = traj[traj.size() - 1];
    ROS_WARN("TrajectoryData::getPose - off the end");

    return true;
}

geometry_msgs::msg::PoseStamped TrajectoryData::interpolatePose(rclcpp::Time &t,
                       geometry_msgs::msg::PoseStamped& p1, geometry_msgs::msg::PoseStamped& p2) {
    geometry_msgs::msg::PoseStamped pt;

    double t_diff = p2.header.stamp.sec - p1.header.stamp.sec;
    double t_inc = t.seconds() - p1.header.stamp.sec;

    if (t_diff <= 0.0 || t_inc < 0.0)
    {
//printf("          Interpolate: simply return p1 t_diff[%0.4f]  t_inc[%0.4f]", t_diff, t_inc);
        return p1;
    }

    double t_ratio = t_inc / t_diff;

    pt.header = p1.header;

    // TODO need to slerp the orientation
    pt.pose.orientation = p1.pose.orientation;

    double p_diff = p2.pose.position.x - p1.pose.position.x;
    pt.pose.position.x = p1.pose.position.x + (p_diff * t_ratio);
    p_diff = p2.pose.position.y - p1.pose.position.y;
    pt.pose.position.y = p1.pose.position.y + (p_diff * t_ratio);
    p_diff = p2.pose.position.z - p1.pose.position.z;
    pt.pose.position.z = p1.pose.position.z + (p_diff * t_ratio);
//printf("          Interpolate: t_diff[%0.4f]  t_inc[%0.4f] ratio[%0.4f] XYZ[%0.4f:%0.4f:%0.4f, %0.4f:%0.4f:%0.4f, %0.4f:%0.4f:%0.4f]\n",
//       t_diff, t_inc, t_ratio,
//       p1.pose.position.x, pt.pose.position.x, p2.pose.position.x,
//       p1.pose.position.y, pt.pose.position.y, p2.pose.position.y,
//       p1.pose.position.z, pt.pose.position.z, p2.pose.position.z);

    return pt;
}

double TrajectoryData::getTimeSpan() {
    if (pose_msgs_.size() < 2)
    {
        return 0.0;
    }

    return (rclcpp::Time(pose_msgs_[pose_msgs_.size()-1].header.stamp) - rclcpp::Time(pose_msgs_[0].header.stamp)).seconds();
}

void TrajectoryData::publishRvizPath(rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pub, bool trim_time, rclcpp::Time& start_time, rclcpp::Time& end_time) {
    if (pose_msgs_.size() < 2)
    {
        return;
    }
    nav_msgs::msg::Path path;
//    ros::Duration time_offset = (ros::Time::now() - pose_msgs_[pose_msgs_.size()-1].header.stamp) + ros::Duration(time_offset_);
    rclcpp::Duration time_offset = (am::clockNow() - pose_msgs_[0].header.stamp) + rclcpp::Duration(am::toDuration(time_offset_));

    for(int i = 0; i < pose_msgs_.size(); i++)
    {
        geometry_msgs::msg::PoseStamped p = pose_msgs_[i];
        p.pose.position.x += coord_offset_[0];
        p.pose.position.y += coord_offset_[1];
        p.pose.position.z += coord_offset_[2];

        p.header.stamp = rclcpp::Time(p.header.stamp) + time_offset;
        rclcpp::Time start = start_time + time_offset;
        rclcpp::Time end = end_time + time_offset;
        if (trim_time)
        {
        	if (start.seconds() < p.header.stamp.sec && p.header.stamp.sec < end.seconds())
        	{
            	path.poses.push_back(p);
        	}
        	else
        	{
        		ROS_INFO("TRIMMED POINT times[%0.3f  %0.3d  %0.3f]", start.seconds(), p.header.stamp.sec, end.seconds());
        	}
        }
        else
        {
        	path.poses.push_back(p);
        }
    }

    path.header.stamp = am::clockNow()();
//    path.header.frame_id = Asset_Frame;
    path.header.frame_id = pose_msgs_[0].header.frame_id;
    pub->publish(path);
}

void TrajectoryData::compareTrajectory(TrajectoryData& truth,
                                       Eigen::Vector3d& coord_offset, double time_offset) {

    std::vector<geometry_msgs::msg::PoseStamped> truth_vector = truth.getPoses();

    // first confirm that there there are elements in the trajectory
    if (truth_vector.size() < 2 || pose_msgs_.size() < 2)
    {
        ROS_ERROR("TrajectoryData::compareTrajectory: Not enough points in trajectories");
        return;
    }
    // get the transform from the truth trajectory to this trajectory
    if (!transformer_.getTransform(pose_msgs_[0].header.frame_id,
                                   truth_vector[0].header.frame_id,
                                   compare_transform_))
    {
        ROS_ERROR_STREAM("TrajectoryData::compareTrajectory: No transform from " << truth_vector[0].header.frame_id << " to " << pose_msgs_[0].header.frame_id);
        return;
    }

    rclcpp::Duration d = rclcpp::Duration(am::toDuration(time_offset+time_offset_));
    geometry_msgs::msg::PoseStamped pt;
    for (std::vector<geometry_msgs::msg::PoseStamped>::iterator it = truth_vector.begin();
            it != truth_vector.end(); it++)
    {
        // transform point
        tf2::doTransform(*it, pt, compare_transform_);

        // calculateStatsAdd time offset
        pt.header.stamp = rclcpp::Time(it->header.stamp) + d;

        // Add location offset
        pt.pose.position.x += (coord_offset[0] + coord_offset_[0]);
        pt.pose.position.y += (coord_offset[1] + coord_offset_[1]);
        pt.pose.position.z += (coord_offset[2] + coord_offset_[2]);

        //truth_vector.push_back(pt);
        compare_trajectory_.push_back(pt);
    }

    calculateStats(pose_msgs_, compare_trajectory_, trajectory_stats_, stats_);
}

void TrajectoryData::calculateStats(std::vector<geometry_msgs::msg::PoseStamped>& traj,
                  std::vector<geometry_msgs::msg::PoseStamped>& truth,
                  std::vector<TrajectoryPointStats>& stats_vector, TrajectoryStats& stat_data) {

    stat_data.closest_dist_RMS = 0.0;
    stat_data.closest_time_RMS = 0.0;
    stat_data.avg_time_error = 0.0;
    stat_data.closest_dist_max = 0.0;
    stat_data.closest_time_max = 0.0;
    stat_data.max_time_error = 0.0;
    stat_data.min_time_error = 9999999.0;

    if (traj.size() == 0)
    {
        ROS_WARN("TrajectoryData::calculateStats: trajectory empty");
        return;
    }

    geometry_msgs::msg::PoseStamped pt;
    TrajectoryPointStats stat;
    Eigen::Vector3d delta_dist;
    //printTrajectory("", "XXXX TRAJECTORY SEARCH POINTS", traj);
    //ros::Duration(30).sleep();
    //printTrajectory("", "XXXX TRAJECTORY TRUTH POINTS", truth);
    //ros::Duration(60).sleep();
    double total_num = 0;

    for (std::vector<geometry_msgs::msg::PoseStamped>::iterator it = traj.begin();
            it != traj.end(); it++)
    {
        // initialize the location
        stat.valid_point = false;
        Point2Eigen(it->pose.position, stat.location.location);
        stat.location.stamp = it->header.stamp.sec;

        // Find the point in the truth vector closest in time
        rclcpp::Time t = it->header.stamp;
        if (getPose(t, truth, pt))
        {
            // Fill in all the details concernding the point found on the trajecgtory
            stat.closest_time.stamp = stat.location.stamp;
            Point2Eigen(pt.pose.position, stat.closest_time.location);
            delta_dist = stat.closest_time.location - stat.location.location;
            stat.closest_time.dist_err = delta_dist.norm();
            stat.closest_time.time_err = 0.0;
        }
        else
        {
            // ignore this in the stats
            stats_vector.push_back(stat);
            continue;
        }

        // Find the point in the truth vector  closest in distance
        if (!getPose(*it, truth, stat.closest_distance, 0.5))
        {
            stats_vector.push_back(stat);
            continue;
        }

        // Push the stat
        stat.valid_point = true;
        stats_vector.push_back(stat);
        total_num += 1.0;

        if (stat.closest_distance.time_err > stat_data.max_time_error) stat_data.max_time_error = stat.closest_distance.time_err;
        if (stat.closest_distance.time_err < stat_data.min_time_error) stat_data.min_time_error = stat.closest_distance.time_err;
        if (stat.closest_distance.dist_err > stat_data.closest_dist_max) stat_data.closest_dist_max = stat.closest_distance.dist_err;
        if (stat.closest_time.dist_err > stat_data.closest_time_max) stat_data.closest_time_max = stat.closest_time.dist_err;

        stat_data.avg_time_error += stat.closest_distance.time_err;
        stat_data.closest_dist_RMS += stat.closest_distance.dist_err;
        stat_data.closest_time_RMS += stat.closest_time.dist_err;
//        printTrajectoryPointStats("STATS POINT: ", stat);
//        printPointStat("CLOSEST_DISTANCE: ", stat.closest_distance);
//        printPointStat("CLOSEST_TIME: ", stat.closest_time);
    }

    stat_data.avg_time_error /= total_num;
    stat_data.closest_dist_RMS /= total_num;
    stat_data.closest_time_RMS /= total_num;

    return;
}

void TrajectoryData::addPose(const nav_msgs::msg::Odometry& pose) {
    geometry_msgs::msg::PoseStamped p;
    p.header = pose.header;
    p.pose = pose.pose.pose;
    pose_msgs_.push_back(p);
}

void TrajectoryData::addComparePose(const nav_msgs::msg::Odometry& pose) {
    geometry_msgs::msg::PoseStamped p;
    p.header = pose.header;
    p.pose = pose.pose.pose;
    compare_trajectory_.push_back(p);
}


void TrajectoryData::printPointStamp(std::string prefix, AM::PointStamp& p) {
    printf("%s sec[%0.3f]  XYZ[%0.3f,  %0.3f,  %0.3f]\n", prefix.c_str(), p.stamp,
           p.location[0], p.location[1], p.location[2]);
}

void TrajectoryData::printPointStat(std::string prefix, PointStat& p) {
    printf("%s sec[%0.3f]  XYZ[%0.3f,  %0.3f,  %0.3f] dist_err[%0.3f] time_err[%0.3f]\n", prefix.c_str(), p.stamp,
           p.location[0], p.location[1], p.location[2],
           p.dist_err, p.time_err);
}

void TrajectoryData::printPoint(std::string prefix, geometry_msgs::msg::PoseStamped& p) {
    printf("%s sec[%d]  XYZ[%0.3f,  %0.3f,  %0.3f]\n", prefix.c_str(), p.header.stamp.sec,
           p.pose.position.x, p.pose.position.y, p.pose.position.z);
}

void TrajectoryData::printTrajectoryPointStats(std::string prefix, TrajectoryPointStats& s) {
    printf("%s valid[%s] ", prefix.c_str(), s.valid_point ? "TRUE" : "FALSE");
    printPointStamp("Location: ", s.location);
    printPointStat("    closest_time: ", s.closest_time);
    printPointStat("    closest_distance: ", s.closest_distance);
//    printf("\n");
}

void TrajectoryData::printAllTrajectoryPointStats(std::string prefix, std::vector<TrajectoryPointStats>& s) {
    for (std::vector<TrajectoryPointStats>::iterator it = s.begin(); it != s.end(); it++)
    {
        printTrajectoryPointStats("===> ", *it);
    }
}

void TrajectoryData::printStats(std::string prefix, TrajectoryStats& stats) {
    printf("%s  TRAJECTORY STATS: \nclosest_time_rms[%0.4f : %0.3f] \nclosest_dist_rms[%0.4f : %0.3f] \navg_time_error[%0.4f : (%0.3f : %0.3f)]\n", prefix.c_str(),
           stats.closest_time_RMS, stats.closest_time_max, stats.closest_dist_RMS, stats.closest_dist_max,
           stats.avg_time_error, stats.min_time_error, stats.max_time_error);
    printf("%s  TRAJECTORY STATS:  %0.4f  %0.3f  %0.4f  %0.3f  %0.4f  %0.3f  %0.3f\n", prefix.c_str(),
           stats.closest_time_RMS, stats.closest_time_max, stats.closest_dist_RMS, stats.closest_dist_max,
           stats.avg_time_error, stats.min_time_error, stats.max_time_error);
}

void TrajectoryData::printTrajectory(std::string prefix, std::string title,
                                     std::vector<geometry_msgs::msg::PoseStamped>& traj_msgs) {
    printf("%s %s:\n", prefix.c_str(), title.c_str());
    for (std::vector<geometry_msgs::msg::PoseStamped>::iterator it = traj_msgs.begin();
            it != traj_msgs.end(); it++)
    {
        std::string pre = prefix + "  ";
        printPoint(pre, *it);
    }
}

void TrajectoryData::test(double x_off, double y_off, double z_off, double t_off) {
    printf("\n\n+++++ TRAJECTORY STATS TEST offset[%0.2f, %0.2f, %0.2f] time_off[%0.2f]\n", x_off, y_off, z_off, t_off);

    geometry_msgs::msg::PoseStamped p;

    double t = 0.0;
    for (int i = 0; i < 10; i++)
    {
        p.header.stamp = rclcpp::Time(t);
        p.pose.position.x = i;
        p.pose.position.y = i;
        p.pose.position.z = 0.0;
        addComparePose(p);
        t += 1.0;
    }
    printTrajectory("", "TRUTH", compare_trajectory_);

    t = t_off;
    double X = x_off;
    double Y = y_off;
    double Z = z_off;
    for (int i = 0; i < 13; i++)
    {
        p.header.stamp = rclcpp::Time(t);
        p.pose.position.x = X;
        p.pose.position.y = Y;
        p.pose.position.z = 0.0;
        addPose(p);
        t += 1.0;
        X += 1.0;
        Y += 1.0;
        Z += 1.0;
    }
    printTrajectory("", "TRAJECTORY", pose_msgs_);

    printf("\n");

    printf("===> GET POSE TEST\n");
    for (std::vector<geometry_msgs::msg::PoseStamped>::iterator it = pose_msgs_.begin();
            it != pose_msgs_.end(); it++)
    {
    	rclcpp::Time t = it->header.stamp;
        if (!getPose(t, compare_trajectory_, p))
        {
            printf("    ERROR - could not find point in compare trajectory at time[%d]  XYZ[%0.2f,  %0.2f, %0.2f]\n",
                   it->header.stamp.sec, it->pose.position.x, it->pose.position.y, it->pose.position.z);
            continue;
        }
        else
        {
            printf("    CLOSEST TIME POINT:    time[%d] IN[%0.2f, %0.2f, %0.2f]  RESULT[%0.3f, %0.3f, %0.3f]\n",
                   it->header.stamp.sec,
                   it->pose.position.x, it->pose.position.y, it->pose.position.z,
                   p.pose.position.x, p.pose.position.y, p.pose.position.z);
        }

//        bool TrajectoryData::getPose(geometry_msgs::PoseStamped& pt, std::vector<geometry_msgs::PoseStamped>& traj,
//                                     PointStat& pose, double time_span) {
        PointStat pstat;
        if (!getPose(*it, compare_trajectory_, pstat, 1.5))
        {
            printf("    ERROR - could not find point in compare trajectory at time\n");
            continue;
        }
        else
        {
            printf("    CLOSEST DIST POINT: in_time[%d] IN[%0.2f, %0.2f, %0.2f]  RESULT[%0.3f, %0.3f, %0.3f] time[%0.2f] time_error[%0.3f] dist_error[%0.3f]\n",
                   it->header.stamp.sec,
                   it->pose.position.x, it->pose.position.y, it->pose.position.z,
                   pstat.location[0], pstat.location[1], pstat.location[2],
                   pstat.stamp, pstat.time_err, pstat.dist_err);
        }

    }

    calculateStats();

    printStats("\nFINAL TEST STATS");
    printf("\n");

    pose_msgs_.clear();
    compare_trajectory_.clear();

    return;
}

