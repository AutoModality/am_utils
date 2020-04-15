#include <ros/ros.h>
#include <rosbag/bag.h>

#include <vb_util_lib/super_status.h>
#include <brain_box_msgs/Heartbeat.h>


// BagLogger *BagLogger::s_instance_ = 0;

std::string SUPER_STATUS = "ALIVE";
std::string SUPER_VALUE = "";
bool init_hb__ = false;
int last_beat__;
ros::Time last_hb__;
int hb_cnt__;

ros::Timer status_timer__;
bool init_node = false;
bool publish_status = false;

SuperStatus *SuperStatus::ss_instance_ = 0;


//#define EXTERN_FUNC

#ifdef EXTERN_FUNC
void logHeartBeat() ;
void statusTimerCB_(const ros::TimerEvent&);
void setupHeartbeatTimer(ros::NodeHandle& n);
void initNode(ros::NodeHandle& n);

#else

void logHeartBeat() {
    if (!init_hb__)
    {
        init_hb__ = true;
        last_hb__ = ros::Time::now();
        last_beat__ = -1;
        return;
    }

    brain_box_msgs::Heartbeat hb;
    hb.header.stamp = ros::Time::now();
    last_beat__ = last_beat__ > 0 ? last_beat__ = -1 : last_beat__ = 1;
    hb.beat = last_beat__;
    ros::Duration d = ros::Time::now() - last_hb__;
    hb.age = d.toSec();
    last_hb__ = ros::Time::now();
    // LOG_MSG("/Heartbeat", hb, 1); // TODO (dan) bag_logger

    return;
}

void statusTimerCB_(const ros::TimerEvent&)
{
    logHeartBeat();
    if (++hb_cnt__ >= 5)
    {
        SuperStatus::instance()->sendStatus(SUPER_STATUS, SUPER_VALUE);
        hb_cnt__ = 0;
    }
}

void setupHeartbeatTimer(ros::NodeHandle& n)
{
    float duration = 0.2;
    status_timer__ = n.createTimer(ros::Duration(duration),statusTimerCB_);
    status_timer__.start();
}

void initNode(ros::NodeHandle& n)
{
    if (init_node)
    {
        return;
    }

    init_node = true;
    n.param("publish_status", publish_status, true);

    /*
            ----------------------------
            Init the SuperStatus Sender
            ----------------------------
     */
    if (publish_status)
    {
        ROS_INFO("%s: INITIALIZING STATUS BROADCAST", ros::this_node::getName().c_str());
        SuperStatus::init(n);
        setupHeartbeatTimer(n);
    }
    else
    {
        ROS_WARN("%s: DISABLING STATUS BROADCAST", ros::this_node::getName().c_str());
    }

    return;
}

#endif
