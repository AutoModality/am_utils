#include <rclcpp/rclcpp.hpp>

#include <vb_util_lib/super_status.h>
#include <brain_box_msgs/msg/heartbeat.hpp>
#include <chrono>

// BagLogger *BagLogger::s_instance_ = 0;

std::string SUPER_STATUS = "ALIVE";
std::string SUPER_VALUE = "";
bool init_hb__ = false;
int last_beat__;
rclcpp::Time last_hb__;
int hb_cnt__;

rclcpp::TimerBase::SharedPtr status_timer__;
bool init_node = false;
bool publish_status = false;

SuperStatus *SuperStatus::ss_instance_ = 0;


//#define EXTERN_FUNC

#ifdef EXTERN_FUNC
void logHeartBeat() ;
void statusTimerCB_();
void setupHeartbeatTimer(rclcpp::Node::SharedPtr& node);
void initNode(rclcpp::Node::SharedPtr& node);

#else

void logHeartBeat() {
    if (!init_hb__)
    {
        init_hb__ = true;
        last_hb__ = rclcpp::Clock().now();
        last_beat__ = -1;
        return;
    }

    brain_box_msgs::msg::Heartbeat hb;
    hb.header.stamp = rclcpp::Clock().now();

    last_beat__ = last_beat__ > 0 ? last_beat__ = -1 : last_beat__ = 1;
    hb.beat = last_beat__;
    rclcpp::Duration d = rclcpp::Clock().now() - last_hb__;
    hb.age = d.seconds();
    last_hb__ = rclcpp::Clock().now();
    // LOG_MSG("/Heartbeat", hb, 1); // TODO (dan) bag_logger

    return;
}

void statusTimerCB_()
{
    logHeartBeat();
    if (++hb_cnt__ >= 5)
    {
        //SuperStatus::instance()->sendStatus(SUPER_STATUS, SUPER_VALUE);
        auto ss = SuperStatus::instance();
        ss->sendStatus(ss->getNode(), SUPER_STATUS, SUPER_VALUE);
        hb_cnt__ = 0;
    }
}

void setupHeartbeatTimer(rclcpp::Node::SharedPtr node)
{
    //float duration = 0.2;
    int duration = 5; //1/0.2
    status_timer__ = node->create_wall_timer(std::chrono::milliseconds(duration), statusTimerCB_);
    //status_timer__->start();
}

void initNode(rclcpp::Node::SharedPtr& node)
{
    if (init_node)
    {
        return;
    }

    init_node = true;
    node->get_parameter_or("publish_status", publish_status, true);

    /*
            ----------------------------
            Init the SuperStatus Sender
            ----------------------------
     */
    if (publish_status)
    {
        RCLCPP_INFO(node->get_logger(), "%s: INITIALIZING STATUS BROADCAST", node->get_name());
        SuperStatus::init(node);
        setupHeartbeatTimer(node);
    }
    else
    {
        RCLCPP_WARN(node->get_logger(), "%s: DISABLING STATUS BROADCAST", node->get_name());
    }

    return;
}

#endif
