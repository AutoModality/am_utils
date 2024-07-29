#include <am_utils/am_ros2_utility.h>
#include <brain_box_msgs/msg/feature_status_list.hpp>


std::shared_ptr<am::AMLifeCycle> am::Node::node = nullptr;
rclcpp::Publisher<brain_box_msgs::msg::FeatureStatusList>::SharedPtr fsl_pub_ = nullptr;
std::vector<std::string> feature_names_;
double delay_ {1.0};
int number_of_publish_ {3};

void getParams();
void timerCB();


void getParams()
{
    am::getParam<std::vector<std::string>>("feature_names", feature_names_, feature_names_);
    am::getParam<double>("delay", delay_, delay_);
    am::getParam<int>("number_of_publish", number_of_publish_, number_of_publish_);

    ROS_INFO(GREEN "feature_names: [%s], delay: %f, number_of_publish: %d" COLOR_RESET, am::vectToStr(feature_names_).c_str(), delay_, number_of_publish_);
}

void timerCB()
{
    static int local_counter = 0;

    //exit once it is done
    if(local_counter > number_of_publish_)
    {
        rclcpp::shutdown();
        exit(0);
    }

    brain_box_msgs::msg::FeatureStatusList fsl;
    fsl.header.stamp = am::ClockNow();
    for(std::string &fn : feature_names_)
    {
        brain_box_msgs::msg::FeatureStatus fs;
        fs.feature_id = fn;
        fsl.features.push_back(fs);
    }
    
    fsl_pub_->publish(fsl);
    local_counter++;
}

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    
    am::Node::node = std::make_shared<am::AMLifeCycle>("feature_status_list_publisher");

    getParams();

    fsl_pub_ = am::Node::node->create_publisher<brain_box_msgs::msg::FeatureStatusList>(am::am_topics::FEATURE_SEARCH_IDS, 1);
    rclcpp::TimerBase::SharedPtr timer_ = am::Node::node->create_wall_timer(am::toDuration(delay_), timerCB);


    rclcpp::spin(am::Node::node);
    rclcpp::shutdown();
    return 0;
}