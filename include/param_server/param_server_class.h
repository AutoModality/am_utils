#ifndef AM_UTILS_INCLUDE_PARAM_SERVER_PARAM_SERVER_CLASS_H_
#define AM_UTILS_INCLUDE_PARAM_SERVER_PARAM_SERVER_CLASS_H_


#include <am_utils/am_ros2_utility.h>
#include <param_server/node_param_handler_class.h>
#include <brain_box_msgs/msg/parameter_array.hpp>
#include <brain_box_msgs/msg/key_value.hpp>
#include <brain_box_msgs/msg/log_control.hpp>
#include <vb_util_lib/bag_logger.h>
#include <vb_util_lib/vb_main.h>
#include <yaml-cpp/yaml.h>
#include <vb_util_lib/key_value_stamped.h>

namespace am
{

class ParameterServer
{
public:
    ParameterServer();

    ~ParameterServer();
private:
    std::unordered_map<std::string, std::shared_ptr<am::NodeParamHandler>> nodes_;
    std::unordered_map<std::string, rclcpp::ParameterValue> global_parameters_;
    rclcpp::Subscription<brain_box_msgs::msg::ParameterArray>::SharedPtr param_sub_;
    rclcpp::Publisher<brain_box_msgs::msg::ParameterArray>::SharedPtr internal_pub_;
    rclcpp::Subscription<brain_box_msgs::msg::KeyValue>::SharedPtr fp_params_sub_;
    rclcpp::Subscription<brain_box_msgs::msg::LogControl>::SharedPtr log_control_sub_;
    rclcpp::TimerBase::SharedPtr updateTimer_;
    std::string log_file_name_;
    std::ofstream mFile;
    std::vector<std::string> black_list_ = {"transform_listener", "parameter_server", "ros2cli_daemon"};

    std::string global_yaml_file_location_ = "/home/ubuntu/ros2_ws/install/am_config/share/am_config/launch/vineyard/vineyard_mockup_amiga.yaml";
     
    const std::string param_server_outlet_topic_ = "/param_server/params";
    void getParams();

    bool extractNodeNameAndParamName(const std::string& input, std::string& nodeName, std::string& paramName);

    //flightplan parameters
    void fpParamCB(const brain_box_msgs::msg::KeyValue::SharedPtr param_msg);


    void printYamlNode(const YAML::Node& node, const std::string& parentKey = "");

    void logControlCB(const brain_box_msgs::msg::LogControl::SharedPtr msg);

    bool paramtoYaml(YAML::Node &yaml_data);

    std::string cleanString(const std::string &input);

    bool isArray(const rclcpp::ParameterValue &pv);

    void logParameters();

    void paramsCB(const brain_box_msgs::msg::ParameterArray::SharedPtr msg);

    void updateTimerCB();
    void updateNodesList();
    bool isNodeInBlackList(const std::string &node_name);
};    
}

#endif //AM_UTILS_INCLUDE_PARAM_SERVER_PARAM_SERVER_CLASS_H_