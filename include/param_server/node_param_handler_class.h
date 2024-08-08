#ifndef AM_UTILS_INCLUDE_PARAM_SERVER_NODE_PARAM_HANDLER_CLASS_H_
#define AM_UTILS_INCLUDE_PARAM_SERVER_NODE_PARAM_HANDLER_CLASS_H_

#include <am_utils/am_ros2_utility.h>
#include <rcl_interfaces/srv/list_parameters.hpp>
#include <rcl_interfaces/srv/get_parameters.hpp>
#include <rcl_interfaces/srv/set_parameters.hpp>


namespace am
{


class NodeParamHandler
{
public: 
    NodeParamHandler(const std::string &name);
    
    ~NodeParamHandler();
    void listParams();

    bool param_exist(const std::string param_name);
    

    void getParams();
    

    void setParam(const rcl_interfaces::msg::Parameter &param);
    
    void print();

    bool all_parameters_are_listed_ = false;
    bool got_all_parameters_ = false;
    
    rclcpp::Client<rcl_interfaces::srv::ListParameters>::SharedPtr list_parameters_srv_;
    rclcpp::Client<rcl_interfaces::srv::GetParameters>::SharedPtr get_parameters_srv_;
    rclcpp::Client<rcl_interfaces::srv::SetParameters>::SharedPtr set_parameters_srv_;
    rclcpp::TimerBase::SharedPtr timer_;
    std::unordered_map<std::string, rclcpp::ParameterValue> parameters_;
private:
    std::string node_name_ {""};
    std::shared_ptr<rcl_interfaces::srv::GetParameters::Request> get_param_req_;
    void timerCB();

    void setParamsCB(rclcpp::Client<rcl_interfaces::srv::SetParameters>::SharedFuture result);

    void listParamCB(rclcpp::Client<rcl_interfaces::srv::ListParameters>::SharedFuture result);

    void getParamsCB(rclcpp::Client<rcl_interfaces::srv::GetParameters>::SharedFuture result);


};

}

#endif //AM_UTILS_INCLUDE_PARAM_SERVER_NODE_PARAM_HANDLER_CLASS_H_