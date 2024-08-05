#include <param_server/node_param_handler_class.h>

namespace am
{

NodeParamHandler::NodeParamHandler(const std::string &name)
{
node_name_ = name.substr(1);
list_parameters_srv_ = am::Node::node->create_client<rcl_interfaces::srv::ListParameters>(name+"/list_parameters");
get_parameters_srv_ = am::Node::node->create_client<rcl_interfaces::srv::GetParameters>(name+"/get_parameters");
set_parameters_srv_ = am::Node::node->create_client<rcl_interfaces::srv::SetParameters>(name+"/set_parameters");


timer_ = am::Node::node->create_wall_timer(am::toDuration(5.0), std::bind(&NodeParamHandler::timerCB, this));

listParams();
}
NodeParamHandler::~NodeParamHandler()
{

}

void NodeParamHandler::listParams()
{
    int cnt = 0;
    while (!list_parameters_srv_->wait_for_service(am::toDuration(1.0)) && cnt < 3)
    {
        cnt++;
        if (!rclcpp::ok())
        {
            ROS_ERROR("Interrupted while waiting for the service. Exiting.");
            exit(-1);
        }
        ROS_INFO("list_parameters_srv service not available for %s, waiting again...",  node_name_.c_str());
    }

    auto req = std::make_shared<rcl_interfaces::srv::ListParameters::Request>();

    //client example with callback
    auto result_future = list_parameters_srv_->async_send_request(req,
            std::bind(&NodeParamHandler::listParamCB, this, std::placeholders::_1));
}

bool NodeParamHandler::param_exist(const std::string param_name)
{
    return (parameters_.find(param_name) != parameters_.end());
}

void NodeParamHandler::getParams()
{
    if(got_all_parameters_)
    {
        got_all_parameters_ = false;
        return;
    }
    int cnt = 0;
    while (!get_parameters_srv_->wait_for_service(am::toDuration(1.0)) && cnt < 3)
    {
        cnt++;
        if (!rclcpp::ok())
        {
            ROS_ERROR("Interrupted while waiting for the service. Exiting.");
            exit(-1);
        }
        ROS_INFO("get_parameters_srv service not available for %s, waiting again...", node_name_.c_str());
    }

    get_param_req_ = std::make_shared<rcl_interfaces::srv::GetParameters::Request>();

    std::unordered_map<std::string, rclcpp::ParameterValue>::iterator it = parameters_.begin();
    for(it; it != parameters_.end(); ++it)
    {
        get_param_req_->names.push_back(it->first);
    }
        
    

    //client example with callback
    auto result_future = get_parameters_srv_->async_send_request(get_param_req_,
            std::bind(&NodeParamHandler::getParamsCB, this, std::placeholders::_1));
    
}

void NodeParamHandler::setParam(const rcl_interfaces::msg::Parameter &param)
{
    if(param_exist(param.name))
    {

        int cnt = 0;
        while (!set_parameters_srv_->wait_for_service(am::toDuration(1.0)) && cnt < 3)
        {
            cnt++;
            if (!rclcpp::ok())
            {
                ROS_ERROR("Interrupted while waiting for the service. Exiting.");
                exit(-1);
            }
            ROS_INFO("get_parameters_srv service not available for %s, waiting again...",  node_name_.c_str());
        }

        auto set_param_req_ = std::make_shared<rcl_interfaces::srv::SetParameters::Request>();
        set_param_req_->parameters.push_back(param);

        auto result_future = set_parameters_srv_->async_send_request(set_param_req_,
            std::bind(&NodeParamHandler::setParamsCB, this, std::placeholders::_1));
    }
}

void NodeParamHandler::print()
{
    
}

void NodeParamHandler::timerCB()
{
    //ROS_INFO("Hello from %s", node_name_.c_str());
    if(all_parameters_are_listed_)
    {
        getParams();
    }
}

void NodeParamHandler::setParamsCB(rclcpp::Client<rcl_interfaces::srv::SetParameters>::SharedFuture result)
{
    std::shared_ptr<rcl_interfaces::srv::SetParameters::Response> response = result.get();
    //do it in callback
    //parameters_[name] = rclcpp::ParameterValue(pv);
}

void NodeParamHandler::listParamCB(rclcpp::Client<rcl_interfaces::srv::ListParameters>::SharedFuture result)
{
    std::shared_ptr<rcl_interfaces::srv::ListParameters::Response> response = result.get();
    for(std::string &param : response->result.names)
    {
        if(param.find("qos") != std::string::npos)
        {
            continue;
        }
        parameters_[param] = rclcpp::ParameterValue();
        //ROS_INFO("%s.%s", node_name_.c_str(), param.c_str());
        
    }

    all_parameters_are_listed_ = true;
}

void NodeParamHandler::getParamsCB(rclcpp::Client<rcl_interfaces::srv::GetParameters>::SharedFuture result)
{
    std::shared_ptr<rcl_interfaces::srv::GetParameters::Response> response = result.get();
    
    for(int i = 0; i < response->values.size(); i++)
    {
        rclcpp::ParameterValue parameter(response->values[i]);
        
        parameters_[get_param_req_->names[i]] = parameter;
        std::string full_param_name = node_name_ + "." + get_param_req_->names[i];

        //ROS_INFO("%s: %s", full_param_name.c_str(), to_string(parameter).c_str());
    }

    got_all_parameters_ = true;
}

}