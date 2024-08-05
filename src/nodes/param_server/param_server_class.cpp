#include <param_server/param_server_class.h>
#include <rcl_interfaces/srv/list_parameters.hpp>
#include <rcl_interfaces/srv/get_parameters.hpp>
#include <rcl_interfaces/srv/set_parameters.hpp>
#include <brain_box_msgs/msg/parameter_array.hpp>
#include <brain_box_msgs/msg/key_value.hpp>
#include <brain_box_msgs/msg/log_control.hpp>
#include <vb_util_lib/bag_logger.h>
#include <vb_util_lib/vb_main.h>
#include <yaml-cpp/yaml.h>
#include <vb_util_lib/key_value_stamped.h>

namespace am
{

ParameterServer::ParameterServer()
{

    getParams();
    
    updateTimer_ = am::Node::node->create_wall_timer(am::toDuration(5.0), std::bind(&ParameterServer::updateTimerCB, this));
    internal_pub_ = am::Node::node->create_publisher<brain_box_msgs::msg::ParameterArray>(param_server_outlet_topic_, 1);
    log_control_sub_ = am::Node::node->create_subscription<brain_box_msgs::msg::LogControl>("/ctrl/log_control", 1, std::bind(&ParameterServer::logControlCB, this, std::placeholders::_1));
    fp_params_sub_ = am::Node::node->create_subscription<brain_box_msgs::msg::KeyValue>("/param_update",100, std::bind(&ParameterServer::fpParamCB, this, std::placeholders::_1));
    param_sub_ = am::Node::node->create_subscription<brain_box_msgs::msg::ParameterArray>(param_server_outlet_topic_, 10, std::bind(&ParameterServer::paramsCB, this, std::placeholders::_1));

}

ParameterServer::~ParameterServer()
{
    if(mFile.is_open())
    {
        mFile.close();
    }
}

    
void ParameterServer::getParams()
{
    am::getParam<std::string>("global_yaml", global_yaml_file_location_, global_yaml_file_location_);
    ROS_INFO("global_yaml: %s", global_yaml_file_location_.c_str());
    
    std::ifstream fin;
    fin.open(global_yaml_file_location_);
    if (!fin.is_open()) {
        ROS_ERROR("Failed to open file.");
        return;
    }

    // Parse the YAML file
    YAML::Node config = YAML::Load(fin);
    printYamlNode(config);

}

bool ParameterServer::extractNodeNameAndParamName(const std::string& input, std::string& nodeName, std::string& paramName) 
{
    // Find the position of the first dot
    size_t dotPos = input.find('.');

    // If there's no dot, set nodeName as the input and paramName as empty
    if (dotPos == std::string::npos) {
        nodeName = input;
        paramName = "";
        return false;
    }

    // Extract the nodeName and paramName
    nodeName = std::string("/") + input.substr(0, dotPos);
    paramName = input.substr(dotPos + 1);
    return true;
}

//flightplan parameters
void ParameterServer::fpParamCB(const brain_box_msgs::msg::KeyValue::SharedPtr param_msg)
{
    ROS_INFO("Received Parameters from FP");
    std::shared_ptr<KeyValueStamped> key_value_ = std::make_shared<KeyValueStamped>(param_msg);
    double d_value = 0.0;
    int i_value = 0;
    std::string s_value = "";
    bool b_value = false;

    brain_box_msgs::msg::ParameterArray pa;
    brain_box_msgs::msg::Parameter p;
    p.receiver_type = brain_box_msgs::msg::Parameter::RECEIVER_TYPE_LOCAL;
    
    if(!extractNodeNameAndParamName(key_value_->getKey(), p.node_name, p.parameter.name))
    {
        ROS_INFO("The received parameter does not have a correct ros2 structure: %s", key_value_->getKey().c_str());
        return;
    }

    if(param_msg->type == brain_box_msgs::msg::KeyValue::DOUBLE)
    {
        key_value_->getValue(p.parameter.value.double_value);
        ROS_INFO(GREEN "Received the following parameter: %s for node: %s --> %f" COLOR_RESET, p.parameter.name.c_str(), p.node_name.c_str(), p.parameter.value.double_value);
    }
    else if(param_msg->type == brain_box_msgs::msg::KeyValue::INT)
    {
        int i_value = 0;
        key_value_->getValue(i_value);
        p.parameter.value.integer_value = i_value;
        ROS_INFO(GREEN "Received the following parameter: %s for node: %s --> %d" COLOR_RESET, p.parameter.name.c_str(), p.node_name.c_str(), (int)p.parameter.value.integer_value);
    }
    else if(param_msg->type == brain_box_msgs::msg::KeyValue::BOOL)
    {
        key_value_->getValue(p.parameter.value.bool_value);
        ROS_INFO(GREEN "Received the following parameter: %s for node: %s --> %s" COLOR_RESET, p.parameter.name.c_str(), p.node_name.c_str(), (p.parameter.value.bool_value?"True":"False"));
    }
    else if(param_msg->type == brain_box_msgs::msg::KeyValue::STRING)
    {
        key_value_->getValue(p.parameter.value.string_value);
        ROS_INFO(GREEN "Received the following parameter: %s for node: %s --> %s" COLOR_RESET, p.parameter.name.c_str(), p.node_name.c_str(), p.parameter.value.string_value.c_str());
    }

    pa.parameters.push_back(p);
    internal_pub_->publish(pa);
    
}


void ParameterServer::printYamlNode(const YAML::Node& node, const std::string& parentKey) 
{
    // Check the type of the node
    switch (node.Type()) {
        case YAML::NodeType::Scalar: 
        {
            if(parentKey.find("/**.ros__parameters.") != std::string::npos)
            {
                std::cout << "Key: " << parentKey.substr(std::string("/**.ros__parameters.").length()) << ", Value: " << node.as<std::string>() << std::endl;
                
                global_parameters_[parentKey.substr(std::string("/**.ros__parameters.").length())] = rclcpp::ParameterValue();
            }
            
            break;
        }
        case YAML::NodeType::Sequence: 
        {
            for (std::size_t i = 0; i < node.size(); ++i) {
                printYamlNode(node[i], parentKey + "[" + std::to_string(i) + "]");
            }
            break;
        }
        case YAML::NodeType::Map: 
        {
            for (const auto& entry : node) {
                std::string key = entry.first.as<std::string>();
                printYamlNode(entry.second, parentKey.empty() ? key : parentKey + "." + key);
            }
        }
        case YAML::NodeType::Null: 
        {
            break;
        }
        default:
            std::cerr << "Unknown YAML node type" << std::endl;
    }   
}

void ParameterServer::logControlCB(const brain_box_msgs::msg::LogControl::SharedPtr msg)
{
    //start logging
    if(msg->enable && !mFile.is_open())
    {
        //yaml writer
        YAML::Node yaml_data;
        if(!paramtoYaml(yaml_data))
        {
            ROS_WARN("faild to convert parameters to yaml file");
            return;
        }
        std::string yaml_file_name = BagLogger::getLogFileName("PM") + ".yaml";
        
        // Write YAML node to a file
        std::ostringstream oss;
        std::ofstream fout(yaml_file_name);
        //fout << yaml_data;
        oss << yaml_data;
        std::string cleaned = cleanString(oss.str());
        //ROS_INFO("oss: %s", cleaned.c_str());
        fout << cleaned;
        fout.close();

        //txt writer
        log_file_name_ = BagLogger::getLogFileName("PM") + ".txt";   
        mFile.open(log_file_name_);
        logParameters();    
    }
    if(!msg->enable)
    {
        mFile.close();
    }

}

bool ParameterServer::paramtoYaml(YAML::Node &yaml_data)
{
    std::string ros_str = "ros__parameters";

    std::unordered_map<std::string, std::shared_ptr<am::NodeParamHandler>>::iterator it = nodes_.begin();
    for(; it != nodes_.end(); ++it)
    {
        std::unordered_map<std::string, rclcpp::ParameterValue>::iterator param_it = it->second->parameters_.begin();
        for(; param_it != it->second->parameters_.end(); ++param_it)
        {
            //check if this is global parameter
            std::unordered_map<std::string, rclcpp::ParameterValue>::iterator global_it = global_parameters_.find(param_it->first);
            if(global_it != global_parameters_.end())
            {
                //ROS_INFO("Found %s", global_it->first.c_str());
                global_it->second = param_it->second;
                continue;
            }
            
            //ROS_INFO("src: %s, result: %s", to_string(param_it->second).c_str(), cleaned.c_str());

            yaml_data[it->first.substr(1)][ros_str][param_it->first]= to_string(param_it->second);
        }         
    }


    //now log the global parameters
    std::unordered_map<std::string, rclcpp::ParameterValue>::iterator it2 = global_parameters_.begin();
    for(; it2 != global_parameters_.end(); ++it2)
    {
        //ROS_INFO("recording global parameter: %s : %s", it2->first.c_str(), to_string(it2->second).c_str());
        yaml_data["/**"][ros_str][it2->first]= to_string(it2->second);
    }

    return true;
}

std::string ParameterServer::cleanString(const std::string &input)
{
    std::string result;
    for (size_t i = 0; i < input.size(); ++i) {
        if (i < input.size() - 1 && input[i] == '"' && input[i + 1] == '[') {
            result += '[';
            ++i; // Skip the next character
        } else if (i < input.size() - 1 && input[i] == ']' && input[i + 1] == '"') {
            result += ']';
            ++i; // Skip the next character
        } else {
            result += input[i];
        }
    }
    return result;
}

bool ParameterServer::isArray(const rclcpp::ParameterValue &pv)
{
    if(pv.get_type() == rclcpp::ParameterType::PARAMETER_BOOL_ARRAY
    || pv.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER_ARRAY
    || pv.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE_ARRAY
    || pv.get_type() == rclcpp::ParameterType::PARAMETER_BYTE_ARRAY
    || pv.get_type() == rclcpp::ParameterType::PARAMETER_STRING_ARRAY)
    {
        return true;
    }
    return false;
}

void ParameterServer::logParameters()
{
    std::unordered_map<std::string, std::shared_ptr<am::NodeParamHandler>>::iterator it = nodes_.begin();
    for(; it != nodes_.end(); ++it)
    {

        std::unordered_map<std::string, rclcpp::ParameterValue>::iterator param_it = it->second->parameters_.begin();
        for(; param_it != it->second->parameters_.end(); ++param_it)
        {
            if(mFile.is_open())
            {
                mFile << it->first.substr(1) + "." + param_it->first << ": " << to_string(param_it->second) << "\n";
            }
        }         
    }
    
    ROS_INFO(GREEN "Parameters are logged" COLOR_RESET);
}

void ParameterServer::paramsCB(const brain_box_msgs::msg::ParameterArray::SharedPtr msg)
{
    for(brain_box_msgs::msg::Parameter &param : msg->parameters)
    {
        //send global parameters
        if(param.receiver_type == brain_box_msgs::msg::Parameter::RECEIVER_TYPE_GLOBAL)
        {
            //sending the param to every single node
            std::unordered_map<std::string, std::shared_ptr<am::NodeParamHandler>>::iterator it = nodes_.begin();
            for(it; it != nodes_.end(); ++it)
            {                   
                it->second->setParam(param.parameter);
            }
            continue;
        }
        //send local parameters
        else if(param.receiver_type == brain_box_msgs::msg::Parameter::RECEIVER_TYPE_LOCAL)
        {
            //lets find the node first
            std::unordered_map<std::string, std::shared_ptr<am::NodeParamHandler>>::iterator it = nodes_.find(param.node_name);
            if(it == nodes_.end())
            {
                ROS_INFO(RED "The param change request for %s node is invalid. The node cannot be found" COLOR_RESET, param.node_name.c_str());
                continue;
            }
            ROS_INFO("Sending %s to %s --> found node name: %s", param.parameter.name.c_str(), param.node_name.c_str(), it->first.c_str());
            it->second->setParam(param.parameter);
        }
    }
}

void ParameterServer::updateTimerCB()
{
    updateNodesList();
}
void ParameterServer::updateNodesList()
{
    std::vector<std::string> node_names = am::Node::node->get_node_names();
    std::string node_name(am::Node::node->get_name());
    int cnt = 0;
    ROS_INFO("Found %d nodes. current name: %s -- > nodes_.size():%d", (int)node_names.size(), node_name.c_str(), (int)nodes_.size());
    for(std::string &name : node_names)
    {
        //ignoring condition        
        if(nodes_.find(name) != nodes_.end() || name.find(node_name) != std::string::npos || isNodeInBlackList(name))
        {
            continue;
        }
        ROS_INFO(CYAN "Node[%d]: %s" COLOR_RESET, cnt++, name.c_str());

        //create a new instance
        nodes_[name] = std::make_shared<am::NodeParamHandler>(name);
    }
} 
bool ParameterServer::isNodeInBlackList(const std::string &node_name)
{
    for(std::string &name : black_list_)
    {
        if(node_name.find(name) != std::string::npos)
        {
            return true;
        }
    }

    return false;
}   
}