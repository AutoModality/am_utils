#include <am_utils/am_ros2_utility.h>
#include <rcl_interfaces/srv/set_parameters.hpp>

std::shared_ptr<am::AMLifeCycle> am::Node::node = nullptr;

class ParamReceiver
{
public:
    ParamReceiver()
    {

        getParams();

        set_param_srv_ = am::Node::node->create_service<rcl_interfaces::srv::SetParameters>("/parameter_receiver_tester/set_parameters",
        std::bind(&ParamReceiver::setParamCB,this, std::placeholders::_1, std::placeholders::_2) );


        testEnvVariables();
    }

    ~ParamReceiver()
    {

    }
private:

    rclcpp::Service<rcl_interfaces::srv::SetParameters>::SharedPtr set_param_srv_;

    //parameters
    std::string world_file = "NO FILE LOCATION HAS BEEN SET";
    double parameter_a {-10000.0};
    int parameter_b {-1};
    bool parameter_c {false};

    void testEnvVariables()
    {
        ROS_INFO(GREEN "AM_PLATFORM: %s" COLOR_RESET, am::getAmPlatform().c_str());
        ROS_INFO(GREEN "AM_ID: %s" COLOR_RESET, am::getAmId().c_str());
        ROS_INFO(GREEN "AM_LOCATION: %s" COLOR_RESET, am::getAmLocation().c_str());
        ROS_INFO(GREEN "AM_APPLICATION: %s" COLOR_RESET, am::getAmApplication().c_str());
        ROS_INFO(GREEN "AM_SENSORS: %s" COLOR_RESET, am::getAmSensors().c_str());
    }

    void getParams()
    {
        am::getParam<std::string>("World.model_file", world_file, world_file);
        ROS_INFO("world_file: %s", world_file.c_str());

        am::getParam<double>("parameter_a", parameter_a, parameter_a);
        ROS_INFO("parameter_a: %f", parameter_a);


        am::getParam<int>("parameter_b", parameter_b, parameter_b);
        ROS_INFO("parameter_b: %d", parameter_b);


        am::getParam<bool>("parameter_c", parameter_c, parameter_c);
        ROS_INFO("parameter_c: %s", am::toString(parameter_c).c_str());

    }

    void setParamCB(const std::shared_ptr<rcl_interfaces::srv::SetParameters::Request> request,
			std::shared_ptr<rcl_interfaces::srv::SetParameters::Response> response)
    {
        for(rcl_interfaces::msg::Parameter &param: request->parameters)
        {
            if(param.name.find("World.model_file") != std::string::npos)
            {
                world_file = param.value.string_value;
                ROS_INFO(GREEN "Received a new value for World.model_file: %s" COLOR_RESET, world_file.c_str());
            }
            if(param.name.find("parameter_a") != std::string::npos)
            {
                parameter_a = param.value.double_value;
                ROS_INFO(GREEN "Received a new value for parameter_a: %f" COLOR_RESET, parameter_a);
            }
            if(param.name.find("parameter_b") != std::string::npos)
            {
                parameter_b = param.value.integer_value;
                ROS_INFO(GREEN "Received a new value for parameter_b: %d" COLOR_RESET, parameter_b);
            }
            if(param.name.find("parameter_c") != std::string::npos)
            {
                parameter_c = param.value.bool_value;
                ROS_INFO(GREEN "Received a new value for parameter_c: %s" COLOR_RESET, am::toString(parameter_c).c_str());
            }
        }
    }
};


int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);

    am::Node::node = std::make_shared<am::AMLifeCycle>("parameter_receiver_tester");

    std::shared_ptr<ParamReceiver> param_receiver = std::make_shared<ParamReceiver>();

    rclcpp::spin(am::Node::node);

    rclcpp::shutdown();

    return 0;
}