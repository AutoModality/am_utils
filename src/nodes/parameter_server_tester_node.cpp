#include <am_utils/am_ros2_utility.h>
#include <brain_box_msgs/msg/parameter_array.hpp>

std::shared_ptr<am::AMLifeCycle> am::Node::node;

class ParamServerTester
{
public:
    ParamServerTester()
    {
        setupParameters();
        param_pub_ = am::Node::node->create_publisher<brain_box_msgs::msg::ParameterArray>("/param_server/params", 1);
        timer_ = am::Node::node->create_wall_timer(am::toDuration(10.0), std::bind(&ParamServerTester::timerCB, this));
    }
    ~ParamServerTester()
    {

    }
private:

    rclcpp::Publisher<brain_box_msgs::msg::ParameterArray>::SharedPtr param_pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    brain_box_msgs::msg::ParameterArray parameters_;

    void setupParameters()
    {
        //local parameters to vineyard
        brain_box_msgs::msg::Parameter param;
        brain_box_msgs::msg::Parameter empty_param;
        param.node_name = "/vineyard";
        param.receiver_type = brain_box_msgs::msg::Parameter::RECEIVER_TYPE_LOCAL;
        param.parameter.name = "ground_detect_min_z";
        param.parameter.value.double_value = -1000.0;
        param.parameter.value.type = 3;
        parameters_.parameters.push_back(param);

        //global parameters to all
        param = empty_param;
        param.node_name = "";
        param.receiver_type = brain_box_msgs::msg::Parameter::RECEIVER_TYPE_GLOBAL;
        param.parameter.name = "World.model_file";
        param.parameter.value.string_value = "/home/ubuntu/ros2_ws/install/am_config/share/am_config/mission/world/world_vineyard_mockup_888888888888888888.xml";
        param.parameter.value.type = 4;
        parameters_.parameters.push_back(param);

        //local parameter to locator
        param = empty_param;
        param.node_name = "/am_locator";
        param.receiver_type = brain_box_msgs::msg::Parameter::RECEIVER_TYPE_LOCAL;
        param.parameter.name = "sensors.input.sensor_6.offset.use";
        param.parameter.value.bool_value = true;
        param.parameter.value.type = 1;
        parameters_.parameters.push_back(param);

        //local parameter to parameter_receiver_tester
        param = empty_param;
        param.node_name = "/parameter_receiver_tester";
        param.receiver_type = brain_box_msgs::msg::Parameter::RECEIVER_TYPE_LOCAL;
        param.parameter.name = "parameter_a";
        param.parameter.value.double_value = 12345.0;
        param.parameter.value.type = 3;
        parameters_.parameters.push_back(param);

        //local parameter to parameter_receiver_tester
        param = empty_param;
        param.node_name = "/parameter_receiver_tester";
        param.receiver_type = brain_box_msgs::msg::Parameter::RECEIVER_TYPE_LOCAL;
        param.parameter.name = "parameter_b";
        param.parameter.value.integer_value = 54321;
        param.parameter.value.type = 2;
        parameters_.parameters.push_back(param);

        //local parameter to parameter_receiver_tester
        param = empty_param;
        param.node_name = "/parameter_receiver_tester";
        param.receiver_type = brain_box_msgs::msg::Parameter::RECEIVER_TYPE_LOCAL;
        param.parameter.name = "parameter_c";
        param.parameter.value.bool_value = true;
        param.parameter.value.type = 1;
        parameters_.parameters.push_back(param);



    }

    void timerCB()
    {
        if(param_pub_->get_subscription_count() == 0)
        {
            return;
        }
        param_pub_->publish(parameters_);

        timer_->cancel();

        exit(0);
    }


};


int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);

    am::Node::node = std::make_shared<am::AMLifeCycle>("parameter_server_tester");

    std::shared_ptr<ParamServerTester> param_server_tester = std::make_shared<ParamServerTester>();

    rclcpp::spin(am::Node::node);

    rclcpp::shutdown();

    return 0;
}