#include <am_utils/am_ros2_utility.h>
#include <param_server/param_server_class.h>
#include <param_server/param_server_node.h>
#include <param_server/param_server_stats.h>

std::shared_ptr<am::AMLifeCycle> am::Node::node = nullptr;


int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);

    std::shared_ptr<am::ParamServerNode> am_node = std::make_shared<am::ParamServerNode>("parameter_server");
    std::shared_ptr<am::ParamServerStats> stats = std::make_shared<am::ParamServerStats>(am_node->stats_list_);    

    am::Node::node = am_node;



    am::Node::node = std::make_shared<am::AMLifeCycle>("parameter_server");

    std::shared_ptr<am::ParameterServer> param_server = std::make_shared<am::ParameterServer>();

    rclcpp::spin(am::Node::node);
    rclcpp::shutdown();

    return 0;
}