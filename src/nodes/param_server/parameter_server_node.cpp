#include <am_utils/am_ros2_utility.h>
#include <param_server/param_server_node.h>

namespace am
{
ParamServerNode::ParamServerNode(const std::string &node_name): AMLifeCycle(node_name)
{

}

void ParamServerNode::setParamServer(std::shared_ptr<am::ParameterServer> am_param_server)
{
    param_server_ = am_param_server;
}


std::shared_ptr<am::ParameterServer> ParamServerNode::getParamServer()
{
    return param_server_;
}

}
