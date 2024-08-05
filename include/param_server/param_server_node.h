#ifndef AM_UTILS_INCLUDE_PARAM_SERVER_PARAM_SERVER_NODE_H_
#define AM_UTILS_INCLUDE_PARAM_SERVER_PARAM_SERVER_NODE_H_

#include <param_server/param_server_class.h>

namespace am
{
class ParamServerNode : public AMLifeCycle
{
public:
    ParamServerNode(const std::string &node_name);
    void setParamServer(std::shared_ptr<ParameterServer> am_param_server);
    std::shared_ptr<am::ParameterServer> getParamServer();


private:
    std::shared_ptr<am::ParameterServer> param_server_ = nullptr;
};
}


#endif //AM_UTILS_INCLUDE_PARAM_SERVER_PARAM_SERVER_NODE_H_