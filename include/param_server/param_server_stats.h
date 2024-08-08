#ifndef AM_UTILS_INCLUDE_PARAM_SERVER_NODE_PARAM_SERVER_STATS_H_
#define AM_UTILS_INCLUDE_PARAM_SERVER_NODE_PARAM_SERVER_STATS_H_

#include <am_utils/am_ros2_utility.h>
#include <super_lib/am_stat_list.h>
namespace am
{


class ParamServerStats
{
public: 
    ParamServerStats(AMStatList &stat_list);
    ~ParamServerStats();
private:
};

}

#endif //AM_UTILS_INCLUDE_PARAM_SERVER_NODE_PARAM_SERVER_STATS_H_