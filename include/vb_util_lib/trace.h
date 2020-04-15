#ifndef VISBOX_PACKAGES_COMMON_UTILS_UTIL_LIB_INCLUDE_VB_UTIL_LIB_TRACE_H_
#define VISBOX_PACKAGES_COMMON_UTILS_UTIL_LIB_INCLUDE_VB_UTIL_LIB_TRACE_H_

#define NODE_FUNC ros::this_node::getName()  << ":" <<  __func__ << ": "
#define NODE_FUNC_CAT (ros::this_node::getName()  + ":" +  __func__ + ": ").c_str()

#endif
