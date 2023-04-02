#ifndef _VB_MAIN_
#define _VB_MAIN_

#include <vb_util_lib/bag_logger.h>
#include <am_utils/am_ros2_utility.h>

void changemode(int dir);
int kbhit (void);
int getchr (void);
bool userInputEnabled(rclcpp::Node::SharedPtr n);


#endif


