#ifndef _VB_MAIN_
#define _VB_MAIN_

#include <vb_util_lib/bag_logger.h>
#include <rclcpp/rclcpp.hpp>

void changemode(int dir);
int kbhit (void);
bool userInputEnabled(rclcpp::Node::SharedPtr n);


#endif


