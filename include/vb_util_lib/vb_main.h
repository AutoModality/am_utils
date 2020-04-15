#ifndef _VB_MAIN_
#define _VB_MAIN_

// #include <vb_util_lib/bag_logger.h>
#include <ros/ros.h>

void changemode(int dir);
int kbhit (void);
bool userInputEnabled(ros::NodeHandle& n);


#endif


