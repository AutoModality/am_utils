/*
 * vb_main.cpp
 *
 *  Created on: Nov 12, 2019
 *      Author: ubuntu
 */

#include <stdio.h>
#include <string.h>
#include <ctype.h>
#include <stdlib.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <termios.h>
#include <fcntl.h>

#include <vb_util_lib/vb_main.h>
#include <brain_box_msgs/msg/heartbeat.hpp>
#include <vb_util_lib/super_status.h>
#include <am_utils/am_ros2_utility.h>

void changemode(int dir)
{
    static struct termios oldt, newt;
    if ( dir == 1 )
    {
        tcgetattr( STDIN_FILENO, &oldt);
        newt = oldt;
        newt.c_lflag &= ~( ICANON | ECHO );
        tcsetattr( STDIN_FILENO, TCSANOW, &newt);
    }
    else
        tcsetattr( STDIN_FILENO, TCSANOW, &oldt);
}

int kbhit(void)
{
  struct termios oldt, newt;
  int ch;
  int oldf;
 
  tcgetattr(STDIN_FILENO, &oldt);
  newt = oldt;
  newt.c_lflag &= ~(ICANON | ECHO);
  tcsetattr(STDIN_FILENO, TCSANOW, &newt);
  oldf = fcntl(STDIN_FILENO, F_GETFL, 0);
  fcntl(STDIN_FILENO, F_SETFL, oldf | O_NONBLOCK);
 
//  ROS_INFO_THROTTLE(0.5 "  GGG getch");
  ch = getchar();
 
  tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
  fcntl(STDIN_FILENO, F_SETFL, oldf);
 ROS_INFO_THROTTLE(0.5, "  GGG getch %d", ch);
 
  if(ch != EOF)
  {
    ungetc(ch, stdin);
    return 1;
  }
 
  return 0;
}

// For non-blocking keyboard inputs
int getchr(void)
{
  int ch;
  struct termios oldt;
  struct termios newt;

  // Store old settings, and copy to new settings
  tcgetattr(STDIN_FILENO, &oldt);
  newt = oldt;

  // Make required changes and apply the settings
  newt.c_lflag &= ~(ICANON | ECHO);
  newt.c_iflag |= IGNBRK;
  newt.c_iflag &= ~(INLCR | ICRNL | IXON | IXOFF);
  newt.c_lflag &= ~(ICANON | ECHO | ECHOK | ECHOE | ECHONL | ISIG | IEXTEN);
  newt.c_cc[VMIN] = 1;
  newt.c_cc[VTIME] = 0;
  tcsetattr(fileno(stdin), TCSANOW, &newt);

  // Get the current character
  ch = getchar();

  // Reapply old settings
  tcsetattr(STDIN_FILENO, TCSANOW, &oldt);

  return ch;
}

// int kbhit (void)
// {
//     struct timeval tv;
//     fd_set rdfs;

//     tv.tv_sec = 0;
//     tv.tv_usec = 0;

//     FD_ZERO(&rdfs);
//     FD_SET (STDIN_FILENO, &rdfs);

//     select(STDIN_FILENO+1, &rdfs, NULL, NULL, &tv);
//     return FD_ISSET(STDIN_FILENO, &rdfs);
// }

bool userInputEnabled(rclcpp::Node::SharedPtr node)
{
    bool user_input = false;
    bool disable_global_input = false;

    am::getParam<bool>("DisableUserInput", disable_global_input, disable_global_input);
    if (disable_global_input)
    {
        user_input = false;
    }
    else
    {
        am::getParam<bool>("UserInput", user_input, user_input);
    }
    ROS_INFO(">>> USER INPUT %s", user_input ? "ENABLED" : "DISABLED");

    return user_input;
}

