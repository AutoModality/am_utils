/*
 * vb_main.cpp
 *
 *  Created on: Nov 12, 2019
 *      Author: ubuntu
 */


#include <vb_util_lib/vb_main.h>
#include <brain_box_msgs/msg/heartbeat.hpp>
#include <termios.h>
#include <vb_util_lib/super_status.h>
#include <string>

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


int kbhit (void)
{
    struct timeval tv;
    fd_set rdfs;

    tv.tv_sec = 0;
    tv.tv_usec = 0;

    FD_ZERO(&rdfs);
    FD_SET (STDIN_FILENO, &rdfs);

    select(STDIN_FILENO+1, &rdfs, NULL, NULL, &tv);
    return FD_ISSET(STDIN_FILENO, &rdfs);

}

bool userInputEnabled(ros::NodeHandle& n)
{
    bool user_input = false;

    if (n.hasParam("/DisableUserInput"))
    {
        user_input = false;
    }
    else
    {
        n.getParam("UserInput", user_input);
    }
    ROS_INFO("USER INPUT %s", user_input ? "ENABLED" : "DISABLED");

    return user_input;
}

