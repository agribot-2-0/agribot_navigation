/*
* Copyright (C) 2020 Agribot. All rights reserved.
*/

#include <ros/ros.h>

#include "agribot_navigation/cmd_vel_arbiter/CmdVelArbiterNode.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "cmd_vel_arbiter_node");
    ros::NodeHandle nh("~");
    CmdVelArbiterNode arbiter = CmdVelArbiterNode(&nh);
    arbiter.run();
}
