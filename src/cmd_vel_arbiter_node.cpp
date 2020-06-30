#include <ros/ros.h>

#include "agribot_navigation/CmdVelArbiterNode.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "cmd_vel_arbiter_node");
    ros::NodeHandle nh("~");
    CmdVelArbiterNode arbiter = CmdVelArbiterNode(&nh);
    arbiter.run();
}
