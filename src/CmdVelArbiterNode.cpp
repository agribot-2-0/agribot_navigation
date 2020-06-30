#include "agribot_navigation/CmdVelArbiterNode.h"

CmdVelArbiterNode::CmdVelArbiterNode(ros::NodeHandle *nh):
    _nh(*nh),
    _arbiterState(NO_SELECTION)
{
    // Out topic
    _outPub = _nh.advertise<geometry_msgs::Twist>("/cmd_vel", 1, true);

    // Read topic parameters
    _navTopic = "/cmd_vel_navigation";
    _teleopTopic = "/cmd_vel_teleop";

    std::string startingState;
    _nh.param<std::string>("starting_state", startingState, "stop");

    if (startingState == "stop")
    {
        switchToStop();
    }
    else if (startingState == "navigation")
    {
        switchToNavigation();
    }
    else if (startingState == "teleop")
    {
        switchToTeleop();
    }
    else
    {
        ROS_ERROR_STREAM(
            "Unsuported starting_state provided. Exiting...");
        exit(0);
    }

    // Switching services
    switchToStopService = _nh.advertiseService(
        "switch_to_stop",
        &CmdVelArbiterNode::switchToStopSrvCb, this);
    switchToNavigationService = _nh.advertiseService(
        "switch_to_navigation",
        &CmdVelArbiterNode::switchToNavigationSrvCb, this);
    switchToTeleopService = _nh.advertiseService(
        "switch_to_teleop",
        &CmdVelArbiterNode::switchToTeleopSrvCb, this);
}

void CmdVelArbiterNode::subscribeToNavigation()
{
    _inSub = _nh.subscribe(
        _navTopic, 1, &CmdVelArbiterNode::inCallback, this);
}

void CmdVelArbiterNode::subscribeToTeleop()
{
    _inSub = _nh.subscribe(
        _teleopTopic, 1, &CmdVelArbiterNode::inCallback, this);
}

bool CmdVelArbiterNode::switchToStop()
{
    if (_arbiterState != STOP)
    {
        _inSub.shutdown();
        _arbiterState = STOP;
        publishStopToCmdVel();

        ROS_INFO_STREAM(
            "cmd_vel switched to stop");
    }
    else
    {
        ROS_WARN_STREAM(
            "cmd_vel IS ALREADY switched to stop");
    }

    return true;
}

bool CmdVelArbiterNode::switchToNavigation()
{
    if (_arbiterState != NAVIGATION)
    {
        _inSub.shutdown();
        _arbiterState = NAVIGATION;
        subscribeToNavigation();

        ROS_INFO_STREAM(
            "cmd_vel switched to navigation (topic: " << _navTopic << ")");
    }
    else
    {
        ROS_WARN_STREAM(
            "cmd_vel IS ALREADY switched to navigation");
    }

    return true;
}

bool CmdVelArbiterNode::switchToTeleop()
{
    if (_arbiterState != TELEOP)
    {
        _inSub.shutdown();
        _arbiterState = TELEOP;
        subscribeToTeleop();

        ROS_INFO_STREAM(
            "cmd_vel switched to teleop (topic: " << _teleopTopic << ")");
    }
    else
    {
        ROS_WARN_STREAM(
            "cmd_vel IS ALREADY switched to teleop");
    }

    return true;
}

bool CmdVelArbiterNode::switchToStopSrvCb(std_srvs::Empty::Request &req,
    std_srvs::Empty::Response &res)
{
    return switchToStop();
}

bool CmdVelArbiterNode::switchToNavigationSrvCb(std_srvs::Empty::Request &req,
    std_srvs::Empty::Response &res)
{
    return switchToNavigation();
}

bool CmdVelArbiterNode::switchToTeleopSrvCb(std_srvs::Empty::Request &req,
    std_srvs::Empty::Response &res)
{
    return switchToTeleop();
}

void CmdVelArbiterNode::inCallback(const geometry_msgs::Twist::ConstPtr& msg)
{
    _outPub.publish(msg);
}

void CmdVelArbiterNode::publishStopToCmdVel()
{
    geometry_msgs::Twist msg;

    msg.linear.x = 0.0;
    msg.linear.y = 0.0;
    msg.linear.z = 0.0;

    msg.angular.x = 0.0;
    msg.angular.y = 0.0;
    msg.angular.z = 0.0;

    _outPub.publish(msg);
}

void CmdVelArbiterNode::run()
{
    ros::spin();
}
