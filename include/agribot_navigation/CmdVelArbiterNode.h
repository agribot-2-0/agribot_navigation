/*
* Copyright (C) 2020 Agribot. All rights reserved.
*/

#pragma once

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <std_srvs/Empty.h>

#include <string>

enum class ArbiterState
{
    NO_SELECTION,
    STOP,
    NAVIGATION,
    TELEOP
};

class CmdVelArbiterNode
{
 private:
    ros::NodeHandle _nh;
    ros::Publisher _outPub;
    ros::Subscriber _inSub;

    ros::ServiceServer switchToStopService;
    ros::ServiceServer switchToNavigationService;
    ros::ServiceServer switchToTeleopService;

    ros::Timer timeoutTimer;
    float timeout;

    ArbiterState _arbiterState;
    std::string _navTopic;
    std::string _teleopTopic;

 public:
    explicit CmdVelArbiterNode(ros::NodeHandle *nh);

    void subscribeToNavigation();
    void subscribeToTeleop();
    bool switchToStop();
    bool switchToNavigation();
    bool switchToTeleop();

    void createTimeoutTimer();
    void stopTimeoutTimer();
    void startTimeoutTimer();
    void restartTimeoutTimer();

    bool switchToStopSrvCb(std_srvs::Empty::Request &req,  // NOLINT(runtime/references)
        std_srvs::Empty::Response &res);  // NOLINT(runtime/references)
    bool switchToNavigationSrvCb(std_srvs::Empty::Request &req,  // NOLINT(runtime/references)
        std_srvs::Empty::Response &res);  // NOLINT(runtime/references)
    bool switchToTeleopSrvCb(std_srvs::Empty::Request &req,  // NOLINT(runtime/references)
        std_srvs::Empty::Response &res);  // NOLINT(runtime/references)

    void inCallback(const geometry_msgs::Twist::ConstPtr& msg);
    void timeoutCallback(const ros::TimerEvent& event);
    void publishStopToCmdVel();
    void run();
};
