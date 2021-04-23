/*
 * Copyright (C) 2020 Agribot. All rights reserved.
 */

#include <agribot_msgs/AgribotCaterpillar.h>
#include <agribot_msgs/SteeringMode.h>
#include <agribot_navigation/KinematicsConfig.h>
#include <agribot_navigation/kinematics_control/KinematicsControl.h>
#include <dynamic_reconfigure/server.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <ros/console.h>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/Float32.h>
#include <tf/tf.h>

#define ROS_TIMEOUT 20
#define PI 3.14159265359
#define RAD_TO_DEG (180 / PI)
#define DEG_TO_RAD (PI / 180)

KinematicsControl KinControl;
bool bDataReceived = false;

void cmdVelCallback(const geometry_msgs::Twist cmd_msg)
{
  KinControl.vSetInputLinear(cmd_msg.linear.x, cmd_msg.linear.y, cmd_msg.linear.z);
  KinControl.vSetInputAngular(cmd_msg.angular.x, cmd_msg.angular.y, cmd_msg.angular.z);
  bDataReceived = true;
}

void encoderFrontCallback(const std_msgs::Float32 front_cart_msg)
{
  KinControl.vSetFrontCartAngle(front_cart_msg.data);
  bDataReceived = true;
}

void encoderBackCallback(const std_msgs::Float32 back_cart_msg)
{
  KinControl.vSetBackCartAngle(back_cart_msg.data);
  bDataReceived = true;
}

void callback(agribot_navigation::KinematicsConfig& config,  // NOLINT(runtime/references)
              uint32_t level)
{
  KinControl.vSetPreciseKp(config.coarse_Kp);
  KinControl.vSetPreciseKi(config.coarse_Ki);
  KinControl.vSetPreciseKd(config.coarse_Kd);
  KinControl.vSetCoarseKp(config.precise_Kp);
  KinControl.vSetCoarseKi(config.precise_Ki);
  KinControl.vSetCoarseKd(config.precise_Kd);
  KinControl.vSetFrontLimit(config.precision_range);
  KinControl.vSetBackLimit(config.front_limit);
  KinControl.vSetPrecisionRange(config.back_limit);
}

bool driving_mode_callback(agribot_msgs::SteeringMode::Request& request,    // NOLINT(runtime/references)
                           agribot_msgs::SteeringMode::Response& response)  // NOLINT(runtime/references)
{
  if (request.steering_mode == agribot_msgs::SteeringMode::Request::FOUR_WHEEL)
  {
    ROS_WARN_STREAM("[KinematicsController] Switched to FOUR WHEEL steering mode");
    KinControl.setDrivingMode(SteeringMode::four_wheel);
    response.result = true;
  }
  else if (request.steering_mode == agribot_msgs::SteeringMode::Request::FRONT_WHEEL)
  {
    ROS_WARN_STREAM("[KinematicsController] Switched to FRONT WHEEL steering mode");
    KinControl.setDrivingMode(SteeringMode::front_wheel);
    response.result = true;
  }
  else if (request.steering_mode == agribot_msgs::SteeringMode::Request::CRAB)
  {
    ROS_WARN_STREAM("[KinematicsController] Switched to CRAB steering mode");
    KinControl.setDrivingMode(SteeringMode::crab);
    response.result = true;
  }
  else
  {
    response.result = false;
  }
  return true;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "kinematics_control");
  dynamic_reconfigure::Server<agribot_navigation::KinematicsConfig> server;
  dynamic_reconfigure::Server<agribot_navigation::KinematicsConfig>::CallbackType f;
  f = boost::bind(&callback, _1, _2);
  server.setCallback(f);

  ros::NodeHandle n;
  ros::NodeHandle pnh("~kinematics_control");

  double CoarseKp;
  double CoarseKi;
  double CoarseKd;
  double PreciseKp;
  double PreciseKd;
  double PreciseKi;
  double PrecisionRange;
  double FrontLimit;
  double BackLimit;

  pnh.param("coarse_Kp", CoarseKp, 0.25);
  pnh.param("coarse_Ki", CoarseKi, 0.05);
  pnh.param("coarse_Kd", CoarseKd, 0.3);

  pnh.param("precise_Kp", PreciseKp, 0.25);
  pnh.param("precise_Ki", PreciseKi, 0.05);
  pnh.param("precise_Kd", PreciseKd, 0.3);

  pnh.param("precision_range", PrecisionRange, 7.2);

  pnh.param("front_limit", FrontLimit, 20.7);
  pnh.param("back_limit", BackLimit, 20.7);

  KinControl.vSetPreciseKp(CoarseKp);
  KinControl.vSetPreciseKi(CoarseKi);
  KinControl.vSetPreciseKd(CoarseKd);
  KinControl.vSetCoarseKp(PreciseKp);
  KinControl.vSetCoarseKi(PreciseKd);
  KinControl.vSetCoarseKd(PreciseKi);
  KinControl.vSetFrontLimit(PrecisionRange);
  KinControl.vSetBackLimit(FrontLimit);
  KinControl.vSetPrecisionRange(BackLimit);

  KinControl.vCreatePubs(n);
  ros::Rate loop_rate(100);
  std::string command_sub_topic;

  if (argc < 2)
  {
    command_sub_topic = "cmd_vel";
  }
  else
  {
    command_sub_topic = argv[1];
  }

  ros::Subscriber kinematics_control_sub = n.subscribe(command_sub_topic, 1000, cmdVelCallback);
  ros::Subscriber front_cart_sub = n.subscribe("/agribot/encoder/cart/front", 1000, encoderFrontCallback);
  ros::Subscriber back_cart_kinematics_sub = n.subscribe("/agribot/encoder/cart/back", 1000, encoderBackCallback);
  ros::Publisher kinematics_control_pub =
      n.advertise<agribot_msgs::AgribotCaterpillar>("agribot/control/caterpillar", 1);
  agribot_msgs::AgribotCaterpillar caterpilar_msg;
  ros::ServiceServer steering_mode_service =
      n.advertiseService("agribot/kinematics_controller/steering_mode", driving_mode_callback);

  while (ros::ok())
  {
    if (bDataReceived)
    {
      bDataReceived = false;
      KinControl.update();
      double* fpCaterpilarVelocities = KinControl.pdGetCaterpillarVelocities();
      caterpilar_msg.header.frame_id = "chassis_link";
      caterpilar_msg.header.stamp = ros::Time::now();
      caterpilar_msg.front_linear = fpCaterpilarVelocities[FRONT_LINEAR];
      caterpilar_msg.front_angular = fpCaterpilarVelocities[FRONT_ANGULAR];
      caterpilar_msg.back_linear = fpCaterpilarVelocities[BACK_LINEAR];
      caterpilar_msg.back_angular = fpCaterpilarVelocities[BACK_ANGULAR];
      kinematics_control_pub.publish(caterpilar_msg);
    }
    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}
