/*
 * Copyright (C) 2020 Agribot. All rights reserved.
 */

#include <agribot_common/CommonFunctions/ros_common_functions.h>
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
  KinControl.setInputLinear(cmd_msg.linear.x, cmd_msg.linear.y, cmd_msg.linear.z);
  KinControl.setInputAngular(cmd_msg.angular.x, cmd_msg.angular.y, cmd_msg.angular.z);
  bDataReceived = true;
}

void encoderFrontCallback(const std_msgs::Float32 front_cart_msg)
{
  KinControl.setFrontCartAngle(front_cart_msg.data);
  bDataReceived = true;
}

void encoderBackCallback(const std_msgs::Float32 back_cart_msg)
{
  KinControl.setBackCartAngle(back_cart_msg.data);
  bDataReceived = true;
}

void callback(agribot_navigation::KinematicsConfig& config,  // NOLINT(runtime/references)
              uint32_t level)
{
  static_cast<void>(level);
  KinControl.setFrontKp(config.front_Kp);
  KinControl.setFrontKi(config.front_Ki);
  KinControl.setFrontKd(config.front_Kd);
  KinControl.setBackKp(config.back_Kp);
  KinControl.setBackKi(config.back_Ki);
  KinControl.setBackKd(config.back_Kd);
  KinControl.setFrontLimit(config.precision_range);
  KinControl.setBackLimit(config.front_limit);
  KinControl.setPrecisionRange(config.back_limit);
}

bool steering_mode_callback(agribot_msgs::SteeringMode::Request& request,    // NOLINT(runtime/references)
                            agribot_msgs::SteeringMode::Response& response)  // NOLINT(runtime/references)
{
  if (request.steering_mode == agribot_msgs::SteeringMode::Request::FOUR_WHEEL)
  {
    ROS_WARN_STREAM("[KinematicsController] Switched to FOUR WHEEL steering mode");
    KinControl.setSteeringMode("four_wheel");
    response.result = true;
  }
  else if (request.steering_mode == agribot_msgs::SteeringMode::Request::FRONT_WHEEL)
  {
    ROS_WARN_STREAM("[KinematicsController] Switched to FRONT WHEEL steering mode");
    KinControl.setSteeringMode("front_wheel");
    response.result = true;
  }
  else if (request.steering_mode == agribot_msgs::SteeringMode::Request::CRAB)
  {
    ROS_WARN_STREAM("[KinematicsController] Switched to CRAB steering mode");
    KinControl.setSteeringMode("crab");
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
  ros::NodeHandle pnh("~");

  try
  {
    double front_kp = ros_common_functions::getRosParam<double>(pnh.getNamespace() + "/front_Kp");
    double front_ki = ros_common_functions::getRosParam<double>(pnh.getNamespace() + "/front_Ki");
    double front_kd = ros_common_functions::getRosParam<double>(pnh.getNamespace() + "/front_Kd");
    double back_kp = ros_common_functions::getRosParam<double>(pnh.getNamespace() + "/back_Kp");
    double back_kd = ros_common_functions::getRosParam<double>(pnh.getNamespace() + "/back_Ki");
    double back_ki = ros_common_functions::getRosParam<double>(pnh.getNamespace() + "/back_Kd");
    double precision_range = ros_common_functions::getRosParam<double>(pnh.getNamespace() + "/precision_range");
    double front_limit = ros_common_functions::getRosParam<double>(pnh.getNamespace() + "/front_limit");
    double back_limit = ros_common_functions::getRosParam<double>(pnh.getNamespace() + "/back_limit");
    std::string steering_mode = ros_common_functions::getRosParam<std::string>(pnh.getNamespace() + "/steering_mode");

    KinControl.setFrontKp(front_kp);
    KinControl.setFrontKi(front_kd);
    KinControl.setFrontKd(front_ki);
    KinControl.setBackKp(back_kp);
    KinControl.setBackKi(back_kd);
    KinControl.setBackKd(back_ki);
    KinControl.setFrontLimit(precision_range);
    KinControl.setBackLimit(front_limit);
    KinControl.setPrecisionRange(back_limit);
    KinControl.setSteeringMode(steering_mode);
  }
  catch (const std::exception& ex)
  {
    ROS_ERROR_STREAM("[KinematicsController] " << ex.what());
    return 0;
  }

  KinControl.createPubs(n);
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
      n.advertiseService("agribot/kinematics_controller/steering_mode", steering_mode_callback);

  while (ros::ok())
  {
    if (bDataReceived)
    {
      bDataReceived = false;
      KinControl.update();
      double* fpCaterpilarVelocities = KinControl.getCaterpillarVelocities();
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
