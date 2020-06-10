#include <ros/ros.h>
#include <ros/console.h>
#include <agribot_navigation/KinematicsControl.h>
#include <agribot_msgs/AgribotCaterpillar.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Float32.h>
#include <sensor_msgs/Imu.h>
#include <nav_msgs/Odometry.h>
#include <tf/tf.h>

#define ROS_TIMEOUT 20
#define PI 3.14159265359
#define RAD_TO_DEG (180/PI)
#define DEG_TO_RAD (PI/180)

KinematicsControl KinControl;
bool bDataReceived = false;

void cmdVelCallback(const geometry_msgs::Twist cmd_msg)
{
    KinControl.vSetInputLinear(
        cmd_msg.linear.x, cmd_msg.linear.y, cmd_msg.linear.z);
    KinControl.vSetInputAngular(
        cmd_msg.angular.x, cmd_msg.angular.y, cmd_msg.angular.z);
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

int main(int argc, char **argv)
{
    ros::init(argc, argv, "kinematics_control");
    ros::NodeHandle n;
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

    ros::Subscriber kinematics_control_sub = n.subscribe(
        command_sub_topic, 1000, cmdVelCallback);
    ros::Subscriber front_cart_sub = n.subscribe(
        "/agribot/encoder/cart/front", 1000, encoderFrontCallback);
    ros::Subscriber back_cart_kinematics_sub = n.subscribe(
        "/agribot/encoder/cart/back", 1000, encoderBackCallback);
    ros::Publisher kinematics_control_pub = n.advertise<
        agribot_msgs::AgribotCaterpillar>("agribot/control/caterpillar", 1);
    agribot_msgs::AgribotCaterpillar caterpilar_msg;

    while (ros::ok())
    {
        if (bDataReceived)
        {
            bDataReceived = false;
            KinControl.update();
            double *fpCaterpilarVelocities =
                KinControl.pdGetCaterpillarVelocities();
            caterpilar_msg.header.frame_id = "chassis_link";
            caterpilar_msg.header.stamp = ros::Time::now();
            caterpilar_msg.front_linear =
                fpCaterpilarVelocities[FRONT_LINEAR];
            caterpilar_msg.front_angular =
                fpCaterpilarVelocities[FRONT_ANGULAR];
            caterpilar_msg.back_linear =
                fpCaterpilarVelocities[BACK_LINEAR];
            caterpilar_msg.back_angular =
                fpCaterpilarVelocities[BACK_ANGULAR];
            kinematics_control_pub.publish(caterpilar_msg);
        }
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}
