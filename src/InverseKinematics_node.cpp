#include <ros/ros.h>
#include <ros/console.h>
#include <agribot_navigation/InverseKinematics.h>
#include <agribot_msgs/AgribotCaterpillar.h>
#include <agribot_msgs/AgribotHeading.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Float32.h>
#include <nav_msgs/Odometry.h>
#include <tf/tf.h>
#define ROS_TIMEOUT 20
#define PI 3.14159265359

enum
{
	X,
	Y,
	Z,
};
enum 
{
	FRONT_LINEAR,
	FRONT_ANGULAR,
	BACK_LINEAR,
	BACK_ANGULAR
};
enum {
	FRONT_CART,
	BACK_CART
};

InverseKinematics InvKinematics;
bool bDataReceived=false;

void cmdVelCallback(const geometry_msgs::Twist cmd_msg)
{
	InvKinematics.vSetInputLinear(cmd_msg.linear.x, cmd_msg.linear.y, cmd_msg.linear.z);
	InvKinematics.vSetInputAngular(cmd_msg.angular.x, cmd_msg.angular.y, cmd_msg.angular.z);
	bDataReceived=true;
}
void odometryCallback(const agribot_msgs::AgribotHeading odom_msg)
{
	InvKinematics.vSetChassisAngle(odom_msg.heading);
	bDataReceived=true;
}
void encoderFrontCallback(const std_msgs::Float32 front_cart_msg)
{
	InvKinematics.vSetFrontCartAngle(front_cart_msg.data*PI/180);
	bDataReceived=true;
}
void encoderBackCallback(const std_msgs::Float32 back_cart_msg)
{
	InvKinematics.vSetBackCartAngle(back_cart_msg.data*PI/180);
	bDataReceived=true;
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "inverse_kinematics");
	ros::NodeHandle n;
	ros::Rate loop_rate(100);
	std::string command_sub_topic;
	if(argc < 2)
	{
		command_sub_topic="cmd_vel";
	}
	else
	{
		command_sub_topic=argv[1];
	}
	ros::Subscriber inverse_kinematics_sub = n.subscribe(command_sub_topic, 1000, cmdVelCallback);
	ros::Subscriber odometry_sub = n.subscribe("/agribot/heading", 1000, odometryCallback);
	ros::Subscriber front_cart_sub = n.subscribe("/agribot/encoder/cart/front", 1000, encoderFrontCallback);
	ros::Subscriber back_cart_kinematics_sub = n.subscribe("/agribot/encoder/cart/back", 1000, encoderBackCallback);
	ros::Publisher inverse_kinematics_pub = n.advertise<agribot_msgs::AgribotCaterpillar>("agribot/control/caterpillar",1);
	agribot_msgs::AgribotCaterpillar caterpilar_msg;
	uint16_t u16TimeTrack = 0;
	while(ros::ok())
	{
		if(bDataReceived)
		{
			bDataReceived=false;
			InvKinematics.update();
			double *fpCaterpilarVelocities=InvKinematics.pdGetCaterpillarVelocities();
			caterpilar_msg.header.frame_id = "agribot/control/caterpillar";
			caterpilar_msg.header.stamp = ros::Time::now();
			caterpilar_msg.front_linear = fpCaterpilarVelocities[FRONT_LINEAR];
			caterpilar_msg.front_angular = fpCaterpilarVelocities[FRONT_ANGULAR]*180/PI;
			caterpilar_msg.back_linear = fpCaterpilarVelocities[BACK_LINEAR];
			caterpilar_msg.back_angular = fpCaterpilarVelocities[BACK_ANGULAR]*180/PI;
			inverse_kinematics_pub.publish(caterpilar_msg);
			u16TimeTrack = 0;
		}
		// if (u16TimeTrack > ROS_TIMEOUT)
		// {
		// 	ROS_ERROR("[InverseKinematics] Communication timeout: data not present");
		// 	return -1;
		// }
		u16TimeTrack++;
		ros::spinOnce();
		loop_rate.sleep();
	}
	return 0;
}
