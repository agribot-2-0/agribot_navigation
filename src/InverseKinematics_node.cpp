#include <ros/ros.h>
#include <ros/console.h>
#include <agribot_navigation/InverseKinematics.h>
#include <agribot_msgs/AgribotCaterpillar.h>
#include <geometry_msgs/Twist.h>
#define ROS_TIMEOUT 20

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

InverseKinematics RevKinematics;
bool bDataReceived=false;

void cmdVelCallback(const geometry_msgs::Twist cmd_msg)
{
    RevKinematics.vSetInputLinear(cmd_msg.linear.x, cmd_msg.linear.y, cmd_msg.linear.z);
    RevKinematics.vSetInputAngular(cmd_msg.angular.x, cmd_msg.angular.y, cmd_msg.angular.z);
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
	ros::Publisher inverse_kinematics_pub = n.advertise<agribot_msgs::AgribotCaterpillar>("agribot/control/caterpillar",1);
    agribot_msgs::AgribotCaterpillar caterpilar_msg;



	uint16_t u16TimeTrack = 0;
	while(ros::ok())
	{
		if(bDataReceived)
		{
			bDataReceived=false;
			RevKinematics.update();
			float fpCaterpilarVelocities [4];
			RevKinematics.vGetCaterpilareVelocities(fpCaterpilarVelocities);
			caterpilar_msg.header.frame_id = "/agribot/inclinometer";
			caterpilar_msg.header.stamp = ros::Time::now();
			caterpilar_msg.front_linear = fpCaterpilarVelocities[FRONT_LINEAR];
			caterpilar_msg.front_angular = fpCaterpilarVelocities[FRONT_ANGULAR];
			caterpilar_msg.back_linear = fpCaterpilarVelocities[BACK_LINEAR];
			caterpilar_msg.back_angular = fpCaterpilarVelocities[BACK_ANGULAR];
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
