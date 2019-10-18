#include <ros/ros.h>
#include <ros/console.h>
#include <agribot_navigation/ReverseKinematics.h>
#include <agribot_msgs/AgribotCaterpillar.h>
#include <geometry_msgs/Twist.h>

enum{
    X,
    Y,
    Z,
};

enum {
    FRONT_LINEAR,
    FRONT_ANGULAR,
    BACK_LINEAR,
    BACK_ANGULAR
};

ReverseKinematics RevKinematics;

void cmdVelCallback(const agribot_msgs::AgribotCaterpillar msg){
    float[3] pfLinearVels;
    float[3] pfAngularVels;

    pfLinearVels[X]=cmd_msg
    pfLinearVels[Y]=cmd_msg
    pfLinearVels[Z]=cmd_msg

    pfAngularVels[X]=cmd_msg
    pfAngularVels[Y]=cmd_msg
    pfAngularVels[Z]=cmd_msg

    RevKinematics.vSetInputLinear(cmd_msg->linear.x, cmd_msg->linear.y, cmd_msg->linear.z)
    RevKinematics.vSetInputAngular(cmd_msg->angular.x, cmd_msg->angular.y, cmd_msg->angular.z)
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "reverse_kinematics");
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
    

    ros::SubscribeOptions so =
      ros::SubscribeOptions::create<geometry_msgs::Twist>(command_sub_topic, 1,
          boost::bind(&cmdVelCallback, this, _1),
          ros::VoidPtr(), &queue_);

    ros::Subscriber reverse_kinematics_sub = n->subscribe(so);
	ros::Publisher reverse_kinematics_pub = n.advertise<agribot_msgs::AgribotCaterpillar>("agribot/cmdvel",1);
    agribot_msgs::AgribotCaterpillar cmdvel_msg;

	

	// uint16_t u16TimeTrack = 0;
	// int iRet = 0;
	// iRet = Incli.configure(); 	
	// if(iRet == 0)
	// {
	// 	if(!Incli.startup())
	// 	{
	// 		ROS_ERROR("[Inclinometer] Communication timeout: Lack of response on startup");
	// 		return -1;
	// 	}
	// 	while(ros::ok())
	// 	{
	// 		if(Incli.update())
	// 		{
	// 			msg.header.frame_id = "/agribot/inclinometer";
	// 			msg.header.stamp = ros::Time::now();
	// 			msg.roll = Incli.fGetRoll();
	// 			msg.pitch = Incli.fGetPitch();
	// 			inclinometer_pub.publish(msg);
	// 			u16TimeTrack = 0;
	// 		}

	// 		if (u16TimeTrack > 20)
	// 		{
	// 			ROS_ERROR("[Inclinometer] Communication timeout: data not present");
	// 			return -1;
	// 		}
	// 		u16TimeTrack++;
	// 		ros::spinOnce();
	// 		loop_rate.sleep();
	// 	}
	// }
	// else
	// {
	// 	ROS_ERROR("[Inclinometer] Configuration failed with error code %d", iRet);
	// }
	return 0;
}
