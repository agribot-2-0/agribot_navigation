#include <ros/ros.h>
#include <ros/console.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/Point.h>
#include <std_msgs/Int32.h>
#include <nav_msgs/Odometry.h>
#include <tf/tf.h>
#include <queue>

#define PI 3.14159265
#define DEG_TO_RAD (PI/180)
#define RAD_TO_DEG (180/PI)

struct pos 
   {
	  double x, y, az;	
   };

int choice = -1;
bool bDataReceived=false;
std::queue<geometry_msgs::Point> plan;
pos robotPosition;

double getYaw(geometry_msgs::Quaternion quat)
{
    tf::Quaternion q(
        quat.x,
        quat.y,
        quat.z,
        quat.w);
    tf::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    return yaw;
}

void choiceCallback (const std_msgs::Int32 choice_msg)
{
	choice = choice_msg.data;
	bDataReceived=true;
}

void odomCallback(const nav_msgs::Odometry odometry_msg)
    {
        // ROS_INFO("Seq: [%d]", odometry_msg.header.seq);
        robotPosition.x = odometry_msg.pose.pose.position.x;
        robotPosition.y = odometry_msg.pose.pose.position.y;
        robotPosition.az = getYaw(odometry_msg.pose.pose.orientation); 
    }


void squareLeftPath()
{
    geometry_msgs::Point point;
    point.x = robotPosition.x + 0.0;
    point.y = robotPosition.y + 0.0;
    plan.push(point);
    point.x = robotPosition.x + 10.0;
    point.y = robotPosition.y + 0.0;
    plan.push(point);
    point.x = robotPosition.x + 10.0;
    point.y = robotPosition.y + 10.0;
    plan.push(point);
    point.x = robotPosition.x + 0.0;
    point.y = robotPosition.y + 10.0;
    plan.push(point);
    point.x = robotPosition.x + 0.0;
    point.y = robotPosition.y + 0.0;
    plan.push(point);
    point.x = robotPosition.x + 0.0;
    point.y = robotPosition.y + 0.0;
    plan.push(point);
}

void squareRightPath()
{
    geometry_msgs::Point point;
    point.x = robotPosition.x + 0.0;
    point.y = robotPosition.y + 0.0;
    plan.push(point);
    point.x = robotPosition.x + 10.0;
    point.y = robotPosition.y + 0.0;
    plan.push(point);
    point.x = robotPosition.x + 10.0;
    point.y = robotPosition.y - 10.0;
    plan.push(point);
    point.x = robotPosition.x + 0.0;
    point.y = robotPosition.y - 10.0;
    plan.push(point);
    point.x = robotPosition.x + 0.0;
    point.y = robotPosition.y + 0.0;
    plan.push(point);
    point.x = robotPosition.x + 0.0;
    point.y = robotPosition.y + 0.0;
    plan.push(point);
}

void straightForwardPath()
{
    geometry_msgs::Point point;
    point.x = robotPosition.x + 0;
    point.y = robotPosition.y + 0;
    plan.push(point);
    point.x = robotPosition.x + 2.5;
    point.y = robotPosition.y + 0;
    plan.push(point);
    point.x = robotPosition.x + 5.0;
    point.y = robotPosition.y + 0.0;
    plan.push(point);
    point.x = robotPosition.x + 7.5;
    point.y = robotPosition.y + 0;
    plan.push(point);
    point.x = robotPosition.x + 10.0;
    point.y = robotPosition.y + 0.0;
    plan.push(point);
    point.x = robotPosition.x + 10.0;
    point.y = robotPosition.y + 0.0;
    plan.push(point);
    
}

void makeNewPath (int choice)
{
    plan = std::queue<geometry_msgs::Point>();
    switch (choice)
    {
    case 1:
        straightForwardPath();
        break;
    case 2:
        squareLeftPath();
        break;
    case 3:
        squareRightPath();
        break;
    }
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "path_publisher");
	ros::NodeHandle n;
	ros::Rate loop_rate(100);
	std::string path_topic;

	if(argc < 2)
	{
		path_topic="/clicked_point";
	}
	else
	{
		path_topic=argv[1];
	}

    ros::Subscriber odom_sub = n.subscribe("/agribot/odometry", 100, odomCallback);
	ros::Subscriber path_choice_sub = n.subscribe("/path_publisher/choice", 100, choiceCallback);
	ros::Publisher path_pub = n.advertise<geometry_msgs::PointStamped>(path_topic,1);
    geometry_msgs::PointStamped point_msg;
    plan = std::queue<geometry_msgs::Point>();

	while(ros::ok())
	{
		if(bDataReceived)
		{
			bDataReceived=false;
            makeNewPath(choice);
		}
        if(!plan.empty())
        {
            point_msg.header.frame_id = "odom";
            point_msg.header.stamp = ros::Time::now();
            point_msg.point = plan.front();
            path_pub.publish(point_msg);
            plan.pop();
        }
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}
