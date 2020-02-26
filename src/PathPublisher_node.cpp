#include <ros/ros.h>
#include <ros/console.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/Point.h>
#include <std_msgs/Int32.h>
#include <nav_msgs/Odometry.h>
#include <tf/tf.h>
#include <queue>
#include <cmath>

#define DEG_TO_RAD (M_PI/180)
#define RAD_TO_DEG (180/M_PI)
#define SQUARE_SIDE_LENGTH 10.0

struct pos
{
     double x, y, az;
};

int choice = -1;
bool bDataReceived = false;
double side_length = 10;
std::queue < geometry_msgs::Point> plan;
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

void choiceCallback(const std_msgs::Int32 choice_msg)
{
    choice = choice_msg.data;
    bDataReceived = true;
}

void odomCallback(const nav_msgs::Odometry odometry_msg)
{
    robotPosition.x = odometry_msg.pose.pose.position.x;
    robotPosition.y = odometry_msg.pose.pose.position.y;
    robotPosition.az = getYaw(odometry_msg.pose.pose.orientation);
}

void polygonCCWPath(double radius, int edges, int max_circle_deg,
                    pos startingPoint, bool finish)
{
    geometry_msgs::Point first_point, point, tmp;
    // pos startingPoint = robotPosition;
    double pivot_x, pivot_y;

    // finding the center of the circumscribed circle
    pivot_x = radius*cos(2*(90*(edges-2)/edges)*DEG_TO_RAD);
    pivot_y = radius*sin(2*(90*(edges-2)/edges)*DEG_TO_RAD);

    for (double theta = 0; theta <= max_circle_deg; theta += max_circle_deg/edges)
    {
        // finding verticles on circumscribed circle
        tmp.x = radius*cos(theta*DEG_TO_RAD+2*(90*(edges-2)/edges)*DEG_TO_RAD);
        tmp.y = radius*sin(theta*DEG_TO_RAD+2*(90*(edges-2)/edges)*DEG_TO_RAD);

        tmp.x -= pivot_x;
        tmp.y -= pivot_y;

        // rotating the polygon to match robot orientation
        point.x = tmp.x * cos(-(90*(edges-2)/edges)*DEG_TO_RAD+robotPosition.az) -
        tmp.y * sin(-(90*(edges-2)/edges)*DEG_TO_RAD+robotPosition.az);
        point.y = tmp.x * sin(-(90*(edges-2)/edges)*DEG_TO_RAD+robotPosition.az) +
        tmp.y * cos(-(90*(edges-2)/edges)*DEG_TO_RAD+robotPosition.az);

        // first edge is in front of robot and following edges go counterclockwise
        point.x = -(pivot_x + point.x);
        point.y = -(pivot_y + point.y);

        // moving the begining of polygon to robot position
        if (theta == 0)
        {
            first_point = point;
        }
        point.x -= (first_point.x - startingPoint.x);
        point.y -= (first_point.y - startingPoint.y);
        plan.push(point);
    }
    // last point published twice to finish the path
    if (finish)
    {
      plan.push(point);
    }
}

void straightForwardPath(double segment_length, int segments, pos startingPoint,
                          bool finish)
{
    geometry_msgs::Point point;
    // pos startingPoint = robotPosition;
    point.x = startingPoint.x;
    point.y = startingPoint.y;
    plan.push(point);
    for (int i = 0; i < segments; i++)
    {
        point.x += segment_length*cos(startingPoint.az);
        point.y += segment_length*sin(startingPoint.az);
        plan.push(point);
    }
    // last point published twice to finish the path
    if (finish)
    {
      plan.push(point);
    }
}

void makeNewPath(int choice)
{
    plan = std::queue < geometry_msgs::Point>();

    if (choice == 123)
    {
        double move_straight_distance = 5;
        double turn_radius_distance = 4;
        pos temp = {move_straight_distance, 0, 0};
        straightForwardPath(move_straight_distance, 1, robotPosition, false);
        polygonCCWPath(turn_radius_distance, 11, 200, temp, false);
        temp = {move_straight_distance, 2*turn_radius_distance, 0};
        straightForwardPath(-move_straight_distance, 1, temp, true);
    }
    else if (choice < 2)
    {
        polygonCCWPath((side_length/(2*sin(M_PI/-choice))), -choice, 360,
                        robotPosition, true);
    }
    else if (choice > 0)
    {
        side_length = choice / 10.0;
    }
    else
    {
        straightForwardPath(side_length, 4, robotPosition, true);
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "path_publisher");
    ros::NodeHandle n;
    ros::Rate loop_rate(100);
    std::string path_topic;

    if (argc < 2)
    {
        path_topic = "/clicked_point";
    }
    else
    {
        path_topic = argv[1];
    }

    ros::Subscriber odom_sub = n.subscribe("/agribot/odometry", 100, odomCallback);
    ros::Subscriber path_choice_sub = n.subscribe("/path_publisher/choice", 100, choiceCallback);
    ros::Publisher path_pub = n.advertise < geometry_msgs::PointStamped>(path_topic, 1);
    geometry_msgs::PointStamped point_msg;
    plan = std::queue < geometry_msgs::Point>();

    while (ros::ok())
    {
        if (bDataReceived)
        {
            bDataReceived = false;
            makeNewPath(choice);
        }
        if (!plan.empty())
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
