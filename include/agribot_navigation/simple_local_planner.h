#ifndef SIMPLE_LOCAL_PLANNER_ROS_H_
#define SIMPLE_LOCAL_PLANNER_ROS_H_

#include <ros/ros.h>
#include <nav_core/base_local_planner.h>
#include <base_local_planner/goal_functions.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>
#include <tf/tf.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <vector>
#define PI 3.14159265
#define DEG_TO_RAD (PI/180)
#define RAD_TO_DEG (180/PI)

namespace simple_local_planner{
   struct pos 
   {
    double x, y, az;	
   };

  class SimplePlannerROS : public nav_core::BaseLocalPlanner{
    public:
      SimplePlannerROS();
      ~SimplePlannerROS();
      void initialize(std::string name, tf2_ros::Buffer* tfBuffer,
        costmap_2d::Costmap2DROS* costmapROS);
      bool setPlan(const std::vector<geometry_msgs::PoseStamped>& orig_global_plan);
      bool computeVelocityCommands(geometry_msgs::Twist& cmd_vel);
      bool isGoalReached();
    private:
      costmap_2d::Costmap2DROS* costmap_ros_; 
      tf2_ros::Buffer* tfBuffer_;
      tf2_ros::TransformListener* tfListener_;
      ros::Subscriber odom_sub;
      ros::Publisher path_pub;
      ros::Publisher predicted_path_pub; 
      ros::Publisher goal_range_pub; 
      ros::Publisher next_goal_pub;
      std::vector<geometry_msgs::PoseStamped> plan; // contains the global plan
      geometry_msgs::Twist cmd; // contains the velocity
      visualization_msgs::Marker traveledPoints;
      pos now; // present robot center frame
      pos nowFront; // present robot front frame
      pos next; // next target frame
      pos last; // last target frame
      pos goal; // followed goal frame
      pos range; // next target range
      pos nError; // error between present robot and next target frames
      pos nFrontError; // error between present robot front and next target frames
      pos nPath; // error between last target and present robot frames
      pos nFramesDifference; // error between last target and next target frames
      pos nGoal; // error between present target ang followed goal frame
      double distance; //distance between last target and next target frames
      double path_error; //distance between present robot and next target frames
      double path_error_front; //distance between present robot front and next target frames
      double path_traveled; //distance between last target and present robot frames
      double deviation; // distance form robot to vector from last to next frame
      double stopTime, startTime, lastTime, actualTime; // time measuring for statistics
      double errorIntegral, traveledDistance; // distance and deviation measuring for statistics
      int length; // number of frames in the global plan 
      int missed; // keeps track of the number of ommited frames
      int count; // keeps track of the number for the next frame in the global plan
      bool goal_reached_;
      bool initialized_;
      bool started_moving_;

      void odomCallback(const nav_msgs::Odometry odometry_msg);
      void setVel();
      void setVelZeros();
      void calculateErrors();
      void calculateGoal();
      pos calculateFramesDifference(pos next, pos now);
      pos getFrame(int count);
      double getYaw(geometry_msgs::Quaternion quat);
      double limitAngle(double angle);

      void pathVisualization();
      void predictedPathVisualization();
      void nextGoalVisualization(pos goal);
      void goalRangeVisualization();
  };
};

#endif

