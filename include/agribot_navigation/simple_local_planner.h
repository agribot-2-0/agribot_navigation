#ifndef SIMPLE_LOCAL_PLANNER_ROS_H_
#define SIMPLE_LOCAL_PLANNER_ROS_H_

#include <ros/ros.h>
#include <nav_core/base_local_planner.h>
#include <base_local_planner/goal_functions.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
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
      pos now; // present frame
      pos next; // next frame
      pos last; // last frame
      pos nError; // error between present and next frames
      pos nPath; // error between last and present frames
      pos nFramesDifference; // error between last and next frames
      double distance; //distance between last and next frames
      double previous_distance;
      double path_error; //distance between present and next frames
      double path_traveled; //distance between last and present frames
		  double deviation;
      int missed;
      int length; // number of frames in the global plan 
      int count; // keeps track of the number for the next frame in the global plan
      std::vector<geometry_msgs::PoseStamped> plan; // contains the global plan
      geometry_msgs::Twist cmd; // contains the velocity
      double stopTime, startTime;
      double error_integral, traveled_distance;
     //measuring
      bool hasStarted;
      // Flags
      bool goal_reached_;
      bool initialized_;
      // Velocity methods
      void setVel();
      void setRot();
      void setVelZeros();
      void odomCallback(const nav_msgs::Odometry odometry_msg);
      double getYaw(geometry_msgs::Quaternion quat);
      pos calculateFramesDifference(pos next, pos now);
      void setNowError();
      void setNext();
  };
};

#endif

