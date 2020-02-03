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
      pos _pRobotCenter; // present robot center frame
      pos _pRobotFront; // present robot front frame
      pos _pNextWaypoint; // next target frame
      pos _pLastWaypoint; // last target frame
      pos _pFollowedGoal; // followed goal frame
      pos _pGoalRange; // next target range
      pos _pDifferencRobotCenterAndNextWaypoint; // error between present robot and next target frames
      pos _pDifferencRobotFrontAndNextWaypoint; // error between present robot front and next target frames
      pos _pDifferencRobotCenterAndLastWaypoint; // error between last target and present robot frames
      pos _pDifferencNextWaypointAndLastWaypoint; // error between last target and next target frames
      pos _pDifferencFollowedGoalAndRobotCenter; // error between present target ang followed goal frame
      double _dDistanceFromLastToNextWaypoint; //distance between last target and next target frames
      double _dDistanceFromRobotCenterToNextWaypoint; //distance between present robot and next target frames
      double _dDistanceFromRobotFrontToNextWaypoint; //distance between present robot front and next target frames
      double _dDistanceFromLastWaypointToRobotCenter; //distance between last target and present robot frames
      double _dRobotDeviationFromLineBetweenWaypoints; // distance form robot to vector from last to next frame
      double _dStopTime, _dStartTime, _dLastTime, _dActualTime; // time measuring for statistics
      double _dDeviationIntegral, _dTraveledDistance; // distance and deviation measuring for statistics
      int _iLength; // number of frames in the global plan 
      int _iMissed; // keeps track of the number of ommited frames
      int _iCount; // keeps track of the number for the next frame in the global plan
      bool _bGoalReached;
      bool _bInitialized;
      bool _bStartedMoving;

      void vOdomCallback(const nav_msgs::Odometry odometry_msg);
      void vSetVel();
      void vStopRobot();
      void vCalculateErrors();
      void vCalculateGoal();
      pos pCalculateFramesDifference(pos next, pos now);
      pos pGetFrame(int count);
      double dGetYawFromQuaternion(geometry_msgs::Quaternion quat);
      double dLimitAngle(double angle);

      void vPathVisualization();
      void vPredictedPathVisualization();
      void vNextGoalVisualization(pos goal);
      void vGoalRangeVisualization();
  };
};

#endif

