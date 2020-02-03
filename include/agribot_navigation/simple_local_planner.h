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
      costmap_2d::Costmap2DROS* _xCostmapROS; 
      tf2_ros::Buffer* _xTFBuffer;
      tf2_ros::TransformListener* _xTFListener;
      ros::Subscriber _rOdomSub;
      ros::Publisher _rPathPub;
      ros::Publisher _rPredictedPathPub; 
      ros::Publisher _rGoalRangePub; 
      ros::Publisher _rNextGoalPub;
      std::vector<geometry_msgs::PoseStamped> _vPlan; // contains the global plan
      geometry_msgs::Twist _xVelocities; // contains the velocity
      visualization_msgs::Marker _traveledPoints;
      pos _pRobotCenter; // present robot center frame
      pos _pRobotFront; // present robot front frame
      pos _pNextWaypoint; // next target frame
      pos _pLastWaypoint; // last target frame
      pos _pFollowedGoal; // followed goal frame
      pos _pGoalRange; // next target range
      double _dAgleRobotNextWaypoint;
      double _dAngleRobotFollowedGoal;
      double _dDistanceWaypoints; //distance between last target and next target frames
      double _dDistanceToNextWaypoint; //distance between present robot and next target frames
      double _dDistanceToNextWaypointFront; //distance between present robot front and next target frames
      double _dDistanceFromLastWaypoint; //distance between last target and present robot frames
      double _dDeviation; // distance form robot to vector from last to next frame
      double _dStopTime, _dStartTime, _dLastTime, _dActualTime; // time measuring for statistics
      double _dDeviationIntegral, _dTraveledDistance; // distance and deviation measuring for statistics
      int _iPlanLength; // number of frames in the global plan 
      int _iMissedWaypointsCount; // keeps track of the number of ommited frames
      int _iCount; // keeps track of the number for the next frame in the global plan
      bool _bGoalReached;
      bool _bInitialized;
      bool _bMoving;


      void vOdomCallback(const nav_msgs::Odometry odometry_msg);
      pos pCalculateRobotFrontPosition(pos robotCenter);
      pos pGetFrame(int count);
      pos pTakeNextGoal();
      pos pCalculateGoal(pos nextWaypoint, pos robotCenter);
      geometry_msgs::Twist xSetVel();
      geometry_msgs::Twist xStopRobot();
      geometry_msgs::Twist xLimitVelocities(geometry_msgs::Twist Velcoities);
      double dGetYawFromQuaternion(geometry_msgs::Quaternion quat);
      double dLimitAngle(double angle);
      double dAngleBetweenFrames(pos targetFrame, pos currentFrame);
      double dDistanceBetweenFrames(pos targetFrame, pos currentFrame);
      double dLinearSpeedLimiter();
      double dArchApproach(pos goal, pos robotCenter);
      double dDeviationApproach(pos goal, pos robotCenter);
      double dCalculateRadius(double distanceRobotWaypoint, double angleRobotWaypoint);
      double dCalculateDeviation(double distance, double remaining, double traveled);
      void vCalculateErrors();
      geometry_msgs::Twist xMovementBehaviour();
      void vMissedWaypointsBehaviour();
      void vCalculateDeviationIntegral();

      void vPathVisualization();
      void vPredictedPathVisualization();
      void vNextGoalVisualization(pos goal);
      void vGoalRangeVisualization();
      void vVisualizations();
  };
};

#endif

