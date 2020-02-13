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
      std::vector<geometry_msgs::PoseStamped> _xPlan; // contains the global plan
      geometry_msgs::Twist _xVelocities; // contains the velocity
      visualization_msgs::Marker _traveledPoints;
      pos _pRobotCenter; // present robot center frame
      pos _pLastWaypoint; // last target frame
      int _iPlanLength; // number of frames in the global plan 
      int _iMissedWaypointsCount; // keeps track of the number of ommited frames
      int _iWaypointCount; // keeps track of the number for the next frame in the global plan
      bool _bGoalReached;
      bool _bInitialized;
      double _dStopTime, _dStartTime, _dLastTime, _dActualTime; // time measuring for statistics
      double _dDeviationIntegral, _dTraveledDistance; // distance and deviation measuring for statistics

      void vOdomCallback(const nav_msgs::Odometry odometry_msg);
      pos pCalculateRobotFrontPosition(pos robotCenter);
      pos pGetFrame(int count);
      pos pCalculateGoal(pos nextWaypoint, pos robotCenter);
      pos pCalculateGoalRange(pos nextWaypoint, pos robotCenter, double range);
      pos pCalculateProjection(pos lastWaypoint, pos nextWaypoint, pos robotCenter);
      geometry_msgs::Twist xSetVel(pos goal, pos nextWaypoint, pos robotCenter);
      geometry_msgs::Twist xArchApproach(pos goal, pos robotCenter, geometry_msgs::Twist velocity);
      geometry_msgs::Twist xDeviationApproach(pos goal, pos robotCenter, geometry_msgs::Twist velocity);
      geometry_msgs::Twist xLinearSpeedLimitedApproach(pos goal, pos robotCenter);
      geometry_msgs::Twist xLimitVelocities(geometry_msgs::Twist Velcoities);
      geometry_msgs::Twist xStopRobot();
      geometry_msgs::Twist xFinishedPathBehaviour();
      void vTakeNextGoal(pos currentGoal, pos nextWaypoint, pos robotCenter);
      void vCalculateDeviationIntegral(pos LastWaypoint, pos nextWaypoint, pos robotCenter);
      double dCloseToZero(double value);
      double dGetYawFromQuaternion(geometry_msgs::Quaternion quat);
      double dLimitAngle(double angle);
      double dAngleBetweenFrames(pos targetFrame, pos currentFrame);
      double dDistanceBetweenFrames(pos targetFrame, pos currentFrame);
      double dCalculateRadius(double distanceRobotWaypoint, double angleRobotWaypoint);
      double dCalculateDeviation(pos lastWaypoint, pos nextWaypoint, pos robotCenter);
      bool bMissedTooManyWaypointsCondition(int missedWaypoints);
      bool bWaypointHitCondition(pos robotCenter, pos lastWaypoint, pos nextWaypoint);
      bool bWaypointMissedCondition(pos robotCenter, pos lastWaypoint, pos nextWaypoint);
      bool bTraveledDistanceBetweenWaypoints(pos robotFront, pos lastWaypoint, pos nextWaypoint);
      bool bWithinGoalRange(pos robotFront, pos nextWaypoint);
      bool bIsLastWaypointInPath(int waypointsCount);
      bool bCenterTraveledBeyondGoal(pos robotCenter, pos lastWaypoint, pos nextWaypoint);
      bool bCenterOutOfGoalRange(pos robotCenter, pos nextWaypoint); 


      void vPathVisualization(pos robotCenter);
      void vPredictedPathVisualization(geometry_msgs::Twist velocity, pos robotCenter);
      void vNextGoalVisualization(pos lastWaypoint, pos goal);
      void vGoalRangeVisualization(pos goal, pos range);
      void vVisualizations(pos lastWaypoint, pos goal, pos nextWaypoint, pos robotCenter, geometry_msgs::Twist velocity);
  };
};

#endif

