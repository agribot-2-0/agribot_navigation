#include "agribot_navigation/simple_local_planner.h"

#include <pluginlib/class_list_macros.h>
#include <cmath>

#define GOAL_DISTANCE_TOLERANCE 5
#define MISSED_POINTS_TOLLERANCE 2

#define CHASSIS_LENGTH 2.1
#define MIN_RADIUS 2

#define PREDICTION_TIME 60
#define PREDICTION_POINTS_PER_SECOND 2

#define DEAD_ZONE 1
#define MAX_LINEAR_VEL 0.5

#define DEVIATION_WEIGHT 1
#define PARALLELNESS_WEIGHT 1

// Planner approach choice
// #define ARCH_APPROACH
// #define SPEED_LIMIT

// #define DEBUG
// #define STATISTICS

PLUGINLIB_EXPORT_CLASS(simple_local_planner::SimplePlannerROS, nav_core::BaseLocalPlanner)

namespace simple_local_planner
{

    SimplePlannerROS::SimplePlannerROS() : _xCostmapROS(NULL), _bInitialized(false) {}

    SimplePlannerROS::~SimplePlannerROS() {}

    void SimplePlannerROS::initialize(std::string name, tf2_ros::Buffer* tfBuffer, costmap_2d::Costmap2DROS* costmapROS)
    {
        if(!_bInitialized)
        {
            _xCostmapROS = costmapROS;
            _xTFBuffer = tfBuffer;
            _xTFListener = new tf2_ros::TransformListener(*tfBuffer);

            ros::NodeHandle gn;
            _rOdomSub = gn.subscribe("/agribot/odometry", 100, &SimplePlannerROS::vOdomCallback, this);
            _rPathPub = gn.advertise<visualization_msgs::Marker>("travelled_path", 10);
            _rPredictedPathPub = gn.advertise<visualization_msgs::Marker>("predicted_path", 10);
            _rGoalRangePub = gn.advertise<visualization_msgs::Marker>("goal_range", 10);
            _rNextGoalPub = gn.advertise<visualization_msgs::Marker>("next_goal", 10);

            _bInitialized = true;

            _pFollowedGoal.x = GOAL_DISTANCE_TOLERANCE/2;
            _pFollowedGoal.y = GOAL_DISTANCE_TOLERANCE/2;

            _bMoving = false;
            #ifdef STATISTICS
                _dStopTime = 0;
                _dStartTime = 0;
                _dLastTime = 0;
                _dActualTime = 0;
            #endif

            ROS_DEBUG("Simple Local Planner plugin initialized.");
        }
        else
        {
            ROS_WARN("This planner has already been initialized, doing nothing.");
        }
    }

    bool SimplePlannerROS::setPlan(const std::vector<geometry_msgs::PoseStamped>& orig_global_plan)
    {
        if(!_bInitialized)
        {
            ROS_ERROR("This planner has not been initialized, please call initialize() before using this planner");
            return false;
        }
        _iCount = 1;
        _iMissedWaypointsCount = 0; 
        _vPlan = orig_global_plan; 
        _iPlanLength = _vPlan.size(); 
        _bGoalReached = false;
        _traveledPoints=visualization_msgs::Marker();

        _pLastWaypoint = _pRobotCenter;
        _pNextWaypoint = pGetFrame(_iCount); 

        _bMoving = true;
        #ifdef STATISTICS
            _dDeviationIntegral = 0;
            _dTraveledDistance = 0;

            _dStartTime = ros::Time::now().toSec();
            _dLastTime = 0;
            _dActualTime = 0;
        #endif

        ROS_INFO("Received new plan, plan length = %d", _iPlanLength);
        return true;
    }

    bool SimplePlannerROS::computeVelocityCommands(geometry_msgs::Twist& cmd_vel)
    {
        if(!_bInitialized)
        {
            ROS_ERROR("This planner has not been initialized, please call initialize() before using this planner");
            return false;
        }
        
        if(_iPlanLength != 0)
        { 
            // Check where robot is
            _pFollowedGoal = pCalculateGoal(_pNextWaypoint, _pRobotCenter);
            vCalculateErrors();

            // Deviation error for statistics
            #ifdef STATISTICS
                vCalculateDeviationIntegral();
            #endif

            // Debug info about robot location in waypoints frames
            #ifdef DEBUG
                ROS_INFO("count %d _iMissedWaypointsCount %d _dDistanceWaypoints %f traveled %f error %f _dDeviation %f parallelness %f", _iCount, _iMissedWaypointsCount, _dDistanceWaypoints, _dDistanceFromLastWaypoint, _dDistanceToNextWaypoint, _dDeviation, cos(_pLastWaypoint.az-_pRobotCenter.az));
            #endif

            // Determine movement behaviour basing on location
            _xVelocities = xMovementBehaviour();
            
            // Behaviour after missing too many points
            if(_iMissedWaypointsCount > MISSED_POINTS_TOLLERANCE)
            {
                vMissedWaypointsBehaviour();
            }

            // Visualization
            vVisualizations();
        }
        cmd_vel = _xVelocities;  
        return true;
    }

    geometry_msgs::Twist SimplePlannerROS::xMovementBehaviour()
    {
        if (_dDistanceFromLastWaypoint >= _dDistanceWaypoints - CHASSIS_LENGTH / 2)
        {
            // Robot is within error distance from goal waypoint
            if (_dDistanceToNextWaypointFront < GOAL_DISTANCE_TOLERANCE)
            {
                if (_iCount < (_iPlanLength - 1))
                {
                    // It is not the last point of the path
                    _pNextWaypoint = pTakeNextGoal();
                    return xSetVel();
                }
                else
                {
                    // Reached final destionation
                    _bMoving = false;
                    _bGoalReached = true;
                    #ifdef STATISTICS
                    _dStopTime = ros::Time::now().toSec();
                    ROS_INFO("GOAL REACHED!    Time = %f s    Traveled  %f    Error = %f", _dStopTime - _dStartTime, _dTraveledDistance, _dDeviationIntegral);
                    #endif
                    return xStopRobot();
                }
            }
        }
        else if (_dDistanceFromLastWaypoint >= _dDistanceWaypoints)
        {
            // Traveled more than distance between last and next waypoints
            if (_dDistanceToNextWaypoint > GOAL_DISTANCE_TOLERANCE)
            {
                // Didn't hit the goal waypoint -> skip it and go to next one
                ROS_ERROR("OUT OF PATH!");
                _iMissedWaypointsCount++;
                _pNextWaypoint = pTakeNextGoal();
                return xSetVel();
            }
        }
        else
        {
            // Somewhere between last and next waypoint
            return xSetVel();
        }
    }

    pos SimplePlannerROS::pTakeNextGoal()
    {
        _iCount++;
        _pLastWaypoint = _pFollowedGoal;
        return pGetFrame(_iCount);
    }

    void SimplePlannerROS::vCalculateDeviationIntegral()
    {
        if(_dActualTime > 0)
        {
            if (!isnan(_dDeviation))
            {
                _dLastTime = _dActualTime;
                _dActualTime = ros::Time::now().toSec();
                _dDeviationIntegral += fabs(_dDeviation)*(_dActualTime-_dLastTime);
            }
        }
        else
        {
            _dActualTime = ros::Time::now().toSec();
        }
    }

    void SimplePlannerROS::vMissedWaypointsBehaviour()
    {
        _xVelocities = xStopRobot();
        _bMoving = false;
        _iPlanLength = 0;
        #ifdef STATISTICS
            _dStopTime = ros::Time::now().toSec();
            ROS_ERROR("STOPPING, ROBOT MISSED %d POINTS Time = %f    Traveled  %f", _iMissedWaypointsCount, _dStopTime-_dStartTime,_dTraveledDistance);
        #else
            ROS_ERROR("STOPPING, ROBOT MISSED %d POINTS", _iMissedWaypointsCount);
        #endif
    }

    bool SimplePlannerROS::isGoalReached()
    {
        if(!_bInitialized)
        {
            ROS_ERROR("This planner has not been initialized, please call initialize() before using this planner");
            return false;
        }
        return _bGoalReached;
    }

    void SimplePlannerROS::vOdomCallback(const nav_msgs::Odometry odometry_msg)
    {
        pos lastRobotPosition = _pRobotCenter;

        _pRobotCenter.x = odometry_msg.pose.pose.position.x;
        _pRobotCenter.y = odometry_msg.pose.pose.position.y;
        _pRobotCenter.az = dGetYawFromQuaternion(odometry_msg.pose.pose.orientation); 

        _pRobotFront = pCalculateRobotFrontPosition(_pRobotCenter);

        if (_bMoving)
        {
            #ifdef STATISTICS
                _dTraveledDistance += std::sqrt((lastRobotPosition.x-_pRobotCenter.x)*(lastRobotPosition.x-_pRobotCenter.x) + (lastRobotPosition.y-_pRobotCenter.y)*(lastRobotPosition.y-_pRobotCenter.y));
            #endif
        }
    }

    pos SimplePlannerROS::pCalculateRobotFrontPosition(pos robotCenter)
    {
        pos frontPos;
        frontPos.x = robotCenter.x + cos(robotCenter.az) * CHASSIS_LENGTH/2;
        frontPos.y = robotCenter.y + sin(robotCenter.az) * CHASSIS_LENGTH/2;
        frontPos.az = robotCenter.az;
        return frontPos;
    }

    double SimplePlannerROS::dGetYawFromQuaternion(geometry_msgs::Quaternion quat)
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

    geometry_msgs::Twist SimplePlannerROS::xSetVel()
    {
        geometry_msgs::Twist velocities;
        #ifdef SPEED_LIMIT
            velocities.linear.x = dLinearSpeedLimiter();
        #else
            velocities.linear.x = MAX_LINEAR_VEL;
        #endif
        
        #ifdef ARCH_APPROACH
            velocities.angular.z = dArchApproach(_pFollowedGoal, _pRobotCenter);
        #else
            velocities.angular.z = dDeviationApproach(_pFollowedGoal, _pRobotCenter);
        #endif

        // Velocity limiter
        return xLimitVelocities(velocities); 
    }

    geometry_msgs::Twist SimplePlannerROS::xLimitVelocities(geometry_msgs::Twist Velcoities)
    {
        if (Velcoities.linear.x > MAX_LINEAR_VEL)
        {
            Velcoities.linear.x = MAX_LINEAR_VEL;
        } 
        if (Velcoities.angular.z > Velcoities.linear.x / MIN_RADIUS)
        {
            Velcoities.angular.z = Velcoities.linear.x / MIN_RADIUS;
        }
        if (Velcoities.angular.z < -Velcoities.linear.x / MIN_RADIUS)
        {
            Velcoities.angular.z = -Velcoities.linear.x / MIN_RADIUS;
        }
        return Velcoities;
    }

    double SimplePlannerROS::dDeviationApproach(pos goal, pos robotCenter)
    {
        double angleRobotNextWaypoint = dLimitAngle(dAngleBetweenFrames(goal, robotCenter) - robotCenter.az);
        if (angleRobotNextWaypoint > DEAD_ZONE*DEG_TO_RAD)
            {
                return (DEVIATION_WEIGHT*_dDeviation/(1+_dDeviation)+
                        PARALLELNESS_WEIGHT*(1-fabs(cos(_pLastWaypoint.az-_pRobotCenter.az))))*
                        MAX_LINEAR_VEL / MIN_RADIUS;
            }
            else if (angleRobotNextWaypoint < -DEAD_ZONE*DEG_TO_RAD)
            {
                return -(DEVIATION_WEIGHT*_dDeviation/(1+_dDeviation)+
                         PARALLELNESS_WEIGHT*(1-fabs(cos(_pLastWaypoint.az-_pRobotCenter.az))))*
                         MAX_LINEAR_VEL / MIN_RADIUS;
            }
            else
            {
                return 0;
            }
    }

    double SimplePlannerROS::dLinearSpeedLimiter()
    {
        double deviationFactor, distanceFactor, multiplier;
        deviationFactor = 1 / (1 + _dDeviation);
        distanceFactor = 1 - fabs((_dDistanceWaypoints - _dDistanceToNextWaypointFront) / _dDistanceWaypoints);
        multiplier = (1 - deviationFactor) * (1 - distanceFactor);
        return MAX_LINEAR_VEL*(deviationFactor * (1 - multiplier));
    }

    double SimplePlannerROS::dArchApproach(pos goal, pos robotCenter)
    {
        double angleRobotGoal =  dLimitAngle(dAngleBetweenFrames(goal, robotCenter)-robotCenter.az);
        double distanceRobotGoal = dDistanceBetweenFrames(goal, robotCenter);

        if (fabs(angleRobotGoal) > DEAD_ZONE*DEG_TO_RAD)
        {
            // Compute arch only outside of dead zone
            if (angleRobotGoal > 0)
            {	
                if(angleRobotGoal < 180*DEG_TO_RAD)
                {
                    return - _xVelocities.linear.x / dCalculateRadius(distanceRobotGoal, angleRobotGoal);
                }
                else
                {
                    return - 2*_xVelocities.linear.x/CHASSIS_LENGTH;
                }
            }
            else
            {	
                if(angleRobotGoal > -180*DEG_TO_RAD)
                {
                    return _xVelocities.linear.x / dCalculateRadius(distanceRobotGoal, angleRobotGoal);
                }
                else
                {
                    return 2*_xVelocities.linear.x/CHASSIS_LENGTH;
                }
            }
        }
        else 
        {
            // Inside dead zone - straight line
            return 0.0;
        }
    }

    double dCalculateRadius(double distanceRobotWaypoint, double angleRobotWaypoint)
    {
        double radius;
        radius = distanceRobotWaypoint/2/sin(fabs(angleRobotWaypoint));
    }

    geometry_msgs::Twist SimplePlannerROS::xStopRobot()
    {
        geometry_msgs::Twist velocities;
        velocities.linear.x= 0;
        velocities.linear.y= 0;
        velocities.angular.z=0;
        return velocities;
    }

    pos SimplePlannerROS::pGetFrame(int count)
    {
        pos frame;
        frame.x = _vPlan[count].pose.position.x;
        frame.y = _vPlan[count].pose.position.y;
        frame.az = dGetYawFromQuaternion(_vPlan[count].pose.orientation);
        return frame;
    }

    double SimplePlannerROS::dLimitAngle(double angle)
    {
        if ( angle > 180*DEG_TO_RAD ) 
        {
            angle -= 360*DEG_TO_RAD; 
        }
        if ( angle < -180*DEG_TO_RAD ) 
        {
            angle += 360*DEG_TO_RAD;  
        }
        return angle;
    }

    double SimplePlannerROS::dAngleBetweenFrames(pos targetFrame, pos currentFrame)
    {
        return std::atan2(targetFrame.y - currentFrame.y, targetFrame.x - currentFrame.x);
    }
    
    double SimplePlannerROS::dDistanceBetweenFrames(pos targetFrame, pos currentFrame)
    {
        return std::sqrt(std::pow(targetFrame.y - currentFrame.y, 2) + std::pow(targetFrame.x - currentFrame.x, 2));
    }

    void SimplePlannerROS::vCalculateErrors()
    {
        _dDistanceWaypoints = dDistanceBetweenFrames(_pNextWaypoint, _pLastWaypoint);
        _dDistanceToNextWaypoint = dDistanceBetweenFrames(_pNextWaypoint, _pRobotCenter);
        _dDistanceToNextWaypointFront = dDistanceBetweenFrames(_pNextWaypoint, _pRobotFront);
        _dDistanceFromLastWaypoint = dDistanceBetweenFrames(_pLastWaypoint, _pRobotCenter);
        _dDeviation = dCalculateDeviation(_dDistanceWaypoints, _dDistanceToNextWaypoint, _dDistanceFromLastWaypoint);
    }

    double SimplePlannerROS::dCalculateDeviation(double distance, double remaining, double traveled)
    {
        // calculating triangle area (known length of all sides) to compute one of the heights afterwards
        double heron=(distance+remaining+traveled)/2;
        if (distance > 0)
        {
            return 2*std::sqrt(heron*(heron-distance)*(heron-remaining)*(heron-traveled))/distance;
        }
        else
        {
            return 0.0;
        }
    }

    pos SimplePlannerROS::pCalculateGoal(pos nextWaypoint, pos robotCenter)
    {
            double angleRobotNextWaypoint = dLimitAngle(dAngleBetweenFrames(nextWaypoint, robotCenter) - robotCenter.az);
            // if ( _iCount > 1)
            // {
            //     pos goal_min, goal_max;
            //     double angle_min, angle_max, angle_now;
            //     double delta_x, delta_y, delta_len;

            //     if (_iCount < _iPlanLength-1) 
            //     {
            //         // // Bisecection of _pLastWaypoint-_pNextWaypoint-future frames angle
            //         // pos tmp;
            //         // tmp = pGetFrame(_iCount+1);
            //         // double length_tmp_next = std::sqrt((tmp.x-_pNextWaypoint.x)*(tmp.x-_pNextWaypoint.x) + (tmp.y-_pNextWaypoint.y)*(tmp.y-_pNextWaypoint.y));
            //         // double length_last_next = std::sqrt((_pLastWaypoint.x-_pNextWaypoint.x)*(_pLastWaypoint.x-_pNextWaypoint.x) + (_pLastWaypoint.y-_pNextWaypoint.y)*(_pLastWaypoint.y-_pNextWaypoint.y));

            //         // delta_x = ((tmp.x-_pNextWaypoint.x)/length_tmp_next + (_pLastWaypoint.x-_pNextWaypoint.x)/length_last_next);
            //         // delta_y = ((tmp.y-_pNextWaypoint.y)/length_tmp_next + (_pLastWaypoint.y-_pNextWaypoint.y)/length_last_next);
            //         // delta_len = std::sqrt(delta_x*delta_x + delta_y*delta_y);
            //         // // 

            //         // Perpendicular to robot-next_frame line
            //         delta_x = cos(angleRobotNextWaypoint - 90*DEG_TO_RAD);
            //         delta_y = sin(angleRobotNextWaypoint - 90*DEG_TO_RAD);
            //         // delta_len = std::sqrt(delta_x*delta_x + delta_y*delta_y);
            //         // 
            //     }
            //     else
            //     {
            //         delta_x = cos(_pNextWaypoint.az - 90*DEG_TO_RAD);
            //         delta_y = sin(_pNextWaypoint.az - 90*DEG_TO_RAD);
            //         // delta_len = std::sqrt(delta_x*delta_x + delta_y*delta_y);
            //     }

            //     _pGoalRange.x = delta_x  * GOAL_DISTANCE_TOLERANCE /2;
            //     _pGoalRange.y = delta_y  * GOAL_DISTANCE_TOLERANCE /2;

            //     goal_min.x = _pNextWaypoint.x + _pGoalRange.x;
            //     goal_min.y = _pNextWaypoint.y + _pGoalRange.y;
            //     goal_max.x = _pNextWaypoint.x - _pGoalRange.x;
            //     goal_max.y = _pNextWaypoint.y - _pGoalRange.y;

            //     angle_min = dLimitAngle(dAngleBetweenFrames(goal_min, _pRobotCenter));
            //     angle_max = dLimitAngle(dAngleBetweenFrames(goal_max, _pRobotCenter));
            //     angle_now = dLimitAngle(_pRobotCenter.az);

            //     // ROS_INFO("min %f max %f current %f",
            //     //             dLimitAngle(angle_now - angle_max)*RAD_TO_DEG,
            //     //             dLimitAngle(angle_now - angle_min)*RAD_TO_DEG,
            //     //             dLimitAngle(angle_now) * RAD_TO_DEG);

            //     if (dLimitAngle(angle_now - angle_max) > 0 || dLimitAngle(angle_now - angle_min) < 0)
            //     {
            //         return _pNextWaypoint;
            //     }
            //     else
            //     {
            //         double factor;
            //         factor = ((angle_max+angle_min)/2 - angle_now) / fabs(dLimitAngle(angle_max - angle_min));
            //         // ROS_INFO("min %f max %f current %f mid %f _pGoalRange %f factor %f",
            //         //         (angle_min)*RAD_TO_DEG,
            //         //         (angle_max)*RAD_TO_DEG,
            //         //         angle_now*RAD_TO_DEG,
            //         //         (angle_max+angle_min)/2*RAD_TO_DEG,
            //         //         fabs(dLimitAngle(angle_max - angle_min))*RAD_TO_DEG,
            //         //         factor);
            //         pos goal;
            //         goal.x = _pNextWaypoint.x + _pGoalRange.x * factor;
            //         goal.y = _pNextWaypoint.y + _pGoalRange.y * factor;
            //         goal.az = std::atan2(goal.y - _pLastWaypoint.y, goal.x - _pLastWaypoint.x);
            //         return goal;
            //     }
            // }
            // else
            // {
            //     return _pNextWaypoint;
            // }

            // #ifndef ARCH_APPROACH
            //     return _pNextWaypoint;
            // #endif

        return _pNextWaypoint;
    }

    void SimplePlannerROS::vPathVisualization()
    {
        _traveledPoints.header.frame_id = "/odom";
        _traveledPoints.header.stamp = ros::Time::now();
        _traveledPoints.ns = "travelled_path";
        _traveledPoints.action = visualization_msgs::Marker::ADD;
        _traveledPoints.id = 0;
        _traveledPoints.type = visualization_msgs::Marker::LINE_STRIP;
        _traveledPoints.scale.x = 0.1;
        _traveledPoints.scale.y = 0.1;
        _traveledPoints.color.b = 1.0;
        _traveledPoints.color.a = 1.0;
        geometry_msgs::Point p;
        p.x = _pRobotCenter.x;
        p.y = _pRobotCenter.y;
        p.z = 0.1;
        _traveledPoints.points.push_back(p);
        _rPathPub.publish(_traveledPoints); 
     }

    void SimplePlannerROS::vPredictedPathVisualization()
    {
        visualization_msgs::Marker futurePoints;
        futurePoints.header.frame_id = "/odom";
        futurePoints.header.stamp = ros::Time::now();
        futurePoints.ns = "predicted_path";
        futurePoints.action = visualization_msgs::Marker::ADD;
        futurePoints.id = 1;
        futurePoints.type = visualization_msgs::Marker::LINE_STRIP;
        futurePoints.scale.x = 0.1;
        futurePoints.scale.y = 0.1;
        futurePoints.color.r = 1.0;
        futurePoints.color.a = 1.0;

        // calculating turn radius from angular and linear velocity
        // R = v / w
        double radius;
        if(_xVelocities.angular.z != 0)
        {
            radius = _xVelocities.linear.x / _xVelocities.angular.z;
        }
        else
        {
            radius = 0;
        }

        if (radius != 0)
        {
            // draw arch
            geometry_msgs::Point p,tmp;
            double theta;
            for(int i=0; i < PREDICTION_POINTS_PER_SECOND*PREDICTION_TIME; i++)
            {
                theta = i*_xVelocities.angular.z/PREDICTION_POINTS_PER_SECOND;
                tmp.x = radius*cos(theta-90*DEG_TO_RAD)-radius*cos(-90*DEG_TO_RAD);
                tmp.y = radius*sin(theta-90*DEG_TO_RAD)-radius*sin(-90*DEG_TO_RAD);

                p.x = _pRobotCenter.x + tmp.x * cos(_pRobotCenter.az) - tmp.y * sin(_pRobotCenter.az);
                p.y = _pRobotCenter.y + tmp.x * sin(_pRobotCenter.az) + tmp.y * cos(_pRobotCenter.az);

                p.z = 0.1;
                futurePoints.points.push_back(p);
            }
        }
        else
        {
            // draw straight line
            geometry_msgs::Point p;
            for(int i=0; i < PREDICTION_POINTS_PER_SECOND*PREDICTION_TIME; i++)
            {
                p.x = _pRobotCenter.x+i*_xVelocities.linear.x/PREDICTION_POINTS_PER_SECOND*cos(_pRobotCenter.az);
                p.y = _pRobotCenter.y+i*_xVelocities.linear.x/PREDICTION_POINTS_PER_SECOND*sin(_pRobotCenter.az);
                p.z = 0.1;
                futurePoints.points.push_back(p);
            }
        }
        _rPredictedPathPub.publish(futurePoints); 
     }

    void SimplePlannerROS::vNextGoalVisualization(pos _pFollowedGoal)
    {
        visualization_msgs::Marker marker;
        marker.header.frame_id = "/odom";
        marker.header.stamp = ros::Time::now();
        marker.ns = "next_goal";
        marker.action = visualization_msgs::Marker::ADD;
        marker.id = 2;
        marker.type = visualization_msgs::Marker::CUBE;
        marker.scale.x = 0.1;
        marker.scale.y = 0.1;
        marker.scale.z = 3;
        marker.color.b = 1;
        marker.color.r = 1;
        marker.color.a = 1.0;

        marker.pose.position.x = _pFollowedGoal.x;
        marker.pose.position.y = _pFollowedGoal.y;
        marker.pose.position.z = 0;
        
        _rNextGoalPub.publish(marker); 
     }

    void SimplePlannerROS::vGoalRangeVisualization()
    {
        visualization_msgs::Marker marker;
        marker.header.frame_id = "/odom";
        marker.header.stamp = ros::Time::now();
        marker.ns = "goal_range";
        marker.action = visualization_msgs::Marker::ADD;
        marker.id = 2;
        marker.type = visualization_msgs::Marker::LINE_STRIP;
        marker.scale.x = 0.1;
        marker.scale.y = 0.1;
        marker.scale.z = 0.1;
        marker.color.b = 1;
        marker.color.r = 1;
        marker.color.a = 1.0;

        geometry_msgs::Point p;
        p.x = _pNextWaypoint.x - _pGoalRange.x;
        p.y = _pNextWaypoint.y - _pGoalRange.y;
        p.z = 0;

        marker.points.push_back(p);
        
        p.x = _pNextWaypoint.x + _pGoalRange.x;
        p.y = _pNextWaypoint.y + _pGoalRange.y;
        p.z = 0;

        marker.points.push_back(p);
        
        _rGoalRangePub.publish(marker); 
    }

    void SimplePlannerROS::vVisualizations()
    {
       vNextGoalVisualization(_pFollowedGoal);
    //    vGoalRangeVisualization(); 
       vPathVisualization();
       vPredictedPathVisualization();
    }
     
}