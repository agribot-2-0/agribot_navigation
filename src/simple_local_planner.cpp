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
#define SPEED_LIMIT

// #define DEBUG
#define STATISTICS

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

            _bStartedMoving = false;
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
        _iMissed = 0; 
        _vPlan = orig_global_plan; 
        _iLength = _vPlan.size(); 
        _bGoalReached = false;
        _traveledPoints=visualization_msgs::Marker();

        _pLastWaypoint = _pRobotCenter;
        _pNextWaypoint = pGetFrame(_iCount); 

        _bStartedMoving = true;
        #ifdef STATISTICS
            _dDeviationIntegral = 0;
            _dTraveledDistance = 0;

            _dStartTime = ros::Time::now().toSec();
            _dLastTime = 0;
            _dActualTime = 0;
        #endif

        ROS_INFO("Received new plan, _iLength = %d", _iLength);
        return true;
    }

    bool SimplePlannerROS::computeVelocityCommands(geometry_msgs::Twist& cmd_vel)
    {
        if(!_bInitialized)
        {
            ROS_ERROR("This planner has not been initialized, please call initialize() before using this planner");
            return false;
        }
        if(_iLength != 0)
        { 
            // Check where robot is
            vCalculateErrors();

            // Deviation error for statistics
            #ifdef STATISTICS
                if(_dActualTime > 0)
                {
                    if (!isnan(_dRobotDeviationFromLineBetweenWaypoints))
                    {
                        _dLastTime = _dActualTime;
                        _dActualTime = ros::Time::now().toSec();
                        _dDeviationIntegral += fabs(_dRobotDeviationFromLineBetweenWaypoints)*(_dActualTime-_dLastTime);
                    }
                }
                else
                {
                    _dActualTime = ros::Time::now().toSec();
                }
            #endif

            #ifdef DEBUG
                ROS_INFO("count %d _iMissed %d _dDistanceFromLastToNextWaypoint %f traveled %f error %f _dRobotDeviationFromLineBetweenWaypoints %f parallelness %f", _iCount, _iMissed, _dDistanceFromLastToNextWaypoint, _dDistanceFromLastWaypointToRobotCenter, _dDistanceFromRobotCenterToNextWaypoint, _dRobotDeviationFromLineBetweenWaypoints, cos(_pLastWaypoint.az-_pRobotCenter.az));
            #endif


            // Determine behaviour basing on location
            if(_dDistanceFromLastWaypointToRobotCenter >= _dDistanceFromLastToNextWaypoint - CHASSIS_LENGTH/2)
            {
                // Robot is within error _pGoalRange from _pFollowedGoal waypoint
                if(_dDistanceFromRobotFrontToNextWaypoint < GOAL_DISTANCE_TOLERANCE)
                {
                    if(_iCount<(_iLength-1))
                    {
                        // It is not the _pLastWaypoint point of the path
                        _iCount++;
                        #ifdef ARCH_APPROACH
                            _pLastWaypoint = _pFollowedGoal;
                        #else
                            _pLastWaypoint = _pNextWaypoint;
                        #endif
                        _pNextWaypoint = pGetFrame(_iCount);
                    }
                    else
                    {
                        // Reached final destionation
                        vStopRobot();
                        _bGoalReached = true;
                        #ifdef STATISTICS
                            _dStopTime = ros::Time::now().toSec();
                            ROS_INFO("GOAL REACHED!    Time = %f s    Traveled  %f    Error = %f", _dStopTime-_dStartTime,_dTraveledDistance, _dDeviationIntegral);
                        #endif
                    }
                }
            }
            else if(_dDistanceFromLastWaypointToRobotCenter >= _dDistanceFromLastToNextWaypoint)
            {
                // Traveled more than _dDistanceFromLastToNextWaypoint between _pLastWaypoint and _pNextWaypoint waypoints
                if(_dDistanceFromRobotCenterToNextWaypoint > GOAL_DISTANCE_TOLERANCE)
                {
                    // Didn't hit the _pFollowedGoal waypoint -> skip it and go to _pNextWaypoint one
                    _iMissed++;
                    _iCount++;
                    #ifdef ARCH_APPROACH
                        _pLastWaypoint = _pFollowedGoal;
                    #else
                        _pLastWaypoint = _pNextWaypoint;
                    #endif
                    _pNextWaypoint = pGetFrame(_iCount);
                    ROS_ERROR("OUT OF PATH!");
                }
            }
            else
            {
                // Somewhere between _pLastWaypoint and _pNextWaypoint waypoint
                vSetVel();
            }

            // Behaviour after missing too many points
            if(_iMissed > MISSED_POINTS_TOLLERANCE)
            {
                vStopRobot();
                _iLength = 0;
                #ifdef STATISTICS
                    _dStopTime = ros::Time::now().toSec();
                    ROS_ERROR("STOPPING, ROBOT MISSED %d POINTS Time = %f    Traveled  %f", _iMissed, _dStopTime-_dStartTime,_dTraveledDistance);
                #else
                    ROS_ERROR("STOPPING, ROBOT MISSED %d POINTS", _iMissed);
                #endif
            }
        }
        cmd_vel = _cmd;  
        return true;
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
        pos tmp;
        tmp = _pRobotCenter;
        _pRobotCenter.x = odometry_msg.pose.pose.position.x;
        _pRobotCenter.y = odometry_msg.pose.pose.position.y;
        _pRobotCenter.az = dGetYawFromQuaternion(odometry_msg.pose.pose.orientation); 
        _pRobotFront.x = _pRobotCenter.x + cos(_pRobotCenter.az) * CHASSIS_LENGTH/2;
        _pRobotFront.y = _pRobotCenter.y + sin(_pRobotCenter.az) * CHASSIS_LENGTH/2;
        _pRobotFront.az = _pRobotCenter.az;

        if (_bStartedMoving)
        {
            #ifdef STATISTICS
                _dTraveledDistance += std::sqrt((tmp.x-_pRobotCenter.x)*(tmp.x-_pRobotCenter.x) + (tmp.y-_pRobotCenter.y)*(tmp.y-_pRobotCenter.y));
            #endif

            // Calculate location ofrobot
            vCalculateErrors();

            // Visualization
            vPathVisualization();
        }
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

    void SimplePlannerROS::vSetVel()
    {
        #ifdef SPEED_LIMIT
            double deviation_factor, distance_factor, multiplier;
            deviation_factor = 5/(5+_dRobotDeviationFromLineBetweenWaypoints);
            distance_factor = 1-fabs((_dDistanceFromLastToNextWaypoint-_dDistanceFromRobotFrontToNextWaypoint)/_dDistanceFromLastToNextWaypoint);
            multiplier = (1 - deviation_factor) * (1-distance_factor);
            _cmd.linear.x = MAX_LINEAR_VEL*(deviation_factor*(1-multiplier));
        #else
            _cmd.linear.x = MAX_LINEAR_VEL;
        #endif
        
        #ifdef ARCH_APPROACH
            double radius;
            if (fabs(_pDifferencFollowedGoalAndRobotCenter.az) > DEAD_ZONE*DEG_TO_RAD)
            {
                if (_pDifferencFollowedGoalAndRobotCenter.az > 0)
                {	
                    if(_pDifferencFollowedGoalAndRobotCenter.az < 180*DEG_TO_RAD)
                    {
                        radius = 0.5*_dDistanceFromRobotCenterToNextWaypoint/cos(90*RAD_TO_DEG-fabs(_pDifferencFollowedGoalAndRobotCenter.az));
                        _cmd.angular.z = - _cmd.linear.x / radius;
                    }
                    else
                    {
                        _cmd.angular.z = - 2*_cmd.linear.x/CHASSIS_LENGTH;
                    }
                }
                else
                {	
                    if(_pDifferencFollowedGoalAndRobotCenter.az > -180*DEG_TO_RAD)
                    {
                        radius = 0.5*_dDistanceFromRobotCenterToNextWaypoint/cos(90*RAD_TO_DEG-fabs(_pDifferencFollowedGoalAndRobotCenter.az));
                        _cmd.angular.z = _cmd.linear.x / radius;
                    }
                    else
                    {
                        _cmd.angular.z = 2*_cmd.linear.x/CHASSIS_LENGTH;
                    }
                }
            }
            else 
            {
                _cmd.angular.z = 0;
            }
        #else
            if (_pDifferencRobotCenterAndNextWaypoint.az > 0)
            {
                _cmd.angular.z = (DEVIATION_WEIGHT*_dRobotDeviationFromLineBetweenWaypoints/(1+_dRobotDeviationFromLineBetweenWaypoints)+PARALLELNESS_WEIGHT*(1-fabs(cos(_pLastWaypoint.az-_pRobotCenter.az))))*MAX_LINEAR_VEL / MIN_RADIUS;
            }
            else if (_pDifferencRobotCenterAndNextWaypoint.az < 0)
            {
                _cmd.angular.z = -(DEVIATION_WEIGHT*_dRobotDeviationFromLineBetweenWaypoints/(1+_dRobotDeviationFromLineBetweenWaypoints)+PARALLELNESS_WEIGHT*(1-fabs(cos(_pLastWaypoint.az-_pRobotCenter.az))))*MAX_LINEAR_VEL / MIN_RADIUS;
            }
        #endif

        // Velocity limiter
        if (_cmd.linear.x > MAX_LINEAR_VEL)
        {
            _cmd.linear.x = MAX_LINEAR_VEL;
        } 
        if (_cmd.angular.z > _cmd.linear.x / MIN_RADIUS)
        {
            _cmd.angular.z = _cmd.linear.x / MIN_RADIUS;
        }
        if (_cmd.angular.z < -_cmd.linear.x / MIN_RADIUS)
        {
            _cmd.angular.z = -_cmd.linear.x / MIN_RADIUS;
        }

        // Visualization
        vPredictedPathVisualization();
    }


    void SimplePlannerROS::vStopRobot()
    {
        _cmd.linear.x= 0;
        _cmd.linear.y= 0;
        _cmd.angular.z=0;
        _bStartedMoving = false;
    }

    pos SimplePlannerROS::pGetFrame(int count)
    {
        pos tmp;
        tmp.x = _vPlan[count].pose.position.x;
        tmp.y = _vPlan[count].pose.position.y;
        tmp.az = dGetYawFromQuaternion(_vPlan[count].pose.orientation);
        return tmp;
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

    pos SimplePlannerROS::pCalculateFramesDifference(pos _pNextWaypoint, pos prev)
    {
        pos difference;
        difference.x = (_pNextWaypoint.x - prev.x);
        difference.y = (_pNextWaypoint.y - prev.y);
        difference.az = std::atan2(difference.y, difference.x) - prev.az;
        difference.az = dLimitAngle(difference.az);
        return difference;
    }

    void SimplePlannerROS::vCalculateErrors()
    {
        vCalculateGoal();
        _pDifferencRobotCenterAndNextWaypoint = pCalculateFramesDifference(_pNextWaypoint, _pRobotCenter);
        _pDifferencRobotFrontAndNextWaypoint = pCalculateFramesDifference(_pNextWaypoint, _pRobotCenter);
        _pDifferencRobotCenterAndLastWaypoint = pCalculateFramesDifference(_pRobotCenter, _pLastWaypoint);
        _pDifferencNextWaypointAndLastWaypoint = pCalculateFramesDifference(_pNextWaypoint, _pLastWaypoint);
        _pDifferencFollowedGoalAndRobotCenter = pCalculateFramesDifference(_pFollowedGoal, _pRobotCenter);

        _pDifferencNextWaypointAndLastWaypoint.az = dLimitAngle(_pDifferencNextWaypointAndLastWaypoint.az - _pLastWaypoint.az);

        _dDistanceFromLastToNextWaypoint = std::sqrt(_pDifferencNextWaypointAndLastWaypoint.x * _pDifferencNextWaypointAndLastWaypoint.x + _pDifferencNextWaypointAndLastWaypoint.y * _pDifferencNextWaypointAndLastWaypoint.y);
        _dDistanceFromRobotCenterToNextWaypoint = std::sqrt(_pDifferencRobotCenterAndNextWaypoint.x * _pDifferencRobotCenterAndNextWaypoint.x + _pDifferencRobotCenterAndNextWaypoint.y * _pDifferencRobotCenterAndNextWaypoint.y);
        _dDistanceFromRobotFrontToNextWaypoint = std::sqrt(_pDifferencRobotFrontAndNextWaypoint.x * _pDifferencRobotFrontAndNextWaypoint.x + _pDifferencRobotFrontAndNextWaypoint.y * _pDifferencRobotFrontAndNextWaypoint.y);
        _dDistanceFromLastWaypointToRobotCenter = std::sqrt(_pDifferencRobotCenterAndLastWaypoint.x * _pDifferencRobotCenterAndLastWaypoint.x + _pDifferencRobotCenterAndLastWaypoint.y * _pDifferencRobotCenterAndLastWaypoint.y);

        // calculating triangle area (known _iLength of all sides) to compute one of the heights afterwards
        double heron=(_dDistanceFromLastToNextWaypoint+_dDistanceFromRobotCenterToNextWaypoint+_dDistanceFromLastWaypointToRobotCenter)/2;
        _dRobotDeviationFromLineBetweenWaypoints=2*sqrt(heron*(heron-_dDistanceFromLastToNextWaypoint)*(heron-_dDistanceFromRobotCenterToNextWaypoint)*(heron-_dDistanceFromLastWaypointToRobotCenter))/_dDistanceFromLastToNextWaypoint;
    }

    void SimplePlannerROS::vCalculateGoal()
    {
            
            // if ( _iCount > 1)
            // {
            //     pos goal_min, goal_max;
            //     double angle_min, angle_max, angle_now;
            //     double delta_x, delta_y, delta_len;

            //     if (_iCount < _iLength-1) 
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
            //         delta_x = cos(_pDifferencRobotCenterAndNextWaypoint.az + _pRobotCenter.az - 90*DEG_TO_RAD);
            //         delta_y = sin(_pDifferencRobotCenterAndNextWaypoint.az + _pRobotCenter.az - 90*DEG_TO_RAD);
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

            //     angle_min = dLimitAngle(pCalculateFramesDifference(goal_min, _pRobotCenter).az+_pRobotCenter.az);
            //     angle_max = dLimitAngle(pCalculateFramesDifference(goal_max, _pRobotCenter).az+_pRobotCenter.az);
            //     angle_now = dLimitAngle(_pRobotCenter.az);

            //     // ROS_INFO("min %f max %f current %f",
            //     //             dLimitAngle(angle_now - angle_max)*RAD_TO_DEG,
            //     //             dLimitAngle(angle_now - angle_min)*RAD_TO_DEG,
            //     //             dLimitAngle(angle_now) * RAD_TO_DEG);

            //     if (dLimitAngle(angle_now - angle_max) > 0 || dLimitAngle(angle_now - angle_min) < 0)
            //     {
            //         _pFollowedGoal = _pNextWaypoint;
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
            //         _pFollowedGoal.x = _pNextWaypoint.x + _pGoalRange.x * factor;
            //         _pFollowedGoal.y = _pNextWaypoint.y + _pGoalRange.y * factor;
            //         _pFollowedGoal.az = std::atan2(_pFollowedGoal.y - _pLastWaypoint.y, _pFollowedGoal.x - _pLastWaypoint.x);
            //     }
            // }
            // else
            // {
            //     _pFollowedGoal = _pNextWaypoint;
            // }

            // #ifndef ARCH_APPROACH
            //     _pFollowedGoal = _pNextWaypoint;
            // #endif

        _pFollowedGoal = _pNextWaypoint;
        
        // Visualization
        vNextGoalVisualization(_pFollowedGoal);
        vGoalRangeVisualization();
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
        if(_cmd.angular.z != 0)
        {
            radius = _cmd.linear.x / _cmd.angular.z;
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
                theta = i*_cmd.angular.z/PREDICTION_POINTS_PER_SECOND;
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
                p.x = _pRobotCenter.x+i*_cmd.linear.x/PREDICTION_POINTS_PER_SECOND*cos(_pRobotCenter.az);
                p.y = _pRobotCenter.y+i*_cmd.linear.x/PREDICTION_POINTS_PER_SECOND*sin(_pRobotCenter.az);
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
     
}