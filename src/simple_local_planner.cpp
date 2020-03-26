#include "agribot_navigation/simple_local_planner.h"

#include <pluginlib/class_list_macros.h>
#include <cmath>

#define GOAL_DISTANCE_TOLERANCE 0.75
#define GOAL_RANGE GOAL_DISTANCE_TOLERANCE*0.66
#define MISSED_POINTS_TOLLERANCE 10000

#define CHASSIS_LENGTH 2.1
#define MIN_RADIUS 2.0

#define PREDICTION_TIME 60
#define PREDICTION_POINTS_PER_SECOND 2

#define DEAD_ZONE 3.0   // degrees
#define MAX_LINEAR_VEL 0.7
#define MIN_LINEAR_VEL 0.1

#define DEVIATION_WEIGHT 0.1
#define PARALLELNESS_WEIGHT 10.0

#define EPSILON 1e-3

// Planner approach choice
// #define ARCH_APPROACH
#define SPEED_LIMIT

// #define DEBUG
#define STATISTICS

PLUGINLIB_EXPORT_CLASS(simple_local_planner::SimplePlannerROS,
nav_core::BaseLocalPlanner)

namespace simple_local_planner
{

    SimplePlannerROS::SimplePlannerROS() :
    _xCostmapROS(NULL), _bInitialized(false) {}

    SimplePlannerROS::~SimplePlannerROS() {}

    void SimplePlannerROS::initialize(
        std::string name, tf2_ros::Buffer* tfBuffer,
        costmap_2d::Costmap2DROS* costmapROS)
    {
        if (!_bInitialized)
        {
            _xCostmapROS = costmapROS;
            _xTFBuffer = tfBuffer;
            _xTFListener = new tf2_ros::TransformListener(*tfBuffer);

            ros::NodeHandle gn;
            _rOdomSub = gn.subscribe(
                "/odom", 100, &SimplePlannerROS::vOdomCallback, this);
            _rPathPub = gn.advertise<
                visualization_msgs::Marker>("travelled_path", 10);
            _rPredictedPathPub = gn.advertise<
                visualization_msgs::Marker>("predicted_path", 10);
            _rGoalRangePub = gn.advertise<
                visualization_msgs::Marker>("goal_range", 10);
            _rNextGoalPub = gn.advertise<
                visualization_msgs::Marker>("next_goal", 10);

            _bInitialized = true;

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
            ROS_WARN(
                "This planner has already been initialized, doing nothing.");
        }
    }

    bool SimplePlannerROS::setPlan(
        const std::vector<geometry_msgs::PoseStamped>& orig_global_plan)
    {
        if (!_bInitialized)
        {
            ROS_ERROR(
                "Planner not initialized, call initialize() before planner");
            return false;
        }
        _iWaypointCount = 1;
        _iMissedWaypointsCount = 0;
        _xPlan = orig_global_plan;
        _iPlanLength = _xPlan.size();
        _bGoalReached = false;
        _traveledPoints = visualization_msgs::Marker();

        _pLastWaypoint = _pRobotCenter;

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

    bool SimplePlannerROS::computeVelocityCommands(
        geometry_msgs::Twist& cmd_vel)
    {
        geometry_msgs::Twist velocity;
        pos goal;

        if (!_bInitialized)
        {
            ROS_ERROR(
                "Planner not initialized, call initialize() before planner");
            return false;
        }
        if (_iPlanLength != 0)
        {
            // Check where robot is
            pos nextWaypoint = pGetFrame(_iWaypointCount);
            if (bIsLastWaypointInPath(_iWaypointCount+1))
            {
                nextWaypoint.az = dAngleBetweenFrames(
                    nextWaypoint, pGetFrame(_iWaypointCount-1));
                nextWaypoint = pCalculateRobotFrontPosition(nextWaypoint);
            }

            goal = pCalculateGoal(nextWaypoint, _pRobotCenter);


            // Deviation error for statistics
            #ifdef STATISTICS
                vCalculateDeviationIntegral(
                    _pLastWaypoint, nextWaypoint, _pRobotCenter);
            #endif

            // Increment missed points number
            if (bWaypointMissedCondition(
                _pRobotCenter, _pLastWaypoint, nextWaypoint))
            {
                 ROS_ERROR("OUT OF PATH!");
                _iMissedWaypointsCount++;
            }

            // Next goal condition
            if (!bIsLastWaypointInPath(_iWaypointCount) &&
                 (bWaypointHitCondition(
                     _pRobotCenter, _pLastWaypoint, nextWaypoint) ||
                  bWaypointMissedCondition(
                      _pRobotCenter, _pLastWaypoint, nextWaypoint)))
            {
                #ifdef STATISTICS
                    ROS_INFO("GOAL REACHED! Traveled  %f", _dTraveledDistance);
                #endif
                vTakeNextGoal(goal, _pRobotCenter);
            }

            // Calculate velocities
            velocity = xSetVel(goal, nextWaypoint, _pRobotCenter);

            velocity.linear.x = dCloseToZero(velocity.linear.x);
            velocity.linear.y = dCloseToZero(velocity.linear.y);
            velocity.linear.z = dCloseToZero(velocity.linear.z);
            velocity.angular.x = dCloseToZero(velocity.angular.x);
            velocity.angular.y = dCloseToZero(velocity.angular.y);
            velocity.angular.z = dCloseToZero(velocity.angular.z);

            // Stop condition
            if (bMissedTooManyWaypointsCondition(_iMissedWaypointsCount) ||
            (bIsLastWaypointInPath(_iWaypointCount) &&
             bWaypointHitCondition(
                _pRobotCenter, _pLastWaypoint, nextWaypoint)) ||
            (bIsLastWaypointInPath(_iWaypointCount) &&
             bWaypointMissedCondition(
                _pRobotCenter, _pLastWaypoint, nextWaypoint)))
            {
                velocity = xFinishedPathBehaviour();
            }

            // Visualization
            vVisualizations(_pLastWaypoint, goal, nextWaypoint,
            _pRobotCenter, velocity);
        }
        cmd_vel = velocity;
        return true;
    }

    double SimplePlannerROS::dCloseToZero(double value)
    {
        if (fabs(value) < EPSILON)
        {
            return 0.0;
        }
        else
        {
            return value;
        }
    }

    bool SimplePlannerROS::bMissedTooManyWaypointsCondition(int missedWaypoints)
    {
        return missedWaypoints > MISSED_POINTS_TOLLERANCE;
    }

    bool SimplePlannerROS::bWaypointHitCondition(
        pos robotCenter, pos lastWaypoint, pos nextWaypoint)
    {
        return
        (bTraveledDistanceBetweenWaypoints(
            pCalculateRobotFrontPosition(robotCenter),
            lastWaypoint, nextWaypoint) &&
        bWithinGoalRange(
            pCalculateRobotFrontPosition(robotCenter),
            nextWaypoint)) ||
        (bTraveledDistanceBetweenWaypoints(
            robotCenter, lastWaypoint, nextWaypoint) &&
        bWithinGoalRange(robotCenter, nextWaypoint));
    }

    bool SimplePlannerROS::bWaypointMissedCondition(
        pos robotCenter, pos lastWaypoint, pos nextWaypoint)
     {
        return bCenterTraveledBeyondGoal(
            robotCenter, lastWaypoint, nextWaypoint) &&
        bCenterOutOfGoalRange(robotCenter, nextWaypoint);
    }

    bool SimplePlannerROS::bTraveledDistanceBetweenWaypoints(
        pos robotFront, pos lastWaypoint, pos nextWaypoint)
    {
        double distanceWaypoints = dDistanceBetweenFrames(
            nextWaypoint, lastWaypoint);
        double traveledDistance = dDistanceBetweenFrames(
            lastWaypoint, robotFront);
        return (traveledDistance >=
        distanceWaypoints - GOAL_DISTANCE_TOLERANCE);
    }

    bool SimplePlannerROS::bWithinGoalRange(pos robotFront, pos nextWaypoint)
    {
        double distanceToGoal = dDistanceBetweenFrames(
            nextWaypoint, robotFront);
        return (distanceToGoal <= GOAL_DISTANCE_TOLERANCE);
    }

    bool SimplePlannerROS::bIsLastWaypointInPath(int waypointsCount)
    {
        return (waypointsCount >= (_iPlanLength - 1));
    }

    bool SimplePlannerROS::bCenterTraveledBeyondGoal(
        pos robotCenter, pos lastWaypoint, pos nextWaypoint)
    {
        double distanceWaypoints = dDistanceBetweenFrames(
            nextWaypoint, lastWaypoint);
        double traveledDistance = dDistanceBetweenFrames(
            lastWaypoint, robotCenter);
        return (traveledDistance >= distanceWaypoints);
    }

    bool SimplePlannerROS::bCenterOutOfGoalRange(
        pos robotCenter, pos nextWaypoint)
    {
        double distanceToGoal = dDistanceBetweenFrames(
            nextWaypoint, robotCenter);
        return (distanceToGoal > GOAL_DISTANCE_TOLERANCE);
    }


    void SimplePlannerROS::vTakeNextGoal(pos currentGoal, pos robotCenter)
    {
        _iWaypointCount++;

        pos nextWaypoint = pGetFrame(_iWaypointCount);
        if (bIsLastWaypointInPath(_iWaypointCount+1))
        {
            nextWaypoint.az = dAngleBetweenFrames(
                nextWaypoint, pGetFrame(_iWaypointCount-1));
            nextWaypoint = pCalculateRobotFrontPosition(nextWaypoint);
        }

        if (dDistanceBetweenFrames(nextWaypoint, currentGoal) > CHASSIS_LENGTH)
        {
            _pLastWaypoint = currentGoal;
        }
        else
        {
            _pLastWaypoint = robotCenter;
        }
    }

    void SimplePlannerROS::vCalculateDeviationIntegral(
        pos LastWaypoint, pos nextWaypoint, pos robotCenter)
    {
        double deviation = dCalculateDeviation(
            LastWaypoint, nextWaypoint, robotCenter);
        if (_dActualTime > 0)
        {
            if (!isnan(deviation))
            {
                _dLastTime = _dActualTime;
                _dActualTime = ros::Time::now().toSec();
                _dDeviationIntegral +=
                fabs(deviation)*(_dActualTime-_dLastTime);
            }
        }
        else
        {
            _dActualTime = ros::Time::now().toSec();
        }
    }

    geometry_msgs::Twist SimplePlannerROS::xFinishedPathBehaviour()
    {
        _bGoalReached = true;
        _iPlanLength = 0;
        #ifdef STATISTICS
        _dStopTime = ros::Time::now().toSec();
        ROS_INFO("T = %f;l¸= %f;Er = %f;n = %d;x = %d",
        _dStopTime - _dStartTime, _dTraveledDistance,
        _dDeviationIntegral, _iWaypointCount, _iMissedWaypointsCount);
        #endif
        return xStopRobot();
    }

    bool SimplePlannerROS::isGoalReached()
    {
        if (!_bInitialized)
        {
            ROS_ERROR(
                "Planner not initialized, call initialize() before planner");
            return false;
        }
        return _bGoalReached;
    }

    void SimplePlannerROS::vOdomCallback(const nav_msgs::Odometry odometry_msg)
    {
        pos lastRobotPosition = _pRobotCenter;

        _pRobotCenter.x = odometry_msg.pose.pose.position.x;
        _pRobotCenter.y = odometry_msg.pose.pose.position.y;
        _pRobotCenter.az = dGetYawFromQuaternion(
            odometry_msg.pose.pose.orientation);

        #ifdef STATISTICS
            _dTraveledDistance += dDistanceBetweenFrames(
                lastRobotPosition, _pRobotCenter);
        #endif
    }

    pos SimplePlannerROS::pCalculateRobotFrontPosition(pos robotCenter)
    {
        pos frontPos;
        frontPos.x = robotCenter.x + cos(robotCenter.az) * CHASSIS_LENGTH/2;
        frontPos.y = robotCenter.y + sin(robotCenter.az) * CHASSIS_LENGTH/2;
        frontPos.az = robotCenter.az;
        return frontPos;
    }

    double SimplePlannerROS::dGetYawFromQuaternion(
        geometry_msgs::Quaternion quat)
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

    geometry_msgs::Twist SimplePlannerROS::xSetVel(
        pos goal, pos nextWaypoint, pos robotCenter)
    {
        geometry_msgs::Twist velocity;
        #ifdef SPEED_LIMIT
            velocity.linear.x = xLinearSpeedLimitedApproach(
                goal, robotCenter).linear.x;
        #else
            velocity.linear.x = MAX_LINEAR_VEL;
        #endif

        #ifdef ARCH_APPROACH
            velocity.angular.z = xArchApproach(
                goal, robotCenter, velocity).angular.z;
        #else
            velocity.angular.z = xDeviationApproach(
                nextWaypoint, robotCenter, velocity).angular.z;
        #endif

        // Velocity limiter
        return xLimitVelocities(velocity);
    }

    geometry_msgs::Twist SimplePlannerROS::xLimitVelocities(
        geometry_msgs::Twist Velcoities)
    {
        if (Velcoities.linear.x > MAX_LINEAR_VEL)
        {
            Velcoities.linear.x = MAX_LINEAR_VEL;
        }
        if (Velcoities.linear.x < MIN_LINEAR_VEL)
        {
            Velcoities.linear.x = MIN_LINEAR_VEL;
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

    geometry_msgs::Twist SimplePlannerROS::xDeviationApproach(
        pos goal, pos robotCenter, geometry_msgs::Twist velocity)
    {
        double deviation =  dCalculateDeviation(
            _pLastWaypoint, goal, _pRobotCenter);

        double factorDeviation = DEVIATION_WEIGHT*deviation/(1+deviation);
        double factorParallelness =
        PARALLELNESS_WEIGHT*(1-fabs(cos(dAngleBetweenFrames(
            goal, _pLastWaypoint)-robotCenter.az)));
        if (dLimitAngle(
            dAngleBetweenFrames(goal, robotCenter) -
            dAngleBetweenFrames(goal, _pLastWaypoint)) < 0)
        {
            factorDeviation *= -1;
        }
        if (dLimitAngle(
            dAngleBetweenFrames(goal, _pLastWaypoint) - robotCenter.az) < 0)
        {
            factorParallelness *= -1;
        }

        velocity.angular.z = (factorDeviation + factorParallelness) *
        MAX_LINEAR_VEL  / MIN_RADIUS;
        return velocity;
    }

    geometry_msgs::Twist SimplePlannerROS::xLinearSpeedLimitedApproach(
        pos goal, pos robotCenter)
    {
        geometry_msgs::Twist velocity;
        double distanceWaypoints =
        dDistanceBetweenFrames(goal, _pLastWaypoint);
        double distanceToNext =
        dDistanceBetweenFrames(goal, robotCenter);
        double waypointsDistanceFactor =
        distanceWaypoints / (1+ distanceWaypoints);
        double deviation =
        dCalculateDeviation(_pLastWaypoint, goal, _pRobotCenter);
        double parallelnessFactor =
        fabs(cos(dAngleBetweenFrames(goal, _pLastWaypoint)-robotCenter.az));
        double deviationFactor =
        1 / (1 + deviation);
        double distanceFactor =
        1 - fabs((distanceWaypoints - distanceToNext) / distanceWaypoints);
        double multiplier =
        (1 - deviationFactor) * (1 - distanceFactor);
        velocity.linear.x =
        MAX_LINEAR_VEL * (deviationFactor * (1 - multiplier)) *
        waypointsDistanceFactor *parallelnessFactor;
        if (isnan(velocity.linear.x))
        {
            velocity.linear.x = 0.0;
        }
        return velocity;
    }

    geometry_msgs::Twist SimplePlannerROS::xArchApproach(
        pos goal, pos robotCenter, geometry_msgs::Twist velocity)
    {
        double angleRobotGoal =
        dLimitAngle(dAngleBetweenFrames(goal, robotCenter)-robotCenter.az);
        double distanceRobotGoal =
        dDistanceBetweenFrames(goal, robotCenter);
        if (fabs(angleRobotGoal) > DEAD_ZONE*DEG_TO_RAD)
        {
            // Compute arch only outside of dead zone
            if (angleRobotGoal > 0)
            {
                if (angleRobotGoal < 180*DEG_TO_RAD)
                {
                    velocity.angular.z = velocity.linear.x /
                    dCalculateRadius(distanceRobotGoal, angleRobotGoal);
                }
                else
                {
                    velocity.angular.z = - 2*velocity.linear.x/CHASSIS_LENGTH;
                }
            }
            else
            {
                if (angleRobotGoal > -180*DEG_TO_RAD)
                {
                    velocity.angular.z = velocity.linear.x /
                    dCalculateRadius(distanceRobotGoal, angleRobotGoal);
                }
                else
                {
                    velocity.angular.z = 2*velocity.linear.x/CHASSIS_LENGTH;
                }
            }
        }
        else
        {
            // Inside dead zone - straight line
            velocity.angular.z = 0.0;
        }
        return velocity;
    }

    double SimplePlannerROS::dCalculateRadius(
        double distanceRobotWaypoint, double angleRobotWaypoint)
    {
        return distanceRobotWaypoint/2/sin((angleRobotWaypoint));
    }

    geometry_msgs::Twist SimplePlannerROS::xStopRobot()
    {
        geometry_msgs::Twist velocity;
        velocity.linear.x = 0;
        velocity.linear.y = 0;
        velocity.angular.z = 0;
        return velocity;
    }

    pos SimplePlannerROS::pGetFrame(int count)
    {
        geometry_msgs::PoseStamped nextPose = _xPlan[count];
        geometry_msgs::PoseStamped nextPoseTransformed;
        _xTFBuffer->transform(nextPose, nextPoseTransformed,
        "odom", ros::Duration(0.2));

        pos frame;
        frame.x = nextPoseTransformed.pose.position.x;
        frame.y = nextPoseTransformed.pose.position.y;
        frame.az = dGetYawFromQuaternion(nextPoseTransformed.pose.orientation);
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

    double SimplePlannerROS::dAngleBetweenFrames(
        pos targetFrame, pos currentFrame)
    {
        return std::atan2(targetFrame.y - currentFrame.y,
        targetFrame.x - currentFrame.x);
    }

    double SimplePlannerROS::dDistanceBetweenFrames(
        pos targetFrame, pos currentFrame)
    {
        return std::sqrt(
            std::pow(targetFrame.y - currentFrame.y, 2) +
            std::pow(targetFrame.x - currentFrame.x, 2));
    }

    double SimplePlannerROS::dCalculateDeviation(
        pos lastWaypoint, pos nextWaypoint, pos robotCenter)
    {
        if (dDistanceBetweenFrames(lastWaypoint, nextWaypoint) > 0)
        {
            pos projection = pCalculateProjection(
                lastWaypoint, nextWaypoint, robotCenter);
            return dDistanceBetweenFrames(robotCenter, projection);
        }
        else
        {
            return 0.0;
        }
    }

    pos SimplePlannerROS::pCalculateProjection(
        pos lastWaypoint, pos nextWaypoint, pos robotCenter)
    {
        pos projection;
        double a = dDistanceBetweenFrames(nextWaypoint, _pLastWaypoint);
        double b = dDistanceBetweenFrames(_pLastWaypoint, robotCenter);
        double c = dDistanceBetweenFrames(nextWaypoint, robotCenter);
        double dx = nextWaypoint.x - lastWaypoint.x;
        double dy = nextWaypoint.y - lastWaypoint.y;
        // taken from cosine law
        // cosine = (a*a + b*b - c*c) / (2*a*b)
        // a1 = b * cosine
        // ratio = a1 / a
        double ratio = (a*a +b*b - c*c) / (2*a*a);
        projection.x = lastWaypoint.x + ratio*dx;
        projection.y = lastWaypoint.y + ratio*dy;
        projection.az = dAngleBetweenFrames(nextWaypoint, lastWaypoint);
        return projection;
    }

    pos SimplePlannerROS::pCalculateGoal(pos nextWaypoint, pos robotCenter)
    {
        #ifdef ARCH_APPROACH

        pos goal_min, goal_max, goal;
        pos goalRange = pCalculateGoalRange(
            nextWaypoint, robotCenter, GOAL_RANGE);

        goal_min.x = nextWaypoint.x - goalRange.x;
        goal_min.y = nextWaypoint.y - goalRange.y;
        goal_max.x = nextWaypoint.x + goalRange.x;
        goal_max.y = nextWaypoint.y + goalRange.y;

        // https://en.wikipedia.org/wiki/Line%E2%80%93line_intersection

        goal.x =
        (((robotCenter.x * pCalculateRobotFrontPosition(robotCenter).y -
        robotCenter.y * pCalculateRobotFrontPosition(robotCenter).x) *
        (goal_min.x - goal_max.x)) -
        ((robotCenter.x - pCalculateRobotFrontPosition(robotCenter).x) *
        (goal_min.x * goal_max.y - goal_min.y * goal_max.x))) /
        (((robotCenter.x - pCalculateRobotFrontPosition(robotCenter).x) *
        (goal_min.y - goal_max.y)) -
        ((robotCenter.y - pCalculateRobotFrontPosition(robotCenter).y) *
        (goal_min.x - goal_max.x)));

        goal.y =
        (((robotCenter.x * pCalculateRobotFrontPosition(robotCenter).y -
        robotCenter.y * pCalculateRobotFrontPosition(robotCenter).x) *
        goal_min.y - goal_max.y)) -
        ((robotCenter.y - pCalculateRobotFrontPosition(robotCenter).y) *
        (goal_min.x * goal_max.y - goal_min.y * goal_max.x))) /
        (((robotCenter.x - pCalculateRobotFrontPosition(robotCenter).x) *
        (goal_min.y - goal_max.y)) -
        ((robotCenter.y - pCalculateRobotFrontPosition(robotCenter).y) *
        (goal_min.x - goal_max.x)));

        // Check if goal lays on the line segment
        // between goal_min and goal_max

        if (dDistanceBetweenFrames(goal, goal_min) >
        dDistanceBetweenFrames(goal_max, goal_min))
        {
            goal = goal_min;
        }
        else if (dDistanceBetweenFrames(goal, goal_max) >
        dDistanceBetweenFrames(goal_max, goal_min))
        {
            goal = goal_max;
        }

        goal.az = dLimitAngle(dAngleBetweenFrames(nextWaypoint, goal));

        return goal;

        #else
            return nextWaypoint;
        #endif
    }

    pos SimplePlannerROS::pCalculateGoalRange(
        pos nextWaypoint, pos robotCenter, double range)
    {
        pos goalRange;
        double deltaX, deltaY;
        double angleRobotNextWaypoint = dLimitAngle(
            dAngleBetweenFrames(nextWaypoint, robotCenter));

        if (!bIsLastWaypointInPath(_iWaypointCount))
        {
            // Perpendicular to robot-next_frame line
            deltaX = sin(angleRobotNextWaypoint);
            deltaY = -cos(angleRobotNextWaypoint);
        }
        else
        {
            deltaX = sin(nextWaypoint.az);
            deltaY = -cos(nextWaypoint.az);
        }
        goalRange.x = deltaX  * range;
        goalRange.y = deltaY  * range;
        goalRange.az = dLimitAngle(dAngleBetweenFrames(
            nextWaypoint, goalRange));

        return goalRange;
    }

    void SimplePlannerROS::vPathVisualization(pos robotCenter)
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
        p.x = robotCenter.x;
        p.y = robotCenter.y;
        p.z = 0.1;
        _traveledPoints.points.push_back(p);
        _rPathPub.publish(_traveledPoints);
     }

    void SimplePlannerROS::vPredictedPathVisualization(
        geometry_msgs::Twist velocity, pos robotCenter)
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
        if (velocity.angular.z != 0)
        {
            radius = velocity.linear.x / velocity.angular.z;
        }
        else
        {
            radius = 0;
        }

        if (radius != 0)
        {
            // draw arch
            geometry_msgs::Point p, tmp;
            for (int i = 0;
            i < PREDICTION_POINTS_PER_SECOND*PREDICTION_TIME; i++)
            {
                double theta =
                i * velocity.angular.z /PREDICTION_POINTS_PER_SECOND;
                tmp.x = radius * cos(theta - 90 * DEG_TO_RAD)-
                radius * cos(-90 * DEG_TO_RAD);
                tmp.y = radius * sin(theta - 90 * DEG_TO_RAD)-
                radius * sin(-90 * DEG_TO_RAD);

                p.x = robotCenter.x +
                tmp.x * cos(robotCenter.az) -
                tmp.y * sin(robotCenter.az);
                p.y = robotCenter.y +
                tmp.x * sin(robotCenter.az) +
                tmp.y * cos(robotCenter.az);

                p.z = 0.1;
                futurePoints.points.push_back(p);
            }
        }
        else
        {
            // draw straight line
            geometry_msgs::Point p;
            for (int i = 0;
            i < PREDICTION_POINTS_PER_SECOND*PREDICTION_TIME; i++)
            {
                p.x =
                robotCenter.x + i * velocity.linear.x /
                PREDICTION_POINTS_PER_SECOND*cos(robotCenter.az);
                p.y =
                robotCenter.y + i * velocity.linear.x /
                PREDICTION_POINTS_PER_SECOND*sin(robotCenter.az);
                p.z = 0.1;
                futurePoints.points.push_back(p);
            }
        }
        _rPredictedPathPub.publish(futurePoints);
     }

    void SimplePlannerROS::vNextGoalVisualization(pos lastWaypoint, pos goal)
    {
        visualization_msgs::Marker marker;
        marker.header.frame_id = "/odom";
        marker.header.stamp = ros::Time::now();
        marker.ns = "next_goal";
        marker.action = visualization_msgs::Marker::ADD;
        marker.id = 2;
        marker.type = visualization_msgs::Marker::LINE_STRIP;
        marker.scale.x = 0.1;
        marker.scale.y = 0.1;
        marker.scale.z = 3;
        marker.color.b = 1;
        marker.color.r = 1;
        marker.color.a = 1.0;

        geometry_msgs::Point p;
        p.x = lastWaypoint.x;
        p.y = lastWaypoint.y;
        p.z = 0;
        marker.points.push_back(p);

        p.x = goal.x;
        p.y = goal.y;
        p.z = 0;
        marker.points.push_back(p);

        _rNextGoalPub.publish(marker);
     }

    void SimplePlannerROS::vGoalRangeVisualization(pos goal, pos range)
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
        p.x = goal.x - range.x;
        p.y = goal.y - range.y;
        p.z = 0;

        marker.points.push_back(p);

        p.x = goal.x + range.x;
        p.y = goal.y + range.y;
        p.z = 0;

        marker.points.push_back(p);

        _rGoalRangePub.publish(marker);
    }

    void SimplePlannerROS::vVisualizations(
        pos lastWaypoint, pos goal, pos nextWaypoint,
    pos robotCenter, geometry_msgs::Twist velocity)
    {
       vNextGoalVisualization(lastWaypoint, goal);
       vGoalRangeVisualization(nextWaypoint, pCalculateGoalRange(
           nextWaypoint, robotCenter, GOAL_RANGE));
    //    vPathVisualization(robotCenter);
       vPredictedPathVisualization(velocity, robotCenter);
    }
}   // namespace simple_local_planner
