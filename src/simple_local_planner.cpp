#include "agribot_navigation/simple_local_planner.h"

// pluginlib macros (defines, ...)
#include <pluginlib/class_list_macros.h>
#include <cmath>

#define GOAL_DISTANCE_TOLERANCE 0.2
#define GOAL_ANGLE_TOLERANCE 360
#define MISSED_POINTS_TOLLERANCE 2
#define CHASSIS_LENGTH 2.1
// #define MIN_RADIUS (CHASSIS_LENGTH / 2)
#define MIN_RADIUS 2

// PLUGINLIB_DECLARE_CLASS(agribot_navigation, SimplePlannerROS, simple_local_planner::SimplePlannerROS, nav_core::BaseLocalPlanner)
PLUGINLIB_EXPORT_CLASS(simple_local_planner::SimplePlannerROS, nav_core::BaseLocalPlanner)

namespace simple_local_planner
{

    SimplePlannerROS::SimplePlannerROS() : costmap_ros_(NULL), initialized_(false) {}

    SimplePlannerROS::~SimplePlannerROS() {}

    void SimplePlannerROS::initialize(std::string name, tf2_ros::Buffer* tfBuffer, costmap_2d::Costmap2DROS* costmapROS)
    {
        if(!initialized_)
        {
            costmap_ros_ = costmapROS;
            tfBuffer_ = tfBuffer;
            tfListener_ = new tf2_ros::TransformListener(*tfBuffer);
            ros::NodeHandle gn;
            odom_sub = gn.subscribe("/agribot/odometry", 100, &SimplePlannerROS::odomCallback, this);
            hasStarted = false;
            initialized_ = true;
            stopTime = 0;
            startTime = 0;
            ROS_DEBUG("Simple Local Planner plugin initialized.");
        }
        else
        {
            ROS_WARN("This planner has already been initialized, doing nothing.");
        }
    }

    bool SimplePlannerROS::setPlan(const std::vector<geometry_msgs::PoseStamped>& orig_global_plan)
    {
        if(!initialized_)
        {
            ROS_ERROR("This planner has not been initialized, please call initialize() before using this planner");
            return false;
        }
        count = 1;
        missed = 0; 
        plan = orig_global_plan; 
        length = plan.size(); 
        ROS_INFO("Received new plan, length = %d", length);
        next=now;
        setNext(); 
        goal_reached_ = false;
        error_integral = 0;
        traveled_distance = 0;
        startTime = ros::Time::now().toSec();
        return true;
    }

    bool SimplePlannerROS::computeVelocityCommands(geometry_msgs::Twist& cmd_vel)
    {
        if(!initialized_)
        {
            ROS_ERROR("This planner has not been initialized, please call initialize() before using this planner");
            return false;
        }
        if(length != 0)
        { 
            setNowError();
            error_integral += fabs(deviation);
           
            // ROS_INFO("count %d missed %d distance %f traveled %f error %f deviation %f angle %f parallelness %f", count, missed, distance, path_traveled, path_error, deviation, nError.az*RAD_TO_DEG, cos(nFramesDifference.az-now.az));
            if(path_traveled >= distance)
            {
                if(path_error <GOAL_DISTANCE_TOLERANCE)
                {
                    if(count<(length-1))
                    {
                        count++;
                        setNext();
                    }
                    else
                    {
                        setVelZeros();
                        goal_reached_ = true;
                        stopTime = ros::Time::now().toSec();
                        ROS_INFO("GOAL REACHED!    Time = %f s    Traveled  %f    Error = %f", stopTime-startTime,traveled_distance, error_integral);
                    }
                }
                else
                {
                    missed++;
                    count++;
                    setNext();
                    ROS_ERROR("OUT OF PATH!");
                }
            }
            else
            {
                setVel();
            }
            if(missed > MISSED_POINTS_TOLLERANCE)
            {
                setVelZeros();
                length = 0;
                stopTime = ros::Time::now().toSec();
                ROS_ERROR("STOPPING, ROBOT MISSED %d POINTS Time = %f    Traveled  %f", missed, stopTime-startTime,traveled_distance);
            }
          }
        ROS_DEBUG("Retrieving velocity command: (%f, %f, %f)", cmd.linear.x, cmd.linear.y, cmd.angular.z);
        cmd_vel = cmd;  
        return true;
    }

    bool SimplePlannerROS::isGoalReached()
    {
        if(!initialized_)
        {
            ROS_ERROR("This planner has not been initialized, please call initialize() before using this planner");
            return false;
        }
        return goal_reached_;
    }

    void SimplePlannerROS::odomCallback(const nav_msgs::Odometry odometry_msg)
    {
        // ROS_INFO("Seq: [%d]", odometry_msg.header.seq);
        pos tmp;
        tmp = now;
        now.x = odometry_msg.pose.pose.position.x;
        now.y = odometry_msg.pose.pose.position.y;
        now.az = getYaw(odometry_msg.pose.pose.orientation); 
        traveled_distance += std::sqrt((tmp.x-now.x)*(tmp.x-now.x) + (tmp.y-now.y)*(tmp.y-now.y));
        setNowError();
        hasStarted = true;
    }

    double SimplePlannerROS::getYaw(geometry_msgs::Quaternion quat)
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

    void SimplePlannerROS::setVel()
    {
        cmd.linear.x= 0.3*(1/(1+deviation));
        double radius;
        double heron;
        heron=(distance+path_error+path_traveled)/2;
        deviation=2*sqrt(heron*(heron-distance)*(heron-path_error)*(heron-path_traveled))/distance;
        
        // // MINIMIZE DEVIATION AND MAXIMIZE PARALLELNESS
        // if (nError.az > 0)
        // {
        // 	cmd.angular.z = 0.5*(2*deviation/(1+deviation)+(1-cos(nFramesDifference.az-now.az))/2)*cmd.linear.x / MIN_RADIUS;
        // }
        // else if (nError.az < 0)
        // {
        // 	cmd.angular.z = -0.5*(2*deviation/(1+deviation)+(1-cos(nFramesDifference.az-now.az))/2)*cmd.linear.x / MIN_RADIUS;
        // }
        
        // ARCH APPROACH
        if (nError.az > 0)
        {	
            if(nError.az < 180*DEG_TO_RAD)
            {
                radius = 0.5*path_error/cos(90*RAD_TO_DEG-fabs(nError.az));
                cmd.angular.z = - cmd.linear.x / radius;
            }
            else
            {
                cmd.angular.z = - 2*cmd.linear.x/CHASSIS_LENGTH;
            }
        }
        else if (nError.az < 0)
        {	
            if(nError.az > -180*DEG_TO_RAD)
            {
                radius = 0.5*path_error/cos(90*RAD_TO_DEG-fabs(nError.az));
                cmd.angular.z = cmd.linear.x / radius;
            }
            else
            {
                cmd.angular.z = 2*cmd.linear.x/CHASSIS_LENGTH;
            }
        }
        else 
        {
            cmd.angular.z = 0;
        }

        if (cmd.linear.x > 0.5)
        {
            cmd.linear.x = 0.5;
        } 
        if (cmd.angular.z > cmd.linear.x / MIN_RADIUS)
        {
            cmd.angular.z = cmd.linear.x / MIN_RADIUS;
        }
        if (cmd.angular.z < -cmd.linear.x / MIN_RADIUS)
        {
            cmd.angular.z = -cmd.linear.x / MIN_RADIUS;
        }
    }


    void SimplePlannerROS::setVelZeros()
    {
        cmd.linear.x= 0;
        cmd.linear.y= 0;
        cmd.angular.z=0;
    }

    void SimplePlannerROS::setNext()
    {
        last = next;
        next.x = plan[count].pose.position.x;
        next.y = plan[count].pose.position.y;
        next.az = getYaw(plan[count].pose.orientation);
    }

    pos SimplePlannerROS::calculateFramesDifference(pos next, pos now)
    {
        pos difference;
        difference.x = (next.x - now.x);
        difference.y = (next.y - now.y);
        difference.az = std::atan2(difference.y, difference.x) - now.az;
        if ( next.az > 180*DEG_TO_RAD ) 
        {
            next.az -= 360*DEG_TO_RAD; 
        }
        if ( next.az < -180*DEG_TO_RAD ) 
        {
            next.az += 360*DEG_TO_RAD;  
        }
        return difference;
    }

    void SimplePlannerROS::setNowError()
    {
        nError = calculateFramesDifference(next, now);
        nPath = calculateFramesDifference(now, last);
        nFramesDifference = calculateFramesDifference(next, last);
        nFramesDifference.az += last.az;
        distance = std::sqrt(nFramesDifference.x*nFramesDifference.x +nFramesDifference.y*nFramesDifference.y);
        path_error = std::sqrt(nError.x*nError.x +nError.y*nError.y);
        path_traveled = std::sqrt(nPath.x*nPath.x +nPath.y*nPath.y);
    }
}