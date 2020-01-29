#include "agribot_navigation/simple_local_planner.h"

#include <pluginlib/class_list_macros.h>
#include <cmath>

#define ARROW_LENGTH 0.5
#define GOAL_DISTANCE_TOLERANCE 0.5
#define GOAL_ANGLE_TOLERANCE 360
#define MISSED_POINTS_TOLLERANCE 2
#define CHASSIS_LENGTH 2.1
#define MIN_RADIUS 2
#define PREDICTION_TIME 30
#define PREDICTION_POINTS_PER_SECOND 2

// Planner approach choice
// #define ARCH_APPROACH
// #define SPEED_LIMIT

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
            path_pub = gn.advertise<visualization_msgs::Marker>("travelled_path", 10);
            predicted_path_pub = gn.advertise<visualization_msgs::Marker>("predicted_path", 10);
            next_goal_pub = gn.advertise<visualization_msgs::Marker>("next_goal", 10);

            initialized_ = true;

            stopTime = 0;
            startTime = 0;
            lastTime = 0;
            actualTime = 0;

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
        goal_reached_ = false;
        traveledPoints=visualization_msgs::Marker();

        last = now;
        next = getFrame(count); 

        errorIntegral = 0;
        traveledDistance = 0;

        startTime = ros::Time::now().toSec();
        lastTime = 0;
        actualTime = 0;

        ROS_INFO("Received new plan, length = %d", length);
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
            // Check where robot is
            calculateErrors();

            // Deviation error for statistics
            if(actualTime > 0)
            {
                if (!isnan(deviation))
                {
                    lastTime = actualTime;
                    actualTime = ros::Time::now().toSec();
                    errorIntegral += fabs(deviation)*(actualTime-lastTime);
                }
            }
            else
            {
                actualTime = ros::Time::now().toSec();
            }

            #ifdef DEBUG
                ROS_INFO("count %d missed %d distance %f traveled %f error %f deviation %f parallelness %f", count, missed, distance, path_traveled, path_error, deviation, cos(last.az-now.az));
            #endif

            // Determine behaviour basing on location
            if(path_traveled >= distance)
            {
                // Traveled more than distance between last and next waypoints
                if(path_error < GOAL_DISTANCE_TOLERANCE)
                {
                    // Robot is within error range from goal waypoint
                    if(count<(length-1))
                    {
                        // It is not the last point of the path
                        count++;
                        #ifdef ARCH_APPROACH
                            last = goal;
                        #else
                            last = next;
                        #endif
                        next = getFrame(count);
                    }
                    else
                    {
                        // Reached final destionation
                        setVelZeros();
                        goal_reached_ = true;
                        stopTime = ros::Time::now().toSec();
                        ROS_INFO("GOAL REACHED!    Time = %f s    Traveled  %f    Error = %f", stopTime-startTime,traveledDistance, errorIntegral);
                    }
                }
                else
                {
                    // Didn't hit the goal waypoint -> skip it and go to next one
                    missed++;
                    count++;
                    #ifdef ARCH_APPROACH
                        last = goal;
                    #else
                        last = next;
                    #endif
                    next = getFrame(count);
                    ROS_ERROR("OUT OF PATH!");
                }
            }
            else
            {
                // Somewhere between last and next waypoint
                setVel();
            }

            // Behaviour after missing too many points
            if(missed > MISSED_POINTS_TOLLERANCE)
            {
                setVelZeros();
                length = 0;
                stopTime = ros::Time::now().toSec();
                ROS_ERROR("STOPPING, ROBOT MISSED %d POINTS Time = %f    Traveled  %f", missed, stopTime-startTime,traveledDistance);
            }
        }
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
        pos tmp;
        tmp = now;
        now.x = odometry_msg.pose.pose.position.x;
        now.y = odometry_msg.pose.pose.position.y;
        now.az = getYaw(odometry_msg.pose.pose.orientation); 
        traveledDistance += std::sqrt((tmp.x-now.x)*(tmp.x-now.x) + (tmp.y-now.y)*(tmp.y-now.y));

        // Calculate location ofrobot
        calculateErrors();

        // Visualization
        pathVisualization();
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
        #ifdef SPEED_LIMIT
            cmd.linear.x= 0.3*(1/(1+deviation));
        #else
            cmd.linear.x= 0.3;
        #endif
        
        #ifdef ARCH_APPROACH
            double radius;
            if (nGoal.az > 0)
            {	
                if(nGoal.az < 180*DEG_TO_RAD)
                {
                    radius = 0.5*path_error/cos(90*RAD_TO_DEG-fabs(nGoal.az));
                    cmd.angular.z = - cmd.linear.x / radius;
                }
                else
                {
                    cmd.angular.z = - 2*cmd.linear.x/CHASSIS_LENGTH;
                }
            }
            else if (nGoal.az < 0)
            {	
                if(nGoal.az > -180*DEG_TO_RAD)
                {
                    radius = 0.5*path_error/cos(90*RAD_TO_DEG-fabs(nGoal.az));
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
        #else
            if (nError.az > 0)
            {
                cmd.angular.z = 0.5*(2*deviation/(1+deviation)+(1-cos(last.az-now.az))/2)*cmd.linear.x / MIN_RADIUS;
            }
            else if (nError.az < 0)
            {
                cmd.angular.z = -0.5*(2*deviation/(1+deviation)+(1-cos(last.az-now.az))/2)*cmd.linear.x / MIN_RADIUS;
            }
        #endif

        // Velocity limiter
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

        // Visualization
        predictedPathVisualization();
    }


    void SimplePlannerROS::setVelZeros()
    {
        cmd.linear.x= 0;
        cmd.linear.y= 0;
        cmd.angular.z=0;
    }

    pos SimplePlannerROS::getFrame(int count)
    {
        pos tmp;
        tmp.x = plan[count].pose.position.x;
        tmp.y = plan[count].pose.position.y;
        tmp.az = getYaw(plan[count].pose.orientation);
        return tmp;
    }

    double SimplePlannerROS::limitAngle(double angle)
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

    pos SimplePlannerROS::calculateFramesDifference(pos next, pos prev)
    {
        pos difference;
        difference.x = (next.x - prev.x);
        difference.y = (next.y - prev.y);
        difference.az = std::atan2(difference.y, difference.x) - prev.az;
        difference.az = limitAngle(difference.az);
        return difference;
    }

    void SimplePlannerROS::calculateErrors()
    {
        calculateGoal();
        nError = calculateFramesDifference(next, now);
        nPath = calculateFramesDifference(now, last);
        nFramesDifference = calculateFramesDifference(next, last);
        nGoal = calculateFramesDifference(goal, now);

        nFramesDifference.az = limitAngle(nFramesDifference.az - last.az);

        distance = std::sqrt(nFramesDifference.x * nFramesDifference.x + nFramesDifference.y * nFramesDifference.y);
        path_error = std::sqrt(nError.x * nError.x + nError.y * nError.y);
        path_traveled = std::sqrt(nPath.x * nPath.x + nPath.y * nPath.y);

        // calculating triangle area (known length of all sides) to compute one of the heights afterwards
        double heron=(distance+path_error+path_traveled)/2;
        deviation=2*sqrt(heron*(heron-distance)*(heron-path_error)*(heron-path_traveled))/distance;
    }

    void SimplePlannerROS::calculateGoal()
    {
        #ifdef ARCH_APPROACH
            if(count<(length-1))
            {
                pos tmp;
                tmp = getFrame(count+1);

                double length_tmp_next = std::sqrt((tmp.x-next.x)*(tmp.x-next.x) + (tmp.y-next.y)*(tmp.y-next.y));
                double length_last_next = std::sqrt((last.x-next.x)*(last.x-next.x) + (last.y-next.y)*(last.y-next.y));

                double delta_x = ((tmp.x-next.x)/length_tmp_next + (last.x-next.x)/length_last_next);
                double delta_y = ((tmp.y-next.y)/length_tmp_next + (last.y-next.y)/length_last_next);
                double delta_len = std::sqrt(delta_x*delta_x + delta_y*delta_y);
                
                delta_x = delta_x / delta_len * GOAL_DISTANCE_TOLERANCE /2;
                delta_y = delta_y / delta_len * GOAL_DISTANCE_TOLERANCE /2;

                goal.x = next.x + delta_x;
                goal.y = next.y + delta_y;
            }
            else
            {
                goal = next;
            }
        #else
            goal = next;
        #endif

        goal.az = std::atan2(goal.y - last.y, goal.x - last.x);

        // Visualization
        nextGoalVisualization(goal);
    }

    void SimplePlannerROS::pathVisualization()
    {
        traveledPoints.header.frame_id = "/odom";
        traveledPoints.header.stamp = ros::Time::now();
        traveledPoints.ns = "travelled_path";
        traveledPoints.action = visualization_msgs::Marker::ADD;
        traveledPoints.id = 0;
        traveledPoints.type = visualization_msgs::Marker::LINE_STRIP;
        traveledPoints.scale.x = 0.1;
        traveledPoints.scale.y = 0.1;
        traveledPoints.color.b = 1.0;
        traveledPoints.color.a = 1.0;
        geometry_msgs::Point p;
        p.x = now.x;
        p.y = now.y;
        p.z = 0.1;
        traveledPoints.points.push_back(p);
        path_pub.publish(traveledPoints); 
     }

    void SimplePlannerROS::predictedPathVisualization()
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
        if(cmd.angular.z != 0)
        {
            radius = cmd.linear.x / cmd.angular.z;
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
                theta = i*cmd.angular.z/PREDICTION_POINTS_PER_SECOND;
                tmp.x = radius*cos(theta-90*DEG_TO_RAD)-radius*cos(-90*DEG_TO_RAD);
                tmp.y = radius*sin(theta-90*DEG_TO_RAD)-radius*sin(-90*DEG_TO_RAD);

                p.x = now.x + tmp.x * cos(now.az) - tmp.y * sin(now.az);
                p.y = now.y + tmp.x * sin(now.az) + tmp.y * cos(now.az);

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
                p.x = now.x+i*cmd.linear.x/PREDICTION_POINTS_PER_SECOND*cos(now.az);
                p.y = now.y+i*cmd.linear.x/PREDICTION_POINTS_PER_SECOND*cos(now.az);
                p.z = 0.1;
                futurePoints.points.push_back(p);

            }
        }
        predicted_path_pub.publish(futurePoints); 
     }

     void SimplePlannerROS::nextGoalVisualization(pos goal)
    {
        visualization_msgs::Marker marker;
        marker.header.frame_id = "/odom";
        marker.header.stamp = ros::Time::now();
        marker.ns = "next_goal";
        marker.action = visualization_msgs::Marker::ADD;
        marker.id = 2;
        marker.type = visualization_msgs::Marker::SPHERE;
        marker.scale.x = 0.1;
        marker.scale.y = 0.1;
        marker.scale.z = 0;
        marker.color.b = 1;
        marker.color.r = 1;
        marker.color.a = 1.0;

        marker.pose.position.x = goal.x;
        marker.pose.position.y = goal.y;
        marker.pose.position.z = 0.1;
        
        predicted_path_pub.publish(marker); 
     }
     
}