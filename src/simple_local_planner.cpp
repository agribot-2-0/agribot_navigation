#include "agribot_navigation/simple_local_planner.h"

// pluginlib macros (defines, ...)
#include <pluginlib/class_list_macros.h>

#define GOAL_DISTANCE_TOLERANCE 1
#define GOAL_ANGLE_TOLERANCE 360
#define FRAMES_JUMP 20
#define CHASSIS_LENGTH 2.1

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
		plan = orig_global_plan; 
		length = plan.size(); 
		ROS_ERROR("New plan, length = %d", length);
		setNext(); 
		goal_reached_ = false;
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
			ROS_ERROR("count %d distance %f", count, distance);
			if(distance < GOAL_DISTANCE_TOLERANCE)
            { 
				if(count<(length-1))
                {
					if((length - 1 - count) <= FRAMES_JUMP)
                    { 
						count = length - 1;
					}
                    else
                    {
						count += FRAMES_JUMP; 
					}
					setNext();
				}
                else
                {
					setVelZeros();
					goal_reached_ = true;
					ROS_ERROR("GOAL REACHED!");
				}
			}
            else
            {
				setVel();
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
		now.x = odometry_msg.pose.pose.position.x;
		now.y = odometry_msg.pose.pose.position.y;
		now.az = odometry_msg.pose.pose.orientation.z; 
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
		cmd.linear.x= 0.2;
        cmd.angular.z= (nError.az)*0.5;
        // if (nError.az > 0)
		// {
        //     cmd.angular.z= -2*cmd.linear.x/CHASSIS_LENGTH;
        // }
		// else
		// {
        //     cmd.angular.z= 2*cmd.linear.x/CHASSIS_LENGTH;
		// }
		if (cmd.linear.x > 0.5)
		{
			cmd.linear.x = 0.5;
		} 
        if (cmd.angular.z > 2*cmd.linear.x/CHASSIS_LENGTH)
        {
            cmd.angular.z = 2*cmd.linear.x/CHASSIS_LENGTH;
        }
		if (cmd.angular.z < -2*cmd.linear.x/CHASSIS_LENGTH)
        {
            cmd.angular.z = -2*cmd.linear.x/CHASSIS_LENGTH;
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
		next.x = plan[count].pose.position.x;
		next.y = plan[count].pose.position.y;
		next.az = getYaw(plan[count].pose.orientation);
	}

	void SimplePlannerROS::setNowError()
	{
		double d;
		nError.x = (next.x - now.x);
		nError.y = (next.y - now.y);
		if (nError.y == 0 & nError.x == 0)
        {  
			d = now.az;
		}
        else
        {	
			d = std::atan2(nError.y, nError.x);
		}
		distance = std::sqrt(nError.x*nError.x +nError.y*nError.y);
		nError.az = d - now.az;
		// nError.az = next.az - now.az;
		// if ( nError.az > 180*DEG_TO_RAD ) 
        // {
        //     nError.az -= 360*DEG_TO_RAD; 
        // }
		// if ( nError.az < -180*DEG_TO_RAD ) 
        // {
        //     nError.az += 360*DEG_TO_RAD;  
        // }
	}
}