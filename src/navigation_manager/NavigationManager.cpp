/*
 * Copyright (C) 2020 Agribot. All rights reserved.
 */

#include <agribot_navigation/navigation_manager/NavigationManager.h>
#include <visualization_msgs/MarkerArray.h>

NavigationManager::NavigationManager() : tf_to_listener_(tf_to_buffer_)
{
  waypoints_subscriber_ = node_handle_.subscribe("agribot/waypoints", 1000, &NavigationManager::waypointCallback, this);
  state_machine_subscriber_ =
      node_handle_.subscribe("/agribot/navigation_manager/command", 10, &NavigationManager::stateMachineCallback, this);
  state_machine_publisher_ =
      node_handle_.advertise<agribot_msgs::AgribotNavigationManagerResult>("/agribot/navigation_manager/result", 10);

  waypoint_marker_publisher_ =
      node_handle_.advertise<visualization_msgs::MarkerArray>("/agribot/navigation_manager/waypoints", 1);
  plan_publisher_ = node_handle_.advertise<nav_msgs::Path>("/agribot/navigation_manager/path", 1);

  node_handle_.param("ray_width", ray_width_, M_PI_2);

  // tell the action client that we want to spin a thread by default
  move_base_action_client_ =
      std::make_unique<actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction>>("move_base", true);
  // wait for the action server to come up
  while (!move_base_action_client_->waitForServer(ros::Duration(5.0)))
  {
    ROS_INFO_STREAM("[NavigationManager]Waiting for the move_base"
                    << "action server to come up");
  }

  node_handle_.setParam("/agribot/navigation_manager/is_ready", true);

  client_navigation_ = node_handle_.serviceClient<std_srvs::Empty>("/cmd_vel_arbiter_node/switch_to_navigation");
  client_stop_ = node_handle_.serviceClient<std_srvs::Empty>("/cmd_vel_arbiter_node/switch_to_stop");

  result_msg_.result = result_msg_.NOACTION;
  result_msg_.current_waypoint = 0;
  result_msg_.total_waypoints = 0;
  pause_goal_ = false;
  pause_goal_latch_ = false;
  path_loaded_ = false;
  user_abort_ = false;

  bool transformStatus = false;

  while (!transformStatus)
  {
    transformStatus = getMapUtmTransform();
    if (transformStatus == false)
    {
      ROS_WARN("[NavigationManager] UTM -> map transform was not available. Retrying!");
    }
  }
  ROS_INFO("[NavigationManager] UTM -> map transform obtained!");
}

void NavigationManager::stateMachineCallback(const agribot_msgs::AgribotMoveBaseCommand& command_msg)
{
  // if the path is blocked then cancel the current goal and set the pause flag
  if (command_msg.command == command_msg.PAUSE)
  {
    pause_goal_latch_ = true;
    pause_goal_ = true;
  }
  else if (command_msg.command == command_msg.CONTINUE)
  {
    pause_goal_ = false;
  }
  // if we want to abort the current task by user request
  else if (command_msg.command == command_msg.ABORT)
  {
    user_abort_ = true;
  }
}

void NavigationManager::isPaused()
{
  bool cancel_trig = true;
  while (pause_goal_)
  {
    if (cancel_trig)
    {
      ros::service::waitForService("/cmd_vel_arbiter_node/switch_to_stop");
      client_stop_.call(reset_srv_);
      ROS_INFO_STREAM("[NavigationManager] Goal PAUSED");
      cancel_trig = false;
    }
    result_msg_.result = result_msg_.PAUSED;
    state_machine_publisher_.publish(result_msg_);
    ros::Duration(0.1).sleep();  // sleep for half a second
    ros::spinOnce();
    if (user_abort_)
      break;
  }
  if (pause_goal_latch_ && !(user_abort_))
  {
    ros::service::waitForService("/cmd_vel_arbiter_node/switch_to_navigation");
    client_navigation_.call(reset_srv_);
    pause_goal_latch_ = false;
  }
}

void NavigationManager::waypointCallback(const sensor_msgs::NavSatFix& waypoint)
{
  if (clear_waypoints_)
  {
    waypoints_.clear();
    clear_waypoints_ = false;
  }

  // Convert NavSatFix to UTM
  geometry_msgs::PoseStamped utm_point = navSatFixToUtm(waypoint);

  // Convert UTM point to map frame
  geometry_msgs::PoseStamped map_point = utmToMap(utm_point);

  // add waypoint to the waypoint vector
  waypoints_.push_back(map_point);

  if (waypoints_.size() < 2)
    return;

  geometry_msgs::Pose* p1 = &(waypoints_.end() - 2)->pose;
  geometry_msgs::Pose* p2 = &(waypoints_.end() - 1)->pose;

  // calculate distance between latest two waypoints
  // and check if it surpasses the threshold epsilon
  double dist = hypot(p1->position.x - p2->position.x, p1->position.y - p2->position.y);

  if (dist < 0.01)
  {
    p2->orientation = p1->orientation;
    path_.header = map_point.header;
    path_.poses.clear();
    path_.poses.insert(path_.poses.end(), waypoints_.begin(), waypoints_.end());
    path_.poses.pop_back();

    updateOrientations(path_, ray_width_);

    publishWaypointsVisualization(path_);
    path_loaded_ = true;
    user_abort_ = false;
  }
}

double NavigationManager::limitAngle(double angle) const
{
  while (angle > M_PI)
  {
    angle -= M_PI * 2.0;
  }
  while (angle < -M_PI)
  {
    angle += M_PI * 2.0;
  }
  return angle;
}

double NavigationManager::angleBetweenPoses(const geometry_msgs::Pose& target_pose,
                                            const geometry_msgs::Pose& current_pose) const
{
  return std::atan2(target_pose.position.y - current_pose.position.y, target_pose.position.x - current_pose.position.x);
}

double NavigationManager::distanceBetweenPoses(const geometry_msgs::Pose& target_pose,
                                               const geometry_msgs::Pose& current_pose) const
{
  return hypot(target_pose.position.y - current_pose.position.y, target_pose.position.x - current_pose.position.x);
}

double NavigationManager::calculateLinesSpan(const double target_line_angle, const double base_line_angle) const
{
  return abs(limitAngle(target_line_angle - base_line_angle));
}

void NavigationManager::updateOrientations(nav_msgs::Path& path, const double ray_width) const
{
  path.poses[0].pose.orientation =
      tf::createQuaternionMsgFromYaw(angleBetweenPoses(path.poses[1].pose, path.poses[0].pose));
  for (std::size_t i = 1; i < path.poses.size() - 1; i++)
  {
    double last_yaw = tf::getYaw(path.poses[i - 1].pose.orientation);
    double left_ray_angle = limitAngle(last_yaw + M_PI_2);
    double right_ray_angle = limitAngle(last_yaw - M_PI_2);
    double waypoint_orientation = angleBetweenPoses(path.poses[i].pose, path.poses[i - 1].pose);
    if ((calculateLinesSpan(waypoint_orientation, left_ray_angle) <= ray_width / 2.0) ||
        (calculateLinesSpan(waypoint_orientation, right_ray_angle) <= ray_width / 2.0))
    {
      path.poses[i].pose.orientation =
          tf::createQuaternionMsgFromYaw(angleBetweenPoses(path.poses[i + 1].pose, path.poses[i].pose));
    }
    else
    {
      path.poses[i].pose.orientation =
          tf::createQuaternionMsgFromYaw(angleBetweenPoses(path.poses[i].pose, path.poses[i - 1].pose));
    }
  }
  path.poses[path.poses.size() - 1].pose.orientation = tf::createQuaternionMsgFromYaw(
      angleBetweenPoses(path.poses[path.poses.size() - 1].pose, path.poses[path.poses.size() - 2].pose));
  return;
}

void NavigationManager::createAndPublishMarkersFromPath(const std::vector<geometry_msgs::PoseStamped>& path)
{
  // clear previous markers
  visualization_msgs::MarkerArray markers;
  visualization_msgs::Marker marker;
  marker.header = path[0].header;
  marker.ns = "/navigation_manager/waypoints";
  marker.type = visualization_msgs::Marker::ARROW;
  marker.action = visualization_msgs::Marker::DELETEALL;
  marker.scale.x = 0.75;
  marker.scale.y = 0.1;
  marker.scale.z = 0.01;
  marker.color.a = 1.0;
  marker.color.r = 0.1;
  marker.color.g = 0.2;
  marker.color.b = 0.7;
  marker.id = 0;
  markers.markers.push_back(marker);
  waypoint_marker_publisher_.publish(markers);
  marker.action = visualization_msgs::Marker::ADD;
  markers.markers.clear();

  for (size_t i = 0; i < path.size(); i++)
  {
    marker.id = i;
    marker.pose.position = path[i].pose.position;
    marker.pose.orientation = path[i].pose.orientation;
    markers.markers.push_back(marker);
  }

  waypoint_marker_publisher_.publish(markers);
}

void NavigationManager::monitorPathWaypoints()
{
  move_base_msgs::MoveBaseGoal goal;
  actionlib::SimpleClientGoalState::StateEnum goal_result;
  ROS_INFO_STREAM("[NavigationManager] Start sending goals");
  node_handle_.setParam("/agribot/navigation_manager/is_ready", false);
  int current_waypoint = 0;
  bool load_point = true;
  auto pose = path_.poses[current_waypoint];
  ros::Rate loop_rate(10);
  while (current_waypoint < path_.poses.size())
  {
    publishWaypointsVisualization(path_);
    result_msg_.current_waypoint = current_waypoint;
    result_msg_.total_waypoints = path_.poses.size();
    // check if the current goal was paused, if so then wait
    isPaused();
    // check if the current path is aborted
    if (user_abort_)
    {
      move_base_action_client_->cancelAllGoals();
      break;
    }
    // check if the next point should be loaded
    if (load_point)
    {
      pose = path_.poses[current_waypoint];
      ROS_INFO_STREAM("[NavigationManager] Point " << pose.pose.position.x << ", " << pose.pose.position.y << " Index "
                                                   << current_waypoint);
      goal.target_pose = pose;
      move_base_action_client_->sendGoal(goal);
      load_point = false;
    }
    else
    {
      goal_result = move_base_action_client_->getState().state_;
      if (goal_result == actionlib::SimpleClientGoalState::ACTIVE ||
          goal_result == actionlib::SimpleClientGoalState::PENDING)
      {
        result_msg_.result = result_msg_.ACTIVE;
        state_machine_publisher_.publish(result_msg_);
      }
      else if (goal_result == actionlib::SimpleClientGoalState::SUCCEEDED)
      {
        result_msg_.result = result_msg_.SUCCES;
        state_machine_publisher_.publish(result_msg_);
        ROS_INFO_STREAM("[NavigationManager] Goal succeeded, "
                        << "taking the next goal");
        current_waypoint++;
        load_point = true;
      }
      else if (goal_result == actionlib::SimpleClientGoalState::PREEMPTED)
      {
        result_msg_.result = result_msg_.PREEMPTED;
        state_machine_publisher_.publish(result_msg_);
        ROS_INFO_STREAM("[NavigationManager] Goal PREEMPTED, "
                        << "trying next goal");
        current_waypoint++;
        load_point = true;
      }
      else if (goal_result == actionlib::SimpleClientGoalState::ABORTED)
      {
        result_msg_.result = result_msg_.ABORTED;
        state_machine_publisher_.publish(result_msg_);
        current_waypoint++;
        load_point = true;
        ROS_INFO_STREAM("[NavigationManager] The goal was ABORTED!, "
                        << "trying next goal");
      }
      else if (goal_result == actionlib::SimpleClientGoalState::REJECTED)
      {
        result_msg_.result = result_msg_.FAILED;
        state_machine_publisher_.publish(result_msg_);
        current_waypoint++;
        load_point = true;
        ROS_INFO_STREAM("[NavigationManager] The goal was REJECTED!, "
                        << "trying next goal");
      }
      else if (goal_result == actionlib::SimpleClientGoalState::RECALLED)
      {
        result_msg_.result = result_msg_.FAILED;
        state_machine_publisher_.publish(result_msg_);
        ROS_INFO_STREAM("[NavigationManager] The goal was RECALLED!,"
                        << "trying next goal");
        current_waypoint++;
        load_point = true;
      }
      else if (goal_result == actionlib::SimpleClientGoalState::LOST)
      {
        result_msg_.result = result_msg_.FAILED;
        state_machine_publisher_.publish(result_msg_);
        ROS_INFO_STREAM("[NavigationManager] The goal was LOST!,"
                        << "trying next goal");
        current_waypoint++;
        load_point = true;
      }
    }
    ros::spinOnce();
    loop_rate.sleep();
  }
  clear_waypoints_ = true;
  path_loaded_ = false;
  pause_goal_ = false;
  pause_goal_latch_ = false;
  if (user_abort_ == true)
  {
    user_abort_ = false;
    ROS_INFO("[NavigationManager] Path aborted by user");
  }
  else
  {
    result_msg_.result = result_msg_.PATH_COMPLETED;
    state_machine_publisher_.publish(result_msg_);
    ROS_INFO("[NavigationManager] Path ended");
  }
  node_handle_.setParam("/agribot/navigation_manager/is_ready", true);
}

bool NavigationManager::getMapUtmTransform(void)
{
  try
  {
    starting_map_utm_transform_ = tf_to_buffer_.lookupTransform("map", "utm", ros::Time(0), ros::Duration(1.0));
  }
  catch (tf2::TransformException& ex)
  {
    return false;
  }
  return true;
}

geometry_msgs::PoseStamped NavigationManager::navSatFixToUtm(const sensor_msgs::NavSatFix& msg)
{
  double utm_x, utm_y;
  std::string utm_zone;
  geometry_msgs::PoseStamped utm_point;

  // Convert Latitude/Longitude from NavSatFix to UTM coordinates
  RobotLocalization::NavsatConversions::LLtoUTM(msg.latitude, msg.longitude, utm_y, utm_x, utm_zone);

  // Fill in utm_point variable
  utm_point.header.frame_id = "utm";
  utm_point.header.stamp = ros::Time::now();
  utm_point.pose.position.x = utm_x;
  utm_point.pose.position.y = utm_y;
  utm_point.pose.position.z = msg.altitude;

  return utm_point;
}

geometry_msgs::PoseStamped NavigationManager::utmToMap(const geometry_msgs::PoseStamped& msg)
{
  geometry_msgs::PoseStamped map_point;
  // Use starting_map_utm_transform_ generated in constructor
  tf2::doTransform(msg, map_point, starting_map_utm_transform_);
  return map_point;
}

void NavigationManager::publishWaypointsVisualization(const nav_msgs::Path& path)
{
  createAndPublishMarkersFromPath(path.poses);
  plan_publisher_.publish(path);
}

void NavigationManager::run()
{
  ros::Rate loop_rate(10);
  while (ros::ok())
  {
    if (path_loaded_)
    {
      monitorPathWaypoints();
    }
    loop_rate.sleep();
    ros::spinOnce();
  }
}
