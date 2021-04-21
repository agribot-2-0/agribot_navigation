/*
 * Copyright (C) 2020 Agribot. All rights reserved.
 */

#include <agribot_navigation/move_base_monitor/MoveBaseMonitor.h>
#include <visualization_msgs/MarkerArray.h>

MoveBaseMonitor::MoveBaseMonitor() : _tf2Listener(_tf2Buffer)
{
  _waypointsSub = _nh.subscribe("agribot/waypoints", 1000, &MoveBaseMonitor::waypointCallback, this);
  _stateMachineSub =
      _nh.subscribe("/agribot/move_base_monitor/command", 10, &MoveBaseMonitor::stateMachineCallback, this);
  _stateMachinePub = _nh.advertise<agribot_msgs::AgribotMoveBaseMonitorResult>("/agribot/move_base_monitor/result", 10);

  waypoint_marker_pub_ = _nh.advertise<visualization_msgs::MarkerArray>("/agribot/move_base_monitor/waypoints", 1);
  plan_pub_ = _nh.advertise<nav_msgs::Path>("/agribot/move_base_monitor/path", 1);

  _nh.param("ray_width", ray_width_, M_PI_2);

  // tell the action client that we want to spin a thread by default
  ac = new MoveBaseClient("move_base", true);
  // wait for the action server to come up
  while (!ac->waitForServer(ros::Duration(5.0)))
  {
    ROS_INFO_STREAM("[MoveBaseMonitor]Waiting for the move_base"
                    << "action server to come up");
  }

  _nh.setParam("/agribot/move_base_monitor/is_ready", true);

  _client_navigation = _nh.serviceClient<std_srvs::Empty>("/cmd_vel_arbiter_node/switch_to_navigation");
  _client_stop = _nh.serviceClient<std_srvs::Empty>("/cmd_vel_arbiter_node/switch_to_stop");

  resultMsg.result = resultMsg.NOACTION;
  resultMsg.current_waypoint = 0;
  resultMsg.total_waypoints = 0;
  _pauseGoal = false;
  _pauseGoalLatch = false;
  _pathLoaded = false;
  _userAbort = false;

  bool transformStatus = false;

  while (!transformStatus)
  {
    transformStatus = getMapUTMTransform();
    if (transformStatus == false)
    {
      ROS_WARN("UTM -> map transform was not available. Retrying!");
    }
  }
  ROS_INFO("UTM -> map transform obtained!");
}

void MoveBaseMonitor::stateMachineCallback(const agribot_msgs::AgribotMoveBaseCommand& command_msg)
{
  // if the path is blocked then cancel the current goal and set the pause flag
  if (command_msg.command == command_msg.PAUSE)
  {
    _pauseGoalLatch = true;
    _pauseGoal = true;
  }
  else if (command_msg.command == command_msg.CONTINUE)
  {
    _pauseGoal = false;
  }
  // if we want to abort the current task by user request
  else if (command_msg.command == command_msg.ABORT)
  {
    _userAbort = true;
  }
}

void MoveBaseMonitor::isPaused()
{
  bool cancelTrig = true;
  while (_pauseGoal)
  {
    if (cancelTrig)
    {
      ros::service::waitForService("/cmd_vel_arbiter_node/switch_to_stop");
      _client_stop.call(_reset_srv);
      ROS_INFO_STREAM("[MoveBaseMonitor] Goal PAUSED");
      cancelTrig = false;
    }
    resultMsg.result = resultMsg.PAUSED;
    _stateMachinePub.publish(resultMsg);
    ros::Duration(0.1).sleep();  // sleep for half a second
    ros::spinOnce();
    if (_userAbort)
      break;
  }
  if (_pauseGoalLatch && !(_userAbort))
  {
    ros::service::waitForService("/cmd_vel_arbiter_node/switch_to_navigation");
    _client_navigation.call(_reset_srv);
    _pauseGoalLatch = false;
  }
}

void MoveBaseMonitor::waypointCallback(const agribot_msgs::NavSatFixOrientation& waypoint)
{
  if (clear_waypoints_)
  {
    waypoints_.clear();
    clear_waypoints_ = false;
  }

  // Convert NavSatFix to UTM
  geometry_msgs::PoseStamped UTM_point = navSatFixToUTM(waypoint);

  // Convert UTM point to map frame
  geometry_msgs::PoseStamped map_point = UTMToMap(UTM_point);

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
    _pathLoaded = true;
    _userAbort = false;
  }
}

double MoveBaseMonitor::limitAngle(double angle) const
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

double MoveBaseMonitor::angleBetweenPoses(const geometry_msgs::Pose& target_pose,
                                          const geometry_msgs::Pose& current_pose) const
{
  return std::atan2(target_pose.position.y - current_pose.position.y, target_pose.position.x - current_pose.position.x);
}

double MoveBaseMonitor::distanceBetweenPoses(const geometry_msgs::Pose& target_pose,
                                             const geometry_msgs::Pose& current_pose) const
{
  return hypot(target_pose.position.y - current_pose.position.y, target_pose.position.x - current_pose.position.x);
}

double MoveBaseMonitor::calculateLinesSpan(const double target_line_angle, const double base_line_angle) const
{
  return abs(limitAngle(target_line_angle - base_line_angle));
}

void MoveBaseMonitor::updateOrientations(nav_msgs::Path& path, const double ray_width) const
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

void MoveBaseMonitor::createAndPublishMarkersFromPath(const std::vector<geometry_msgs::PoseStamped>& path)
{
  // clear previous markers
  visualization_msgs::MarkerArray markers;
  visualization_msgs::Marker marker;
  marker.header = path[0].header;
  marker.ns = "/move_base_monitor/waypoints";
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
  waypoint_marker_pub_.publish(markers);
  marker.action = visualization_msgs::Marker::ADD;
  markers.markers.clear();

  for (size_t i = 0; i < path.size(); i++)
  {
    marker.id = i;
    marker.pose.position = path[i].pose.position;
    marker.pose.orientation = path[i].pose.orientation;
    markers.markers.push_back(marker);
  }

  waypoint_marker_pub_.publish(markers);
}

void MoveBaseMonitor::monitorPathWaypoints()
{
  move_base_msgs::MoveBaseGoal goal;
  actionlib::SimpleClientGoalState::StateEnum goal_result;
  ROS_INFO_STREAM("[MoveBaseMonitor] Start sending goals");
  _nh.setParam("/agribot/move_base_monitor/is_ready", false);
  uint currentWaypoint = 0;
  bool load_point = true;
  auto pose = path_.poses[currentWaypoint];
  ros::Rate loop_rate(10);
  while (currentWaypoint < path_.poses.size())
  {
    publishWaypointsVisualization(path_);
    resultMsg.current_waypoint = currentWaypoint;
    resultMsg.total_waypoints = path_.poses.size();
    // check if the current goal was paused, if so then wait
    isPaused();
    // check if the current path is aborted
    if (_userAbort)
    {
      ac->cancelAllGoals();
      break;
    }
    // check if the next point should be loaded
    if (load_point)
    {
      pose = path_.poses[currentWaypoint];
      ROS_INFO_STREAM("[MoveBaseMonitor] Point " << pose.pose.position.x << ", " << pose.pose.position.y << " Index "
                                                 << currentWaypoint);
      goal.target_pose = pose;
      ac->sendGoal(goal);
      load_point = false;
    }
    else
    {
      goal_result = ac->getState().state_;
      if (goal_result == actionlib::SimpleClientGoalState::ACTIVE ||
          goal_result == actionlib::SimpleClientGoalState::PENDING)
      {
        resultMsg.result = resultMsg.ACTIVE;
        _stateMachinePub.publish(resultMsg);
      }
      else if (goal_result == actionlib::SimpleClientGoalState::SUCCEEDED)
      {
        resultMsg.result = resultMsg.SUCCES;
        _stateMachinePub.publish(resultMsg);
        ROS_INFO_STREAM("[MoveBaseMonitor] Goal succeeded, "
                        << "taking the next goal");
        currentWaypoint++;
        load_point = true;
      }
      else if (goal_result == actionlib::SimpleClientGoalState::PREEMPTED)
      {
        resultMsg.result = resultMsg.PREEMPTED;
        _stateMachinePub.publish(resultMsg);
        ROS_INFO_STREAM("[MoveBaseMonitor] Goal PREEMPTED, "
                        << "trying next goal");
        currentWaypoint++;
        load_point = true;
      }
      else if (goal_result == actionlib::SimpleClientGoalState::ABORTED)
      {
        resultMsg.result = resultMsg.ABORTED;
        _stateMachinePub.publish(resultMsg);
        currentWaypoint++;
        load_point = true;
        ROS_INFO_STREAM("[MoveBaseMonitor] The goal was ABORTED!, "
                        << "trying next goal");
      }
      else if (goal_result == actionlib::SimpleClientGoalState::REJECTED)
      {
        resultMsg.result = resultMsg.FAILED;
        _stateMachinePub.publish(resultMsg);
        currentWaypoint++;
        load_point = true;
        ROS_INFO_STREAM("[MoveBaseMonitor] The goal was REJECTED!, "
                        << "trying next goal");
      }
      else if (goal_result == actionlib::SimpleClientGoalState::RECALLED)
      {
        resultMsg.result = resultMsg.FAILED;
        _stateMachinePub.publish(resultMsg);
        ROS_INFO_STREAM("[MoveBaseMonitor] The goal was RECALLED!,"
                        << "trying next goal");
        currentWaypoint++;
        load_point = true;
      }
      else if (goal_result == actionlib::SimpleClientGoalState::LOST)
      {
        resultMsg.result = resultMsg.FAILED;
        _stateMachinePub.publish(resultMsg);
        ROS_INFO_STREAM("[MoveBaseMonitor] The goal was LOST!,"
                        << "trying next goal");
        currentWaypoint++;
        load_point = true;
      }
    }
    ros::spinOnce();
    loop_rate.sleep();
  }
  clear_waypoints_ = true;
  _pathLoaded = false;
  _pauseGoal = false;
  _pauseGoalLatch = false;
  if (_userAbort == true)
  {
    _userAbort = false;
    ROS_INFO("[MoveBaseMonitor] Path aborted by user");
  }
  else
  {
    resultMsg.result = resultMsg.PATH_COMPLETED;
    _stateMachinePub.publish(resultMsg);
    ROS_INFO("[MoveBaseMonitor] Path ended");
  }
  _nh.setParam("/agribot/move_base_monitor/is_ready", true);
}

bool MoveBaseMonitor::getMapUTMTransform(void)
{
  try
  {
    _startingMapUTMTransform = _tf2Buffer.lookupTransform("map", "utm", ros::Time(0), ros::Duration(1.0));
  }
  catch (tf2::TransformException& ex)
  {
    return false;
  }
  return true;
}

geometry_msgs::PoseStamped MoveBaseMonitor::navSatFixToUTM(const agribot_msgs::NavSatFixOrientation& msg)
{
  double utm_x, utm_y;
  std::string utm_zone;
  geometry_msgs::PoseStamped UTM_point;

  // Convert Latitude/Longitude from NavSatFix to UTM coordinates
  RobotLocalization::NavsatConversions::LLtoUTM(msg.latitude, msg.longitude, utm_y, utm_x, utm_zone);

  // Fill in UTM_point variable
  UTM_point.header.frame_id = "utm";
  UTM_point.header.stamp = ros::Time::now();
  UTM_point.pose.position.x = utm_x;
  UTM_point.pose.position.y = utm_y;
  UTM_point.pose.position.z = msg.altitude;
  UTM_point.pose.orientation = msg.orientation;

  return UTM_point;
}

geometry_msgs::PoseStamped MoveBaseMonitor::UTMToMap(const geometry_msgs::PoseStamped& msg)
{
  geometry_msgs::PoseStamped map_point;
  // Use _startingMapUTMTransform generated in constructor
  tf2::doTransform(msg, map_point, _startingMapUTMTransform);
  return map_point;
}

void MoveBaseMonitor::publishWaypointsVisualization(const nav_msgs::Path& path)
{
  createAndPublishMarkersFromPath(path.poses);
  plan_pub_.publish(path);
}

void MoveBaseMonitor::run()
{
  ros::Rate loop_rate(10);
  while (ros::ok())
  {
    if (_pathLoaded)
    {
      monitorPathWaypoints();
    }
    loop_rate.sleep();
    ros::spinOnce();
  }
}
