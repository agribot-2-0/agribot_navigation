/*
 * Copyright (C) 2020 Agribot. All rights reserved.
 */

#include <agribot_navigation/move_base_monitor/MoveBaseMonitor.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "move_base_monitor_node");
  MoveBaseMonitor _moveBaseMonitor;
  _moveBaseMonitor.run();
  return 0;
}
