/*
 * Copyright (C) 2020 Agribot. All rights reserved.
 */

#include <agribot_navigation/navigation_manager/NavigationManager.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "navigation_manager_node");
  NavigationManager navigation_manager;
  navigation_manager.run();
  return 0;
}
