cmake_minimum_required(VERSION 2.8.3)
project(agribot_navigation)

## Compilation options
add_compile_options(-std=c++17 -Wall -Wextra -pedantic -Werror)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  base_local_planner
  costmap_2d
  geometry_msgs
  agribot_msgs
  nav_core
  nav_msgs
  pluginlib
  roscpp
  visualization_msgs
  move_base
  agribot_common
  tf
  tf2
  tf2_ros
  tf2_geometry_msgs
  dynamic_reconfigure
  costmap_converter
  robot_localization
)

## System dependencies are found with CMake's conventions
find_package(Boost REQUIRED COMPONENTS system)

# Geographiclib installs FindGeographicLib.cmake to this non-standard location
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "/usr/share/cmake/geographiclib/")
find_package(GeographicLib REQUIRED)

## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################

## To declare and build messages, services or actions from within this
## package, follow these steps:
## * Let MSG_DEP_SET be the set of packages whose message types you use in
##   your messages/services/actions (e.g. std_msgs, actionlib_msgs, ...).
## * In the file package.xml:
##   * add a build_depend tag for "message_generation"
##   * add a build_depend and a exec_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependency has been pulled in
##     but can be declared for certainty nonetheless:
##     * add a exec_depend tag for "message_runtime"
## * In this file (CMakeLists.txt):
##   * add "message_generation" and every package in MSG_DEP_SET to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * add "message_runtime" and every package in MSG_DEP_SET to
##     catkin_package(CATKIN_DEPENDS ...)
##   * uncomment the add_*_files sections below as needed
##     and list every .msg/.srv/.action file to be processed
##   * uncomment the generate_messages entry below
##   * add every package in MSG_DEP_SET to generate_messages(DEPENDENCIES ...)

## Generate messages in the 'msg' folder
# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
# generate_messages(
#   DEPENDENCIES
#   std_msgs  # Or other packages containing msgs
# )

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

## To declare and build dynamic reconfigure parameters within this
## package, follow these steps:
## * In the file package.xml:
##   * add a build_depend and a exec_depend tag for "dynamic_reconfigure"
## * In this file (CMakeLists.txt):
##   * add "dynamic_reconfigure" to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * uncomment the "generate_dynamic_reconfigure_options" section below
##     and list every .cfg file to be processed

## Generate dynamic reconfigure parameters in the 'cfg' folder
generate_dynamic_reconfigure_options(
  cfg/kinematics_control/Kinematics.cfg
)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES agribot_navigation
  CATKIN_DEPENDS
  base_local_planner
  costmap_2d
  geometry_msgs
  nav_core
  nav_msgs
  pluginlib
  roscpp
  visualization_msgs
  move_base
  tf
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(include
  ${Boost_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
  ${dynamic_reconfigure_PACKAGE_PATH}/cmake/cfgbuild.cmake
  ${GAZEBO_INCLUDE_DIRS}
  ${agribot_common_SOURCE_DIR}/include
)

include_directories(
  include
  ${Eigen_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)

link_directories(
  ${GAZEBO_LIBRARY_DIRS}
  ${PROJECT_SOURCE_DIR}
)

add_library(common_functions
  src/common/common_functions.cpp
)

add_library(visualization_functions
  src/common/visualization_functions.cpp
)

add_library(obstacle_detection
  src/obstacle_detection/obstacle.cpp
  src/obstacle_detection/detection.cpp
)

add_library(circles_planner
  src/planners/local_planners/circles_planner.cpp
)

add_library(hermite_planner
  src/planners/local_planners/hermite_planner.cpp
)
add_library(straight_path_planner
  src/planners/local_planners/straight_path_planner.cpp
)

add_library(agribot_local_planner
  src/planners/agribot_local_planner.cpp
  src/common/common_functions.cpp
  src/common/visualization_functions.cpp
  src/planners/local_planners/circles_planner.cpp
  src/planners/local_planners/hermite_planner.cpp
  src/planners/local_planners/narrow_planner.cpp
  src/planners/local_planners/straight_path_planner.cpp
  src/path_trackers/following_tracker.cpp
  src/path_trackers/ray_tracker.cpp
  src/path_controllers/pure_pursuit_controller.cpp
  src/path_controllers/stanley_controller.cpp
  src/path_controllers/deviation_parallelness_controller.cpp
  src/path_validators/distance_validator.cpp
  src/path_validators/curvature_validator.cpp
  src/obstacle_detection/obstacle.cpp
  src/obstacle_detection/detection.cpp
)

add_library(following_tracker
  src/path_trackers/following_tracker.cpp
)

add_library(ray_tracker
  src/path_trackers/ray_tracker.cpp
)

add_library(agribot_global_planner
  src/planners/agribot_global_planner.cpp
)

add_library(pure_pursuit_controller
  src/path_controllers/pure_pursuit_controller.cpp
)

add_library(stanley_controller
  src/path_controllers/stanley_controller.cpp
)

add_library(deviation_parallelness_controller
  src/path_controllers/deviation_parallelness_controller.cpp
)

add_library(distance_validator
  src/path_validators/distance_validator.cpp
)

add_library(curvature_validator
  src/path_validators/curvature_validator.cpp
)

add_library(pid
  src/kinematics_control/PID.cpp
)

add_executable(kinematics_control
  src/kinematics_control/PID.cpp
  src/kinematics_control/KinematicsControl.cpp
  src/kinematics_control/KinematicsControl_node.cpp
)

add_executable(cmd_vel_arbiter_node
  src/cmd_vel_arbiter/cmd_vel_arbiter_node.cpp
  src/cmd_vel_arbiter/CmdVelArbiterNode.cpp
  src/common/common_functions.cpp
)

add_executable(navigation_manager_node src/navigation_manager/navigation_manager_node.cpp src/navigation_manager/NavigationManager.cpp)
target_link_libraries(navigation_manager_node ${catkin_LIBRARIES} ${GeographicLib_LIBRARIES})
add_dependencies(navigation_manager_node ${catkin_EXPORTED_TARGETS})

add_executable(velocity_smoother_node src/velocity_smoother/velocity_smoother_node.cpp src/velocity_smoother/velocity_smoother.cpp)

target_link_libraries(
  velocity_smoother_node
  ${catkin_LIBRARIES}
)

target_link_libraries(
  pure_pursuit_controller
  stanley_controller
  deviation_parallelness_controller
  ${catkin_LIBRARIES}
)

target_link_libraries(kinematics_control
  ${GAZEBO_LIBRARIES}
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
)

target_link_libraries(common_functions
  ${catkin_LIBRARIES}
)

target_link_libraries(visualization_functions
  ${catkin_LIBRARIES}
)

target_link_libraries(obstacle_detection
  ${catkin_LIBRARIES}
)

target_link_libraries(circles_planner
  ${catkin_LIBRARIES}
)

target_link_libraries(hermite_planner
  ${catkin_LIBRARIES}
)

target_link_libraries(agribot_local_planner
  ${catkin_LIBRARIES}
)

target_link_libraries(agribot_global_planner
  ${catkin_LIBRARIES}
)

target_link_libraries(pid
  ${catkin_LIBRARIES}
)

target_link_libraries(cmd_vel_arbiter_node
  ${catkin_LIBRARIES}
)

## Declare a C++ library
# add_library(${PROJECT_NAME}
#   src/${PROJECT_NAME}/agribot_navigation.cpp
# )

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
# add_executable(${PROJECT_NAME}_node src/agribot_navigation_node.cpp)

## Rename C++ executable without prefix
## The above recommended prefix causes long target names, the following renames the
## target back to the shorter version for ease of user use
## e.g. "rosrun someones_pkg node" instead of "rosrun someones_pkg someones_pkg_node"
# set_target_properties(${PROJECT_NAME}_node PROPERTIES OUTPUT_NAME node PREFIX "")

## Add cmake target dependencies of the executable
## same as for the library above
# add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
# target_link_libraries(${PROJECT_NAME}_node
#   ${catkin_LIBRARIES}
# )

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html
install(TARGETS
  kinematics_control
  cmd_vel_arbiter_node
  navigation_manager_node
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

install(
  TARGETS agribot_local_planner
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

install(
  TARGETS agribot_global_planner
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

install(
  FILES bgp_plugin.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(
  FILES AP_plugin.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables and/or libraries for installation
# install(TARGETS ${PROJECT_NAME} ${PROJECT_NAME}_node
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  PATTERN ".svn" EXCLUDE
)

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
catkin_add_gtest(${PROJECT_NAME}-test test/utest.cpp)
if(TARGET ${PROJECT_NAME}-test)
  target_link_libraries(${PROJECT_NAME}-test pid)
endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)

add_dependencies(kinematics_control
  ${catkin_EXPORTED_TARGETS}
  ${PROJECT_NAME}_gencfg)
