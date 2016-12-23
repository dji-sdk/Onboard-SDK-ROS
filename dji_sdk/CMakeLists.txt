cmake_minimum_required(VERSION 2.8.3)
project(dji_sdk)

option(GTEST "Build with Google Test framework" OFF)

# Add module path for <name>.cmake conf files
set(CMAKE_MODULE_PATH ${CMAKE_SOURCE_DIR}/Onboard-SDK-ROS/contrib)
if (GTEST) 
include(${CMAKE_MODULE_PATH}/External_GTest.cmake)
endif () 

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  message_generation
  nav_msgs
  roscpp
  rospy
  std_msgs
  actionlib_msgs 
  actionlib
  dji_sdk_lib
)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)

set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")
## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################

## To declare and build messages, services or actions from within this
## package, follow these steps:
## * Let MSG_DEP_SET be the set of packages whose message types you use in
##   your messages/services/actions (e.g. std_msgs, actionlib_msgs, ...).
## * In the file package.xml:
##   * add a build_depend and a run_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependencies might have been
##     pulled in transitively but can be declared for certainty nonetheless:
##     * add a build_depend tag for "message_generation"
##     * add a run_depend tag for "message_runtime"
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
add_message_files(
  FILES
  A3GPS.msg
  A3RTK.msg
  Acceleration.msg
  AttitudeQuaternion.msg
  Compass.msg
  FlightControlInfo.msg
  Gimbal.msg
  GlobalPosition.msg
  LocalPosition.msg
  PowerStatus.msg
  RCChannels.msg
  Velocity.msg
  Waypoint.msg
  WaypointList.msg
  TransparentTransmissionData.msg
  TimeStamp.msg

  MissionPushInfo.msg
  MissionWaypointAction.msg
  MissionWaypoint.msg
  MissionWaypointTask.msg
  MissionHotpointTask.msg
  MissionFollowmeTask.msg
  MissionFollowmeTarget.msg
  MissionStatusWaypoint.msg
  MissionStatusHotpoint.msg
  MissionStatusFollowme.msg
  MissionStatusOther.msg
  MissionEventWpUpload.msg
  MissionEventWpAction.msg
  MissionEventWpReach.msg
)

## Generate services in the 'srv' folder
add_service_files(
  FILES
  Activation.srv
  AttitudeControl.srv
  CameraActionControl.srv
  DroneTaskControl.srv
  GimbalAngleControl.srv
  GimbalSpeedControl.srv
  GlobalPositionControl.srv
  LocalPositionControl.srv
  SDKPermissionControl.srv
  SendDataToRemoteDevice.srv
  VelocityControl.srv
  VersionCheck.srv

  DroneArmControl.srv
  SyncFlagControl.srv
  MessageFrequencyControl.srv
  VirtualRCEnableControl.srv
  VirtualRCDataControl.srv

  MissionStart.srv
  MissionPause.srv
  MissionCancel.srv
  MissionWpUpload.srv
  MissionWpSetSpeed.srv
  MissionWpGetSpeed.srv
  MissionWpDownload.srv
  MissionHpUpload.srv
  MissionHpDownload.srv
  MissionHpSetSpeed.srv
  MissionHpSetRadius.srv
  MissionHpResetYaw.srv
  MissionFmUpload.srv
  MissionFmSetTarget.srv
)

## Generate actions in the 'action' folder
add_action_files(
  FILES
  GlobalPositionNavigation.action
  LocalPositionNavigation.action
  WaypointNavigation.action
  DroneTask.action
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  geometry_msgs 
  nav_msgs
  std_msgs
  actionlib_msgs
)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  INCLUDE_DIRS include
#  LIBRARIES dji_sdk
  CATKIN_DEPENDS message_runtime actionlib actionlib_msgs geometry_msgs nav_msgs std_msgs dji_sdk_lib
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

## Declare a cpp library
# add_library(dji_sdk
#   src/${PROJECT_NAME}/dji_sdk.cpp
# )

## Declare a cpp executable
add_executable(dji_sdk_node
  src/dji_sdk_node.cpp 
  src/modules/dji_sdk_node_actions.cpp 
  src/modules/dji_sdk_node_services.cpp 
  src/modules/dji_sdk_node_main.cpp 
  src/modules/dji_sdk_node_mission.cpp
)

## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes
add_dependencies(dji_sdk_node
  dji_sdk_generate_messages_cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(dji_sdk_node
  ${catkin_LIBRARIES}
)

#############
## Testing ##
#############
find_package(rostest REQUIRED)
find_package(GTest QUIET)

if(GTEST_FOUND)
  message("Found GTest on the system\n")
endif()

# Create a gtest executable with target name "dji_sdk_test"
# which is not built by "make all" but only by "make tests"
# add one/multiple source files to the executable target
# register the rostest launch file "test/dji_sdk-node.test"
add_rostest_gtest(${PROJECT_NAME}_test
  test/${PROJECT_NAME}-node.test
  src/test/${PROJECT_NAME}_test.cpp
  src/modules/dji_sdk_node_actions.cpp
  src/modules/dji_sdk_node_services.cpp
  src/modules/dji_sdk_node_main.cpp
  src/modules/dji_sdk_node_mission.cpp)

if(TARGET ${PROJECT_NAME}_test)
  add_dependencies(${PROJECT_NAME}_test
    dji_sdk_generate_messages_cpp
    dji_sdk_lib)

  target_link_libraries(${PROJECT_NAME}_test
     ${catkin_LIBRARIES}
     dji_sdk_lib
     gtest
     gtest_main
     pthread)
endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)

option(USE_COLLISION_AVOIDANCE "Use DJI collision avoidance library" OFF)
if(USE_COLLISION_AVOIDANCE)
  include(${CMAKE_MODULE_PATH}/External_CollisionAvoidance.cmake)
endif()

option(USE_POINTCLOUD2LAS "Use DJI Pointcloud to LAS converter library" OFF)
if(USE_POINTCLOUD2LAS)
  include(${CMAKE_MODULE_PATH}/External_Pointcloud2Las.cmake)
endif()

#############
## Install ##
#############
# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables and/or libraries for installation
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(TARGETS dji_sdk_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )


