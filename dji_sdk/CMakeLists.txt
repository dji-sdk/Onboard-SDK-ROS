cmake_minimum_required(VERSION 2.8.3)
project(dji_sdk)

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
  sensor_msgs
)
find_package(DJIOSDK REQUIRED)

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
  Gimbal.msg
  Waypoint.msg
  WaypointList.msg
  MobileData.msg
  MissionWaypointAction.msg
  MissionWaypoint.msg
  MissionWaypointTask.msg
  MissionHotpointTask.msg
)

## Generate services in the 'srv' folder
add_service_files(
  FILES
  Activation.srv
  CameraAction.srv
  DroneTaskControl.srv
  SDKControlAuthority.srv
  SetLocalPosRef.srv
  MFIOConfig.srv
  MFIOSetValue.srv
  DroneArmControl.srv
  MissionStatus.srv
  MissionWpAction.srv
  MissionHpAction.srv
  MissionWpUpload.srv
  MissionWpSetSpeed.srv
  MissionWpGetSpeed.srv
  MissionWpGetInfo.srv
  MissionHpUpload.srv
  MissionHpGetInfo.srv
  MissionHpUpdateYawRate.srv
  MissionHpUpdateRadius.srv
  MissionHpResetYaw.srv
  SendMobileData.srv
  SetHardSync.srv
  QueryDroneVersion.srv
  Stereo240pSubscription.srv
  StereoVGASubscription.srv
  StereoDepthSubscription.srv
  SetupCameraStream.srv
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  geometry_msgs
  nav_msgs
  std_msgs
  sensor_msgs
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
  CATKIN_DEPENDS
        message_runtime
        geometry_msgs
        nav_msgs
        std_msgs
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  include
  ${DJIOSDK_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)

## Declare a cpp library
# add_library(dji_sdk
#   src/${PROJECT_NAME}/dji_sdk.cpp
# )

## Declare a cpp executable
add_executable(dji_sdk_node
  src/main.cpp
  src/modules/dji_sdk_node_control.cpp
  src/modules/dji_sdk_node_services.cpp
  src/modules/dji_sdk_node.cpp
  src/modules/dji_sdk_node_mission_services.cpp
  src/modules/dji_sdk_node_subscriber.cpp
  src/modules/dji_sdk_node_publisher.cpp
  src/modules/dji_sdk_node_mobile_comm.cpp
)

## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes
add_dependencies(dji_sdk_node
  dji_sdk_generate_messages_cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(dji_sdk_node
  ${catkin_LIBRARIES}
  ${DJIOSDK_LIBRARIES}
)

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


