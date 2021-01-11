cmake_minimum_required(VERSION 2.8.3)
project(dji_osdk_ros) ## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
    geometry_msgs
    message_generation
    nav_msgs
    nmea_msgs
    roscpp
    rospy
    std_msgs
    sensor_msgs
    message_filters # for message_filter
  )

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)

set(CMAKE_CXX_FLAGS "-std=c++11 -fPIC ${CMAKE_CXX_FLAGS}")
set(CMAKE_C_FLAGS "-fPIC ${CMAKE_C_FLAGS}")
set(CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake-modules")

set(ENABLE_ADVANCED_SENSING 1)
if(ENABLE_ADVANCED_SENSING)
	MESSAGE(STATUS "Support Advanced Sensing")
	ADD_DEFINITIONS(-DADVANCED_SENSING)
ENDIF()


find_package(LibUSB REQUIRED)
find_package(FFMPEG REQUIRED)

## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
#catkin_python_setup()

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
# )

## Generate msg in the 'msg' folder
add_message_files(
  FILES

  FCTimeInUTC.msg
  JoystickParams.msg
  GPSUTC.msg
  Gimbal.msg
  Waypoint.msg
  WaypointList.msg
  WaypointV2AircraftControlActuatorFlying.msg
  WaypointV2AircraftControlActuatorRotateHeading.msg
  WaypointV2AircraftControlActuator.msg
  WaypointV2AssociateTrigger.msg
  WaypointV2CameraActuatorFocalLengthParam.msg
  WaypointV2CameraActuatorFocusParam.msg
  WaypointV2CameraActuator.msg
  WaypointV2Config.msg
  WaypointV2GimbalActuatorRotationParam.msg
  WaypointV2GimbalActuator.msg
  WaypointV2InitSetting.msg
  WaypointV2IntervalTrigger.msg
  WaypointV2ReachpointTrigger.msg
  WaypointV2SampleReachPointTrigger.msg
  WaypointV2TrajectoryTrigger.msg
  WaypointV2Action.msg
  WaypointV2.msg
  WaypointV2MissionEventPush.msg
  WaypointV2MissionStatePush.msg

  MobileData.msg
  PayloadData.msg
  MissionWaypointAction.msg
  MissionWaypoint.msg
  MissionWaypointTask.msg
  MissionHotpointTask.msg
  FlightAnomaly.msg
  VOPosition.msg
  BatteryState.msg
  BatteryWholeInfo.msg
  SmartBatteryState.msg
  SmartBatteryDynamicInfo.msg
  HMSPushInfo.msg
)

## Generate services in the 'srv' folder
add_service_files(
  FILES

  GetDroneType.srv
  GetM300StereoParams.srv
  FlightTaskControl.srv
  SetJoystickMode.srv
  JoystickAction.srv
  ObtainControlAuthority.srv
  KillSwitch.srv
  EmergencyBrake.srv
  GimbalAction.srv
  CameraEV.srv
  CameraShutterSpeed.srv
  CameraAperture.srv
  CameraISO.srv
  CameraFocusPoint.srv
  CameraTapZoomPoint.srv
  CameraSetZoomPara.srv
  CameraZoomCtrl.srv
  CameraStartShootSinglePhoto.srv
  CameraStartShootBurstPhoto.srv
  CameraStartShootAEBPhoto.srv
  CameraStartShootIntervalPhoto.srv
  CameraStopShootPhoto.srv
  CameraRecordVideoAction.srv
  GetWholeBatteryInfo.srv
  GetSingleBatteryDynamicInfo.srv
  MFIO.srv
  SetGoHomeAltitude.srv
  GetGoHomeAltitude.srv
  SetCurrentAircraftLocAsHomePoint.srv
  SetHomePoint.srv
  SetupCameraH264.srv
  SetAvoidEnable.srv
  GetAvoidEnable.srv
  InitWaypointV2Setting.srv
  UploadWaypointV2Mission.srv
  UploadWaypointV2Action.srv
  DownloadWaypointV2Mission.srv
  StartWaypointV2Mission.srv
  StopWaypointV2Mission.srv
  PauseWaypointV2Mission.srv
  ResumeWaypointV2Mission.srv
  GenerateWaypointV2Action.srv
  SetGlobalCruisespeed.srv
  GetGlobalCruisespeed.srv
  GetHMSData.srv
  SubscribeWaypointV2Event.srv
  SubscribeWaypointV2State.srv

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
  SendPayloadData.srv
  SetHardSync.srv
  QueryDroneVersion.srv
  Stereo240pSubscription.srv
  StereoVGASubscription.srv
  StereoDepthSubscription.srv
  SetupCameraStream.srv
)

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
 generate_messages(
   DEPENDENCIES
   geometry_msgs
   nav_msgs
   std_msgs  # Or other packages containing msgs
   sensor_msgs
 )

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
# generate_dynamic_reconfigure_options(
#   cfg/DynReconf1.cfg
#   cfg/DynReconf2.cfg
# )

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
MESSAGE(STATUS "USB INCLUDE: ${LIBUSB_1_INCLUDE_DIR}")
catkin_package(
  INCLUDE_DIRS include
#  LIBRARIES onboard-sdk-ros
  CATKIN_DEPENDS
    message_runtime
    geometry_msgs
    nav_msgs
    std_msgs
    nmea_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
 include
 #${DJIOSDK_INCLUDE_DIRS}
 ${LIBUSB_1_INCLUDE_DIR}
 ${catkin_INCLUDE_DIRS}
)

add_subdirectory(src)

## Declare a C++ library
# add_library(${PROJECT_NAME}
#   src/${PROJECT_NAME}/onboard-sdk-ros.cpp
# )

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
# add_executable(${PROJECT_NAME}_node src/onboard-sdk-ros_node.cpp)
#add_executable(dji_vehicle_node src/dji_vehicle_node.cc)

## Rename C++ executable without prefix
## The above recommended prefix causes long target names, the following renames the
## target back to the shorter version for ease of user use
## e.g. "rosrun someones_pkg node" instead of "rosrun someones_pkg someones_pkg_node"
# set_target_properties(${PROJECT_NAME}_node PROPERTIES OUTPUT_NAME node PREFIX "")

## Add cmake target dependencies of the executable
## same as for the library above
# add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
#add_dependencies(dji_sdk_node
#                 dji_sdk_generate_messages_cpp)
## Specify libraries to link a library or executable target against
# target_link_libraries(${PROJECT_NAME}_node
#   ${catkin_LIBRARIES}
# )
# target_link_libraries(dji_vehicle_node
#   ${catkin_LIBRARIES}
#   ${DJIOSDK_LIBRARIES}
# )

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

## Mark executables for installation
## See http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_executables.html
# install(TARGETS ${PROJECT_NAME}_node
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark libraries for installation
## See http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_libraries.html
# install(TARGETS ${PROJECT_NAME}
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
# )

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

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_onboard-sdk-ros.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
