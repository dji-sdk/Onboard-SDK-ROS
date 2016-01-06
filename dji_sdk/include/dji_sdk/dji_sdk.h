#ifndef SDK_LIBRARY_H
#define SDK_LIBRARY_H

//SDK library
#include "DJI_LIB/DJI_LIB_ROS_Adapter.h" 

//msgs
#include <dji_sdk/Acceleration.h>
#include <dji_sdk/AttitudeQuaternion.h>
#include <dji_sdk/Compass.h>
#include <dji_sdk/FlightControlInfo.h>
#include <dji_sdk/Gimbal.h>
#include <dji_sdk/GlobalPosition.h>
#include <dji_sdk/LocalPosition.h>
#include <dji_sdk/PowerStatus.h>
#include <dji_sdk/RCChannels.h>
#include <dji_sdk/Velocity.h>
#include <dji_sdk/Waypoint.h>
#include <dji_sdk/WaypointList.h>

//srvs
#include <dji_sdk/AttitudeControl.h>
#include <dji_sdk/CameraActionControl.h>
#include <dji_sdk/DroneTaskControl.h>
#include <dji_sdk/GimbalAngleControl.h>
#include <dji_sdk/GimbalSpeedControl.h>
#include <dji_sdk/GlobalPositionControl.h>
#include <dji_sdk/LocalPositionControl.h>
#include <dji_sdk/SDKPermissionControl.h>
#include <dji_sdk/VelocityControl.h>

//actions
#include <dji_sdk/DroneTaskAction.h>
#include <dji_sdk/GlobalPositionNavigationAction.h>
#include <dji_sdk/LocalPositionNavigationAction.h>
#include <dji_sdk/WaypointNavigationAction.h>


#endif
