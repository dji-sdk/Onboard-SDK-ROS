#ifndef SDK_LIBRARY_H
#define SDK_LIBRARY_H

//SDK library
#include "DJI_LIB/DJI_Pro_App.h" 
#include "DJI_LIB/DJI_Pro_Codec.h" 
#include "DJI_LIB/DJI_Pro_Config.h" 
#include "DJI_LIB/DJI_Pro_Hw.h" 
#include "DJI_LIB/DJI_Pro_Link.h" 
#include "DJI_LIB/DJI_Pro_Rmu.h"

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

#include <dji_sdk/TimeStamp.h>

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

#include <dji_sdk/DroneArmControl.h>
#include <dji_sdk/SyncFlagControl.h>
#include <dji_sdk/MessageFrequencyControl.h>
#include <dji_sdk/VirtualRCEnableControl.h>
#include <dji_sdk/VirtualRCDataControl.h>

//actions
#include <dji_sdk/DroneTaskAction.h>
#include <dji_sdk/GlobalPositionNavigationAction.h>
#include <dji_sdk/LocalPositionNavigationAction.h>
#include <dji_sdk/WaypointNavigationAction.h>

//missions
#include <dji_sdk/MissionFollowmeTask.h>
#include <dji_sdk/MissionFollowmeTarget.h>
#include <dji_sdk/MissionHotpointTask.h>
#include <dji_sdk/MissionPushInfo.h>
#include <dji_sdk/MissionWaypointAction.h>
#include <dji_sdk/MissionWaypoint.h>
#include <dji_sdk/MissionWaypointTask.h>
#include <dji_sdk/MissionCancel.h>
#include <dji_sdk/MissionDownload.h>
#include <dji_sdk/MissionFmSetTarget.h>
#include <dji_sdk/MissionFmUpload.h>
#include <dji_sdk/MissionHpResetYaw.h>
#include <dji_sdk/MissionHpSetRadiu.h>
#include <dji_sdk/MissionHpSetSpeed.h>
#include <dji_sdk/MissionHpUpload.h>
#include <dji_sdk/MissionPause.h>
#include <dji_sdk/MissionStart.h>
#include <dji_sdk/MissionWpGetSpeed.h>
#include <dji_sdk/MissionWpSetSpeed.h>
#include <dji_sdk/MissionWpUpload.h>


#endif
