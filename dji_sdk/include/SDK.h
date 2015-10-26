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
#include <dji_sdk/accel.h>
#include <dji_sdk/attitude_quad.h>
#include <dji_sdk/gimbal.h>
#include <dji_sdk/global_position.h>
#include <dji_sdk/local_position.h>
#include <dji_sdk/rc_channels.h>
#include <dji_sdk/velocity.h>
#include <dji_sdk/compass.h>
#include <dji_sdk/ctrl_info.h>
#include <dji_sdk/waypoint.h>
#include <dji_sdk/waypointList.h>
#include <dji_sdk/map_nav_srv_cmd.h>

//srvs
#include <dji_sdk/attitude.h>
#include <dji_sdk/action.h>
#include <dji_sdk/camera_action.h>
#include <dji_sdk/control_manager.h>
#include <dji_sdk/gimbal_speed.h>
#include <dji_sdk/gimbal_angle.h>

//action
#include <dji_sdk/taskAction.h>
#include <dji_sdk/local_navigationAction.h>
#include <dji_sdk/gps_navigationAction.h>
#include <dji_sdk/waypoint_navigationAction.h>
#include <dji_sdk/web_waypoint_receiveAction.h>
#endif
