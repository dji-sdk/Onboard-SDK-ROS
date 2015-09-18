#ifndef __DJI_SDK_NODE_H__
#define __DJI_SDK_NODE_H__
#include "DJI_LIB/DJI_Pro_App.h"
#include "DJI_LIB/DJI_Pro_Codec.h"
#include "DJI_LIB/DJI_Pro_Config.h"
#include "DJI_LIB/DJI_Pro_Hw.h"
#include "DJI_LIB/DJI_Pro_Link.h"
#include "DJI_LIB/DJI_Pro_Rmu.h"

#include <dji_ros/acc.h>
#include <dji_ros/attitude_quad.h>
#include <dji_ros/gimbal.h>
#include <dji_ros/global_position.h>
#include <dji_ros/local_position.h>
#include <dji_ros/rc_channels.h>
#include <dji_ros/velocity.h>

std::string serial_name;
int	baud_rate;
int	app_id;
int 	app_api_level;
int	app_version;
std::string app_bundle_id;
std::string enc_key;

activate_data_t user_act_data;

int DJI_Setup(std::string serial_port, int baudrate);
#endif
