/**
  * @Copyright (C) 2020 All rights reserved.
  * @date: 2020
  * @file: gimbal_camera_control_node.cc
  * @version: v0.0.1
  * @author: kevin.hoo@dji.com
  * @create_date: 2020-03-04 23:56:34
  * @last_modified_date: 2020-03-11 22:32:52
  * @brief: TODO
  * @details: TODO
  */

//INCLUDE
#include <ros/ros.h>
#include <dji_osdk_ros/common_type.h>
#include <dji_osdk_ros/GimbalAction.h>
#include <dji_osdk_ros/CameraTaskControl.h>

//CODE
using namespace dji_osdk_ros;

void packGimbalAction(const GimbalContainer& gimbal_container, GimbalAction& action);
void packZoomParams(const CameraZoomDataType& zoom_data, CameraTaskControl& camera_action);

int main(int argc, char** argv)
{
  ros::init(argc, argv, "gimbal_camera_control_node");
  ros::NodeHandle nh;

  auto gimbal_control_client = nh.serviceClient<GimbalAction>("gimbal_task_control");
  auto camera_control_client = nh.serviceClient<CameraTaskControl>("camera_task_control");

  // Display interactive prompt
  std::cout
    << "| Available commands:                                            |"
    << std::endl;
  std::cout
    << "| [a] Exercise gimbal and camera control                         |\n"
    << "| [b] Exercise camera zoom control                               |"
    << std::endl;
  char inputChar;
  std::cin >> inputChar;

  switch (inputChar)
  {
    case 'a':
    {
      {
        // Turn around Gimbal
        GimbalContainer gimbal_value(0, 200, 1800, 20, 0);
        GimbalAction gimbal_action;
        packGimbalAction(gimbal_value, gimbal_action);

        gimbal_control_client.call(gimbal_action);
      }

      {
        // Take a Picture
        CameraTaskControl camera_action;
        camera_action.request.action = CameraTaskControl::Request::CAMERA_ACTION_TAKE_PICTURE;
        camera_control_client.call(camera_action);
      }

      {
        CameraTaskControl camera_action;
        // Record a video
        camera_action.request.action = CameraTaskControl::Request::CAMERA_ACTION_START_RECORD;
        camera_control_client.call(camera_action);

        ros::Duration(2.0).sleep();
        // Stop Recording
        camera_action.request.action = CameraTaskControl::Request::CAMERA_ACTION_STOP_RECORD;
        camera_control_client.call(camera_action);
      }

      break;
    }

    case 'b':
    {
      CameraTaskControl camera_action;
      camera_action.request.action = CameraTaskControl::Request::CAMERA_ACTION_ZOOM;
      CameraZoomDataType zoom_data;

      zoom_data.func_index = 19;
      zoom_data.cam_index = 1;
      zoom_data.zoom_config.optical_zoom_mode = 0;
      zoom_data.zoom_config.optical_zoom_enable = 1;
      zoom_data.optical_zoom_param.step_param.zoom_step_level = 200;
      zoom_data.optical_zoom_param.step_param.zoom_step_direction = 1;

      packZoomParams(zoom_data, camera_action);

      camera_control_client.call(camera_action);
      break;
    }
    default:
      break;
  }

  return 0;
}


void packGimbalAction(const GimbalContainer& gimbal_value, GimbalAction& gimbal_action)
{
  gimbal_action.request.roll     = gimbal_value.roll;
  gimbal_action.request.pitch    = gimbal_value.pitch;
  gimbal_action.request.yaw      = gimbal_value.yaw;
  gimbal_action.request.duration = gimbal_value.duration;
  gimbal_action.request.mode |= 0;
  gimbal_action.request.mode |= gimbal_value.isAbsolute;
  gimbal_action.request.mode |= gimbal_value.yaw_cmd_ignore << 1;
  gimbal_action.request.mode |= gimbal_value.roll_cmd_ignore << 2;
  gimbal_action.request.mode |= gimbal_value.pitch_cmd_ignore << 3;
}

void packZoomParams(const CameraZoomDataType& zoom_data, CameraTaskControl& camera_action)
{
  memcpy(&camera_action.request.func_index, &zoom_data.func_index, sizeof(zoom_data.func_index));
  memcpy(&camera_action.request.cam_index, &zoom_data.cam_index, sizeof(zoom_data.cam_index));
  memcpy(&camera_action.request.zoom_config, &zoom_data.zoom_config, sizeof(zoom_data.zoom_config));
  memcpy(&camera_action.request.step_param, &zoom_data.optical_zoom_param.step_param, sizeof(zoom_data.optical_zoom_param.step_param));
  memcpy(&camera_action.request.cont_param, &zoom_data.optical_zoom_param.cont_param, sizeof(zoom_data.optical_zoom_param.cont_param));
  memcpy(&camera_action.request.pos_param, &zoom_data.optical_zoom_param.pos_param, sizeof(zoom_data.optical_zoom_param.pos_param));
}
