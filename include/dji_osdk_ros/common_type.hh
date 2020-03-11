#ifndef __COMMON_TYPE_HH__
#define __COMMON_TYPE_HH__
/**-----------------------------------------------
  * @Copyright (C) 2020 All rights reserved.
  * @date: 2020
  * @file: common_type.hh
  * @version: v0.0.1
  * @author: kevin.hoo@dji.com
  * @create_date: 2020-03-08 22:29:50
  * @last_modified_date: 2020-03-11 18:00:54
  * @brief: TODO
  * @details: TODO
  *-----------------------------------------------*/

// Header include
#include <djiosdk/dji_type.hpp>

// Declaration
namespace dji_osdk_ros
{
  struct MoveOffset
  {
    MoveOffset(double x_in, double y_in, double z_in,
               double yaw_in, double pos_threshold_in = 0.5, double yaw_threshold_in = 1.0)
      : x(x_in),
        y(y_in),
        z(z_in),
        yaw(yaw_in),
        pos_threshold(pos_threshold_in),
        yaw_threshold(yaw_threshold_in)
    {}
    double x;
    double y;
    double z;
    double yaw;
    double pos_threshold;
    double yaw_threshold;
  };

  struct RotationAngle
  {
    DJI::OSDK::float32_t roll;
    DJI::OSDK::float32_t pitch;
    DJI::OSDK::float32_t yaw;
  };
  
  struct GimbalContainer
  {
    int           roll             = 0;
    int           pitch            = 0;
    int           yaw              = 0;
    int           duration         = 0;
    int           isAbsolute       = 0;
    bool          yaw_cmd_ignore   = false;
    bool          pitch_cmd_ignore = false;
    bool          roll_cmd_ignore  = false;
    RotationAngle initialAngle;
    RotationAngle currentAngle;
    GimbalContainer(int roll = 0, int pitch = 0, int yaw = 0, int duration = 0,
                    int isAbsolute = 0, RotationAngle initialAngle = {},
                    RotationAngle currentAngle = {})
      : roll(roll)
      , pitch(pitch)
      , yaw(yaw)
      , duration(duration)
      , isAbsolute(isAbsolute)
      , initialAngle(initialAngle)
      , currentAngle(currentAngle)
    {
    }
  };

  // For zoom
  #pragma pack(1)
  typedef struct ZoomConfig
  {
    uint8_t digital_zoom_mode   : 2;  /* 0:step 1:position 2:continuous */
    uint8_t digital_reserve   : 1;  /* reserve */
    uint8_t digital_zoom_enable  : 1;  /* 0:not_ctrl 1:ctrl */
    uint8_t optical_zoom_mode    : 2;  /* 0:step 1:position 2:continuous */
    uint8_t optical_reserve    : 1;  /* reserve */
    uint8_t optical_zoom_enable  : 1;  /* 0:not_ctrl 1:ctrl */
  } zoom_config_t;

  typedef union zoom_param
  {
    struct
    {
      uint16_t zoom_cont_speed        : 8;  /* continuous speed 0~100 */
      uint16_t zoom_cont_direction    : 1;
      uint16_t zoom_cont_reserve      : 7;
    }cont_param;
    struct
    {
      uint16_t zoom_step_level    : 8;  /* level time * 100 = 1 times */
      uint16_t zoom_step_direction    : 1;
      uint16_t zoom_step_reserve      : 7;
    }step_param;
    struct
    {
      uint16_t zoom_pos_level;            /* 180 = 1.8times */
    }pos_param;
  } zoom_param;

  typedef struct camera_zoom_data_type
  {
    uint8_t func_index;
    uint8_t cam_index;
    zoom_config_t zoom_config;
    zoom_param optical_zoom_param;
    zoom_param digital_zoom_param;
  } CameraZoomDataType;
#pragma pack()
}

#endif // __COMMON_TYPE_HH__

