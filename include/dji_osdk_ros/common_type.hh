#ifndef __COMMON_TYPE_HH__
#define __COMMON_TYPE_HH__
/**-----------------------------------------------
  * @Copyright (C) 2020 All rights reserved.
  * @date: 2020
  * @file: common_type.hh
  * @version: v0.0.1
  * @author: kevin.hoo@dji.com
  * @create_date: 2020-03-08 22:29:50
  * @last_modified_date: 2020-03-09 10:40:51
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
               double yaw_in, double pos_threshold_in, double yaw_threshold_in)
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
}

#endif // __COMMON_TYPE_HH__

