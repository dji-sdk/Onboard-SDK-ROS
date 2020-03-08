#ifndef __COMMON_TYPE_HH__
#define __COMMON_TYPE_HH__
/**-----------------------------------------------
  * @Copyright (C) 2020 All rights reserved.
  * @date: 2020
  * @file: common_type.hh
  * @version: v0.0.1
  * @author: kevin.hoo@dji.com
  * @create_date: 2020-03-08 22:29:50
  * @last_modified_date: 2020-03-08 22:35:01
  * @brief: TODO
  * @details: TODO
  *-----------------------------------------------*/

// Header include

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
}

#endif // __COMMON_TYPE_HH__

