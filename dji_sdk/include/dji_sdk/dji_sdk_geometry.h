#pragma once

#include <geometry_msgs/Quaternion.h>
#include <tf/tf.h>

namespace DJISDKGeometry
{
  static const tf::Matrix3x3 R_ENU2NED(0, 1, 0, 1, 0, 0, 0, 0, -1);
  static const tf::Matrix3x3 R_FLU2FRD(1, 0, 0, 0, -1, 0, 0, 0, -1);

  double wrapTo2Pi(double angle);
  double wrapToPi(double angle);
  
  geometry_msgs::Quaternion RTKYawQuaternion(double rtk_yaw_radians);
} // namespace DJISDKGeometry
