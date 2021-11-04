#include <dji_sdk/dji_sdk_node.h>
#include <tf/tf.h>
#include <dji_sdk/dji_sdk_geometry.h>

double DJISDKGeometry::wrapTo2Pi(double angle)
{
  bool was_neg = angle < 0;
  angle = fmod(angle, 2.0 * M_PI);
  if (was_neg) angle += 2.0 * M_PI;
  return angle;
}

double DJISDKGeometry::wrapToPi(double angle)
{
  return wrapTo2Pi(angle + M_PI) - M_PI;
}

geometry_msgs::Quaternion DJISDKGeometry::RTKYawQuaternion(double rtk_yaw_radians)
{
  tf::Matrix3x3 R_RTK2FRD;
  tf::Matrix3x3 R_NED2RTK;
  tf::Matrix3x3 R_FLU2ENU;

  R_RTK2FRD.setRPY(0.0, 0.0, DEG2RAD(90.0));
  R_NED2RTK.setRPY(0.0, 0.0, rtk_yaw_radians);
  R_FLU2ENU = R_ENU2NED.transpose() * R_NED2RTK * R_RTK2FRD * R_FLU2FRD.transpose();

  tf::Quaternion q_FLU2ENU;
  R_FLU2ENU.getRotation(q_FLU2ENU);

  geometry_msgs::Quaternion rtk_yaw_quaternion;
  rtk_yaw_quaternion.w = q_FLU2ENU.getW();
  rtk_yaw_quaternion.x = q_FLU2ENU.getX();
  rtk_yaw_quaternion.y = q_FLU2ENU.getY();
  rtk_yaw_quaternion.z = q_FLU2ENU.getZ();

  return rtk_yaw_quaternion;
}

// TODO: Implement GPS to ENU (x, y) function
