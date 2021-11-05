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

double DJISDKGeometry::transformRtkYaw(double raw_rtk_yaw_radians)
{
  tf::Matrix3x3 R_RTK2FRD;
  tf::Matrix3x3 R_NED2RTK;
  tf::Matrix3x3 R_FLU2ENU;

  R_RTK2FRD.setRPY(0.0, 0.0, DEG2RAD(90.0));
  R_NED2RTK.setRPY(0.0, 0.0, raw_rtk_yaw_radians);
  R_FLU2ENU = R_ENU2NED.transpose() * R_NED2RTK * R_RTK2FRD * R_FLU2FRD.transpose();

  double roll, pitch, yaw;
  R_FLU2ENU.getRPY(roll, pitch, yaw);
  return yaw;
}

void DJISDKGeometry::gpsConvertENU(double &ENU_x, double &ENU_y,
                                   double gps_t_lon, double gps_t_lat,
                                   double gps_r_lon, double gps_r_lat)
{
  double d_lon = gps_t_lon - gps_r_lon;
  double d_lat = gps_t_lat - gps_r_lat;
  ENU_y = DEG2RAD(d_lat) * C_EARTH;
  ENU_x = DEG2RAD(d_lon) * C_EARTH * std::cos(DEG2RAD(gps_t_lat));
};

static double radiusOfCurvaturePrimeVertical(double latitude)
{
  double squared_term = std::pow(DJISDKGeometry::earth_eccentricity * std::sin(latitude), 2);
  return DJISDKGeometry::earth_equatorial_radius / std::sqrt(1 - squared_term);
}

static double radiusOfCurvatureMeridian(double radius_of_curvature_prime_vertical, double latitude)
{
  double squared_term = std::pow(DJISDKGeometry::earth_eccentricity * std::sin(latitude), 2);
  return radius_of_curvature_prime_vertical * (1 - std::pow(DJISDKGeometry::earth_eccentricity, 2)) / std::sqrt(1 - squared_term);
}

void DJISDKGeometry::GPS2ENU_WGS84(
  double& x_ENU,
  double& y_ENU,
  double lon_GPS,
  double lat_GPS,
  double ref_lon_GPS,
  double ref_lat_GPS
)
{
  lon_GPS = DEG2RAD(lon_GPS);
  lat_GPS = DEG2RAD(lat_GPS);
  ref_lon_GPS = DEG2RAD(ref_lon_GPS);
  ref_lat_GPS = DEG2RAD(ref_lat_GPS);

  double r_n = radiusOfCurvaturePrimeVertical(ref_lat_GPS);
  double r_m = radiusOfCurvatureMeridian(r_n, ref_lat_GPS);

  double delta_lon = wrapToPi(lon_GPS - ref_lon_GPS);
  double delta_lat = wrapToPi(lat_GPS - ref_lat_GPS);

  x_ENU = delta_lon / std::atan2(1, r_n * std::cos(ref_lat_GPS));
  y_ENU = delta_lat / std::atan2(1, r_m);
}
