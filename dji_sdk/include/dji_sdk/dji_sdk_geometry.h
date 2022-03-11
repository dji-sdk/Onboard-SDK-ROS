#pragma once

#include <utility>

namespace DJISDKGeometry
{
  static constexpr double earth_equatorial_radius{6378137.0};
  static constexpr double earth_eccentricity{0.0818};

  double wrapToPi(double angle);
  double RTKYawMeasurement2ENUYaw(double raw_rtk_yaw_radians);

  void gpsConvertENU(double &ENU_x, double &ENU_y,
                     double gps_t_lon, double gps_t_lat,
                     double gps_r_lon, double gps_r_lat);

  std::pair<double, double> GPS2ENU_WGS84(
    double lon_GPS,
    double lat_GPS,
    double ref_lon_GPS,
    double ref_lat_GPS
  );

} // namespace DJISDKGeometry
