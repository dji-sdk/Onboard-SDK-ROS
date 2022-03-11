#include <tuple>
#include <dji_sdk/dji_sdk_geometry.h>
#include <dji_sdk/dji_sdk_node.h>
#include <gtest/gtest.h>

using namespace DJISDKGeometry;

TEST(TestGPS2ENU_WGS84, compareToSphericalConversion)
{
  double ref_lon_GPS = 59.9;
  double ref_lat_GPS = 10.7;

  double lon_GPS = ref_lon_GPS + 0.0002;
  double lat_GPS = ref_lat_GPS + 0.0002;

  double x_ENU_spherical, y_ENU_spherical, x_ENU_WGS84, y_ENU_WGS84;
  gpsConvertENU(x_ENU_spherical, y_ENU_spherical, lon_GPS, lat_GPS, ref_lon_GPS, ref_lat_GPS);
  std::tie(x_ENU_WGS84, y_ENU_WGS84) = GPS2ENU_WGS84(lon_GPS, lat_GPS, ref_lon_GPS, ref_lat_GPS);

  std::cout << "Using spherical earth (x, y) = " << x_ENU_spherical << ", " << y_ENU_spherical << std::endl;
  std::cout << "Using WGS 84 ellipsoid (x, y) = " << x_ENU_WGS84 << ", " << y_ENU_WGS84 << std::endl;

  std::cout << "Difference in east (x) position in centimeters = " << (x_ENU_spherical - x_ENU_WGS84) * 100 << std::endl;
  std::cout << "Difference in north (y) in centimeters = " << (y_ENU_spherical - y_ENU_WGS84) * 100 << std::endl;
}

int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

