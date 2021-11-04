#include <dji_sdk/dji_sdk_geometry.h>
#include <dji_sdk/dji_sdk_node.h>
#include <gtest/gtest.h>

using namespace DJISDKGeometry;

TEST(TestRTKTransform, testRTKTransform)
{
  constexpr double tolerance = 1e-12;
  double roll, pitch, yaw;

  auto simple_but_non_readable_rtk_transform = [](double rtk_yaw) { return -rtk_yaw; };
  std::vector<double> rtk_yaw_input_vec = {
    0.0, 45.0, 90.0, 135.0, 180.0, 270.0, 360.0, -45.0, -90.0, -135.0, -180.0, -270.0, -360.0
  };

  for (auto rtk_yaw : rtk_yaw_input_vec)
  {
    rtk_yaw = DEG2RAD(rtk_yaw);
    double desired_yaw = simple_but_non_readable_rtk_transform(rtk_yaw);

    geometry_msgs::Quaternion quat = RTKYawQuaternion(rtk_yaw);
    tf::Matrix3x3 R_FLU2ENU(tf::Quaternion(quat.x, quat.y, quat.z, quat.w));
    R_FLU2ENU.getRPY(roll, pitch, yaw);

    EXPECT_TRUE(std::abs(wrapToPi(yaw - desired_yaw)) < tolerance);
    EXPECT_TRUE(std::abs(wrapToPi(pitch - 0)) < tolerance);
    EXPECT_TRUE(std::abs(wrapToPi(roll - 0)) < tolerance);
  }
}

int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}