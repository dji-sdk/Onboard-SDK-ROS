#include "dji_sdk_test.h"

void dji_sdk_node::SetUp() {

  ASSERT_TRUE(ros::master::check()) << "ROS master is not up\n";

  rosAdapter = new DJI::onboardSDK::ROSAdapter;
  ros::NodeHandle nh;
  ros::NodeHandle nh_private("~");
  djiSDKNode = new DJISDKNode(nh, nh_private);

  activation_publisher = nh.advertise<std_msgs::UInt8>("dji_sdk/activation", 10);
  activation_subscriber = nh.subscribe<std_msgs::UInt8>("dji_sdk/activation", 10, &dji_sdk_node::activation_subscriber_callback, this);
}

void dji_sdk_node::TearDown() {
  
  activation_subscriber.shutdown();
  activation_publisher.shutdown();
 
  //TODO find a better way to clean 
/*  delete rosAdapter;
  rosAdapter = NULL;
  delete djiSDKNode;
  djiSDKNode = NULL;
*/
}

TEST_F(dji_sdk_node, test01) {

  ros::Duration(1.0).sleep();
  ros::spinOnce();

  /** 
   *@note Activation Error Codes
   *
   * Successful: 0
   * Parameter error: 1
   * Encode error: 2
   * New device: 3
   * App not connected:  4
   * No Internet: 5
   * Server refused: 6
   * Access level error: 7
   * Version error: 8
   */ 
  ASSERT_FALSE(activation);
}

int main(int argc, char **argv) {

  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "dji_sdk");
 
  ros::AsyncSpinner spinner(1);
  spinner.start();

  int ret =  RUN_ALL_TESTS();
  
  spinner.stop();
  ros::shutdown();

  return ret;
}
