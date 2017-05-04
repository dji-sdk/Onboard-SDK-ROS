#include "gtest/gtest.h"
#include <dji_sdk/dji_sdk_node.h>
#include <std_msgs/UInt8.h>

using namespace DJI::onboardSDK;

DJI::onboardSDK::ROSAdapter *rosAdapter;
DJISDKNode *djiSDKNode;

class dji_sdk_node : public testing::Test {
 protected:
  virtual void SetUp();
  virtual void TearDown();

  ros::Subscriber activation_subscriber;
  ros::Publisher activation_publisher;

  uint8_t activation;

 public:
  void activation_subscriber_callback(std_msgs::UInt8 activation){
      this->activation = activation.data;
  }

};

