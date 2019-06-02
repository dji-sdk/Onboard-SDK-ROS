/** @file demo_payload_comm.cpp
 *  @version 3.8.1
 *  @date May, 2019
 *
 *  @brief
 *  demo sample of how to use payload communication APIs
 *
 *  @copyright 2019 DJI. All rights reserved.
 *
 */

#include <dji_sdk_demo/demo_payload_comm.h>
#include <std_msgs/String.h>
#include <dji_sdk/SendPayloadData.h>
#include "dji_sdk/dji_sdk.h"

#define TEST_NUM_0 0x11
#define TEST_NUM_1 0x22

using namespace DJI::OSDK;

ros::ServiceClient payload_data_service;
ros::Subscriber from_payload_data_subscriber;

int main(int argc, char *argv[]) {
  uint32_t loop_flag = 0;
  ros::init(argc, argv, "sdk_demo_payload_comm");
  ROS_INFO("sdk_demo_payload_comm_test");
  ros::NodeHandle nh;

  payload_data_service = nh.serviceClient<dji_sdk::SendPayloadData>
          ("dji_sdk/send_data_to_payload");
  from_payload_data_subscriber = nh.subscribe<dji_sdk::PayloadData>
          ("dji_sdk/from_payload_data", 10, &fromPayloadDataSubscriberCallback);
  while (ros::ok()){
    if (loop_flag >= 50)  // send about once a second
    {
      static uint8_t testBuf[2] = {TEST_NUM_0, TEST_NUM_1};
      dji_sdk::SendPayloadData payload_data;
      payload_data.request.data.resize(sizeof(testBuf));
      payload_data.request.data[0] = testBuf[0];
      payload_data.request.data[1] = testBuf[1];
      sendToPayload(payload_data);
      loop_flag = 0;
    }
    ros::spinOnce();
    ros::Duration(0.02).sleep();
    loop_flag++;
  }
  return 0;
}

bool sendToPayload(dji_sdk::SendPayloadData &payload_data)
{
  return payload_data_service.call(payload_data);
}

void fromPayloadDataSubscriberCallback(const dji_sdk::PayloadData::ConstPtr& from_payload_data) {
  std_msgs::String str;
  str.data.insert(str.data.begin(), from_payload_data->data.begin(), from_payload_data->data.end());
  ROS_INFO("%s\n", str.data.c_str());
}
