/** @file payload_device_node.cpp
 *  @version 4.0
 *  @date May 2020
 *
 *  @brief sample node of payload device.
 *
 *  @Copyright (c) 2020 DJI
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 *
 */

//INCLUDE
#include "dji_osdk_ros/payload_device_node.h"
#include <std_msgs/String.h>

bool sendToPayload(dji_osdk_ros::SendPayloadData sendPayloadData)
{
  send_to_payload_data_client.call(sendPayloadData);
  return sendPayloadData.response.result;
}


void fromPayloadDataSubCallback(const dji_osdk_ros::PayloadData::ConstPtr& fromPayloadData)
{
  ROS_INFO("frompayloadData:");
  std_msgs::String str;
  str.data.insert(str.data.begin(), fromPayloadData->data.begin(), fromPayloadData->data.end());
  ROS_INFO("%s\n", str.data.c_str());
}


//CODE
int main(int argc, char** argv)
{
  ros::init(argc, argv, "payload_device_node");
  ros::NodeHandle nh;
  ROS_INFO("payload device node start!");

  send_to_payload_data_client = nh.serviceClient<dji_osdk_ros::SendPayloadData>("send_data_to_payload_device_server");
  fromPayloadData = nh.subscribe("dji_osdk_ros/from_payload_data", 10, &fromPayloadDataSubCallback);

  ros::Rate rate(1);
  while(ros::ok())
  {
    uint8_t data[2] = {TEST_DATA_0 , TEST_DATA_1};
    SendPayloadData sendPayloadData;
    sendPayloadData.request.data.resize(sizeof(data));
    sendPayloadData.request.data[0] = data[0];
    sendPayloadData.request.data[1] = data[1];
    sendToPayload(sendPayloadData);

    ros::spinOnce();
    rate.sleep();
  }
}

