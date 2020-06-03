/** @file payload_device_node.h
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

#ifndef SRC_PAYLOAD_DEVICE_NODE_H
#define SRC_PAYLOAD_DEVICE_NODE_H

//include
#include <ros/ros.h>
#include <dji_osdk_ros/dji_vehicle_node.h>

using namespace dji_osdk_ros;

//const
#define TEST_DATA_0 0x32
#define TEST_DATA_1 0x33

//service
ros::ServiceClient send_to_payload_data_client;
ros::Subscriber fromPayloadData;

//function
bool sendToPayload(dji_osdk_ros::SendPayloadData sendPayloadData);
void fromPayloadDataSubCallback(const dji_osdk_ros::PayloadData::ConstPtr& fromPayloadData);


#endif //SRC_PAYLOAD_DEVICE_NODE_H
