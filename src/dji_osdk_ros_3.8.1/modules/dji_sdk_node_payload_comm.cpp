/** @file dji_sdk_node_payload_comm.cpp
 *  @version 3.8.1
 *  @date May, 2019
 *
 *  @brief
 *  Implementation of the payload communication functions of DJISDKNode
 *
 *  @copyright 2019 DJI. All rights reserved.
 *
 */

#include <dji_sdk/dji_sdk_node.h>

void DJISDKNode::SDKfromPayloadDataCallback(Vehicle *vehicle, RecvContainer recvFrame, DJI::OSDK::UserData userData) {
  ((DJISDKNode*)userData)->fromPayloadDataCallback(recvFrame);
}

void DJISDKNode::fromPayloadDataCallback(RecvContainer recvFrame) {
  int dataLength = recvFrame.recvInfo.len - OpenProtocol::PackageMin - 2;
    ROS_INFO( "Received payload Data of len %d\n", recvFrame.recvInfo.len);
    dji_osdk_ros::PayloadData payload_data;
    payload_data.data.assign(recvFrame.recvData.raw_ack_array,recvFrame.recvData.raw_ack_array + dataLength);
    from_payload_data_publisher.publish(payload_data);
}

bool DJISDKNode::sendToPayloadCallback(dji_osdk_ros::SendPayloadData::Request& request,
                                      dji_osdk_ros::SendPayloadData::Response& response){
  ROS_INFO("Send data to payload, size : %ldbytes", request.data.size());
  vehicle->payloadDevice->sendDataToPSDK(&request.data[0], request.data.size());
  response.result = true;
  return true;
}
