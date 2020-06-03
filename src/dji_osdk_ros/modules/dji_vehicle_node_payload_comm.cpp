//
// Created by dji on 2020/5/8.
//
#include <dji_osdk_ros/dji_vehicle_node.h>
using namespace dji_osdk_ros;

void VehicleNode::SDKfromPayloadDataCallback(Vehicle *vehicle, RecvContainer recvFrame, DJI::OSDK::UserData userData) {
  ((VehicleNode*)userData)->fromPayloadDataCallback(recvFrame);
}

void VehicleNode::fromPayloadDataCallback(RecvContainer recvFrame) {
  int dataLength = recvFrame.recvInfo.len - OpenProtocol::PackageMin - 2;
  ROS_INFO( "Received payload Data of len %d\n", recvFrame.recvInfo.len);
  dji_osdk_ros::PayloadData payload_data;
  payload_data.data.assign(recvFrame.recvData.raw_ack_array,recvFrame.recvData.raw_ack_array + dataLength);
  from_payload_data_publisher_.publish(payload_data);
}

bool VehicleNode::sendToPayloadCallback(dji_osdk_ros::SendPayloadData::Request& request,dji_osdk_ros::SendPayloadData::Response& response)
{
  ROS_DEBUG("called sendToPayloadCallback");
  response.result = false;
  if(ptr_wrapper_ == nullptr)
  {
    ROS_ERROR_STREAM("Vehicle modules is nullptr");
    return true;
  }
  ptr_wrapper_->sendDataToPSDK(&request.data[0], request.data.size());
  response.result = true;
  return true;
}

