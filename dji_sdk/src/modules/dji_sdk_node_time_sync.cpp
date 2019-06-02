/** @file dji_sdk_node_time_sync.cpp
 *  @version 3.8.1
 *  @date May, 2019
 *
 *  @brief
 *  Implementation of the time sync functions of DJISDKNode
 *
 *  @copyright 2019 DJI. All rights reserved.
 *
 */

#include <dji_sdk/dji_sdk_node.h>

void DJISDKNode::NMEACallback(Vehicle* vehiclePtr,
                              RecvContainer recvFrame,
                              UserData userData)
{
  nmea_msgs::Sentence nmeaSentence;
  int length = recvFrame.recvInfo.len - OpenProtocol::PackageMin - 4;
  uint8_t rawBuf[length];
  memcpy(rawBuf, recvFrame.recvData.raw_ack_array, length);
  nmeaSentence.header.frame_id = "NMEA";
  nmeaSentence.header.stamp = ros::Time::now();
  nmeaSentence.sentence = std::string((char*)rawBuf, length);
  DJISDKNode *p = (DJISDKNode *) userData;
  p->time_sync_nmea_publisher.publish(nmeaSentence);
}

void DJISDKNode::GPSUTCTimeCallback(Vehicle *vehiclePtr,
                                    RecvContainer recvFrame,
                                    UserData userData)
{
  dji_sdk::GPSUTC GPSUTC;
  int length = recvFrame.recvInfo.len - OpenProtocol::PackageMin - 4;
  uint8_t rawBuf[length];
  memcpy(rawBuf, recvFrame.recvData.raw_ack_array, length);
  GPSUTC.stamp = ros::Time::now();
  GPSUTC.UTCTimeData = std::string((char*)rawBuf, length).c_str();
  DJISDKNode *p = (DJISDKNode *) userData;
  p->time_sync_gps_utc_publisher.publish(GPSUTC);
}

void DJISDKNode::FCTimeInUTCCallback(Vehicle* vehiclePtr,
                                     RecvContainer recvFrame,
                                     UserData userData)
{
  dji_sdk::FCTimeInUTC fcTimeInUtc;
  fcTimeInUtc.stamp = ros::Time::now();
  fcTimeInUtc.fc_timestamp_us = recvFrame.recvData.fcTimeInUTC.fc_timestamp_us;
  fcTimeInUtc.fc_utc_hhmmss = recvFrame.recvData.fcTimeInUTC.utc_hhmmss;
  fcTimeInUtc.fc_utc_yymmdd = recvFrame.recvData.fcTimeInUTC.utc_yymmdd;
  DJISDKNode *p = (DJISDKNode *) userData;
  p->time_sync_fc_utc_publisher.publish(fcTimeInUtc);
}

void DJISDKNode::PPSSourceCallback(Vehicle* vehiclePtr,
                                   RecvContainer recvFrame,
                                   UserData userData)
{
  std_msgs::String PPSSourceData;
  std::vector<std::string> stringVec = {"0", "INTERNAL_GPS", "EXTERNAL_GPS", "RTK"};
  DJISDKNode *p = (DJISDKNode *) userData;
  if(recvFrame.recvData.ppsSourceType < stringVec.size())
  {
    PPSSourceData.data = stringVec[recvFrame.recvData.ppsSourceType];
    p->time_sync_pps_source_publisher.publish(PPSSourceData);
  }
}
