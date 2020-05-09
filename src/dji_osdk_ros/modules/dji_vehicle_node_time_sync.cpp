//
// Created by dji on 2020/5/8.
//

#include <dji_osdk_ros/dji_vehicle_node.h>
using namespace dji_osdk_ros;

void VehicleNode::NMEACallback(Vehicle* vehiclePtr,
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
  VehicleNode *p = (VehicleNode *) userData;
  p->time_sync_nmea_publisher_.publish(nmeaSentence);
}

void VehicleNode::GPSUTCTimeCallback(Vehicle *vehiclePtr,
                                    RecvContainer recvFrame,
                                    UserData userData)
{
  dji_osdk_ros::GPSUTC GPSUTC;
  int length = recvFrame.recvInfo.len - OpenProtocol::PackageMin - 4;
  uint8_t rawBuf[length];
  memcpy(rawBuf, recvFrame.recvData.raw_ack_array, length);
  GPSUTC.stamp = ros::Time::now();
  GPSUTC.UTCTimeData = std::string((char*)rawBuf, length).c_str();
  VehicleNode *p = (VehicleNode *) userData;
  p->time_sync_gps_utc_publisher_.publish(GPSUTC);
}

void VehicleNode::FCTimeInUTCCallback(Vehicle* vehiclePtr,
                                     RecvContainer recvFrame,
                                     UserData userData)
{
  dji_osdk_ros::FCTimeInUTC fcTimeInUtc;
  fcTimeInUtc.stamp = ros::Time::now();
  fcTimeInUtc.fc_timestamp_us = recvFrame.recvData.fcTimeInUTC.fc_timestamp_us;
  fcTimeInUtc.fc_utc_hhmmss = recvFrame.recvData.fcTimeInUTC.utc_hhmmss;
  fcTimeInUtc.fc_utc_yymmdd = recvFrame.recvData.fcTimeInUTC.utc_yymmdd;
  VehicleNode *p = (VehicleNode *) userData;
  p->time_sync_fc_utc_publisher_.publish(fcTimeInUtc);
}

void VehicleNode::PPSSourceCallback(Vehicle* vehiclePtr,
                                   RecvContainer recvFrame,
                                   UserData userData)
{
  std_msgs::String PPSSourceData;
  std::vector<std::string> stringVec = {"0", "INTERNAL_GPS", "EXTERNAL_GPS", "RTK"};
  VehicleNode *p = (VehicleNode *) userData;
  if(recvFrame.recvData.ppsSourceType < stringVec.size())
  {
    PPSSourceData.data = stringVec[recvFrame.recvData.ppsSourceType];
    p->time_sync_pps_source_publisher_.publish(PPSSourceData);
  }
}

