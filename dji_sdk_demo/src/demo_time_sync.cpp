/** @file demo_time_sync.cpp
 *  @version 3.8.1
 *  @date May, 2019
 *
 *  @brief
 *  demo sample of how to use time sync APIs
 *
 *  @copyright 2019 DJI. All rights reserved.
 *
 */

#include <dji_sdk_demo/demo_time_sync.h>
#include <dji_sdk/dji_sdk.h>
#include <nmea_msgs/Sentence.h>

int
main(int argc, char** argv)
{
  ros::init(argc, argv, "sdk_demo_time_sync");
  ros::NodeHandle nh;

  // ROS stuff
  ros::Subscriber time_sync_nmea_subscriber =
          nh.subscribe<nmea_msgs::Sentence>("dji_sdk/time_sync_nmea_msg", 10, &NMEADataCallback);
  ros::Subscriber time_sync_gps_utc_subscriber =
          nh.subscribe<dji_sdk::GPSUTC>("dji_sdk/time_sync_gps_utc", 10, &GPSUTCTimeDataCallback);
  ros::Subscriber time_sync_fc_utc_subscriber =
          nh.subscribe<dji_sdk::FCTimeInUTC>("dji_sdk/time_sync_fc_time_utc", 10, &FCUTCTimeDataCallback);
  ros::Subscriber time_sync_pps_source_subscriber =
          nh.subscribe("dji_sdk/time_sync_pps_source", 10, &PPSSourceDataCallback);

  ros::spin();

  return 0;
}

void
NMEADataCallback(const nmea_msgs::Sentence::ConstPtr& msg)
{
  ROS_INFO("[%s] Recv Time\t: sec:%d nsec:%d", __FUNCTION__, msg->header.stamp.sec, msg->header.stamp.nsec);
  ROS_INFO("[%s] NMEA Data\t: %s", __FUNCTION__, msg->sentence.c_str());
}

void
GPSUTCTimeDataCallback(const dji_sdk::GPSUTC::ConstPtr& msg)
{
  ROS_INFO("[%s] Recv Time\t: sec:%d nsec:%d", __FUNCTION__, msg->stamp.sec, msg->stamp.nsec);
  ROS_INFO("[%s] GPSUTC Data\t: %s", __FUNCTION__, msg->UTCTimeData.c_str());
}

void
FCUTCTimeDataCallback(const dji_sdk::FCTimeInUTC::ConstPtr& msg)
{
  ROS_INFO("[%s] Recv Time\t: sec:%d nsec:%d", __FUNCTION__, msg->stamp.sec, msg->stamp.nsec);
  ROS_INFO("[%s] FCUTC Data\t: %u, UTC time: %u, UTC date: %u", __FUNCTION__, msg->fc_timestamp_us,
           msg->fc_utc_hhmmss, msg->fc_utc_yymmdd);
}

void
PPSSourceDataCallback(const std_msgs::String::ConstPtr& msg)
{
  ROS_INFO("[%s] PPSSource Data\t: %s", __FUNCTION__, msg->data.c_str());
}
