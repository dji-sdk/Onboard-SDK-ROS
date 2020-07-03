/** @file time_sync_node.cpp
 *  @version 4.0
 *  @date May 2020
 *
 *  @brief sample node of time_sync.
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

#include <ros/ros.h>
#include <dji_osdk_ros/dji_vehicle_node.h>

void timeSyncNmeaSubCallback(const nmea_msgs::Sentence::ConstPtr& timeSyncNmeaMsg)
{
  ROS_INFO("[%s] Recv Time\t: sec:%d nsec:%d", __FUNCTION__, timeSyncNmeaMsg->header.stamp.sec, timeSyncNmeaMsg->header.stamp.nsec);
  ROS_INFO("[%s] NMEA Data\t: %s", __FUNCTION__, timeSyncNmeaMsg->sentence.c_str());
}

void timeSyncGpsUtcSubCallback(const dji_osdk_ros::GPSUTC::ConstPtr& timeSyncGpsUtc)
{
  ROS_INFO("[%s] Recv Time\t: sec:%d nsec:%d", __FUNCTION__, timeSyncGpsUtc->stamp.sec, timeSyncGpsUtc->stamp.nsec);
  ROS_INFO("[%s] GPSUTC Data\t: %s", __FUNCTION__, timeSyncGpsUtc->UTCTimeData.c_str());
}

void timeSyncFcUtcSubCallback(const dji_osdk_ros::FCTimeInUTC::ConstPtr& timeSyncFcUtc)
{
  ROS_INFO("[%s] Recv Time\t: sec:%d nsec:%d", __FUNCTION__, timeSyncFcUtc->stamp.sec, timeSyncFcUtc->stamp.nsec);
  ROS_INFO("[%s] FCUTC Data\t: %u, UTC time: %u, UTC date: %u", __FUNCTION__, timeSyncFcUtc->fc_timestamp_us,
           timeSyncFcUtc->fc_utc_hhmmss, timeSyncFcUtc->fc_utc_yymmdd);
}

void timeSyncPpsSourceSubCallback(const std_msgs::String::ConstPtr& timeSyncPpsSource)
{
  ROS_INFO("[%s] PPSSource Data\t: %s", __FUNCTION__, timeSyncPpsSource->data.c_str());
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "time_sync_node");
  ros::NodeHandle nh;

  ros::Subscriber timeSyncNmeaSub = nh.subscribe("dji_osdk_ros/time_sync_nmea_msg", 10, &timeSyncNmeaSubCallback);
  ros::Subscriber timeSyncGpsUtcSub = nh.subscribe("dji_osdk_ros/time_sync_gps_utc", 10, &timeSyncGpsUtcSubCallback);
  ros::Subscriber timeSyncFcUtcSub = nh.subscribe("dji_osdk_ros/time_sync_fc_time_utc", 10, &timeSyncFcUtcSubCallback);
  ros::Subscriber timeSyncPpsSourceSub = nh.subscribe("dji_osdk_ros/time_sync_pps_source", 10, &timeSyncPpsSourceSubCallback);

  ros::spin();
  return 0;
}