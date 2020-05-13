/** @file telemetry_node.cpp
 *  @version 4.0
 *  @date May 2020
 *
 *  @brief sample node of telemetry.
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
#include <ros/ros.h>
#include <dji_osdk_ros/common_type.h>
#include <dji_osdk_ros/dji_vehicle_node.h>

//CODE
using namespace dji_osdk_ros;

sensor_msgs::BatteryState batteryState_;
geometry_msgs::QuaternionStamped attitudeData_;
sensor_msgs::Imu imuData_;
std_msgs::UInt8 flightData_;
std_msgs::UInt8 gpsHealth_;
sensor_msgs::NavSatFix gpsPosition_;
dji_osdk_ros::VOPosition voPosition_;
std_msgs::Float32 heightAboveTakeoff_;
geometry_msgs::Vector3Stamped velocity_;
dji_osdk_ros::MobileData fromMobileData_;
dji_osdk_ros::PayloadData fromPayloadData_;
geometry_msgs::Vector3Stamped gimbalAngleData_;
sensor_msgs::Joy rcData_;
geometry_msgs::PointStamped localPosition_;
sensor_msgs::NavSatFix localFrameRef_;
nmea_msgs::Sentence timeSyncNmeaMsg_;
dji_osdk_ros::GPSUTC timeSyncGpsUtc_;
dji_osdk_ros::FCTimeInUTC timeSyncFcUtc_;
std_msgs::String timeSyncPpsSource_;
geometry_msgs::Vector3Stamped angularRate_;
geometry_msgs::Vector3Stamped acceleration_;
std_msgs::UInt8 displayMode_;
sensor_msgs::TimeReference trigger_;
std_msgs::UInt8 rcConnectionStatus_;
sensor_msgs::NavSatFix rtkPosition_;
geometry_msgs::Vector3Stamped rtkVelocity_;
std_msgs::Int16 rtkYaw_;
std_msgs::UInt8 rtkPositionInfo_;
std_msgs::UInt8 rtkYawInfo_;
std_msgs::UInt8 rtkConnectionStatus_;
dji_osdk_ros::FlightAnomaly flightAnomaly_;

void attitudeSubCallback(const geometry_msgs::QuaternionStamped attitudeData)
{
  attitudeData_ = attitudeData;
}

void batteryStateSubCallback(const sensor_msgs::BatteryState& batteryState)
{
  batteryState_ = batteryState;
  ROS_INFO("%f", batteryState_.voltage);
  ROS_INFO("%f", batteryState_.percentage);
}

void imuSubCallback(const sensor_msgs::Imu imuData)
{
  imuData_ = imuData;
}

void flightStatusSubCallback(const std_msgs::UInt8 flightData)
{
  flightData_ = flightData;
}

void gpsHealthSubCallback(const std_msgs::UInt8 gpsHealth)
{
  gpsHealth_ = gpsHealth;
}

void gpsPositionSubCallback(sensor_msgs::NavSatFix gpsPosition)
{
  gpsPosition_ = gpsPosition;
}

void voPositionSubCallback(const dji_osdk_ros::VOPosition voPosition)
{
  voPosition_ = voPosition;
}

void heightSubCallback(const std_msgs::Float32 heightAboveTakeoff)
{
  heightAboveTakeoff_ = heightAboveTakeoff;
}

void velocitySubCallback(const geometry_msgs::Vector3Stamped velocity)
{
  velocity_ = velocity;
}

void fromMobileDataSubCallback(const dji_osdk_ros::MobileData fromMobileData)
{
  fromMobileData_ = fromMobileData;
}

void fromPayloadDataSubCallback(const dji_osdk_ros::PayloadData fromPayloadData)
{
  fromPayloadData_ = fromPayloadData;
}

void gimbalAngleSubCallback(const geometry_msgs::Vector3Stamped gimbalAngleData)
{
  gimbalAngleData_ = gimbalAngleData;
}

void rcDataCallback(const sensor_msgs::Joy rcData)
{
  rcData_ = rcData;
}

void localPositionSubCallback(const geometry_msgs::PointStamped localPosition)
{
  localPosition_ = localPosition;
}

void localFrameRefSubCallback(const sensor_msgs::NavSatFix localFrameRef)
{
  localFrameRef_ = localFrameRef;
}

void timeSyncNmeaSubSCallback(const nmea_msgs::Sentence timeSyncNmeaMsg)
{
  timeSyncNmeaMsg_ = timeSyncNmeaMsg;
}

void timeSyncGpsUtcSubCallback(const dji_osdk_ros::GPSUTC timeSyncGpsUtc)
{
  timeSyncGpsUtc_ = timeSyncGpsUtc;
}

void timeSyncFcUtcSubCallback(const dji_osdk_ros::FCTimeInUTC timeSyncFcUtc)
{
  timeSyncFcUtc_ = timeSyncFcUtc;
}

void timeSyncPpsSourceSubCallback(const std_msgs::String timeSyncPpsSource)
{
  timeSyncPpsSource_ = timeSyncPpsSource;
}

void angularRateSubSCallback(const geometry_msgs::Vector3Stamped angularRate)
{
  angularRate_ = angularRate;
}

void accelerationSubCallback(const geometry_msgs::Vector3Stamped acceleration)
{
  acceleration_ = acceleration;
}

void displayModeSubCallback(const std_msgs::UInt8 displayMode)
{
  displayMode_ = displayMode;
}

void triggerSubCallback(const sensor_msgs::TimeReference trigger)
{
  trigger_ = trigger;
}

void rcConnectionStatusSubCallback(const std_msgs::UInt8 rcConnectionStatus)
{
  rcConnectionStatus_ = rcConnectionStatus;
}

void rtkPositionSubCallback(const sensor_msgs::NavSatFix rtkPosition)
{
  rtkPosition_ = rtkPosition;
}

void rtkVelocitySubCallback(const geometry_msgs::Vector3Stamped rtkVelocity)
{
  rtkVelocity_ = rtkVelocity;
}

void rtkYawSubCallback(const std_msgs::Int16 rtkYaw)
{
  rtkYaw_ = rtkYaw;
}

void rtkPositionInfoSubCallback(const std_msgs::UInt8 rtkPositionInfo)
{
  rtkPositionInfo_ = rtkPositionInfo;
}

void rtkYawInfoSubCallback(const std_msgs::UInt8 rtkYawInfo)
{
  rtkYawInfo_ = rtkYawInfo;
}

void rtkConnectionStatusSubCallback(const std_msgs::UInt8 rtkConnectionStatus)
{
  rtkConnectionStatus_ = rtkConnectionStatus;

}

void flightAnomalySubCallback(const dji_osdk_ros::FlightAnomaly flightAnomaly)
{
  flightAnomaly_ = flightAnomaly;
}

int main(int argc ,char** argv)
{
  ros::init(argc, argv, "telemetry_node");
  ros::NodeHandle nh;
  bool userSelectBroadcast = false;
  nh.getParam("/vehicle_node/use_broadcast", userSelectBroadcast);

  ros::Subscriber attitudeSub = nh.subscribe("dji_osdk_ros/attitude", 10, &attitudeSubCallback);
  ros::Subscriber batteryStateSub = nh.subscribe("dji_osdk_ros/battery_state", 10, &batteryStateSubCallback);
  ros::Subscriber imuSub = nh.subscribe("dji_osdk_ros/imu", 10, &imuSubCallback);
  ros::Subscriber flightStatusSub = nh.subscribe("dji_osdk_ros/flight_status", 10, &flightStatusSubCallback);
  ros::Subscriber gpsHealthSub = nh.subscribe("dji_osdk_ros/gps_health", 10, &gpsHealthSubCallback);
  ros::Subscriber gpsPositionSub = nh.subscribe("dji_osdk_ros/gps_position", 10, &gpsPositionSubCallback);
  ros::Subscriber voPositionSub = nh.subscribe("dji_osdk_ros/vo_position", 10, &voPositionSubCallback);
  ros::Subscriber heightSub = nh.subscribe("dji_osdk_ros/height_above_takeoff", 10, &heightSubCallback);
  ros::Subscriber velocitySub = nh.subscribe("dji_osdk_ros/velocity", 10, &velocitySubCallback);
  ros::Subscriber fromMobileDataSub = nh.subscribe("dji_osdk_ros/from_mobile_data", 10, &fromMobileDataSubCallback);
  ros::Subscriber fromPayloadDataSub = nh.subscribe("dji_osdk_ros/from_payload_data", 10, &fromPayloadDataSubCallback);
  ros::Subscriber gimbalAngleSub = nh.subscribe("dji_osdk_ros/gimbal_angle", 10, &gimbalAngleSubCallback);
  ros::Subscriber rcDataSub = nh.subscribe("dji_osdk_ros/rc", 10, &rcDataCallback);
  ros::Subscriber flightAnomalySub = nh.subscribe("dji_osdk_ros/flight_anomaly", 10, &flightAnomalySubCallback);
  ros::Subscriber localPositionSub = nh.subscribe("dji_osdk_ros/local_position", 10, &localPositionSubCallback);
  ros::Subscriber localFrameRefSub = nh.subscribe("dji_osdk_ros/local_frame_ref", 10, &localFrameRefSubCallback);
  ros::Subscriber timeSyncNmeaSub = nh.subscribe("dji_osdk_ros/time_sync_nmea_msg", 10, &timeSyncNmeaSubSCallback);
  ros::Subscriber timeSyncGpsUtcSub = nh.subscribe("dji_osdk_ros/time_sync_gps_utc", 10, &timeSyncGpsUtcSubCallback);
  ros::Subscriber timeSyncFcUtcSub = nh.subscribe("dji_osdk_ros/time_sync_fc_time_utc", 10, &timeSyncFcUtcSubCallback);
  ros::Subscriber timeSyncPpsSourceSub = nh.subscribe("dji_osdk_ros/time_sync_pps_source", 10, &timeSyncPpsSourceSubCallback);

  if(!userSelectBroadcast)
  {
    ros::Subscriber angularRateSub = nh.subscribe("dji_osdk_ros/angular_velocity_fused", 10, &angularRateSubSCallback);
    ros::Subscriber accelerationSub = nh.subscribe("dji_osdk_ros/acceleration_ground_fused", 10, &accelerationSubCallback);
    ros::Subscriber displayModeSub = nh.subscribe("dji_osdk_ros/display_mode", 10, &displayModeSubCallback);
    ros::Subscriber triggerSub = nh.subscribe("dji_osdk_ros/trigger_time", 10, &triggerSubCallback);
  }

  ros::Subscriber rcConnectionStatusSub = nh.subscribe("dji_osdk_ros/rc_connection_status", 10, &rcConnectionStatusSubCallback);
  ros::Subscriber rtkPositionSub = nh.subscribe("dji_osdk_ros/rtk_position", 10, &rtkPositionSubCallback);
  ros::Subscriber rtkVelocitySub = nh.subscribe("dji_osdk_ros/rtk_velocity", 10, &rtkVelocitySubCallback);
  ros::Subscriber rtkYawSub = nh.subscribe("dji_osdk_ros/rtk_yaw", 10, &rtkYawSubCallback);
  ros::Subscriber rtkPositionInfoSub = nh.subscribe("dji_osdk_ros/rtk_info_position", 10, &rtkPositionInfoSubCallback);
  ros::Subscriber rtkYawInfoSub = nh.subscribe("dji_osdk_ros/rtk_info_yaw", 10, &rtkYawInfoSubCallback);
  ros::Subscriber rtkConnectionStatusSub = nh.subscribe("dji_osdk_ros/rtk_connection_status", 10, &rtkConnectionStatusSubCallback);
  ROS_INFO_STREAM("Finished. Press CTRL-C to terminate the node");

  ros::spin();
  return 0;
}

