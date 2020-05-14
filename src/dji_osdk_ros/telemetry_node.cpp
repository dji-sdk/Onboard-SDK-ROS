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

sensor_msgs::BatteryState battery_state_;
geometry_msgs::QuaternionStamped attitude_data_;
sensor_msgs::Imu imu_data_;
std_msgs::UInt8 flight_data_;
std_msgs::UInt8 gps_health_;
sensor_msgs::NavSatFix gps_position_;
dji_osdk_ros::VOPosition vo_position_;
std_msgs::Float32 height_above_takeoff_;
geometry_msgs::Vector3Stamped velocity_;
dji_osdk_ros::MobileData from_mobile_data_;
dji_osdk_ros::PayloadData from_payload_data_;
geometry_msgs::Vector3Stamped gimbal_angle_data_;
sensor_msgs::Joy rc_data_;
geometry_msgs::PointStamped local_position_;
sensor_msgs::NavSatFix local_Frame_ref_;
nmea_msgs::Sentence time_sync_nmea_msg_;
dji_osdk_ros::GPSUTC time_sync_gps_utc_;
dji_osdk_ros::FCTimeInUTC time_sync_fc_utc_;
std_msgs::String time_sync_pps_source_;
geometry_msgs::Vector3Stamped angular_rate_;
geometry_msgs::Vector3Stamped acceleration_;
std_msgs::UInt8 display_mode_;
sensor_msgs::TimeReference trigger_;
std_msgs::UInt8 rc_connection_status_;
sensor_msgs::NavSatFix rtk_position_;
geometry_msgs::Vector3Stamped rtk_velocity_;
std_msgs::Int16 rtk_yaw_;
std_msgs::UInt8 rtk_position_info_;
std_msgs::UInt8 rtk_yaw_info_;
std_msgs::UInt8 rtk_connection_status_;
dji_osdk_ros::FlightAnomaly flight_anomaly_;

void attitudeSubCallback(const geometry_msgs::QuaternionStampedConstPtr& attitudeData)
{
  ROS_INFO("attitude Info :");
  attitude_data_ = *attitudeData;
  ROS_INFO("attitude w: %f",attitude_data_.quaternion.w);
  ROS_INFO("attitude x: %f",attitude_data_.quaternion.x);
  ROS_INFO("attitude y: %f",attitude_data_.quaternion.y);
  ROS_INFO("attitude z: %f\n",attitude_data_.quaternion.z);
  ros::Duration(1.0).sleep();
}

void batteryStateSubCallback(const sensor_msgs::BatteryState::ConstPtr& batteryState)
{
  ROS_INFO("battery Info :");
  battery_state_ = *batteryState;
  ROS_INFO("battery's capacity: %f", battery_state_.capacity);
  ROS_INFO("battery's voltage: %f", battery_state_.voltage);
  ROS_INFO("battery's current: %f", battery_state_.current);
  ROS_INFO("battery's percentage : %f\n", battery_state_.percentage);
  ros::Duration(1.0).sleep();
}

void imuSubCallback(const sensor_msgs::Imu::ConstPtr& imuData)
{
  ROS_INFO("imu Info :");
  imu_data_ = *imuData;
  ROS_INFO("imu w: %f",imu_data_.orientation.w);
  ROS_INFO("imu x: %f",imu_data_.orientation.x);
  ROS_INFO("imu y: %f",imu_data_.orientation.y);
  ROS_INFO("imu z: %f\n",imu_data_.orientation.z);
  ros::Duration(1.0).sleep();
}

void flightStatusSubCallback(const std_msgs::UInt8::ConstPtr& flightData)
{
  ROS_INFO("flight Info :");
  flight_data_ = *flightData;
  ROS_INFO("flight_data_: %d\n",flight_data_.data);
  ros::Duration(1.0).sleep();
}

void gpsHealthSubCallback(const std_msgs::UInt8::ConstPtr& gpsHealth)
{
  ROS_INFO("gps Health:");
  gps_health_ = *gpsHealth;
  ROS_INFO("gps_health_ :%d \n", gps_health_.data);
  ros::Duration(1.0).sleep();
}

void gpsPositionSubCallback(const sensor_msgs::NavSatFix::ConstPtr& gpsPosition)
{
  ROS_INFO("gps Position:");
  gps_position_ = *gpsPosition;
  ROS_INFO("gps_position_(latitude, longitude, altitude) :%f, %f, %f \n",
            gps_position_.latitude, gps_position_.longitude, gps_position_.altitude);
  ros::Duration(1.0).sleep();
}

void voPositionSubCallback(const dji_osdk_ros::VOPosition::ConstPtr& voPosition)
{
  ROS_INFO("vo Position:");
  vo_position_ = *voPosition;
  ROS_INFO("vo_position_(x, y, z ,xHealth, yHealth, zHealth:%f, %f, %f, %d, %d, %d\n ",
           vo_position_.x, vo_position_.y, vo_position_.z,
           vo_position_.xHealth, vo_position_.yHealth, vo_position_.zHealth);
  ros::Duration(1.0).sleep();
}

void heightSubCallback(const std_msgs::Float32::ConstPtr& heightAboveTakeoff)
{
  ROS_INFO("height above takeoff:");
  height_above_takeoff_ = *heightAboveTakeoff;
  ROS_INFO("height_above_takeoff_ :%f\n", height_above_takeoff_.data);
  ros::Duration(1.0).sleep();
}

void velocitySubCallback(const geometry_msgs::Vector3Stamped::ConstPtr& velocity)
{
  ROS_INFO("velocity:");
  velocity_ = *velocity;
  ROS_INFO("velocity(x,y,z) :%f, %f, %f\n", velocity_.vector.x, velocity_.vector.y,
            velocity_.vector.z);
  ros::Duration(1.0).sleep();
}

void fromMobileDataSubCallback(const dji_osdk_ros::MobileData::ConstPtr& fromMobileData)
{
  ROS_INFO("fromMobileData:");
  from_mobile_data_ = *fromMobileData;
  if (from_mobile_data_.data.size() > 1)
  {
    ROS_INFO("from_mobile_data_(0,1....) :%d, %d\n", from_mobile_data_.data[0], from_mobile_data_.data[1]);
  }
  ros::Duration(1.0).sleep();
}

void fromPayloadDataSubCallback(const dji_osdk_ros::PayloadData::ConstPtr& fromPayloadData)
{
  ROS_INFO("fromPayloadData:");
  from_payload_data_ = *fromPayloadData;
  if (from_payload_data_.data.size() > 1)
  {
    ROS_INFO("from_payload_data_(0,1....) :%d, %d\n", from_payload_data_.data[0], from_payload_data_.data[1]);
  }
  ros::Duration(1.0).sleep();
}

void gimbalAngleSubCallback(const geometry_msgs::Vector3Stamped::ConstPtr& gimbalAngleData)
{
  ROS_INFO("gimbalAngleData:");
  gimbal_angle_data_ = *gimbalAngleData;
  ROS_INFO("gimbal_angle_data_(x,y,z) :%f, %f, %f\n", gimbal_angle_data_.vector.x, gimbal_angle_data_.vector.y,
           gimbal_angle_data_.vector.z);
  ros::Duration(1.0).sleep();
}

void rcDataCallback(const sensor_msgs::Joy::ConstPtr& rcData)
{
  ROS_INFO("rcData(here only print basic data):");
  rc_data_ = *rcData;
  if (rc_data_.axes.size() >= 4)
  {
    ROS_INFO("rc_data_(roll,pitch,yaw, throttle) :%f, %f, %f, %f\n", rc_data_.axes[0], rc_data_.axes[1],
             rc_data_.axes[2], rc_data_.axes[3]);
  }
  ros::Duration(1.0).sleep();
}

void localPositionSubCallback(const geometry_msgs::PointStamped::ConstPtr& localPosition)
{
  ROS_INFO("localPosition:");
  local_position_ = *localPosition;
  ROS_INFO("local_position_(x,y,z) :%f, %f, %f\n", local_position_.point.x, local_position_.point.y,
           local_position_.point.z);
  ros::Duration(1.0).sleep();
}

void localFrameRefSubCallback(const sensor_msgs::NavSatFix::ConstPtr& localFrameRef)
{
  ROS_INFO("localFrameRef:");
  local_Frame_ref_ = *localFrameRef;
  ROS_INFO("local_Frame_ref_(latitude, longitude, altitude) :%f, %f, %f\n ",
           local_Frame_ref_.latitude, local_Frame_ref_.longitude, local_Frame_ref_.altitude);
  ros::Duration(1.0).sleep();
}

void timeSyncNmeaSubSCallback(const nmea_msgs::Sentence::ConstPtr& timeSyncNmeaMsg)
{
  ROS_INFO("timeSyncNmeaMsg:");
  time_sync_nmea_msg_ = *timeSyncNmeaMsg;
  ROS_INFO("time_sync_nmea_msg_ :%s \n",
           time_sync_nmea_msg_.sentence.data());
  ros::Duration(1.0).sleep();
}

void timeSyncGpsUtcSubCallback(const dji_osdk_ros::GPSUTC::ConstPtr& timeSyncGpsUtc)
{
  ROS_INFO("timeSyncGpsUtc:");
  time_sync_gps_utc_ = *timeSyncGpsUtc;
  ROS_INFO("time_sync_gps_utc_(timestamp,UTCTimeData):%d, %s \n",
           time_sync_gps_utc_.stamp.sec, time_sync_gps_utc_.UTCTimeData.data());
  ros::Duration(1.0).sleep();
}

void timeSyncFcUtcSubCallback(const dji_osdk_ros::FCTimeInUTC::ConstPtr& timeSyncFcUtc)
{
  time_sync_fc_utc_ = *timeSyncFcUtc;
  ROS_INFO("time_sync_gps_utc_(fc_timestamp_us,fc_utc_yymmdd,fc_utc_hhmmss):%d, %d, %d\n",
           time_sync_fc_utc_.fc_timestamp_us, time_sync_fc_utc_.fc_utc_yymmdd,time_sync_fc_utc_.fc_utc_hhmmss);
  ros::Duration(1.0).sleep();
}

void timeSyncPpsSourceSubCallback(const std_msgs::String::ConstPtr& timeSyncPpsSource)
{
  ROS_INFO("timeSyncPpsSource:");
  time_sync_pps_source_ = *timeSyncPpsSource;
  ROS_INFO("time_sync_pps_source_ :%s\n", time_sync_pps_source_.data.data());
  ros::Duration(1.0).sleep();
}

void angularRateSubSCallback(const geometry_msgs::Vector3Stamped::ConstPtr& angularRate)
{
  ROS_INFO("angularRate:");
  angular_rate_ = *angularRate;
  ROS_INFO("angular_rate_(x,y,z) :%f, %f, %f\n", angular_rate_.vector.x,
           angular_rate_.vector.y, angular_rate_.vector.z);
  ros::Duration(1.0).sleep();
}

void accelerationSubCallback(const geometry_msgs::Vector3Stamped::ConstPtr& acceleration)
{
  ROS_INFO("acceleration:");
  acceleration_ = *acceleration;
  ROS_INFO("acceleration_(x,y,z) :%f, %f, %f\n", acceleration_.vector.x,
           acceleration_.vector.y, acceleration_.vector.z);
  ros::Duration(1.0).sleep();
}

void displayModeSubCallback(const std_msgs::UInt8::ConstPtr& displayMode)
{
  ROS_INFO("displayMode:");
  display_mode_ = *displayMode;
  ROS_INFO("display_mode_: %d\n", display_mode_.data);
  ros::Duration(1.0).sleep();
}

void triggerSubCallback(const sensor_msgs::TimeReference::ConstPtr& trigger)
{
  ROS_INFO("trigger:");
  trigger_ = *trigger;
  ROS_INFO("trigger_: %s\n", trigger_.source.data());
  ros::Duration(1.0).sleep();
}

void rcConnectionStatusSubCallback(const std_msgs::UInt8::ConstPtr& rcConnectionStatus)
{
  ROS_INFO("rcConnectionStatus:");
  rc_connection_status_ = *rcConnectionStatus;
  ROS_INFO("rc_connection_status_: %d\n", rc_connection_status_.data);
  ros::Duration(1.0).sleep();
}

void rtkPositionSubCallback(const sensor_msgs::NavSatFix::ConstPtr& rtkPosition)
{
  ROS_INFO("rtkPosition:");
  rtk_position_ = *rtkPosition;
  ROS_INFO("rtk_position_(latitude, longitude, altitude) :%f, %f, %f \n",
           rtk_position_.latitude, rtk_position_.longitude, rtk_position_.altitude);
  ros::Duration(1.0).sleep();
}

void rtkVelocitySubCallback(const geometry_msgs::Vector3Stamped::ConstPtr& rtkVelocity)
{
  ROS_INFO("rtkVelocity:");
  rtk_velocity_ = *rtkVelocity;
  ROS_INFO("rtk_velocity_(x,y,z) :%f, %f, %f\n", rtk_velocity_.vector.x,
            rtk_velocity_.vector.y, rtk_velocity_.vector.z);
  ros::Duration(1.0).sleep();
}

void rtkYawSubCallback(const std_msgs::Int16::ConstPtr& rtkYaw)
{
  ROS_INFO("rtkYaw:");
  rtk_yaw_ = *rtkYaw;
  ROS_INFO("rtk_yaw_: %d\n", rtk_yaw_.data);
  ros::Duration(1.0).sleep();
}

void rtkPositionInfoSubCallback(const std_msgs::UInt8::ConstPtr& rtkPositionInfo)
{
  ROS_INFO("rtkPositionInfo:");
  rtk_position_info_ = *rtkPositionInfo;
  ROS_INFO("rtk_position_info_: %d\n", rtk_position_info_.data);
  ros::Duration(1.0).sleep();

}

void rtkYawInfoSubCallback(const std_msgs::UInt8::ConstPtr& rtkYawInfo)
{
  ROS_INFO("rtkYawInfo:");
  rtk_yaw_info_ = *rtkYawInfo;
  ROS_INFO("rtk_yaw_info_: %d\n", rtk_yaw_info_.data);
  ros::Duration(1.0).sleep();
}

void rtkConnectionStatusSubCallback(const std_msgs::UInt8::ConstPtr& rtkConnectionStatus)
{
  ROS_INFO("rtkConnectionStatus:");
  rtk_connection_status_ = *rtkConnectionStatus;
  ROS_INFO("rtk_connection_status_: %d\n", rtk_connection_status_.data);
  ros::Duration(1.0).sleep();
}

void flightAnomalySubCallback(const dji_osdk_ros::FlightAnomaly::ConstPtr& flightAnomaly)
{
  ROS_INFO("flightAnomaly:");
  flight_anomaly_ = *flightAnomaly;
  if (flight_anomaly_.data && dji_osdk_ros::FlightAnomaly::COMPASS_INSTALLATION_ERROR)
  {
    ROS_INFO("COMPASS_INSTALLATION_ERROR\n");
  }
  if (flight_anomaly_.data && dji_osdk_ros::FlightAnomaly::IMU_INSTALLATION_ERROR)
  {
    ROS_INFO("IMU_INSTALLATION_ERROR\n");
  }

  // etc...
  ros::Duration(1.0).sleep();
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

