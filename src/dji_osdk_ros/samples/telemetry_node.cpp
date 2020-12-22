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
std_msgs::UInt8 flight_status_;
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
  attitude_data_ = *attitudeData;
}

void batteryStateSubCallback(const sensor_msgs::BatteryState::ConstPtr& batteryState)
{
  battery_state_ = *batteryState;
}

void imuSubCallback(const sensor_msgs::Imu::ConstPtr& imuData)
{
  imu_data_ = *imuData;
}

void flightStatusSubCallback(const std_msgs::UInt8::ConstPtr& flightData)
{
  flight_status_ = *flightData;
}

void gpsHealthSubCallback(const std_msgs::UInt8::ConstPtr& gpsHealth)
{

  gps_health_ = *gpsHealth;
}

void gpsPositionSubCallback(const sensor_msgs::NavSatFix::ConstPtr& gpsPosition)
{
  gps_position_ = *gpsPosition;
}

void voPositionSubCallback(const dji_osdk_ros::VOPosition::ConstPtr& voPosition)
{
  vo_position_ = *voPosition;
}

void heightSubCallback(const std_msgs::Float32::ConstPtr& heightAboveTakeoff)
{
  height_above_takeoff_ = *heightAboveTakeoff;
}

void velocitySubCallback(const geometry_msgs::Vector3Stamped::ConstPtr& velocity)
{
  velocity_ = *velocity;
}

void fromMobileDataSubCallback(const dji_osdk_ros::MobileData::ConstPtr& fromMobileData)
{
  from_mobile_data_ = *fromMobileData;
}

void fromPayloadDataSubCallback(const dji_osdk_ros::PayloadData::ConstPtr& fromPayloadData)
{
  from_payload_data_ = *fromPayloadData;
}

void gimbalAngleSubCallback(const geometry_msgs::Vector3Stamped::ConstPtr& gimbalAngleData)
{
  gimbal_angle_data_ = *gimbalAngleData;
}

void rcDataCallback(const sensor_msgs::Joy::ConstPtr& rcData)
{
  rc_data_ = *rcData;
}

void localPositionSubCallback(const geometry_msgs::PointStamped::ConstPtr& localPosition)
{
  local_position_ = *localPosition;
}

void localFrameRefSubCallback(const sensor_msgs::NavSatFix::ConstPtr& localFrameRef)
{
  local_Frame_ref_ = *localFrameRef;
}

void timeSyncNmeaSubSCallback(const nmea_msgs::Sentence::ConstPtr& timeSyncNmeaMsg)
{
  time_sync_nmea_msg_ = *timeSyncNmeaMsg;
}

void timeSyncGpsUtcSubCallback(const dji_osdk_ros::GPSUTC::ConstPtr& timeSyncGpsUtc)
{
  time_sync_gps_utc_ = *timeSyncGpsUtc;
}

void timeSyncFcUtcSubCallback(const dji_osdk_ros::FCTimeInUTC::ConstPtr& timeSyncFcUtc)
{
  time_sync_fc_utc_ = *timeSyncFcUtc;
}

void timeSyncPpsSourceSubCallback(const std_msgs::String::ConstPtr& timeSyncPpsSource)
{
  time_sync_pps_source_ = *timeSyncPpsSource;
}

void angularRateSubSCallback(const geometry_msgs::Vector3Stamped::ConstPtr& angularRate)
{
  angular_rate_ = *angularRate;
}

void accelerationSubCallback(const geometry_msgs::Vector3Stamped::ConstPtr& acceleration)
{
  acceleration_ = *acceleration;
}

void displayModeSubCallback(const std_msgs::UInt8::ConstPtr& displayMode)
{
  display_mode_ = *displayMode;
}

void triggerSubCallback(const sensor_msgs::TimeReference::ConstPtr& trigger)
{
  trigger_ = *trigger;
}

void rcConnectionStatusSubCallback(const std_msgs::UInt8::ConstPtr& rcConnectionStatus)
{
  rc_connection_status_ = *rcConnectionStatus;
}

void rtkPositionSubCallback(const sensor_msgs::NavSatFix::ConstPtr& rtkPosition)
{
  rtk_position_ = *rtkPosition;
}

void rtkVelocitySubCallback(const geometry_msgs::Vector3Stamped::ConstPtr& rtkVelocity)
{
  rtk_velocity_ = *rtkVelocity;
}

void rtkYawSubCallback(const std_msgs::Int16::ConstPtr& rtkYaw)
{
  rtk_yaw_ = *rtkYaw;
}

void rtkPositionInfoSubCallback(const std_msgs::UInt8::ConstPtr& rtkPositionInfo)
{
  rtk_position_info_ = *rtkPositionInfo;
}

void rtkYawInfoSubCallback(const std_msgs::UInt8::ConstPtr& rtkYawInfo)
{
  rtk_yaw_info_ = *rtkYawInfo;
}

void rtkConnectionStatusSubCallback(const std_msgs::UInt8::ConstPtr& rtkConnectionStatus)
{
  rtk_connection_status_ = *rtkConnectionStatus;
}

void flightAnomalySubCallback(const dji_osdk_ros::FlightAnomaly::ConstPtr& flightAnomaly)
{
  flight_anomaly_ = *flightAnomaly;
}

ros::Subscriber batteryStateSub;
ros::Subscriber imuSub;
ros::Subscriber flightStatusSub;
ros::Subscriber gpsHealthSub;
ros::Subscriber gpsPositionSub;
ros::Subscriber heightSub;
ros::Subscriber localPositionSub;
ros::Subscriber velocitySub;
ros::Subscriber gimbalAngleSub;
ros::Subscriber rcDataSub;
ros::Subscriber attitudeSub;

ros::Subscriber fromMobileDataSub;
ros::Subscriber fromPayloadDataSub;

ros::Subscriber localFrameRefSub;
ros::Subscriber timeSyncNmeaSub;
ros::Subscriber timeSyncGpsUtcSub;
ros::Subscriber timeSyncFcUtcSub;
ros::Subscriber timeSyncPpsSourceSub;

ros::Subscriber voPositionSub;
ros::Subscriber angularRateSub;
ros::Subscriber accelerationSub;
ros::Subscriber displayModeSub;
ros::Subscriber triggerSub;

ros::Subscriber rcConnectionStatusSub;
ros::Subscriber rtkPositionSub;
ros::Subscriber rtkVelocitySub;
ros::Subscriber rtkYawSub;
ros::Subscriber rtkPositionInfoSub;
ros::Subscriber rtkYawInfoSub;
ros::Subscriber rtkConnectionStatusSub;
ros::Subscriber flightAnomalySub;

int main(int argc ,char** argv)
{
  ros::init(argc, argv, "telemetry_node");
  ros::NodeHandle nh;
  bool userSelectBroadcast = false;
  nh.getParam("/vehicle_node/use_broadcast", userSelectBroadcast);

  batteryStateSub  = nh.subscribe("dji_osdk_ros/battery_state", 10, &batteryStateSubCallback);
  imuSub           = nh.subscribe("dji_osdk_ros/imu", 10, &imuSubCallback);
  flightStatusSub  = nh.subscribe("dji_osdk_ros/flight_status", 10, &flightStatusSubCallback);
  gpsHealthSub     = nh.subscribe("dji_osdk_ros/gps_health", 10, &gpsHealthSubCallback);
  gpsPositionSub   = nh.subscribe("dji_osdk_ros/gps_position", 10, &gpsPositionSubCallback);
  heightSub        = nh.subscribe("dji_osdk_ros/height_above_takeoff", 10, &heightSubCallback);
  localPositionSub = nh.subscribe("dji_osdk_ros/local_position", 10, &localPositionSubCallback);
  velocitySub      = nh.subscribe("dji_osdk_ros/velocity", 10, &velocitySubCallback);
  gimbalAngleSub   = nh.subscribe("dji_osdk_ros/gimbal_angle", 10, &gimbalAngleSubCallback);
  rcDataSub        = nh.subscribe("dji_osdk_ros/rc", 10, &rcDataCallback);

  fromMobileDataSub  = nh.subscribe("dji_osdk_ros/from_mobile_data", 10, &fromMobileDataSubCallback);
  fromPayloadDataSub = nh.subscribe("dji_osdk_ros/from_payload_data", 10, &fromPayloadDataSubCallback);

  /* only if you call the service of set_local_pos_reference ,the topic can be valid" */
  localFrameRefSub     = nh.subscribe("dji_osdk_ros/local_frame_ref", 10, &localFrameRefSubCallback);
  timeSyncNmeaSub      = nh.subscribe("dji_osdk_ros/time_sync_nmea_msg", 10, &timeSyncNmeaSubSCallback);
  timeSyncGpsUtcSub    = nh.subscribe("dji_osdk_ros/time_sync_gps_utc", 10, &timeSyncGpsUtcSubCallback);
  timeSyncFcUtcSub     = nh.subscribe("dji_osdk_ros/time_sync_fc_time_utc", 10, &timeSyncFcUtcSubCallback);
  timeSyncPpsSourceSub = nh.subscribe("dji_osdk_ros/time_sync_pps_source", 10, &timeSyncPpsSourceSubCallback);
  
  if (!userSelectBroadcast)
  {
    voPositionSub   = nh.subscribe("dji_osdk_ros/vo_position", 10, &voPositionSubCallback);
    angularRateSub  = nh.subscribe("dji_osdk_ros/angular_velocity_fused", 10, &angularRateSubSCallback);
    accelerationSub = nh.subscribe("dji_osdk_ros/acceleration_ground_fused", 10, &accelerationSubCallback);
    displayModeSub  = nh.subscribe("dji_osdk_ros/display_mode", 10, &displayModeSubCallback);
    triggerSub      = nh.subscribe("dji_osdk_ros/trigger_time", 10, &triggerSubCallback);

    rcConnectionStatusSub  = nh.subscribe("dji_osdk_ros/rc_connection_status", 10, &rcConnectionStatusSubCallback);
    rtkPositionSub         = nh.subscribe("dji_osdk_ros/rtk_position", 10, &rtkPositionSubCallback);
    rtkVelocitySub         = nh.subscribe("dji_osdk_ros/rtk_velocity", 10, &rtkVelocitySubCallback);
    rtkYawSub              = nh.subscribe("dji_osdk_ros/rtk_yaw", 10, &rtkYawSubCallback);
    rtkPositionInfoSub     = nh.subscribe("dji_osdk_ros/rtk_info_position", 10, &rtkPositionInfoSubCallback);
    rtkYawInfoSub          = nh.subscribe("dji_osdk_ros/rtk_info_yaw", 10, &rtkYawInfoSubCallback);
    rtkConnectionStatusSub = nh.subscribe("dji_osdk_ros/rtk_connection_status", 10, &rtkConnectionStatusSubCallback);
    flightAnomalySub       = nh.subscribe("dji_osdk_ros/flight_anomaly", 10, &flightAnomalySubCallback);
    attitudeSub      = nh.subscribe("dji_osdk_ros/attitude", 10, &attitudeSubCallback);
  }

  ros::Duration(1).sleep();
  ros::AsyncSpinner spinner(4);
  spinner.start();

  int elapsedTimeInS = 0;
  int timeToPrintInS = 30;

  while (elapsedTimeInS < timeToPrintInS)
  {
    std::cout << "Counter = " << elapsedTimeInS << ":\n";
    std::cout << "-------\n";

    ROS_INFO("battery Info :");
    ROS_INFO("battery's capacity: %f", battery_state_.capacity);
    ROS_INFO("battery's voltage: %f", battery_state_.voltage);
    ROS_INFO("battery's current: %f", battery_state_.current);
    ROS_INFO("battery's percentage : %f\n", battery_state_.percentage);

    ROS_INFO("height above takeoff:");
    ROS_INFO("height_above_takeoff_ :%f\n", height_above_takeoff_.data);
    ROS_INFO("gimbalAngleData:");
    ROS_INFO("gimbal_angle_data_(x,y,z) :%f, %f, %f\n", gimbal_angle_data_.vector.x, gimbal_angle_data_.vector.y,
              gimbal_angle_data_.vector.z);
    ROS_INFO("rcData(here only print basic data):");
    if (rc_data_.axes.size() >= 4)
    {
       ROS_INFO("rc_data_(roll,pitch,yaw, throttle) :%f, %f, %f, %f\n", rc_data_.axes[0], rc_data_.axes[1],
                 rc_data_.axes[2], rc_data_.axes[3]);
    }

    ROS_INFO("gps Position:");
    ROS_INFO("gps_position_(latitude, longitude, altitude) :%f, %f, %f \n",
              gps_position_.latitude, gps_position_.longitude, gps_position_.altitude);
    ROS_INFO("gps Health:");
    ROS_INFO("gps_health_ :%d \n", gps_health_.data);
    ROS_INFO("flight Info :");
    ROS_INFO("flight_status_: %d\n",flight_status_.data);
    ROS_INFO("localPosition:");
    ROS_INFO("local_position_(x,y,z) :%f, %f, %f\n", local_position_.point.x, local_position_.point.y,
              local_position_.point.z);
    ROS_INFO("imu Info :");
    ROS_INFO("imu w: %f",imu_data_.orientation.w);
    ROS_INFO("imu x: %f",imu_data_.orientation.x);
    ROS_INFO("imu y: %f",imu_data_.orientation.y);
    ROS_INFO("imu z: %f\n",imu_data_.orientation.z);
    ROS_INFO("imu linear acceleration x: %f", imu_data_.linear_acceleration.x);
    ROS_INFO("imu linear acceleration y: %f", imu_data_.linear_acceleration.y);
    ROS_INFO("imu linear acceleration z: %f", imu_data_.linear_acceleration.z);
    ROS_INFO("fromMobileData:");
    if (from_mobile_data_.data.size() > 1)
    {
      ROS_INFO("from_mobile_data_(0,1....) :%d, %d\n", from_mobile_data_.data[0], from_mobile_data_.data[1]);
    }

    ROS_INFO("fromPayloadData:");
    if (from_payload_data_.data.size() > 1)
    {
      ROS_INFO("from_payload_data_(0,1....) :%d, %d\n", from_payload_data_.data[0], from_payload_data_.data[1]);
    }
    auto local_frame_ref_client = nh.serviceClient<dji_osdk_ros::SetLocalPosRef>("set_local_pos_reference");
    dji_osdk_ros::SetLocalPosRef local_frame_ref;
    local_frame_ref_client.call(local_frame_ref);
    if (local_frame_ref.response.result)
    {
       ROS_INFO("localFrameRef:");
       ROS_INFO("local_Frame_ref_(latitude, longitude, altitude) :%f, %f, %f\n ",
       local_Frame_ref_.latitude, local_Frame_ref_.longitude, local_Frame_ref_.altitude);
    }

    ROS_INFO("timeSyncNmeaMsg:");
    ROS_INFO("time_sync_nmea_msg_ :%s \n",
              time_sync_nmea_msg_.sentence.data());
    ROS_INFO("timeSyncGpsUtc:");
    ROS_INFO("time_sync_gps_utc_(timestamp,UTCTimeData):%d, %s \n",
              time_sync_gps_utc_.stamp.sec, time_sync_gps_utc_.UTCTimeData.data());
    ROS_INFO("timeSyncFcUtc:");      
    ROS_INFO("time_sync_fc_utc_(fc_timestamp_us,fc_utc_yymmdd,fc_utc_hhmmss):%d, %d, %d\n",
              time_sync_fc_utc_.fc_timestamp_us, time_sync_fc_utc_.fc_utc_yymmdd,time_sync_fc_utc_.fc_utc_hhmmss);
    ROS_INFO("timeSyncPpsSource:");
    ROS_INFO("time_sync_pps_source_ :%s\n", time_sync_pps_source_.data.data());
    if (!userSelectBroadcast)
    {
      //5HZ
        ROS_INFO("if RTK is available:");
        ROS_INFO("rtkPosition:");
        ROS_INFO("rtk_position_(latitude, longitude, altitude) :%f, %f, %f \n",
                rtk_position_.latitude, rtk_position_.longitude, rtk_position_.altitude);
        ROS_INFO("rtkVelocity:");
        ROS_INFO("rtk_velocity_(x,y,z) :%f, %f, %f\n", rtk_velocity_.vector.x,
                rtk_velocity_.vector.y, rtk_velocity_.vector.z);
        ROS_INFO("rtkYaw:");
        ROS_INFO("rtk_yaw_: %d\n", rtk_yaw_.data);
        ROS_INFO("rtkPositionInfo:");
        ROS_INFO("rtk_position_info_: %d\n", rtk_position_info_.data);
        ROS_INFO("rtkYawInfo:");
        ROS_INFO("rtk_yaw_info_: %d\n", rtk_yaw_info_.data);
        ROS_INFO("rtkConnectionStatus:");
        ROS_INFO("rtk_connection_status_: %d\n", rtk_connection_status_.data);
      //50HZ
        ROS_INFO("velocity:");
        ROS_INFO("velocity(x,y,z) :%f, %f, %f\n", velocity_.vector.x, velocity_.vector.y,
                  velocity_.vector.z);
        ROS_INFO("vo Position:");
        ROS_INFO("vo_position_(x, y, z ,xHealth, yHealth, zHealth:%f, %f, %f, %d, %d, %d\n ",
                  vo_position_.x, vo_position_.y, vo_position_.z,
                  vo_position_.xHealth, vo_position_.yHealth, vo_position_.zHealth);
        ROS_INFO("displayMode:");
        ROS_INFO("display_mode_: %d\n", display_mode_.data);
        ROS_INFO("rcConnectionStatus:");
        ROS_INFO("rc_connection_status_: %d\n", rc_connection_status_.data);
        ROS_INFO("flightAnomaly:");
        if (flight_anomaly_.data && dji_osdk_ros::FlightAnomaly::COMPASS_INSTALLATION_ERROR)
        {
           ROS_INFO("COMPASS_INSTALLATION_ERROR\n");
        }
        if (flight_anomaly_.data && dji_osdk_ros::FlightAnomaly::IMU_INSTALLATION_ERROR)
        {
           ROS_INFO("IMU_INSTALLATION_ERROR\n");
        }
      //100HZ
        ROS_INFO("attitude Info :");
        ROS_INFO("attitude w: %f",attitude_data_.quaternion.w);
        ROS_INFO("attitude x: %f",attitude_data_.quaternion.x);
        ROS_INFO("attitude y: %f",attitude_data_.quaternion.y);
        ROS_INFO("attitude z: %f\n",attitude_data_.quaternion.z);
        ROS_INFO("angularRate:");
        ROS_INFO("angular_rate_(x,y,z) :%f, %f, %f\n", angular_rate_.vector.x,
                  angular_rate_.vector.y, angular_rate_.vector.z);
        ROS_INFO("acceleration:");
        ROS_INFO("acceleration_(x,y,z) :%f, %f, %f\n", acceleration_.vector.x,
                   acceleration_.vector.y, acceleration_.vector.z);
      //400HZ
        ROS_INFO("trigger:");
        ROS_INFO("trigger_fc_time(if you choose align_time): %d", trigger_.header.stamp.nsec);
        ROS_INFO("trigger_ros_time: %d", trigger_.time_ref.nsec);
        ROS_INFO("trigger_time_source: %s", trigger_.source.c_str());
    }
    std::cout << "-------\n\n";
    ros::Duration(1).sleep();
    elapsedTimeInS += 1;
  }

  std::cout << "Done printing!\n";
  
  ROS_INFO_STREAM("Finished. Press CTRL-C to terminate the node");
  ros::waitForShutdown();

  return 0;
}

