/** @file flight_controller.cc
 *  @version 4.0
 *  @date Mar, 2020
 *
 *  @brief
 *  FlightControllerROS:
 *
 *  @copyright 2020 DJI. All rights reserved.
 *
 */

#include "dji_sdk/dji_sdk.h"
#include "dji_sdk/dji_sdk_node.h"

#include <dji_sdk_demo/flight_controller_ros.hh>

#include <dji_sdk/SDKControlAuthority.h>
#include <dji_sdk/SetLocalPosRef.h>
#include <dji_sdk/DroneTaskControl.h>
#include <dji_sdk/QueryDroneVersion.h>

const float deg2rad = C_PI/180.0;
const float rad2deg = 180.0/C_PI;

FlightControllerROS::FlightControllerROS()
{
  attitude_suber_ = node_handler_.subscribe("dji_sdk/attitude", 10, &FlightControllerROS::attitudeCallback, this);
  gps_pos_suber_ = node_handler_.subscribe("dji_sdk/gps_position", 10, &FlightControllerROS::gpsCallback, this);
  flight_status_suber_ = node_handler_.subscribe("dji_sdk/flight_status", 10, &FlightControllerROS::flightStatusCallback, this);
  display_mode_suber_ = node_handler_.subscribe("dji_sdk/display_mode", 10, &FlightControllerROS::displayModeCallback, this);
  local_pos_suber_ = node_handler_.subscribe("dji_sdk/local_position", 10, &FlightControllerROS::localPositionCallback, this);;

  ctrl_authority_client_ = node_handler_.serviceClient<dji_sdk::SDKControlAuthority>("dji_sdk/sdk_control_authority");
  enable_local_pos_client_ = node_handler_.serviceClient<dji_sdk::SetLocalPosRef> ("dji_sdk/set_local_pos_ref");
  query_version_client_ = node_handler_.serviceClient<dji_sdk::QueryDroneVersion>("dji_sdk/query_drone_version");
}

void FlightControllerROS::init()
{
  drone_version_ = getVersion();

  if(gainControl() == false)
  {
    ROS_ERROR("Obtain flight control failed!");
//    exit(1);
  }

  if(enableLocalPosition() == false)
  {
    ROS_ERROR("GPS health insufficient - No local frame reference for height. Exiting.");
//    exit(1);
  }

  callback_thread_ = std::thread(&FlightControllerROS::listenCallback, this);
}

void FlightControllerROS::listenCallback()
{
  ROS_INFO_STREAM("Listen callbacks Thread");
  ros::spin();
}

uint32_t FlightControllerROS::getVersion()
{
  dji_sdk::QueryDroneVersion query;
  query_version_client_.call(query);
  return query.response.version;
}

bool FlightControllerROS::gainControl()
{
    dji_sdk::SDKControlAuthority authority;
    authority.request.control_enable=1;
    ctrl_authority_client_.call(authority);

    return authority.response.result;
}

bool FlightControllerROS::enableLocalPosition()
{
  dji_sdk::SetLocalPosRef localPosReferenceSetter;
  enable_local_pos_client_.call(localPosReferenceSetter);
  return localPosReferenceSetter.response.result;
}

bool FlightControllerROS::takeoff()
{
  ROS_INFO_STREAM("Taking off");
  auto start_time = ros::Time::now();
  auto result = callTakeoffTask(dji_sdk::DroneTaskControl::Request::TASK_TAKEOFF);
  float home_altitude;
  {
    std::lock_guard<std::mutex> guard(gps_mutex_);
    home_altitude = latest_gps_pos_.altitude;
  }

  if(result == false)
  {
    ROS_ERROR_STREAM("Takeoff request failed.");
    return false;
  }

  bool takeoff_complete = false;

  uint8_t tmp_fs;
  uint8_t tmp_dm;
  if(drone_version_ == DJISDK::DroneFirmwareVersion::M100_31)
  {
    float latest_altitude;
    while(ros::Time::now() - start_time < ros::Duration(10))
    {
      {
        std::lock_guard<std::mutex> guard(gps_mutex_);
        latest_altitude = latest_gps_pos_.altitude;
      }
      if(latest_altitude - home_altitude >= 1.0)
      {
        return true;
      }
    }
    return false;
  }
  else
  {
    bool motor_start = false;

    // Motor Start Check
    while(ros::Time::now() - start_time < ros::Duration(5))
    {
      {
        std::lock_guard<std::mutex> guard(fs_mutex_);
        tmp_fs = flight_status_;
      }
      {
        std::lock_guard<std::mutex> guard(dm_mutex_);
        tmp_dm = display_mode_;
      }

      if(tmp_fs != DJISDK::FlightStatus::STATUS_ON_GROUND &&
         tmp_dm != DJISDK::DisplayMode::MODE_ENGINE_START)
      {
        ;
      }
      else
      {
        ROS_INFO_STREAM("Motor Spinning ...");
        motor_start = true;
        break;
      }
    }

    if(motor_start == false)
    {
      ROS_ERROR_STREAM("Takeoff failed. Motors are not spinnning.");
      return false;
    }

    // In air check
    bool in_air = false;
    start_time = ros::Time::now();
    while(ros::Time::now() - start_time > ros::Duration(20))
    {
      {
        std::lock_guard<std::mutex> guard(fs_mutex_);
        tmp_fs = flight_status_;
      }
      {
        std::lock_guard<std::mutex> guard(dm_mutex_);
        tmp_dm = display_mode_;
      }

      if(tmp_fs != DJISDK::FlightStatus::STATUS_IN_AIR &&
         (tmp_dm != DJISDK::DisplayMode::MODE_ASSISTED_TAKEOFF ||
          tmp_dm != DJISDK::DisplayMode::MODE_AUTO_TAKEOFF))
      {
        ;
      }
      else
      {
        ROS_INFO_STREAM("Ascending ...");
        in_air = true;
        break;
      }
    }

    if(in_air == false)
    {
      ROS_ERROR_STREAM("Takeoff failed. Aircraft is still on the ground, but the motors are spinning.");
      return false;
    }

    // Takeoff complete check
    start_time = ros::Time::now();
    while(ros::Time::now() - start_time > ros::Duration(20))
    {
      {
        std::lock_guard<std::mutex> guard(fs_mutex_);
        tmp_fs = flight_status_;
      }
      {
        std::lock_guard<std::mutex> guard(dm_mutex_);
        tmp_dm = display_mode_;
      }

      if(tmp_dm == DJISDK::DisplayMode::MODE_ASSISTED_TAKEOFF ||
         tmp_dm == DJISDK::DisplayMode::MODE_AUTO_TAKEOFF)
      {
        ;
      }
      else
      {
        ROS_INFO_STREAM("Motor Spinning ...");
        takeoff_complete = true;
        break;
      }
    }

    if(takeoff_complete == true)
    {
      if(tmp_dm != DJISDK::DisplayMode::MODE_P_GPS ||
         tmp_dm != DJISDK::DisplayMode::MODE_ATTITUDE)
      {
        ROS_INFO("Successful takeoff!");
        return true;
      }
      else
      {
        ROS_ERROR_STREAM("Takeoff finished, but the aircraft is in an unexpected mode. Please connect DJI GO.");
        return false;
      }
    }
    return false;
  }
}

bool FlightControllerROS::callTakeoffTask(int task)
{
  dji_sdk::DroneTaskControl drone_task;
  drone_task.request.task = task;
  dispatch_task_client_.call(drone_task);

  return drone_task.response.result;
}

bool FlightControllerROS::land()
{
  ;
}

void FlightControllerROS::attitudeCallback(const geometry_msgs::QuaternionStamped::ConstPtr& msg)
{
  std::lock_guard<std::mutex> guard(attitude_mutex_);
  ROS_INFO_STREAM("Attitude callback");
  attitude_ = msg->quaternion;
}

void FlightControllerROS::gpsCallback(const sensor_msgs::NavSatFix::ConstPtr& msg)
{
  std::lock_guard<std::mutex> guard(gps_mutex_);
  ROS_INFO_STREAM("GPS callback");
  latest_gps_pos_ = *msg;
}

void FlightControllerROS::flightStatusCallback(const std_msgs::UInt8::ConstPtr& msg)
{
  std::lock_guard<std::mutex> guard(fs_mutex_);
  ROS_INFO_STREAM("Flight Status Callback");
  flight_status_ = msg->data;
}

void FlightControllerROS::displayModeCallback(const std_msgs::UInt8::ConstPtr& msg)
{
  std::lock_guard<std::mutex> guard(dm_mutex_);
  ROS_INFO_STREAM("Display Mode Callback");
  display_mode_ = msg->data;
}

void FlightControllerROS::localPositionCallback(const geometry_msgs::PointStamped::ConstPtr& msg)
{
  std::lock_guard<std::mutex> guard(llp_mutex_);
  ROS_INFO_STREAM("Local Position Callback");
  latest_local_pos_ = msg->point;
}


//void Mission::step(sensor_msgs::NavSatFix current_gps, geometry_msgs::Quaternion current_atti)
//{
//  static int info_counter = 0;
//  geometry_msgs::Vector3     localOffset;
//
//  float speedFactor         = 2;
//  float yawThresholdInDeg   = 2;
//
//  float xCmd, yCmd, zCmd;
//
//  localOffsetFromGpsOffset(localOffset, current_gps, start_gps_location);
//
//  double xOffsetRemaining = target_offset_x - localOffset.x;
//  double yOffsetRemaining = target_offset_y - localOffset.y;
//  double zOffsetRemaining = target_offset_z - localOffset.z;
//
//  double yawDesiredRad     = deg2rad * target_yaw;
//  double yawThresholdInRad = deg2rad * yawThresholdInDeg;
//  double yawInRad          = toEulerAngle(current_atti).z;
//
//  info_counter++;
//  if(info_counter > 25)
//  {
//    info_counter = 0;
//    ROS_INFO("-----x=%f, y=%f, z=%f, yaw=%f ...", localOffset.x,localOffset.y, localOffset.z,yawInRad);
//    ROS_INFO("+++++dx=%f, dy=%f, dz=%f, dyaw=%f ...", xOffsetRemaining,yOffsetRemaining, zOffsetRemaining,yawInRad - yawDesiredRad);
//  }
//  if (abs(xOffsetRemaining) >= speedFactor)
//    xCmd = (xOffsetRemaining>0) ? speedFactor : -1 * speedFactor;
//  else
//    xCmd = xOffsetRemaining;
//
//  if (abs(yOffsetRemaining) >= speedFactor)
//    yCmd = (yOffsetRemaining>0) ? speedFactor : -1 * speedFactor;
//  else
//    yCmd = yOffsetRemaining;
//
//  zCmd = start_local_position.z + target_offset_z;
//
//
//  /*!
//   * @brief: if we already started breaking, keep break for 50 sample (1sec)
//   *         and call it done, else we send normal command
//   */
//
//  if (break_counter > 50)
//  {
//    ROS_INFO("##### Route %d finished....", state);
//    finished = true;
//    return;
//  }
//  else if(break_counter > 0)
//  {
//    sensor_msgs::Joy controlVelYawRate;
//    uint8_t flag = (DJISDK::VERTICAL_VELOCITY   |
//                    DJISDK::HORIZONTAL_VELOCITY |
//                    DJISDK::YAW_RATE            |
//                    DJISDK::HORIZONTAL_GROUND   |
//                    DJISDK::STABLE_ENABLE);
//    controlVelYawRate.axes.push_back(0);
//    controlVelYawRate.axes.push_back(0);
//    controlVelYawRate.axes.push_back(0);
//    controlVelYawRate.axes.push_back(0);
//    controlVelYawRate.axes.push_back(flag);
//
//    ctrlBrakePub.publish(controlVelYawRate);
//    break_counter++;
//    return;
//  }
//  else //break_counter = 0, not in break stage
//  {
//    sensor_msgs::Joy controlPosYaw;
//
//
//    controlPosYaw.axes.push_back(xCmd);
//    controlPosYaw.axes.push_back(yCmd);
//    controlPosYaw.axes.push_back(zCmd);
//    controlPosYaw.axes.push_back(yawDesiredRad);
//    ctrlPosYawPub.publish(controlPosYaw);
//  }
//
//  if (std::abs(xOffsetRemaining) < 0.5 &&
//      std::abs(yOffsetRemaining) < 0.5 &&
//      std::abs(zOffsetRemaining) < 0.5 &&
//      std::abs(yawInRad - yawDesiredRad) < yawThresholdInRad)
//  {
//    //! 1. We are within bounds; start incrementing our in-bound counter
//    inbound_counter ++;
//  }
//  else
//  {
//    if (inbound_counter != 0)
//    {
//      //! 2. Start incrementing an out-of-bounds counter
//      outbound_counter ++;
//    }
//  }
//
//  //! 3. Reset withinBoundsCounter if necessary
//  if (outbound_counter > 10)
//  {
//    ROS_INFO("##### Route %d: out of bounds, reset....", state);
//    inbound_counter  = 0;
//    outbound_counter = 0;
//  }
//
//  if (inbound_counter > 50)
//  {
//    ROS_INFO("##### Route %d start break....", state);
//    break_counter = 1;
//  }
//
//}
//
//void Mission::localOffsetFromGpsOffset(geometry_msgs::Vector3&  deltaNed,
//                                       sensor_msgs::NavSatFix& target,
//                                       sensor_msgs::NavSatFix& origin)
//{
//  double deltaLon = target.longitude - origin.longitude;
//  double deltaLat = target.latitude - origin.latitude;
//
//  deltaNed.y = deltaLat * deg2rad * C_EARTH;
//  deltaNed.x = deltaLon * deg2rad * C_EARTH * cos(deg2rad*target.latitude);
//  deltaNed.z = target.altitude - origin.altitude;
//}