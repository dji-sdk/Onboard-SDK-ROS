/** @file demo_mission.h
 *  @version 3.3
 *  @date May, 2017
 *
 *  @brief
 *  demo sample of how to use mission APIs
 *
 *  @copyright 2017 DJI. All rights reserved.
 *
 */

#ifndef DEMO_MISSION_H
#define DEMO_MISSION_H

// System includes
#include "unistd.h"
#include <iostream>

// DJI SDK includes
#include <dji_sdk/Activation.h>
#include <dji_sdk/DroneTaskControl.h>
#include <dji_sdk/SDKControlAuthority.h>
#include <dji_sdk/MissionWpAction.h>
#include <dji_sdk/MissionHpAction.h>
#include <dji_sdk/MissionWpUpload.h>
#include <dji_sdk/MissionHpUpload.h>
#include <dji_sdk/MissionHpUpdateRadius.h>
#include <dji_sdk/MissionHpUpdateYawRate.h>

// SDK core library
#include <djiosdk/dji_vehicle.hpp>

// ROS includes
#include <ros/ros.h>
#include <sensor_msgs/NavSatFix.h>

#define C_EARTH (double)6378137.0
#define C_PI (double)3.141592653589793
#define DEG2RAD(DEG) ((DEG) * ((C_PI) / (180.0)))

typedef struct ServiceAck
{
  bool         result;
  int          cmd_set;
  int          cmd_id;
  unsigned int ack_data;
  ServiceAck(bool res, int set, int id, unsigned int ack)
    : result(res)
    , cmd_set(set)
    , cmd_id(id)
    , ack_data(ack)
  {
  }
  ServiceAck()
  {
  }
} ServiceAck;

bool runWaypointMission(uint8_t numWaypoints, int responseTimeout);

void setWaypointDefaults(DJI::OSDK::WayPointSettings* wp);

void setWaypointInitDefaults(dji_sdk::MissionWaypointTask& waypointTask);

std::vector<DJI::OSDK::WayPointSettings> createWaypoints(
  int numWaypoints, DJI::OSDK::float64_t distanceIncrement,
  DJI::OSDK::float32_t start_alt);

std::vector<DJI::OSDK::WayPointSettings> generateWaypointsPolygon(
  DJI::OSDK::WayPointSettings* start_data, DJI::OSDK::float64_t increment,
  int num_wp);

void uploadWaypoints(std::vector<DJI::OSDK::WayPointSettings>& wp_list,
                     int                                       responseTimeout,
                     dji_sdk::MissionWaypointTask&             waypointTask);

bool runHotpointMission(int initialRadius, int responseTimeout);

void setHotpointInitDefault(dji_sdk::MissionHotpointTask& hotpointTask);

ServiceAck initWaypointMission(dji_sdk::MissionWaypointTask& waypointTask);

ServiceAck initHotpointMission(dji_sdk::MissionHotpointTask& hotpointTask);

ServiceAck missionAction(DJI::OSDK::DJI_MISSION_TYPE type,
                         DJI::OSDK::MISSION_ACTION   action);

ServiceAck activate();

ServiceAck obtainCtrlAuthority();

ServiceAck takeoff();

ServiceAck land();

ServiceAck hotpointUpdateRadius(float radius);

ServiceAck hotpointUpdateYawRate(float yawRate, int direction);

void gpsPosCallback(const sensor_msgs::NavSatFix::ConstPtr& msg);

#endif // DEMO_MISSION_H
