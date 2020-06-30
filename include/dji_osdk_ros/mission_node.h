/** @file mission_node.h
 *  @version 4.0
 *  @date June 2020
 *
 *  @brief node of hotpoint 1.0/waypoint 1.0.
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

#ifndef MISSION_NODE_H
#define MISSION_NODE_H

// System includes
#include "unistd.h"
#include <iostream>

// DJI SDK includes
#include <dji_osdk_ros/MissionWpAction.h>
#include <dji_osdk_ros/MissionHpAction.h>
#include <dji_osdk_ros/MissionWpUpload.h>
#include <dji_osdk_ros/MissionHpUpload.h>
#include <dji_osdk_ros/MissionHpUpdateRadius.h>
#include <dji_osdk_ros/MissionHpUpdateYawRate.h>

#include <dji_osdk_ros/dji_vehicle_node.h>

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

void setWaypointInitDefaults(dji_osdk_ros::MissionWaypointTask& waypointTask);

std::vector<DJI::OSDK::WayPointSettings> createWaypoints(
  int numWaypoints, DJI::OSDK::float64_t distanceIncrement,
  DJI::OSDK::float32_t start_alt);

std::vector<DJI::OSDK::WayPointSettings> generateWaypointsPolygon(
  DJI::OSDK::WayPointSettings* start_data, DJI::OSDK::float64_t increment,
  int num_wp);

void uploadWaypoints(std::vector<DJI::OSDK::WayPointSettings>& wp_list,
                     int                                       responseTimeout,
                     dji_osdk_ros::MissionWaypointTask&        waypointTask);

bool runHotpointMission(int initialRadius, int responseTimeout);

void setHotpointInitDefault(dji_osdk_ros::MissionHotpointTask& hotpointTask);

ServiceAck initWaypointMission(dji_osdk_ros::MissionWaypointTask& waypointTask);

ServiceAck initHotpointMission(dji_osdk_ros::MissionHotpointTask& hotpointTask);

ServiceAck missionAction(DJI::OSDK::DJI_MISSION_TYPE type,
                         DJI::OSDK::MISSION_ACTION   action);

ServiceAck activate();

ServiceAck obtainCtrlAuthority();

bool takeoff();

bool land();

ServiceAck hotpointUpdateRadius(float radius);

ServiceAck hotpointUpdateYawRate(float yawRate, int direction);

void gpsPosCallback(const sensor_msgs::NavSatFix::ConstPtr& msg);

#endif // MISSION_NODE_H