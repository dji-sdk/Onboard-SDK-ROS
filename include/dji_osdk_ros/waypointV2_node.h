/** @file mission_node.cpp
 *  @version 4.0
 *  @date July 2020
 *
 *  @brief node of waypoint V2.0.
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

#ifndef WAYPOINTV2_NODE_H
#define WAYPOINTV2_NODE_H

#include <dji_osdk_ros/dji_vehicle_node.h>

// ROS includes
#include <ros/ros.h>

dji_osdk_ros::GetDroneType drone_type;
dji_osdk_ros::InitWaypointV2Setting initWaypointV2Setting_;
dji_osdk_ros::UploadWaypointV2Mission uploadWaypointV2Mission_;
dji_osdk_ros::UploadWaypointV2Action uploadWaypointV2Action_;
dji_osdk_ros::DownloadWaypointV2Mission downloadWaypointV2Mission_;
dji_osdk_ros::StartWaypointV2Mission startWaypointV2Mission_;
dji_osdk_ros::StopWaypointV2Mission stopWaypointV2Mission_;
dji_osdk_ros::PauseWaypointV2Mission pauseWaypointV2Mission_;
dji_osdk_ros::ResumeWaypointV2Mission resumeWaypointV2Mission_;
dji_osdk_ros::SetGlobalCruisespeed setGlobalCruisespeed_;
dji_osdk_ros::GetGlobalCruisespeed getGlobalCruisespeed_;
dji_osdk_ros::GenerateWaypointV2Action generateWaypointV2Action_;
dji_osdk_ros::SubscribeWaypointV2Event subscribeWaypointV2Event_;
dji_osdk_ros::SubscribeWaypointV2State subscribeWaypointV2State_;

ros::ServiceClient waypointV2_init_setting_client;
ros::ServiceClient waypointV2_upload_mission_client;
ros::ServiceClient waypointV2_upload_action_client;
ros::ServiceClient waypointV2_download_mission_client;
ros::ServiceClient waypointV2_start_mission_client;
ros::ServiceClient waypointV2_stop_mission_client;
ros::ServiceClient waypointV2_pause_mission_client;
ros::ServiceClient waypointV2_resume_mission_client;
ros::ServiceClient waypointV2_set_global_cruisespeed_client;
ros::ServiceClient waypointV2_get_global_cruisespeed_client;
ros::ServiceClient waypointV2_generate_actions_client;
ros::ServiceClient waypointV2_mission_event_push_client;
ros::ServiceClient waypointV2_mission_state_push_client;

ros::Subscriber waypointV2EventSub;
ros::Subscriber waypointV2StateSub;

ros::ServiceClient get_drone_type_client;
sensor_msgs::NavSatFix gps_position_;
dji_osdk_ros::WaypointV2MissionEventPush waypoint_V2_mission_event_push_;
dji_osdk_ros::WaypointV2MissionStatePush waypoint_V2_mission_state_push_;

void gpsPositionSubCallback(const sensor_msgs::NavSatFix::ConstPtr& gpsPosition);
void waypointV2MissionStateSubCallback(const dji_osdk_ros::WaypointV2MissionStatePush::ConstPtr& waypointV2MissionStatePush);
void waypointV2MissionEventSubCallback(const dji_osdk_ros::WaypointV2MissionEventPush::ConstPtr& waypointV2MissionEventPush);

void setWaypointV2Defaults(dji_osdk_ros::WaypointV2& waypointV2);
std::vector<dji_osdk_ros::WaypointV2> generatePolygonWaypoints(const ros::NodeHandle &nh, float32_t radius, uint16_t polygonNum);
bool initWaypointV2Setting(ros::NodeHandle &nh);
bool uploadWaypointV2Mission(ros::NodeHandle &nh);
bool uploadWaypointV2Action(ros::NodeHandle &nh);
bool downloadWaypointV2Mission(ros::NodeHandle &nh, std::vector<dji_osdk_ros::WaypointV2> &mission);
bool startWaypointV2Mission(ros::NodeHandle &nh);
bool stopWaypointV2Mission(ros::NodeHandle &nh);
bool pauseWaypointV2Mission(ros::NodeHandle &nh);
bool resumeWaypointV2Mission(ros::NodeHandle &nh);
bool generateWaypointV2Actions(ros::NodeHandle &nh, uint16_t actionNum);
bool setGlobalCruiseSpeed(ros::NodeHandle &nh, float32_t cruiseSpeed);
float32_t getGlobalCruiseSpeed(ros::NodeHandle &nh);

bool runWaypointV2Mission(ros::NodeHandle &nh);


#endif // WAYPOINTV2_NODE_H