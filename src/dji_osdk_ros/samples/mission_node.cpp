/** @file mission_node.cpp
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

#include <dji_osdk_ros/mission_node.h>
#include <dji_osdk_ros/FlightTaskControl.h>
#include <dji_osdk_ros/ObtainControlAuthority.h>

using namespace DJI::OSDK;

// global variables
ros::ServiceClient     obtain_ctrl_authority_client;
ros::ServiceClient     waypoint_upload_client;
ros::ServiceClient     waypoint_action_client;
ros::ServiceClient     hotpoint_upload_client;
ros::ServiceClient     hotpoint_action_client;
ros::ServiceClient     hotpoint_update_yawRate_client;
ros::ServiceClient     hotpoint_updateRadius_client;
ros::ServiceClient     flight_control_client;
sensor_msgs::NavSatFix gps_pos;
ros::Subscriber        gps_pos_subscriber;

void gpsPosCallback(const sensor_msgs::NavSatFix::ConstPtr& msg)
{
  gps_pos = *msg;
}

bool runWaypointMission(uint8_t numWaypoints, int responseTimeout)
{
  ros::spinOnce();

  // Waypoint Mission : Initialization
  dji_osdk_ros::MissionWaypointTask waypointTask;
  setWaypointInitDefaults(waypointTask);

  // Waypoint Mission: Create Waypoints
  float64_t increment = 0.000001 / C_PI * 180;
  float32_t start_alt = 10;
  ROS_INFO("Creating Waypoints..\n");
  std::vector<WayPointSettings> generatedWaypts =
    createWaypoints(numWaypoints, increment, start_alt);

  // Waypoint Mission: Upload the waypoints
  ROS_INFO("Uploading Waypoints..\n");
  uploadWaypoints(generatedWaypts, responseTimeout, waypointTask);

  // Waypoint Mission: Init mission
  ROS_INFO("Initializing Waypoint Mission..\n");
  if (initWaypointMission(waypointTask).result)
  {
    ROS_INFO("Waypoint upload command sent successfully");
  }
  else
  {
    ROS_WARN("Failed sending waypoint upload command");
    return false;
  }

  // Waypoint Mission: Start
  if (missionAction(DJI_MISSION_TYPE::WAYPOINT,
                    MISSION_ACTION::START)
        .result)
  {
    ROS_INFO("Mission start command sent successfully");
  }
  else
  {
    ROS_WARN("Failed sending mission start command");
    return false;
  }

  return true;
}

void setWaypointDefaults(WayPointSettings* wp)
{
  wp->damping         = 0;
  wp->yaw             = 0;
  wp->gimbalPitch     = 0;
  wp->turnMode        = 0;
  wp->hasAction       = 0;
  wp->actionTimeLimit = 100;
  wp->actionNumber    = 0;
  wp->actionRepeat    = 0;
  for (int i = 0; i < 16; ++i)
  {
    wp->commandList[i]      = 0;
    wp->commandParameter[i] = 0;
  }
}

void setWaypointInitDefaults(dji_osdk_ros::MissionWaypointTask& waypointTask)
{
  waypointTask.velocity_range     = 10;
  waypointTask.idle_velocity      = 5;
  waypointTask.action_on_finish   = dji_osdk_ros::MissionWaypointTask::FINISH_NO_ACTION;
  waypointTask.mission_exec_times = 1;
  waypointTask.yaw_mode           = dji_osdk_ros::MissionWaypointTask::YAW_MODE_AUTO;
  waypointTask.trace_mode         = dji_osdk_ros::MissionWaypointTask::TRACE_POINT;
  waypointTask.action_on_rc_lost  = dji_osdk_ros::MissionWaypointTask::ACTION_AUTO;
  waypointTask.gimbal_pitch_mode  = dji_osdk_ros::MissionWaypointTask::GIMBAL_PITCH_FREE;
}

std::vector<DJI::OSDK::WayPointSettings>
createWaypoints(int numWaypoints, float64_t distanceIncrement,
                float32_t start_alt)
{
  // Create Start Waypoint
  WayPointSettings start_wp;
  setWaypointDefaults(&start_wp);
  start_wp.latitude  = gps_pos.latitude;
  start_wp.longitude = gps_pos.longitude;
  start_wp.altitude  = start_alt;
  ROS_INFO("Waypoint created at (LLA): %f \t%f \t%f\n", gps_pos.latitude,
           gps_pos.longitude, start_alt);

  std::vector<DJI::OSDK::WayPointSettings> wpVector =
    generateWaypointsPolygon(&start_wp, distanceIncrement, numWaypoints);
  return wpVector;
}

std::vector<DJI::OSDK::WayPointSettings>
generateWaypointsPolygon(WayPointSettings* start_data, float64_t increment,
                         int num_wp)
{
  // Let's create a vector to store our waypoints in.
  std::vector<DJI::OSDK::WayPointSettings> wp_list;

  // Some calculation for the polygon
  float64_t extAngle = 2 * M_PI / num_wp;

  // First waypoint
  start_data->index = 0;
  wp_list.push_back(*start_data);

  // Iterative algorithm
  for (int i = 1; i < num_wp; i++)
  {
    WayPointSettings  wp;
    WayPointSettings* prevWp = &wp_list[i - 1];
    setWaypointDefaults(&wp);
    wp.index     = i;
    wp.latitude  = (prevWp->latitude + (increment * cos(i * extAngle)));
    wp.longitude = (prevWp->longitude + (increment * sin(i * extAngle)));
    wp.altitude  = (prevWp->altitude + 1);
    wp_list.push_back(wp);
  }

  // Come back home
  start_data->index = num_wp;
  wp_list.push_back(*start_data);

  return wp_list;
}

void uploadWaypoints(std::vector<DJI::OSDK::WayPointSettings>& wp_list,
                     int responseTimeout,
                     dji_osdk_ros::MissionWaypointTask& waypointTask)
{
  dji_osdk_ros::MissionWaypoint waypoint;
  for (std::vector<WayPointSettings>::iterator wp = wp_list.begin();
       wp != wp_list.end(); ++wp)
  {
    ROS_INFO("Waypoint created at (LLA): %f \t%f \t%f\n ", wp->latitude,
             wp->longitude, wp->altitude);
    waypoint.latitude            = wp->latitude;
    waypoint.longitude           = wp->longitude;
    waypoint.altitude            = wp->altitude;
    waypoint.damping_distance    = 0;
    waypoint.target_yaw          = 0;
    waypoint.target_gimbal_pitch = 0;
    waypoint.turn_mode           = 0;
    waypoint.has_action          = 0;
    waypointTask.mission_waypoint.push_back(waypoint);
  }
}

bool runHotpointMission(int initialRadius, int responseTimeout)
{
  ros::spinOnce();

  // Hotpoint Mission: Create hotpoint
  dji_osdk_ros::MissionHotpointTask hotpointTask;
  setHotpointInitDefault(hotpointTask);

  // Hotpoint Mission: Initialize
  initHotpointMission(hotpointTask);

  // Takeoff
  if (takeoff())
  {
    ROS_INFO("Takeoff command sent successfully");
  }
  else
  {
    ROS_WARN("Failed sending takeoff command");
    return false;
  }
  ros::Duration(15).sleep();

  // Start
  ROS_INFO("Start with default rotation rate: 15 deg/s");
  if (missionAction(DJI::OSDK::DJI_MISSION_TYPE::HOTPOINT,
                    DJI::OSDK::MISSION_ACTION::START).result)
  {
    ROS_INFO("Mission start command sent successfully");
  }
  else
  {
    ROS_WARN("Failed sending mission start command");
    return false;
  }
  ros::Duration(20).sleep();

  // Pause
  ROS_INFO("Pause for 5s");
  if (missionAction(DJI::OSDK::DJI_MISSION_TYPE::HOTPOINT,
                    DJI::OSDK::MISSION_ACTION::PAUSE).result)
  {
    ROS_INFO("Mission pause command sent successfully");
  }
  else
  {
    ROS_WARN("Failed sending mission pause command");
    return false;
  }
  ros::Duration(5).sleep();

  // Resume
  ROS_INFO("Resume");
  if (missionAction(DJI::OSDK::DJI_MISSION_TYPE::HOTPOINT,
                    DJI::OSDK::MISSION_ACTION::RESUME).result)
  {
    ROS_INFO("Mission resume command sent successfully");
  }
  else
  {
    ROS_WARN("Failed sending mission resume command");
    return false;
  }
  ros::Duration(10).sleep();

  // Update radius, no ACK
  ROS_INFO("Update radius to 1.5x: new radius = %f", 1.5 * initialRadius);
  if (hotpointUpdateRadius(1.5 * initialRadius).result)
  {
    ROS_INFO("Hotpoint update radius command sent successfully");
  }
  else
  {
    ROS_WARN("Failed sending hotpoint update radius command");
    return false;
  }
  ros::Duration(10).sleep();

  // Update velocity (yawRate), no ACK
  ROS_INFO("Update hotpoint rotation rate: new rate = 5 deg/s");
  if (hotpointUpdateYawRate(5, 1).result)
  {
    ROS_INFO("Hotpoint update yaw rate command sent successfully");
  }
  else
  {
    ROS_WARN("Failed sending hotpoint update yaw rate command");
    return false;
  }
  ros::Duration(10).sleep();

  // Stop
  ROS_INFO("Stop");
  if (missionAction(DJI::OSDK::DJI_MISSION_TYPE::HOTPOINT,
                    DJI::OSDK::MISSION_ACTION::STOP)
        .result)
  {
    ROS_INFO("Mission stop command sent successfully");
  }
  else
  {
    ROS_WARN("Failed sending mission stop command");
    return false;
  }

  ROS_INFO("land");
  if (land())
  {
    ROS_INFO("Land command sent successfully");
  }
  else
  {
    ROS_WARN("Failed sending land command");
    return false;
  }

  return true;
}

void setHotpointInitDefault(dji_osdk_ros::MissionHotpointTask& hotpointTask)
{
  hotpointTask.latitude      = gps_pos.latitude;
  hotpointTask.longitude     = gps_pos.longitude;
  hotpointTask.altitude      = 20;
  hotpointTask.radius        = 10;
  hotpointTask.angular_speed = 15;
  hotpointTask.is_clockwise  = 1;
  hotpointTask.start_point   = 0;
  hotpointTask.yaw_mode      = 1;
}

ServiceAck initWaypointMission(dji_osdk_ros::MissionWaypointTask& waypointTask)
{
  dji_osdk_ros::MissionWpUpload missionWpUpload;
  missionWpUpload.request.waypoint_task = waypointTask;
  waypoint_upload_client.call(missionWpUpload);
  if (!missionWpUpload.response.result)
  {
    ROS_WARN("ack.info: set = %i id = %i", missionWpUpload.response.cmd_set,
             missionWpUpload.response.cmd_id);
    ROS_WARN("ack.data: %i", missionWpUpload.response.ack_data);
  }
  return ServiceAck(
    missionWpUpload.response.result, missionWpUpload.response.cmd_set,
    missionWpUpload.response.cmd_id, missionWpUpload.response.ack_data);
}

ServiceAck initHotpointMission(dji_osdk_ros::MissionHotpointTask& hotpointTask)
{
  dji_osdk_ros::MissionHpUpload missionHpUpload;
  missionHpUpload.request.hotpoint_task = hotpointTask;
  hotpoint_upload_client.call(missionHpUpload);
  return ServiceAck(
    missionHpUpload.response.result, missionHpUpload.response.cmd_set,
    missionHpUpload.response.cmd_id, missionHpUpload.response.ack_data);
}

ServiceAck missionAction(DJI::OSDK::DJI_MISSION_TYPE type,
              DJI::OSDK::MISSION_ACTION   action)
{
  dji_osdk_ros::MissionWpAction missionWpAction;
  dji_osdk_ros::MissionHpAction missionHpAction;
  switch (type)
  {
    case DJI::OSDK::WAYPOINT:
      missionWpAction.request.action = action;
      waypoint_action_client.call(missionWpAction);
      if (!missionWpAction.response.result)
      {
        ROS_WARN("ack.info: set = %i id = %i", missionWpAction.response.cmd_set,
                 missionWpAction.response.cmd_id);
        ROS_WARN("ack.data: %i", missionWpAction.response.ack_data);
      }
      return { missionWpAction.response.result,
               missionWpAction.response.cmd_set,
               missionWpAction.response.cmd_id,
               missionWpAction.response.ack_data };
    case DJI::OSDK::HOTPOINT:
      missionHpAction.request.action = action;
      hotpoint_action_client.call(missionHpAction);
      if (!missionHpAction.response.result)
      {
        ROS_WARN("ack.info: set = %i id = %i", missionHpAction.response.cmd_set,
                 missionHpAction.response.cmd_id);
        ROS_WARN("ack.data: %i", missionHpAction.response.ack_data);
      }
      return ServiceAck(
        missionHpAction.response.result, missionHpAction.response.cmd_set,
        missionHpAction.response.cmd_id, missionHpAction.response.ack_data);
  }
}

bool takeoff()
{
  dji_osdk_ros::FlightTaskControl flightTaskControl;
  flightTaskControl.request.task = dji_osdk_ros::FlightTaskControl::Request ::TASK_TAKEOFF;
  flight_control_client.call(flightTaskControl);

  return flightTaskControl.response.result;
}

bool land()
{
  dji_osdk_ros::FlightTaskControl flightTaskControl;
  flightTaskControl.request.task = dji_osdk_ros::FlightTaskControl::Request ::TASK_LAND;
  flight_control_client.call(flightTaskControl);

  return flightTaskControl.response.result;
}

ServiceAck hotpointUpdateRadius(float radius)
{
  dji_osdk_ros::MissionHpUpdateRadius missionHpUpdateRadius;
  missionHpUpdateRadius.request.radius = radius;
  hotpoint_updateRadius_client.call(missionHpUpdateRadius);
  if (!missionHpUpdateRadius.response.result)
  {
    ROS_WARN("ack.info: set = %i id = %i",
             missionHpUpdateRadius.response.cmd_set,
             missionHpUpdateRadius.response.cmd_id);
    ROS_WARN("ack.data: %i", missionHpUpdateRadius.response.ack_data);
  }
  return ServiceAck(missionHpUpdateRadius.response.result,
                    missionHpUpdateRadius.response.cmd_set,
                    missionHpUpdateRadius.response.cmd_id,
                    missionHpUpdateRadius.response.ack_data);
}

ServiceAck hotpointUpdateYawRate(float yawRate, int direction)
{
  dji_osdk_ros::MissionHpUpdateYawRate missionHpUpdateYawRate;
  missionHpUpdateYawRate.request.yaw_rate  = yawRate;
  missionHpUpdateYawRate.request.direction = direction;
  hotpoint_update_yawRate_client.call(missionHpUpdateYawRate);
  if (!missionHpUpdateYawRate.response.result)
  {
    ROS_WARN("ack.info: set = %i id = %i",
             missionHpUpdateYawRate.response.cmd_set,
             missionHpUpdateYawRate.response.cmd_id);
    ROS_WARN("ack.data: %i", missionHpUpdateYawRate.response.ack_data);
  }
  return ServiceAck(missionHpUpdateYawRate.response.result,
                    missionHpUpdateYawRate.response.cmd_set,
                    missionHpUpdateYawRate.response.cmd_id,
                    missionHpUpdateYawRate.response.ack_data);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "mission_node");
  ros::NodeHandle nh;

  // ROS stuff
  obtain_ctrl_authority_client = nh.serviceClient<dji_osdk_ros::ObtainControlAuthority>(
    "obtain_release_control_authority");
  waypoint_upload_client = nh.serviceClient<dji_osdk_ros::MissionWpUpload>(
    "dji_osdk_ros/mission_waypoint_upload");
  waypoint_action_client = nh.serviceClient<dji_osdk_ros::MissionWpAction>(
    "dji_osdk_ros/mission_waypoint_action");
  hotpoint_upload_client = nh.serviceClient<dji_osdk_ros::MissionHpUpload>(
    "dji_osdk_ros/mission_hotpoint_upload");
  hotpoint_action_client = nh.serviceClient<dji_osdk_ros::MissionHpAction>(
    "dji_osdk_ros/mission_hotpoint_action");
  hotpoint_updateRadius_client =
    nh.serviceClient<dji_osdk_ros::MissionHpUpdateRadius>(
      "dji_osdk_ros/mission_hotpoint_updateRadius");
  hotpoint_update_yawRate_client =
    nh.serviceClient<dji_osdk_ros::MissionHpUpdateYawRate>(
      "dji_osdk_ros/mission_hotpoint_updateYawRate");
  flight_control_client =
    nh.serviceClient<dji_osdk_ros::FlightTaskControl>("flight_task_control");
  gps_pos_subscriber = nh.subscribe<sensor_msgs::NavSatFix>(
    "dji_osdk_ros/gps_position", 10, &gpsPosCallback);

  // Setup variables for use
  dji_osdk_ros::ObtainControlAuthority obtainCtrlAuthority;
  obtainCtrlAuthority.request.enable_obtain = true;
  uint8_t wayptPolygonSides;
  int     hotptInitRadius;
  int     responseTimeout = 1;

  obtain_ctrl_authority_client.call(obtainCtrlAuthority);

  // Display interactive prompt
  std::cout
    << "| Available commands:                                            |"
    << std::endl;
  std::cout
    << "| [a] Waypoint Mission(just for M210V2)                          |"
    << std::endl;
  std::cout
    << "| [b] Hotpoint Mission                                           |"
    << std::endl;
  char inputChar;
  std::cin >> inputChar;
  switch (inputChar)
  {
    case 'a':
      // Waypoint call
      wayptPolygonSides = 6;
      runWaypointMission(wayptPolygonSides, responseTimeout);
      break;
    case 'b':
      // Hotpoint call
      hotptInitRadius = 10;
      runHotpointMission(hotptInitRadius, responseTimeout);
      break;
    default:
      break;
  }

  ros::spin();

  return 0;
}
