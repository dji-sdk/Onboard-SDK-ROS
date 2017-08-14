/** @file demo_mission.cpp
 *  @version 3.3
 *  @date May, 2017
 *
 *  @brief
 *  demo sample of how to use mission APIs
 *
 *  @copyright 2017 DJI. All rights reserved.
 *
 */

#include <dji_sdk_demo/demo_mission.h>

using namespace DJI::OSDK;

// global variables
ros::ServiceClient     waypoint_upload_service;
ros::ServiceClient     waypoint_action_service;
ros::ServiceClient     hotpoint_upload_service;
ros::ServiceClient     hotpoint_action_service;
ros::ServiceClient     hotpoint_update_yawRate_Service;
ros::ServiceClient     hotpoint_updateRadius_service;
ros::ServiceClient     drone_activation_service;
ros::ServiceClient     sdk_ctrl_authority_service;
ros::ServiceClient     drone_task_service;
sensor_msgs::NavSatFix gps_pos;
ros::Subscriber        gps_pos_subscriber;

void
gpsPosCallback(const sensor_msgs::NavSatFix::ConstPtr& msg)
{
  gps_pos = *msg;
}

bool
runWaypointMission(uint8_t numWaypoints, int responseTimeout)
{
  ros::spinOnce();

  // Waypoint Mission : Initialization
  dji_sdk::MissionWaypointTask waypointTask;
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

void
setWaypointDefaults(WayPointSettings* wp)
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

void
setWaypointInitDefaults(dji_sdk::MissionWaypointTask& waypointTask)
{
  waypointTask.velocity_range     = 10;
  waypointTask.idle_velocity      = 5;
  waypointTask.action_on_finish   = dji_sdk::MissionWaypointTask::FINISH_NO_ACTION;
  waypointTask.mission_exec_times = 1;
  waypointTask.yaw_mode           = dji_sdk::MissionWaypointTask::YAW_MODE_AUTO;
  waypointTask.trace_mode         = dji_sdk::MissionWaypointTask::TRACE_POINT;
  waypointTask.action_on_rc_lost  = dji_sdk::MissionWaypointTask::ACTION_AUTO;
  waypointTask.gimbal_pitch_mode  = dji_sdk::MissionWaypointTask::GIMBAL_PITCH_FREE;
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

void
uploadWaypoints(std::vector<DJI::OSDK::WayPointSettings>& wp_list,
                int responseTimeout, dji_sdk::MissionWaypointTask& waypointTask)
{
  dji_sdk::MissionWaypoint waypoint;
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

bool
runHotpointMission(int initialRadius, int responseTimeout)
{
  ros::spinOnce();

  // Hotpoint Mission: Create hotpoint
  dji_sdk::MissionHotpointTask hotpointTask;
  setHotpointInitDefault(hotpointTask);

  // Hotpoint Mission: Initialize
  initHotpointMission(hotpointTask);

  // Takeoff
  if (takeoff().result)
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
  ros::Duration(25).sleep();

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
  if (land().result)
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

void
setHotpointInitDefault(dji_sdk::MissionHotpointTask& hotpointTask)
{
  hotpointTask.latitude      = gps_pos.latitude;
  hotpointTask.longitude     = gps_pos.longitude;
  hotpointTask.altitude      = 20;
  hotpointTask.radius        = 10;
  hotpointTask.angular_speed = 15;
  hotpointTask.is_clockwise  = 0;
  hotpointTask.start_point   = 0;
  hotpointTask.yaw_mode      = 0;
}

ServiceAck
initWaypointMission(dji_sdk::MissionWaypointTask& waypointTask)
{
  dji_sdk::MissionWpUpload missionWpUpload;
  missionWpUpload.request.waypoint_task = waypointTask;
  waypoint_upload_service.call(missionWpUpload);
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

ServiceAck
initHotpointMission(dji_sdk::MissionHotpointTask& hotpointTask)
{
  dji_sdk::MissionHpUpload missionHpUpload;
  missionHpUpload.request.hotpoint_task = hotpointTask;
  hotpoint_upload_service.call(missionHpUpload);
  return ServiceAck(
    missionHpUpload.response.result, missionHpUpload.response.cmd_set,
    missionHpUpload.response.cmd_id, missionHpUpload.response.ack_data);
}

ServiceAck
missionAction(DJI::OSDK::DJI_MISSION_TYPE type,
              DJI::OSDK::MISSION_ACTION   action)
{
  dji_sdk::MissionWpAction missionWpAction;
  dji_sdk::MissionHpAction missionHpAction;
  switch (type)
  {
    case DJI::OSDK::WAYPOINT:
      missionWpAction.request.action = action;
      waypoint_action_service.call(missionWpAction);
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
      hotpoint_action_service.call(missionHpAction);
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

ServiceAck
activate()
{
  dji_sdk::Activation activation;
  drone_activation_service.call(activation);
  if (!activation.response.result)
  {
    ROS_WARN("ack.info: set = %i id = %i", activation.response.cmd_set,
             activation.response.cmd_id);
    ROS_WARN("ack.data: %i", activation.response.ack_data);
  }
  return ServiceAck(activation.response.result, activation.response.cmd_set,
                    activation.response.cmd_id, activation.response.ack_data);
}

ServiceAck
obtainCtrlAuthority()
{
  dji_sdk::SDKControlAuthority sdkAuthority;
  sdkAuthority.request.control_enable = 1;
  sdk_ctrl_authority_service.call(sdkAuthority);
  if (!sdkAuthority.response.result)
  {
    ROS_WARN("ack.info: set = %i id = %i", sdkAuthority.response.cmd_set,
             sdkAuthority.response.cmd_id);
    ROS_WARN("ack.data: %i", sdkAuthority.response.ack_data);
  }
  return ServiceAck(sdkAuthority.response.result, sdkAuthority.response.cmd_set,
                    sdkAuthority.response.cmd_id,
                    sdkAuthority.response.ack_data);
}

ServiceAck
takeoff()
{
  dji_sdk::DroneTaskControl droneTaskControl;
  droneTaskControl.request.task = 4;
  drone_task_service.call(droneTaskControl);
  if (!droneTaskControl.response.result)
  {
    ROS_WARN("ack.info: set = %i id = %i", droneTaskControl.response.cmd_set,
             droneTaskControl.response.cmd_id);
    ROS_WARN("ack.data: %i", droneTaskControl.response.ack_data);
  }
  return ServiceAck(
    droneTaskControl.response.result, droneTaskControl.response.cmd_set,
    droneTaskControl.response.cmd_id, droneTaskControl.response.ack_data);
}

ServiceAck
land()
{
  dji_sdk::DroneTaskControl droneTaskControl;
  droneTaskControl.request.task = 6;
  drone_task_service.call(droneTaskControl);
  if (!droneTaskControl.response.result)
  {
    ROS_WARN("ack.info: set = %i id = %i", droneTaskControl.response.cmd_set,
             droneTaskControl.response.cmd_id);
    ROS_WARN("ack.data: %i", droneTaskControl.response.ack_data);
  }
  return ServiceAck(
    droneTaskControl.response.result, droneTaskControl.response.cmd_set,
    droneTaskControl.response.cmd_id, droneTaskControl.response.ack_data);
}

ServiceAck
hotpointUpdateRadius(float radius)
{
  dji_sdk::MissionHpUpdateRadius missionHpUpdateRadius;
  missionHpUpdateRadius.request.radius = radius;
  hotpoint_updateRadius_service.call(missionHpUpdateRadius);
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

ServiceAck
hotpointUpdateYawRate(float yawRate, int direction)
{
  dji_sdk::MissionHpUpdateYawRate missionHpUpdateYawRate;
  missionHpUpdateYawRate.request.yaw_rate  = yawRate;
  missionHpUpdateYawRate.request.direction = direction;
  hotpoint_update_yawRate_Service.call(missionHpUpdateYawRate);
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

int
main(int argc, char** argv)
{
  ros::init(argc, argv, "sdk_demo_mission");
  ros::NodeHandle nh;

  // ROS stuff
  waypoint_upload_service = nh.serviceClient<dji_sdk::MissionWpUpload>(
    "dji_sdk/mission_waypoint_upload");
  waypoint_action_service = nh.serviceClient<dji_sdk::MissionWpAction>(
    "dji_sdk/mission_waypoint_action");
  hotpoint_upload_service = nh.serviceClient<dji_sdk::MissionHpUpload>(
    "dji_sdk/mission_hotpoint_upload");
  hotpoint_action_service = nh.serviceClient<dji_sdk::MissionHpAction>(
    "dji_sdk/mission_hotpoint_action");
  hotpoint_updateRadius_service =
    nh.serviceClient<dji_sdk::MissionHpUpdateRadius>(
      "dji_sdk/mission_hotpoint_updateRadius");
  hotpoint_update_yawRate_Service =
    nh.serviceClient<dji_sdk::MissionHpUpdateYawRate>(
      "dji_sdk/mission_hotpoint_updateYawRate");
  drone_activation_service =
    nh.serviceClient<dji_sdk::Activation>("dji_sdk/activation");
  sdk_ctrl_authority_service = nh.serviceClient<dji_sdk::SDKControlAuthority>(
    "dji_sdk/sdk_control_authority");
  drone_task_service =
    nh.serviceClient<dji_sdk::DroneTaskControl>("dji_sdk/drone_task_control");
  gps_pos_subscriber = nh.subscribe<sensor_msgs::NavSatFix>(
    "dji_sdk/gps_position", 10, &gpsPosCallback);

  // Activate
  if (activate().result)
  {
    ROS_INFO("Activated successfully");
  }
  else
  {
    ROS_WARN("Failed activation");
    return -1;
  }

  // Obtain Control Authority
  ServiceAck ack = obtainCtrlAuthority();
  if (ack.result)
  {
    ROS_INFO("Obtain SDK control Authority successfully");
  }
  else
  {
    if (ack.ack_data == 3 && ack.cmd_set == 1 && ack.cmd_id == 0)
    {
      ROS_INFO("Obtain SDK control Authority in progess, "
               "send the cmd again");
      obtainCtrlAuthority();
    }
    else
    {
      ROS_WARN("Failed Obtain SDK control Authority");
      return -1;

    }
  }

  // Setup variables for use
  uint8_t wayptPolygonSides;
  int     hotptInitRadius;
  int     responseTimeout = 1;

  // Display interactive prompt
  std::cout
    << "| Available commands:                                            |"
    << std::endl;
  std::cout
    << "| [a] Waypoint Mission                                           |"
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
