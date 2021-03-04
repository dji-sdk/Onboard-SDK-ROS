/** @file dji_vehicle_node_mission_services.cpp
 *  @version 4.0
 *  @date June 2020
 *
 *  @brief mission services.
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

#include <dji_osdk_ros/dji_vehicle_node.h>
using namespace dji_osdk_ros;

bool VehicleNode::missionStatusCallback(dji_osdk_ros::MissionStatus::Request&  request,
                                        dji_osdk_ros::MissionStatus::Response& response)
{
  ROS_DEBUG("called missionStatusCallback");

  Vehicle* vehicle = ptr_wrapper_->getVehicle();

  response.waypoint_mission_count = vehicle->missionManager->wpMissionVector.size();
  response.hotpoint_mission_count = vehicle->missionManager->hpMissionVector.size();
  return true;
}

bool VehicleNode::missionWpUploadCallback(dji_osdk_ros::MissionWpUpload::Request&  request,
                                          dji_osdk_ros::MissionWpUpload::Response& response)
{
  ROS_DEBUG("called missionWpUpload");

  Vehicle* vehicle = ptr_wrapper_->getVehicle();

  //! initialize waypoint mission related info
  ACK::ErrorCode                  initAck;
  DJI::OSDK::WayPointInitSettings wpInitData;
  wpInitData.indexNumber  = (unsigned char)request.waypoint_task.mission_waypoint.size();
  wpInitData.maxVelocity  = (float)request.waypoint_task.velocity_range;
  wpInitData.idleVelocity = (float)request.waypoint_task.idle_velocity;
  wpInitData.finishAction = (unsigned char)request.waypoint_task.action_on_finish;
  wpInitData.executiveTimes = (unsigned char)request.waypoint_task.mission_exec_times;
  wpInitData.yawMode        = (unsigned char)request.waypoint_task.yaw_mode;
  wpInitData.traceMode      = (unsigned char)request.waypoint_task.trace_mode;
  wpInitData.RCLostAction   = (unsigned char)request.waypoint_task.action_on_rc_lost;
  wpInitData.gimbalPitch    = (unsigned char)request.waypoint_task.gimbal_pitch_mode;
  wpInitData.latitude = 0.0;
  wpInitData.longitude = 0.0;
  wpInitData.altitude = 0.0;
  for (int i = 0; i < 16; i++){  
    wpInitData.reserved[i] = 0;
  } 

  initAck = vehicle->missionManager->init(DJI_MISSION_TYPE::WAYPOINT,
                                          WAIT_TIMEOUT, &wpInitData);

  ROS_DEBUG("ack.info: set=%i id=%i", initAck.info.cmd_set,
            initAck.info.cmd_id);
  ROS_DEBUG("ack.data: %i", initAck.data);

  response.cmd_set  = (int)initAck.info.cmd_set;
  response.cmd_id   = (int)initAck.info.cmd_id;
  response.ack_data = (unsigned int)initAck.data;

  if (ACK::getError(initAck))
  {
    ACK::getErrorCodeMessage(initAck, __func__);
    response.result = false;
  }

  ROS_INFO("initialized waypoint mission");
  sleep(1);

  //! initialize waypoint mission related info
  ACK::WayPointIndex          uploadAck;
  DJI::OSDK::WayPointSettings wpData;
  int                         i = 0;
  for (auto waypoint : request.waypoint_task.mission_waypoint)
  {
    wpData.latitude        = waypoint.latitude  * C_PI / 180;
    wpData.longitude       = waypoint.longitude * C_PI / 180;
    wpData.altitude        = waypoint.altitude;
    wpData.damping         = waypoint.damping_distance;
    wpData.yaw             = waypoint.target_yaw;
    wpData.gimbalPitch     = waypoint.target_gimbal_pitch;
    wpData.turnMode        = waypoint.turn_mode;
    wpData.hasAction       = waypoint.has_action;
    wpData.actionTimeLimit = waypoint.action_time_limit;
    wpData.actionNumber    = 15;
    wpData.actionRepeat    = waypoint.waypoint_action.action_repeat;
    wpData.index           = i;
    std::copy(waypoint.waypoint_action.command_list.begin(),
              waypoint.waypoint_action.command_list.end(), wpData.commandList);
    std::copy(waypoint.waypoint_action.command_parameter.begin(),
              waypoint.waypoint_action.command_parameter.end(),
              wpData.commandParameter);

    uploadAck = vehicle->missionManager->wpMission->uploadIndexData(
      &wpData, WAIT_TIMEOUT);

    ROS_DEBUG("uploaded waypoint lat: %f lon: %f alt: %f", waypoint.latitude,
              waypoint.longitude, waypoint.altitude);

    response.cmd_set  = (int)uploadAck.ack.info.cmd_set;
    response.cmd_id   = (int)uploadAck.ack.info.cmd_id;
    response.ack_data = (unsigned int)uploadAck.ack.data;

    if (ACK::getError(uploadAck.ack))
    {
      ACK::getErrorCodeMessage(uploadAck.ack, __func__);
      response.result = false;
    }
    else
    {
      response.result = true;
    }

    ROS_INFO("uploaded the %dth waypoint\n", (wpData.index + 1));
    i += 1;
    sleep(1);
  }

  ROS_INFO("waypoint mission initialized and uploaded");
  return true;
}

bool
VehicleNode::missionWpActionCallback(
  dji_osdk_ros::MissionWpAction::Request&  request,
  dji_osdk_ros::MissionWpAction::Response& response)
{
  ROS_DEBUG("called missionWpActionCallback");
  Vehicle* vehicle = ptr_wrapper_->getVehicle();

  if (vehicle->missionManager->wpMissionVector.size() == 0)
  {
    ROS_ERROR("no waypoint mission uploaded");
    response.result = false;
  }

  ACK::ErrorCode ack;
  switch (request.action)
  {
    case DJI::OSDK::MISSION_ACTION::START:
      ack = vehicle->missionManager->wpMission->start(WAIT_TIMEOUT);
      ROS_DEBUG("start waypoint mission");
      break;
    case DJI::OSDK::MISSION_ACTION::STOP:
      ack = vehicle->missionManager->wpMission->stop(WAIT_TIMEOUT);
      ROS_DEBUG("stop waypoint mission");
      break;
    case DJI::OSDK::MISSION_ACTION::PAUSE:
      ack = vehicle->missionManager->wpMission->pause(WAIT_TIMEOUT);
      ROS_DEBUG("pause waypoint mission");
      break;
    case DJI::OSDK::MISSION_ACTION::RESUME:
      ack = vehicle->missionManager->wpMission->resume(WAIT_TIMEOUT);
      ROS_DEBUG("resume waypoint mission");
      break;
    default:
      ROS_WARN("unknown action specified in MissionWpAction service");
      break;
  }

  ROS_DEBUG("ack.info: set=%i id=%i", ack.info.cmd_set, ack.info.cmd_id);
  ROS_DEBUG("ack.data: %i", ack.data);

  response.cmd_set  = (int)ack.info.cmd_set;
  response.cmd_id   = (int)ack.info.cmd_id;
  response.ack_data = (unsigned int)ack.data;

  if (ACK::getError(ack))
  {
    ACK::getErrorCodeMessage(ack, __func__);
    response.result = false;
  }
  else
  {
    response.result = true;
  }

  return true;
}

bool
VehicleNode::missionWpGetSpeedCallback(
  dji_osdk_ros::MissionWpGetSpeed::Request&  request,
  dji_osdk_ros::MissionWpGetSpeed::Response& response)
{
  ROS_DEBUG("called wpGetSpeedCallback");
  Vehicle* vehicle = ptr_wrapper_->getVehicle();

  if (vehicle->missionManager->wpMissionVector.size() > 0)
  {
    //! @todo bug here
    //    response.speed =
    //      (vehicle->missionManager->wpMission->readIdleVelocity(WAIT_TIMEOUT))
    //        .idleVelocity;
    //    vehicle->missionManager->wpMission->readIdleVelocity();
  }
  else
  {
    ROS_ERROR("no waypoint mission initiated ");
  }
  // @todo some bug in FC side, need to follow up
  std::cout << "response.speed " << response.speed << std::endl;

  return true;
}

bool
VehicleNode::missionWpSetSpeedCallback(
  dji_osdk_ros::MissionWpSetSpeed::Request&  request,
  dji_osdk_ros::MissionWpSetSpeed::Response& response)
{
  ROS_DEBUG("called wpSetSpeedCallback");
  Vehicle* vehicle = ptr_wrapper_->getVehicle();

  ACK::WayPointVelocity velAck;

  if (vehicle->missionManager->wpMissionVector.size() > 0)
  {
    velAck = (vehicle->missionManager->wpMission->updateIdleVelocity(
      request.speed, WAIT_TIMEOUT));
  }
  else
  {
    ROS_ERROR("no waypoint mission initiated ");
    response.result = false;
  }

  if (ACK::getError(velAck.ack))
  {
    ROS_DEBUG("wpSetSpeedCallback ack value: %d", (uint32_t)velAck.ack.data);
    response.result = false;
  }
  else
  {
    response.result = true;
  }

  return true;
}

bool VehicleNode::missionWpGetInfoCallback(dji_osdk_ros::MissionWpGetInfo::Request&  request,
                                           dji_osdk_ros::MissionWpGetInfo::Response& response)
{
  ROS_DEBUG("called missionWpGetInfoCallback");
  Vehicle* vehicle = ptr_wrapper_->getVehicle();

  DJI::OSDK::WayPointInitSettings info;
  if (vehicle->missionManager->wpMissionVector.size() > 0)
  {
    info = vehicle->missionManager->wpMission->getWaypointSettings(10).data;
  }
  else
  {
    ROS_ERROR("no waypoint mission initiated ");
    return false;
  }

  response.waypoint_task.mission_waypoint.resize(info.indexNumber);
  response.waypoint_task.velocity_range     = info.maxVelocity;
  response.waypoint_task.idle_velocity      = info.idleVelocity;
  response.waypoint_task.action_on_finish   = info.finishAction;
  response.waypoint_task.mission_exec_times = info.executiveTimes;
  response.waypoint_task.yaw_mode           = info.yawMode;
  response.waypoint_task.trace_mode         = info.traceMode;
  response.waypoint_task.action_on_rc_lost  = info.RCLostAction;
  response.waypoint_task.gimbal_pitch_mode  = info.gimbalPitch;

  for (int i=0; i< info.indexNumber; i++)
  {
    DJI::OSDK::WayPointSettings wpData; 
    wpData = vehicle->missionManager->wpMission->getIndex(i, 10).data;
    response.waypoint_task.mission_waypoint[i].latitude            = wpData.latitude  * 180.0 / C_PI;
    response.waypoint_task.mission_waypoint[i].longitude           = wpData.longitude * 180.0 / C_PI;
    response.waypoint_task.mission_waypoint[i].altitude            = wpData.altitude;
    response.waypoint_task.mission_waypoint[i].damping_distance    = wpData.damping;
    response.waypoint_task.mission_waypoint[i].target_yaw          = wpData.yaw;
    response.waypoint_task.mission_waypoint[i].target_gimbal_pitch = wpData.gimbalPitch;
    response.waypoint_task.mission_waypoint[i].turn_mode           = wpData.turnMode;
    response.waypoint_task.mission_waypoint[i].has_action          = wpData.hasAction;
    response.waypoint_task.mission_waypoint[i].action_time_limit   = wpData.actionTimeLimit;
    response.waypoint_task.mission_waypoint[i].waypoint_action.action_repeat = wpData.actionNumber + (wpData.actionRepeat << 4);

    std::copy(std::begin(wpData.commandList),
              std::end(wpData.commandList), 
              response.waypoint_task.mission_waypoint[i].waypoint_action.command_list.begin());

    std::copy(std::begin(wpData.commandParameter),
              std::end(wpData.commandParameter), 
              response.waypoint_task.mission_waypoint[i].waypoint_action.command_parameter.begin());

  }
  return true;
}

bool
VehicleNode::missionHpUploadCallback(
  dji_osdk_ros::MissionHpUpload::Request&  request,
  dji_osdk_ros::MissionHpUpload::Response& response)
{
  ROS_DEBUG("called missionHpUploadCallback");
  Vehicle* vehicle = ptr_wrapper_->getVehicle();

  DJI::OSDK::HotPointSettings* hpInitData = new DJI::OSDK::HotPointSettings();
  hpInitData->latitude   = request.hotpoint_task.latitude * C_PI / 180;
  hpInitData->longitude  = request.hotpoint_task.longitude * C_PI / 180;
  hpInitData->height     = request.hotpoint_task.altitude;
  hpInitData->radius     = request.hotpoint_task.radius;
  hpInitData->yawRate    = request.hotpoint_task.angular_speed;
  hpInitData->clockwise  = request.hotpoint_task.is_clockwise;
  hpInitData->startPoint = request.hotpoint_task.start_point;
  hpInitData->yawMode    = request.hotpoint_task.yaw_mode;

  vehicle->missionManager->init(DJI_MISSION_TYPE::HOTPOINT, WAIT_TIMEOUT,
                                (void*)hpInitData);

  response.result = true;
  return true;
}

bool
VehicleNode::missionHpActionCallback(
  dji_osdk_ros::MissionHpAction::Request&  request,
  dji_osdk_ros::MissionHpAction::Response& response)
{
  ROS_DEBUG("called missionHpActionCallback");
  Vehicle* vehicle = ptr_wrapper_->getVehicle();

  if (vehicle->missionManager->hpMissionVector.size() == 0)
  {
    ROS_ERROR("no hotpoint mission uploaded");
    response.result = false;
  }

  ACK::ErrorCode ack;
  switch (request.action)
  {
    case DJI::OSDK::MISSION_ACTION::START:
      ack = vehicle->missionManager->hpMission->start(WAIT_TIMEOUT);
      ROS_DEBUG("start hotpoint mission");
      break;
    case DJI::OSDK::MISSION_ACTION::STOP:
      ack = vehicle->missionManager->hpMission->stop(WAIT_TIMEOUT);
      ROS_DEBUG("stop hotpoint mission");
      break;
    case DJI::OSDK::MISSION_ACTION::PAUSE:
      ack = vehicle->missionManager->hpMission->pause(WAIT_TIMEOUT);
      ROS_DEBUG("pause hotpoint mission");
      break;
    case DJI::OSDK::MISSION_ACTION::RESUME:
      ack = vehicle->missionManager->hpMission->resume(WAIT_TIMEOUT);
      ROS_DEBUG("resume hotpoint mission");
      break;
    default:
      ROS_WARN("unknown action specified in MissionHpAction service");
      break;
  }

  ROS_DEBUG("ack.info: set=%i id=%i", ack.info.cmd_set, ack.info.cmd_id);
  ROS_DEBUG("ack.data: %i", ack.data);

  response.cmd_set  = (int)ack.info.cmd_set;
  response.cmd_id   = (int)ack.info.cmd_id;
  response.ack_data = (unsigned int)ack.data;

  if (ACK::getError(ack))
  {
    ACK::getErrorCodeMessage(ack, __func__);
    response.result = false;
  }
  else
  {
    response.result = true;
  }

  return true;
}

bool
VehicleNode::missionHpGetInfoCallback(
  dji_osdk_ros::MissionHpGetInfo::Request&  request,
  dji_osdk_ros::MissionHpGetInfo::Response& response)
{
  ROS_DEBUG("called missionHpGetInfoCallback");
  Vehicle* vehicle = ptr_wrapper_->getVehicle();

  DJI::OSDK::HotPointSettings info;
  if (vehicle->missionManager->hpMissionVector.size() > 0)
  {
    info = vehicle->missionManager->hpMission->getData();
  }
  else
  {
    ROS_ERROR("no hotpoint mission initiated ");
  }

  response.hotpoint_task.latitude      = info.latitude  * 180.0 / C_PI;
  response.hotpoint_task.longitude     = info.longitude * 180.0 / C_PI;
  response.hotpoint_task.altitude      = info.height;
  response.hotpoint_task.radius        = info.radius;
  response.hotpoint_task.angular_speed = info.yawRate;
  response.hotpoint_task.is_clockwise  = info.clockwise;
  response.hotpoint_task.start_point   = info.startPoint;
  response.hotpoint_task.yaw_mode      = info.yawMode;

  return true;
}

bool
VehicleNode::missionHpUpdateYawRateCallback(
  dji_osdk_ros::MissionHpUpdateYawRate::Request&  request,
  dji_osdk_ros::MissionHpUpdateYawRate::Response& response)
{
  ROS_DEBUG("called missionHpUpdateYawRateCallback");
  Vehicle* vehicle = ptr_wrapper_->getVehicle();

  DJI::OSDK::HotpointMission::YawRate yawRate;
  yawRate.yawRate   = request.yaw_rate;
  yawRate.clockwise = request.direction;

  if (vehicle->missionManager->hpMissionVector.size() > 0)
  {
    vehicle->missionManager->hpMission->updateYawRate(yawRate);
  }
  else
  {
    ROS_ERROR("no hotpoint mission initiated ");
  }

  response.result = true;

  return true;
}

bool
VehicleNode::missionHpResetYawCallback(
  dji_osdk_ros::MissionHpResetYaw::Request&  request,
  dji_osdk_ros::MissionHpResetYaw::Response& response)
{
  ROS_DEBUG("called missionHpResetYawCallback");
  Vehicle* vehicle = ptr_wrapper_->getVehicle();

  ACK::ErrorCode ack;
  if (vehicle->missionManager->hpMissionVector.size() > 0)
  {
    ack = vehicle->missionManager->hpMission->resetYaw(WAIT_TIMEOUT);
  }
  else
  {
    ROS_ERROR("no hotpoint mission initiated ");
  }

  ROS_DEBUG("ack.info: set=%i id=%i", ack.info.cmd_set, ack.info.cmd_id);
  ROS_DEBUG("ack.data: %i", ack.data);

  response.cmd_set  = (int)ack.info.cmd_set;
  response.cmd_id   = (int)ack.info.cmd_id;
  response.ack_data = (unsigned int)ack.data;

  if (ACK::getError(ack))
  {
    ACK::getErrorCodeMessage(ack, __func__);
    response.result = false;
  }
  else
  {
    response.result = true;
  }

  return true;
}

bool
VehicleNode::missionHpUpdateRadiusCallback(
  dji_osdk_ros::MissionHpUpdateRadius::Request&  request,
  dji_osdk_ros::MissionHpUpdateRadius::Response& response)
{
  ROS_DEBUG("called missionHpUpdateRadiusCallback");
  Vehicle* vehicle = ptr_wrapper_->getVehicle();

  if (vehicle->missionManager->hpMissionVector.size() > 0)
  {
    vehicle->missionManager->hpMission->updateRadius(request.radius);
  }
  else
  {
    ROS_ERROR("no hotpoint mission initiated ");
  }
   
   response.result = true;

  return true;
}

bool VehicleNode::waypointV2InitSettingCallback(
  dji_osdk_ros::InitWaypointV2Setting::Request&  request,
  dji_osdk_ros::InitWaypointV2Setting::Response& response)
{
  ROS_INFO("called waypointV2InitSettingCallback");
  response.result = false;
  if(ptr_wrapper_ == nullptr)
  {
    ROS_ERROR_STREAM("Vehicle modules is nullptr");
    return false;
  }
  uint16_t polygonNum = request.polygonNum;
  float32_t radius = request.radius;

  uint16_t actionNum = request.actionNum;
  srand(int(time(0)));

  /*! Init waypoint settings*/
  DJI::OSDK::WayPointV2InitSettings missionInitSettings;
  DJI::OSDK::WaypointV2 waypointV2Vector;
  missionInitSettings.missionID                         = rand();
  missionInitSettings.repeatTimes                       = request.waypointV2InitSettings.repeatTimes;
  missionInitSettings.finishedAction                    = static_cast<DJI::OSDK::DJIWaypointV2MissionFinishedAction>(request.waypointV2InitSettings.finishedAction);
  missionInitSettings.maxFlightSpeed                    = request.waypointV2InitSettings.maxFlightSpeed;
  missionInitSettings.autoFlightSpeed                   = request.waypointV2InitSettings.autoFlightSpeed;
  missionInitSettings.exitMissionOnRCSignalLost         = request.waypointV2InitSettings.exitMissionOnRCSignalLost;
  missionInitSettings.gotoFirstWaypointMode             = static_cast<DJI::OSDK::DJIWaypointV2MissionGotoFirstWaypointMode>(request.waypointV2InitSettings.gotoFirstWaypointMode);

  for (uint16_t i = 0; i < request.waypointV2InitSettings.mission.size(); i++)
  {
    waypointV2Vector.longitude                 = request.waypointV2InitSettings.mission[i].longitude;
    waypointV2Vector.latitude                  = request.waypointV2InitSettings.mission[i].latitude;
    waypointV2Vector.relativeHeight            = request.waypointV2InitSettings.mission[i].relativeHeight;
    waypointV2Vector.waypointType              = static_cast<DJI::OSDK::DJIWaypointV2FlightPathMode>(request.waypointV2InitSettings.mission[i].waypointType);
    waypointV2Vector.headingMode               = static_cast<DJI::OSDK::DJIWaypointV2HeadingMode>(request.waypointV2InitSettings.mission[i].headingMode);
    waypointV2Vector.config.useLocalCruiseVel  = request.waypointV2InitSettings.mission[i].config.useLocalCruiseVel;
    waypointV2Vector.config.useLocalMaxVel     = request.waypointV2InitSettings.mission[i].config.useLocalMaxVel;
    waypointV2Vector.dampingDistance           = request.waypointV2InitSettings.mission[i].dampingDistance;
    waypointV2Vector.heading                   = request.waypointV2InitSettings.mission[i].heading;
    waypointV2Vector.turnMode                  = static_cast<DJI::OSDK::DJIWaypointV2TurnMode>(request.waypointV2InitSettings.mission[i].turnMode);
    waypointV2Vector.pointOfInterest.positionX = request.waypointV2InitSettings.mission[i].positionX;
    waypointV2Vector.pointOfInterest.positionY = request.waypointV2InitSettings.mission[i].positionY;
    waypointV2Vector.pointOfInterest.positionZ = request.waypointV2InitSettings.mission[i].positionZ;
    waypointV2Vector.maxFlightSpeed            = request.waypointV2InitSettings.mission[i].maxFlightSpeed;
    waypointV2Vector.autoFlightSpeed           = request.waypointV2InitSettings.mission[i].autoFlightSpeed;

    missionInitSettings.mission.push_back(waypointV2Vector);
  }

  missionInitSettings.missTotalLen = missionInitSettings.mission.size();

  response.result = ptr_wrapper_->initWaypointV2(&missionInitSettings,WAIT_TIMEOUT);

  return response.result;

}

bool VehicleNode::waypointV2UploadMissionCallback(
  dji_osdk_ros::UploadWaypointV2Mission::Request&  request,
  dji_osdk_ros::UploadWaypointV2Mission::Response& response)
{
  ROS_INFO("called waypointV2UploadMissionCallback");
  response.result = false;
  if(ptr_wrapper_ == nullptr)
  {
    ROS_ERROR_STREAM("Vehicle modules is nullptr");
    return false;
  }

  response.result = ptr_wrapper_->uploadWaypointV2(WAIT_TIMEOUT);

  return response.result;
}

bool VehicleNode::waypointV2DownloadMissionCallback(
  dji_osdk_ros::DownloadWaypointV2Mission::Request&  request,
  dji_osdk_ros::DownloadWaypointV2Mission::Response& response)
{
  ROS_INFO("called waypointV2DownloadMissionCallback");
  response.result = false;
  if(ptr_wrapper_ == nullptr)
  {
    ROS_ERROR_STREAM("Vehicle modules is nullptr");
    return false;
  }

  std::vector<DJI::OSDK::WaypointV2> mission;
  dji_osdk_ros::WaypointV2 waypointV2Vector;

  response.result = ptr_wrapper_->downloadWaypointV2(mission, WAIT_TIMEOUT);
  for (uint16_t i = 0; i < mission.size(); i++)
  {
    waypointV2Vector.longitude                 = mission[i].longitude;
    waypointV2Vector.latitude                  = mission[i].latitude;
    waypointV2Vector.relativeHeight            = mission[i].relativeHeight;
    waypointV2Vector.waypointType              = static_cast<DJI::OSDK::DJIWaypointV2FlightPathMode>(mission[i].waypointType);
    waypointV2Vector.headingMode               = static_cast<DJI::OSDK::DJIWaypointV2HeadingMode>(mission[i].headingMode);
    waypointV2Vector.config.useLocalCruiseVel  = mission[i].config.useLocalCruiseVel;
    waypointV2Vector.config.useLocalMaxVel     = mission[i].config.useLocalMaxVel;
    waypointV2Vector.dampingDistance           = mission[i].dampingDistance;
    waypointV2Vector.heading                   = mission[i].heading;
    waypointV2Vector.turnMode                  = static_cast<DJI::OSDK::DJIWaypointV2TurnMode>(mission[i].turnMode);
    waypointV2Vector.positionX                 = mission[i].pointOfInterest.positionX;
    waypointV2Vector.positionY                 = mission[i].pointOfInterest.positionY;
    waypointV2Vector.positionZ                 = mission[i].pointOfInterest.positionZ;
    waypointV2Vector.maxFlightSpeed            = mission[i].maxFlightSpeed;
    waypointV2Vector.autoFlightSpeed           = mission[i].autoFlightSpeed;

    response.mission.push_back(waypointV2Vector);
  }

  return response.result;
}

bool VehicleNode::waypointV2UploadActionCallback(
  dji_osdk_ros::UploadWaypointV2Action::Request&  request,
  dji_osdk_ros::UploadWaypointV2Action::Response& response)
{
  ROS_INFO("called waypointV2UploadActionCallback");
  response.result = false;
  if(ptr_wrapper_ == nullptr)
  {
    ROS_ERROR_STREAM("Vehicle modules is nullptr");
    return false;
  }
  if (this->actions.size() == 0)
  {
    return false;
  }
  response.result = ptr_wrapper_->uploadWaypointV2Actions(this->actions, WAIT_TIMEOUT);

  return response.result;
}

bool VehicleNode::waypointV2StartMissionCallback(
  dji_osdk_ros::StartWaypointV2Mission::Request&  request,
  dji_osdk_ros::StartWaypointV2Mission::Response& response)
{
  ROS_INFO("called waypointV2StartMissionCallback");
  response.result = false;
  if(ptr_wrapper_ == nullptr)
  {
    ROS_ERROR_STREAM("Vehicle modules is nullptr");
    return false;
  }

  response.result = ptr_wrapper_->startWaypointV2Mission(WAIT_TIMEOUT);
}

bool VehicleNode::waypointV2StopMissionCallback(
  dji_osdk_ros::StopWaypointV2Mission::Request&  request,
  dji_osdk_ros::StopWaypointV2Mission::Response& response)
{
  ROS_INFO("called waypointV2StopMissionCallback");
  response.result = false;
  if(ptr_wrapper_ == nullptr)
  {
    ROS_ERROR_STREAM("Vehicle modules is nullptr");
    return false;
  }

  response.result = ptr_wrapper_->stopWaypointV2Mission(WAIT_TIMEOUT);
}

bool VehicleNode::waypointV2PauseMissionCallback(
  dji_osdk_ros::PauseWaypointV2Mission::Request&  request,
  dji_osdk_ros::PauseWaypointV2Mission::Response& response)
{
  ROS_INFO("called waypointV2PauseMissionCallback");
  response.result = false;
  if(ptr_wrapper_ == nullptr)
  {
    ROS_ERROR_STREAM("Vehicle modules is nullptr");
    return false;
  }

  response.result = ptr_wrapper_->pauseWaypointV2Mission(WAIT_TIMEOUT);
}

bool VehicleNode::waypointV2ResumeMissionCallback(
  dji_osdk_ros::ResumeWaypointV2Mission::Request&  request,
  dji_osdk_ros::ResumeWaypointV2Mission::Response& response)
{
  ROS_INFO("called waypointV2ResumeMissionCallback");
  response.result = false;
  if(ptr_wrapper_ == nullptr)
  {
    ROS_ERROR_STREAM("Vehicle modules is nullptr");
    return false;
  }

  response.result = ptr_wrapper_->resumeWaypointV2Mission(WAIT_TIMEOUT);
}

bool VehicleNode::waypointV2GenerateActionsCallback(
  dji_osdk_ros::GenerateWaypointV2Action::Request&  request,
  dji_osdk_ros::GenerateWaypointV2Action::Response& response)
{
  ROS_INFO("called waypointV2GenerateActionsCallback");
  response.result = false;
  if(ptr_wrapper_ == nullptr)
  {
    ROS_ERROR_STREAM("Vehicle modules is nullptr");
    return false;
  }
  DJI::OSDK::DJIWaypointV2Trigger *trigger;
  DJI::OSDK::DJIWaypointV2Actuator *actuator;
  DJI::OSDK::DJIWaypointV2CameraActuatorParam *cameraActuatorParam;
  DJI::OSDK::DJIWaypointV2GimbalActuatorParam *gimbalActuatorParam;
  DJI::OSDK::DJIWaypointV2AircraftControlParam *aircraftControlActuatorParam;

  for (uint16_t i = 0; i < request.actions.size(); i++)
  {
     switch(request.actions[i].waypointV2ActionTriggerType)
     {
       case dji_osdk_ros::WaypointV2Action::DJIWaypointV2ActionTriggerTypeActionAssociated:
       { 
         DJI::OSDK::DJIWaypointV2AssociateTriggerParam param;
         param.actionAssociatedType = static_cast<DJI::OSDK::DJIWaypointV2TriggerAssociatedTimingType>
                                      (request.actions[i].waypointV2AssociateTrigger.actionAssociatedType);
         param.actionIdAssociated   = request.actions[i].waypointV2AssociateTrigger.actionIdAssociated;
         param.waitingTime          = request.actions[i].waypointV2AssociateTrigger.waitingTime;
         trigger = new DJI::OSDK::DJIWaypointV2Trigger(
           DJI::OSDK::DJIWaypointV2ActionTriggerTypeActionAssociated, &param);
         response.result = true;

         break;
       }

       case dji_osdk_ros::WaypointV2Action::DJIWaypointV2ActionTriggerTypeTrajectory:
       { 
         DJI::OSDK::DJIWaypointV2TrajectoryTriggerParam param;
         param.endIndex   = request.actions[i].waypointV2TrajectoryTrigger.endIndex;
         param.startIndex = request.actions[i].waypointV2TrajectoryTrigger.startIndex;
         trigger = new DJI::OSDK::DJIWaypointV2Trigger(
           DJI::OSDK::DJIWaypointV2ActionTriggerTypeTrajectory, &param);

         response.result = true;
         break;
       }

       case dji_osdk_ros::WaypointV2Action::DJIWaypointV2ActionTriggerTypeInterval:
       { 
         DJI::OSDK::DJIWaypointV2IntervalTriggerParam param;
         param.actionIntervalType = static_cast<DJI::OSDK::DJIWaypointV2ActionIntervalType>
                                    (request.actions[i].waypointV2IntervalTrigger.actionIntervalType);
         param.interval           = request.actions[i].waypointV2IntervalTrigger.interval;
         param.startIndex         = request.actions[i].waypointV2IntervalTrigger.startIndex;
         trigger = new DJI::OSDK::DJIWaypointV2Trigger(
           DJI::OSDK::DJIWaypointV2ActionTriggerTypeInterval, &param);

         response.result = true;
         break;
       }

       case dji_osdk_ros::WaypointV2Action::DJIWaypointV2ActionTriggerTypeSampleReachPoint:
       { 
         DJI::OSDK::DJIWaypointV2SampleReachPointTriggerParam param;
         param.terminateNum  = request.actions[i].waypointV2SampleReachPointTrigger.terminateNum;
         param.waypointIndex = request.actions[i].waypointV2SampleReachPointTrigger.waypointIndex;
         trigger = new DJI::OSDK::DJIWaypointV2Trigger(
           DJI::OSDK::DJIWaypointV2ActionTriggerTypeSampleReachPoint, &param);

         response.result = true;
         break;
       }

       default:
       {
         ROS_DEBUG("Invalid trigger type%d\n", request.actions[i].waypointV2ActionTriggerType);
         response.result = false;
         break;
       }

     }

     switch(request.actions[i].waypointV2ACtionActuatorType)
     {
       case dji_osdk_ros::WaypointV2Action::DJIWaypointV2ActionActuatorTypeCamera:
       { 
         if (request.actions[i].waypointV2CameraActuator.DJIWaypointV2ActionActuatorCameraOperationType ==  
             dji_osdk_ros::WaypointV2CameraActuator::DJIWaypointV2ActionActuatorCameraOperationTypeFocus)
         {
            DJI::OSDK::DJIWaypointV2CameraFocusParam focusParam;
            focusParam.focusTarget.x = request.actions[i].waypointV2CameraActuator.focusParam.x;
            focusParam.focusTarget.y = request.actions[i].waypointV2CameraActuator.focusParam.y;
            focusParam.regionType = request.actions[i].waypointV2CameraActuator.focusParam.regionType;
            focusParam.width = request.actions[i].waypointV2CameraActuator.focusParam.width;
            focusParam.height = request.actions[i].waypointV2CameraActuator.focusParam.height;
            focusParam.retryTimes = request.actions[i].waypointV2CameraActuator.focusParam.retryTimes;
            cameraActuatorParam = new DJI::OSDK::DJIWaypointV2CameraActuatorParam(DJI::OSDK::DJIWaypointV2ActionActuatorCameraOperationTypeFocus, &focusParam);
         }
         else if(request.actions[i].waypointV2CameraActuator.DJIWaypointV2ActionActuatorCameraOperationType ==  
                 dji_osdk_ros::WaypointV2CameraActuator::DJIWaypointV2ActionActuatorCameraOperationTypeFocalLength)
         {
            DJI::OSDK::DJIWaypointV2CameraFocalLengthParam zoomParam;
            zoomParam.retryTimes = request.actions[i].waypointV2CameraActuator.zoomParam.retryTimes;
            zoomParam.focalLength = request.actions[i].waypointV2CameraActuator.zoomParam.focalLength;
            cameraActuatorParam = new DJI::OSDK::DJIWaypointV2CameraActuatorParam(DJI::OSDK::DJIWaypointV2ActionActuatorCameraOperationTypeFocalLength, &zoomParam);
         }
         else if(request.actions[i].waypointV2CameraActuator.DJIWaypointV2ActionActuatorCameraOperationType ==  
                 dji_osdk_ros::WaypointV2CameraActuator::DJIWaypointV2ActionActuatorCameraOperationTypeTakePhoto)
         {
            cameraActuatorParam = new DJI::OSDK::DJIWaypointV2CameraActuatorParam(DJI::OSDK::DJIWaypointV2ActionActuatorCameraOperationTypeTakePhoto, nullptr);
         }
         else if(request.actions[i].waypointV2CameraActuator.DJIWaypointV2ActionActuatorCameraOperationType ==  
                 dji_osdk_ros::WaypointV2CameraActuator::DJIWaypointV2ActionActuatorCameraOperationTypeStartRecordVideo)
         {
            cameraActuatorParam = new DJI::OSDK::DJIWaypointV2CameraActuatorParam(DJI::OSDK::DJIWaypointV2ActionActuatorCameraOperationTypeStartRecordVideo, nullptr);
         }
         else if(request.actions[i].waypointV2CameraActuator.DJIWaypointV2ActionActuatorCameraOperationType ==  
                 dji_osdk_ros::WaypointV2CameraActuator::DJIWaypointV2ActionActuatorCameraOperationTypeStopRecordVideo)
         {
            cameraActuatorParam = new DJI::OSDK::DJIWaypointV2CameraActuatorParam(DJI::OSDK::DJIWaypointV2ActionActuatorCameraOperationTypeStopRecordVideo, nullptr);
         }
         else
         {
            cameraActuatorParam = nullptr;
         }
         
         actuator = new DJI::OSDK::DJIWaypointV2Actuator(
           DJI::OSDK::DJIWaypointV2ActionActuatorTypeCamera, request.actions[i].waypointV2CameraActuator.actuatorIndex, cameraActuatorParam);
         response.result = true;

         break;
       }

       case dji_osdk_ros::WaypointV2Action::DJIWaypointV2ActionActuatorTypeGimbal:
       { 
         if (request.actions[i].waypointV2GimbalActuator.DJIWaypointV2ActionActuatorGimbalOperationType == 
             dji_osdk_ros::WaypointV2GimbalActuator::DJIWaypointV2ActionActuatorGimbalOperationTypeRotateGimbal)
         {
            DJI::OSDK::DJIGimbalRotation param;
            param.x              = request.actions[i].waypointV2GimbalActuator.waypointV2GimbalActuatorRotationParam.x;
            param.y              = request.actions[i].waypointV2GimbalActuator.waypointV2GimbalActuatorRotationParam.y;
            param.z              = request.actions[i].waypointV2GimbalActuator.waypointV2GimbalActuatorRotationParam.z;
            param.ctrl_mode      = request.actions[i].waypointV2GimbalActuator.waypointV2GimbalActuatorRotationParam.ctrl_mode;
            param.rollCmdIgnore  = request.actions[i].waypointV2GimbalActuator.waypointV2GimbalActuatorRotationParam.rollCmdIgnore;
            param.pitchCmdIgnore = request.actions[i].waypointV2GimbalActuator.waypointV2GimbalActuatorRotationParam.pitchCmdIgnore;
            param.yawCmdIgnore   = request.actions[i].waypointV2GimbalActuator.waypointV2GimbalActuatorRotationParam.yawCmdIgnore;
            param.absYawModeRef  = request.actions[i].waypointV2GimbalActuator.waypointV2GimbalActuatorRotationParam.absYawModeRef;
            param.duationTime    = request.actions[i].waypointV2GimbalActuator.waypointV2GimbalActuatorRotationParam.duationTime;
            gimbalActuatorParam  = new DJI::OSDK::DJIWaypointV2GimbalActuatorParam(DJI::OSDK::DJIWaypointV2ActionActuatorGimbalOperationTypeRotateGimbal, &param);
         }
         else
         {
            gimbalActuatorParam = nullptr;
         }
         actuator = new DJI::OSDK::DJIWaypointV2Actuator(
           DJI::OSDK::DJIWaypointV2ActionActuatorTypeGimbal, request.actions[i].waypointV2GimbalActuator.actuatorIndex, gimbalActuatorParam);
         response.result = true;

         break;
       }

       case dji_osdk_ros::WaypointV2Action::DJIWaypointV2ActionActuatorTypeAircraftControl:
       { 
         if (request.actions[i].waypointV2AircraftControlActuator.DJIWaypointV2ActionActuatorAircraftControlOperationType ==
             dji_osdk_ros::WaypointV2AircraftControlActuator::DJIWaypointV2ActionActuatorAircraftControlOperationTypeRotateYaw)             
         {
            DJI::OSDK::DJIWaypointV2AircraftControlRotateHeadingParam param;
            param.isRelative = request.actions[i].waypointV2AircraftControlActuator.waypointV2AircraftControlActuatorRotateHeading.isRelative;
            param.yaw        = request.actions[i].waypointV2AircraftControlActuator.waypointV2AircraftControlActuatorRotateHeading.yaw;
            aircraftControlActuatorParam = new DJI::OSDK::DJIWaypointV2AircraftControlParam(DJI::OSDK::DJIWaypointV2ActionActuatorAircraftControlOperationTypeRotateYaw, &param);
         }
         else if(request.actions[i].waypointV2AircraftControlActuator.DJIWaypointV2ActionActuatorAircraftControlOperationType ==
                 dji_osdk_ros::WaypointV2AircraftControlActuator::DJIWaypointV2ActionActuatorAircraftControlOperationTypeFlyingControl)
         {
            DJI::OSDK::DJIWaypointV2AircraftControlFlyingParam param;
            param.isStartFlying = request.actions[i].waypointV2AircraftControlActuator.waypointV2AircraftControlActuatorFlying.isStartFlying;
            aircraftControlActuatorParam = new DJI::OSDK::DJIWaypointV2AircraftControlParam(DJI::OSDK::DJIWaypointV2ActionActuatorAircraftControlOperationTypeFlyingControl, &param);
         }
         else
         {
            aircraftControlActuatorParam = nullptr;
         }
         actuator = new DJI::OSDK::DJIWaypointV2Actuator(
           DJI::OSDK::DJIWaypointV2ActionActuatorTypeAircraftControl, request.actions[i].waypointV2AircraftControlActuator.actuatorIndex, aircraftControlActuatorParam);
         response.result = true;

         break;
       }

       default:
       {
         ROS_DEBUG("Invalid actuator type%d\n", request.actions[i].waypointV2ACtionActuatorType);
         response.result = true;
 
         break;
       }

     }

    if(trigger != NULL && actuator != NULL)
    {
      auto *action = new DJI::OSDK::DJIWaypointV2Action(i, *trigger,*actuator);
      this->actions.push_back(*action);
      response.result = true;
    }
  }
  return response.result;
}

bool VehicleNode::waypointV2SetGlobalCruisespeedCallback(
  dji_osdk_ros::SetGlobalCruisespeed::Request& request,
  dji_osdk_ros::SetGlobalCruisespeed::Response& response
)
{
  ROS_INFO("called waypointV2SetGlobalCruisespeedCallback");
  response.result = false;

  if(ptr_wrapper_ == nullptr)
  {
    ROS_ERROR_STREAM("Vehicle modules is nullptr");
    return false;
  }

  response.result = ptr_wrapper_->setGlobalCruiseSpeed(request.global_cruisespeed, WAIT_TIMEOUT);

  return response.result;
}

bool VehicleNode::waypointV2GetGlobalCruisespeedCallback(
  dji_osdk_ros::GetGlobalCruisespeed::Request& request,
  dji_osdk_ros::GetGlobalCruisespeed::Response& response
)
{
  ROS_INFO("called waypointV2GetGlobalCruisespeedCallback");
  response.result = false;

  if(ptr_wrapper_ == nullptr)
  {
    ROS_ERROR_STREAM("Vehicle modules is nullptr");
    return false;
  }

  response.result = ptr_wrapper_->getGlobalCruiseSpeed(response.global_cruisespeed, WAIT_TIMEOUT);
}

//10HZ push
E_OsdkStat VehicleNode::updateMissionState(T_CmdHandle *cmdHandle, const T_CmdInfo *cmdInfo,
                                           const uint8_t *cmdData, void *userData) {
  if (cmdInfo) {
    if (userData) {
      auto *vh_ = (VehicleNode *)userData;
      auto *missionStatePushAck =
        (DJI::OSDK::MissionStatePushAck *)cmdData;
      dji_osdk_ros::WaypointV2MissionStatePush waypointV2MissionStatePushInfo;

      waypointV2MissionStatePushInfo.commonDataVersion = missionStatePushAck->commonDataVersion;
      waypointV2MissionStatePushInfo.commonDataLen     = missionStatePushAck->commonDataLen;
      waypointV2MissionStatePushInfo.state             = missionStatePushAck->data.state;
      waypointV2MissionStatePushInfo.curWaypointIndex  = missionStatePushAck->data.curWaypointIndex;
      waypointV2MissionStatePushInfo.velocity          = missionStatePushAck->data.velocity;

      vh_->waypointV2_mission_state_publisher_.publish(waypointV2MissionStatePushInfo);

    } else {
      DERROR("cmdInfo is a null value");
    }

    return OSDK_STAT_OK;
  }
  return OSDK_STAT_ERR_ALLOC;
}

/*! only push 0x00,0x10,0x11 event*/
E_OsdkStat VehicleNode::updateMissionEvent(T_CmdHandle *cmdHandle, const T_CmdInfo *cmdInfo,
                                           const uint8_t *cmdData, void *userData) {

  if (cmdInfo) {
    if (userData) {
      auto *MissionEventPushAck =
        (DJI::OSDK::MissionEventPushAck *)cmdData;
      auto *vh_ = (VehicleNode *)userData;

      dji_osdk_ros::WaypointV2MissionEventPush waypointV2MissionEventPushInfo;
      waypointV2MissionEventPushInfo.event = MissionEventPushAck->event;

      if(MissionEventPushAck->event == 0x01)
      {
        waypointV2MissionEventPushInfo.interruptReason = MissionEventPushAck->data.interruptReason;
      }
      if(MissionEventPushAck->event == 0x02)
      {
        waypointV2MissionEventPushInfo.recoverProcess  = MissionEventPushAck->data.recoverProcess;
      }
      if(MissionEventPushAck->event == 0x03)
      {
        waypointV2MissionEventPushInfo.finishReason    = MissionEventPushAck->data.finishReason;
      }

      if(MissionEventPushAck->event == 0x10)
      {
        waypointV2MissionEventPushInfo.waypointIndex   = MissionEventPushAck->data.waypointIndex;
      }

      if(MissionEventPushAck->event == 0x11)
      {
        waypointV2MissionEventPushInfo.currentMissionExecNum = MissionEventPushAck->data.MissionExecEvent.currentMissionExecNum;
        waypointV2MissionEventPushInfo.finishedAllExecNum    = MissionEventPushAck->data.MissionExecEvent.finishedAllExecNum;
      }

      vh_->waypointV2_mission_event_publisher_.publish(waypointV2MissionEventPushInfo);

      return OSDK_STAT_OK;
    }
  }
  return OSDK_STAT_SYS_ERR;
}

bool VehicleNode::waypointV2SubscribeMissionEventCallback(
  dji_osdk_ros::SubscribeWaypointV2Event::Request& request,
  dji_osdk_ros::SubscribeWaypointV2Event::Response& response
)
{
  ROS_INFO("called waypointV2SubscribeMissionEventCallback");
  response.result = false;

  if(ptr_wrapper_ == nullptr)
  {
    ROS_ERROR_STREAM("Vehicle modules is nullptr");
    return false;
  }
  if (request.enable_sub)
  {
    response.result = ptr_wrapper_->RegisterMissionEventCallback(this, updateMissionEvent);
  }
  else
  {
    response.result = ptr_wrapper_->RegisterMissionEventCallback(this, nullptr);
  }

  return response.result;
}

bool VehicleNode::waypointV2SubscribeMissionStateCallback(
  dji_osdk_ros::SubscribeWaypointV2State::Request& request,
  dji_osdk_ros::SubscribeWaypointV2State::Response& response
)
{
  ROS_INFO("called waypointV2SubscribeMissionStateCallback");
  response.result = false;

  if(ptr_wrapper_ == nullptr)
  {
    ROS_ERROR_STREAM("Vehicle modules is nullptr");
    return false;
  }

  if (request.enable_sub)
  {
    response.result = ptr_wrapper_->RegisterMissionStateCallback(this, updateMissionState);
  }
  else
  {
    response.result = ptr_wrapper_->RegisterMissionStateCallback(this, nullptr);
  }

  return response.result;
}
