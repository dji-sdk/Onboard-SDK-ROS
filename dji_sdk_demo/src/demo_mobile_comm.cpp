/** @file demo_mobile_comm.cpp
 *  @version 3.3
 *  @date May, 2017
 *
 *  @brief
 *  demo sample of how to use mobile communication APIs
 *
 *  @copyright 2017 DJI. All rights reserved.
 *
 */

#include <dji_sdk_demo/demo_mobile_comm.h>
#include "dji_sdk/dji_sdk.h"

using namespace DJI::OSDK;

//! Subscriber
ros::Subscriber from_mobile_data_subscriber;
ros::Subscriber gps_pos_subscriber;
//! Publishers
ros::Publisher ctrlPosYawPub;
ros::Publisher ctrlVelYawRatePub;
//! Flight control mission
Mission square_mission;
//! Services
ros::ServiceClient drone_arm_service;
ros::ServiceClient drone_task_service;
ros::ServiceClient query_version_service;
ros::ServiceClient mobile_data_service;
ros::ServiceClient drone_activation_service;
ros::ServiceClient sdk_ctrl_authority_service;
ros::ServiceClient waypoint_upload_service;
ros::ServiceClient waypoint_action_service;
ros::ServiceClient hotpoint_upload_service;
ros::ServiceClient hotpoint_action_service;
ros::ServiceClient hotpoint_update_yawRate_Service;
ros::ServiceClient hotpoint_updateRadius_service;
//! Constants
const float deg2rad = C_PI/180.0;
const float rad2deg = 180.0/C_PI;
uint8_t flight_status = 255;
uint8_t display_mode  = 255;
//! ROS related data
sensor_msgs::NavSatFix    gps_pos;
dji_sdk::MobileData       mobile_data;
sensor_msgs::NavSatFix    current_gps;
geometry_msgs::Quaternion current_atti;


int main(int argc, char *argv[]) {
  int main_operate_code = 0;
  int temp32;
  bool userExitCommand = false;
  ros::init(argc, argv, "sdk_client");
  ROS_INFO("sdk_service_client_test");
  ros::NodeHandle nh;

  from_mobile_data_subscriber = nh.subscribe<dji_sdk::MobileData>
      ("dji_sdk/from_mobile_data", 10, &fromMobileDataSubscriberCallback);
  ros::Subscriber flightStatusSub = nh.subscribe("dji_sdk/flight_status",
                                                 10, &flight_status_callback);
  ros::Subscriber attitudeSub = nh.subscribe("dji_sdk/attitude", 10,
                                             &attitude_callback);
  ros::Subscriber gpsSub      = nh.subscribe("dji_sdk/gps_position", 10,
                                             &gps_callback);
  ros::Subscriber displayModeSub = nh.subscribe("dji_sdk/display_mode", 10,
                                                &display_mode_callback);
  mobile_data_service = nh.serviceClient<dji_sdk::SendMobileData>
      ("dji_sdk/send_data_to_mobile");
  drone_activation_service  = nh.serviceClient<dji_sdk::Activation>
      ("dji_sdk/activation");
  drone_arm_service         = nh.serviceClient<dji_sdk::DroneArmControl>
      ("dji_sdk/drone_arm_control");
  drone_task_service        = nh.serviceClient<dji_sdk::DroneTaskControl>
      ("dji_sdk/drone_task_control");
  sdk_ctrl_authority_service= nh.serviceClient<dji_sdk::SDKControlAuthority>
      ("dji_sdk/sdk_control_authority");
  ctrlPosYawPub = nh.advertise<sensor_msgs::Joy>
      ("/dji_sdk/flight_control_setpoint_ENUposition_yaw", 10);
  ctrlVelYawRatePub = nh.advertise<sensor_msgs::Joy>
      ("dji_sdk/flight_control_setpoint_ENUvelocity_yawrate", 10);
  waypoint_upload_service   = nh.serviceClient<dji_sdk::MissionWpUpload>
      ("dji_sdk/mission_waypoint_upload");
  waypoint_action_service   = nh.serviceClient<dji_sdk::MissionWpAction>
      ("dji_sdk/mission_waypoint_action");
  hotpoint_upload_service   = nh.serviceClient<dji_sdk::MissionHpUpload>
      ("dji_sdk/mission_hotpoint_upload");
  hotpoint_action_service   = nh.serviceClient<dji_sdk::MissionHpAction>
      ("dji_sdk/mission_hotpoint_action");
  hotpoint_updateRadius_service   = nh.serviceClient<dji_sdk::MissionHpUpdateRadius>
      ("dji_sdk/mission_hotpoint_updateRadius");
  hotpoint_update_yawRate_Service = nh.serviceClient<dji_sdk::MissionHpUpdateYawRate>
      ("dji_sdk/mission_hotpoint_updateYawRate");
  gps_pos_subscriber = nh.subscribe<sensor_msgs::NavSatFix>
      ("dji_sdk/gps_position", 10, &gpsPosCallback);
  query_version_service      = nh.serviceClient<dji_sdk::QueryDroneVersion>
      ("dji_sdk/query_drone_version");

  Display_Main_Menu();
  while (!userExitCommand && ros::ok()) {

    ros::spinOnce();
    std::cout << "Enter Input Val: ";
    while (!(std::cin >> temp32)) {
      std::cin.clear();
      std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
      std::cout << "Invalid input.  Try again: ";
    }

    if (temp32 > 0 && temp32 < 3) {
      main_operate_code = temp32;
    } else {
      ROS_ERROR("Out of range Input ");
      Display_Main_Menu();
      continue;
    }
    switch (main_operate_code) {
      case 1: {
        ROS_INFO("Mobile Data Commands mode entered. Use OSDK Mobile App to use this feature");
        ROS_INFO("End program to exit this mode");
        while (ros::ok()) {
          ros::spinOnce();
          ros::Duration(0.005).sleep();
        }
        break;
      }
      case 2: {
        userExitCommand = true;
        break;
      }
    }
    main_operate_code = -1;
    Display_Main_Menu();
  }
  return 0;
}

static void Display_Main_Menu(void)
{
  printf("\r\n");
  printf("+------------------------------ < Main menu > ---------------------------------+\n");
  printf("| [1]  Enter Mobile Communication Mode \n");
  printf("| [2]  Exit Demo");
  printf("\r\n-------------------------------------------------------------------------------\r\n");
}

bool sendToMobile(AckReturnToMobile returnAckMobile) {
  dji_sdk::SendMobileData mobile_data;
  mobile_data.request.data.resize(sizeof(AckReturnToMobile));
  memcpy(&mobile_data.request.data[0], (uint8_t *)(&returnAckMobile), sizeof(AckReturnToMobile));
  mobile_data_service.call(mobile_data);
  return mobile_data.response.result;
}

void fromMobileDataSubscriberCallback(const dji_sdk::MobileData::ConstPtr& from_mobile_data) {
  mobile_data = *from_mobile_data;
  AckReturnToMobile ackReturnAckMobileSend;
  memcpy(&ackReturnAckMobileSend, &mobile_data.data[0], sizeof(AckReturnToMobile));
  uint8_t wayptPolygonSides;
  int     hotptInitRadius;
  int     responseTimeout = 1;

  int cmdID = ackReturnAckMobileSend.cmdID;
  switch (cmdID) {
    case 2: {
      /* request control authority */
      ServiceAck service_ack;
      service_ack = obtainCtrlAuthority();
      ackReturnAckMobileSend.cmdID = cmdID;
      ackReturnAckMobileSend.ack = service_ack.ack_data;
      sendToMobile(ackReturnAckMobileSend);
      if (service_ack.result) {
        ROS_INFO("Obtain SDK control Authority successfully");
      } else {
        ROS_WARN("Failed Obtain SDK control Authority");
      }
    }
      break;
    case 3: {
      /* release control authority */
      ServiceAck service_ack;
      service_ack = releaseCtrlAuthority();
      ackReturnAckMobileSend.cmdID = cmdID;
      ackReturnAckMobileSend.ack = service_ack.ack_data;
      sendToMobile(ackReturnAckMobileSend);
      if (service_ack.result) {
        ROS_INFO("Release SDK control Authority successfully");
      } else {
        ROS_WARN("Failed release SDK control Authority");
      }
    }
      break;
    case 4: {
      /* drone activation */
      ServiceAck service_ack;
      service_ack = activate();
      ackReturnAckMobileSend.cmdID = cmdID;
      ackReturnAckMobileSend.ack = service_ack.ack_data;
      sendToMobile(ackReturnAckMobileSend);
      if (service_ack.result) {
        ROS_INFO("Activated successfully");
      } else {
        ROS_WARN("Failed activation");
      }
    }
      break;
    case 5: {
      /* arm motors */
      ServiceAck service_ack;
      service_ack = armMotors();
      ackReturnAckMobileSend.cmdID = cmdID;
      ackReturnAckMobileSend.ack = service_ack.ack_data;
      sendToMobile(ackReturnAckMobileSend);
      if (service_ack.result) {
        ROS_INFO("ArmMotors command sent successfully");
      } else {
        ROS_WARN("Failed sending armMotors command");
      }
    }
      break;
    case 6: {
      /* disarm motors */
      ServiceAck service_ack;
      service_ack = disArmMotors();
      ackReturnAckMobileSend.cmdID = cmdID;
      ackReturnAckMobileSend.ack = service_ack.ack_data;
      sendToMobile(ackReturnAckMobileSend);
      if (service_ack.result) {
        ROS_INFO("DisArmMotors command sent successfully");
      } else {
        ROS_WARN("Failed sending disArmMotors command");
      }
    }
    break;
    case 7: {
      /* take off */
      ServiceAck service_ack;
      service_ack = takeoff();
      ackReturnAckMobileSend.cmdID = cmdID;
      ackReturnAckMobileSend.ack = service_ack.ack_data;
      sendToMobile(ackReturnAckMobileSend);
      if (service_ack.result) {
        ROS_INFO("Takeoff command sent successfully");
      } else {
        ROS_WARN("Failed sending takeoff command");
      }
    }
    break;
    case 8: {
      /* land */
      ServiceAck service_ack;
      service_ack = land();
      ackReturnAckMobileSend.cmdID = cmdID;
      ackReturnAckMobileSend.ack = service_ack.ack_data;
      sendToMobile(ackReturnAckMobileSend);
      if (service_ack.result) {
        ROS_INFO("Land command sent successfully");
      } else {
        ROS_WARN("Failed sending land command");
      }
    }
    break;
    case 9: {
      /* go home */
      ServiceAck service_ack;
      service_ack = goHome();
      ackReturnAckMobileSend.cmdID = cmdID;
      ackReturnAckMobileSend.ack = service_ack.ack_data;
      sendToMobile(ackReturnAckMobileSend);
      if (service_ack.result) {
        ROS_INFO("GoHome command sent successfully");
      } else {
        ROS_WARN("Failed sending goHome command");
      }
    }
    break;
    case 62: {

      bool obtain_control_result = obtain_control();
      bool takeoff_result;

      if(is_M100())
      {
        ROS_INFO("M100 taking off!");
        takeoff_result = M100monitoredTakeoff();
      }
      else
      {
        ROS_INFO("A3/N3 taking off!");
        takeoff_result = monitoredTakeoff();
      }

      if(takeoff_result)
      {
        square_mission.reset();
        square_mission.start_gps_location = current_gps;
        square_mission.setTarget(0, 20, 3, 60);
        square_mission.state = 1;
        ROS_INFO("##### Start route %d ....", square_mission.state);
      }
    }
    break;
    case 65: {
      ROS_INFO("Waypoint mission sample executing");
      wayptPolygonSides = 6;
      runWaypointMission(wayptPolygonSides, responseTimeout);

    }
    break;
    case 66: {
      ROS_INFO("Hotpoint mission sample executing");
      hotptInitRadius = 10;
      runHotpointMission(hotptInitRadius, responseTimeout);
    }
    break;
  }
}

ServiceAck armMotors() {
    dji_sdk::DroneArmControl droneArmControl;
  droneArmControl.request.arm = 1;
  drone_arm_service.call(droneArmControl);
  if(!droneArmControl.response.result) {
    ROS_WARN("ack.info: set = %i id = %i", droneArmControl.response.cmd_set, droneArmControl.response.cmd_id);
    ROS_WARN("ack.data: %i", droneArmControl.response.ack_data);
  }
  return {droneArmControl.response.result, droneArmControl.response.cmd_set,
          droneArmControl.response.cmd_id, droneArmControl.response.ack_data};
}

ServiceAck disArmMotors() {
  dji_sdk::DroneArmControl droneArmControl;
  droneArmControl.request.arm = 0;
  drone_arm_service.call(droneArmControl);
  if(!droneArmControl.response.result) {
    ROS_WARN("ack.info: set = %i id = %i", droneArmControl.response.cmd_set, droneArmControl.response.cmd_id);
    ROS_WARN("ack.data: %i", droneArmControl.response.ack_data);
  }
  return {droneArmControl.response.result, droneArmControl.response.cmd_set,
          droneArmControl.response.cmd_id, droneArmControl.response.ack_data};
}

ServiceAck
obtainCtrlAuthority() {
  dji_sdk::SDKControlAuthority sdkAuthority;
  sdkAuthority.request.control_enable = 1;
  sdk_ctrl_authority_service.call(sdkAuthority);
  if(!sdkAuthority.response.result) {
    ROS_WARN("ack.info: set = %i id = %i", sdkAuthority.response.cmd_set, sdkAuthority.response.cmd_id);
    ROS_WARN("ack.data: %i", sdkAuthority.response.ack_data);
  }
  return {sdkAuthority.response.result, sdkAuthority.response.cmd_set,
          sdkAuthority.response.cmd_id, sdkAuthority.response.ack_data};
}

ServiceAck
releaseCtrlAuthority() {
  dji_sdk::SDKControlAuthority sdkAuthority;
  sdkAuthority.request.control_enable = 0;
  sdk_ctrl_authority_service.call(sdkAuthority);
  if(!sdkAuthority.response.result) {
    ROS_WARN("ack.info: set = %i id = %i", sdkAuthority.response.cmd_set, sdkAuthority.response.cmd_id);
    ROS_WARN("ack.data: %i", sdkAuthority.response.ack_data);
  }
  return {sdkAuthority.response.result, sdkAuthority.response.cmd_set,
          sdkAuthority.response.cmd_id, sdkAuthority.response.ack_data};
}

ServiceAck
takeoff() {
  dji_sdk::DroneTaskControl droneTaskControl;
  droneTaskControl.request.task = 4;
  drone_task_service.call(droneTaskControl);
  if(!droneTaskControl.response.result) {
    ROS_WARN("ack.info: set = %i id = %i", droneTaskControl.response.cmd_set, droneTaskControl.response.cmd_id);
    ROS_WARN("ack.data: %i", droneTaskControl.response.ack_data);
  }
  return {droneTaskControl.response.result, droneTaskControl.response.cmd_set,
          droneTaskControl.response.cmd_id, droneTaskControl.response.ack_data};
}

ServiceAck
goHome() {
  dji_sdk::DroneTaskControl droneTaskControl;
  droneTaskControl.request.task = 1;
  drone_task_service.call(droneTaskControl);
  if(!droneTaskControl.response.result) {
    ROS_WARN("ack.info: set = %i id = %i", droneTaskControl.response.cmd_set, droneTaskControl.response.cmd_id);
    ROS_WARN("ack.data: %i", droneTaskControl.response.ack_data);
  }
  return {droneTaskControl.response.result, droneTaskControl.response.cmd_set,
          droneTaskControl.response.cmd_id, droneTaskControl.response.ack_data};
}

ServiceAck
land() {
  dji_sdk::DroneTaskControl droneTaskControl;
  droneTaskControl.request.task = 6;
  drone_task_service.call(droneTaskControl);
  if(!droneTaskControl.response.result) {
    ROS_WARN("ack.info: set = %i id = %i", droneTaskControl.response.cmd_set, droneTaskControl.response.cmd_id);
    ROS_WARN("ack.data: %i", droneTaskControl.response.ack_data);
  }
  return {droneTaskControl.response.result, droneTaskControl.response.cmd_set,
          droneTaskControl.response.cmd_id, droneTaskControl.response.ack_data};
}

ServiceAck activate() {
  dji_sdk::Activation activation;
  drone_activation_service.call(activation);
  if(!activation.response.result) {
    ROS_WARN("ack.info: set = %i id = %i", activation.response.cmd_set, activation.response.cmd_id);
    ROS_WARN("ack.data: %i", activation.response.ack_data);
  }
  return {activation.response.result, activation.response.cmd_set,
          activation.response.cmd_id, activation.response.ack_data};
}

/*! Very simple calculation of local NED offset between two pairs of GPS
/coordinates. Accurate when distances are small.
!*/
void
localOffsetFromGpsOffset(geometry_msgs::Vector3&  deltaNed,
                         sensor_msgs::NavSatFix& target,
                         sensor_msgs::NavSatFix& origin)
{
  double deltaLon = target.longitude - origin.longitude;
  double deltaLat = target.latitude - origin.latitude;

  deltaNed.y = deltaLat * deg2rad * C_EARTH;
  deltaNed.x = deltaLon * deg2rad * C_EARTH * cos(deg2rad*target.latitude);
  deltaNed.z = target.altitude - origin.altitude;
}

geometry_msgs::Vector3 toEulerAngle(geometry_msgs::Quaternion quat)
{
  geometry_msgs::Vector3 ans;

  tf::Matrix3x3 R_FLU2ENU(tf::Quaternion(quat.x, quat.y, quat.z, quat.w));
  R_FLU2ENU.getRPY(ans.x, ans.y, ans.z);
  return ans;
}
void Mission::step()
{
  static int info_counter = 0;
  geometry_msgs::Vector3     localOffset;

  float speedFactor         = 2;
  float yawThresholdInDeg   = 2;

  float xCmd, yCmd, zCmd;

  localOffsetFromGpsOffset(localOffset, current_gps, start_gps_location);

  double xOffsetRemaining = target_offset_x - localOffset.x;
  double yOffsetRemaining = target_offset_y - localOffset.y;
  double zOffsetRemaining = target_offset_z - localOffset.z;

  double yawDesiredRad     = deg2rad * target_yaw;
  double yawThresholdInRad = deg2rad * yawThresholdInDeg;
  double yawInRad          = toEulerAngle(current_atti).z;

  info_counter++;
  if(info_counter > 25)
  {
    info_counter = 0;
    ROS_INFO("-----x=%f, y=%f, z=%f, yaw=%f ...", localOffset.x,localOffset.y, localOffset.z,yawInRad);
    ROS_INFO("+++++dx=%f, dy=%f, dz=%f, dyaw=%f ...", xOffsetRemaining,yOffsetRemaining, zOffsetRemaining,yawInRad - yawDesiredRad);
  }
  if (abs(xOffsetRemaining) >= speedFactor)
    xCmd = (xOffsetRemaining>0) ? speedFactor : -1 * speedFactor;
  else
    xCmd = xOffsetRemaining;

  if (abs(yOffsetRemaining) >= speedFactor)
    yCmd = (yOffsetRemaining>0) ? speedFactor : -1 * speedFactor;
  else
    yCmd = yOffsetRemaining;

  zCmd = start_gps_location.altitude + target_offset_z;


  /*!
   * @brief: if we already started breaking, keep break for 50 sample (1sec)
   *         and call it done, else we send normal command
   */

  if (break_counter > 50)
  {
    ROS_INFO("##### Route %d finished....", state);
    finished = true;
    return;
  }
  else if(break_counter > 0)
  {
    sensor_msgs::Joy controlVelYawRate;
    controlVelYawRate.axes.push_back(0);
    controlVelYawRate.axes.push_back(0);
    controlVelYawRate.axes.push_back(0);
    controlVelYawRate.axes.push_back(0);
    ctrlVelYawRatePub.publish(controlVelYawRate);
    break_counter++;
    return;
  }
  else //break_counter = 0, not in break stage
  {
    sensor_msgs::Joy controlPosYaw;
    controlPosYaw.axes.push_back(xCmd);
    controlPosYaw.axes.push_back(yCmd);
    controlPosYaw.axes.push_back(zCmd);
    controlPosYaw.axes.push_back(yawDesiredRad);
    ctrlPosYawPub.publish(controlPosYaw);
  }

  if (std::abs(xOffsetRemaining) < 0.5 &&
      std::abs(yOffsetRemaining) < 0.5 &&
      std::abs(zOffsetRemaining) < 0.5 &&
      std::abs(yawInRad - yawDesiredRad) < yawThresholdInRad)
  {
    //! 1. We are within bounds; start incrementing our in-bound counter
    inbound_counter ++;
  }
  else
  {
    if (inbound_counter != 0)
    {
      //! 2. Start incrementing an out-of-bounds counter
      outbound_counter ++;
    }
  }

  //! 3. Reset withinBoundsCounter if necessary
  if (outbound_counter > 10)
  {
    ROS_INFO("##### Route %d: out of bounds, reset....", state);
    inbound_counter  = 0;
    outbound_counter = 0;
  }

  if (inbound_counter > 50)
  {
    ROS_INFO("##### Route %d start break....", state);
    break_counter = 1;
  }

}

bool takeoff_land(int task)
{
  dji_sdk::DroneTaskControl droneTaskControl;

  droneTaskControl.request.task = task;

  drone_task_service.call(droneTaskControl);

  if(!droneTaskControl.response.result)
  {
    ROS_ERROR("takeoff_land fail");
    return false;
  }

  return true;
}

bool obtain_control()
{
  dji_sdk::SDKControlAuthority authority;
  authority.request.control_enable=1;
  sdk_ctrl_authority_service.call(authority);

  if(!authority.response.result)
  {
    ROS_ERROR("obtain control failed!");
    return false;
  }

  return true;
}

bool is_M100()
{
  dji_sdk::QueryDroneVersion query;
  query_version_service.call(query);

  if(query.response.version == DJISDK::DroneFirmwareVersion::M100_31)
  {
    return true;
  }

  return false;
}

void attitude_callback(const geometry_msgs::QuaternionStamped::ConstPtr& msg)
{
  current_atti = msg->quaternion;
}
void gps_callback(const sensor_msgs::NavSatFix::ConstPtr& msg)
{
  static ros::Time start_time = ros::Time::now();
  ros::Duration elapsed_time = ros::Time::now() - start_time;
  current_gps = *msg;

  // Down sampled to 50Hz loop
  if(elapsed_time > ros::Duration(0.02))
  {
    start_time = ros::Time::now();
    switch(square_mission.state)
    {
      case 0:
        break;

      case 1:
        if(!square_mission.finished)
        {
          square_mission.step();
        }
        else
        {
          square_mission.reset();
          square_mission.start_gps_location = current_gps;
          square_mission.setTarget(20, 0, 0, 0);
          square_mission.state = 2;
          ROS_INFO("##### Start route %d ....", square_mission.state);
        }
        break;

      case 2:
        if(!square_mission.finished)
        {
          square_mission.step();
        }
        else
        {
          square_mission.reset();
          square_mission.start_gps_location = current_gps;
          square_mission.setTarget(0, -20, 0, 0);
          square_mission.state = 3;
          ROS_INFO("##### Start route %d ....", square_mission.state);
        }
        break;
      case 3:
        if(!square_mission.finished)
        {
          square_mission.step();
        }
        else
        {
          square_mission.reset();
          square_mission.start_gps_location = current_gps;
          square_mission.setTarget(-20, 0, 0, 0);
          square_mission.state = 4;
          ROS_INFO("##### Start route %d ....", square_mission.state);
        }
        break;
      case 4:
        if(!square_mission.finished)
        {
          square_mission.step();
        }
        else
        {
          ROS_INFO("##### Mission %d Finished ....", square_mission.state);
          square_mission.state = 0;
        }
        break;
    }
  }
}

void flight_status_callback(const std_msgs::UInt8::ConstPtr& msg)
{
  flight_status = msg->data;
}

void display_mode_callback(const std_msgs::UInt8::ConstPtr& msg)
{
  display_mode = msg->data;
}
bool
monitoredTakeoff()
{
  ros::Time start_time = ros::Time::now();

  if(!takeoff_land(dji_sdk::DroneTaskControl::Request::TASK_TAKEOFF)) {
    return false;
  }

  ros::Duration(0.01).sleep();
  ros::spinOnce();

  // Step 1.1: Spin the motor
  while (flight_status != DJISDK::FlightStatus::STATUS_ON_GROUND &&
         display_mode != DJISDK::DisplayMode::MODE_ENGINE_START &&
         ros::Time::now() - start_time < ros::Duration(5)) {
    ros::Duration(0.01).sleep();
    ros::spinOnce();
  }

  if(ros::Time::now() - start_time > ros::Duration(5)) {
    ROS_ERROR("Takeoff failed. Motors are not spinnning.");
    return false;
  }
  else {
    start_time = ros::Time::now();
    ROS_INFO("Motor Spinning ...");
    ros::spinOnce();
  }


  // Step 1.2: Get in to the air
  while (flight_status != DJISDK::FlightStatus::STATUS_IN_AIR &&
         (display_mode != DJISDK::DisplayMode::MODE_ASSISTED_TAKEOFF || display_mode != DJISDK::DisplayMode::MODE_AUTO_TAKEOFF) &&
         ros::Time::now() - start_time < ros::Duration(20)) {
    ros::Duration(0.01).sleep();
    ros::spinOnce();
  }

  if(ros::Time::now() - start_time > ros::Duration(20)) {
    ROS_ERROR("Takeoff failed. Aircraft is still on the ground, but the motors are spinning.");
    return false;
  }
  else {
    start_time = ros::Time::now();
    ROS_INFO("Ascending...");
    ros::spinOnce();
  }

  // Final check: Finished takeoff
  while ( (display_mode == DJISDK::DisplayMode::MODE_ASSISTED_TAKEOFF || display_mode == DJISDK::DisplayMode::MODE_AUTO_TAKEOFF) &&
          ros::Time::now() - start_time < ros::Duration(20)) {
    ros::Duration(0.01).sleep();
    ros::spinOnce();
  }

  if ( display_mode != DJISDK::DisplayMode::MODE_P_GPS || display_mode != DJISDK::DisplayMode::MODE_ATTITUDE)
  {
    ROS_INFO("Successful takeoff!");
    start_time = ros::Time::now();
  }
  else
  {
    ROS_ERROR("Takeoff finished, but the aircraft is in an unexpected mode. Please connect DJI GO.");
    return false;
  }

  return true;
}


bool
M100monitoredTakeoff()
{
  ros::Time start_time = ros::Time::now();

  float home_altitude = current_gps.altitude;
  if(!takeoff_land(dji_sdk::DroneTaskControl::Request::TASK_TAKEOFF))
  {
    return false;
  }

  ros::Duration(0.01).sleep();
  ros::spinOnce();

  // Step 1: If M100 is not in the air after 10 seconds, fail.
  while (ros::Time::now() - start_time < ros::Duration(10))
  {
    ros::Duration(0.01).sleep();
    ros::spinOnce();
  }

  if(flight_status != DJISDK::M100FlightStatus::M100_STATUS_IN_AIR ||
     current_gps.altitude - home_altitude < 1.0)
  {
    ROS_ERROR("Takeoff failed.");
    return false;
  }
  else
  {
    start_time = ros::Time::now();
    ROS_INFO("Successful takeoff!");
    ros::spinOnce();
  }

  return true;
}

bool
runWaypointMission(uint8_t numWaypoints, int responseTimeout)
{
  ros::spinOnce();

  // Waypoint Mission : Initialization
  dji_sdk::MissionWaypointTask waypointTask;
  setWaypointInitDefaults(waypointTask);

  // Waypoint Mission: Create Waypoints
  float64_t      increment = 0.000001/C_PI*180;
  float32_t      start_alt = 10;
  ROS_INFO("Creating Waypoints..\n");
  std::vector<WayPointSettings> generatedWaypts =
    createWaypoints(numWaypoints, increment, start_alt);

  // Waypoint Mission: Upload the waypoints
  ROS_INFO("Uploading Waypoints..\n");
  uploadWaypoints(generatedWaypts, responseTimeout, waypointTask);

  // Waypoint Mission: Init mission
  ROS_INFO("Initializing Waypoint Mission..\n");
  if (initWaypointMission(waypointTask).result) {
    ROS_INFO("Waypoint upload command sent successfully");
  } else {
    ROS_WARN("Failed sending waypoint upload command");
    return false;
  }

  // Waypoint Mission: Start
  if (missionAction(DJI::OSDK::DJI_MISSION_TYPE::WAYPOINT,
                    DJI::OSDK::MISSION_ACTION::START).result) {
    ROS_INFO("Mission start command sent successfully");
  } else {
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
setWaypointInitDefaults(dji_sdk::MissionWaypointTask &waypointTask)
{
  waypointTask.velocity_range    = 10;
  waypointTask.idle_velocity   = 5;
  waypointTask.action_on_finish   = 0;
  waypointTask.mission_exec_times = 1;
  waypointTask.yaw_mode        = 0;
  waypointTask.trace_mode      = 0;
  waypointTask.action_on_rc_lost   = 1;
  waypointTask.gimbal_pitch_mode    = 0;
}

std::vector<DJI::OSDK::WayPointSettings>
createWaypoints(int numWaypoints,
                float64_t distanceIncrement,
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
generateWaypointsPolygon(WayPointSettings* start_data,
                         float64_t increment,
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
                int responseTimeout,
                dji_sdk::MissionWaypointTask &waypointTask)
{
  dji_sdk::MissionWaypoint waypoint;
  for (std::vector<WayPointSettings>::iterator wp = wp_list.begin();
       wp != wp_list.end(); ++wp)
  {
    ROS_INFO("Waypoint created at (LLA): %f \t%f \t%f\n ", wp->latitude,
             wp->longitude, wp->altitude);
    waypoint.latitude = wp->latitude;
    waypoint.longitude = wp->longitude;
    waypoint.altitude = wp->altitude;
    waypoint.damping_distance = 0;
    waypoint.target_yaw = 0;
    waypoint.target_gimbal_pitch = 0;
    waypoint.turn_mode = 0;
    waypoint.has_action = 0;
    waypointTask.mission_waypoint.push_back(waypoint);
  }
}

bool
runHotpointMission(int initialRadius,
                   int responseTimeout)
{
  ros::spinOnce();

  // Hotpoint Mission: Create hotpoint
  dji_sdk::MissionHotpointTask hotpointTask;
  setHotpointInitDefault(hotpointTask);

  // Hotpoint Mission: Initialize
  initHotpointMission(hotpointTask);

  // Takeoff
  if (takeoff().result) {
    ROS_INFO("Takeoff command sent successfully");
  } else {
    ROS_WARN("Failed sending takeoff command");
    return false;
  }
  ros::Duration(15).sleep();

  // Start
  ROS_INFO("Start with default rotation rate: 15 deg/s");
  if (missionAction(DJI::OSDK::DJI_MISSION_TYPE::HOTPOINT,
                    DJI::OSDK::MISSION_ACTION::START).result) {
    ROS_INFO("Mission start command sent successfully");
  } else {
    ROS_WARN("Failed sending mission start command");
    return false;
  }
  ros::Duration(25).sleep();

  // Pause
  ROS_INFO("Pause for 5s");
  if (missionAction(DJI::OSDK::DJI_MISSION_TYPE::HOTPOINT,
                    DJI::OSDK::MISSION_ACTION::PAUSE).result) {
    ROS_INFO("Mission pause command sent successfully");
  } else {
    ROS_WARN("Failed sending mission pause command");
    return false;
  }
  ros::Duration(5).sleep();

  // Resume
  ROS_INFO("Resume");
  if (missionAction(DJI::OSDK::DJI_MISSION_TYPE::HOTPOINT,
                    DJI::OSDK::MISSION_ACTION::RESUME).result) {
    ROS_INFO("Mission resume command sent successfully");
  } else {
    ROS_WARN("Failed sending mission resume command");
    return false;
  }
  ros::Duration(10).sleep();

  // Update radius, no ACK
  ROS_INFO("Update radius to 1.5x: new radius = %f", 1.5*initialRadius);
  if (hotpointUpdateRadius(1.5*initialRadius).result) {
    ROS_INFO("Hotpoint update radius command sent successfully");
  } else {
    ROS_WARN("Failed sending hotpoint update radius command");
    return false;
  }
  ros::Duration(10).sleep();

  // Update velocity (yawRate), no ACK
  ROS_INFO("Update hotpoint rotation rate: new rate = 5 deg/s");
  if (hotpointUpdateYawRate(5, 1).result) {
    ROS_INFO("Hotpoint update yaw rate command sent successfully");
  } else {
    ROS_WARN("Failed sending hotpoint update yaw rate command");
    return false;
  }
  ros::Duration(10).sleep();

  // Stop
  ROS_INFO("Stop");
  if (missionAction(DJI::OSDK::DJI_MISSION_TYPE::HOTPOINT,
                    DJI::OSDK::MISSION_ACTION::STOP).result) {
    ROS_INFO("Mission stop command sent successfully");
  } else {
    ROS_WARN("Failed sending mission stop command");
    return false;
  }

  ROS_INFO("land");
  if (land().result) {
    ROS_INFO("Land command sent successfully");
  } else {
    ROS_WARN("Failed sending land command");
    return false;
  }

  return true;
}

void
setHotpointInitDefault(dji_sdk::MissionHotpointTask &hotpointTask)
{
  hotpointTask.latitude = gps_pos.latitude;
  hotpointTask.longitude = gps_pos.longitude;
  hotpointTask.altitude = 20;
  hotpointTask.radius = 10;
  hotpointTask.angular_speed = 15;
  hotpointTask.is_clockwise = 0;
  hotpointTask.start_point = 0;
  hotpointTask.yaw_mode = 0;
}

ServiceAck
initWaypointMission(dji_sdk::MissionWaypointTask &waypointTask)
{
  dji_sdk::MissionWpUpload missionWpUpload;
  missionWpUpload.request.waypoint_task = waypointTask;
  waypoint_upload_service.call(missionWpUpload);
  if(!missionWpUpload.response.result) {
    ROS_WARN("ack.info: set = %i id = %i", missionWpUpload.response.cmd_set, missionWpUpload.response.cmd_id);
    ROS_WARN("ack.data: %i", missionWpUpload.response.ack_data);
  }
  return {missionWpUpload.response.result, missionWpUpload.response.cmd_set,
          missionWpUpload.response.cmd_id, missionWpUpload.response.ack_data};
}

ServiceAck
initHotpointMission(dji_sdk::MissionHotpointTask &hotpointTask)
{
  dji_sdk::MissionHpUpload missionHpUpload;
  missionHpUpload.request.hotpoint_task = hotpointTask;
  hotpoint_upload_service.call(missionHpUpload);
  return {missionHpUpload.response.result, missionHpUpload.response.cmd_set,
          missionHpUpload.response.cmd_id, missionHpUpload.response.ack_data};
}

ServiceAck
missionAction(DJI::OSDK::DJI_MISSION_TYPE type,
              DJI::OSDK::MISSION_ACTION action)
{
  dji_sdk::MissionWpAction missionWpAction;
  dji_sdk::MissionHpAction missionHpAction;
  switch(type) {
    case DJI::OSDK::WAYPOINT:
      missionWpAction.request.action = action;
      waypoint_action_service.call(missionWpAction);
      if(!missionWpAction.response.result) {
        ROS_WARN("ack.info: set = %i id = %i", missionWpAction.response.cmd_set, missionWpAction.response.cmd_id);
        ROS_WARN("ack.data: %i", missionWpAction.response.ack_data);
      }
      return {missionWpAction.response.result, missionWpAction.response.cmd_set,
              missionWpAction.response.cmd_id, missionWpAction.response.ack_data};
    case DJI::OSDK::HOTPOINT:
      missionHpAction.request.action = action;
      hotpoint_action_service.call(missionHpAction);
      if(!missionHpAction.response.result) {
        ROS_WARN("ack.info: set = %i id = %i", missionHpAction.response.cmd_set, missionHpAction.response.cmd_id);
        ROS_WARN("ack.data: %i", missionHpAction.response.ack_data);
      }
      return {missionHpAction.response.result, missionHpAction.response.cmd_set,
              missionHpAction.response.cmd_id, missionHpAction.response.ack_data};
  }
}

ServiceAck
hotpointUpdateRadius(float radius)
{
  dji_sdk::MissionHpUpdateRadius missionHpUpdateRadius;
  missionHpUpdateRadius.request.radius = radius;
  hotpoint_updateRadius_service.call(missionHpUpdateRadius);
  if(!missionHpUpdateRadius.response.result) {
    ROS_WARN("ack.info: set = %i id = %i", missionHpUpdateRadius.response.cmd_set, missionHpUpdateRadius.response.cmd_id);
    ROS_WARN("ack.data: %i", missionHpUpdateRadius.response.ack_data);
  }
  return {missionHpUpdateRadius.response.result, missionHpUpdateRadius.response.cmd_set,
          missionHpUpdateRadius.response.cmd_id, missionHpUpdateRadius.response.ack_data};
}

ServiceAck
hotpointUpdateYawRate(float yawRate, int direction)
{
  dji_sdk::MissionHpUpdateYawRate missionHpUpdateYawRate;
  missionHpUpdateYawRate.request.yaw_rate = yawRate;
  missionHpUpdateYawRate.request.direction = direction;
  hotpoint_update_yawRate_Service.call(missionHpUpdateYawRate);
  if(!missionHpUpdateYawRate.response.result) {
    ROS_WARN("ack.info: set = %i id = %i", missionHpUpdateYawRate.response.cmd_set, missionHpUpdateYawRate.response.cmd_id);
    ROS_WARN("ack.data: %i", missionHpUpdateYawRate.response.ack_data);
  }
  return {missionHpUpdateYawRate.response.result, missionHpUpdateYawRate.response.cmd_set,
          missionHpUpdateYawRate.response.cmd_id, missionHpUpdateYawRate.response.ack_data};
}

void
gpsPosCallback(const sensor_msgs::NavSatFix::ConstPtr& msg)
{
  gps_pos = *msg;
}

