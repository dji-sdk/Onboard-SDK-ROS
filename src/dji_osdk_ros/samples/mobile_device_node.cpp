/** @file mobile_device_node.cpp
 *  @version 4.0
 *  @date May 2020
 *
 *  @brief sample node of mobile device.
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
#include "dji_osdk_ros/mobile_device_node.h"

//CODE

bool sendToMobile(AckReturnToMobile returnAckMobile)
{
  dji_osdk_ros::SendMobileData mobileData;
  mobileData.request.data.resize(sizeof(AckReturnToMobile));
  memcpy(&mobileData.request.data[0], (uint8_t *)(&returnAckMobile), sizeof(AckReturnToMobile));
  send_to_mobile_data_client.call(mobileData);
  return mobileData.response.result;
}

void fromMobileDataSubCallback(const dji_osdk_ros::MobileData::ConstPtr& fromMobileData)
{
  dji_osdk_ros::MobileData fromMobileInfo = *fromMobileData;
  AckReturnToMobile ackReturnToMobile;
  memcpy(&ackReturnToMobile, &fromMobileInfo.data[0], sizeof(AckReturnToMobile));
  ROS_INFO("fromMobileData:%d",ackReturnToMobile.cmdID);

  int cmd = ackReturnToMobile.cmdID;
  switch (cmd)
  {
    case 1:
    {
       ackReturnToMobile.ackResult = takeoff();
       sendToMobile(ackReturnToMobile);
       break;
    }
    case 2:
    {
      ackReturnToMobile.ackResult = land();
      sendToMobile(ackReturnToMobile);
      break;
    }
    case 3:
    {
      ackReturnToMobile.ackResult = gohome();
      sendToMobile(ackReturnToMobile);
      break;
    }
    case 4:
    {
      ackReturnToMobile.ackResult = gohomeAndConfirmLanding();
      sendToMobile(ackReturnToMobile);
      break;
    }
    case 5:
    {
      ackReturnToMobile.ackResult = localPositionCtrl();
      sendToMobile(ackReturnToMobile);
      break;
    }
    case 6:
    {
      ackReturnToMobile.ackResult = resetGimbal();
      sendToMobile(ackReturnToMobile);
      break;
    }
    case 7:
    {
      ackReturnToMobile.ackResult = Rotategimbal();
      sendToMobile(ackReturnToMobile);
      break;
    }

    case 8:
    {
      ackReturnToMobile.ackResult = takePicture();
      sendToMobile(ackReturnToMobile);
      break;
    }

    case 9:
    {
      ackReturnToMobile.ackResult = recordVideo();
      sendToMobile(ackReturnToMobile);
    }

    case 256:
    {
      ackReturnToMobile.ackResult = obtainJoystickControlAuthority();
    }
      break;
  }
  ros::Duration(1.0).sleep();
}

bool takeoff()
{
  FlightTaskControl flightTaskControl;
  flightTaskControl.request.task = dji_osdk_ros::FlightTaskControl::Request ::TASK_TAKEOFF;
  flight_control_client.call(flightTaskControl);

  return flightTaskControl.response.result;
}

bool land()
{
  FlightTaskControl flightTaskControl;
  flightTaskControl.request.task = dji_osdk_ros::FlightTaskControl::Request ::TASK_LAND;
  flight_control_client.call(flightTaskControl);

  return flightTaskControl.response.result;
}

bool gohome()
{
  FlightTaskControl flightTaskControl;
  flightTaskControl.request.task = dji_osdk_ros::FlightTaskControl::Request ::TASK_GOHOME;
  flight_control_client.call(flightTaskControl);

  return flightTaskControl.response.result;
}

bool gohomeAndConfirmLanding()
{
  FlightTaskControl flightTaskControl;
  flightTaskControl.request.task = dji_osdk_ros::FlightTaskControl::Request ::TASK_GOHOME_AND_CONFIRM_LANDING;
  flight_control_client.call(flightTaskControl);

  return flightTaskControl.response.result;
}


bool moveByPosOffset(const JoystickCommand &offsetDesired,
                     float posThresholdInM,
                     float yawThresholdInDeg)
{
  FlightTaskControl flightTaskControl;
  flightTaskControl.request.task = FlightTaskControl::Request::TASK_POSITION_AND_YAW_CONTROL;
  flightTaskControl.request.joystickCommand.x = offsetDesired.x;
  flightTaskControl.request.joystickCommand.y = offsetDesired.y;
  flightTaskControl.request.joystickCommand.z = offsetDesired.z;
  flightTaskControl.request.joystickCommand.yaw = offsetDesired.yaw;
  flightTaskControl.request.posThresholdInM   = posThresholdInM;
  flightTaskControl.request.yawThresholdInDeg = yawThresholdInDeg;

  flight_control_client.call(flightTaskControl);
  return flightTaskControl.response.result;
}

bool localPositionCtrl()
{
  ROS_INFO_STREAM("Move by position offset request sending ...");
  moveByPosOffset({0.0, 6.0, 6.0, 30.0}, 0.8, 1);
  ROS_INFO_STREAM("Step 1 over!");
  moveByPosOffset({6.0, 0.0, -3, -30.0}, 0.8, 1);
  ROS_INFO_STREAM("Step 2 over!");
  moveByPosOffset({-6.0, -6.0, 0.0, 0.0}, 0.8, 1);
  ROS_INFO_STREAM("Step 3 over!");

  return true;
}

bool resetGimbal()
{
  GimbalAction gimbalAction;
  gimbalAction.request.is_reset = true;
  gimbalAction.request.payload_index = static_cast<uint8_t>(dji_osdk_ros::PayloadIndex::PAYLOAD_INDEX_0);
  gimbal_control_client.call(gimbalAction);

  return gimbalAction.response.result;
}

bool Rotategimbal()
{
  GimbalAction gimbalAction;
  gimbalAction.request.is_reset = false;
  gimbalAction.request.payload_index = static_cast<uint8_t>(dji_osdk_ros::PayloadIndex::PAYLOAD_INDEX_0);
  gimbalAction.request.rotationMode = 0;
  gimbalAction.request.pitch = 25.0f;
  gimbalAction.request.roll = 0.0f;
  gimbalAction.request.yaw = 90.0f;
  gimbalAction.request.time = 0.5;
  gimbal_control_client.call(gimbalAction);

  return gimbalAction.response.result;
}

bool takePicture()
{
  CameraStartShootSinglePhoto cameraStartShootSinglePhoto;
  cameraStartShootSinglePhoto.request.payload_index = static_cast<uint8_t>(dji_osdk_ros::PayloadIndex::PAYLOAD_INDEX_0);
  camera_start_shoot_single_photo_client.call(cameraStartShootSinglePhoto);
}

bool recordVideo()
{
  CameraRecordVideoAction cameraRecordVideoAction;
  cameraRecordVideoAction.request.start_stop = 1;
  cameraRecordVideoAction.request.payload_index = static_cast<uint8_t>(dji_osdk_ros::PayloadIndex::PAYLOAD_INDEX_0);
  camera_record_video_action_client.call(cameraRecordVideoAction);
  sleep(10);
  cameraRecordVideoAction.request.start_stop = 0;
  camera_record_video_action_client.call(cameraRecordVideoAction);
}

bool obtainJoystickControlAuthority()
{
  ObtainControlAuthority obtainJoystickControlAuthority;
  obtainJoystickControlAuthority.request.enable_obtain = true;
  obtain_ctrl_authority_client.call(obtainJoystickControlAuthority);

  return obtainJoystickControlAuthority.response.result;
}

static void DisplayMainMenu(void)
{
  printf("\r\n");
  printf("+------------------------------ < Main menu > ---------------------------------+\n");
  printf("| [1]  Enter Mobile Communication Mode \n");
  printf("| [2]  Exit Demo");
  printf("\r\n-------------------------------------------------------------------------------\r\n");
}

int main(int argc, char *argv[]) {
  int main_operate_code = 0;
  int temp32;
  bool userExitCommand = false;
  ros::init(argc, argv, "mobile_device_node");
  ROS_INFO("mobile device node is running!");
  ros::NodeHandle nh;

  obtain_ctrl_authority_client = nh.serviceClient<dji_osdk_ros::ObtainControlAuthority>("obtain_release_control_authority");
  send_to_mobile_data_client = nh.serviceClient<SendMobileData>("send_data_to_mobile_device");
  flight_control_client = nh.serviceClient<FlightTaskControl>("flight_task_control");
  gimbal_control_client = nh.serviceClient<GimbalAction>("gimbal_task_control");
  camera_start_shoot_single_photo_client = nh.serviceClient<CameraStartShootSinglePhoto>(
      "camera_start_shoot_single_photo");
  camera_record_video_action_client = nh.serviceClient<CameraRecordVideoAction>("camera_record_video_action");

  fromMobileDataSub = nh.subscribe("dji_osdk_ros/from_mobile_data", 10, &fromMobileDataSubCallback);

  DisplayMainMenu();
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
      DisplayMainMenu();
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
    DisplayMainMenu();
  }
  return 0;
}
