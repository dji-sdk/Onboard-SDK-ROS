/**
  * @Copyright (C) 2020 All rights reserved.
  * @date: 2020
  * @file: dji_vehicle_node.cc
  * @version: v0.0.1
  * @author: kevin.hoo@dji.com
  * @create_date: 2020-03-08 16:08:55
  * @last_modified_date: 2020-03-10 11:42:43
  * @brief: TODO
  * @details: TODO
  */

//INCLUDE
#include <dji_osdk_ros/dji_vehicle_node.hh>
#include <dji_osdk_ros/vehicle_wrapper.hh>
//CODE
using namespace dji_osdk_ros;
const int WAIT_TIMEOUT = 10;

VehicleNode::VehicleNode()
{

  initService();
}

VehicleNode::VehicleNode(int test)
{
  nh_.param("app_id",        app_id_,    123456);
  nh_.param("enc_key",       enc_key_, std::string("abcd1234"));
  nh_.param("acm_name",      device_acm_, std::string("/dev/ttyACM0"));
  nh_.param("serial_name",   device_, std::string("/dev/ttyUSB0"));
  nh_.param("baud_rate",     baud_rate_, 921600);
  nh_.param("app_version",   app_version_, 1);
  nh_.param("drone_version", drone_version_, std::string("M100")); // choose M100 as default
  nh_.param("gravity_const", gravity_const_, 9.801);
  ptr_wrapper_ = std::make_unique<VehicleWrapper>(app_id_, enc_key_, device_, baud_rate_);

  if(ptr_wrapper_ == nullptr)
  {
    ROS_ERROR_STREAM("Vehicle Wrapper inited failed");
    ros::shutdown();
  }
  ROS_INFO_STREAM("VehicleNode Start");

  initSubscribe();
  initService();
}

void VehicleNode::initService()
{
  task_control_server_ = nh_.advertiseService("drone_task_control", &VehicleNode::taskCtrlCallback, this);
  gimbal_control_server_ = nh_.advertiseService("gimbal_task_control", &VehicleNode::gimbalCtrlCallback, this);
  camera_action_control_server_ = nh_.advertiseService("camera_task_control", &VehicleNode::cameraCtrlCallback, this);
}

bool VehicleNode::taskCtrlCallback(DroneTaskControl::Request&  request, DroneTaskControl::Response& response)
{
  ROS_DEBUG("called droneTaskCallback");
  response.result = false;
  ACK::ErrorCode ack;
  if(ptr_wrapper_ == nullptr)
  {
    ROS_ERROR_STREAM("Vehicle Wrapper is nullptr");
    return true;
  }

  switch (request.task)
  {
    case DroneTaskControl::Request::TASK_GOHOME:
      {
        ROS_INFO_STREAM("call go home service");
        ack = ptr_wrapper_->getVehicle()->control->goHome(WAIT_TIMEOUT);
        break;
      }
    case DroneTaskControl::Request::TASK_GO_LOCAL_POS:
      {
        ROS_INFO_STREAM("call move local position service");
        if(request.pos_offset.size() < 3 && request.yaw_params.size() < 3)
        {
          ROS_INFO_STREAM("Local Position Service Params Error");
          break;
        }
        MoveOffset tmp_offset{request.pos_offset[0],
                             request.pos_offset[1],
                             request.pos_offset[2],
                             request.yaw_params[0],
                             request.yaw_params[1],
                             request.yaw_params[2]
                            };
        moveByPositionOffset(ack, WAIT_TIMEOUT, tmp_offset);
        break;
      }
    case DroneTaskControl::Request::TASK_TAKEOFF:
      {
        ROS_INFO_STREAM("call takeoff service");
        monitoredTakeoff(ack, WAIT_TIMEOUT);
        break;
      }
    case DroneTaskControl::Request::TASK_LAND:
      {
        ROS_INFO_STREAM("call land service");
        monitoredLanding(ack, WAIT_TIMEOUT);
        response.result = true;
        break;
      }
    default:
      {
        ROS_INFO_STREAM("No recognized task");
        response.result = false;
        break;
      }
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

bool VehicleNode::gimbalCtrlCallback(GimbalAction::Request& request, GimbalAction::Response& response)
{
  response.result = false;
  RotationAngle initial_angle;
  //if(getCurrentGimbal(initial_angle) == false)
  //{
  //  ROS_ERROR_STREAM("Get Current Gimbal Angle Failed");
  //  return false;
  //}
  GimbalContainer gimbal(request.yaw, request.pitch, request.roll, 0, 1, initial_angle);
  response.result = setGimbalAngle(gimbal);
  return true;
}

bool VehicleNode::cameraCtrlCallback(CameraAction::Request& request, CameraAction::Response& response)
{
  response.result = false;
  switch (request.action)
  {
    case CameraAction::Request::CAMERA_ACTION_TAKE_PICTURE:
      {
        ROS_INFO_STREAM("Call take picture.");
        response.result = takePicture();
        break;
      }
    case CameraAction::Request::CAMERA_ACTION_START_RECORD:
      {
        ROS_INFO_STREAM("Call record video.");
        response.result = startCaptureVideo();
        break;
      }
    case CameraAction::Request::CAMERA_ACTION_STOP_RECORD:
      {
        ROS_INFO_STREAM("Call stop video.");
        response.result = stopCaptureVideo();
        break;
      }
    defaule:
      response.result = false;
      break;
  }
  //return response.result;
  return true;
}


bool VehicleNode::takePicture()
{
  if(ptr_wrapper_ == nullptr)
  {
    ROS_ERROR_STREAM("VehicleWrapper not initialized");
    return false;
  }
  // Take picture
  std::cout << "Ensure SD card is present.\n";
  std::cout << "Taking picture..\n";
  ptr_wrapper_->getVehicle()->camera->shootPhoto();
  std::cout << "Check DJI GO App or SD card for a new picture.\n";

  std::cout << "Setting new Gimbal rotation angle to [0,-50, 0] using absolute "
               "control:\n";
  return true;
}

bool VehicleNode::startCaptureVideo()
{
  if(ptr_wrapper_ == nullptr)
  {
    ROS_ERROR_STREAM("VehicleWrapper not initialized");
    return false;
  }
  std::cout << "Ensure SD card is present.\n";
  std::cout << "Starting video..\n";
  ptr_wrapper_->getVehicle()->camera->videoStart();
  return true;
}

bool VehicleNode::stopCaptureVideo()
{
  if(ptr_wrapper_ == nullptr)
  {
    ROS_ERROR_STREAM("VehicleWrapper not initialized");
    return false;
  }
    // Stop the video
  std::cout << "Stopping video...\n";
  ptr_wrapper_->getVehicle()->camera->videoStop();
  std::cout << "Check DJI GO App or SD card for a new video.\n";
  return true;
}

bool VehicleNode::getCurrentGimbal(RotationAngle& initial_angle)
{
  if(ptr_wrapper_ == nullptr)
  {
    return false;
  }
  // Get Gimbal initial values
  if (!ptr_wrapper_->getVehicle()->isM100() && !ptr_wrapper_->getVehicle()->isLegacyM600())
  {
    initial_angle.roll  = ptr_wrapper_->getVehicle()->subscribe->getValue<TOPIC_GIMBAL_ANGLES>().y;
    initial_angle.pitch = ptr_wrapper_->getVehicle()->subscribe->getValue<TOPIC_GIMBAL_ANGLES>().x;
    initial_angle.yaw   = ptr_wrapper_->getVehicle()->subscribe->getValue<TOPIC_GIMBAL_ANGLES>().z;
  }
  else
  {
    initial_angle.roll  = ptr_wrapper_->getVehicle()->broadcast->getGimbal().roll;
    initial_angle.pitch = ptr_wrapper_->getVehicle()->broadcast->getGimbal().pitch;
    initial_angle.yaw   = ptr_wrapper_->getVehicle()->broadcast->getGimbal().yaw;
  }
  return true;
}

bool VehicleNode::initSubscribe()
{
  int responseTimeout = 1;
  int pkgIndex;

  /*
   * Subscribe to gimbal data not supported in MAtrice 100
   */

  if (!ptr_wrapper_->getVehicle()->isM100() && !ptr_wrapper_->getVehicle()->isLegacyM600())
  {
    // Telemetry: Verify the subscription
    ACK::ErrorCode subscribeStatus;
    subscribeStatus = ptr_wrapper_->getVehicle()->subscribe->verify(responseTimeout);
    if (ACK::getError(subscribeStatus) != ACK::SUCCESS)
    {
      ACK::getErrorCodeMessage(subscribeStatus, __func__);
      return false;
    }

    // Telemetry: Subscribe to gimbal status and gimbal angle at freq 10 Hz
    pkgIndex                  = 0;
    int       freq            = 10;
    TopicName topicList10Hz[] = { TOPIC_GIMBAL_ANGLES, TOPIC_GIMBAL_STATUS };
    int       numTopic = sizeof(topicList10Hz) / sizeof(topicList10Hz[0]);
    bool      enableTimestamp = false;

    bool pkgStatus = ptr_wrapper_->getVehicle()->subscribe->initPackageFromTopicList(
      pkgIndex, numTopic, topicList10Hz, enableTimestamp, freq);
    if (!(pkgStatus))
    {
      return pkgStatus;
    }
    subscribeStatus =
      ptr_wrapper_->getVehicle()->subscribe->startPackage(pkgIndex, responseTimeout);
    if (ACK::getError(subscribeStatus) != ACK::SUCCESS)
    {
      ACK::getErrorCodeMessage(subscribeStatus, __func__);
      // Cleanup before return
      ptr_wrapper_->getVehicle()->subscribe->removePackage(pkgIndex, responseTimeout);
      return false;
    }
  }

  sleep(1);

  std::cout
    << "Please note that the gimbal yaw angle you see in the telemetry is "
       "w.r.t absolute North"
       ", and the accuracy depends on your magnetometer calibration.\n\n"; 
  return true;
}

bool VehicleNode::monitoredTakeoff(ACK::ErrorCode& ack, int timeout)
{
  using namespace Telemetry;
  //@todo: remove this once the getErrorCode function signature changes
  char func[50];
  int  pkgIndex;

  if (!ptr_wrapper_->getVehicle()->isM100() && !ptr_wrapper_->getVehicle()->isLegacyM600())
  {
    // Telemetry: Verify the subscription
    ack = ptr_wrapper_->getVehicle()->subscribe->verify(timeout);
    if (ACK::getError(ack) != ACK::SUCCESS)
    {
      ACK::getErrorCodeMessage(ack, func);
      return false;
    }

    // Telemetry: Subscribe to flight status and mode at freq 10 Hz
    pkgIndex                  = 0;
    int       freq            = 10;
    TopicName topicList10Hz[] = { TOPIC_STATUS_FLIGHT,
                                  TOPIC_STATUS_DISPLAYMODE };
    int  numTopic        = sizeof(topicList10Hz) / sizeof(topicList10Hz[0]);
    bool enableTimestamp = false;

    bool pkgStatus = ptr_wrapper_->getVehicle()->subscribe->initPackageFromTopicList(
      pkgIndex, numTopic, topicList10Hz, enableTimestamp, freq);
    if (!(pkgStatus))
    {
      return pkgStatus;
    }
    ack = ptr_wrapper_->getVehicle()->subscribe->startPackage(pkgIndex, timeout);
    if (ACK::getError(ack) != ACK::SUCCESS)
    {
      ACK::getErrorCodeMessage(ack, func);
      // Cleanup before return
      ptr_wrapper_->getVehicle()->subscribe->removePackage(pkgIndex, timeout);
      return false;
    }
  }

  // Start takeoff
  ack = ptr_wrapper_->getVehicle()->control->takeoff(timeout);
  if (ACK::getError(ack) != ACK::SUCCESS)
  {
    ACK::getErrorCodeMessage(ack, func);
    return false;
  }

  // First check: Motors started
  int motorsNotStarted = 0;
  int timeoutCycles    = 20;

  if (!ptr_wrapper_->getVehicle()->isM100() && !ptr_wrapper_->getVehicle()->isLegacyM600())
  {
    while (ptr_wrapper_->getVehicle()->subscribe->getValue<TOPIC_STATUS_FLIGHT>() !=
             VehicleStatus::FlightStatus::ON_GROUND &&
           ptr_wrapper_->getVehicle()->subscribe->getValue<TOPIC_STATUS_DISPLAYMODE>() !=
             VehicleStatus::DisplayMode::MODE_ENGINE_START &&
           motorsNotStarted < timeoutCycles)
    {
      motorsNotStarted++;
      usleep(100000);
    }

    if (motorsNotStarted == timeoutCycles)
    {
      std::cout << "Takeoff failed. Motors are not spinning." << std::endl;
      // Cleanup
      if (!ptr_wrapper_->getVehicle()->isM100() && !ptr_wrapper_->getVehicle()->isLegacyM600())
      {
        ptr_wrapper_->getVehicle()->subscribe->removePackage(0, timeout);
      }
      return false;
    }
    else
    {
      std::cout << "Motors spinning...\n";
    }
  }
  else if (ptr_wrapper_->getVehicle()->isLegacyM600())
  {
    while ((ptr_wrapper_->getVehicle()->broadcast->getStatus().flight <
            DJI::OSDK::VehicleStatus::FlightStatus::ON_GROUND) &&
           motorsNotStarted < timeoutCycles)
    {
      motorsNotStarted++;
      usleep(100000);
    }

    if (motorsNotStarted < timeoutCycles)
    {
      std::cout << "Successful TakeOff!" << std::endl;
    }
  }
  else // M100
  {
    while ((ptr_wrapper_->getVehicle()->broadcast->getStatus().flight <
            DJI::OSDK::VehicleStatus::M100FlightStatus::TAKEOFF) &&
           motorsNotStarted < timeoutCycles)
    {
      motorsNotStarted++;
      usleep(100000);
    }

    if (motorsNotStarted < timeoutCycles)
    {
      std::cout << "Successful TakeOff!" << std::endl;
    }
  }

  // Second check: In air
  int stillOnGround = 0;
  timeoutCycles     = 110;

  if (!ptr_wrapper_->getVehicle()->isM100() && !ptr_wrapper_->getVehicle()->isLegacyM600())
  {
    while (ptr_wrapper_->getVehicle()->subscribe->getValue<TOPIC_STATUS_FLIGHT>() !=
             VehicleStatus::FlightStatus::IN_AIR &&
           (ptr_wrapper_->getVehicle()->subscribe->getValue<TOPIC_STATUS_DISPLAYMODE>() !=
              VehicleStatus::DisplayMode::MODE_ASSISTED_TAKEOFF ||
            ptr_wrapper_->getVehicle()->subscribe->getValue<TOPIC_STATUS_DISPLAYMODE>() !=
              VehicleStatus::DisplayMode::MODE_AUTO_TAKEOFF) &&
           stillOnGround < timeoutCycles)
    {
      stillOnGround++;
      usleep(100000);
    }

    if (stillOnGround == timeoutCycles)
    {
      std::cout << "Takeoff failed. Aircraft is still on the ground, but the "
                   "motors are spinning."
                << std::endl;
      // Cleanup
      if (!ptr_wrapper_->getVehicle()->isM100() && !ptr_wrapper_->getVehicle()->isLegacyM600())
      {
        ptr_wrapper_->getVehicle()->subscribe->removePackage(0, timeout);
      }
      return false;
    }
    else
    {
      std::cout << "Ascending...\n";
    }
  }
  else if (ptr_wrapper_->getVehicle()->isLegacyM600())
  {
    while ((ptr_wrapper_->getVehicle()->broadcast->getStatus().flight <
            DJI::OSDK::VehicleStatus::FlightStatus::IN_AIR) &&
           stillOnGround < timeoutCycles)
    {
      stillOnGround++;
      usleep(100000);
    }

    if (stillOnGround < timeoutCycles)
    {
      std::cout << "Aircraft in air!" << std::endl;
    }
  }
  else // M100
  {
    while ((ptr_wrapper_->getVehicle()->broadcast->getStatus().flight !=
            DJI::OSDK::VehicleStatus::M100FlightStatus::IN_AIR_STANDBY) &&
           stillOnGround < timeoutCycles)
    {
      stillOnGround++;
      usleep(100000);
    }

    if (stillOnGround < timeoutCycles)
    {
      std::cout << "Aircraft in air!" << std::endl;
    }
  }

  // Final check: Finished takeoff
  if (!ptr_wrapper_->getVehicle()->isM100() && !ptr_wrapper_->getVehicle()->isLegacyM600())
  {
    while (ptr_wrapper_->getVehicle()->subscribe->getValue<TOPIC_STATUS_DISPLAYMODE>() ==
             VehicleStatus::DisplayMode::MODE_ASSISTED_TAKEOFF ||
           ptr_wrapper_->getVehicle()->subscribe->getValue<TOPIC_STATUS_DISPLAYMODE>() ==
             VehicleStatus::DisplayMode::MODE_AUTO_TAKEOFF)
    {
      sleep(1);
    }

    if (!ptr_wrapper_->getVehicle()->isM100() && !ptr_wrapper_->getVehicle()->isLegacyM600())
    {
      if (ptr_wrapper_->getVehicle()->subscribe->getValue<TOPIC_STATUS_DISPLAYMODE>() !=
            VehicleStatus::DisplayMode::MODE_P_GPS ||
          ptr_wrapper_->getVehicle()->subscribe->getValue<TOPIC_STATUS_DISPLAYMODE>() !=
            VehicleStatus::DisplayMode::MODE_ATTITUDE)
      {
        std::cout << "Successful takeoff!\n";
      }
      else
      {
        std::cout
          << "Takeoff finished, but the aircraft is in an unexpected mode. "
             "Please connect DJI GO.\n";
        ptr_wrapper_->getVehicle()->subscribe->removePackage(0, timeout);
        return false;
      }
    }
  }
  else
  {
    float32_t                 delta;
    Telemetry::GlobalPosition currentHeight;
    Telemetry::GlobalPosition deltaHeight =
      ptr_wrapper_->getVehicle()->broadcast->getGlobalPosition();

    do
    {
      sleep(4);
      currentHeight = ptr_wrapper_->getVehicle()->broadcast->getGlobalPosition();
      delta         = fabs(currentHeight.altitude - deltaHeight.altitude);
      deltaHeight.altitude = currentHeight.altitude;
    } while (delta >= 0.009);

    std::cout << "Aircraft hovering at " << currentHeight.altitude << "m!\n";
  }

  // Cleanup
  if (!ptr_wrapper_->getVehicle()->isM100() && !ptr_wrapper_->getVehicle()->isLegacyM600())
  {
    ack = ptr_wrapper_->getVehicle()->subscribe->removePackage(pkgIndex, timeout);
    if (ACK::getError(ack))
    {
      std::cout
        << "Error unsubscribing; please restart the drone/FC to get back "
           "to a clean state.\n";
    }
  }

  return true;
}

bool VehicleNode::monitoredLanding(ACK::ErrorCode& ack, int timeout)
{
  using namespace Telemetry;
  //@todo: remove this once the getErrorCode function signature changes
  char func[50];
  int  pkgIndex;

  if (!ptr_wrapper_->getVehicle()->isM100() && !ptr_wrapper_->getVehicle()->isLegacyM600())
  {
    // Telemetry: Verify the subscription
    ack = ptr_wrapper_->getVehicle()->subscribe->verify(timeout);
    if (ACK::getError(ack) != ACK::SUCCESS)
    {
      ACK::getErrorCodeMessage(ack, func);
      return false;
    }

    // Telemetry: Subscribe to flight status and mode at freq 10 Hz
    pkgIndex                  = 0;
    int       freq            = 10;
    TopicName topicList10Hz[] = { TOPIC_STATUS_FLIGHT,
                                  TOPIC_STATUS_DISPLAYMODE };
    int  numTopic        = sizeof(topicList10Hz) / sizeof(topicList10Hz[0]);
    bool enableTimestamp = false;

    bool pkgStatus = ptr_wrapper_->getVehicle()->subscribe->initPackageFromTopicList(
      pkgIndex, numTopic, topicList10Hz, enableTimestamp, freq);
    if (!(pkgStatus))
    {
      return pkgStatus;
    }
    ack = ptr_wrapper_->getVehicle()->subscribe->startPackage(pkgIndex, timeout);
    if (ACK::getError(ack) != ACK::SUCCESS)
    {
      ACK::getErrorCodeMessage(ack, func);
      // Cleanup before return
      ptr_wrapper_->getVehicle()->subscribe->removePackage(pkgIndex, timeout);
      return false;
    }
  }

  // Start landing
  ack = ptr_wrapper_->getVehicle()->control->land(timeout);
  if (ACK::getError(ack) != ACK::SUCCESS)
  {
    ACK::getErrorCodeMessage(ack, func);
    return false;
  }

  // First check: Landing started
  int landingNotStarted = 0;
  int timeoutCycles     = 20;

  if (!ptr_wrapper_->getVehicle()->isM100() && !ptr_wrapper_->getVehicle()->isLegacyM600())
  {
    while (ptr_wrapper_->getVehicle()->subscribe->getValue<TOPIC_STATUS_DISPLAYMODE>() !=
             VehicleStatus::DisplayMode::MODE_AUTO_LANDING &&
           landingNotStarted < timeoutCycles)
    {
      landingNotStarted++;
      usleep(100000);
    }
  }
  else if (ptr_wrapper_->getVehicle()->isM100())
  {
    while (ptr_wrapper_->getVehicle()->broadcast->getStatus().flight !=
             DJI::OSDK::VehicleStatus::M100FlightStatus::LANDING &&
           landingNotStarted < timeoutCycles)
    {
      landingNotStarted++;
      usleep(100000);
    }
  }

  if (landingNotStarted == timeoutCycles)
  {
    std::cout << "Landing failed. Aircraft is still in the air." << std::endl;
    if (!ptr_wrapper_->getVehicle()->isM100() && !ptr_wrapper_->getVehicle()->isLegacyM600())
    {
      // Cleanup before return
      ack = ptr_wrapper_->getVehicle()->subscribe->removePackage(pkgIndex, timeout);
      if (ACK::getError(ack)) {
        std::cout << "Error unsubscribing; please restart the drone/FC to get "
                     "back to a clean state.\n";
      }
    }
    return false;
  }
  else
  {
    std::cout << "Landing...\n";
  }

  // Second check: Finished landing
  if (!ptr_wrapper_->getVehicle()->isM100() && !ptr_wrapper_->getVehicle()->isLegacyM600())
  {
    while (ptr_wrapper_->getVehicle()->subscribe->getValue<TOPIC_STATUS_DISPLAYMODE>() ==
             VehicleStatus::DisplayMode::MODE_AUTO_LANDING &&
           ptr_wrapper_->getVehicle()->subscribe->getValue<TOPIC_STATUS_FLIGHT>() ==
             VehicleStatus::FlightStatus::IN_AIR)
    {
      sleep(1);
    }

    if (ptr_wrapper_->getVehicle()->subscribe->getValue<TOPIC_STATUS_DISPLAYMODE>() !=
          VehicleStatus::DisplayMode::MODE_P_GPS ||
        ptr_wrapper_->getVehicle()->subscribe->getValue<TOPIC_STATUS_DISPLAYMODE>() !=
          VehicleStatus::DisplayMode::MODE_ATTITUDE)
    {
      std::cout << "Successful landing!\n";
    }
    else
    {
      std::cout
        << "Landing finished, but the aircraft is in an unexpected mode. "
           "Please connect DJI GO.\n";
      ack = ptr_wrapper_->getVehicle()->subscribe->removePackage(pkgIndex, timeout);
      if (ACK::getError(ack))
      {
        std::cout << "Error unsubscribing; please restart the drone/FC to get "
                     "back to a clean state.\n";
      }
      return false;
    }
  }
  else if (ptr_wrapper_->getVehicle()->isLegacyM600())
  {
    while (ptr_wrapper_->getVehicle()->broadcast->getStatus().flight >
           DJI::OSDK::VehicleStatus::FlightStatus::STOPED)
    {
      sleep(1);
    }

    Telemetry::GlobalPosition gp;
    do
    {
      sleep(2);
      gp = ptr_wrapper_->getVehicle()->broadcast->getGlobalPosition();
    } while (gp.altitude != 0);

    if (gp.altitude != 0)
    {
      std::cout
        << "Landing finished, but the aircraft is in an unexpected mode. "
           "Please connect DJI GO.\n";
      return false;
    }
    else
    {
      std::cout << "Successful landing!\n";
    }
  }
  else // M100
  {
    while (ptr_wrapper_->getVehicle()->broadcast->getStatus().flight ==
           DJI::OSDK::VehicleStatus::M100FlightStatus::FINISHING_LANDING)
    {
      sleep(1);
    }

    Telemetry::GlobalPosition gp;
    do
    {
      sleep(2);
      gp = ptr_wrapper_->getVehicle()->broadcast->getGlobalPosition();
    } while (gp.altitude != 0);

    if (gp.altitude != 0)
    {
      std::cout
        << "Landing finished, but the aircraft is in an unexpected mode. "
           "Please connect DJI GO.\n";
      return false;
    }
    else
    {
      std::cout << "Successful landing!\n";
    }
  }

  // Cleanup
  if (!ptr_wrapper_->getVehicle()->isM100() && !ptr_wrapper_->getVehicle()->isLegacyM600())
  {
    ack = ptr_wrapper_->getVehicle()->subscribe->removePackage(pkgIndex, timeout);
    if (ACK::getError(ack))
    {
      std::cout
        << "Error unsubscribing; please restart the drone/FC to get back "
           "to a clean state.\n";
    }
  }

  return true;
}

bool VehicleNode::moveByPositionOffset(ACK::ErrorCode& ack, int timeout, MoveOffset& p_offset)
{
  // Set timeout: this timeout is the time you allow the drone to take to finish
  // the
  // mission
  using namespace Telemetry;
  auto xOffsetDesired = p_offset.x;
  auto yOffsetDesired = p_offset.y;
  auto zOffsetDesired = p_offset.z;
  auto yawDesired = p_offset.yaw;
  auto posThresholdInM = p_offset.pos_threshold;
  auto yawThresholdInDeg = p_offset.yaw_threshold;

  int responseTimeout              = 1;
  int timeoutInMilSec              = 40000;
  int controlFreqInHz              = 50; // Hz
  int cycleTimeInMs                = 1000 / controlFreqInHz;
  int outOfControlBoundsTimeLimit  = 10 * cycleTimeInMs; // 10 cycles
  int withinControlBoundsTimeReqmt = 50 * cycleTimeInMs; // 50 cycles
  int pkgIndex;

  //@todo: remove this once the getErrorCode function signature changes
  char func[50];

  if (!ptr_wrapper_->getVehicle()->isM100() && !ptr_wrapper_->getVehicle()->isLegacyM600())
  {
    // Telemetry: Verify the subscription
    ack = ptr_wrapper_->getVehicle()->subscribe->verify(responseTimeout);
    if (ACK::getError(ack) != ACK::SUCCESS)
    {
      ACK::getErrorCodeMessage(ack, func);
      return false;
    }

    // Telemetry: Subscribe to quaternion, fused lat/lon and altitude at freq 50
    // Hz
    pkgIndex                  = 0;
    int       freq            = 50;
    TopicName topicList50Hz[] = { TOPIC_QUATERNION, TOPIC_GPS_FUSED };
    int       numTopic = sizeof(topicList50Hz) / sizeof(topicList50Hz[0]);
    bool      enableTimestamp = false;

    bool pkgStatus = ptr_wrapper_->getVehicle()->subscribe->initPackageFromTopicList(
      pkgIndex, numTopic, topicList50Hz, enableTimestamp, freq);
    if (!(pkgStatus))
    {
      return pkgStatus;
    }
    ack =
      ptr_wrapper_->getVehicle()->subscribe->startPackage(pkgIndex, responseTimeout);
    if (ACK::getError(ack) != ACK::SUCCESS)
    {
      ACK::getErrorCodeMessage(ack, func);
      // Cleanup before return
      ptr_wrapper_->getVehicle()->subscribe->removePackage(pkgIndex, responseTimeout);
      return false;
    }

    // Also, since we don't have a source for relative height through subscription,
    // start using broadcast height
    if (!startGlobalPositionBroadcast())
    {
      // Cleanup before return
      ptr_wrapper_->getVehicle()->subscribe->removePackage(pkgIndex, responseTimeout);
      return false;
    }
  }

  // Wait for data to come in
  sleep(1);

  // Get data

  // Global position retrieved via subscription
  Telemetry::TypeMap<TOPIC_GPS_FUSED>::type currentSubscriptionGPS;
  Telemetry::TypeMap<TOPIC_GPS_FUSED>::type originSubscriptionGPS;
  // Global position retrieved via broadcast
  Telemetry::GlobalPosition currentBroadcastGP;
  Telemetry::GlobalPosition originBroadcastGP;

  // Convert position offset from first position to local coordinates
  Telemetry::Vector3f localOffset;

  if (!ptr_wrapper_->getVehicle()->isM100() && !ptr_wrapper_->getVehicle()->isLegacyM600())
  {
    currentSubscriptionGPS = ptr_wrapper_->getVehicle()->subscribe->getValue<TOPIC_GPS_FUSED>();
    originSubscriptionGPS  = currentSubscriptionGPS;
    localOffsetFromGpsOffset(localOffset,
                             static_cast<void*>(&currentSubscriptionGPS),
                             static_cast<void*>(&originSubscriptionGPS));

    // Get the broadcast GP since we need the height for zCmd
    currentBroadcastGP = ptr_wrapper_->getVehicle()->broadcast->getGlobalPosition();
  }
  else
  {
    currentBroadcastGP = ptr_wrapper_->getVehicle()->broadcast->getGlobalPosition();
    originBroadcastGP  = currentBroadcastGP;
    localOffsetFromGpsOffset(localOffset,
                             static_cast<void*>(&currentBroadcastGP),
                             static_cast<void*>(&originBroadcastGP));
  }

  // Get initial offset. We will update this in a loop later.
  double xOffsetRemaining = xOffsetDesired - localOffset.x;
  double yOffsetRemaining = yOffsetDesired - localOffset.y;
  double zOffsetRemaining = zOffsetDesired - localOffset.z;

  // Conversions
  double yawDesiredRad     = DEG2RAD * yawDesired;
  double yawThresholdInRad = DEG2RAD * yawThresholdInDeg;

  //! Get Euler angle

  // Quaternion retrieved via subscription
  Telemetry::TypeMap<TOPIC_QUATERNION>::type subscriptionQ;
  // Quaternion retrieved via broadcast
  Telemetry::Quaternion broadcastQ;

  double yawInRad;
  if (!ptr_wrapper_->getVehicle()->isM100() && !ptr_wrapper_->getVehicle()->isLegacyM600())
  {
    subscriptionQ = ptr_wrapper_->getVehicle()->subscribe->getValue<TOPIC_QUATERNION>();
    yawInRad = toEulerAngle((static_cast<void*>(&subscriptionQ))).z / DEG2RAD;
  }
  else
  {
    broadcastQ = ptr_wrapper_->getVehicle()->broadcast->getQuaternion();
    yawInRad   = toEulerAngle((static_cast<void*>(&broadcastQ))).z / DEG2RAD;
  }

  int   elapsedTimeInMs     = 0;
  int   withinBoundsCounter = 0;
  int   outOfBounds         = 0;
  int   brakeCounter        = 0;
  int   speedFactor         = 2;
  float xCmd, yCmd, zCmd;

  /*! Calculate the inputs to send the position controller. We implement basic
   *  receding setpoint position control and the setpoint is always 1 m away
   *  from the current position - until we get within a threshold of the goal.
   *  From that point on, we send the remaining distance as the setpoint.
   */
  if (xOffsetDesired > 0)
    xCmd = (xOffsetDesired < speedFactor) ? xOffsetDesired : speedFactor;
  else if (xOffsetDesired < 0)
    xCmd =
      (xOffsetDesired > -1 * speedFactor) ? xOffsetDesired : -1 * speedFactor;
  else
    xCmd = 0;

  if (yOffsetDesired > 0)
    yCmd = (yOffsetDesired < speedFactor) ? yOffsetDesired : speedFactor;
  else if (yOffsetDesired < 0)
    yCmd =
      (yOffsetDesired > -1 * speedFactor) ? yOffsetDesired : -1 * speedFactor;
  else
    yCmd = 0;

  if (!ptr_wrapper_->getVehicle()->isM100() && !ptr_wrapper_->getVehicle()->isLegacyM600())
  {
    zCmd = currentBroadcastGP.height + zOffsetDesired; //Since subscription cannot give us a relative height, use broadcast.
  }
  else
  {
    zCmd = currentBroadcastGP.height + zOffsetDesired;
  }

  //! Main closed-loop receding setpoint position control
  while (elapsedTimeInMs < timeoutInMilSec)
  {
    ptr_wrapper_->getVehicle()->control->positionAndYawCtrl(xCmd, yCmd, zCmd,
                                         yawDesiredRad / DEG2RAD);

    usleep(cycleTimeInMs * 1000);
    elapsedTimeInMs += cycleTimeInMs;

    //! Get current position in required coordinates and units
    if (!ptr_wrapper_->getVehicle()->isM100() && !ptr_wrapper_->getVehicle()->isLegacyM600())
    {
      subscriptionQ = ptr_wrapper_->getVehicle()->subscribe->getValue<TOPIC_QUATERNION>();
      yawInRad      = toEulerAngle((static_cast<void*>(&subscriptionQ))).z;
      currentSubscriptionGPS = ptr_wrapper_->getVehicle()->subscribe->getValue<TOPIC_GPS_FUSED>();
      localOffsetFromGpsOffset(localOffset,
                               static_cast<void*>(&currentSubscriptionGPS),
                               static_cast<void*>(&originSubscriptionGPS));

      // Get the broadcast GP since we need the height for zCmd
      currentBroadcastGP = ptr_wrapper_->getVehicle()->broadcast->getGlobalPosition();
    }
    else
    {
      broadcastQ         = ptr_wrapper_->getVehicle()->broadcast->getQuaternion();
      yawInRad           = toEulerAngle((static_cast<void*>(&broadcastQ))).z;
      currentBroadcastGP = ptr_wrapper_->getVehicle()->broadcast->getGlobalPosition();
      localOffsetFromGpsOffset(localOffset,
                               static_cast<void*>(&currentBroadcastGP),
                               static_cast<void*>(&originBroadcastGP));
    }

    //! See how much farther we have to go
    xOffsetRemaining = xOffsetDesired - localOffset.x;
    yOffsetRemaining = yOffsetDesired - localOffset.y;
    zOffsetRemaining = zOffsetDesired - localOffset.z;

    //! See if we need to modify the setpoint
    if (std::abs(xOffsetRemaining) < speedFactor)
    {
      xCmd = xOffsetRemaining;
    }
    if (std::abs(yOffsetRemaining) < speedFactor)
    {
      yCmd = yOffsetRemaining;
    }

    if (ptr_wrapper_->getVehicle()->isM100() && std::abs(xOffsetRemaining) < posThresholdInM &&
        std::abs(yOffsetRemaining) < posThresholdInM &&
        std::abs(yawInRad - yawDesiredRad) < yawThresholdInRad)
    {
      //! 1. We are within bounds; start incrementing our in-bound counter
      withinBoundsCounter += cycleTimeInMs;
    }
    else if (std::abs(xOffsetRemaining) < posThresholdInM &&
             std::abs(yOffsetRemaining) < posThresholdInM &&
             std::abs(zOffsetRemaining) < posThresholdInM &&
             std::abs(yawInRad - yawDesiredRad) < yawThresholdInRad)
    {
      //! 1. We are within bounds; start incrementing our in-bound counter
      withinBoundsCounter += cycleTimeInMs;
    }
    else
    {
      if (withinBoundsCounter != 0)
      {
        //! 2. Start incrementing an out-of-bounds counter
        outOfBounds += cycleTimeInMs;
      }
    }
    //! 3. Reset withinBoundsCounter if necessary
    if (outOfBounds > outOfControlBoundsTimeLimit)
    {
      withinBoundsCounter = 0;
      outOfBounds         = 0;
    }
    //! 4. If within bounds, set flag and break
    if (withinBoundsCounter >= withinControlBoundsTimeReqmt)
    {
      break;
    }
  }

  //! Set velocity to zero, to prevent any residual velocity from position
  //! command
  if (!ptr_wrapper_->getVehicle()->isM100() && !ptr_wrapper_->getVehicle()->isLegacyM600())
  {
    while (brakeCounter < withinControlBoundsTimeReqmt)
    {
      ptr_wrapper_->getVehicle()->control->emergencyBrake();
      usleep(cycleTimeInMs * 10);
      brakeCounter += cycleTimeInMs;
    }
  }

  if (elapsedTimeInMs >= timeoutInMilSec)
  {
    std::cout << "Task timeout!\n";
    if (!ptr_wrapper_->getVehicle()->isM100() && !ptr_wrapper_->getVehicle()->isLegacyM600())
    {
      ack =
        ptr_wrapper_->getVehicle()->subscribe->removePackage(pkgIndex, responseTimeout);
      if (ACK::getError(ack))
      {
        std::cout << "Error unsubscribing; please restart the drone/FC to get "
                     "back to a clean state.\n";
      }
    }
    return ACK::FAIL;
  }

  if (!ptr_wrapper_->getVehicle()->isM100() && !ptr_wrapper_->getVehicle()->isLegacyM600())
  {
    ack =
      ptr_wrapper_->getVehicle()->subscribe->removePackage(pkgIndex, responseTimeout);
    if (ACK::getError(ack))
    {
      std::cout
        << "Error unsubscribing; please restart the drone/FC to get back "
           "to a clean state.\n";
    }
  }

  return ACK::SUCCESS;
}


bool VehicleNode::startGlobalPositionBroadcast()
{
  uint8_t freq[16];
  /* Channels definition for A3/N3/M600
   * 0 - Timestamp
   * 1 - Attitude Quaternions
   * 2 - Acceleration
   * 3 - Velocity (Ground Frame)
   * 4 - Angular Velocity (Body Frame)
   * 5 - Position
   * 6 - GPS Detailed Information
   * 7 - RTK Detailed Information
   * 8 - Magnetometer
   * 9 - RC Channels Data
   * 10 - Gimbal Data
   * 11 - Flight Status
   * 12 - Battery Level
   * 13 - Control Information
   */
  freq[0]  = DataBroadcast::FREQ_HOLD;
  freq[1]  = DataBroadcast::FREQ_HOLD;
  freq[2]  = DataBroadcast::FREQ_HOLD;
  freq[3]  = DataBroadcast::FREQ_HOLD;
  freq[4]  = DataBroadcast::FREQ_HOLD;
  freq[5]  = DataBroadcast::FREQ_50HZ; // This is the only one we want to change
  freq[6]  = DataBroadcast::FREQ_HOLD;
  freq[7]  = DataBroadcast::FREQ_HOLD;
  freq[8]  = DataBroadcast::FREQ_HOLD;
  freq[9]  = DataBroadcast::FREQ_HOLD;
  freq[10] = DataBroadcast::FREQ_HOLD;
  freq[11] = DataBroadcast::FREQ_HOLD;
  freq[12] = DataBroadcast::FREQ_HOLD;
  freq[13] = DataBroadcast::FREQ_HOLD;

  ACK::ErrorCode ack = ptr_wrapper_->getVehicle()->broadcast->setBroadcastFreq(freq, 1);
  if (ACK::getError(ack))
  {
    ACK::getErrorCodeMessage(ack, __func__);
    return false;
  }
  else
  {
    return true;
  }
}

void VehicleNode::localOffsetFromGpsOffset(Telemetry::Vector3f& deltaNed, void* target, void* origin)
{
  Telemetry::GPSFused*       subscriptionTarget;
  Telemetry::GPSFused*       subscriptionOrigin;
  Telemetry::GlobalPosition* broadcastTarget;
  Telemetry::GlobalPosition* broadcastOrigin;
  double                     deltaLon;
  double                     deltaLat;

  if (!ptr_wrapper_->getVehicle()->isM100() && !ptr_wrapper_->getVehicle()->isLegacyM600())
  {
    subscriptionTarget = (Telemetry::GPSFused*)target;
    subscriptionOrigin = (Telemetry::GPSFused*)origin;
    deltaLon   = subscriptionTarget->longitude - subscriptionOrigin->longitude;
    deltaLat   = subscriptionTarget->latitude - subscriptionOrigin->latitude;
    deltaNed.x = deltaLat * C_EARTH;
    deltaNed.y = deltaLon * C_EARTH * cos(subscriptionTarget->latitude);
    deltaNed.z = subscriptionTarget->altitude - subscriptionOrigin->altitude;
  }
  else
  {
    broadcastTarget = (Telemetry::GlobalPosition*)target;
    broadcastOrigin = (Telemetry::GlobalPosition*)origin;
    deltaLon        = broadcastTarget->longitude - broadcastOrigin->longitude;
    deltaLat        = broadcastTarget->latitude - broadcastOrigin->latitude;
    deltaNed.x      = deltaLat * C_EARTH;
    deltaNed.y      = deltaLon * C_EARTH * cos(broadcastTarget->latitude);
    deltaNed.z      = broadcastTarget->altitude - broadcastOrigin->altitude;
  }
}

Telemetry::Vector3f VehicleNode::toEulerAngle(void* quaternionData)
{
  Telemetry::Vector3f    ans;
  Telemetry::Quaternion* quaternion = (Telemetry::Quaternion*)quaternionData;

  double q2sqr = quaternion->q2 * quaternion->q2;
  double t0    = -2.0 * (q2sqr + quaternion->q3 * quaternion->q3) + 1.0;
  double t1 =
    +2.0 * (quaternion->q1 * quaternion->q2 + quaternion->q0 * quaternion->q3);
  double t2 =
    -2.0 * (quaternion->q1 * quaternion->q3 - quaternion->q0 * quaternion->q2);
  double t3 =
    +2.0 * (quaternion->q2 * quaternion->q3 + quaternion->q0 * quaternion->q1);
  double t4 = -2.0 * (quaternion->q1 * quaternion->q1 + q2sqr) + 1.0;

  t2 = (t2 > 1.0) ? 1.0 : t2;
  t2 = (t2 < -1.0) ? -1.0 : t2;

  ans.x = asin(t2);
  ans.y = atan2(t3, t4);
  ans.z = atan2(t1, t0);

  return ans;
}

bool VehicleNode::setGimbalAngle(const GimbalContainer& gimbal)
{
  DJI::OSDK::Gimbal::AngleData gimbalAngle = {};
  gimbalAngle.roll     = gimbal.roll;
  gimbalAngle.pitch    = gimbal.pitch;
  gimbalAngle.yaw      = gimbal.yaw;
  gimbalAngle.duration = gimbal.duration;
  gimbalAngle.mode |= 0;
  gimbalAngle.mode |= gimbal.isAbsolute;
  gimbalAngle.mode |= gimbal.yaw_cmd_ignore << 1;
  gimbalAngle.mode |= gimbal.roll_cmd_ignore << 2;
  gimbalAngle.mode |= gimbal.pitch_cmd_ignore << 3;

  if(ptr_wrapper_ == nullptr)
  {
    return false;
  }
  ptr_wrapper_->getVehicle()->gimbal->setAngle(&gimbalAngle);
  // Give time for gimbal to sync
  sleep(4);
  return true;
}



int main(int argc, char** argv)
{
  ros::init(argc, argv, "vehicle_node");
  VehicleNode vh_node;

  ros::spin();
  return 0;
}
