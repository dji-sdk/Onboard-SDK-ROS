/**
  * @Copyright (C) 2020 All rights reserved.
  * @date: 2020
  * @file: dji_vehicle_node.cc
  * @version: v0.0.1
  * @author: kevin.hoo@dji.com
  * @create_date: 2020-03-08 16:08:55
  * @last_modified_date: 2020-03-25 18:28:21
  * @brief: TODO
  * @details: TODO
  */

//INCLUDE
#include <dji_osdk_ros/dji_vehicle_node.h>
#include <dji_osdk_ros/vehicle_wrapper.h>
#include <vector>
//CODE
using namespace dji_osdk_ros;
const int WAIT_TIMEOUT = 10;

VehicleNode::VehicleNode(int test)
{
  initService();
}

VehicleNode::VehicleNode()
{
  nh_.param("/vehicle_node/app_id",        app_id_, 123456);
  nh_.param("/vehicle_node/enc_key",       enc_key_, std::string("abcde123"));
  nh_.param("/vehicle_node/acm_name",      device_acm_, std::string("/dev/ttyACM0"));
  nh_.param("/vehicle_node/serial_name",   device_, std::string("/dev/ttyUSB0"));
  nh_.param("/vehicle_node/baud_rate",     baud_rate_, 230400);
  nh_.param("/vehicle_node/app_version",   app_version_, 1);
  nh_.param("/vehicle_node/drone_version", drone_version_, std::string("M100")); // choose M100 as default
  nh_.param("/vehicle_node/gravity_const", gravity_const_, 9.801);
  bool enable_ad = false;
#ifdef ADVANCED_SENSING
  enable_ad = true;
#else
  enable_ad = false;
#endif
  ptr_wrapper_ = new VehicleWrapper(app_id_, enc_key_, device_acm_, device_, baud_rate_, enable_ad);

  if(ptr_wrapper_ == nullptr)
  {
    ROS_ERROR_STREAM("Vehicle Wrapper inited failed");
    ros::shutdown();
  }
  ROS_INFO_STREAM("VehicleNode Start");

  subscribeGimbalData();
  initCameraModule();
  initService();
  initTopic();
}

VehicleNode::~VehicleNode()
{
  unSubScribeGimbalData();
}

bool VehicleNode::subscribeGimbalData()
{
  if(ptr_wrapper_ == nullptr)
  {
    ROS_ERROR_STREAM("Vehicle Wrapper is nullptr");
    return true;
  }
  Vehicle* vehicle = ptr_wrapper_->getVehicle();
  /*! verify the subscribe function */
  ACK::ErrorCode ack = vehicle->subscribe->verify(1);
  if (ACK::getError(ack) != ACK::SUCCESS) {
    ACK::getErrorCodeMessage(ack, __func__);
    return false;
  }

  /*! Package 0: Subscribe to gimbal data at freq 50 Hz */
  ACK::ErrorCode subscribeStatus;
  int       pkgIndex        = static_cast<int>(dji_osdk_ros::SubscribePackgeIndex::GIMBA_SUB_PACKAGE_INDEX);
  int       freq            = 50;
  TopicName topicList50Hz[]  = { vehicle->isM300() ? TOPIC_THREE_GIMBAL_DATA : TOPIC_DUAL_GIMBAL_DATA };
  int       numTopic        = sizeof(topicList50Hz) / sizeof(topicList50Hz[0]);
  bool      enableTimestamp = false;

  bool pkgStatus = vehicle->subscribe->initPackageFromTopicList(
      pkgIndex, numTopic, topicList50Hz, enableTimestamp, freq);
  if (!(pkgStatus))
  {
    std::cout << "init package for gimbal data failed." << std::endl;
    return false;
  }
  subscribeStatus = vehicle->subscribe->startPackage(pkgIndex, 1);
  if (ACK::getError(subscribeStatus) != ACK::SUCCESS)
  {
    ACK::getErrorCodeMessage(subscribeStatus, __func__);
    vehicle->subscribe->removePackage(pkgIndex, 1);
    std::cout << "subscribe gimbal data failed."<< std::endl;
    return false;
  }

  /*! init gimbal modules for gimbalManager */
  ErrorCode::ErrorCodeType ret;
  /*! main gimbal init */
  ret = vehicle->gimbalManager->initGimbalModule(PAYLOAD_INDEX_0,
                                                 "main_gimbal");
  if (ret != ErrorCode::SysCommonErr::Success)
  {
    std::cout << "Init Camera module main_gimbal failed."<< std::endl;
    ErrorCode::printErrorCodeMsg(ret);
    return false;
  }
  /*! vice gimbal init */
  ret = vehicle->gimbalManager->initGimbalModule(PAYLOAD_INDEX_1,
                                                 "vice_gimbal");
  if (ret != ErrorCode::SysCommonErr::Success)
  {
    std::cout << "Init Camera module vice_gimbal failed." << std::endl;
    ErrorCode::printErrorCodeMsg(ret);
    return false;
  }
  /*! top gimbal init */
  if (vehicle->isM300())
  {
    ret = vehicle->gimbalManager->initGimbalModule(PAYLOAD_INDEX_2,
                                                   "top_gimbal");
    if (ret != ErrorCode::SysCommonErr::Success) {
      std::cout << "Init Camera module top_gimbal failed." << std::endl;
      ErrorCode::printErrorCodeMsg(ret);
      return false;
    }
  }

  return true;
}

bool VehicleNode::unSubScribeGimbalData()
{
  if(ptr_wrapper_ == nullptr)
  {
    ROS_ERROR_STREAM("Vehicle Wrapper is nullptr");
    return true;
  }
  Vehicle* vehicle = ptr_wrapper_->getVehicle();

  int pkgIndex = static_cast<int>(dji_osdk_ros::SubscribePackgeIndex::GIMBA_SUB_PACKAGE_INDEX);
  ACK::ErrorCode subscribeStatus =
      vehicle->subscribe->removePackage(pkgIndex, 1);
  if (ACK::getError(subscribeStatus) != ACK::SUCCESS)
  {
    ACK::getErrorCodeMessage(subscribeStatus, __func__);
    std::cout << "remove subscribe package gimbal data failed." << std::endl;
    return false;
  }
  return true;
}

bool VehicleNode::initCameraModule()
{
  if(ptr_wrapper_ == nullptr)
  {
    ROS_ERROR_STREAM("Vehicle Wrapper is nullptr");
    return true;
  }
  Vehicle* vehicle = ptr_wrapper_->getVehicle();

  /*! init camera modules for cameraManager */
  /*! main camera init */
  ErrorCode::ErrorCodeType ret = vehicle->cameraManager->initCameraModule(
      PAYLOAD_INDEX_0, "main_camera");
  if (ret != ErrorCode::SysCommonErr::Success) {
    DERROR("Init Camera module main_camera failed.");
    ErrorCode::printErrorCodeMsg(ret);
    return false;
  }
  /*! vice camera init */
  ret = vehicle->cameraManager->initCameraModule(PAYLOAD_INDEX_1,
                                                 "vice_camera");
  if (ret != ErrorCode::SysCommonErr::Success) {
    DERROR("Init Camera module vice_camera failed.");
    ErrorCode::printErrorCodeMsg(ret);
    return false;
  }
  /*! top camera init for M300 */
  if (vehicle->isM300()) {
    ret = vehicle->cameraManager->initCameraModule(PAYLOAD_INDEX_2,
                                                   "top_camera");
    if (ret != ErrorCode::SysCommonErr::Success)
    {
      DERROR("Init Camera module top_camera failed.");
      ErrorCode::printErrorCodeMsg(ret);
      return false;
    }
  }

  return true;
}

void VehicleNode::initService()
{
  task_control_server_ = nh_.advertiseService("flight_task_control", &VehicleNode::taskCtrlCallback, this);
  gimbal_control_server_ = nh_.advertiseService("gimbal_task_control", &VehicleNode::gimbalCtrlCallback, this);
  camera_control_set_EV_server_ = nh_.advertiseService("camera_task_set_EV", &VehicleNode::cameraSetEVCallback, this);
  camera_control_set_shutter_speed_server_ = nh_.advertiseService("camera_task_set_shutter_speed", &VehicleNode::cameraSetShutterSpeedCallback, this);
  camera_control_set_aperture_server_ = nh_.advertiseService("camera_task_set_aperture", &VehicleNode::cameraSetApertureCallback, this);
  camera_control_set_ISO_server_ = nh_.advertiseService("camera_task_set_ISO", &VehicleNode::cameraSetISOCallback, this);
  camera_control_set_focus_point_server_ = nh_.advertiseService("camera_task_set_focus_point", &VehicleNode::cameraSetFocusPointCallback, this);
  camera_control_set_tap_zoom_point_server_ = nh_.advertiseService("camera_task_tap_zoom_point", &VehicleNode::cameraSetTapZoomPointCallback, this);
  camera_control_zoom_ctrl_server_ = nh_.advertiseService("camera_task_zoom_ctrl", &VehicleNode::cameraZoomCtrlCallback, this);
  camera_control_start_shoot_single_photo_server_ = nh_.advertiseService("camera_start_shoot_single_photo", &VehicleNode::cameraStartShootSinglePhotoCallback, this);
  camera_control_start_shoot_AEB_photo_server_ = nh_.advertiseService("camera_start_shoot_aeb_photo", &VehicleNode::cameraStartShootAEBPhotoCallback, this);
  camera_control_start_shoot_burst_photo_server_ = nh_.advertiseService("camera_start_shoot_burst_photo", &VehicleNode::cameraStartShootBurstPhotoCallback, this);
  camera_control_start_shoot_interval_photo_server_ = nh_.advertiseService("camera_start_shoot_interval_photo", &VehicleNode::cameraStartShootIntervalPhotoCallback, this);
  camera_control_stop_shoot_photo_server_ = nh_.advertiseService("camera_stop_shoot_photo", &VehicleNode::cameraStopShootPhotoCallback, this);
  camera_control_record_video_action_server_ = nh_.advertiseService("camera_record_video_action", &VehicleNode::cameraRecordVideoActionCallback, this);

  mfio_control_server_ = nh_.advertiseService("mfio_control", &VehicleNode::mfioCtrlCallback, this);

  set_home_altitude_server_ = nh_.advertiseService("set_go_home_altitude", &VehicleNode::setGoHomeAltitudeCallback,this);
  set_current_point_as_home_server_ = nh_.advertiseService("set_current_point_as_home", &VehicleNode::setHomeCallback,this);
  avoid_enable_server_ = nh_.advertiseService("enable_avoid", &VehicleNode::setAvoidCallback,this);
#ifdef ADVANCED_SENSING
  advanced_sensing_server_ = nh_.advertiseService("advanced_sensing", &VehicleNode::advancedSensingCallback,this);
#endif
  ROS_INFO_STREAM("Services startup!");
}

void VehicleNode::initTopic()
{
#ifdef ADVANCED_SENSING
    advanced_sensing_pub_ = nh_.advertise<dji_osdk_ros::CameraData>("cameradata", 1000);
#endif
    ROS_INFO_STREAM("Topic startup!");
}

void VehicleNode::publishTopic()
{
#ifdef ADVANCED_SENSING
    publishAdvancedSeningData();
#endif
}

#ifdef ADVANCED_SENSING
void VehicleNode::publishAdvancedSeningData()
{
    ros::AsyncSpinner spinner(4);
    spinner.start();

    dji_osdk_ros::CameraData cameraData;
    std::vector<uint8_t> lastCameraData = cameraData.raw_data;

    ros::Rate rate(40);
    while(ros::ok())
    {
        cameraData = getCameraData();
        if (cameraData.raw_data != lastCameraData)
        {
            lastCameraData = cameraData.raw_data;
            ROS_INFO("raw data len is %ld\n",cameraData.raw_data.size());
            advanced_sensing_pub_.publish(cameraData);

            ros::spinOnce();
            rate.sleep();
        }
    }

    ros::waitForShutdown();
}

bool VehicleNode::advancedSensingCallback(AdvancedSensing::Request& request, AdvancedSensing::Response& response)
{
    ROS_DEBUG("called advancedSensingCallback");
    response.result = false;
    if(ptr_wrapper_ == nullptr)
    {
        ROS_ERROR_STREAM("Vehicle Wrapper is nullptr");
        return true;
    }

    ptr_wrapper_->setAcmDevicePath(device_acm_);
    is_h264_ = request.is_h264;

    if (request.is_open)
    {
        response.result = ptr_wrapper_->startStream(request.is_h264, request.request_view);
    }
    else
    {
        response.result = ptr_wrapper_->stopStream(request.is_h264, request.request_view);
    }

    return response.result;
}

dji_osdk_ros::CameraData VehicleNode::getCameraData()
{
    dji_osdk_ros::CameraData cameraData;
    if (is_h264_)
    {
        cameraData.raw_data = ptr_wrapper_->getCameraRawData();
    }
    else
    {
        CameraRGBImage image = ptr_wrapper_->getCameraImage();
        cameraData.raw_data = image.rawData;
        cameraData.height = image.height;
        cameraData.width = image.width;
    }

    return cameraData;
}
#endif

bool VehicleNode::taskCtrlCallback(FlightTaskControl::Request&  request, FlightTaskControl::Response& response)
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
    case FlightTaskControl::Request::TASK_GOHOME:
      {
        ROS_INFO_STREAM("call go home service");
        ptr_wrapper_->goHome(WAIT_TIMEOUT);
        break;
      }
    case FlightTaskControl::Request::TASK_GOHOME_AND_CONFIRM_LANDING:
    {
      ROS_INFO_STREAM("call go home and confirm landing service");
      ptr_wrapper_->goHomeAndConfirmLanding(WAIT_TIMEOUT);
      break;
    }
    case FlightTaskControl::Request::TASK_GO_LOCAL_POS:
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
        ptr_wrapper_->moveByPositionOffset(ack, WAIT_TIMEOUT, tmp_offset);
        break;
      }
    case FlightTaskControl::Request::TASK_TAKEOFF:
      {
        ROS_INFO_STREAM("call takeoff service");
        ptr_wrapper_->monitoredTakeoff(ack, WAIT_TIMEOUT);
        break;
      }
    case FlightTaskControl::Request::TASK_LAND:
      {
        ROS_INFO_STREAM("call land service");
        ptr_wrapper_->monitoredLanding(ack, WAIT_TIMEOUT);
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
  if(ptr_wrapper_ == nullptr)
  {
    ROS_ERROR_STREAM("Vehicle Wrapper is nullptr");
    return true;
  }
  response.result = false;
  ROS_INFO("Current gimbal %d, angle (p,r,y) = (%0.2f, %0.2f, %0.2f)", static_cast<int>(request.payload_index),
           ptr_wrapper_->getGimbalData(static_cast<PayloadIndex>(request.payload_index)).pitch,
           ptr_wrapper_->getGimbalData(static_cast<PayloadIndex>(request.payload_index)).roll,
           ptr_wrapper_->getGimbalData(static_cast<PayloadIndex>(request.payload_index)).yaw);
  ROS_INFO_STREAM("Call gimbal Ctrl.");

  if (request.is_reset)
  {
    response.result = ptr_wrapper_->resetGimbal(static_cast<PayloadIndex>(request.payload_index));
  }
  else
  {
    GimbalRotationData gimbalRotationData;
    gimbalRotationData.rotationMode = request.rotationMode;
    gimbalRotationData.pitch = request.pitch;
    gimbalRotationData.roll  = request.roll;
    gimbalRotationData.yaw   = request.yaw;
    gimbalRotationData.time  = request.time;
    response.result = ptr_wrapper_->rotateGimbal(static_cast<PayloadIndex>(request.payload_index), gimbalRotationData);
  }

  sleep(2);
  ROS_INFO("Current gimbal %d , angle (p,r,y) = (%0.2f, %0.2f, %0.2f)", request.payload_index,
           ptr_wrapper_->getGimbalData(static_cast<PayloadIndex>(request.payload_index)).pitch,
           ptr_wrapper_->getGimbalData(static_cast<PayloadIndex>(request.payload_index)).roll,
           ptr_wrapper_->getGimbalData(static_cast<PayloadIndex>(request.payload_index)).yaw);
  return true;
}

bool VehicleNode::cameraSetEVCallback(CameraEV::Request& request, CameraEV::Response& response)
{
  if(ptr_wrapper_ == nullptr)
  {
    ROS_ERROR_STREAM("Vehicle Wrapper is nullptr");
    return true;
  }
  response.result = false;
  if (ptr_wrapper_->setExposureMode(static_cast<PayloadIndex>(request.payload_index), static_cast<ExposureMode>(request.exposure_mode)))
  {
    return false;
  }
  if (ptr_wrapper_->setEV(static_cast<PayloadIndex>(request.payload_index),
                          static_cast<ExposureCompensation>(request.exposure_compensation)))
  {
    return false;
  }
  response.result = true;
  return true;
}

bool VehicleNode::cameraSetShutterSpeedCallback(CameraShutterSpeed::Request& request, CameraShutterSpeed::Response& response)
{
  if(ptr_wrapper_ == nullptr)
  {
    ROS_ERROR_STREAM("Vehicle Wrapper is nullptr");
    return true;
  }
  response.result = false;
  if (ptr_wrapper_->setExposureMode(static_cast<PayloadIndex>(request.payload_index), static_cast<ExposureMode>(request.exposure_mode)))
  {
    return false;
  }
  if (ptr_wrapper_->setShutterSpeed(static_cast<PayloadIndex>(request.payload_index), static_cast<ShutterSpeed>(request.shutter_speed)))
  {
    return false;
  }
  response.result = true;
  return true;
}

bool VehicleNode::cameraSetApertureCallback(CameraAperture::Request& request, CameraAperture::Response& response)
{
  if(ptr_wrapper_ == nullptr)
  {
    ROS_ERROR_STREAM("Vehicle Wrapper is nullptr");
    return true;
  }
  response.result = false;
  if (ptr_wrapper_->setExposureMode(static_cast<PayloadIndex>(request.payload_index), static_cast<ExposureMode>(request.exposure_mode)))
  {
    return false;
  }
  if (ptr_wrapper_->setAperture(static_cast<PayloadIndex>(request.payload_index), static_cast<Aperture>(request.aperture)))
  {
    return false;
  }
  response.result = true;
  return true;
}

bool VehicleNode::cameraSetISOCallback(CameraISO::Request& request, CameraISO::Response& response)
{
  if(ptr_wrapper_ == nullptr)
  {
    ROS_ERROR_STREAM("Vehicle Wrapper is nullptr");
    return true;
  }
  response.result = false;
  if (ptr_wrapper_->setExposureMode(static_cast<PayloadIndex>(request.payload_index), static_cast<ExposureMode>(request.exposure_mode)))
  {
    return false;
  }
  if (ptr_wrapper_->setISO(static_cast<PayloadIndex>(request.payload_index), static_cast<ISO>(request.iso_data)))
  {
    return false;
  }
  response.result = true;
  return true;
}

bool VehicleNode::cameraSetFocusPointCallback(CameraFocusPoint::Request& request, CameraFocusPoint::Response& response)
{
  if(ptr_wrapper_ == nullptr)
  {
    ROS_ERROR_STREAM("Vehicle Wrapper is nullptr");
    return true;
  }
  response.result = ptr_wrapper_->setFocusPoint(static_cast<PayloadIndex>(request.payload_index), request.x, request.y);
  return response.result;
}

bool VehicleNode::cameraSetTapZoomPointCallback(CameraTapZoomPoint::Request& request, CameraTapZoomPoint::Response& response)
{
  if(ptr_wrapper_ == nullptr)
  {
    ROS_ERROR_STREAM("Vehicle Wrapper is nullptr");
    return true;
  }
  response.result = ptr_wrapper_->setTapZoomPoint(static_cast<PayloadIndex>(request.payload_index),request.multiplier, request.x, request.y);
  return response.result;
}

bool VehicleNode::cameraZoomCtrlCallback(CameraZoomCtrl::Request& request, CameraZoomCtrl::Response& response)
{
  if(ptr_wrapper_ == nullptr)
  {
    ROS_ERROR_STREAM("Vehicle Wrapper is nullptr");
    return true;
  }
  if (request.start_stop == 1)
  {
    response.result = ptr_wrapper_->startZoom(static_cast<PayloadIndex>(request.payload_index), request.direction, request.speed);
  }
  if (request.start_stop == 0)
  {
    response.result = ptr_wrapper_->stopZoom(static_cast<PayloadIndex>(request.payload_index));
  }
  return response.result;
}

bool VehicleNode::cameraStartShootSinglePhotoCallback(CameraStartShootSinglePhoto::Request& request, CameraStartShootSinglePhoto::Response& response)
{
  if(ptr_wrapper_ == nullptr)
  {
    ROS_ERROR_STREAM("Vehicle Wrapper is nullptr");
    return true;
  }
  response.result = ptr_wrapper_->startShootSinglePhoto(static_cast<PayloadIndex>(request.payload_index));
  return response.result;
}

bool VehicleNode::cameraStartShootAEBPhotoCallback(CameraStartShootAEBPhoto::Request& request, CameraStartShootAEBPhoto::Response& response)
{
  if(ptr_wrapper_ == nullptr)
  {
    ROS_ERROR_STREAM("Vehicle Wrapper is nullptr");
    return true;
  }
  response.result = ptr_wrapper_->startShootAEBPhoto(static_cast<PayloadIndex>(request.payload_index), static_cast<PhotoAEBCount>(request.photo_aeb_count));
  return response.result;
}

bool VehicleNode::cameraStartShootBurstPhotoCallback(CameraStartShootBurstPhoto::Request& request, CameraStartShootBurstPhoto::Response& response)
{
  if(ptr_wrapper_ == nullptr)
  {
    ROS_ERROR_STREAM("Vehicle Wrapper is nullptr");
    return true;
  }
  response.result = ptr_wrapper_->startShootBurstPhoto(static_cast<PayloadIndex>(request.payload_index),static_cast<PhotoBurstCount>(request.photo_burst_count));
  return response.result;
}

bool VehicleNode::cameraStartShootIntervalPhotoCallback(CameraStartShootIntervalPhoto::Request& request, CameraStartShootIntervalPhoto::Response& response)
{
  if(ptr_wrapper_ == nullptr)
  {
    ROS_ERROR_STREAM("Vehicle Wrapper is nullptr");
    return true;
  }
  PhotoIntervalData photoIntervalData;
  photoIntervalData.photoNumConticap = request.photo_num_conticap;
  photoIntervalData.timeInterval = request.time_interval;
  response.result = ptr_wrapper_->startShootIntervalPhoto(static_cast<PayloadIndex>(request.payload_index), photoIntervalData);
  return response.result;
}

bool VehicleNode::cameraStopShootPhotoCallback(CameraStopShootPhoto::Request& request, CameraStopShootPhoto::Response& response)
{
  if(ptr_wrapper_ == nullptr)
  {
    ROS_ERROR_STREAM("Vehicle Wrapper is nullptr");
    return true;
  }
  response.result = ptr_wrapper_->shootPhotoStop(static_cast<PayloadIndex>(request.payload_index));
  return response.result;
}

bool VehicleNode::cameraRecordVideoActionCallback(CameraRecordVideoAction::Request& request, CameraRecordVideoAction::Response& response)
{
  if(ptr_wrapper_ == nullptr)
  {
    ROS_ERROR_STREAM("Vehicle Wrapper is nullptr");
    return true;
  }
  if(request.start_stop == 1)
  {
    response.result = ptr_wrapper_->startRecordVideo(static_cast<PayloadIndex>(request.payload_index));
  }
  if(request.start_stop == 0)
  {
    response.result = ptr_wrapper_->stopRecordVideo(static_cast<PayloadIndex>(request.payload_index));
  }
  return response.result;
}

bool VehicleNode::mfioCtrlCallback(MFIO::Request& request, MFIO::Response& response)
{
  ROS_INFO_STREAM("MFIO Control callback");
  if(ptr_wrapper_ == nullptr)
  {
    ROS_ERROR_STREAM("Vehicle Wrapper is nullptr");
    return true;
  }

  if(request.mode == MFIO::Request::MODE_PWM_OUT ||
     request.mode == MFIO::Request::MODE_GPIO_OUT)
  {
    switch (request.action)
    {
      case MFIO::Request::TURN_ON:
        {
          ptr_wrapper_->outputMFIO(request.mode, request.channel, request.init_on_time_us, request.pwm_freq, request.block, request.gpio_value);
        break;
        }
      default:
        {
          response.read_value = ptr_wrapper_->stopMFIO(request.mode, request.channel);
          break;
        }
    }
  }
  else if(request.mode == MFIO::Request::MODE_GPIO_IN ||
          request.mode == MFIO::Request::MODE_ADC)
  {
    response.read_value = ptr_wrapper_->inputMFIO(request.mode, request.channel, request.block);
  }
  return true;
}

bool VehicleNode::setGoHomeAltitudeCallback(SetGoHomeAltitude::Request& request, SetGoHomeAltitude::Response& response)
{
  ROS_INFO_STREAM("Set go home altitude callback");
  if(ptr_wrapper_ == nullptr)
  {
    ROS_ERROR_STREAM("Vehicle Wrapper is nullptr");
    return true;
  }
  if(request.altitude < 5)
  {
    ROS_WARN_STREAM("Altitude for going Home is TOO LOW");
    response.result = false;
    return true;
  }
  if(ptr_wrapper_->setHomeAltitude(request.altitude) == true)
  {
    response.result = true;
  }
  else
  {
    response.result = false;
  }
  return true;
}

bool VehicleNode::setHomeCallback(SetNewHomePoint::Request& request, SetNewHomePoint::Response& response)
{
  ROS_INFO_STREAM("Set new home point callback");
  if(ptr_wrapper_ == nullptr)
  {
    ROS_ERROR_STREAM("Vehicle Wrapper is nullptr");
    return true;
  }

  if(ptr_wrapper_->setNewHomeLocation() == true)
  {
    response.result = true;
  }
  else
  {
    response.result = false;
  }

  return true;
}

bool VehicleNode::setAvoidCallback(AvoidEnable::Request& request, AvoidEnable::Response& response)
{
  ROS_INFO_STREAM("Set avoid function callback");
  if(ptr_wrapper_ == nullptr)
  {
    ROS_ERROR_STREAM("Vehicle Wrapper is nullptr");
    return true;
  }

  if(ptr_wrapper_->setAvoid(request.enable) == true)
  {
    response.result = true;
  }
  else
  {
    response.result = false;
  }

  return true;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "vehicle_node");
//  VehicleNode vh_node(1);
  VehicleNode vh_node;
  vh_node.publishTopic();

  ros::spin();
  return 0;
}
