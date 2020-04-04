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
#include <dji_osdk_ros/dji_vehicle_node.hh>
#include <dji_osdk_ros/vehicle_wrapper.hh>
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
  nh_.param("/vehicle_node/app_id",        app_id_,    12345);
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
  ptr_wrapper_ = std::make_unique<VehicleWrapper>(app_id_, enc_key_, device_, baud_rate_, enable_ad);

  if(ptr_wrapper_ == nullptr)
  {
    ROS_ERROR_STREAM("Vehicle Wrapper inited failed");
    ros::shutdown();
  }
  ROS_INFO_STREAM("VehicleNode Start");

  initSubscribe();
  initService();
  initTopic();
}

void VehicleNode::initService()
{
  task_control_server_ = nh_.advertiseService("drone_task_control", &VehicleNode::taskCtrlCallback, this);
  gimbal_control_server_ = nh_.advertiseService("gimbal_task_control", &VehicleNode::gimbalCtrlCallback, this);
  camera_action_control_server_ = nh_.advertiseService("camera_task_control", &VehicleNode::cameraCtrlCallback, this);
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

bool VehicleNode::publishTopic()
{
#ifdef ADVANCED_SENSING
    publishAdvancedSeningData();
#endif
}

#ifdef ADVANCED_SENSING
bool VehicleNode::publishAdvancedSeningData()
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
//          ROS_INFO("raw data len is %ld\n",cameraData.raw_data.size());
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
        ptr_wrapper_->goHome(WAIT_TIMEOUT);
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
        ptr_wrapper_->moveByPositionOffset(ack, WAIT_TIMEOUT, tmp_offset);
        break;
      }
    case DroneTaskControl::Request::TASK_TAKEOFF:
      {
        ROS_INFO_STREAM("call takeoff service");
        ptr_wrapper_->monitoredTakeoff(ack, WAIT_TIMEOUT);
        break;
      }
    case DroneTaskControl::Request::TASK_LAND:
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
  RotationAngle initial_angle;
  //if(getCurrentGimbal(initial_angle) == false)
  //{
  //  ROS_ERROR_STREAM("Get Current Gimbal Angle Failed");
  //  return false;
  //}
  GimbalContainer gimbal(request.yaw, request.pitch, request.roll, 0, 1, initial_angle);
  response.result = ptr_wrapper_->setGimbalAngle(gimbal);
  return true;
}

bool VehicleNode::cameraCtrlCallback(CameraAction::Request& request, CameraAction::Response& response)
{
  if(ptr_wrapper_ == nullptr)
  {
    ROS_ERROR_STREAM("Vehicle Wrapper is nullptr");
    return true;
  }
  response.result = false;
  switch (request.action)
  {
    case CameraAction::Request::CAMERA_ACTION_TAKE_PICTURE:
      {
        ROS_INFO_STREAM("Call take picture.");
        response.result = ptr_wrapper_->takePicture();
        break;
      }
    case CameraAction::Request::CAMERA_ACTION_START_RECORD:
      {
        ROS_INFO_STREAM("Call record video.");
        response.result = ptr_wrapper_->startCaptureVideo();
        break;
      }
    case CameraAction::Request::CAMERA_ACTION_STOP_RECORD:
      {
        ROS_INFO_STREAM("Call stop video.");
        response.result = ptr_wrapper_->stopCaptureVideo();
        break;
      }
    case CameraAction::Request::CAMERA_ACTION_ZOOM:
      {
        ROS_INFO_STREAM("Call Camera Zoom");
        CameraZoomDataType zoom_data;

        memcpy(&zoom_data.func_index, &request.func_index, sizeof(zoom_data.func_index));
        memcpy(&zoom_data.cam_index, &request.cam_index, sizeof(zoom_data.cam_index));
        memcpy(&zoom_data.zoom_config, &request.zoom_config, sizeof(zoom_data.zoom_config));
        memcpy(&zoom_data.optical_zoom_param.step_param, &request.step_param, sizeof(zoom_data.optical_zoom_param.step_param));
        memcpy(&zoom_data.optical_zoom_param.cont_param, &request.cont_param, sizeof(zoom_data.optical_zoom_param.cont_param));
        memcpy(&zoom_data.optical_zoom_param.pos_param, &request.pos_param, sizeof(zoom_data.optical_zoom_param.pos_param));

        response.result = ptr_wrapper_->zoomCtrl(zoom_data);
        break;
      }
    defaule:
      break;
  }
  //return response.result;
  return true;
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

int main(int argc, char** argv)
{
  ros::init(argc, argv, "vehicle_node");
//  VehicleNode vh_node(1);
  VehicleNode vh_node;
  vh_node.publishTopic();

  ros::spin();
  return 0;
}
