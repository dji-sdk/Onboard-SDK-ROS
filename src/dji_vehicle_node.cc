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
    defaule:
      break;
  }
  //return response.result;
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
  VehicleNode vh_node;

  ros::spin();
  return 0;
}
