/** @file dji_sdk_node_services.cpp
 *  @version 3.7
 *  @date July, 2018
 *
 *  @brief
 *  Implementation of the general services of DJISDKNode
 *
 *  @copyright 2018 DJI. All rights reserved.
 *
 */

#include <dji_sdk/dji_sdk_node.h>

bool
DJISDKNode::droneActivationCallback(dji_osdk_ros::Activation::Request&  request,
                                    dji_osdk_ros::Activation::Response& response)
{
  ROS_DEBUG("called droneActivationCallback");

  //! @note activation arguments should be specified in launch files
  ACK::ErrorCode ack;
  ack = this->activate(this->app_id, this->enc_key);

  ROS_DEBUG("ack.info: set=%i id=%i", ack.info.cmd_set, ack.info.cmd_id);
  ROS_DEBUG("ack.data: %i", ack.data);

  response.cmd_set  = (int)ack.info.cmd_set;
  response.cmd_id   = (int)ack.info.cmd_id;
  response.ack_data = (unsigned int)ack.data;

  if (ACK::getError(ack))
  {
    response.result = false;
    ACK::getErrorCodeMessage(ack, __func__);
  }
  else
  {
    response.result = true;
    ROS_DEBUG("drone activated");
  }

  return true;
}

bool
DJISDKNode::droneArmCallback(dji_osdk_ros::DroneArmControl::Request&  request,
                             dji_osdk_ros::DroneArmControl::Response& response)
{
  ROS_DEBUG("called droneArmCallback");

  ACK::ErrorCode ack;

  if (request.arm)
  {
    ack = vehicle->control->armMotors(WAIT_TIMEOUT);
    ROS_DEBUG("called vehicle->control->armMotors()");
  }
  else
  {
    ack = vehicle->control->disArmMotors(WAIT_TIMEOUT);
    ROS_DEBUG("called vehicle->control->disArmMotors()");
  }

  ROS_DEBUG("ack.info: set=%i id=%i", ack.info.cmd_set, ack.info.cmd_id);
  ROS_DEBUG("ack.data: %i", ack.data);

  response.cmd_set  = (int)ack.info.cmd_set;
  response.cmd_id   = (int)ack.info.cmd_id;
  response.ack_data = (unsigned int)ack.data;

  if (ACK::getError(ack))
  {
    response.result = false;
    ACK::getErrorCodeMessage(ack, __func__);
  }
  else
  {
    response.result = true;
  }

  return true;
}

bool
DJISDKNode::sdkCtrlAuthorityCallback(
  dji_osdk_ros::SDKControlAuthority::Request&  request,
  dji_osdk_ros::SDKControlAuthority::Response& response)
{

  ROS_DEBUG("called sdkCtrlAuthorityCallback");

  ACK::ErrorCode ack;
  if (request.control_enable)
  {
    ack = vehicle->control->obtainCtrlAuthority(WAIT_TIMEOUT);
    ROS_DEBUG("called vehicle->obtainCtrlAuthority");
  }
  else
  {
    ack = vehicle->control->releaseCtrlAuthority(WAIT_TIMEOUT);
    ROS_DEBUG("called vehicle->releaseCtrlAuthority");
  }

  ROS_DEBUG("ack.info: set=%i id=%i", ack.info.cmd_set, ack.info.cmd_id);
  ROS_DEBUG("ack.data: %i", ack.data);

  response.cmd_set  = (int)ack.info.cmd_set;
  response.cmd_id   = (int)ack.info.cmd_id;
  response.ack_data = (unsigned int)ack.data;

  if (ACK::getError(ack))
  {
    response.result = false;
    ACK::getErrorCodeMessage(ack, __func__);
  }
  else
  {
    response.result = true;
  }

  return true;
}

bool
DJISDKNode::setLocalPosRefCallback(dji_osdk_ros::SetLocalPosRef::Request &request,
                                     dji_osdk_ros::SetLocalPosRef::Response &response) {
  printf("Currrent GPS health is %d \n",current_gps_health );
  if (current_gps_health > 3)
  {
    local_pos_ref_latitude = current_gps_latitude;
    local_pos_ref_longitude = current_gps_longitude;
    local_pos_ref_altitude = current_gps_altitude;
    ROS_INFO("Local Position reference has been set.");
    ROS_INFO("MONITOR GPS HEALTH WHEN USING THIS TOPIC");
    local_pos_ref_set = true;

    // Create message to publish to a topic
    sensor_msgs::NavSatFix localFrameLLA;
    localFrameLLA.latitude = local_pos_ref_latitude;
    localFrameLLA.longitude = local_pos_ref_longitude;
    localFrameLLA.altitude = local_pos_ref_altitude;
    local_frame_ref_publisher.publish(localFrameLLA);

    response.result = true;
  }
  else
  {
    ROS_INFO("Not enough GPS Satellites. ");
    ROS_INFO("Cannot set Local Position reference");
    local_pos_ref_set = false;
    response.result = false;
  }
  return true;
}

bool
DJISDKNode::droneTaskCallback(dji_osdk_ros::DroneTaskControl::Request&  request,
                              dji_osdk_ros::DroneTaskControl::Response& response)
{

  ROS_DEBUG("called droneTaskCallback");

  ACK::ErrorCode ack;
  if (request.task == 4)
  {
    // takeoff
    ack = vehicle->control->takeoff(WAIT_TIMEOUT);
    ROS_DEBUG("called vehicle->control->takeoff()");
  }
  else if (request.task == 6)
  {
    // landing
    ack = vehicle->control->land(WAIT_TIMEOUT);
    ROS_DEBUG("called vehicle->control->land()");
  }
  else if (request.task == 1)
  {
    // gohome
    ack = vehicle->control->goHome(WAIT_TIMEOUT);
    ROS_DEBUG("called vehicle->control->goHome()");
  }
  else
  {
    ROS_WARN("unknown request task in droneTaskCallback");
    response.result = false;
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
DJISDKNode::cameraActionCallback(dji_osdk_ros::CameraAction::Request&  request,
                                 dji_osdk_ros::CameraAction::Response& response)
{
  ROS_DEBUG("called cameraActionCallback");

  if (request.camera_action == 0)
  {
    vehicle->camera->shootPhoto();
    response.result = true;
  }
  else if (request.camera_action == 1)
  {
    vehicle->camera->videoStart();
    response.result = true;
  }
  else if (request.camera_action == 2)
  {
    vehicle->camera->videoStop();
    response.result = true;
  }
  else
  {
    ROS_WARN("unknown request task in cameraActionCallback");
    response.result = false;
  }

  return true;
}

bool
DJISDKNode::MFIOConfigCallback(dji_osdk_ros::MFIOConfig::Request&  request,
                               dji_osdk_ros::MFIOConfig::Response& response)
{
  ROS_DEBUG("called MFIOConfigCallback");

  vehicle->mfio->config((MFIO::MODE)request.mode,
                        (MFIO::CHANNEL)request.channel,
                        (uint32_t)request.init_on_time_us,
                        (uint16_t)request.pwm_freq, WAIT_TIMEOUT);
  return true;
}

bool
DJISDKNode::MFIOSetValueCallback(dji_osdk_ros::MFIOSetValue::Request&  request,
                                 dji_osdk_ros::MFIOSetValue::Response& response)
{
  ROS_DEBUG("called MFIOSetValueCallback");

  vehicle->mfio->setValue((MFIO::CHANNEL)request.channel,
                          (uint32_t)request.init_on_time_us, WAIT_TIMEOUT);
  return true;
}

bool
DJISDKNode::setHardsyncCallback(dji_osdk_ros::SetHardSync::Request&  request,
                                dji_osdk_ros::SetHardSync::Response& response)
{
  ROS_DEBUG("called setHardsyncCallback");
  if (request.frequency == 0)
  {
    ROS_INFO("Call setSyncFreq with parameters (freq=%d, tag=%d). Will do one "
             "time trigger...",
             request.frequency, request.tag);
    vehicle->hardSync->setSyncFreq(request.frequency, request.tag);
    response.result = true;
    return true;
  }

  // The frequency must be between 0 and 200, and be a divisor of 400
  if (request.frequency > 0 && request.frequency <= 200)
  {
    if (400 % (request.frequency) == 0)
    {
      ROS_INFO("Call setSyncFreq with parameters (freq=%d, tag=%d).",
               request.frequency, request.tag);
      vehicle->hardSync->setSyncFreq(request.frequency, request.tag);
      response.result = true;
      return true;
    }
  }

  ROS_INFO("In valid frequency!");
  response.result = false;
  return true;
}

bool DJISDKNode::queryVersionCallback(dji_osdk_ros::QueryDroneVersion::Request& request,
                                      dji_osdk_ros::QueryDroneVersion::Response& response)
{
  response.version = vehicle->getFwVersion();
  response.hardware = std::string(vehicle->getHwVersion());

  if(response.version == 0)
  {
    ROS_INFO("Failed to get a valid Firmware version from drone!");
  }

  return true;
}

#ifdef ADVANCED_SENSING
bool
DJISDKNode::stereo240pSubscriptionCallback(dji_osdk_ros::Stereo240pSubscription::Request&  request,
                                           dji_osdk_ros::Stereo240pSubscription::Response& response)
{
  ROS_DEBUG("called stereo240pSubscriptionCallback");

  if (request.unsubscribe_240p == 1)
  {
    vehicle->advancedSensing->unsubscribeStereoImages();
    response.result = true;
    ROS_INFO("unsubscribe stereo 240p images");
    return true;
  }

  AdvancedSensing::ImageSelection image_select;
  memset(&image_select, 0, sizeof(AdvancedSensing::ImageSelection));

  if (request.front_right_240p == 1)
    image_select.front_right = 1;

  if (request.front_left_240p == 1)
    image_select.front_left = 1;

  if (request.down_front_240p == 1)
    image_select.down_front = 1;

  if (request.down_back_240p == 1)
    image_select.down_back = 1;

  this->stereo_subscription_success = false;
  vehicle->advancedSensing->subscribeStereoImages(&image_select, &publish240pStereoImage, this);

  ros::Duration(1).sleep();

  if (this->stereo_subscription_success == true)
  {
    response.result = true;
  }
  else
  {
    response.result = false;
    ROS_WARN("Stereo 240p subscription service failed, please check your request content.");
  }

  return true;
}

bool
DJISDKNode::stereoDepthSubscriptionCallback(dji_osdk_ros::StereoDepthSubscription::Request&  request,
                                            dji_osdk_ros::StereoDepthSubscription::Response& response)
{
  ROS_DEBUG("called stereoDepthSubscriptionCallback");

  if (request.unsubscribe_240p == 1)
  {
    vehicle->advancedSensing->unsubscribeStereoImages();
    response.result = true;
    ROS_INFO("unsubscribe stereo 240p images");
    return true;
  }

  if (request.front_depth_240p == 1)
  {
    this->stereo_subscription_success = false;
    vehicle->advancedSensing->subscribeFrontStereoDisparity(&publish240pStereoImage, this);
  }
  else
  {
    ROS_WARN("no depth image is subscribed");
    return true;
  }

  ros::Duration(1).sleep();

  if (this->stereo_subscription_success == true)
  {
    response.result = true;
  }
  else
  {
    response.result = false;
    ROS_WARN("Stereo 240p subscription service failed, please check your request content.");
  }

  return true;
}



bool
DJISDKNode::stereoVGASubscriptionCallback(dji_osdk_ros::StereoVGASubscription::Request&  request,
                                          dji_osdk_ros::StereoVGASubscription::Response& response)
{
  ROS_DEBUG("called stereoVGASubscriptionCallback");

  if (request.unsubscribe_vga == 1)
  {
    vehicle->advancedSensing->unsubscribeVGAImages(request.front_vga);
    response.result = true;
    ROS_INFO("unsubscribe stereo vga images");
    latest_camera_ = -1;
    return true;
  }

  if (request.vga_freq != request.VGA_20_HZ
      && request.vga_freq != request.VGA_10_HZ)
  {
    ROS_ERROR("VGA subscription frequency is wrong");
    response.result = false;
    return true;
  }
	
	
	/* In this version of the function, request.front_vga represent the side of the camera according to the followings 	
	 *	1 ---------------------> front
   *	2 ---------------------> left
   *	3 ---------------------> right
   *	4 ---------------------> rear
   *	5 ---------------------> down
   *	6 ---------------------> top 
   */
  if (request.front_vga > 0 && request.front_vga <= 6)
  {
  	//un-subscribe from the previous stream first
  	if(request.front_vga != latest_camera_ && latest_camera_ > 0)
  	{
  		vehicle->advancedSensing->unsubscribeVGAImages(latest_camera_);
		  ROS_INFO("unsubscribe stereo vga images");
		  latest_camera_ = -1;
		  ros::Duration(1).sleep();
  	}
  
    this->stereo_vga_subscription_success = false;
    vehicle->advancedSensing->subscribeFrontStereoVGA(request.vga_freq, request.front_vga, &publishVGAStereoImage, this);
    ros::Duration(1).sleep();
  }

  if (this->stereo_vga_subscription_success == true)
  {
    response.result = true;
    //if the service call returns success, only then update the buffer
    latest_camera_ = request.front_vga;
  }
  else
  {
    response.result = false;
    ROS_WARN("Stereo VGA subscription service failed, please check your request content.");
  }

  return true;
}


bool
DJISDKNode::setupCameraStreamCallback(dji_osdk_ros::SetupCameraStream::Request&  request,
                                      dji_osdk_ros::SetupCameraStream::Response& response)
{
  ROS_DEBUG("called cameraStreamCallback");
  bool result = false;

  if(request.cameraType == request.FPV_CAM)
  {
    if(request.start == 1)
    {
      result = vehicle->advancedSensing->startFPVCameraStream(&publishFPVCameraImage, this);
    }
    else
    {
      vehicle->advancedSensing->stopFPVCameraStream();
      result = true;
    }
  }
  else if(request.cameraType == request.MAIN_CAM)
  {
    if(request.start == 1)
    {
      result = vehicle->advancedSensing->startMainCameraStream(&publishMainCameraImage, this);
    }
    else
    {
      vehicle->advancedSensing->stopMainCameraStream();
      result = true;
    }
  }

  response.result = result;
  return true;
}

#endif // ADVANCED_SENSING
