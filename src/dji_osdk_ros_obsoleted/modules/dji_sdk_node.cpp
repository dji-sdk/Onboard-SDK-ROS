/** @file dji_sdk_node.cpp
 *  @version 3.3
 *  @date May, 2017
 *
 *  @brief
 *  Implementation of the initialization functions of DJISDKNode
 *
 *  @copyright 2017 DJI. All rights reserved.
 *
 */

#include <dji_sdk/dji_sdk_node.h>

using namespace DJI::OSDK;

DJISDKNode::DJISDKNode(ros::NodeHandle& nh, ros::NodeHandle& nh_private, int argc, char** argv)
  : telemetry_from_fc(USE_BROADCAST),
    R_FLU2FRD(tf::Matrix3x3(1,  0,  0, 0, -1,  0, 0,  0, -1)),
    R_ENU2NED(tf::Matrix3x3(0,  1,  0, 1,  0,  0, 0,  0, -1)),
    curr_align_state(UNALIGNED)
{
  nh_private.param("acm_name",      acm_device, std::string("/dev/ttyACM0"));
  nh_private.param("serial_name",   serial_device, std::string("/dev/ttyUSB0"));
  nh_private.param("baud_rate",     baud_rate, 921600);
  nh_private.param("app_id",        app_id,    123456);
  nh_private.param("app_version",   app_version, 1);
  nh_private.param("enc_key",       enc_key, std::string("abcd1234"));
  nh_private.param("drone_version", drone_version, std::string("M100")); // choose M100 as default
  nh_private.param("gravity_const", gravity_const, 9.801);
  nh_private.param("align_time",    align_time_with_FC, false);
  nh_private.param("use_broadcast", user_select_broadcast, false);
  nh_private.param("enable_advanced_sensing", enable_advanced_sensing, true);

  //! Default values for local Position
  local_pos_ref_latitude  = 0;
  local_pos_ref_longitude = 0;
  local_pos_ref_altitude  = 0;
  local_pos_ref_set       = false;

  //! RTK support check
  rtkSupport = false;

  // @todo need some error handling for init functions
  //! @note parsing launch file to get environment parameters
  if (!initVehicle(nh_private, argc, argv))
  {
    ROS_ERROR("Vehicle initialization failed");
  }

  else
  {
    if (!initServices(nh))
    {
      ROS_ERROR("initServices failed");
    }

    if (!initFlightControl(nh))
    {
      ROS_ERROR("initFlightControl failed");
    }

    if (!initSubscriber(nh))
    {
      ROS_ERROR("initSubscriber failed");
    }

    if (!initPublisher(nh))
    {
      ROS_ERROR("initPublisher failed");
    }
  }
}

DJISDKNode::~DJISDKNode()
{
  if(!isM100())
  {
    cleanUpSubscribeFromFC();
  }

#ifdef ADVANCED_SENSING
	if(latest_camera_ > 0)
	{
		vehicle->advancedSensing->unsubscribeVGAImages(latest_camera_);
		ROS_INFO("Unsubscribed from the VGA Stream");
	}
#endif

  if (vehicle)
  {
    delete vehicle;
  }
}

bool
DJISDKNode::initVehicle(ros::NodeHandle& nh_private, int argc, char** argv)
{
  bool threadSupport = true;
  bool advanced_sensing = false;

#ifdef ADVANCED_SENSING
  advanced_sensing = enable_advanced_sensing;
  ROS_INFO("Advanced Sensing is %s on M210.", advanced_sensing ? "Enabled" : "Disabled");
#endif

  //! @note currently does not work without thread support
  this->linuxEnvironment = new LinuxSetup(argc, argv);
  this->vehicle = linuxEnvironment->getVehicle();

  if (NULL != vehicle->subscribe && (!user_select_broadcast))
  {
    telemetry_from_fc = USE_SUBSCRIBE;
  }

#ifdef ADVANCED_SENSING
if (vehicle->advancedSensing)
{
  vehicle->advancedSensing->setAcmDevicePath(acm_device.c_str());
}
#endif

  return true;
}

// clang-format off
bool DJISDKNode::initServices(ros::NodeHandle& nh) {
  // Common to A3/N3 and M100
  drone_activation_server   = nh.advertiseService("dji_osdk_ros/activation",                     &DJISDKNode::droneActivationCallback,        this);
  drone_arm_server          = nh.advertiseService("dji_osdk_ros/drone_arm_control",              &DJISDKNode::droneArmCallback,               this);
  drone_task_server         = nh.advertiseService("dji_osdk_ros/drone_task_control",             &DJISDKNode::droneTaskCallback,              this);
  sdk_ctrlAuthority_server  = nh.advertiseService("dji_osdk_ros/sdk_control_authority",          &DJISDKNode::sdkCtrlAuthorityCallback,       this);
  camera_action_server      = nh.advertiseService("dji_osdk_ros/camera_action",                  &DJISDKNode::cameraActionCallback,           this);
  waypoint_upload_server    = nh.advertiseService("dji_osdk_ros/mission_waypoint_upload",        &DJISDKNode::missionWpUploadCallback,        this);
  waypoint_action_server    = nh.advertiseService("dji_osdk_ros/mission_waypoint_action",        &DJISDKNode::missionWpActionCallback,        this);
  waypoint_getInfo_server   = nh.advertiseService("dji_osdk_ros/mission_waypoint_getInfo",       &DJISDKNode::missionWpGetInfoCallback,       this);
  waypoint_getSpeed_server  = nh.advertiseService("dji_osdk_ros/mission_waypoint_getSpeed",      &DJISDKNode::missionWpGetSpeedCallback,      this);
  waypoint_setSpeed_server  = nh.advertiseService("dji_osdk_ros/mission_waypoint_setSpeed",      &DJISDKNode::missionWpSetSpeedCallback,      this);
  hotpoint_upload_server    = nh.advertiseService("dji_osdk_ros/mission_hotpoint_upload",        &DJISDKNode::missionHpUploadCallback,        this);
  hotpoint_action_server    = nh.advertiseService("dji_osdk_ros/mission_hotpoint_action",        &DJISDKNode::missionHpActionCallback,        this);
  hotpoint_getInfo_server   = nh.advertiseService("dji_osdk_ros/mission_hotpoint_getInfo",       &DJISDKNode::missionHpGetInfoCallback,       this);
  hotpoint_setSpeed_server  = nh.advertiseService("dji_osdk_ros/mission_hotpoint_updateYawRate", &DJISDKNode::missionHpUpdateYawRateCallback, this);
  hotpoint_resetYaw_server  = nh.advertiseService("dji_osdk_ros/mission_hotpoint_resetYaw",      &DJISDKNode::missionHpResetYawCallback,      this);
  hotpoint_setRadius_server = nh.advertiseService("dji_osdk_ros/mission_hotpoint_updateRadius",  &DJISDKNode::missionHpUpdateRadiusCallback,  this);
  mission_status_server     = nh.advertiseService("dji_osdk_ros/mission_status",                 &DJISDKNode::missionStatusCallback,          this);
  send_to_mobile_server     = nh.advertiseService("dji_osdk_ros/send_data_to_mobile",            &DJISDKNode::sendToMobileCallback,           this);
  send_to_payload_server    = nh.advertiseService("dji_osdk_ros/send_data_to_payload",           &DJISDKNode::sendToPayloadCallback,          this);
  query_version_server      = nh.advertiseService("dji_osdk_ros/query_drone_version",            &DJISDKNode::queryVersionCallback,           this);
  local_pos_ref_server      = nh.advertiseService("dji_osdk_ros/set_local_pos_ref",              &DJISDKNode::setLocalPosRefCallback,         this);
#ifdef ADVANCED_SENSING
  subscribe_stereo_240p_server  = nh.advertiseService("dji_osdk_ros/stereo_240p_subscription",   &DJISDKNode::stereo240pSubscriptionCallback, this);
  subscribe_stereo_depth_server = nh.advertiseService("dji_osdk_ros/stereo_depth_subscription",  &DJISDKNode::stereoDepthSubscriptionCallback,this);
  subscribe_stereo_vga_server   = nh.advertiseService("dji_osdk_ros/stereo_vga_subscription",    &DJISDKNode::stereoVGASubscriptionCallback,  this);
  camera_stream_server          = nh.advertiseService("dji_osdk_ros/setup_camera_stream",        &DJISDKNode::setupCameraStreamCallback,      this);
#endif

  // A3/N3 only
  if(!isM100())
  {
    set_hardsync_server   = nh.advertiseService("dji_osdk_ros/set_hardsyc", &DJISDKNode::setHardsyncCallback, this);
    mfio_config_server    = nh.advertiseService("dji_osdk_ros/mfio_config", &DJISDKNode::MFIOConfigCallback, this);
    mfio_set_value_server = nh.advertiseService("dji_osdk_ros/mfio_set_value", &DJISDKNode::MFIOSetValueCallback, this);
  }
  return true;
}
// clang-format on

bool
DJISDKNode::initFlightControl(ros::NodeHandle& nh)
{
  flight_control_sub = nh.subscribe<sensor_msgs::Joy>(
    "dji_osdk_ros/flight_control_setpoint_generic", 10,
    &DJISDKNode::flightControlSetpointCallback,   this);

  flight_control_position_yaw_sub =
    nh.subscribe<sensor_msgs::Joy>(
      "dji_osdk_ros/flight_control_setpoint_ENUposition_yaw", 10,
      &DJISDKNode::flightControlPxPyPzYawCallback, this);

  flight_control_velocity_yawrate_sub =
    nh.subscribe<sensor_msgs::Joy>(
      "dji_osdk_ros/flight_control_setpoint_ENUvelocity_yawrate", 10,
      &DJISDKNode::flightControlVxVyVzYawrateCallback, this);

  flight_control_rollpitch_yawrate_vertpos_sub =
    nh.subscribe<sensor_msgs::Joy>(
      "dji_osdk_ros/flight_control_setpoint_rollpitch_yawrate_zposition", 10,
      &DJISDKNode::flightControlRollPitchPzYawrateCallback, this);

  return true;
}

bool DJISDKNode::isM100()
{
  return(vehicle->isM100());
}


ACK::ErrorCode
DJISDKNode::activate(int l_app_id, std::string l_enc_key)
{
  usleep(1000000);
  Vehicle::ActivateData testActivateData;
  char                  app_key[65];
  testActivateData.encKey = app_key;
  strcpy(testActivateData.encKey, l_enc_key.c_str());
  testActivateData.ID = l_app_id;

  ROS_DEBUG("called vehicle->activate(&testActivateData, WAIT_TIMEOUT)");
  return vehicle->activate(&testActivateData, WAIT_TIMEOUT);
}

bool
DJISDKNode::initSubscriber(ros::NodeHandle& nh)
{
  gimbal_angle_cmd_subscriber = nh.subscribe<dji_osdk_ros::Gimbal>(
    "dji_osdk_ros/gimbal_angle_cmd", 10, &DJISDKNode::gimbalAngleCtrlCallback, this);
  gimbal_speed_cmd_subscriber = nh.subscribe<geometry_msgs::Vector3Stamped>(
    "dji_osdk_ros/gimbal_speed_cmd", 10, &DJISDKNode::gimbalSpeedCtrlCallback, this);
  return true;
}

bool
DJISDKNode::initPublisher(ros::NodeHandle& nh)
{
  rc_publisher = nh.advertise<sensor_msgs::Joy>("dji_osdk_ros/rc", 10);

  relative_position_publisher = nh.advertise<dji_osdk_ros::RelPosition>("dji_osdk_ros/relative_position", 10);

  // device control status, 0=RC, 1=MSDK, 2=OSDK
  device_status_publisher = nh.advertise<std_msgs::UInt8>("dji_osdk_ros/control_status", 10);

  attitude_publisher =
    nh.advertise<geometry_msgs::QuaternionStamped>("dji_osdk_ros/attitude", 10);

  battery_state_publisher =
    nh.advertise<sensor_msgs::BatteryState>("dji_osdk_ros/battery_state",10);

  /*!
   * - Fused attitude (duplicated from attitude topic)
   * - Raw linear acceleration (body frame: FLU, m/s^2)
   *       Z value is +9.8 when placed on level ground statically
   * - Raw angular velocity (body frame: FLU, rad/s)
   */
  imu_publisher = nh.advertise<sensor_msgs::Imu>("dji_osdk_ros/imu", 10);

  // Refer to dji_sdk.h for different enums for M100 and A3/N3
  flight_status_publisher =
    nh.advertise<std_msgs::UInt8>("dji_osdk_ros/flight_status", 10);

  /*!
   * gps_health needs to be greater than 3 for gps_position and velocity topics
   * to be trusted
   */
  gps_health_publisher =
    nh.advertise<std_msgs::UInt8>("dji_osdk_ros/gps_health", 10);

  /*!
   * NavSatFix specs:
   *   Latitude [degrees]. Positive is north of equator; negative is south.
   *   Longitude [degrees]. Positive is east of prime meridian; negative is
   * west.
   *   Altitude [m]. Positive is above the WGS 84 ellipsoid
   */
  gps_position_publisher =
    nh.advertise<sensor_msgs::NavSatFix>("dji_osdk_ros/gps_position", 10);

  /*!
   *   x [m]. Positive along navigation frame x axis
   *   y [m]. Positive along navigation frame y axis
   *   z [m]. Positive is down
   *   For details about navigation frame, please see telemetry documentation in API reference
  */
  vo_position_publisher =
          nh.advertise<dji_osdk_ros::VOPosition>("dji_osdk_ros/vo_position", 10);
  /*!
   * Height above home altitude. It is valid only after drone
   * is armed.
   */
  height_publisher =
    nh.advertise<std_msgs::Float32>("dji_osdk_ros/height_above_takeoff", 10);

  velocity_publisher =
    nh.advertise<geometry_msgs::Vector3Stamped>("dji_osdk_ros/velocity", 10);

  from_mobile_data_publisher =
    nh.advertise<dji_osdk_ros::MobileData>("dji_osdk_ros/from_mobile_data", 10);

  from_payload_data_publisher =
    nh.advertise<dji_osdk_ros::PayloadData>("dji_osdk_ros/from_payload_data", 10);

  // TODO: documentation and proper frame id
  gimbal_angle_publisher =
    nh.advertise<geometry_msgs::Vector3Stamped>("dji_osdk_ros/gimbal_angle", 10);

  local_position_publisher =
      nh.advertise<geometry_msgs::PointStamped>("dji_osdk_ros/local_position", 10);

  local_frame_ref_publisher =
      nh.advertise<sensor_msgs::NavSatFix>("dji_osdk_ros/local_frame_ref", 10, true);

  time_sync_nmea_publisher =
      nh.advertise<nmea_msgs::Sentence>("dji_osdk_ros/time_sync_nmea_msg", 10);

  time_sync_gps_utc_publisher =
      nh.advertise<dji_osdk_ros::GPSUTC>("dji_osdk_ros/time_sync_gps_utc", 10);

  time_sync_fc_utc_publisher =
      nh.advertise<dji_osdk_ros::FCTimeInUTC>("dji_osdk_ros/time_sync_fc_time_utc", 10);

  time_sync_pps_source_publisher =
      nh.advertise<std_msgs::String>("dji_osdk_ros/time_sync_pps_source", 10);

#ifdef ADVANCED_SENSING
  stereo_240p_front_left_publisher =
    nh.advertise<sensor_msgs::Image>("dji_osdk_ros/stereo_240p_front_left_images", 10);

  stereo_240p_front_right_publisher =
    nh.advertise<sensor_msgs::Image>("dji_osdk_ros/stereo_240p_front_right_images", 10);

  stereo_240p_down_front_publisher =
    nh.advertise<sensor_msgs::Image>("dji_osdk_ros/stereo_240p_down_front_images", 10);

  stereo_240p_down_back_publisher =
    nh.advertise<sensor_msgs::Image>("dji_osdk_ros/stereo_240p_down_back_images", 10);

  stereo_240p_front_depth_publisher =
    nh.advertise<sensor_msgs::Image>("dji_osdk_ros/stereo_240p_front_depth_images", 10);

  stereo_vga_front_left_publisher =
    nh.advertise<sensor_msgs::Image>("dji_osdk_ros/stereo_vga/left/image_raw", 10);
    
  left_camera_info_pub_ = 
  	nh.advertise<sensor_msgs::CameraInfo>("/dji_osdk_ros/stereo_vga/left/camera_info",10);

  stereo_vga_front_right_publisher =
    nh.advertise<sensor_msgs::Image>("dji_osdk_ros/stereo_vga/right/image_raw", 10);
  
  right_camera_info_pub_ = 
  	nh.advertise<sensor_msgs::CameraInfo>("/dji_osdk_ros/stereo_vga/right/camera_info",10);

  main_camera_stream_publisher =
    nh.advertise<sensor_msgs::Image>("dji_osdk_ros/main_camera_images", 10);

  fpv_camera_stream_publisher =
    nh.advertise<sensor_msgs::Image>("dji_osdk_ros/fpv_camera_images", 10);
    
  
  
  
#endif



  if (telemetry_from_fc == USE_BROADCAST)
  {
    ACK::ErrorCode broadcast_set_freq_ack;
    ROS_INFO("Use legacy data broadcast to get telemetry data!");

    uint8_t defaultFreq[16];

    if(isM100())
    {
      setUpM100DefaultFreq(defaultFreq);
    }
    else
    {
      setUpA3N3DefaultFreq(defaultFreq);
    }

    broadcast_set_freq_ack =
      vehicle->broadcast->setBroadcastFreq(defaultFreq, WAIT_TIMEOUT);
//      vehicle->broadcast->setBroadcastFreqDefaults(WAIT_TIMEOUT);

    if (ACK::getError(broadcast_set_freq_ack))
    {
      ACK::getErrorCodeMessage(broadcast_set_freq_ack, __func__);
      return false;
    }
    // register a callback function whenever a broadcast data is in
    vehicle->broadcast->setUserBroadcastCallback(
      &DJISDKNode::SDKBroadcastCallback, this);
  }
  else if (telemetry_from_fc == USE_SUBSCRIBE)
  {
    ROS_INFO("Use data subscription to get telemetry data!");
    if(!align_time_with_FC)
    {
      ROS_INFO("align_time_with_FC set to false. We will use ros time to time stamp messages!");
    }
    else
    {
      ROS_INFO("align_time_with_FC set to true. We will time stamp messages based on flight controller time!");
    }

    // Extra topics that is only available from subscription

    // Details can be found in DisplayMode enum in dji_sdk.h
    displaymode_publisher =
      nh.advertise<std_msgs::UInt8>("dji_osdk_ros/display_mode", 10);

    angularRate_publisher =
      nh.advertise<geometry_msgs::Vector3Stamped>("dji_osdk_ros/angular_velocity_fused", 10);

    acceleration_publisher =
      nh.advertise<geometry_msgs::Vector3Stamped>("dji_osdk_ros/acceleration_ground_fused", 10);

    trigger_publisher = nh.advertise<sensor_msgs::TimeReference>("dji_osdk_ros/trigger_time", 10);

    if (!initDataSubscribeFromFC(nh))
    {
      return false;
    }
  }
  vehicle->mobileDevice->setFromMSDKCallback(&DJISDKNode::SDKfromMobileDataCallback,
                                    this);
  if (vehicle->payloadDevice)
  {
    vehicle->payloadDevice->setFromPSDKCallback(&DJISDKNode::SDKfromPayloadDataCallback, this);
  }

  if (vehicle->hardSync)
  {
    vehicle->hardSync->subscribeNMEAMsgs(NMEACallback, this);
    vehicle->hardSync->subscribeUTCTime(GPSUTCTimeCallback, this);
    vehicle->hardSync->subscribeFCTimeInUTCRef(FCTimeInUTCCallback, this);
    vehicle->hardSync->subscribePPSSource(PPSSourceCallback, this);
  }
  return true;
}

bool
DJISDKNode::initDataSubscribeFromFC(ros::NodeHandle& nh)
{
  ACK::ErrorCode ack = vehicle->subscribe->verify(WAIT_TIMEOUT);
  if (ACK::getError(ack))
  {
    return false;
  }

  std::vector<Telemetry::TopicName> topicList100Hz;
  topicList100Hz.push_back(Telemetry::TOPIC_QUATERNION);
  topicList100Hz.push_back(Telemetry::TOPIC_ACCELERATION_GROUND);
  topicList100Hz.push_back(Telemetry::TOPIC_ANGULAR_RATE_FUSIONED);

  int nTopic100Hz    = topicList100Hz.size();
  if (vehicle->subscribe->initPackageFromTopicList(PACKAGE_ID_100HZ, nTopic100Hz,
                                                   topicList100Hz.data(), 1, 100))
  {
    ack = vehicle->subscribe->startPackage(PACKAGE_ID_100HZ, WAIT_TIMEOUT);
    if (ACK::getError(ack))
    {
      vehicle->subscribe->removePackage(PACKAGE_ID_100HZ, WAIT_TIMEOUT);
      ROS_ERROR("Failed to start 100Hz package");
      return false;
    }
    else
    {
      vehicle->subscribe->registerUserPackageUnpackCallback(
              PACKAGE_ID_100HZ, publish100HzData, this);
    }
  }

  std::vector<Telemetry::TopicName> topicList50Hz;
  // 50 Hz package from FC
  topicList50Hz.push_back(Telemetry::TOPIC_GPS_FUSED);
  topicList50Hz.push_back(Telemetry::TOPIC_ALTITUDE_FUSIONED);
  topicList50Hz.push_back(Telemetry::TOPIC_HEIGHT_FUSION);
  topicList50Hz.push_back(Telemetry::TOPIC_STATUS_FLIGHT);
  topicList50Hz.push_back(Telemetry::TOPIC_STATUS_DISPLAYMODE);
  topicList50Hz.push_back(Telemetry::TOPIC_GIMBAL_ANGLES);
  topicList50Hz.push_back(Telemetry::TOPIC_GIMBAL_STATUS);
  topicList50Hz.push_back(Telemetry::TOPIC_RC);
  topicList50Hz.push_back(Telemetry::TOPIC_VELOCITY);
  topicList50Hz.push_back(Telemetry::TOPIC_GPS_CONTROL_LEVEL);

  if(vehicle->getFwVersion() > versionBase33)
  {
    topicList50Hz.push_back(Telemetry::TOPIC_POSITION_VO);
    topicList50Hz.push_back(Telemetry::TOPIC_AVOID_DATA);
    topicList50Hz.push_back(Telemetry::TOPIC_CONTROL_DEVICE);
    topicList50Hz.push_back(Telemetry::TOPIC_RC_WITH_FLAG_DATA);
    topicList50Hz.push_back(Telemetry::TOPIC_FLIGHT_ANOMALY);

    // A3 and N3 has access to more buttons on RC
    std::string hardwareVersion(vehicle->getHwVersion());
    if( (hardwareVersion == std::string(Version::N3)) || hardwareVersion == std::string(Version::A3))
    {
      topicList50Hz.push_back(Telemetry::TOPIC_RC_FULL_RAW_DATA);      
    }

    // Advertise rc connection status only if this topic is supported by FW
    rc_connection_status_publisher =
            nh.advertise<std_msgs::UInt8>("dji_osdk_ros/rc_connection_status", 10);

    flight_anomaly_publisher =
            nh.advertise<dji_osdk_ros::FlightAnomaly>("dji_osdk_ros/flight_anomaly", 10);
  }

  int nTopic50Hz    = topicList50Hz.size();
  if (vehicle->subscribe->initPackageFromTopicList(PACKAGE_ID_50HZ, nTopic50Hz,
                                                   topicList50Hz.data(), 1, 50))
  {
    ack = vehicle->subscribe->startPackage(PACKAGE_ID_50HZ, WAIT_TIMEOUT);
    if (ACK::getError(ack))
    {
      vehicle->subscribe->removePackage(PACKAGE_ID_50HZ, WAIT_TIMEOUT);
      ROS_ERROR("Failed to start 50Hz package");
      return false;
    }
    else
    {
      vehicle->subscribe->registerUserPackageUnpackCallback(
              PACKAGE_ID_50HZ, publish50HzData, (UserData) this);
    }
  }

  //! Check if RTK is supported in the FC
  Telemetry::TopicName topicRTKSupport[] =
  {
    Telemetry::TOPIC_RTK_POSITION
  };

  int nTopicRTKSupport    = sizeof(topicRTKSupport)/sizeof(topicRTKSupport[0]);
  if (vehicle->subscribe->initPackageFromTopicList(PACKAGE_ID_5HZ, nTopicRTKSupport,
                                                   topicRTKSupport, 1, 5))
  {
    ack = vehicle->subscribe->startPackage(PACKAGE_ID_5HZ, WAIT_TIMEOUT);
    if (ack.data == ErrorCode::SubscribeACK::SOURCE_DEVICE_OFFLINE)
    {
      rtkSupport = false;
      ROS_INFO("Flight Controller does not support RTK");
    }
    else
    {
      rtkSupport = true;
      vehicle->subscribe->removePackage(PACKAGE_ID_5HZ, WAIT_TIMEOUT);
    }
  }

  std::vector<Telemetry::TopicName> topicList5hz;
  topicList5hz.push_back(Telemetry::TOPIC_GPS_DATE);
  topicList5hz.push_back(Telemetry::TOPIC_GPS_TIME);
  topicList5hz.push_back(Telemetry::TOPIC_GPS_POSITION);
  topicList5hz.push_back(Telemetry::TOPIC_GPS_VELOCITY);
  topicList5hz.push_back(Telemetry::TOPIC_GPS_DETAILS);
  topicList5hz.push_back(Telemetry::TOPIC_BATTERY_INFO);

  if(rtkSupport)
  {
    topicList5hz.push_back(Telemetry::TOPIC_RTK_POSITION);
    topicList5hz.push_back(Telemetry::TOPIC_RTK_VELOCITY);
    topicList5hz.push_back(Telemetry::TOPIC_RTK_YAW);
    topicList5hz.push_back(Telemetry::TOPIC_RTK_YAW_INFO);
    topicList5hz.push_back(Telemetry::TOPIC_RTK_POSITION_INFO);

    // Advertise rtk data only when rtk is supported
    rtk_position_publisher =
            nh.advertise<sensor_msgs::NavSatFix>("dji_osdk_ros/rtk_position", 10);

    rtk_velocity_publisher =
            nh.advertise<geometry_msgs::Vector3Stamped>("dji_osdk_ros/rtk_velocity", 10);

    rtk_yaw_publisher =
            nh.advertise<std_msgs::Int16>("dji_osdk_ros/rtk_yaw", 10);

    rtk_position_info_publisher =
            nh.advertise<std_msgs::UInt8>("dji_osdk_ros/rtk_info_position", 10);

    rtk_yaw_info_publisher =
            nh.advertise<std_msgs::UInt8>("dji_osdk_ros/rtk_info_yaw", 10);

    if(vehicle->getFwVersion() > versionBase33)
    {
      topicList5hz.push_back(Telemetry::TOPIC_RTK_CONNECT_STATUS);

      // Advertise rtk connection only when rtk is supported
      rtk_connection_status_publisher =
              nh.advertise<std_msgs::UInt8>("dji_osdk_ros/rtk_connection_status", 10);
    }
  }

  int nTopic5hz    = topicList5hz.size();
  if (vehicle->subscribe->initPackageFromTopicList(PACKAGE_ID_5HZ, nTopic5hz,
                                                   topicList5hz.data(), 1, 5))
  {
    ack = vehicle->subscribe->startPackage(PACKAGE_ID_5HZ, WAIT_TIMEOUT);
    if (ACK::getError(ack))
    {
      vehicle->subscribe->removePackage(PACKAGE_ID_5HZ, WAIT_TIMEOUT);
      ROS_ERROR("Failed to start 5hz package");
      return false;
    }
    else
    {
      vehicle->subscribe->registerUserPackageUnpackCallback(
              PACKAGE_ID_5HZ, publish5HzData, (UserData) this);
    }
  }

  // 400 Hz data from FC
  std::vector<Telemetry::TopicName> topicList400Hz;
  topicList400Hz.push_back(Telemetry::TOPIC_HARD_SYNC);

  int nTopic400Hz = topicList400Hz.size();
  if (vehicle->subscribe->initPackageFromTopicList(PACKAGE_ID_400HZ, nTopic400Hz,
                                                   topicList400Hz.data(), 1, 400))
  {
    ack = vehicle->subscribe->startPackage(PACKAGE_ID_400HZ, WAIT_TIMEOUT);
    if(ACK::getError(ack))
    {
      vehicle->subscribe->removePackage(PACKAGE_ID_400HZ, WAIT_TIMEOUT);
      ROS_ERROR("Failed to start 400Hz package");
      return false;
    }
    else
    {
      vehicle->subscribe->registerUserPackageUnpackCallback(PACKAGE_ID_400HZ, publish400HzData, this);
    }
  }

  ros::Duration(1).sleep();
  return true;
}

void
DJISDKNode::cleanUpSubscribeFromFC()
{
  vehicle->subscribe->removePackage(0, WAIT_TIMEOUT);
  vehicle->subscribe->removePackage(1, WAIT_TIMEOUT);
  vehicle->subscribe->removePackage(2, WAIT_TIMEOUT);
  vehicle->subscribe->removePackage(3, WAIT_TIMEOUT);
  if (vehicle->hardSync)
  {
    vehicle->hardSync->unsubscribeNMEAMsgs();
    vehicle->hardSync->unsubscribeUTCTime();
    vehicle->hardSync->unsubscribeFCTimeInUTCRef();
    vehicle->hardSync->unsubscribePPSSource();
  }
}

//bool DJISDKNode::validateSerialDevice(LinuxSerialDevice* serialDevice)
//{
//  static const int BUFFER_SIZE = 2048;
//  //! Check the serial channel for data
//  uint8_t buf[BUFFER_SIZE];
//  if (!serialDevice->setSerialPureTimedRead())
//  {
//    ROS_ERROR("Failed to set up port for timed read.\n");
//    return (false);
//  };
//  usleep(100000);
//  if(serialDevice->serialRead(buf, BUFFER_SIZE))
//  {
//    ROS_INFO("Succeeded to read from serial device");
//  }
//  else
//  {
//    ROS_ERROR("Failed to read from serial device. The Onboard SDK is not communicating with your drone.");
//    return (false);
//  }
//
//  // All the tests passed and the serial device is properly set up
//  serialDevice->unsetSerialPureTimedRead();
//  return (true);
//}

void
DJISDKNode::setUpM100DefaultFreq(uint8_t freq[16])
{
  freq[0]  = DataBroadcast::FREQ_100HZ;
  freq[1]  = DataBroadcast::FREQ_100HZ;
  freq[2]  = DataBroadcast::FREQ_100HZ;
  freq[3]  = DataBroadcast::FREQ_50HZ;
  freq[4]  = DataBroadcast::FREQ_100HZ;
  freq[5]  = DataBroadcast::FREQ_50HZ;
  freq[6]  = DataBroadcast::FREQ_10HZ;
  freq[7]  = DataBroadcast::FREQ_50HZ;
  freq[8]  = DataBroadcast::FREQ_50HZ;
  freq[9]  = DataBroadcast::FREQ_50HZ;
  freq[10] = DataBroadcast::FREQ_10HZ;
  freq[11] = DataBroadcast::FREQ_10HZ;
}

void
DJISDKNode::setUpA3N3DefaultFreq(uint8_t freq[16])
{
  freq[0]  = DataBroadcast::FREQ_100HZ;
  freq[1]  = DataBroadcast::FREQ_100HZ;
  freq[2]  = DataBroadcast::FREQ_100HZ;
  freq[3]  = DataBroadcast::FREQ_50HZ;
  freq[4]  = DataBroadcast::FREQ_100HZ;
  freq[5]  = DataBroadcast::FREQ_50HZ;
  freq[6]  = DataBroadcast::FREQ_50HZ;
  freq[7]  = DataBroadcast::FREQ_50HZ;
  freq[8]  = DataBroadcast::FREQ_10HZ;
  freq[9]  = DataBroadcast::FREQ_50HZ;
  freq[10] = DataBroadcast::FREQ_50HZ;
  freq[11] = DataBroadcast::FREQ_50HZ;
  freq[12] = DataBroadcast::FREQ_10HZ;
  freq[13] = DataBroadcast::FREQ_10HZ;
}

void DJISDKNode::gpsConvertENU(double &ENU_x, double &ENU_y,
                                 double gps_t_lon, double gps_t_lat,
                                 double gps_r_lon, double gps_r_lat)
{
  double d_lon = gps_t_lon - gps_r_lon;
  double d_lat = gps_t_lat - gps_r_lat;
  ENU_y = DEG2RAD(d_lat) * C_EARTH;
  ENU_x = DEG2RAD(d_lon) * C_EARTH * cos(DEG2RAD(gps_t_lat));
};
