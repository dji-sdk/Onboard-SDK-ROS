/** @file dji_vehicle_node.hpp
 *  @version 4.0
 *  @date May 2020
 *
 *  @brief main node of osdk ros 4.0.All services and topics are inited here.
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
#include <dji_osdk_ros/dji_vehicle_node.h>
#include <dji_osdk_ros/vehicle_wrapper.h>
#include <vector>
//CODE
using namespace dji_osdk_ros;
const int WAIT_TIMEOUT = 10;
const int FLIGHT_CONTROL_WAIT_TIMEOUT = 1;

VehicleNode::VehicleNode(int test)
{
  initService();
}

VehicleNode::VehicleNode():telemetry_from_fc_(TelemetryType::USE_ROS_BROADCAST),
                           R_FLU2FRD_(tf::Matrix3x3(1,  0,  0, 0, -1,  0, 0,  0, -1)),
                           R_ENU2NED_(tf::Matrix3x3(0,  1,  0, 1,  0,  0, 0,  0, -1)),
                           curr_align_state_(AlignStatus::UNALIGNED)
{
  nh_.param("/vehicle_node/app_id",        app_id_, 12345);
  nh_.param("/vehicle_node/enc_key",       enc_key_, std::string("abcde123"));
  nh_.param("/vehicle_node/acm_name",      device_acm_, std::string("/dev/ttyACM0"));
  nh_.param("/vehicle_node/serial_name",   device_, std::string("/dev/ttyUSB0"));
  nh_.param("/vehicle_node/baud_rate",     baud_rate_, 921600);
  nh_.param("/vehicle_node/app_version",   app_version_, 1);
  nh_.param("/vehicle_node/drone_version", drone_version_, std::string("M300")); // choose M300 as default
  nh_.param("/vehicle_node/gravity_const", gravity_const_, 9.801);
  nh_.param("/vehicle_node/align_time",    align_time_with_FC_, false);
  nh_.param("/vehicle_node/use_broadcast", user_select_broadcast_, false);
  bool enable_ad = false;
#ifdef ADVANCED_SENSING
  enable_ad = true;
#else
  enable_ad = false;
#endif
  ptr_wrapper_ = new VehicleWrapper(app_id_, enc_key_, device_acm_, device_, baud_rate_, enable_ad);

  if(ptr_wrapper_ == nullptr)
  {
    ROS_ERROR_STREAM("Vehicle modules inited failed");
    ros::shutdown();
  }
  ROS_INFO_STREAM("VehicleNode Start");

  if (NULL != ptr_wrapper_->getVehicle()->subscribe && (!user_select_broadcast_))
  {
    telemetry_from_fc_ = TelemetryType::USE_ROS_SUBSCRIBE;
  }

  subscribeGimbalData();
  initCameraModule();
  initService();
  initTopic();
}

VehicleNode::~VehicleNode()
{
  unSubScribeGimbalData();

  if(!ptr_wrapper_->isM100() && telemetry_from_fc_ == TelemetryType::USE_ROS_SUBSCRIBE)
  {
    cleanUpSubscribeFromFC();
  }
  else if(telemetry_from_fc_ == TelemetryType::USE_ROS_BROADCAST)
  {
    int pkgIndex = static_cast<int>(SubscribePackgeIndex::BROADCAST_BUT_NEED_SUBSCRIBE);
    int timeout = 1;
    ptr_wrapper_->teardownSubscription(pkgIndex, timeout);
  }
}

bool VehicleNode::subscribeGimbalData()
{
  if(ptr_wrapper_ == nullptr)
  {
    ROS_ERROR_STREAM("Vehicle modules is nullptr");
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
  int       pkgIndex        = static_cast<int>(dji_osdk_ros::SubscribePackgeIndex::GIMBAL_SUB_PACKAGE_INDEX);
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
    std::cout << "Init Camera modules main_gimbal failed."<< std::endl;
    ErrorCode::printErrorCodeMsg(ret);
    return false;
  }
  /*! vice gimbal init */
  ret = vehicle->gimbalManager->initGimbalModule(PAYLOAD_INDEX_1,
                                                 "vice_gimbal");
  if (ret != ErrorCode::SysCommonErr::Success)
  {
    std::cout << "Init Camera modules vice_gimbal failed." << std::endl;
    ErrorCode::printErrorCodeMsg(ret);
    return false;
  }
  /*! top gimbal init */
  if (vehicle->isM300())
  {
    ret = vehicle->gimbalManager->initGimbalModule(PAYLOAD_INDEX_2,
                                                   "top_gimbal");
    if (ret != ErrorCode::SysCommonErr::Success) {
      std::cout << "Init Camera modules top_gimbal failed." << std::endl;
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
    ROS_ERROR_STREAM("Vehicle modules is nullptr");
    return true;
  }

  int pkgIndex = static_cast<int>(dji_osdk_ros::SubscribePackgeIndex::GIMBAL_SUB_PACKAGE_INDEX);
  ACK::ErrorCode subscribeStatus = ptr_wrapper_->removePackage(pkgIndex, 1);
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
    ROS_ERROR_STREAM("Vehicle modules is nullptr");
    return true;
  }
  Vehicle* vehicle = ptr_wrapper_->getVehicle();

  /*! init camera modules for cameraManager */
  /*! main camera init */
  ErrorCode::ErrorCodeType ret = ptr_wrapper_->initCameraModule(
      PAYLOAD_INDEX_0, "main_camera");
  if (ret != ErrorCode::SysCommonErr::Success) {
    DERROR("Init Camera modules main_camera failed.");
    ErrorCode::printErrorCodeMsg(ret);
    return false;
  }
  /*! vice camera init */
  ret = vehicle->cameraManager->initCameraModule(PAYLOAD_INDEX_1,
                                                 "vice_camera");
  if (ret != ErrorCode::SysCommonErr::Success) {
    DERROR("Init Camera modules vice_camera failed.");
    ErrorCode::printErrorCodeMsg(ret);
    return false;
  }
  /*! top camera init for M300 */
  if (vehicle->isM300()) {
    ret = vehicle->cameraManager->initCameraModule(PAYLOAD_INDEX_2,
                                                   "top_camera");
    if (ret != ErrorCode::SysCommonErr::Success)
    {
      DERROR("Init Camera modules top_camera failed.");
      ErrorCode::printErrorCodeMsg(ret);
      return false;
    }
  }

  return true;
}

void VehicleNode::initService()
{
  ROS_INFO_STREAM("Topic startup!");
  /*! flight control server */
  task_control_server_ = nh_.advertiseService("flight_task_control", &VehicleNode::taskCtrlCallback, this);
  set_home_altitude_server_ = nh_.advertiseService("set_go_home_altitude", &VehicleNode::setGoHomeAltitudeCallback,this);
  set_current_point_as_home_server_ = nh_.advertiseService("set_current_point_as_home", &VehicleNode::setHomeCallback,this);
  set_local_pos_reference_server_ = nh_.advertiseService("set_local_pos_reference", &VehicleNode::setLocalPosRefCallback,this);
  avoid_enable_server_ = nh_.advertiseService("enable_avoid", &VehicleNode::setAvoidCallback,this);
  upwards_avoid_enable_server_ = nh_.advertiseService("enable_upwards_avoid", &VehicleNode::setUpwardsAvoidCallback, this);

  /*! gimbal control server */
  gimbal_control_server_ = nh_.advertiseService("gimbal_task_control", &VehicleNode::gimbalCtrlCallback, this);

  /*! camera control server */
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

  /*! mfio control server */
  mfio_control_server_ = nh_.advertiseService("mfio_control", &VehicleNode::mfioCtrlCallback, this);

  /*! mobile device server */
  send_data_to_mobile_device_server_ = nh_.advertiseService("send_data_to_mobile_device", &VehicleNode::sendToMobileCallback, this);
  /*! payload device server*/
  send_data_to_payload_device_server_ = nh_.advertiseService("send_data_to_payload_device_server", &VehicleNode::sendToPayloadCallback, this);
  /*! advanced sensing server */
#ifdef ADVANCED_SENSING
  advanced_sensing_server_ = nh_.advertiseService("advanced_sensing", &VehicleNode::advancedSensingCallback,this);
#endif
  ROS_INFO_STREAM("Services startup!");
}

bool VehicleNode::initTopic()
{
  attitude_publisher_ = nh_.advertise<geometry_msgs::QuaternionStamped>("dji_osdk_ros/attitude", 10);
  battery_state_publisher_ = nh_.advertise<sensor_msgs::BatteryState>("dji_osdk_ros/battery_state",10);
  /*!
   * - Fused attitude (duplicated from attitude topic)
   * - Raw linear acceleration (body frame: FLU, m/s^2)
   *       Z value is +9.8 when placed on level ground statically
   * - Raw angular velocity (body frame: FLU, rad/s^2)
   */
  imu_publisher_ = nh_.advertise<sensor_msgs::Imu>("dji_osdk_ros/imu", 10);
  // Refer to dji_sdk.h for different enums for M100 and A3/N3
  flight_status_publisher_ = nh_.advertise<std_msgs::UInt8>("dji_osdk_ros/flight_status", 10);
  /*!
   * gps_health needs to be greater than 3 for gps_position and velocity topics
   * to be trusted
   */
  gps_health_publisher_ = nh_.advertise<std_msgs::UInt8>("dji_osdk_ros/gps_health", 10);

  /*!
   * NavSatFix specs:
   *   Latitude [degrees]. Positive is north of equator; negative is south.
   *   Longitude [degrees]. Positive is east of prime meridian; negative is
   * west.
   *   Altitude [m]. Positive is above the WGS 84 ellipsoid
   */
  gps_position_publisher_ = nh_.advertise<sensor_msgs::NavSatFix>("dji_osdk_ros/gps_position", 10);

  /*!
   *   x [m]. Positive along navigation frame x axis
   *   y [m]. Positive along navigation frame y axis
   *   z [m]. Positive is down
   *   For details about navigation frame, please see telemetry documentation in API reference
  */
  vo_position_publisher_ = nh_.advertise<dji_osdk_ros::VOPosition>("dji_osdk_ros/vo_position", 10);
  /*!
   * Height above home altitude. It is valid only after drone
   * is armed.
   */
  height_publisher_ = nh_.advertise<std_msgs::Float32>("dji_osdk_ros/height_above_takeoff", 10);
  velocity_publisher_ = nh_.advertise<geometry_msgs::Vector3Stamped>("dji_osdk_ros/velocity", 10);
  from_mobile_data_publisher_ = nh_.advertise<dji_osdk_ros::MobileData>("dji_osdk_ros/from_mobile_data", 10);
  from_payload_data_publisher_ = nh_.advertise<dji_osdk_ros::PayloadData>("dji_osdk_ros/from_payload_data", 10);
  // TODO: documentation and proper frame id
  gimbal_angle_publisher_ = nh_.advertise<geometry_msgs::Vector3Stamped>("dji_osdk_ros/gimbal_angle", 10);
  rc_publisher_ = nh_.advertise<sensor_msgs::Joy>("dji_osdk_ros/rc", 10);

  local_position_publisher_ = nh_.advertise<geometry_msgs::PointStamped>("dji_osdk_ros/local_position", 10);
  local_frame_ref_publisher_ = nh_.advertise<sensor_msgs::NavSatFix>("dji_osdk_ros/local_frame_ref", 10, true);
  time_sync_nmea_publisher_ = nh_.advertise<nmea_msgs::Sentence>("dji_osdk_ros/time_sync_nmea_msg", 10);
  time_sync_gps_utc_publisher_ = nh_.advertise<dji_osdk_ros::GPSUTC>("dji_osdk_ros/time_sync_gps_utc", 10);
  time_sync_fc_utc_publisher_ = nh_.advertise<dji_osdk_ros::FCTimeInUTC>("dji_osdk_ros/time_sync_fc_time_utc", 10);
  time_sync_pps_source_publisher_ = nh_.advertise<std_msgs::String>("dji_osdk_ros/time_sync_pps_source", 10);
  // Extra topics that is only available from subscription
  // Details can be found in DisplayMode enum in dji_sdk.h
  #ifdef ADVANCED_SENSING
    advanced_sensing_pub_ = nh_.advertise<dji_osdk_ros::CameraData>("cameradata", 1000);
  #endif

  if(ptr_wrapper_ == nullptr)
  {
    ROS_ERROR_STREAM("Vehicle modules is nullptr");
    return true;
  }

  Vehicle* vehicle = ptr_wrapper_->getVehicle();
  if (telemetry_from_fc_ == TelemetryType::USE_ROS_BROADCAST)
  {
    ACK::ErrorCode broadcast_set_freq_ack;
    ROS_INFO("Use legacy data broadcast to get telemetry data!");

    uint8_t defaultFreq[16];

    if (ptr_wrapper_->isM100()) {
      ptr_wrapper_->setUpM100DefaultFreq(defaultFreq);
    } else {
      ptr_wrapper_->setUpA3N3DefaultFreq(defaultFreq);
    }
    broadcast_set_freq_ack = ptr_wrapper_->setBroadcastFreq(defaultFreq, WAIT_TIMEOUT);

    if (ACK::getError(broadcast_set_freq_ack)) {
      ACK::getErrorCodeMessage(broadcast_set_freq_ack, __func__);
      return false;
    }
    // register a callback function whenever a broadcast data is in
    ptr_wrapper_->setUserBroadcastCallback(&VehicleNode::SDKBroadcastCallback, this);

    /*! some data still need to be subscribed*/
    int pkgIndex = static_cast<int>(SubscribePackgeIndex::BROADCAST_BUT_NEED_SUBSCRIBE);
    int freq = 10;
    int timeout = 1;
    TopicName topicList10Hz[] = { TOPIC_STATUS_FLIGHT, TOPIC_STATUS_DISPLAYMODE,TOPIC_VELOCITY,
                                 TOPIC_GPS_FUSED ,TOPIC_QUATERNION };
    int topicSize = sizeof(topicList10Hz) / sizeof(topicList10Hz[0]);
    ptr_wrapper_->setUpSubscription(pkgIndex, freq, topicList10Hz, topicSize, timeout);
  }
  else if (telemetry_from_fc_ == TelemetryType::USE_ROS_SUBSCRIBE)
  {
    ROS_INFO("Use data subscription to get telemetry data!");
    if(!align_time_with_FC_)
    {
      ROS_INFO("align_time_with_FC set to false. We will use ros time to time stamp messages!");
    }
    else
    {
      ROS_INFO("align_time_with_FC set to true. We will time stamp messages based on flight controller time!");
    }

    // Extra topics that is only available from subscription

    // Details can be found in DisplayMode enum in common_type.h
    angularRate_publisher_ = nh_.advertise<geometry_msgs::Vector3Stamped>("dji_osdk_ros/angular_velocity_fused", 10);
    acceleration_publisher_ = nh_.advertise<geometry_msgs::Vector3Stamped>("dji_osdk_ros/acceleration_ground_fused", 10);
    displaymode_publisher_ = nh_.advertise<std_msgs::UInt8>("dji_osdk_ros/display_mode", 10);
    trigger_publisher_ = nh_.advertise<sensor_msgs::TimeReference>("dji_osdk_ros/trigger_time", 10);

    if (!initDataSubscribeFromFC())
    {
      return false;
    }
  }
  ptr_wrapper_->setFromMSDKCallback(&VehicleNode::SDKfromMobileDataCallback, this);
  if (vehicle->payloadDevice)
  {
    ptr_wrapper_->setFromPSDKCallback(&VehicleNode::SDKfromPayloadDataCallback, this);
  }

  if (vehicle->hardSync)
  {
    ptr_wrapper_->subscribeNMEAMsgs(&VehicleNode::NMEACallback, this);
    ptr_wrapper_->subscribeUTCTime(&VehicleNode::GPSUTCTimeCallback, this);
    ptr_wrapper_->subscribeFCTimeInUTCRef(&VehicleNode::FCTimeInUTCCallback, this);
    ptr_wrapper_->subscribePPSSource(&VehicleNode::PPSSourceCallback, this);
  }
  return true;
}

bool VehicleNode::initDataSubscribeFromFC()
{
  if(ptr_wrapper_ == nullptr)
  {
    ROS_ERROR_STREAM("Vehicle modules is nullptr");
    return true;
  }

  ACK::ErrorCode ack = ptr_wrapper_->verify(WAIT_TIMEOUT);
  if (ACK::getError(ack))
  {
    return false;
  }

  std::vector<Telemetry::TopicName> topicList100Hz;
  topicList100Hz.push_back(Telemetry::TOPIC_QUATERNION);
  topicList100Hz.push_back(Telemetry::TOPIC_ACCELERATION_GROUND);
  topicList100Hz.push_back(Telemetry::TOPIC_ANGULAR_RATE_FUSIONED);

  int nTopic100Hz    = topicList100Hz.size();
  if (ptr_wrapper_->initPackageFromTopicList(static_cast<int>(SubscribePackgeIndex::PACKAGE_ID_100HZ), nTopic100Hz,
                                             topicList100Hz.data(), 1, 100))
  {
    ack = ptr_wrapper_->startPackage(static_cast<int>(SubscribePackgeIndex::PACKAGE_ID_100HZ), WAIT_TIMEOUT);
    if (ACK::getError(ack))
    {
      ptr_wrapper_->removePackage(static_cast<int>(SubscribePackgeIndex::PACKAGE_ID_100HZ), WAIT_TIMEOUT);
      ROS_ERROR("Failed to start 100Hz package");
      return false;
    }
    else
    {
      ptr_wrapper_->registerUserPackageUnpackCallback(static_cast<int>(SubscribePackgeIndex::PACKAGE_ID_100HZ), VehicleNode::publish100HzData, this);
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

  if(ptr_wrapper_->getFwVersion() > versionBase33)
  {
    topicList50Hz.push_back(Telemetry::TOPIC_POSITION_VO);
    topicList50Hz.push_back(Telemetry::TOPIC_RC_WITH_FLAG_DATA);
    topicList50Hz.push_back(Telemetry::TOPIC_FLIGHT_ANOMALY);

    // A3 and N3 has access to more buttons on RC
    std::string hardwareVersion(ptr_wrapper_->getHwVersion());
    if( (hardwareVersion == std::string(Version::N3)) || hardwareVersion == std::string(Version::A3))
    {
      topicList50Hz.push_back(Telemetry::TOPIC_RC_FULL_RAW_DATA);
    }

    // Advertise rc connection status only if this topic is supported by FW
    rc_connection_status_publisher_ = nh_.advertise<std_msgs::UInt8>("dji_osdk_ros/rc_connection_status", 10);
    flight_anomaly_publisher_ = nh_.advertise<dji_osdk_ros::FlightAnomaly>("dji_osdk_ros/flight_anomaly", 10);
  }

  int nTopic50Hz    = topicList50Hz.size();
  if (ptr_wrapper_->initPackageFromTopicList(static_cast<int>(SubscribePackgeIndex::PACKAGE_ID_50HZ), nTopic50Hz,
                                                   topicList50Hz.data(), 1, 50))
  {
    ack = ptr_wrapper_->startPackage(static_cast<int>(SubscribePackgeIndex::PACKAGE_ID_50HZ), WAIT_TIMEOUT);
    if (ACK::getError(ack))
    {
      ptr_wrapper_->removePackage(static_cast<int>(SubscribePackgeIndex::PACKAGE_ID_50HZ), WAIT_TIMEOUT);
      ROS_ERROR("Failed to start 50Hz package");
      return false;
    }
    else
    {
      ptr_wrapper_->registerUserPackageUnpackCallback(static_cast<int>(SubscribePackgeIndex::PACKAGE_ID_50HZ), VehicleNode::publish50HzData, (UserData) this);
    }
  }

  //! Check if RTK is supported in the FC
  Telemetry::TopicName topicRTKSupport[] = { Telemetry::TOPIC_RTK_POSITION  };

  int nTopicRTKSupport    = sizeof(topicRTKSupport)/sizeof(topicRTKSupport[0]);
  if (ptr_wrapper_->initPackageFromTopicList(static_cast<int>(SubscribePackgeIndex::PACKAGE_ID_5HZ), nTopicRTKSupport,
                                             topicRTKSupport, 1, 5))
  {
    ack = ptr_wrapper_->startPackage(static_cast<int>(SubscribePackgeIndex::PACKAGE_ID_5HZ), WAIT_TIMEOUT);
    if (ack.data == ErrorCode::SubscribeACK::SOURCE_DEVICE_OFFLINE)
    {
      rtk_support_ = false;
      ROS_INFO("Flight Controller does not support RTK");
    }
    else
    {
      rtk_support_ = true;
      ptr_wrapper_->removePackage(static_cast<int>(SubscribePackgeIndex::PACKAGE_ID_5HZ), WAIT_TIMEOUT);
    }
  }

  std::vector<Telemetry::TopicName> topicList5hz;
  topicList5hz.push_back(Telemetry::TOPIC_GPS_DATE);
  topicList5hz.push_back(Telemetry::TOPIC_GPS_TIME);
  topicList5hz.push_back(Telemetry::TOPIC_GPS_POSITION);
  topicList5hz.push_back(Telemetry::TOPIC_GPS_VELOCITY);
  topicList5hz.push_back(Telemetry::TOPIC_GPS_DETAILS);
  topicList5hz.push_back(Telemetry::TOPIC_BATTERY_INFO);

  if(rtk_support_)
  {
    topicList5hz.push_back(Telemetry::TOPIC_RTK_POSITION);
    topicList5hz.push_back(Telemetry::TOPIC_RTK_VELOCITY);
    topicList5hz.push_back(Telemetry::TOPIC_RTK_YAW);
    topicList5hz.push_back(Telemetry::TOPIC_RTK_YAW_INFO);
    topicList5hz.push_back(Telemetry::TOPIC_RTK_POSITION_INFO);

    // Advertise rtk data only when rtk is supported
    rtk_position_publisher_ = nh_.advertise<sensor_msgs::NavSatFix>("dji_osdk_ros/rtk_position", 10);
    rtk_velocity_publisher_ = nh_.advertise<geometry_msgs::Vector3Stamped>("dji_osdk_ros/rtk_velocity", 10);
    rtk_yaw_publisher_ = nh_.advertise<std_msgs::Int16>("dji_osdk_ros/rtk_yaw", 10);
    rtk_position_info_publisher_ = nh_.advertise<std_msgs::UInt8>("dji_osdk_ros/rtk_info_position", 10);
    rtk_yaw_info_publisher_ = nh_.advertise<std_msgs::UInt8>("dji_osdk_ros/rtk_info_yaw", 10);

    if(ptr_wrapper_->getFwVersion() > versionBase33)
    {
      topicList5hz.push_back(Telemetry::TOPIC_RTK_CONNECT_STATUS);

      // Advertise rtk connection only when rtk is supported
      rtk_connection_status_publisher_ = nh_.advertise<std_msgs::UInt8>("dji_osdk_ros/rtk_connection_status", 10);
    }
  }

  int nTopic5hz    = topicList5hz.size();
  if (ptr_wrapper_->initPackageFromTopicList(static_cast<int>(SubscribePackgeIndex::PACKAGE_ID_5HZ), nTopic5hz,
                                             topicList5hz.data(), 1, 5))
  {
    ack = ptr_wrapper_->startPackage(static_cast<int>(SubscribePackgeIndex::PACKAGE_ID_5HZ), WAIT_TIMEOUT);
    if (ACK::getError(ack))
    {
      ptr_wrapper_->removePackage(static_cast<int>(SubscribePackgeIndex::PACKAGE_ID_5HZ), WAIT_TIMEOUT);
      ROS_ERROR("Failed to start 5hz package");
      return false;
    }
    else
    {
      ptr_wrapper_->registerUserPackageUnpackCallback(static_cast<int>(SubscribePackgeIndex::PACKAGE_ID_5HZ), VehicleNode::publish5HzData, (UserData) this);
    }
  }

  // 400 Hz data from FC
  std::vector<Telemetry::TopicName> topicList400Hz;
  topicList400Hz.push_back(Telemetry::TOPIC_HARD_SYNC);

  int nTopic400Hz = topicList400Hz.size();
  if (ptr_wrapper_->initPackageFromTopicList(static_cast<int>(SubscribePackgeIndex::PACKAGE_ID_400HZ), nTopic400Hz,
                                             topicList400Hz.data(), 1, 400))
  {
    ack = ptr_wrapper_->startPackage(static_cast<int>(SubscribePackgeIndex::PACKAGE_ID_400HZ), WAIT_TIMEOUT);
    if(ACK::getError(ack))
    {
      ptr_wrapper_->removePackage(static_cast<int>(SubscribePackgeIndex::PACKAGE_ID_400HZ), WAIT_TIMEOUT);
      ROS_ERROR("Failed to start 400Hz package");
      return false;
    }
    else
    {
      ptr_wrapper_->registerUserPackageUnpackCallback(static_cast<int>(SubscribePackgeIndex::PACKAGE_ID_400HZ), VehicleNode::publish400HzData, this);
    }
  }

  ros::Duration(1).sleep();
  return true;
}

bool VehicleNode::cleanUpSubscribeFromFC()
{
  if(ptr_wrapper_ == nullptr)
  {
    ROS_ERROR_STREAM("Vehicle modules is nullptr");
    return false;
  }
  Vehicle* vehicle = ptr_wrapper_->getVehicle();

  ptr_wrapper_->removePackage(static_cast<int>(SubscribePackgeIndex::PACKAGE_ID_5HZ), WAIT_TIMEOUT);
  ptr_wrapper_->removePackage(static_cast<int>(SubscribePackgeIndex::PACKAGE_ID_50HZ), WAIT_TIMEOUT);
  ptr_wrapper_->removePackage(static_cast<int>(SubscribePackgeIndex::PACKAGE_ID_100HZ), WAIT_TIMEOUT);
  ptr_wrapper_->removePackage(static_cast<int>(SubscribePackgeIndex::PACKAGE_ID_400HZ), WAIT_TIMEOUT);
  if (vehicle->hardSync)
  {
    ptr_wrapper_->unsubscribeNMEAMsgs();
    ptr_wrapper_->unsubscribeUTCTime();
    ptr_wrapper_->unsubscribeFCTimeInUTCRef();
    ptr_wrapper_->unsubscribePPSSource();
  }
  return true;
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
        ROS_ERROR_STREAM("Vehicle modules is nullptr");
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
    ROS_ERROR_STREAM("Vehicle modules is nullptr");
    return true;
  }

  switch (request.task)
  {
    case FlightTaskControl::Request::TASK_GOHOME:
      {
        ROS_INFO_STREAM("call go home service");
        ptr_wrapper_->goHome(FLIGHT_CONTROL_WAIT_TIMEOUT);
        break;
      }
    case FlightTaskControl::Request::TASK_GOHOME_AND_CONFIRM_LANDING:
    {
      ROS_INFO_STREAM("call go home and confirm landing service");
      ptr_wrapper_->goHomeAndConfirmLanding(FLIGHT_CONTROL_WAIT_TIMEOUT);
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
        ptr_wrapper_->moveByPositionOffset(ack, FLIGHT_CONTROL_WAIT_TIMEOUT, tmp_offset);
        break;
      }
    case FlightTaskControl::Request::TASK_TAKEOFF:
      {
        ROS_INFO_STREAM("call takeoff service");
        ptr_wrapper_->monitoredTakeoff(ack, FLIGHT_CONTROL_WAIT_TIMEOUT);
        break;
      }
    case FlightTaskControl::Request::TASK_LAND:
      {
        ROS_INFO_STREAM("call land service");
        ptr_wrapper_->monitoredLanding(ack, FLIGHT_CONTROL_WAIT_TIMEOUT);
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
    ROS_ERROR_STREAM("Vehicle modules is nullptr");
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
    ROS_ERROR_STREAM("Vehicle modules is nullptr");
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
    ROS_ERROR_STREAM("Vehicle modules is nullptr");
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
    ROS_ERROR_STREAM("Vehicle modules is nullptr");
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
    ROS_ERROR_STREAM("Vehicle modules is nullptr");
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
    ROS_ERROR_STREAM("Vehicle modules is nullptr");
    return true;
  }
  response.result = ptr_wrapper_->setFocusPoint(static_cast<PayloadIndex>(request.payload_index), request.x, request.y);
  return response.result;
}

bool VehicleNode::cameraSetTapZoomPointCallback(CameraTapZoomPoint::Request& request, CameraTapZoomPoint::Response& response)
{
  if(ptr_wrapper_ == nullptr)
  {
    ROS_ERROR_STREAM("Vehicle modules is nullptr");
    return true;
  }
  response.result = ptr_wrapper_->setTapZoomPoint(static_cast<PayloadIndex>(request.payload_index),request.multiplier, request.x, request.y);
  return response.result;
}

bool VehicleNode::cameraZoomCtrlCallback(CameraZoomCtrl::Request& request, CameraZoomCtrl::Response& response)
{
  if(ptr_wrapper_ == nullptr)
  {
    ROS_ERROR_STREAM("Vehicle modules is nullptr");
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
    ROS_ERROR_STREAM("Vehicle modules is nullptr");
    return true;
  }
  response.result = ptr_wrapper_->startShootSinglePhoto(static_cast<PayloadIndex>(request.payload_index));
  return response.result;
}

bool VehicleNode::cameraStartShootAEBPhotoCallback(CameraStartShootAEBPhoto::Request& request, CameraStartShootAEBPhoto::Response& response)
{
  if(ptr_wrapper_ == nullptr)
  {
    ROS_ERROR_STREAM("Vehicle modules is nullptr");
    return true;
  }
  response.result = ptr_wrapper_->startShootAEBPhoto(static_cast<PayloadIndex>(request.payload_index), static_cast<PhotoAEBCount>(request.photo_aeb_count));
  return response.result;
}

bool VehicleNode::cameraStartShootBurstPhotoCallback(CameraStartShootBurstPhoto::Request& request, CameraStartShootBurstPhoto::Response& response)
{
  if(ptr_wrapper_ == nullptr)
  {
    ROS_ERROR_STREAM("Vehicle modules is nullptr");
    return true;
  }
  response.result = ptr_wrapper_->startShootBurstPhoto(static_cast<PayloadIndex>(request.payload_index),static_cast<PhotoBurstCount>(request.photo_burst_count));
  return response.result;
}

bool VehicleNode::cameraStartShootIntervalPhotoCallback(CameraStartShootIntervalPhoto::Request& request, CameraStartShootIntervalPhoto::Response& response)
{
  if(ptr_wrapper_ == nullptr)
  {
    ROS_ERROR_STREAM("Vehicle modules is nullptr");
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
    ROS_ERROR_STREAM("Vehicle modules is nullptr");
    return true;
  }
  response.result = ptr_wrapper_->shootPhotoStop(static_cast<PayloadIndex>(request.payload_index));
  return response.result;
}

bool VehicleNode::cameraRecordVideoActionCallback(CameraRecordVideoAction::Request& request, CameraRecordVideoAction::Response& response)
{
  if(ptr_wrapper_ == nullptr)
  {
    ROS_ERROR_STREAM("Vehicle modules is nullptr");
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
    ROS_ERROR_STREAM("Vehicle modules is nullptr");
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
    ROS_ERROR_STREAM("Vehicle modules is nullptr");
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
    ROS_ERROR_STREAM("Vehicle modules is nullptr");
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

bool VehicleNode::setLocalPosRefCallback(dji_osdk_ros::SetLocalPosRef::Request &request,
                                         dji_osdk_ros::SetLocalPosRef::Response &response)
{
  printf("Currrent GPS health is %d \n",current_gps_health_ );
  if (current_gps_health_ > 3)
  {
    local_pos_ref_latitude_ = current_gps_latitude_;
    local_pos_ref_longitude_ = current_gps_longitude_;
    local_pos_ref_altitude_ = current_gps_altitude_;
    ROS_INFO("Local Position reference has been set.");
    ROS_INFO("MONITOR GPS HEALTH WHEN USING THIS TOPIC");
    local_pos_ref_set_ = true;

    // Create message to publish to a topic
    sensor_msgs::NavSatFix localFrameLLA;
    localFrameLLA.latitude = local_pos_ref_latitude_;
    localFrameLLA.longitude = local_pos_ref_longitude_;
    localFrameLLA.altitude = local_pos_ref_altitude_;
    local_frame_ref_publisher_.publish(localFrameLLA);

    response.result = true;
  }
  else
  {
    ROS_INFO("Not enough GPS Satellites. ");
    ROS_INFO("Cannot set Local Position reference");
    local_pos_ref_set_ = false;
    response.result = false;
  }
  return true;
}

bool VehicleNode::setAvoidCallback(AvoidEnable::Request& request, AvoidEnable::Response& response)
{
  ROS_INFO_STREAM("Set avoid function callback");
  if(ptr_wrapper_ == nullptr)
  {
    ROS_ERROR_STREAM("Vehicle modules is nullptr");
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

bool VehicleNode::setUpwardsAvoidCallback(AvoidEnable::Request& request, AvoidEnable::Response& response)
{
  ROS_INFO_STREAM("Set upwards avoid function callback");
  if(ptr_wrapper_ == nullptr)
  {
    ROS_ERROR_STREAM("Vehicle modules is nullptr");
    return true;
  }

  if(ptr_wrapper_->setUpwardsAvoidance(request.enable) == true)
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
  VehicleNode vh_node;
  vh_node.publishTopic();

  ros::spin();
  return 0;
}
