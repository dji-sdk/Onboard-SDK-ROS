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

#ifdef OPEN_CV_INSTALLED
#include <dji_osdk_ros/stereo_utility/m300_stereo_param_tool.hpp>
#endif

#include <vector>
//CODE
using namespace dji_osdk_ros;
#define M300_FRONT_STEREO_PARAM_YAML_NAME "m300_front_stereo_param.yaml"

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

  initGimbalModule();
  initCameraModule();
  initService();
  initTopic();
}

VehicleNode::~VehicleNode()
{
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

bool VehicleNode::initGimbalModule()
{
  if(ptr_wrapper_ == nullptr)
  {
    ROS_ERROR_STREAM("Vehicle modules is nullptr");
    return false;
  }

  /*! init gimbal modules for gimbalManager */
  ErrorCode::ErrorCodeType ret;
  /*! main gimbal init */
  ret = ptr_wrapper_->initGimbalModule(dji_osdk_ros::PayloadIndex::PAYLOAD_INDEX_0, "main_gimbal");
  if (ret != ErrorCode::SysCommonErr::Success)
  {
    std::cout << "Init Camera modules main_gimbal failed."<< std::endl;
    ErrorCode::printErrorCodeMsg(ret);
    return false;
  }
  /*! vice gimbal init */
  ret = ptr_wrapper_->initGimbalModule(dji_osdk_ros::PayloadIndex::PAYLOAD_INDEX_1, "vice_gimbal");
  if (ret != ErrorCode::SysCommonErr::Success)
  {
    std::cout << "Init Camera modules vice_gimbal failed." << std::endl;
    ErrorCode::printErrorCodeMsg(ret);
    return false;
  }
  /*! top gimbal init */
  if (ptr_wrapper_->isM300())
  {
    ret = ptr_wrapper_->initGimbalModule(dji_osdk_ros::PayloadIndex::PAYLOAD_INDEX_2, "top_gimbal");
    if (ret != ErrorCode::SysCommonErr::Success) {
      std::cout << "Init Camera modules top_gimbal failed." << std::endl;
      ErrorCode::printErrorCodeMsg(ret);
      return false;
    }
  }

  return true;
}

bool VehicleNode::initCameraModule()
{
  if(ptr_wrapper_ == nullptr)
  {
    ROS_ERROR_STREAM("Vehicle modules is nullptr");
    return false;
  }

  /*! init camera modules for cameraManager */
  /*! main camera init */
  ErrorCode::ErrorCodeType ret = ptr_wrapper_->initCameraModule(
      dji_osdk_ros::PayloadIndex::PAYLOAD_INDEX_0, "main_camera");
  if (ret != ErrorCode::SysCommonErr::Success) {
    DERROR("Init Camera modules main_camera failed.");
    ErrorCode::printErrorCodeMsg(ret);
    return false;
  }
  /*! vice camera init */
  ret = ptr_wrapper_->initCameraModule(dji_osdk_ros::PayloadIndex::PAYLOAD_INDEX_1, "vice_camera");
  if (ret != ErrorCode::SysCommonErr::Success) {
    DERROR("Init Camera modules vice_camera failed.");
    ErrorCode::printErrorCodeMsg(ret);
    return false;
  }
  /*! top camera init for M300 */
  if (ptr_wrapper_->isM300()) {
    ret = ptr_wrapper_->initCameraModule(dji_osdk_ros::PayloadIndex::PAYLOAD_INDEX_2, "top_camera");
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
  /*! @brief
   *  general server
   *  @platforms M210V2, M300
   */
  get_drone_type_server_ = nh_.advertiseService("get_drone_type", &VehicleNode::getDroneTypeCallback, this);

  /*! @brief
   *  flight control server
   *  @platforms M210V2, M300
   */
  task_control_server_ = nh_.advertiseService("flight_task_control", &VehicleNode::taskCtrlCallback, this);
  joystick_action_server_ = nh_.advertiseService("joystick_action", &VehicleNode::JoystickActionCallback, this);
  set_joystick_mode_server_ = nh_.advertiseService("set_joystick_mode", &VehicleNode::setJoystickModeCallback, this);
  set_home_altitude_server_ = nh_.advertiseService("set_go_home_altitude", &VehicleNode::setGoHomeAltitudeCallback,this);
  get_home_altitude_server_ = nh_.advertiseService("get_go_home_altitude", &VehicleNode::getGoHomeAltitudeCallback,this);
  set_current_aircraft_point_as_home_server_ = nh_.advertiseService("set_current_aircraft_point_as_home",
                                               &VehicleNode::setCurrentAircraftLocAsHomeCallback,this);
  set_home_point_server_ = nh_.advertiseService("set_home_point", &VehicleNode::setHomePointCallback, this);
  set_local_pos_reference_server_ = nh_.advertiseService("set_local_pos_reference", &VehicleNode::setLocalPosRefCallback,this);
  set_horizon_avoid_enable_server_ = nh_.advertiseService("set_horizon_avoid_enable", &VehicleNode::setHorizonAvoidCallback,this);
  set_upwards_avoid_enable_server_ = nh_.advertiseService("set_upwards_avoid_enable", &VehicleNode::setUpwardsAvoidCallback, this);
  get_avoid_enable_status_server_ = nh_.advertiseService("get_avoid_enable_status", &VehicleNode::getAvoidEnableStatusCallback, this);
  obtain_releae_control_authority_server_ = nh_.advertiseService("obtain_release_control_authority",
                                            &VehicleNode::obtainReleaseControlAuthorityCallback, this);
  kill_switch_server_ = nh_.advertiseService("kill_switch", &VehicleNode::killSwitchCallback, this);
  emergency_brake_action_server_ = nh_.advertiseService("emergency_brake", &VehicleNode::emergencyBrakeCallback, this);
  /*! @brief
   *  gimbal control server
   *  @platforms M210V2, M300
   */
  gimbal_control_server_ = nh_.advertiseService("gimbal_task_control", &VehicleNode::gimbalCtrlCallback, this);

  /*! @brief
   *  camera control server
   *  @platforms M210V2, M300
   */
  camera_control_set_EV_server_ = nh_.advertiseService("camera_task_set_EV", &VehicleNode::cameraSetEVCallback, this);
  camera_control_set_shutter_speed_server_ = nh_.advertiseService("camera_task_set_shutter_speed", &VehicleNode::cameraSetShutterSpeedCallback, this);
  camera_control_set_aperture_server_ = nh_.advertiseService("camera_task_set_aperture", &VehicleNode::cameraSetApertureCallback, this);
  camera_control_set_ISO_server_ = nh_.advertiseService("camera_task_set_ISO", &VehicleNode::cameraSetISOCallback, this);
  camera_control_set_focus_point_server_ = nh_.advertiseService("camera_task_set_focus_point", &VehicleNode::cameraSetFocusPointCallback, this);
  camera_control_set_tap_zoom_point_server_ = nh_.advertiseService("camera_task_tap_zoom_point", &VehicleNode::cameraSetTapZoomPointCallback, this);
  camera_control_set_zoom_para_server_ = nh_.advertiseService("camera_task_set_zoom_para", &VehicleNode::cameraSetZoomParaCallback, this);
  camera_control_zoom_ctrl_server_ = nh_.advertiseService("camera_task_zoom_ctrl", &VehicleNode::cameraZoomCtrlCallback, this);
  camera_control_start_shoot_single_photo_server_ = nh_.advertiseService("camera_start_shoot_single_photo", &VehicleNode::cameraStartShootSinglePhotoCallback, this);
  camera_control_start_shoot_AEB_photo_server_ = nh_.advertiseService("camera_start_shoot_aeb_photo", &VehicleNode::cameraStartShootAEBPhotoCallback, this);
  camera_control_start_shoot_burst_photo_server_ = nh_.advertiseService("camera_start_shoot_burst_photo", &VehicleNode::cameraStartShootBurstPhotoCallback, this);
  camera_control_start_shoot_interval_photo_server_ = nh_.advertiseService("camera_start_shoot_interval_photo", &VehicleNode::cameraStartShootIntervalPhotoCallback, this);
  camera_control_stop_shoot_photo_server_ = nh_.advertiseService("camera_stop_shoot_photo", &VehicleNode::cameraStopShootPhotoCallback, this);
  camera_control_record_video_action_server_ = nh_.advertiseService("camera_record_video_action", &VehicleNode::cameraRecordVideoActionCallback, this);

  /* @brief
   * get whole battery info server
   * @platforms M210V2
   */
  get_whole_battery_info_server_ = nh_.advertiseService("get_whole_battery_info", &VehicleNode::getWholeBatteryInfoCallback, this);
  get_single_battery_dynamic_info_server_ = nh_.advertiseService("get_single_battery_dynamic_info", &VehicleNode::getSingleBatteryDynamicInfoCallback, this);
  
  /*! @brief
   *  mfio control server
   *  @platforms M300
   */
  get_hms_data_server_ = nh_.advertiseService("get_hms_data", &VehicleNode::getHMSDataCallback, this);
  /*! @brief
   *  mfio control server
   *  @platforms null
   */
  mfio_control_server_ = nh_.advertiseService("mfio_control", &VehicleNode::mfioCtrlCallback, this);

  /*! @brief
   *  mobile device server
   *  @platforms M210V2, M300
   */
  send_data_to_mobile_device_server_ = nh_.advertiseService("send_data_to_mobile_device", &VehicleNode::sendToMobileCallback, this);

  /*! @brief
   *  payload device server
   *  @platforms M210V2, M300
   */
  send_data_to_payload_device_server_ = nh_.advertiseService("send_data_to_payload_device_server", &VehicleNode::sendToPayloadCallback, this);

  /*! @brief
   *  advanced sensing server
   *  @platforms M210V2, M300
   */
#ifdef ADVANCED_SENSING
  setup_camera_stream_server_ = nh_.advertiseService("setup_camera_stream", &VehicleNode::setupCameraStreamCallback, this);
  setup_camera_h264_server_ = nh_.advertiseService("setup_camera_h264", &VehicleNode::setupCameraH264Callback, this);
  subscribe_stereo_240p_server_  = nh_.advertiseService("stereo_240p_subscription",   &VehicleNode::stereo240pSubscriptionCallback, this);
  subscribe_stereo_depth_server_ = nh_.advertiseService("stereo_depth_subscription",  &VehicleNode::stereoDepthSubscriptionCallback,this);
  subscribe_stereo_vga_server_   = nh_.advertiseService("stereo_vga_subscription",    &VehicleNode::stereoVGASubscriptionCallback,  this);
#ifdef OPEN_CV_INSTALLED
  /*! @brief
   *  get m300 stereo params server
   *  @platforms M300
   */
  get_m300_stereo_params_server_ = nh_.advertiseService("get_m300_stereo_params", &VehicleNode::getM300StereoParamsCallback, this);
#endif
#endif
  /*! @brief
   *  waypointV1.0 server
   *  @platforms M210
   */
  waypoint_upload_server_    = nh_.advertiseService("dji_osdk_ros/mission_waypoint_upload",        &VehicleNode::missionWpUploadCallback,        this);
  waypoint_action_server_    = nh_.advertiseService("dji_osdk_ros/mission_waypoint_action",        &VehicleNode::missionWpActionCallback,        this);
  waypoint_getInfo_server_   = nh_.advertiseService("dji_osdk_ros/mission_waypoint_getInfo",       &VehicleNode::missionWpGetInfoCallback,       this);
  waypoint_getSpeed_server_  = nh_.advertiseService("dji_osdk_ros/mission_waypoint_getSpeed",      &VehicleNode::missionWpGetSpeedCallback,      this);
  waypoint_setSpeed_server_  = nh_.advertiseService("dji_osdk_ros/mission_waypoint_setSpeed",      &VehicleNode::missionWpSetSpeedCallback,      this);

  /*! @brief
   *  hotpoint server
   *  @platforms M210, M300
   */
  hotpoint_upload_server_    = nh_.advertiseService("dji_osdk_ros/mission_hotpoint_upload",        &VehicleNode::missionHpUploadCallback,        this);
  hotpoint_action_server_    = nh_.advertiseService("dji_osdk_ros/mission_hotpoint_action",        &VehicleNode::missionHpActionCallback,        this);
  hotpoint_getInfo_server_   = nh_.advertiseService("dji_osdk_ros/mission_hotpoint_getInfo",       &VehicleNode::missionHpGetInfoCallback,       this);
  hotpoint_setSpeed_server_  = nh_.advertiseService("dji_osdk_ros/mission_hotpoint_updateYawRate", &VehicleNode::missionHpUpdateYawRateCallback, this);
  hotpoint_resetYaw_server_  = nh_.advertiseService("dji_osdk_ros/mission_hotpoint_resetYaw",      &VehicleNode::missionHpResetYawCallback,      this);
  hotpoint_setRadius_server_ = nh_.advertiseService("dji_osdk_ros/mission_hotpoint_updateRadius",  &VehicleNode::missionHpUpdateRadiusCallback,  this);
  mission_status_server_     = nh_.advertiseService("dji_osdk_ros/mission_status",                 &VehicleNode::missionStatusCallback,          this);

  /*! @brief
   *  waypoint2.0 server
   *  @platforms M300
   */
  waypointV2_init_setting_server_     = nh_.advertiseService("dji_osdk_ros/waypointV2_initSetting",    &VehicleNode::waypointV2InitSettingCallback, this);
  waypointV2_upload_mission_server_   = nh_.advertiseService("dji_osdk_ros/waypointV2_uploadMission", &VehicleNode::waypointV2UploadMissionCallback, this);
  waypointV2_download_mission_server_ = nh_.advertiseService("dji_osdk_ros/waypointV2_downloadMission", &VehicleNode::waypointV2DownloadMissionCallback, this);
  waypointV2_upload_action_server_    = nh_.advertiseService("dji_osdk_ros/waypointV2_uploadAction", &VehicleNode::waypointV2UploadActionCallback, this);
  waypointV2_start_mission_server_    = nh_.advertiseService("dji_osdk_ros/waypointV2_startMission", &VehicleNode::waypointV2StartMissionCallback, this);
  waypointV2_stop_mission_server_     = nh_.advertiseService("dji_osdk_ros/waypointV2_stopMission", &VehicleNode::waypointV2StopMissionCallback, this);
  waypointV2_pause_mission_server_    = nh_.advertiseService("dji_osdk_ros/waypointV2_pauseMission", &VehicleNode::waypointV2PauseMissionCallback, this);
  waypointV2_resume_mission_server_   = nh_.advertiseService("dji_osdk_ros/waypointV2_resumeMission", &VehicleNode::waypointV2ResumeMissionCallback, this);
  waypointV2_generate_actions_server_ = nh_.advertiseService("dji_osdk_ros/waypointV2_generateActions", &VehicleNode::waypointV2GenerateActionsCallback, this);
  waypointv2_set_global_cruisespeed_server_ = nh_.advertiseService("dji_osdk_ros/waypointV2_setGlobalCruisespeed", &VehicleNode::waypointV2SetGlobalCruisespeedCallback, this);
  waypointv2_get_global_cruisespeed_server_ = nh_.advertiseService("dji_osdk_ros/waypointV2_getGlobalCruisespeed", &VehicleNode::waypointV2GetGlobalCruisespeedCallback, this);
  waypointv2_subscribe_mission_event_server_ = nh_.advertiseService("dji_osdk_ros/waypointV2_subscribeMissionEvent", &VehicleNode::waypointV2SubscribeMissionEventCallback, this);
  waypointv2_subscribe_mission_state_server_ = nh_.advertiseService("dji_osdk_ros/waypointV2_subscribeMissionState", &VehicleNode::waypointV2SubscribeMissionStateCallback, this);

  ROS_INFO_STREAM("Services startup!");
}

bool VehicleNode::initTopic()
{
  attitude_publisher_ = nh_.advertise<geometry_msgs::QuaternionStamped>("dji_osdk_ros/attitude", 10);
/* @brief Provides various data about the battery
 * @note Most of these details need a DJI Intelligent battery to work correctly
 * (this is usually not the case with A3/N3 based setups)
 * @details Please be aware that some of the data elements in this topic may not be able to update
 * at high rates due to the limitations of the sensing for that data. e.g. current can only update @ 1 Hz.
 * @platforms M210,M300
 * @units
 * |voltage           | mV |
 * |current           | mA |
 * @datastruct \ref Battery
 */
  battery_state_publisher_ = nh_.advertise<sensor_msgs::BatteryState>("dji_osdk_ros/battery_state",10);
  /*!
   * - Fused attitude (duplicated from attitude topic)
   * - Raw linear acceleration (body frame: FLU, m/s^2)
   *       Z value is +9.8 when placed on level ground statically
   * - Raw angular velocity (body frame: FLU, rad/s)
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

  #ifdef ADVANCED_SENSING
  main_camera_stream_publisher_ = nh_.advertise<sensor_msgs::Image>("dji_osdk_ros/main_camera_images", 10);
  fpv_camera_stream_publisher_ = nh_.advertise<sensor_msgs::Image>("dji_osdk_ros/fpv_camera_images", 10);
  camera_h264_publisher_ = nh_.advertise<sensor_msgs::Image>("dji_osdk_ros/camera_h264_stream", 10);
  stereo_240p_front_left_publisher_ = nh_.advertise<sensor_msgs::Image>("dji_osdk_ros/stereo_240p_front_left_images", 10);
  stereo_240p_front_right_publisher_ = nh_.advertise<sensor_msgs::Image>("dji_osdk_ros/stereo_240p_front_right_images", 10);
  stereo_240p_down_front_publisher_ = nh_.advertise<sensor_msgs::Image>("dji_osdk_ros/stereo_240p_down_front_images", 10);
  stereo_240p_down_back_publisher_ = nh_.advertise<sensor_msgs::Image>("dji_osdk_ros/stereo_240p_down_back_images", 10);
  stereo_240p_front_depth_publisher_ = nh_.advertise<sensor_msgs::Image>("dji_osdk_ros/stereo_240p_front_depth_images", 10);
  stereo_vga_front_left_publisher_ = nh_.advertise<sensor_msgs::Image>("dji_osdk_ros/stereo_vga_front_left_images", 10);
  stereo_vga_front_right_publisher_ = nh_.advertise<sensor_msgs::Image>("dji_osdk_ros/stereo_vga_front_right_images", 10);
  #endif

  waypointV2_mission_state_publisher_ = nh_.advertise<dji_osdk_ros::WaypointV2MissionStatePush>("dji_osdk_ros/waypointV2_mission_state", 10);
  waypointV2_mission_event_publisher_ = nh_.advertise<dji_osdk_ros::WaypointV2MissionEventPush>("dji_osdk_ros/waypointV2_mission_event", 10);

  if(ptr_wrapper_ == nullptr)
  {
    ROS_ERROR_STREAM("Vehicle modules is nullptr");
    return false;
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
    int freq = 50;
    int timeout = 1;
    std::vector<Telemetry::TopicName> topicList50Hz;

    topicList50Hz.push_back(Telemetry::TOPIC_STATUS_FLIGHT);
    topicList50Hz.push_back(Telemetry::TOPIC_STATUS_DISPLAYMODE);
    topicList50Hz.push_back(Telemetry::TOPIC_VELOCITY);
    topicList50Hz.push_back(Telemetry::TOPIC_GPS_FUSED);
    topicList50Hz.push_back(Telemetry::TOPIC_QUATERNION);
    if (ptr_wrapper_->isM300())
    {
      topicList50Hz.push_back(Telemetry::TOPIC_THREE_GIMBAL_DATA);
    }
    else
    {
      topicList50Hz.push_back(Telemetry::TOPIC_DUAL_GIMBAL_DATA);

    }
    int topicSize = topicList50Hz.size();;
    ptr_wrapper_->setUpSubscription(pkgIndex, freq, topicList50Hz.data(), topicSize, timeout);
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
    return false;
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

  // acturally gimbal data is from Gimbal directly
  if (ptr_wrapper_->isM300())
  {
    topicList50Hz.push_back(Telemetry::TOPIC_THREE_GIMBAL_DATA);
  }
  else
  {
    topicList50Hz.push_back(Telemetry::TOPIC_DUAL_GIMBAL_DATA);
  }
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


#ifdef ADVANCED_SENSING
bool VehicleNode::setupCameraStreamCallback(dji_osdk_ros::SetupCameraStream::Request& request,
                                            dji_osdk_ros::SetupCameraStream::Response& response)
{
  ROS_DEBUG("called cameraStreamCallback");
  if(ptr_wrapper_ == nullptr)
  {
    ROS_ERROR_STREAM("Vehicle modules is nullptr");
    response.result = false;
  }

  ptr_wrapper_->setAcmDevicePath(device_acm_);

  if(request.cameraType == request.FPV_CAM)
  {
    if(request.start == 1)
    {
      response.result = ptr_wrapper_->startFPVCameraStream(&publishFPVCameraImage, this);
    }
    else
    {
      response.result = ptr_wrapper_->stopFPVCameraStream();
    }
  }
  else if(request.cameraType == request.MAIN_CAM)
  {
    if(request.start == 1)
    {
      response.result = ptr_wrapper_->startMainCameraStream(&publishMainCameraImage, this);
    }
    else
    {
      response.result = ptr_wrapper_->stopMainCameraStream();
    }
  }

  return response.result;
}

bool VehicleNode::setupCameraH264Callback(dji_osdk_ros::SetupCameraH264::Request& request,
                                          dji_osdk_ros::SetupCameraH264::Response& response)
{
  ROS_DEBUG("called camerah264Callback");
  if(ptr_wrapper_ == nullptr)
  {
    ROS_ERROR_STREAM("Vehicle modules is nullptr");
    response.result = false;
  }

  ptr_wrapper_->setAcmDevicePath(device_acm_);

  if (request.start == 1)
  {
    response.result = ptr_wrapper_->startH264Stream(static_cast<DJI::OSDK::LiveView::LiveViewCameraPosition>(request.request_view), &publishCameraH264, this);
  }
  else
  {
    response.result = ptr_wrapper_->stopH264Stream(static_cast<DJI::OSDK::LiveView::LiveView::LiveViewCameraPosition>(request.request_view));
  }

  return response.result;
}

bool VehicleNode::stereo240pSubscriptionCallback(dji_osdk_ros::Stereo240pSubscription::Request&  request,
                                                 dji_osdk_ros::Stereo240pSubscription::Response& response)
{
  ROS_DEBUG("called stereo240pSubscriptionCallback");

  if(ptr_wrapper_ == nullptr)
  {
      ROS_ERROR_STREAM("Vehicle modules is nullptr");
      return false;
  }

  if (request.unsubscribe_240p == 1)
  {
    ptr_wrapper_->unsubscribeStereoImages();
    response.result = true;
    ROS_INFO("unsubscribe stereo 240p images");
    return true;
  }

  dji_osdk_ros::ImageSelection image_select;
  memset(&image_select, 0, sizeof(dji_osdk_ros::ImageSelection));

  if (request.front_right_240p == 1)
    image_select.front_right = 1;

  if (request.front_left_240p == 1)
    image_select.front_left = 1;

  if (request.down_front_240p == 1)
    image_select.down_front = 1;

  if (request.down_back_240p == 1)
    image_select.down_back = 1;

  this->stereo_subscription_success = false;
  ptr_wrapper_->subscribeStereoImages(&image_select, &publish240pStereoImage, this);

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
VehicleNode::stereoDepthSubscriptionCallback(dji_osdk_ros::StereoDepthSubscription::Request&  request,
                                             dji_osdk_ros::StereoDepthSubscription::Response& response)
{
  ROS_DEBUG("called stereoDepthSubscriptionCallback");

  if(ptr_wrapper_ == nullptr)
  {
    ROS_ERROR_STREAM("Vehicle modules is nullptr");
    return false;
  }

  if (request.unsubscribe_240p == 1)
  {
    ptr_wrapper_->unsubscribeStereoImages();
    response.result = true;
    ROS_INFO("unsubscribe stereo 240p images");
    return true;
  }

  if (request.front_depth_240p == 1)
  {
    this->stereo_subscription_success = false;
    ptr_wrapper_->subscribeFrontStereoDisparity(&publish240pStereoImage, this);
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

bool VehicleNode::stereoVGASubscriptionCallback(dji_osdk_ros::StereoVGASubscription::Request&  request,
                                                dji_osdk_ros::StereoVGASubscription::Response& response)
{
  ROS_INFO("called stereoVGASubscriptionCallback");

  if(ptr_wrapper_ == nullptr)
  {
    ROS_ERROR_STREAM("Vehicle modules is nullptr");
    return false;
  }

  if (request.unsubscribe_vga == 1)
  {
    ptr_wrapper_->unsubscribeVGAImages();
    response.result = true;
    ROS_INFO("unsubscribe stereo vga images");
    return true;
  }

  if (request.vga_freq != request.VGA_20_HZ
      && request.vga_freq != request.VGA_10_HZ)
  {
    ROS_ERROR("VGA subscription frequency is wrong");
    response.result = false;
    return true;
  }

  if (request.front_vga == 1)
  {
    this->stereo_vga_subscription_success = false;
    ptr_wrapper_->subscribeFrontStereoVGA(request.vga_freq, &publishVGAStereoImage, this);
    ros::Duration(1).sleep();
  }

  if (this->stereo_vga_subscription_success == true)
  {
    response.result = true;
  }
  else
  {
    response.result = false;
    ROS_WARN("Stereo VGA subscription service failed, please check your request content.");
  }

  return true;
}

#ifdef OPEN_CV_INSTALLED
bool VehicleNode::getM300StereoParamsCallback(dji_osdk_ros::GetM300StereoParams::Request& request, 
                                              dji_osdk_ros::GetM300StereoParams::Response& response)
{
  ROS_INFO("called getM300StereoParamsCallback");

  if(ptr_wrapper_ == nullptr)
  {
    ROS_ERROR_STREAM("Vehicle modules is nullptr");
    response.result = false;
  }

  Vehicle* vehicle = ptr_wrapper_->getVehicle();
  M300StereoParamTool *tool = new M300StereoParamTool(vehicle);
  Perception::CamParamType stereoParam =
      tool->getM300stereoParams(Perception::DirectionType::RECTIFY_FRONT);
  if (tool->createStereoParamsYamlFile(M300_FRONT_STEREO_PARAM_YAML_NAME, stereoParam))
  {
    tool->setParamFileForM300(M300_FRONT_STEREO_PARAM_YAML_NAME);
    response.result = true;
  }
  else
  {
    response.result = false;
  }

  return response.result;
}
#endif
#endif

bool VehicleNode::getDroneTypeCallback(dji_osdk_ros::GetDroneType::Request &request,
                                       dji_osdk_ros::GetDroneType::Response &response)
{
  ROS_DEBUG("called getDroneTypeCallback");

  if(ptr_wrapper_ == nullptr)
  {
    ROS_ERROR_STREAM("Vehicle modules is nullptr");
    return false;
  }

  if (ptr_wrapper_->isM100())
  {
    response.drone_type = static_cast<uint8_t>(dji_osdk_ros::Dronetype::M100);
    return true;
  }
  else if (ptr_wrapper_->isM200V2())
  {
    response.drone_type = static_cast<uint8_t>(dji_osdk_ros::Dronetype::M210V2);
    return true;
  }
  else if (ptr_wrapper_->isM300())
  {
    response.drone_type = static_cast<uint8_t>(dji_osdk_ros::Dronetype::M300);
    return true;
  }
  else if (ptr_wrapper_->isM600())
  {
    response.drone_type = static_cast<uint8_t>(dji_osdk_ros::Dronetype::M600);
    return true;
  }
  else
  {
    response.drone_type = static_cast<uint8_t>(dji_osdk_ros::Dronetype::INVALID_TYPE);
    return false;
  }

}

bool VehicleNode::taskCtrlCallback(FlightTaskControl::Request&  request, FlightTaskControl::Response& response)
{
  ROS_DEBUG("called taskCtrlCallback");
  response.result = false;
  ACK::ErrorCode ack;
  if(ptr_wrapper_ == nullptr)
  {
    ROS_ERROR_STREAM("Vehicle modules is nullptr");
    return false;
  }

  switch (request.task)
  {
    case FlightTaskControl::Request::TASK_GOHOME:
     {
        ROS_INFO_STREAM("call go home service");
        if (ptr_wrapper_->goHome(FLIGHT_CONTROL_WAIT_TIMEOUT))
        {
          response.result = true;
        }
        break;
      }
    case FlightTaskControl::Request::TASK_GOHOME_AND_CONFIRM_LANDING:
      {
        ROS_INFO_STREAM("call go home and confirm landing service");
        if (ptr_wrapper_->goHomeAndConfirmLanding(FLIGHT_CONTROL_WAIT_TIMEOUT))
        {
            response.result = true;
        }
        break;
      }
    case FlightTaskControl::Request::TASK_POSITION_AND_YAW_CONTROL:
      {
        ROS_INFO_STREAM("call move local position offset service");
        dji_osdk_ros::JoystickCommand joystickCommand;
        joystickCommand.x   = request.joystickCommand.x;
        joystickCommand.y   = request.joystickCommand.y;
        joystickCommand.z   = request.joystickCommand.z;
        joystickCommand.yaw = request.joystickCommand.yaw;

        if (ptr_wrapper_->moveByPositionOffset(joystickCommand, FLIGHT_CONTROL_WAIT_TIMEOUT,
                                               request.posThresholdInM, request.yawThresholdInDeg))
        {
          response.result = true;
        }
        break;
      }
    case FlightTaskControl::Request::TASK_TAKEOFF:
      {
        ROS_INFO_STREAM("call takeoff service");
        if (ptr_wrapper_->monitoredTakeoff(FLIGHT_CONTROL_WAIT_TIMEOUT))
        {
          response.result = true;
        }
        break;
      }
    case FlightTaskControl::Request::TASK_VELOCITY_AND_YAWRATE_CONTROL:
      {
        ROS_INFO_STREAM("call velocity and yaw rate service");

        dji_osdk_ros::JoystickCommand joystickCommand;
        joystickCommand.x   = request.joystickCommand.x;
        joystickCommand.y   = request.joystickCommand.y;
        joystickCommand.z   = request.joystickCommand.z;
        joystickCommand.yaw = request.joystickCommand.yaw;

        ptr_wrapper_->velocityAndYawRateCtrl(joystickCommand, request.velocityControlTimeMs);
        response.result = true;

        break;
      }
    case FlightTaskControl::Request::TASK_LAND:
      {
        ROS_INFO_STREAM("call land service");
        if (ptr_wrapper_->monitoredLanding(FLIGHT_CONTROL_WAIT_TIMEOUT))
        {
          response.result = true;
        }
        break;
      }
    case FlightTaskControl::Request::START_MOTOR:
    {
        ROS_INFO_STREAM("call start motor service");
        if (ptr_wrapper_->turnOnOffMotors(true))
        {
          response.result = true;
        }
        break;
    }
    case FlightTaskControl::Request::STOP_MOTOR:
    {
        ROS_INFO_STREAM("call stop motor service");
        if (ptr_wrapper_->turnOnOffMotors(false))
        {
          response.result = true;
        }
        break;
    }
    case FlightTaskControl::Request::TASK_EXIT_GO_HOME:
      {
        ROS_INFO_STREAM("call cancel go home service");
        if (ptr_wrapper_->cancelGoHome(FLIGHT_CONTROL_WAIT_TIMEOUT))
        {
          response.result = true;
        }
        break;
      }
    case FlightTaskControl::Request::TASK_EXIT_LANDING:
      {
        ROS_INFO_STREAM("call cancel landing service");
        if (ptr_wrapper_->cancelLanding(FLIGHT_CONTROL_WAIT_TIMEOUT))
        {
          response.result = true;
        }
        break;
      }
    case FlightTaskControl::Request::TASK_FORCE_LANDING_AVOID_GROUND:
    {
        ROS_INFO_STREAM("call confirm landing service");
        if (ptr_wrapper_->startConfirmLanding(FLIGHT_CONTROL_WAIT_TIMEOUT))
        {
          response.result = true;
        }
        break;
    }
    case FlightTaskControl::Request::TASK_FORCE_LANDING:
    {
        ROS_INFO_STREAM("call force landing service");
        if (ptr_wrapper_->startForceLanding(FLIGHT_CONTROL_WAIT_TIMEOUT))
        {
          response.result = true;
        }
        break;
    }
    default:
      {
        ROS_INFO_STREAM("No recognized task");
        response.result = false;
        break;
      }
  }

  if (response.result)
  {
    return true;
  }
  else
  {
    return false;
  }
}

bool VehicleNode::setJoystickModeCallback(SetJoystickMode::Request& request, SetJoystickMode::Response& response)
{
  ROS_DEBUG("called setJoystickModeCallback");
  if(ptr_wrapper_ == nullptr)
  {
    ROS_ERROR_STREAM("Vehicle modules is nullptr");
    return false;
  }

  dji_osdk_ros::JoystickMode joystickMode;
  joystickMode.horizontalLogic = request.horizontal_mode;
  joystickMode.verticalLogic   = request.vertical_mode;
  joystickMode.yawLogic        = request.yaw_mode;
  joystickMode.horizontalCoordinate = request.horizontal_coordinate;
  joystickMode.stableMode      = request.stable_mode;

  ptr_wrapper_->setJoystickMode(joystickMode);

  response.result = true;
  return true;
}

bool VehicleNode::JoystickActionCallback(JoystickAction::Request& request, JoystickAction::Response& response)
{
  if(ptr_wrapper_ == nullptr)
  {
    ROS_ERROR_STREAM("Vehicle modules is nullptr");
    return false;
  }

  dji_osdk_ros::JoystickCommand joystickCommand;
  joystickCommand.x = request.joystickCommand.x;
  joystickCommand.y = request.joystickCommand.y;
  joystickCommand.z = request.joystickCommand.z;
  joystickCommand.yaw = request.joystickCommand.yaw;

  ptr_wrapper_->JoystickAction(joystickCommand);

  response.result = true;
  return true;
}

bool VehicleNode::gimbalCtrlCallback(GimbalAction::Request& request, GimbalAction::Response& response)
{
  if(ptr_wrapper_ == nullptr)
  {
    ROS_ERROR_STREAM("Vehicle modules is nullptr");
    return false;
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
    return false;
  }
  response.result = true;
  response.result &= ptr_wrapper_->setExposureMode(static_cast<PayloadIndex>(request.payload_index),
                                                   static_cast<ExposureMode>(request.exposure_mode));
  response.result &= ptr_wrapper_->setEV(static_cast<PayloadIndex>(request.payload_index),
                                         static_cast<ExposureCompensation>(request.exposure_compensation));
  return response.result;
}

bool VehicleNode::cameraSetShutterSpeedCallback(CameraShutterSpeed::Request& request, CameraShutterSpeed::Response& response)
{
  if(ptr_wrapper_ == nullptr)
  {
    ROS_ERROR_STREAM("Vehicle modules is nullptr");
    return false;
  }
  response.result = true;
  response.result &= ptr_wrapper_->setExposureMode(static_cast<PayloadIndex>(request.payload_index),
                                                   static_cast<ExposureMode>(request.exposure_mode));
  response.result &= ptr_wrapper_->setShutterSpeed(static_cast<PayloadIndex>(request.payload_index),
                                                   static_cast<ShutterSpeed>(request.shutter_speed));
  return response.result;
}

bool VehicleNode::cameraSetApertureCallback(CameraAperture::Request& request, CameraAperture::Response& response)
{
  if(ptr_wrapper_ == nullptr)
  {
    ROS_ERROR_STREAM("Vehicle modules is nullptr");
    return false;
  }
  response.result = true;
  response.result &= ptr_wrapper_->setExposureMode(static_cast<PayloadIndex>(request.payload_index),
                                                   static_cast<ExposureMode>(request.exposure_mode));
  response.result &= ptr_wrapper_->setAperture(static_cast<PayloadIndex>(request.payload_index),
                                               static_cast<Aperture>(request.aperture));
  return response.result;
}

bool VehicleNode::cameraSetISOCallback(CameraISO::Request& request, CameraISO::Response& response)
{
  if(ptr_wrapper_ == nullptr)
  {
    ROS_ERROR_STREAM("Vehicle modules is nullptr");
    return false;
  }
  response.result = true;
  response.result &= ptr_wrapper_->setExposureMode(static_cast<PayloadIndex>(request.payload_index),
                                                   static_cast<ExposureMode>(request.exposure_mode));
  response.result &= ptr_wrapper_->setISO(static_cast<PayloadIndex>(request.payload_index),
                                          static_cast<ISO>(request.iso_data));
  return response.result;
}

bool VehicleNode::cameraSetFocusPointCallback(CameraFocusPoint::Request& request, CameraFocusPoint::Response& response)
{
  if(ptr_wrapper_ == nullptr)
  {
    ROS_ERROR_STREAM("Vehicle modules is nullptr");
    return false;
  }
  response.result = ptr_wrapper_->setFocusPoint(static_cast<PayloadIndex>(request.payload_index), request.x, request.y);
  return response.result;
}

bool VehicleNode::cameraSetTapZoomPointCallback(CameraTapZoomPoint::Request& request, CameraTapZoomPoint::Response& response)
{
  if(ptr_wrapper_ == nullptr)
  {
    ROS_ERROR_STREAM("Vehicle modules is nullptr");
    return false;
  }
  response.result = ptr_wrapper_->setTapZoomPoint(static_cast<PayloadIndex>(request.payload_index),request.multiplier, request.x, request.y);
  return response.result;
}

bool VehicleNode::cameraSetZoomParaCallback(CameraSetZoomPara::Request& request, CameraSetZoomPara::Response& response)
{
  if(ptr_wrapper_ == nullptr)
  {
    ROS_ERROR_STREAM("Vehicle modules is nullptr");
    return false;
  }

  response.result = ptr_wrapper_->setZoom(static_cast<PayloadIndex>(request.payload_index), request.factor);
  return response.result;
}

bool VehicleNode::cameraZoomCtrlCallback(CameraZoomCtrl::Request& request, CameraZoomCtrl::Response& response)
{
  if(ptr_wrapper_ == nullptr)
  {
    ROS_ERROR_STREAM("Vehicle modules is nullptr");
    return false;
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
    return false;
  }
  response.result = ptr_wrapper_->startShootSinglePhoto(static_cast<PayloadIndex>(request.payload_index));
  return response.result;
}

bool VehicleNode::cameraStartShootAEBPhotoCallback(CameraStartShootAEBPhoto::Request& request, CameraStartShootAEBPhoto::Response& response)
{
  if(ptr_wrapper_ == nullptr)
  {
    ROS_ERROR_STREAM("Vehicle modules is nullptr");
    return false;
  }
  response.result = ptr_wrapper_->startShootAEBPhoto(static_cast<PayloadIndex>(request.payload_index), static_cast<PhotoAEBCount>(request.photo_aeb_count));
  return response.result;
}

bool VehicleNode::cameraStartShootBurstPhotoCallback(CameraStartShootBurstPhoto::Request& request, CameraStartShootBurstPhoto::Response& response)
{
  if(ptr_wrapper_ == nullptr)
  {
    ROS_ERROR_STREAM("Vehicle modules is nullptr");
    return false;
  }
  response.result = ptr_wrapper_->startShootBurstPhoto(static_cast<PayloadIndex>(request.payload_index),static_cast<PhotoBurstCount>(request.photo_burst_count));
  return response.result;
}

bool VehicleNode::cameraStartShootIntervalPhotoCallback(CameraStartShootIntervalPhoto::Request& request, CameraStartShootIntervalPhoto::Response& response)
{
  if(ptr_wrapper_ == nullptr)
  {
    ROS_ERROR_STREAM("Vehicle modules is nullptr");
    return false;
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
    return false;
  }
  response.result = ptr_wrapper_->shootPhotoStop(static_cast<PayloadIndex>(request.payload_index));
  return response.result;
}

bool VehicleNode::cameraRecordVideoActionCallback(CameraRecordVideoAction::Request& request, CameraRecordVideoAction::Response& response)
{
  if(ptr_wrapper_ == nullptr)
  {
    ROS_ERROR_STREAM("Vehicle modules is nullptr");
    return false;
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

bool VehicleNode::getWholeBatteryInfoCallback(GetWholeBatteryInfo::Request& request,GetWholeBatteryInfo::Response& response)
{
  ROS_INFO_STREAM("get Whole Battery Info callback");
  if (ptr_wrapper_ == nullptr)
  {
    ROS_ERROR_STREAM("Vehicle modules is nullptr");
    return false;
  }

  DJI::OSDK::BatteryWholeInfo batteryWholeInfo;
  if (ptr_wrapper_->getBatteryWholeInfo(batteryWholeInfo))
  {
    response.battery_whole_info.remainFlyTime  = batteryWholeInfo.remainFlyTime;
    response.battery_whole_info.goHomeNeedTime = batteryWholeInfo.goHomeNeedTime ;
    response.battery_whole_info.landNeedTime   = batteryWholeInfo.landNeedTime;
    response.battery_whole_info.goHomeNeedCapacity = batteryWholeInfo.goHomeNeedCapacity;
    response.battery_whole_info.landNeedCapacity = batteryWholeInfo.landNeedCapacity ;
    response.battery_whole_info.safeFlyRadius = batteryWholeInfo.safeFlyRadius;
    response.battery_whole_info.capacityConsumeSpeed = batteryWholeInfo.capacityConsumeSpeed;
    response.battery_whole_info.goHomeCountDownState = batteryWholeInfo.goHomeCountDownState;
    response.battery_whole_info.gohomeCountDownvalue = batteryWholeInfo.gohomeCountDownvalue;
    response.battery_whole_info.voltage = batteryWholeInfo.voltage;
    response.battery_whole_info.batteryCapacityPercentage = batteryWholeInfo.batteryCapacityPercentage;
    response.battery_whole_info.lowBatteryAlarmThreshold = batteryWholeInfo.lowBatteryAlarmThreshold;
    response.battery_whole_info.lowBatteryAlarmEnable = batteryWholeInfo.lowBatteryAlarmEnable;
    response.battery_whole_info.seriousLowBatteryAlarmThreshold = batteryWholeInfo.seriousLowBatteryAlarmThreshold;
    response.battery_whole_info.seriousLowBatteryAlarmEnable = batteryWholeInfo.seriousLowBatteryAlarmEnable;

    response.battery_whole_info.batteryState.voltageNotSafety        = batteryWholeInfo.batteryState.voltageNotSafety;
    response.battery_whole_info.batteryState.veryLowVoltageAlarm     = batteryWholeInfo.batteryState.veryLowVoltageAlarm;
    response.battery_whole_info.batteryState.LowVoltageAlarm         = batteryWholeInfo.batteryState.LowVoltageAlarm;
    response.battery_whole_info.batteryState.seriousLowCapacityAlarm = batteryWholeInfo.batteryState.seriousLowCapacityAlarm;
    response.battery_whole_info.batteryState.LowCapacityAlarm        = batteryWholeInfo.batteryState.LowCapacityAlarm;
  }
  else
  {
    DSTATUS("get Battery Whole Info failed!");
    return false;
  }
  return true;
}

bool VehicleNode::getSingleBatteryDynamicInfoCallback(GetSingleBatteryDynamicInfo::Request& request, 
                                                      GetSingleBatteryDynamicInfo::Response& response)
{
  ROS_INFO_STREAM("get Single Battery Dynamic Info callback");
  if(ptr_wrapper_ == nullptr)
  {
    ROS_ERROR_STREAM("Vehicle modules is nullptr");
    return false;
  }

  DJI::OSDK::SmartBatteryDynamicInfo SmartBatteryDynamicInfo;
  if (ptr_wrapper_->getSingleBatteryDynamicInfo(static_cast<DJI::OSDK::DJIBattery::RequestSmartBatteryIndex>(request.batteryIndex),
                                                SmartBatteryDynamicInfo))
  {
    response.smartBatteryDynamicInfo.batteryIndex           = SmartBatteryDynamicInfo.batteryIndex;
    response.smartBatteryDynamicInfo.currentVoltage         = SmartBatteryDynamicInfo.currentVoltage;
    response.smartBatteryDynamicInfo.currentElectric        = SmartBatteryDynamicInfo.currentElectric;
    response.smartBatteryDynamicInfo.fullCapacity           = SmartBatteryDynamicInfo.fullCapacity;
    response.smartBatteryDynamicInfo.remainedCapacity       = SmartBatteryDynamicInfo.remainedCapacity;
    response.smartBatteryDynamicInfo.batteryTemperature     = SmartBatteryDynamicInfo.batteryTemperature;
    response.smartBatteryDynamicInfo.cellCount              = SmartBatteryDynamicInfo.cellCount;
    response.smartBatteryDynamicInfo.batteryCapacityPercent = SmartBatteryDynamicInfo.batteryCapacityPercent;
    response.smartBatteryDynamicInfo.SOP                    = SmartBatteryDynamicInfo.SOP;

    response.smartBatteryDynamicInfo.batteryState.cellBreak                    = SmartBatteryDynamicInfo.batteryState.cellBreak;
    response.smartBatteryDynamicInfo.batteryState.selfCheckError               = SmartBatteryDynamicInfo.batteryState.selfCheckError;
    response.smartBatteryDynamicInfo.batteryState.batteryClosedReason          = SmartBatteryDynamicInfo.batteryState.batteryClosedReason;
    response.smartBatteryDynamicInfo.batteryState.batSOHState                  = SmartBatteryDynamicInfo.batteryState.batSOHState;
    response.smartBatteryDynamicInfo.batteryState.maxCycleLimit                = SmartBatteryDynamicInfo.batteryState.maxCycleLimit;
    response.smartBatteryDynamicInfo.batteryState.batteryCommunicationAbnormal = SmartBatteryDynamicInfo.batteryState.batteryCommunicationAbnormal;
    response.smartBatteryDynamicInfo.batteryState.hasCellBreak                 = SmartBatteryDynamicInfo.batteryState.hasCellBreak;
    response.smartBatteryDynamicInfo.batteryState.heatState                    = SmartBatteryDynamicInfo.batteryState.heatState;
  }
  else
  {
    DSTATUS("get Single Battery Dynamic Info failed!");
    return false;
  }

  return true;
}

bool VehicleNode::getHMSDataCallback(GetHMSData::Request& request, GetHMSData::Response& response)
{
  //ROS_INFO_STREAM("Get HMS Data callback");
  if(ptr_wrapper_ == nullptr)
  {
    ROS_ERROR_STREAM("Vehicle modules is nullptr");
    return false;
  }
  response.result = true;

  static uint8_t count = 0;
  while (count < 1)
  {
    response.result = ptr_wrapper_->enableSubscribeHMSInfo(request.enable);
    count++;
  }

  if (request.enable == true)
  {
    dji_osdk_ros::HMSPushPacket hmsPushPacket;
    ptr_wrapper_->getHMSListInfo(hmsPushPacket);
    ptr_wrapper_->getHMSDeviceIndex(response.deviceIndex);
    response.timeStamp = hmsPushPacket.timeStamp;

    if (hmsPushPacket.hmsPushData.errList.size())
    {
      response.errList.clear();
      response.errList.resize(hmsPushPacket.hmsPushData.errList.size());
    }
    
    for (int i = 0; i < hmsPushPacket.hmsPushData.errList.size(); i++)
    {
      response.errList[i].alarmID     = hmsPushPacket.hmsPushData.errList[i].alarmID;
      response.errList[i].reportLevel = hmsPushPacket.hmsPushData.errList[i].reportLevel;
      response.errList[i].sensorIndex = hmsPushPacket.hmsPushData.errList[i].sensorIndex;
      // DSTATUS("%ld, response.errList.size():%d,0x%08x,%d,%d",response.timeStamp,response.errList.size(),response.errList[i].alarmID,
      // response.errList[i].sensorIndex,
      // response.errList[i].reportLevel);
    }
  }

  return response.result;
}

bool VehicleNode::mfioCtrlCallback(MFIO::Request& request, MFIO::Response& response)
{
  ROS_INFO_STREAM("MFIO Control callback");
  if(ptr_wrapper_ == nullptr)
  {
    ROS_ERROR_STREAM("Vehicle modules is nullptr");
    return false;
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
    return false;
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

bool VehicleNode::getGoHomeAltitudeCallback(GetGoHomeAltitude::Request& request, GetGoHomeAltitude::Response& response)
{
  ROS_INFO_STREAM("Get go home altitude callback");
  if(ptr_wrapper_ == nullptr)
  {
    ROS_ERROR_STREAM("Vehicle modules is nullptr");
    return false;
  }

  uint16_t altitude = 0;
  if (!(ptr_wrapper_->getHomeAltitude(altitude)))
  {
    response.result = false;
    return false;
  }
  
  response.altitude = altitude;
  response.result = true;
  return true;
}

bool VehicleNode::setCurrentAircraftLocAsHomeCallback(SetCurrentAircraftLocAsHomePoint::Request& request, SetCurrentAircraftLocAsHomePoint::Response& response)
{
  ROS_INFO_STREAM("Set current aircraft location as new home point callback");
  if(ptr_wrapper_ == nullptr)
  {
    ROS_ERROR_STREAM("Vehicle modules is nullptr");
    return false;
  }

  if(ptr_wrapper_->setCurrentAircraftLocAsHomePoint() == true)
  {
    response.result = true;
  }
  else
  {
    response.result = false;
  }

  return true;
}

bool VehicleNode::setHomePointCallback(SetHomePoint::Request& request, SetHomePoint::Response& response)
{
  ROS_INFO_STREAM("Set home point callback");
  if(ptr_wrapper_ == nullptr)
  {
    ROS_ERROR_STREAM("Vehicle modules is nullptr");
    return false;
  }

  if (ptr_wrapper_->setHomePoint(request.latitude, request.longitude))
  {
    response.result = true;
    return true;
  }

  response.result = false;
  return false;
}

bool VehicleNode::setLocalPosRefCallback(dji_osdk_ros::SetLocalPosRef::Request &request,
                                         dji_osdk_ros::SetLocalPosRef::Response &response)
{
  ROS_INFO("Currrent GPS health is %d",current_gps_health_ );
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

bool VehicleNode::setHorizonAvoidCallback(SetAvoidEnable::Request& request, SetAvoidEnable::Response& response)
{
  ROS_INFO_STREAM("Set horizon avoid function callback");
  if(ptr_wrapper_ == nullptr)
  {
    ROS_ERROR_STREAM("Vehicle modules is nullptr");
    return false;
  }

  if (!(ptr_wrapper_->setCollisionAvoidance(request.enable)))
  {
    response.result = false;
    return false;
  }

  response.result = true;
  return true;
}

bool VehicleNode::setUpwardsAvoidCallback(SetAvoidEnable::Request& request, SetAvoidEnable::Response& response)
{
  ROS_INFO_STREAM("Set upwards avoid function callback");
  if(ptr_wrapper_ == nullptr)
  {
    ROS_ERROR_STREAM("Vehicle modules is nullptr");
    return false;
  }

  if(!(ptr_wrapper_->setUpwardsAvoidance(request.enable)))
  {
    response.result = false;
    return false;
  }


  response.result = true;
  return true;
}

bool VehicleNode::getAvoidEnableStatusCallback(GetAvoidEnable::Request& request, GetAvoidEnable::Response& response)
{
  ROS_INFO_STREAM("Set upwards avoid function callback");
  if(ptr_wrapper_ == nullptr)
  {
    ROS_ERROR_STREAM("Vehicle modules is nullptr");
    return false;
  }

  uint8_t get_horizon_avoid_enable_status = 0xF;
  uint8_t get_upwards_avoid_enable_status = 0xF;

  if (!(ptr_wrapper_->getCollisionAvoidance(get_horizon_avoid_enable_status)))
  {
    response.result = false;
    return false;
  }
  response.horizon_avoid_enable_status = get_horizon_avoid_enable_status;

  if(!(ptr_wrapper_->getUpwardsAvoidance(get_upwards_avoid_enable_status)))
  {
    response.result = false;
    return false;
  }

  response.upwards_avoid_enable_status = get_horizon_avoid_enable_status;

  response.result = true;
  return true;
}

bool VehicleNode::obtainReleaseControlAuthorityCallback(ObtainControlAuthority::Request& request, ObtainControlAuthority::Response& response)
{
  if(request.enable_obtain)
  {
    ROS_INFO_STREAM("Obtain Control Authority Callback");
  }
  else
  {
    ROS_INFO_STREAM("release Control Authority Callback");
  }

  if(ptr_wrapper_ == nullptr)
  {
    ROS_ERROR_STREAM("Vehicle modules is nullptr");
    return false;
  }

  response.result = ptr_wrapper_->obtainReleaseCtrlAuthority(request.enable_obtain, FLIGHT_CONTROL_WAIT_TIMEOUT);

  return response.result;
}

bool VehicleNode::killSwitchCallback(KillSwitch::Request& request, KillSwitch::Response& response)
{
  if(ptr_wrapper_ == nullptr)
  {
    ROS_ERROR_STREAM("Vehicle modules is nullptr");
    return false;
  }
  char msg[10] = "StopFLy";
  response.result = ptr_wrapper_->killSwitch(request.enable, msg);

  return response.result;
}

bool VehicleNode::emergencyBrakeCallback(EmergencyBrake::Request& request, EmergencyBrake::Response& response)
{
  if(ptr_wrapper_ == nullptr)
  {
    ROS_ERROR_STREAM("Vehicle modules is nullptr");
    return false;
  }

  response.result = ptr_wrapper_->emergencyBrake();

  return response.result;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "vehicle_node");
  VehicleNode vh_node;

  ros::spin();
  return 0;
}
