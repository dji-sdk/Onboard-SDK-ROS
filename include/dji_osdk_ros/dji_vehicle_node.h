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

#ifndef __DJI_VEHICLE_NODE_HH__
#define __DJI_VEHICLE_NODE_HH__

// Header include
#include <ros/ros.h>
#include <dji_vehicle.hpp>

#include <dji_osdk_ros/vehicle_wrapper.h>
#include <dji_osdk_ros/common_type.h>

#include <memory>
#include <string>

//! ROS standard msgs
#include <tf/tf.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/QuaternionStamped.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <geometry_msgs/PointStamped.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/Joy.h>
#include <sensor_msgs/TimeReference.h>
#include <sensor_msgs/BatteryState.h>
#include <sensor_msgs/Image.h>
#include <std_msgs/UInt8.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Float32.h>
#include <std_msgs/String.h>
#include <nmea_msgs/Sentence.h>

/*! services */
//flight control services
#include <dji_osdk_ros/GetDroneType.h>
#include <dji_osdk_ros/FlightTaskControl.h>
#include <dji_osdk_ros/SetJoystickMode.h>
#include <dji_osdk_ros/JoystickAction.h>
#include <dji_osdk_ros/SetGoHomeAltitude.h>
#include <dji_osdk_ros/GetGoHomeAltitude.h>
#include <dji_osdk_ros/SetHomePoint.h>
#include <dji_osdk_ros/SetCurrentAircraftLocAsHomePoint.h>
#include <dji_osdk_ros/SetLocalPosRef.h>
#include <dji_osdk_ros/SetAvoidEnable.h>
#include <dji_osdk_ros/GetAvoidEnable.h>
#include <dji_osdk_ros/ObtainControlAuthority.h>
#include <dji_osdk_ros/KillSwitch.h>
#include <dji_osdk_ros/EmergencyBrake.h>
//Gimbal control services
#include <dji_osdk_ros/GimbalAction.h>

//Camera control services
#include <dji_osdk_ros/CameraEV.h>
#include <dji_osdk_ros/CameraShutterSpeed.h>
#include <dji_osdk_ros/CameraAperture.h>
#include <dji_osdk_ros/CameraISO.h>
#include <dji_osdk_ros/CameraFocusPoint.h>
#include <dji_osdk_ros/CameraTapZoomPoint.h>
#include <dji_osdk_ros/CameraSetZoomPara.h>
#include <dji_osdk_ros/CameraZoomCtrl.h>
#include <dji_osdk_ros/CameraStartShootBurstPhoto.h>
#include <dji_osdk_ros/CameraStartShootAEBPhoto.h>
#include <dji_osdk_ros/CameraStartShootSinglePhoto.h>
#include <dji_osdk_ros/CameraStartShootIntervalPhoto.h>
#include <dji_osdk_ros/CameraStopShootPhoto.h>
#include <dji_osdk_ros/CameraRecordVideoAction.h>

//HMS services
#include <dji_osdk_ros/GetHMSData.h>
//mfio services
#include <dji_osdk_ros/MFIO.h>
//MOP services
#include <dji_osdk_ros/SendMobileData.h>
#include <dji_osdk_ros/SendPayloadData.h>
//mission services
#include <dji_osdk_ros/MissionStatus.h>
#include <dji_osdk_ros/MissionWpUpload.h>
#include <dji_osdk_ros/MissionWpAction.h>
#include <dji_osdk_ros/MissionWpGetSpeed.h>
#include <dji_osdk_ros/MissionWpSetSpeed.h>
#include <dji_osdk_ros/MissionWpGetInfo.h>
#include <dji_osdk_ros/MissionHpUpload.h>
#include <dji_osdk_ros/MissionHpAction.h>
#include <dji_osdk_ros/MissionHpGetInfo.h>
#include <dji_osdk_ros/MissionHpUpdateYawRate.h>
#include <dji_osdk_ros/MissionHpResetYaw.h>
#include <dji_osdk_ros/MissionHpUpdateRadius.h>
//battery services
#include <dji_osdk_ros/GetWholeBatteryInfo.h>
#include <dji_osdk_ros/GetSingleBatteryDynamicInfo.h>

//waypointV2.0 services
#include <dji_osdk_ros/InitWaypointV2Setting.h>
#include <dji_osdk_ros/UploadWaypointV2Mission.h>
#include <dji_osdk_ros/UploadWaypointV2Action.h>
#include <dji_osdk_ros/DownloadWaypointV2Mission.h>
#include <dji_osdk_ros/StartWaypointV2Mission.h>
#include <dji_osdk_ros/StopWaypointV2Mission.h>
#include <dji_osdk_ros/PauseWaypointV2Mission.h>
#include <dji_osdk_ros/ResumeWaypointV2Mission.h>
#include <dji_osdk_ros/GenerateWaypointV2Action.h>
#include <dji_osdk_ros/SetGlobalCruisespeed.h>
#include <dji_osdk_ros/GetGlobalCruisespeed.h>
#include <dji_osdk_ros/SubscribeWaypointV2Event.h>
#include <dji_osdk_ros/SubscribeWaypointV2State.h>

#ifdef ADVANCED_SENSING
#include <dji_osdk_ros/SetupCameraH264.h>
#include <dji_osdk_ros/SetupCameraStream.h>
#include <dji_osdk_ros/Stereo240pSubscription.h>
#include <dji_osdk_ros/StereoDepthSubscription.h>
#include <dji_osdk_ros/StereoVGASubscription.h>
#include <dji_osdk_ros/GetM300StereoParams.h>
#endif

/*! msgs */
#include <dji_osdk_ros/Gimbal.h>
#include <dji_osdk_ros/MobileData.h>
#include <dji_osdk_ros/PayloadData.h>
#include <dji_osdk_ros/FlightAnomaly.h>
#include <dji_osdk_ros/VOPosition.h>
#include <dji_osdk_ros/FCTimeInUTC.h>
#include <dji_osdk_ros/GPSUTC.h>

//waypointV2.0
#include <dji_osdk_ros/WaypointV2.h>
#include <dji_osdk_ros/WaypointV2Action.h>
#include <dji_osdk_ros/WaypointV2AircraftControlActuator.h>
#include <dji_osdk_ros/WaypointV2AircraftControlActuatorFlying.h>
#include <dji_osdk_ros/WaypointV2AircraftControlActuatorRotateHeading.h>
#include <dji_osdk_ros/WaypointV2AssociateTrigger.h>
#include <dji_osdk_ros/WaypointV2CameraActuator.h>
#include <dji_osdk_ros/WaypointV2CameraActuatorFocusParam.h>
#include <dji_osdk_ros/WaypointV2CameraActuatorFocalLengthParam.h>
#include <dji_osdk_ros/WaypointV2Config.h>
#include <dji_osdk_ros/WaypointV2GimbalActuator.h>
#include <dji_osdk_ros/WaypointV2GimbalActuatorRotationParam.h>
#include <dji_osdk_ros/WaypointV2InitSetting.h>
#include <dji_osdk_ros/WaypointV2IntervalTrigger.h>
#include <dji_osdk_ros/WaypointV2ReachpointTrigger.h>
#include <dji_osdk_ros/WaypointV2SampleReachPointTrigger.h>
#include <dji_osdk_ros/WaypointV2TrajectoryTrigger.h>
#include <dji_osdk_ros/WaypointV2MissionEventPush.h>
#include <dji_osdk_ros/WaypointV2MissionStatePush.h>

#define C_EARTH (double)6378137.0
#define C_PI (double)3.141592653589793
#define DEG2RAD(DEG) ((DEG) * ((C_PI) / (180.0)))
#define RAD2DEG(RAD) ((RAD) * (180.0) / (C_PI))
const int WAIT_TIMEOUT = 10;
const int FLIGHT_CONTROL_WAIT_TIMEOUT = 1;

// Declaration
namespace dji_osdk_ros
{
  using namespace DJI::OSDK;
  using namespace Telemetry;

  class VehicleNode
  {
    public:
      VehicleNode();
      VehicleNode(int test);

      ~VehicleNode();

      bool initGimbalModule();
      bool initCameraModule();
      void initService();
      bool initTopic();
      bool initDataSubscribeFromFC();
      bool cleanUpSubscribeFromFC();
    protected:
      /*! services */
      /*! for general */
      ros::ServiceServer get_drone_type_server_;
      /*! for flight control */
      ros::ServiceServer obtain_releae_control_authority_server_;
      ros::ServiceServer task_control_server_;
      ros::ServiceServer set_joystick_mode_server_;
      ros::ServiceServer joystick_action_server_;
      ros::ServiceServer set_home_altitude_server_;
      ros::ServiceServer get_home_altitude_server_;
      ros::ServiceServer set_home_point_server_;
      ros::ServiceServer set_current_aircraft_point_as_home_server_;
      ros::ServiceServer set_local_pos_reference_server_;
      ros::ServiceServer set_horizon_avoid_enable_server_;
      ros::ServiceServer get_avoid_enable_status_server_;
      ros::ServiceServer set_upwards_avoid_enable_server_;
      ros::ServiceServer kill_switch_server_;
      ros::ServiceServer emergency_brake_action_server_;

      /*! for gimbal */
      ros::ServiceServer gimbal_control_server_;
      /*! for camera */
      ros::ServiceServer camera_control_set_EV_server_;
      ros::ServiceServer camera_control_set_shutter_speed_server_;
      ros::ServiceServer camera_control_set_aperture_server_;
      ros::ServiceServer camera_control_set_ISO_server_;
      ros::ServiceServer camera_control_set_focus_point_server_;
      ros::ServiceServer camera_control_set_tap_zoom_point_server_;
      ros::ServiceServer camera_control_set_zoom_para_server_;
      ros::ServiceServer camera_control_zoom_ctrl_server_;
      ros::ServiceServer camera_control_start_shoot_single_photo_server_;
      ros::ServiceServer camera_control_start_shoot_burst_photo_server_;
      ros::ServiceServer camera_control_start_shoot_AEB_photo_server_;
      ros::ServiceServer camera_control_start_shoot_interval_photo_server_;
      ros::ServiceServer camera_control_stop_shoot_photo_server_;
      ros::ServiceServer camera_control_record_video_action_server_;

      /*! for battery */
      ros::ServiceServer get_single_battery_dynamic_info_server_;
      ros::ServiceServer get_whole_battery_info_server_;
      /*! for hms */
      ros::ServiceServer get_hms_data_server_;
      /*! for mfio */
      ros::ServiceServer mfio_control_server_;
      /*! for mobile device */
      ros::ServiceServer send_data_to_mobile_device_server_;
      /*! for payload device */
      ros::ServiceServer send_data_to_payload_device_server_;
      /*! for advanced sensing */
#ifdef ADVANCED_SENSING
      ros::ServiceServer setup_camera_stream_server_;
      ros::ServiceServer setup_camera_h264_server_;
      ros::ServiceServer subscribe_stereo_240p_server_;
      ros::ServiceServer subscribe_stereo_depth_server_;
      ros::ServiceServer subscribe_stereo_vga_server_;
      ros::ServiceServer get_m300_stereo_params_server_;
#endif
      /*! for mission */
      ros::ServiceServer waypoint_upload_server_;
      ros::ServiceServer waypoint_action_server_;
      ros::ServiceServer waypoint_getInfo_server_;
      ros::ServiceServer waypoint_getSpeed_server_;
      ros::ServiceServer waypoint_setSpeed_server_;
      ros::ServiceServer hotpoint_upload_server_;
      ros::ServiceServer hotpoint_action_server_;
      ros::ServiceServer hotpoint_getInfo_server_;
      ros::ServiceServer hotpoint_setSpeed_server_;
      ros::ServiceServer hotpoint_resetYaw_server_;
      ros::ServiceServer hotpoint_setRadius_server_;
      ros::ServiceServer mission_status_server_;

      /*! for waypoint2.0 */
      ros::ServiceServer waypointV2_init_setting_server_;
      ros::ServiceServer waypointV2_upload_mission_server_;
      ros::ServiceServer waypointV2_download_mission_server_;
      ros::ServiceServer waypointV2_upload_action_server_;
      ros::ServiceServer waypointV2_start_mission_server_;
      ros::ServiceServer waypointV2_stop_mission_server_;
      ros::ServiceServer waypointV2_pause_mission_server_;
      ros::ServiceServer waypointV2_resume_mission_server_;
      ros::ServiceServer waypointV2_generate_actions_server_;
      ros::ServiceServer waypointv2_set_global_cruisespeed_server_;
      ros::ServiceServer waypointv2_get_global_cruisespeed_server_;
      ros::ServiceServer waypointV2_generate_polygon_server_;
      ros::ServiceServer waypointv2_subscribe_mission_event_server_;
      ros::ServiceServer waypointv2_subscribe_mission_state_server_;

      /*! publishers */
      //! telemetry data publisher
      ros::Publisher attitude_publisher_;
      ros::Publisher angularRate_publisher_;
      ros::Publisher acceleration_publisher_;
      ros::Publisher battery_state_publisher_;
      ros::Publisher trigger_publisher_;
      ros::Publisher imu_publisher_;
      ros::Publisher flight_status_publisher_;
      ros::Publisher gps_health_publisher_;
      ros::Publisher gps_position_publisher_;
      ros::Publisher vo_position_publisher_;
      ros::Publisher height_publisher_;
      ros::Publisher velocity_publisher_;
      ros::Publisher from_mobile_data_publisher_;
      ros::Publisher from_payload_data_publisher_;
      ros::Publisher gimbal_angle_publisher_;
      ros::Publisher displaymode_publisher_;
      ros::Publisher rc_publisher_;
      ros::Publisher rc_connection_status_publisher_;
      ros::Publisher rtk_position_publisher_;
      ros::Publisher rtk_velocity_publisher_;
      ros::Publisher rtk_yaw_publisher_;
      ros::Publisher rtk_position_info_publisher_;
      ros::Publisher rtk_yaw_info_publisher_;
      ros::Publisher rtk_connection_status_publisher_;
      ros::Publisher flight_anomaly_publisher_;
      //! Local Position Publisher (Publishes local position in ENU frame)
      ros::Publisher local_position_publisher_;
      ros::Publisher local_frame_ref_publisher_;
      ros::Publisher time_sync_nmea_publisher_;
      ros::Publisher time_sync_gps_utc_publisher_;
      ros::Publisher time_sync_fc_utc_publisher_;
      ros::Publisher time_sync_pps_source_publisher_;

      //advanced sensing
      #ifdef ADVANCED_SENSING
      ros::Publisher main_camera_stream_publisher_;
      ros::Publisher fpv_camera_stream_publisher_;
      ros::Publisher camera_h264_publisher_;
      ros::Publisher stereo_240p_front_left_publisher_;
      ros::Publisher stereo_240p_front_right_publisher_;
      ros::Publisher stereo_240p_down_front_publisher_;
      ros::Publisher stereo_240p_down_back_publisher_;
      ros::Publisher stereo_240p_front_depth_publisher_;
      ros::Publisher stereo_vga_front_left_publisher_;
      ros::Publisher stereo_vga_front_right_publisher_;
      #endif

      //waypointV2
      ros::Publisher waypointV2_mission_state_publisher_;
      ros::Publisher waypointV2_mission_event_publisher_;

    protected:
      /*! for general */
      bool getDroneTypeCallback(dji_osdk_ros::GetDroneType::Request &request,
                                dji_osdk_ros::GetDroneType::Response &response);
      /*! for flight control */
      bool taskCtrlCallback(FlightTaskControl::Request& request, FlightTaskControl::Response& response);
      bool setJoystickModeCallback(SetJoystickMode::Request& request, SetJoystickMode::Response& response);
      bool JoystickActionCallback(JoystickAction::Request& request, JoystickAction::Response& response);
      bool setGoHomeAltitudeCallback(SetGoHomeAltitude::Request& request, SetGoHomeAltitude::Response& response);
      bool getGoHomeAltitudeCallback(GetGoHomeAltitude::Request& request, GetGoHomeAltitude::Response& response);
      bool setCurrentAircraftLocAsHomeCallback(SetCurrentAircraftLocAsHomePoint::Request& request, 
                                               SetCurrentAircraftLocAsHomePoint::Response& response);
      bool setHomePointCallback(SetHomePoint::Request& request, SetHomePoint::Response& response);
      bool setLocalPosRefCallback(dji_osdk_ros::SetLocalPosRef::Request &request,
                                  dji_osdk_ros::SetLocalPosRef::Response &response);
      bool setHorizonAvoidCallback(SetAvoidEnable::Request& request, SetAvoidEnable::Response& response);
      bool setUpwardsAvoidCallback(SetAvoidEnable::Request& request, SetAvoidEnable::Response& response);
      bool getAvoidEnableStatusCallback(GetAvoidEnable::Request& request, GetAvoidEnable::Response& response);
      bool obtainReleaseControlAuthorityCallback(ObtainControlAuthority::Request& request, 
                                                 ObtainControlAuthority::Response& reponse);
      bool killSwitchCallback(KillSwitch::Request& request, KillSwitch::Response& response);
      bool emergencyBrakeCallback(EmergencyBrake::Request& request, EmergencyBrake::Response& response);
      /*! for gimbal control */
      bool gimbalCtrlCallback(GimbalAction::Request& request, GimbalAction::Response& response);
      /*! for camera conrol */
      bool cameraSetEVCallback(CameraEV::Request& request, CameraEV::Response& response);
      bool cameraSetShutterSpeedCallback(CameraShutterSpeed::Request& request, CameraShutterSpeed::Response& response);
      bool cameraSetApertureCallback(CameraAperture::Request& request, CameraAperture::Response& response);
      bool cameraSetISOCallback(CameraISO::Request& request, CameraISO::Response& response);
      bool cameraSetFocusPointCallback(CameraFocusPoint::Request& request, CameraFocusPoint::Response& response);
      bool cameraSetTapZoomPointCallback(CameraTapZoomPoint::Request& request, CameraTapZoomPoint::Response& response);
      bool cameraSetZoomParaCallback(CameraSetZoomPara::Request& request, CameraSetZoomPara::Response& response);
      bool cameraZoomCtrlCallback(CameraZoomCtrl::Request& request, CameraZoomCtrl::Response& response);
      bool cameraStartShootSinglePhotoCallback(CameraStartShootSinglePhoto::Request& request, 
                                               CameraStartShootSinglePhoto::Response& response);
      bool cameraStartShootAEBPhotoCallback(CameraStartShootAEBPhoto::Request& request,
                                            CameraStartShootAEBPhoto::Response& response);
      bool cameraStartShootBurstPhotoCallback(CameraStartShootBurstPhoto::Request& request,
                                              CameraStartShootBurstPhoto::Response& response);
      bool cameraStartShootIntervalPhotoCallback(CameraStartShootIntervalPhoto::Request& request, 
                                                 CameraStartShootIntervalPhoto::Response& response);
      bool cameraStopShootPhotoCallback(CameraStopShootPhoto::Request& request, 
                                        CameraStopShootPhoto::Response& response);
      bool cameraRecordVideoActionCallback(CameraRecordVideoAction::Request& request,
                                           CameraRecordVideoAction::Response& response);
      /*! for battery info */
      bool getWholeBatteryInfoCallback(GetWholeBatteryInfo::Request& request,GetWholeBatteryInfo::Response& reponse);
      bool getSingleBatteryDynamicInfoCallback(GetSingleBatteryDynamicInfo::Request& request,
                                               GetSingleBatteryDynamicInfo::Response& response);
      /*! for hms info */
      bool getHMSDataCallback(GetHMSData::Request& request, GetHMSData::Response& response);
      /*! for mfio conrol */
      bool mfioCtrlCallback(MFIO::Request& request, MFIO::Response& response);
      /*! for mobile device */
      bool sendToMobileCallback(dji_osdk_ros::SendMobileData::Request& request,
                                dji_osdk_ros::SendMobileData::Response& response);
      /*! for payload device */
      bool sendToPayloadCallback(dji_osdk_ros::SendPayloadData::Request& request,
                                 dji_osdk_ros::SendPayloadData::Response& response);
      /*! for advanced sensing conrol */
#ifdef ADVANCED_SENSING
      bool setupCameraStreamCallback(dji_osdk_ros::SetupCameraStream::Request& request,
                                     dji_osdk_ros::SetupCameraStream::Response& response);
      bool setupCameraH264Callback(dji_osdk_ros::SetupCameraH264::Request& request,
                                   dji_osdk_ros::SetupCameraH264::Response& response);
      //! stereo image service callback
      bool stereo240pSubscriptionCallback(dji_osdk_ros::Stereo240pSubscription::Request&  request,
                                          dji_osdk_ros::Stereo240pSubscription::Response& response);
      bool stereoDepthSubscriptionCallback(dji_osdk_ros::StereoDepthSubscription::Request&  request,
                                          dji_osdk_ros::StereoDepthSubscription::Response& response);
      bool stereoVGASubscriptionCallback(dji_osdk_ros::StereoVGASubscription::Request&  request,
                                          dji_osdk_ros::StereoVGASubscription::Response& response);
      bool getM300StereoParamsCallback(dji_osdk_ros::GetM300StereoParams::Request& request,
                                       dji_osdk_ros::GetM300StereoParams::Response& response);
      void publishAdvancedSeningData();
#endif
      /*! for mission service callback*/
      // mission manager
      bool missionStatusCallback(dji_osdk_ros::MissionStatus::Request&  request,
                                 dji_osdk_ros::MissionStatus::Response& response);
      // waypoint mission
      bool missionWpUploadCallback(dji_osdk_ros::MissionWpUpload::Request&  request,
                                    dji_osdk_ros::MissionWpUpload::Response& response);
      bool missionWpActionCallback(dji_osdk_ros::MissionWpAction::Request&  request,
                                   dji_osdk_ros::MissionWpAction::Response& response);
      bool missionWpGetInfoCallback(dji_osdk_ros::MissionWpGetInfo::Request&  request,
                                    dji_osdk_ros::MissionWpGetInfo::Response& response);
      bool missionWpGetSpeedCallback(dji_osdk_ros::MissionWpGetSpeed::Request&  request,
                                     dji_osdk_ros::MissionWpGetSpeed::Response& response);
      bool missionWpSetSpeedCallback(dji_osdk_ros::MissionWpSetSpeed::Request&  request,
                                     dji_osdk_ros::MissionWpSetSpeed::Response& response);
      // hotpoint mission
      bool missionHpUploadCallback(dji_osdk_ros::MissionHpUpload::Request&  request,
                                   dji_osdk_ros::MissionHpUpload::Response& response);
      bool missionHpActionCallback(dji_osdk_ros::MissionHpAction::Request&  request,
                                   dji_osdk_ros::MissionHpAction::Response& response);
      bool missionHpGetInfoCallback(dji_osdk_ros::MissionHpGetInfo::Request&  request,
                                    dji_osdk_ros::MissionHpGetInfo::Response& response);
      bool missionHpUpdateYawRateCallback(dji_osdk_ros::MissionHpUpdateYawRate::Request&  request,
                                          dji_osdk_ros::MissionHpUpdateYawRate::Response& response);
      bool missionHpResetYawCallback(dji_osdk_ros::MissionHpResetYaw::Request&  request,
                                     dji_osdk_ros::MissionHpResetYaw::Response& response);
      bool missionHpUpdateRadiusCallback(dji_osdk_ros::MissionHpUpdateRadius::Request&  request,
                                         dji_osdk_ros::MissionHpUpdateRadius::Response& response);
      /*! for waypiontV2.0 service callback*/
      bool waypointV2InitSettingCallback(dji_osdk_ros::InitWaypointV2Setting::Request&  request,
                                         dji_osdk_ros::InitWaypointV2Setting::Response& response);
      bool waypointV2UploadMissionCallback(dji_osdk_ros::UploadWaypointV2Mission::Request&  request,
                                           dji_osdk_ros::UploadWaypointV2Mission::Response& response);
      bool waypointV2DownloadMissionCallback(dji_osdk_ros::DownloadWaypointV2Mission::Request&  request,
                                             dji_osdk_ros::DownloadWaypointV2Mission::Response& response);
      bool waypointV2UploadActionCallback(dji_osdk_ros::UploadWaypointV2Action::Request&  request,
                                          dji_osdk_ros::UploadWaypointV2Action::Response& response);
      bool waypointV2StartMissionCallback(dji_osdk_ros::StartWaypointV2Mission::Request&  request,
                                          dji_osdk_ros::StartWaypointV2Mission::Response& response);
      bool waypointV2StopMissionCallback(dji_osdk_ros::StopWaypointV2Mission::Request&  request,
                                         dji_osdk_ros::StopWaypointV2Mission::Response& response);
      bool waypointV2PauseMissionCallback(dji_osdk_ros::PauseWaypointV2Mission::Request&  request,
                                          dji_osdk_ros::PauseWaypointV2Mission::Response& response);
      bool waypointV2ResumeMissionCallback(dji_osdk_ros::ResumeWaypointV2Mission::Request&  request,
                                           dji_osdk_ros::ResumeWaypointV2Mission::Response& response);
      bool waypointV2GenerateActionsCallback(dji_osdk_ros::GenerateWaypointV2Action::Request&  request,
                                             dji_osdk_ros::GenerateWaypointV2Action::Response& response);
      bool waypointV2SetGlobalCruisespeedCallback(dji_osdk_ros::SetGlobalCruisespeed::Request& request,
                                                  dji_osdk_ros::SetGlobalCruisespeed::Response& respons);
      bool waypointV2GetGlobalCruisespeedCallback(dji_osdk_ros::GetGlobalCruisespeed::Request& request,
                                                  dji_osdk_ros::GetGlobalCruisespeed::Response& response);
      bool waypointV2SubscribeMissionEventCallback(dji_osdk_ros::SubscribeWaypointV2Event::Request& request,
                                                   dji_osdk_ros::SubscribeWaypointV2Event::Response& response);
      bool waypointV2SubscribeMissionStateCallback(dji_osdk_ros::SubscribeWaypointV2State::Request& request,
                                                   dji_osdk_ros::SubscribeWaypointV2State::Response& response);

      bool initSubscribe();

    private:
      ros::NodeHandle nh_;
      VehicleWrapper* ptr_wrapper_;
      TelemetryType telemetry_from_fc_;

      int           app_id_;
      int           app_version_;
      int           baud_rate_;
      double        gravity_const_;
      std::string   enc_key_;
      std::string   device_acm_;
      std::string   device_;
      std::string   sample_case_;
      std::string   drone_version_;
      std::string   app_bundle_id_; // reserved
      bool          user_select_broadcast_;
      bool          align_time_with_FC_;

      AlignStatus curr_align_state_;
      ros::Time   base_time_;
      double      local_pos_ref_latitude_, local_pos_ref_longitude_, local_pos_ref_altitude_;
      double      current_gps_latitude_, current_gps_longitude_, current_gps_altitude_;
      bool        local_pos_ref_set_;
      int         current_gps_health_;
      const       tf::Matrix3x3 R_FLU2FRD_;
      const       tf::Matrix3x3 R_ENU2NED_;
      bool        rtk_support_;

      bool stereo_subscription_success;
      bool stereo_vga_subscription_success;

      std::vector<DJIWaypointV2Action> actions;

    //! data broadcast callback
    void dataBroadcastCallback();
    void fromMobileDataCallback(RecvContainer recvFrame);

    void fromPayloadDataCallback(RecvContainer recvFrame);

    static void NMEACallback(Vehicle* vehiclePtr,
                             RecvContainer recvFrame,
                             UserData userData);

    static void GPSUTCTimeCallback(Vehicle *vehiclePtr,
                                   RecvContainer recvFrame,
                                   UserData userData);


    static void FCTimeInUTCCallback(Vehicle* vehiclePtr,
                                    RecvContainer recvFrame,
                                    UserData userData);

    static void PPSSourceCallback(Vehicle* vehiclePtr,
                                  RecvContainer recvFrame,
                                  UserData userData);
    static void SDKfromMobileDataCallback(Vehicle*            vehicle,
                                          RecvContainer       recvFrame,
                                          DJI::OSDK::UserData userData);

    static void SDKfromPayloadDataCallback(Vehicle *vehicle,
                                           RecvContainer recvFrame,
                                           DJI::OSDK::UserData userData);

    static void SDKBroadcastCallback(Vehicle*            vehicle,
                                     RecvContainer       recvFrame,
                                     DJI::OSDK::UserData userData);

    static void publish5HzData(Vehicle*            vehicle,
                               RecvContainer       recvFrame,
                               DJI::OSDK::UserData userData);

    static void publish50HzData(Vehicle*            vehicle,
                                RecvContainer       recvFrame,
                                DJI::OSDK::UserData userData);

    static void publish100HzData(Vehicle*            vehicle,
                                 RecvContainer       recvFrame,
                                 DJI::OSDK::UserData userData);

    static void publish400HzData(Vehicle*            vehicle,
                                 RecvContainer       recvFrame,
                                 DJI::OSDK::UserData userData);

#ifdef ADVANCED_SENSING
    static void publishMainCameraImage(CameraRGBImage rgbImg, void* userData);
    static void publishFPVCameraImage(CameraRGBImage rgbImg, void* userData);
    static void publishCameraH264(uint8_t* buf, int bufLen, void* userData);
    static void publish240pStereoImage(Vehicle*            vehicle,
                                       RecvContainer       recvFrame,
                                       DJI::OSDK::UserData userData);

    static void publishVGAStereoImage(Vehicle*            vehicle,
                                      RecvContainer       recvFrame,
                                      DJI::OSDK::UserData userData);
#endif

    static E_OsdkStat updateMissionEvent(T_CmdHandle *cmdHandle, const T_CmdInfo *cmdInfo,
                                         const uint8_t *cmdData, void *userData);
    static E_OsdkStat updateMissionState(T_CmdHandle *cmdHandle, const T_CmdInfo *cmdInfo,
                                         const uint8_t *cmdData, void *userData);

public:
    void gpsConvertENU(double &ENU_x, double &ENU_y,
                       double gps_t_lon, double gps_t_lat,
                       double gps_r_lon, double gps_r_lat);
    void alignRosTimeWithFlightController(ros::Time now_time, uint32_t tick);
  };
}
#endif // __DJI_VEHICLE_NODE_HH__

