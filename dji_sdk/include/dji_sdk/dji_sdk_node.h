/** @file dji_sdk_node.h
 *  @version 3.7
 *  @date July, 2018
 *
 *  @brief
 *  A ROS wrapper to interact with DJI onboard SDK
 *
 *  @copyright 2018 DJI. All rights reserved.
 *
 */

#ifndef DJI_SDK_NODE_MAIN_H
#define DJI_SDK_NODE_MAIN_H

//! ROS
#include <ros/ros.h>
#include <tf/tf.h>

//! ROS standard msgs
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <geometry_msgs/TransformStamped.h>
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
#include <tf2_ros/transform_broadcaster.h>

//! msgs
#include <dji_sdk/Gimbal.h>
#include <dji_sdk/MobileData.h>
#include <dji_sdk/PayloadData.h>
#include <dji_sdk/FlightAnomaly.h>
#include <dji_sdk/VOPosition.h>
#include <dji_sdk/FCTimeInUTC.h>
#include <dji_sdk/GPSUTC.h>
#include <dji_sdk/RTKYaw.h>
#include <dji_sdk/RTKPosition.h>

//! mission service
// missionManager
#include <dji_sdk/MissionStatus.h>
// waypoint
#include <dji_sdk/MissionWpAction.h>
#include <dji_sdk/MissionWpGetInfo.h>
#include <dji_sdk/MissionWpGetSpeed.h>
#include <dji_sdk/MissionWpSetSpeed.h>
#include <dji_sdk/MissionWpUpload.h>
// hotpoint
#include <dji_sdk/MissionHpAction.h>
#include <dji_sdk/MissionHpGetInfo.h>
#include <dji_sdk/MissionHpResetYaw.h>
#include <dji_sdk/MissionHpUpdateRadius.h>
#include <dji_sdk/MissionHpUpdateYawRate.h>
#include <dji_sdk/MissionHpUpload.h>
// hardsync
#include <dji_sdk/SetHardSync.h>

//! service headers
#include <dji_sdk/Activation.h>
#include <dji_sdk/CameraAction.h>
#include <dji_sdk/DroneArmControl.h>
#include <dji_sdk/DroneTaskControl.h>
#include <dji_sdk/MFIOConfig.h>
#include <dji_sdk/MFIOSetValue.h>
#include <dji_sdk/SDKControlAuthority.h>
#include <dji_sdk/SetLocalPosRef.h>
#include <dji_sdk/SendMobileData.h>
#include <dji_sdk/SendPayloadData.h>
#include <dji_sdk/QueryDroneVersion.h>
#ifdef ADVANCED_SENSING
#include <dji_sdk/Stereo240pSubscription.h>
#include <dji_sdk/StereoDepthSubscription.h>
#include <dji_sdk/StereoVGASubscription.h>
#include <dji_sdk/SetupCameraStream.h>
#endif

//! SDK library
#include <djiosdk/dji_vehicle.hpp>

#define C_EARTH (double)6378137.0
#define C_PI (double)3.141592653589793
#define DEG2RAD(DEG) ((DEG) * ((C_PI) / (180.0)))
#define RAD2DEG(RAD) ((RAD) * (180.0) / (C_PI))

using namespace DJI::OSDK;

class DJISDKNode
{
public:
  DJISDKNode(ros::NodeHandle& nh, ros::NodeHandle& nh_private);
  ~DJISDKNode();

  enum TELEMETRY_TYPE
  {
    USE_BROADCAST = 0,
    USE_SUBSCRIBE = 1
  };

  enum
  {
    PACKAGE_ID_5HZ   = 0,
    PACKAGE_ID_50HZ  = 1,
    PACKAGE_ID_100HZ = 2,
    PACKAGE_ID_400HZ = 3
  };

private:
  bool initVehicle(ros::NodeHandle& nh_private);
  bool initServices(ros::NodeHandle& nh);
  bool initFlightControl(ros::NodeHandle& nh);
  bool initSubscriber(ros::NodeHandle& nh);
  bool initPublisher(ros::NodeHandle& nh);
  bool initActions(ros::NodeHandle& nh);
  bool initDataSubscribeFromFC(ros::NodeHandle& nh);
  void cleanUpSubscribeFromFC();
  bool validateSerialDevice(LinuxSerialDevice* serialDevice);
  bool isM100();

  /*!
   * @note this function exists here instead of inside the callback function
   *        due to the usages, i.e. we not only provide service call but also
   *        call it for the user when this node was instantiated
   *        we cannot call a service without serviceClient, which is in another
   * node
   */
  ACK::ErrorCode activate(int l_app_id, std::string l_enc_key);

  //! flight control subscriber callbacks
  void flightControlSetpointCallback(
    const sensor_msgs::Joy::ConstPtr& pMsg);

  void flightControlPxPyPzYawCallback(
    const sensor_msgs::Joy::ConstPtr& pMsg);

  void flightControlVxVyVzYawrateCallback(
    const sensor_msgs::Joy::ConstPtr& pMsg);

  void flightControlRollPitchPzYawrateCallback(
    const sensor_msgs::Joy::ConstPtr& pMsg);

  //! general subscriber callbacks
  void gimbalAngleCtrlCallback(const dji_sdk::Gimbal::ConstPtr& msg);
  void gimbalSpeedCtrlCallback(
    const geometry_msgs::Vector3Stamped::ConstPtr& msg);

  //! general service callbacks
  bool droneActivationCallback(dji_sdk::Activation::Request&  request,
                               dji_sdk::Activation::Response& response);
  bool sdkCtrlAuthorityCallback(
    dji_sdk::SDKControlAuthority::Request&  request,
    dji_sdk::SDKControlAuthority::Response& response);

  bool setLocalPosRefCallback(
      dji_sdk::SetLocalPosRef::Request&  request,
      dji_sdk::SetLocalPosRef::Response& response);
  //! control service callbacks
  bool droneArmCallback(dji_sdk::DroneArmControl::Request&  request,
                        dji_sdk::DroneArmControl::Response& response);
  bool droneTaskCallback(dji_sdk::DroneTaskControl::Request&  request,
                         dji_sdk::DroneTaskControl::Response& response);

  //! Mobile Data Service
  bool sendToMobileCallback(dji_sdk::SendMobileData::Request&  request,
                            dji_sdk::SendMobileData::Response& response);
  //! Payload Data Service
  bool sendToPayloadCallback(dji_sdk::SendPayloadData::Request& request,
                             dji_sdk::SendPayloadData::Response& response);
  //! Query Drone FW version
  bool queryVersionCallback(dji_sdk::QueryDroneVersion::Request& request,
                            dji_sdk::QueryDroneVersion::Response& response);

  bool cameraActionCallback(dji_sdk::CameraAction::Request&  request,
                            dji_sdk::CameraAction::Response& response);
  //! mfio service callbacks
  bool MFIOConfigCallback(dji_sdk::MFIOConfig::Request&  request,
                          dji_sdk::MFIOConfig::Response& response);
  bool MFIOSetValueCallback(dji_sdk::MFIOSetValue::Request&  request,
                            dji_sdk::MFIOSetValue::Response& response);
  //! mission service callbacks
  // mission manager
  bool missionStatusCallback(dji_sdk::MissionStatus::Request&  request,
                             dji_sdk::MissionStatus::Response& response);
  // waypoint mission
  bool missionWpUploadCallback(dji_sdk::MissionWpUpload::Request&  request,
                               dji_sdk::MissionWpUpload::Response& response);
  bool missionWpActionCallback(dji_sdk::MissionWpAction::Request&  request,
                               dji_sdk::MissionWpAction::Response& response);
  bool missionWpGetInfoCallback(dji_sdk::MissionWpGetInfo::Request&  request,
                                dji_sdk::MissionWpGetInfo::Response& response);
  bool missionWpGetSpeedCallback(
    dji_sdk::MissionWpGetSpeed::Request&  request,
    dji_sdk::MissionWpGetSpeed::Response& response);
  bool missionWpSetSpeedCallback(
    dji_sdk::MissionWpSetSpeed::Request&  request,
    dji_sdk::MissionWpSetSpeed::Response& response);
  // hotpoint mission
  bool missionHpUploadCallback(dji_sdk::MissionHpUpload::Request&  request,
                               dji_sdk::MissionHpUpload::Response& response);
  bool missionHpActionCallback(dji_sdk::MissionHpAction::Request&  request,
                               dji_sdk::MissionHpAction::Response& response);
  bool missionHpGetInfoCallback(dji_sdk::MissionHpGetInfo::Request&  request,
                                dji_sdk::MissionHpGetInfo::Response& response);
  bool missionHpUpdateYawRateCallback(
    dji_sdk::MissionHpUpdateYawRate::Request&  request,
    dji_sdk::MissionHpUpdateYawRate::Response& response);
  bool missionHpResetYawCallback(
    dji_sdk::MissionHpResetYaw::Request&  request,
    dji_sdk::MissionHpResetYaw::Response& response);
  bool missionHpUpdateRadiusCallback(
    dji_sdk::MissionHpUpdateRadius::Request&  request,
    dji_sdk::MissionHpUpdateRadius::Response& response);
  //! hard sync service callback
  bool setHardsyncCallback(dji_sdk::SetHardSync::Request&  request,
                           dji_sdk::SetHardSync::Response& response);

#ifdef ADVANCED_SENSING
  //! stereo image service callback
  bool stereo240pSubscriptionCallback(dji_sdk::Stereo240pSubscription::Request&  request,
                                      dji_sdk::Stereo240pSubscription::Response& response);
  bool stereoDepthSubscriptionCallback(dji_sdk::StereoDepthSubscription::Request&  request,
                                       dji_sdk::StereoDepthSubscription::Response& response);
  bool stereoVGASubscriptionCallback(dji_sdk::StereoVGASubscription::Request&  request,
                                     dji_sdk::StereoVGASubscription::Response& response);
  bool setupCameraStreamCallback(dji_sdk::SetupCameraStream::Request&  request,
                                 dji_sdk::SetupCameraStream::Response& response);
#endif

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
  static void publish240pStereoImage(Vehicle*            vehicle,
                                     RecvContainer       recvFrame,
                                     DJI::OSDK::UserData userData);

  static void publishVGAStereoImage(Vehicle*            vehicle,
                                    RecvContainer       recvFrame,
                                    DJI::OSDK::UserData userData);

  static void publishMainCameraImage(CameraRGBImage img, void* userData);

  static void publishFPVCameraImage(CameraRGBImage img, void* userData);
#endif

private:
  //! OSDK core
  Vehicle* vehicle;
  //! general service servers
  ros::ServiceServer drone_activation_server;
  ros::ServiceServer sdk_ctrlAuthority_server;
  ros::ServiceServer camera_action_server;
  //! flight control service servers
  ros::ServiceServer drone_arm_server;
  ros::ServiceServer drone_task_server;
  //! mfio service servers
  ros::ServiceServer mfio_config_server;
  ros::ServiceServer mfio_set_value_server;
  //! mission service servers
  // mission manager
  ros::ServiceServer mission_status_server;
  // waypoint mission
  ros::ServiceServer waypoint_upload_server;
  ros::ServiceServer waypoint_action_server;
  ros::ServiceServer waypoint_getInfo_server;
  ros::ServiceServer waypoint_getSpeed_server;
  ros::ServiceServer waypoint_setSpeed_server;
  // hotpoint mission
  ros::ServiceServer hotpoint_upload_server;
  ros::ServiceServer hotpoint_action_server;
  ros::ServiceServer hotpoint_getInfo_server;
  ros::ServiceServer hotpoint_setSpeed_server;
  ros::ServiceServer hotpoint_resetYaw_server;
  ros::ServiceServer hotpoint_setRadius_server;
  // send data to mobile device
  ros::ServiceServer send_to_mobile_server;
  // send data to payload device
  ros::ServiceServer send_to_payload_server;
  //! hardsync service
  ros::ServiceServer set_hardsync_server;
  //! Query FW version of FC
  ros::ServiceServer query_version_server;
  //! Set Local position reference
  ros::ServiceServer local_pos_ref_server;

#ifdef ADVANCED_SENSING
  //! stereo image service
  ros::ServiceServer subscribe_stereo_240p_server;
  ros::ServiceServer subscribe_stereo_depth_server;
  ros::ServiceServer subscribe_stereo_vga_server;
  ros::ServiceServer camera_stream_server;
#endif

  //! flight control subscribers
  ros::Subscriber flight_control_sub;

  ros::Subscriber flight_control_position_yaw_sub;
  ros::Subscriber flight_control_velocity_yawrate_sub;
  ros::Subscriber flight_control_rollpitch_yawrate_vertpos_sub;

  //! general subscribers
  ros::Subscriber gimbal_angle_cmd_subscriber;
  ros::Subscriber gimbal_speed_cmd_subscriber;
  //! telemetry data publisher
  ros::Publisher attitude_publisher;
  ros::Publisher angularRate_publisher;
  ros::Publisher acceleration_publisher;
  ros::Publisher battery_state_publisher;
  ros::Publisher trigger_publisher;
  ros::Publisher imu_publisher;
  ros::Publisher flight_status_publisher;
  ros::Publisher gps_health_publisher;
  ros::Publisher gps_position_publisher;
  ros::Publisher vo_position_publisher;
  ros::Publisher height_publisher;
  ros::Publisher velocity_publisher;
  ros::Publisher from_mobile_data_publisher;
  ros::Publisher from_payload_data_publisher;
  ros::Publisher gimbal_angle_publisher;
  ros::Publisher displaymode_publisher;
  ros::Publisher rc_publisher;
  ros::Publisher rc_connection_status_publisher;
  ros::Publisher rtk_position_publisher;
  ros::Publisher rtk_velocity_publisher;
  ros::Publisher raw_rtk_yaw_publisher;
  ros::Publisher rtk_yaw_publisher;
  ros::Publisher rtk_position_info_publisher;
  ros::Publisher rtk_yaw_info_publisher;
  ros::Publisher rtk_connection_status_publisher;
  ros::Publisher flight_anomaly_publisher;
  //! Local Position Publisher (Publishes local position in ENU frame)
  ros::Publisher local_position_publisher;
  ros::Publisher local_frame_ref_publisher;
  //! Local RTK Position Publisher (Publishes local RTK position in ENU frame)
  ros::Publisher local_rtk_position_publisher;
  ros::Publisher local_rtk_frame_ref_publisher;
  //! Local RTK/GPS fused position publisher (Publishes high rate local RTK position in ENU frame)
  ros::Publisher local_rtk_fused_position_publisher;
  ros::Publisher time_sync_nmea_publisher;
  ros::Publisher time_sync_gps_utc_publisher;
  ros::Publisher time_sync_fc_utc_publisher;
  ros::Publisher time_sync_pps_source_publisher;

#ifdef ADVANCED_SENSING
  ros::Publisher stereo_240p_front_left_publisher;
  ros::Publisher stereo_240p_front_right_publisher;
  ros::Publisher stereo_240p_down_front_publisher;
  ros::Publisher stereo_240p_down_back_publisher;
  ros::Publisher stereo_240p_front_depth_publisher;
  ros::Publisher stereo_vga_front_left_publisher;
  ros::Publisher stereo_vga_front_right_publisher;
  ros::Publisher main_camera_stream_publisher;
  ros::Publisher fpv_camera_stream_publisher;
#endif
  //! constant
  const int WAIT_TIMEOUT           = 10;
  const int MAX_SUBSCRIBE_PACKAGES = 5;
  const int INVALID_VERSION        = 0;

  //! configurations
  int         app_id;
  std::string enc_key;
  std::string drone_version;
  std::string serial_device;
  int         baud_rate;
  int         app_version;
  std::string app_bundle_id; // reserved
  int         uart_or_usb;
  double      gravity_const;

  //! use broadcast or subscription to get telemetry data
  TELEMETRY_TYPE telemetry_from_fc;
  bool stereo_subscription_success;
  bool stereo_vga_subscription_success;
  bool user_select_broadcast;
  const tf::Matrix3x3 R_FLU2FRD;
  const tf::Matrix3x3 R_ENU2NED;

  void flightControl(uint8_t flag, float32_t xSP, float32_t ySP, float32_t zSP, float32_t yawSP);

  enum AlignState
  {
    UNALIGNED,
    ALIGNING,
    ALIGNED
  };

  AlignState curr_align_state;

  static int constexpr STABLE_ALIGNMENT_COUNT = 400;
  static double constexpr TIME_DIFF_CHECK = 0.008;
  static double constexpr TIME_DIFF_ALERT = 0.020;

  ros::Time base_time;

  bool align_time_with_FC;

  bool local_pos_ref_set;
  bool local_rtk_pos_ref_set;

  void alignRosTimeWithFlightController(ros::Time now_time, uint32_t tick);
  void setUpM100DefaultFreq(uint8_t freq[16]);
  void setUpA3N3DefaultFreq(uint8_t freq[16]);
  void gpsConvertENU(double &ENU_x, double &ENU_y,
                     double gps_t_lon, double gps_t_lat,
                     double gps_r_lon, double gps_r_lat);

  double local_pos_ref_latitude, local_pos_ref_longitude, local_pos_ref_altitude;
  double current_gps_latitude, current_gps_longitude, current_gps_altitude;
  int current_gps_health;
  double local_rtk_pos_ref_latitude, local_rtk_pos_ref_longitude, local_rtk_pos_ref_altitude;
  double current_rtk_latitude, current_rtk_longitude, current_rtk_altitude;
  double bias_gps_latitude, bias_gps_longitude, bias_gps_altitude;
  int current_rtk_health;
  bool rtkSupport;
};

#endif // DJI_SDK_NODE_MAIN_H
