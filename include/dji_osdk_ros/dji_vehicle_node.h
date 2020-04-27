#ifndef __DJI_VEHICLE_NODE_HH__
#define __DJI_VEHICLE_NODE_HH__
/**-----------------------------------------------
  * @Copyright (C) 2020 All rights reserved.
  * @date: 2020
  * @file: dji_vehicle_node.hh
  * @version: v0.0.1
  * @author: kevin.hoo@dji.com
  * @create_date: 2020-03-08 16:08:55
  * @last_modified_date: 2020-03-10 13:30:22
  * @brief: TODO
  * @details: TODO
  *-----------------------------------------------*/

// Header include
#include <ros/ros.h>
#include <dji_vehicle.hpp>

#include <dji_osdk_ros/vehicle_wrapper.h>
#include <dji_osdk_ros/common_type.h>

#include <memory>
#include <string>

#include <dji_osdk_ros/FlightTaskControl.h>
#include <dji_osdk_ros/GimbalAction.h>
#include <dji_osdk_ros/CameraTaskControl.h>
#include <dji_osdk_ros/CameraEV.h>
#include <dji_osdk_ros/CameraShutterSpeed.h>
#include <dji_osdk_ros/CameraAperture.h>
#include <dji_osdk_ros/CameraISO.h>
#include <dji_osdk_ros/CameraFocusPoint.h>
#include <dji_osdk_ros/CameraTapZoomPoint.h>
#include <dji_osdk_ros/CameraZoomCtrl.h>
#include <dji_osdk_ros/CameraStartShootBurstPhoto.h>
#include <dji_osdk_ros/CameraStartShootAEBPhoto.h>
#include <dji_osdk_ros/CameraStartShootSinglePhoto.h>
#include <dji_osdk_ros/CameraStartShootIntervalPhoto.h>
#include <dji_osdk_ros/CameraStopShootPhoto.h>
#include <dji_osdk_ros/CameraRecordVideoAction.h>
#include <dji_osdk_ros/MFIO.h>
#include <dji_osdk_ros/SetGoHomeAltitude.h>
#include <dji_osdk_ros/SetNewHomePoint.h>
#include <dji_osdk_ros/AvoidEnable.h>
#ifdef ADVANCED_SENSING
#include <dji_osdk_ros/AdvancedSensing.h>
#include <dji_osdk_ros/CameraData.h>
#endif




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
      //~VehicleNode() = default;
      ~VehicleNode();

      bool subscribeGimbalData();
      bool unSubScribeGimbalData();
      bool initCameraModule();
      void initService();
      void initTopic();
      void publishTopic();
#ifdef ADVANCED_SENSING
      dji_osdk_ros::CameraData getCameraData();
#endif
    protected:
      ros::ServiceServer task_control_server_;
      ros::ServiceServer gimbal_control_server_;
      /*! for camera*/
      ros::ServiceServer camera_control_set_EV_server_;
      ros::ServiceServer camera_control_set_shutter_speed_server_;
      ros::ServiceServer camera_control_set_aperture_server_;
      ros::ServiceServer camera_control_set_ISO_server_;
      ros::ServiceServer camera_control_set_focus_point_server_;
      ros::ServiceServer camera_control_set_tap_zoom_point_server_;
      ros::ServiceServer camera_control_zoom_ctrl_server_;
      ros::ServiceServer camera_control_start_shoot_single_photo_server_;
      ros::ServiceServer camera_control_start_shoot_burst_photo_server_;
      ros::ServiceServer camera_control_start_shoot_AEB_photo_server_;
      ros::ServiceServer camera_control_start_shoot_interval_photo_server_;
      ros::ServiceServer camera_control_stop_shoot_photo_server_;
      ros::ServiceServer camera_control_record_video_action_server_;

//      ros::ServiceServer camera_action_control_server_;
      ros::ServiceServer mfio_control_server_;

      ros::ServiceServer set_home_altitude_server_;
      ros::ServiceServer set_current_point_as_home_server_;
      ros::ServiceServer avoid_enable_server_;
#ifdef ADVANCED_SENSING
      ros::ServiceServer advanced_sensing_server_;
      ros::Publisher advanced_sensing_pub_;
#endif

    protected:
      bool taskCtrlCallback(FlightTaskControl::Request& request, FlightTaskControl::Response& response);
      /*! for gimbal */
      bool gimbalCtrlCallback(GimbalAction::Request& request, GimbalAction::Response& response);
      /*! for camera*/
      bool cameraSetEVCallback(CameraEV::Request& request, CameraEV::Response& response);
      bool cameraSetShutterSpeedCallback(CameraShutterSpeed::Request& request, CameraShutterSpeed::Response& response);
      bool cameraSetApertureCallback(CameraAperture::Request& request, CameraAperture::Response& response);
      bool cameraSetISOCallback(CameraISO::Request& request, CameraISO::Response& response);
      bool cameraSetFocusPointCallback(CameraFocusPoint::Request& request, CameraFocusPoint::Response& response);
      bool cameraSetTapZoomPointCallback(CameraTapZoomPoint::Request& request, CameraTapZoomPoint::Response& response);
      bool cameraZoomCtrlCallback(CameraZoomCtrl::Request& request, CameraZoomCtrl::Response& response);
      bool cameraStartShootSinglePhotoCallback(CameraStartShootSinglePhoto::Request& request, CameraStartShootSinglePhoto::Response& response);
      bool cameraStartShootAEBPhotoCallback(CameraStartShootAEBPhoto::Request& request, CameraStartShootAEBPhoto::Response& response);
      bool cameraStartShootBurstPhotoCallback(CameraStartShootBurstPhoto::Request& request, CameraStartShootBurstPhoto::Response& response);
      bool cameraStartShootIntervalPhotoCallback(CameraStartShootIntervalPhoto::Request& request, CameraStartShootIntervalPhoto::Response& response);
      bool cameraStopShootPhotoCallback(CameraStopShootPhoto::Request& request, CameraStopShootPhoto::Response& response);
      bool cameraRecordVideoActionCallback(CameraRecordVideoAction::Request& request, CameraRecordVideoAction::Response& response);

      bool mfioCtrlCallback(MFIO::Request& request, MFIO::Response& response);

      bool setGoHomeAltitudeCallback(SetGoHomeAltitude::Request& request, SetGoHomeAltitude::Response& response);
      bool setHomeCallback(SetNewHomePoint::Request& request, SetNewHomePoint::Response& response);
      bool setAvoidCallback(AvoidEnable::Request& request, AvoidEnable::Response& response);

#ifdef ADVANCED_SENSING
      bool advancedSensingCallback(AdvancedSensing::Request& request, AdvancedSensing::Response& response);
      void publishAdvancedSeningData();
#endif

      bool initSubscribe();

    private:
      ros::NodeHandle nh_;
      VehicleWrapper* ptr_wrapper_;

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

#ifdef ADVANCED_SENSING
      bool is_h264_;
#endif
  };
}
#endif // __DJI_VEHICLE_NODE_HH__

