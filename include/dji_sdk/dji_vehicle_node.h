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

#include <dji_sdk/vehicle_wrapper.h>
#include <dji_sdk/common_type.h>

#include <memory>
#include <string>

#include <dji_sdk/FlightTaskControl.h>
#include <dji_sdk/GimbalAction.h>
#include <dji_sdk/CameraEV.h>
#include <dji_sdk/CameraShutterSpeed.h>
#include <dji_sdk/CameraAperture.h>
#include <dji_sdk/CameraISO.h>
#include <dji_sdk/CameraFocusPoint.h>
#include <dji_sdk/CameraTapZoomPoint.h>
#include <dji_sdk/CameraZoomCtrl.h>
#include <dji_sdk/CameraStartShootBurstPhoto.h>
#include <dji_sdk/CameraStartShootAEBPhoto.h>
#include <dji_sdk/CameraStartShootSinglePhoto.h>
#include <dji_sdk/CameraStartShootIntervalPhoto.h>
#include <dji_sdk/CameraStopShootPhoto.h>
#include <dji_sdk/CameraRecordVideoAction.h>
#include <dji_sdk/MFIO.h>
#include <dji_sdk/SetGoHomeAltitude.h>
#include <dji_sdk/SetNewHomePoint.h>
#include <dji_sdk/AvoidEnable.h>
#ifdef ADVANCED_SENSING
#include <dji_sdk/AdvancedSensing.h>
#include <dji_sdk/CameraData.h>
#endif




// Declaration
namespace dji_sdk
{
  using namespace DJI::OSDK;
  using namespace Telemetry;

  class VehicleNode
  {
    public:
      VehicleNode();
      VehicleNode(int test);

      ~VehicleNode();

      bool subscribeGimbalData();
      bool unSubScribeGimbalData();
      bool initCameraModule();
      void initService();
      void initTopic();
      void publishTopic();
#ifdef ADVANCED_SENSING
      dji_sdk::CameraData getCameraData();
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

