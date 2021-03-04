/** @file gimbal_camera_control_node.cpp
 *  @version 4.0
 *  @date May 2020
 *
 *  @brief sample node of gimbal & camera control .
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
#include <ros/ros.h>
#include <dji_osdk_ros/common_type.h>
#include <dji_osdk_ros/GimbalAction.h>
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

//CODE
using namespace dji_osdk_ros;

int main(int argc, char** argv) {
  ros::init(argc, argv, "gimbal_camera_control_node");
  ros::NodeHandle nh;

  auto gimbal_control_client = nh.serviceClient<GimbalAction>("gimbal_task_control");
  auto camera_set_EV_client = nh.serviceClient<CameraEV>("camera_task_set_EV");
  auto camera_set_shutter_speed_client = nh.serviceClient<CameraShutterSpeed>("camera_task_set_shutter_speed");
  auto camera_set_aperture_client = nh.serviceClient<CameraAperture>("camera_task_set_aperture");
  auto camera_set_iso_client = nh.serviceClient<CameraISO>("camera_task_set_ISO");
  auto camera_set_focus_point_client = nh.serviceClient<CameraFocusPoint>("camera_task_set_focus_point");
  auto camera_set_tap_zoom_point_client = nh.serviceClient<CameraTapZoomPoint>("camera_task_tap_zoom_point");
  auto camera_set_zoom_para_client = nh.serviceClient<CameraSetZoomPara>("camera_task_set_zoom_para");
  auto camera_task_zoom_ctrl_client = nh.serviceClient<CameraZoomCtrl>("camera_task_zoom_ctrl");
  auto camera_start_shoot_single_photo_client = nh.serviceClient<CameraStartShootSinglePhoto>(
      "camera_start_shoot_single_photo");
  auto camera_start_shoot_aeb_photo_client = nh.serviceClient<CameraStartShootAEBPhoto>("camera_start_shoot_aeb_photo");
  auto camera_start_shoot_burst_photo_client = nh.serviceClient<CameraStartShootBurstPhoto>(
      "camera_start_shoot_burst_photo");
  auto camera_start_shoot_interval_photo_client = nh.serviceClient<CameraStartShootIntervalPhoto>(
      "camera_start_shoot_interval_photo");
  auto camera_stop_shoot_photo_client = nh.serviceClient<CameraStopShootPhoto>("camera_stop_shoot_photo");
  auto camera_record_video_action_client = nh.serviceClient<CameraRecordVideoAction>("camera_record_video_action");

  /*! sample loop start */
  char inputChar = 0;
  while (true) {
    std::cout << std::endl;
    std::cout
        << "| Available commands:                                            |\n"
        << "| [a] Set camera shutter speed                                   |\n"
        << "|     Main camera : X5S, X7, Z30, H20/H20T(zoom mode) etc.       |\n"
        << "| [b] Set camera aperture                                        |\n"
        << "|     Main camera : X5S, X7, Z30, H20/H20T(zoom mode) etc.       |\n"
        << "| [c] Set camera EV value                                        |\n"
        << "|     Main camera : X5S, X7, Z30, H20/H20T(zoom mode) etc.       |\n"
        << "| [d] Set camera ISO value                                       |\n"
        << "|     Main camera : X5S, X7, Z30, H20/H20T(zoom mode) etc.       |\n"
        << "| [e] Set camera focus point                                     |\n"
        << "|     Main camera : X5S, X7, H20/H20T(zoom mode) etc.            |\n"
        << "| [f] Set camera tap zoom point                                  |\n"
        << "|     Vice camera : Z30, H20/H20T(zoom mode) etc.                |\n"
        << "| [g] Set camera zoom parameter                                  |\n"
        << "|     Vice camera : Z30, H20/H20T(zoom mode) etc.                |\n"
        << "| [h] Shoot Single photo Sample                                  |\n"
        << "|     Main camera : X5S, X7, XTS, Z30, H20/H20T etc.             |\n"
        << "| [i] Shoot AEB photo Sample                                     |\n"
        << "|     Main camera : X5S, X7 etc.                                 |\n"
        << "| [j] Shoot Burst photo Sample                                   |\n"
        << "|     Main camera : X5S, X7 etc.                                 |\n"
        << "| [k] Shoot Interval photo Sample                                |\n"
        << "|     Main camera : X5S, X7, XTS, Z30, H20/H20T etc.             |\n"
        << "| [l] Record video Sample                                        |\n"
        << "|     Main camera : X5S, X7, XTS, Z30, H20/H20T etc.             |\n"
        << "| [m] Rotate gimbal sample                                       |\n"
        << "|     Main camera : X5S, X7, XTS, Z30, H20/H20T etc.             |\n"
        << "| [n] Reset gimbal sample                                        |\n"
        << "|     Main camera : X5S, X7, XTS, Z30, H20/H20T etc.             |\n"
        << "| [q] Quit                                                       |\n";
    std::cin >> inputChar;

    switch (inputChar) {
      case 'a': {
        CameraShutterSpeed cameraShutterSpeed;
        cameraShutterSpeed.request.payload_index = static_cast<uint8_t>(dji_osdk_ros::PayloadIndex::PAYLOAD_INDEX_0);
        cameraShutterSpeed.request.exposure_mode = static_cast<uint8_t>(ExposureMode::EXPOSURE_MANUAL);
        cameraShutterSpeed.request.shutter_speed = static_cast<uint8_t>(ShutterSpeed::SHUTTER_SPEED_1_100);
        camera_set_shutter_speed_client.call(cameraShutterSpeed);
        sleep(2);
        break;
      }

      case 'b': {
        CameraAperture cameraAperture;
        cameraAperture.request.payload_index = static_cast<uint8_t>(dji_osdk_ros::PayloadIndex::PAYLOAD_INDEX_0);
        cameraAperture.request.exposure_mode = static_cast<uint8_t>(ExposureMode::EXPOSURE_MANUAL);
        cameraAperture.request.aperture = static_cast<uint16_t>(Aperture::F_4);
        camera_set_aperture_client.call(cameraAperture);
        sleep(2);
        break;
      }
      case 'c': {
        CameraEV cameraEv;
        cameraEv.request.payload_index = static_cast<uint8_t>(dji_osdk_ros::PayloadIndex::PAYLOAD_INDEX_0);
        cameraEv.request.exposure_mode = static_cast<uint8_t>(ExposureMode::PROGRAM_AUTO);
        cameraEv.request.exposure_compensation = static_cast<uint8_t>(ExposureCompensation::P_0_3);
        camera_set_EV_client.call(cameraEv);
        sleep(2);
        break;
      }

      case 'd': {
        CameraISO cameraIso;
        cameraIso.request.payload_index = static_cast<uint8_t>(dji_osdk_ros::PayloadIndex::PAYLOAD_INDEX_0);
        cameraIso.request.exposure_mode = static_cast<uint8_t>(ExposureMode::EXPOSURE_MANUAL);
        cameraIso.request.iso_data = static_cast<uint8_t>(ISO::ISO_200);
        camera_set_iso_client.call(cameraIso);
        sleep(2);
        break;
      }

      case 'e': {
        CameraFocusPoint cameraFocusPoint;
        cameraFocusPoint.request.payload_index = static_cast<uint8_t>(dji_osdk_ros::PayloadIndex::PAYLOAD_INDEX_0);
        cameraFocusPoint.request.x = 0.8;
        cameraFocusPoint.request.y = 0.8;
        camera_set_focus_point_client.call(cameraFocusPoint);
        break;
      }

      case 'f': {
        CameraTapZoomPoint cameraTapZoomPoint;
        cameraTapZoomPoint.request.payload_index = static_cast<uint8_t>(dji_osdk_ros::PayloadIndex::PAYLOAD_INDEX_1);
        cameraTapZoomPoint.request.multiplier = 5;
        cameraTapZoomPoint.request.x = 0.3;
        cameraTapZoomPoint.request.y = 0.3;
        camera_set_tap_zoom_point_client.call(cameraTapZoomPoint);
        sleep(5);
        cameraTapZoomPoint.request.x = 0.8;
        cameraTapZoomPoint.request.y = 0.7;
        camera_set_tap_zoom_point_client.call(cameraTapZoomPoint);
        sleep(5);
        break;
      }
      case 'g': {
        CameraSetZoomPara cameraSetZoomPara;
        cameraSetZoomPara.request.payload_index = static_cast<uint8_t>(dji_osdk_ros::PayloadIndex::PAYLOAD_INDEX_1);
        cameraSetZoomPara.request.factor = 5;
        camera_set_zoom_para_client.call(cameraSetZoomPara);
        sleep(4);
        cameraSetZoomPara.request.factor = 10;
        camera_set_zoom_para_client.call(cameraSetZoomPara);
        sleep(4);
        CameraZoomCtrl cameraZoomCtrl;
        cameraZoomCtrl.request.start_stop = 1;
        cameraZoomCtrl.request.payload_index = static_cast<uint8_t>(dji_osdk_ros::PayloadIndex::PAYLOAD_INDEX_1);
        cameraZoomCtrl.request.direction = static_cast<uint8_t>(dji_osdk_ros::ZoomDirection::ZOOM_OUT);
        cameraZoomCtrl.request.speed = static_cast<uint8_t>(dji_osdk_ros::ZoomSpeed::FASTEST);
        camera_task_zoom_ctrl_client.call(cameraZoomCtrl);
        sleep(8);
        cameraZoomCtrl.request.start_stop = 0;
        cameraZoomCtrl.request.payload_index = static_cast<uint8_t>(dji_osdk_ros::PayloadIndex::PAYLOAD_INDEX_1);
        camera_task_zoom_ctrl_client.call(cameraZoomCtrl);
        break;
      }

      case 'h': {
        CameraStartShootSinglePhoto cameraStartShootSinglePhoto;
        cameraStartShootSinglePhoto.request.payload_index = static_cast<uint8_t>(dji_osdk_ros::PayloadIndex::PAYLOAD_INDEX_0);
        camera_start_shoot_single_photo_client.call(cameraStartShootSinglePhoto);
        break;
      }

      case 'i': {
        CameraStartShootAEBPhoto cameraStartShootAebPhoto;
        cameraStartShootAebPhoto.request.payload_index = static_cast<uint8_t>(dji_osdk_ros::PayloadIndex::PAYLOAD_INDEX_0);
        cameraStartShootAebPhoto.request.photo_aeb_count = static_cast<uint8_t>(dji_osdk_ros::PhotoAEBCount::AEB_COUNT_5);
        camera_start_shoot_aeb_photo_client.call(cameraStartShootAebPhoto);
        break;
      }

      case 'j': {
        CameraStartShootBurstPhoto cameraStartShootBurstPhoto;
        cameraStartShootBurstPhoto.request.payload_index = static_cast<uint8_t>(dji_osdk_ros::PayloadIndex::PAYLOAD_INDEX_0);
        cameraStartShootBurstPhoto.request.photo_burst_count = static_cast<uint8_t>(dji_osdk_ros::PhotoBurstCount::BURST_COUNT_7);
        camera_start_shoot_burst_photo_client.call(cameraStartShootBurstPhoto);
        break;
      }

      case 'k': {
        CameraStartShootIntervalPhoto cameraStartShootIntervalPhoto;
        cameraStartShootIntervalPhoto.request.payload_index = static_cast<uint8_t>(dji_osdk_ros::PayloadIndex::PAYLOAD_INDEX_0);
        cameraStartShootIntervalPhoto.request.photo_num_conticap = 255;
        cameraStartShootIntervalPhoto.request.time_interval = 3;
        camera_start_shoot_interval_photo_client.call(cameraStartShootIntervalPhoto);
        std::cout << "Sleep 15 seconds" << std::endl;
        sleep(15);
        CameraStopShootPhoto cameraStopShootPhoto;
        cameraStopShootPhoto.request.payload_index = static_cast<uint8_t>(dji_osdk_ros::PayloadIndex::PAYLOAD_INDEX_0);
        camera_stop_shoot_photo_client.call(cameraStopShootPhoto);
        break;
      }

      case 'l': {
        CameraRecordVideoAction cameraRecordVideoAction;
        cameraRecordVideoAction.request.start_stop = 1;
        cameraRecordVideoAction.request.payload_index = static_cast<uint8_t>(dji_osdk_ros::PayloadIndex::PAYLOAD_INDEX_0);
        camera_record_video_action_client.call(cameraRecordVideoAction);
        sleep(10);
        cameraRecordVideoAction.request.start_stop = 0;
        camera_record_video_action_client.call(cameraRecordVideoAction);
        break;
      }

      case 'm': {
        GimbalAction gimbalAction;
        gimbalAction.request.is_reset = false;
        gimbalAction.request.payload_index = static_cast<uint8_t>(dji_osdk_ros::PayloadIndex::PAYLOAD_INDEX_0);
        gimbalAction.request.rotationMode = 0;
        gimbalAction.request.pitch = 25.0f;
        gimbalAction.request.roll = 0.0f;
        gimbalAction.request.yaw = 90.0f;
        gimbalAction.request.time = 0.5;
        gimbal_control_client.call(gimbalAction);
        break;
      }

      case 'n': {
        GimbalAction gimbalAction;
        gimbalAction.request.is_reset = true;
        gimbalAction.request.payload_index = static_cast<uint8_t>(dji_osdk_ros::PayloadIndex::PAYLOAD_INDEX_0);
        gimbal_control_client.call(gimbalAction);
        break;
      }

      case 'q': {
        std::cout << "Quit now ..." << std::endl;
        return 0;
      }

      default:
        break;
    }
  }
}