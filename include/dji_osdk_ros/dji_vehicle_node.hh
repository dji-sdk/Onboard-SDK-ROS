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

#include <dji_osdk_ros/vehicle_wrapper.hh>
#include <dji_osdk_ros/common_type.hh>

#include <memory>
#include <string>

#include <dji_osdk_ros/DroneTaskControl.h>
#include <dji_osdk_ros/GimbalAction.h>
#include <dji_osdk_ros/CameraAction.h>
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
      ~VehicleNode() = default;

      void initService();
      void initTopic();
      bool publishTopic();
#ifdef ADVANCED_SENSING
      dji_osdk_ros::CameraData getCameraData();
#endif
    protected:
      ros::ServiceServer task_control_server_;
      ros::ServiceServer gimbal_control_server_;
      ros::ServiceServer camera_action_control_server_;
      ros::ServiceServer mfio_control_server_;

      ros::ServiceServer set_home_altitude_server_;
      ros::ServiceServer set_current_point_as_home_server_;
      ros::ServiceServer avoid_enable_server_;
#ifdef ADVANCED_SENSING
      ros::ServiceServer advanced_sensing_server_;
      ros::Publisher advanced_sensing_pub_;
#endif

    protected:
      bool taskCtrlCallback(DroneTaskControl::Request& request, DroneTaskControl::Response& response);

      bool gimbalCtrlCallback(GimbalAction::Request& request, GimbalAction::Response& response);
      bool cameraCtrlCallback(CameraAction::Request& request, CameraAction::Response& response);

      bool mfioCtrlCallback(MFIO::Request& request, MFIO::Response& response);

      bool setGoHomeAltitudeCallback(SetGoHomeAltitude::Request& request, SetGoHomeAltitude::Response& response);
      bool setHomeCallback(SetNewHomePoint::Request& request, SetNewHomePoint::Response& response);
      bool setAvoidCallback(AvoidEnable::Request& request, AvoidEnable::Response& response);

#ifdef ADVANCED_SENSING
      bool advancedSensingCallback(AdvancedSensing::Request& request, AdvancedSensing::Response& response);
      bool publishAdvancedSeningData();
#endif

      bool initSubscribe();

    private:
      ros::NodeHandle nh_;
      std::unique_ptr<VehicleWrapper> ptr_wrapper_;

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

