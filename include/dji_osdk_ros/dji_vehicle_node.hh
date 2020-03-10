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
#include <djiosdk/dji_vehicle.hpp>

#include <dji_osdk_ros/vehicle_wrapper.hh>
#include <dji_osdk_ros/common_type.hh>

#include <memory>
#include <string>

#include <dji_osdk_ros/DroneTaskControl.h>
#include <dji_osdk_ros/GimbalAction.h>
#include <dji_osdk_ros/CameraAction.h>
#include <dji_osdk_ros/MFIO.h>



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

    protected:
      ros::ServiceServer task_control_server_;
      ros::ServiceServer gimbal_control_server_;
      ros::ServiceServer camera_action_control_server_;
      ros::ServiceServer mfio_control_server_;

    //ros::ServiceServer camera_zoom_control_server_;

    protected:
      bool taskCtrlCallback(DroneTaskControl::Request& request, DroneTaskControl::Response& response);
      bool gimbalCtrlCallback(GimbalAction::Request& request, GimbalAction::Response& response);
      bool cameraCtrlCallback(CameraAction::Request& request, CameraAction::Response& response);
      bool mfioCtrlCallback(MFIO::Request& request, MFIO::Response& response);

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
  };
}
#endif // __DJI_VEHICLE_NODE_HH__

