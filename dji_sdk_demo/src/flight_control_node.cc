/** @file flight_control_node.cc
 *  @version 4.0
 *  @date Mar, 2020
 *
 *  @brief
 *  FlightControl:
 *
 *  @copyright 2020 DJI. All rights reserved.
 *
 */

#include <ros/ros.h>
#include <dji_sdk_demo/flight_controller_ros.hh>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "flight_control_node");
  FlightControllerROS flight_controller;
  flight_controller.init();
  if(flight_controller.takeoff() == false)
  {
    ROS_ERROR_STREAM("Takeoff Failed!");
  }
  else
  {

  }


  flight_controller.deinit();
  while(ros::ok());
  return 0;
}