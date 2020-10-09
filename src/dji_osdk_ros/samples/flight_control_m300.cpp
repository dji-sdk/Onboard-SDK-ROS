/** @file flight_control_m300.cpp
 *  @version 4.0.1
 *  @date May 2020
 *
 *  @brief sample node of flight control test for m300.
 *
 *  @Copyright (c) 2020 SensynRobotics
 *
 */

//INCLUDE
#include <ros/ros.h>
#include <dji_osdk_ros/common_type.h>

#include <dji_osdk_ros/FlightTaskControl.h>
#include <dji_osdk_ros/ObtainControlAuthority.h>

//CODE
using namespace dji_osdk_ros;

ros::ServiceClient flight_control_client_pos;
ros::ServiceClient flight_control_client_vel;
ros::ServiceClient flight_control_client_ang;

bool moveByPosOffset(ros::ServiceClient& client, FlightTaskControl& task, MoveOffset&& move_offset);


int main(int argc, char** argv)
{
  ros::init(argc, argv, "flight_control_m300");
  ros::NodeHandle nh;
  flight_control_client_pos = nh.serviceClient<FlightTaskControl>("/flight_control_position");
  flight_control_client_vel = nh.serviceClient<FlightTaskControl>("/flight_control_velocity");
  flight_control_client_ang = nh.serviceClient<FlightTaskControl>("/flight_control_angle");
  auto obtain_ctrl_authority_client = nh.serviceClient<dji_osdk_ros::ObtainControlAuthority>("obtain_release_control_authority");

  FlightTaskControl control_task;
  ObtainControlAuthority obtainCtrlAuthority;
  obtainCtrlAuthority.request.enable_obtain = true;
  obtain_ctrl_authority_client.call(obtainCtrlAuthority);
  if (obtainCtrlAuthority.response.result == true) {
    ROS_INFO("ObtainControlAuthority successful");
  } else {
    ROS_ERROR("ObtainControlAuthority failed!!!");
  }

  std::cout
      << "| Available commands:                                            |"
      << std::endl;
  std::cout
      << "| [a] Move by Position                                           |"
      << std::endl;
  std::cout
      << "| [b] Move by Velocity                                           |"
      << std::endl;
  std::cout
      << "| [c] Move by Angle                                              |"
      << std::endl;

  std::cout << "Please select command: ";
  char inputChar;
  std::cin >> inputChar;
  switch (inputChar)
  {
    case 'a':
      {
        ROS_INFO("Move by position ...");
        moveByPosOffset(flight_control_client_pos, control_task, MoveOffset(1.0, 0.0, 0.0, 0.0));
        break;
      }
    case 'b':
      {
        ROS_INFO("Move by velocity ...");
        moveByPosOffset(flight_control_client_vel, control_task, MoveOffset(1.0, 0.0, 0.0, 0.0));
        break;
      }
    case 'c':
      {
        ROS_INFO("Move by angle ...");
        moveByPosOffset(flight_control_client_ang, control_task, MoveOffset(1.0, 0.0, 0.0, 0.0));
        break;
      }
    default:
      break;
  }

  ROS_INFO("Finished. Press CTRL-C to terminate the node");

  ros::spin();
  return 0;
}


bool moveByPosOffset(ros::ServiceClient& client, FlightTaskControl& task, MoveOffset&& move_offset)
{
  task.request.task = FlightTaskControl::Request::TASK_GO_LOCAL_POS;
  // pos_offset: A vector contains that position_x_offset, position_y_offset, position_z_offset in order
  task.request.pos_offset.clear();
  task.request.pos_offset.push_back(move_offset.x);
  task.request.pos_offset.push_back(move_offset.y);
  task.request.pos_offset.push_back(move_offset.z);
  // yaw_params: A vector contains that yaw_desired, position_threshold(Meter), yaw_threshold(Degree)
  task.request.yaw_params.clear();
  task.request.yaw_params.push_back(move_offset.yaw);
  task.request.yaw_params.push_back(move_offset.pos_threshold);
  task.request.yaw_params.push_back(move_offset.yaw_threshold);
  client.call(task);
  return task.response.result;
}
