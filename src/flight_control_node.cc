/**
  * @Copyright (C) 2020 All rights reserved.
  * @date: 2020
  * @file: flight_control_node.cc
  * @version: v0.0.1
  * @author: kevin.hoo@dji.com
  * @create_date: 2020-03-08 21:38:56
  * @last_modified_date: 2020-03-08 22:35:46
  * @brief: TODO
  * @details: TODO
  */

//INCLUDE
#include <ros/ros.h>
#include <dji_osdk_ros/DroneTaskControl.h>
#include <dji_osdk_ros/common_type.hh>

//CODE
using namespace dji_osdk_ros;

ros::ServiceClient task_control_client;

bool moveByPosOffset(DroneTaskControl& task, MoveOffset&& move_offset)
{
  task.request.task = DroneTaskControl::Request::TASK_GO_LOCAL_POS;
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
  task_control_client.call(task);
  return task.response.result;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "flight_control_node");
  ros::NodeHandle nh;
  task_control_client = nh.serviceClient<DroneTaskControl>("/drone_task_control");

  std::cout
      << "| Available commands:                                            |"
      << std::endl;
  std::cout
      << "| [a] Monitored Takeoff + Landing                                |"
      << std::endl;
  std::cout
      << "| [b] Monitored Takeoff + Position Control + Landing             |"
      << std::endl;
  std::cout << "| [c] Monitored Takeoff + Position Control + Force Landing "
               "Avoid Ground  |"
            << std::endl;

  std::cout << "Please select command: ";
  char inputChar;
  std::cin >> inputChar;

  DroneTaskControl control_task;

  switch (inputChar)
  {
    case 'a':
      {
        control_task.request.task = DroneTaskControl::Request::TASK_TAKEOFF;
        ROS_INFO_STREAM("Takeoff request sending ...");
        task_control_client.call(control_task);
        if(control_task.response.result == true)
        {
          ROS_INFO_STREAM("Takeoff task successful");
          ros::Duration(2.0).sleep();

          ROS_INFO_STREAM("Land request sending ...");
          control_task.request.task = DroneTaskControl::Request::TASK_LAND;
          task_control_client.call(control_task);
          if(control_task.response.result == true)
          {
            ROS_INFO_STREAM("Land task successful");
            break;
          }
          ROS_INFO_STREAM("Land task failed.");
          break;
        }
        ROS_ERROR_STREAM("Takeoff task failed");
        break;
      }
    case 'b':
      {
        control_task.request.task = DroneTaskControl::Request::TASK_TAKEOFF;
        ROS_INFO_STREAM("Takeoff request sending ...");
        task_control_client.call(control_task);
        if(control_task.response.result == true)
        {
          ROS_INFO_STREAM("Takeoff task successful");
          ros::Duration(2.0).sleep();
          
          ROS_INFO_STREAM("Move by position offset request sending ...");
          moveByPosOffset(control_task, MoveOffset(0.0, 6.0, 6.0, 30.0, 1.0, 0.5));
          ros::Duration(2.0).sleep();
          moveByPosOffset(control_task, MoveOffset(-6.0, 0.0, -3.0, -30.0, 1.0, 0.5));
          ros::Duration(2.0).sleep();
          moveByPosOffset(control_task, MoveOffset(0.0, -6.0, 0.0, -30.0, 1.0, 0.5));
          ros::Duration(2.0).sleep();

          control_task.request.task = DroneTaskControl::Request::TASK_TAKEOFF;
          ROS_INFO_STREAM("Takeoff request sending ...");
          task_control_client.call(control_task);
          if(control_task.response.result == true)
          {
            ROS_INFO_STREAM("Land task successful");
            break;
          }
          ROS_INFO_STREAM("Land task failed.");
          break;
        }
        break;
      }
    case 'c':
      {
        break;
      }
    default:
      break;
  }

  ROS_INFO_STREAM("Control Task has been done, press CTRL-C to terminate the node");

  ros::spin();
  return 0;
}
