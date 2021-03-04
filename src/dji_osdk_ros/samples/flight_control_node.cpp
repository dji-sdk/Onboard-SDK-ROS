/** @file advanced_sensing_node.cpp
 *  @version 4.0
 *  @date May 2020
 *
 *  @brief sample node of flight control.
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

#include <dji_osdk_ros/FlightTaskControl.h>
#include <dji_osdk_ros/SetGoHomeAltitude.h>
#include <dji_osdk_ros/GetGoHomeAltitude.h>
#include <dji_osdk_ros/SetCurrentAircraftLocAsHomePoint.h>
#include <dji_osdk_ros/SetAvoidEnable.h>
#include <dji_osdk_ros/ObtainControlAuthority.h>
#include <dji_osdk_ros/EmergencyBrake.h>
#include <dji_osdk_ros/GetAvoidEnable.h>

#include<dji_osdk_ros/SetJoystickMode.h>
#include<dji_osdk_ros/JoystickAction.h>

//CODE
using namespace dji_osdk_ros;

ros::ServiceClient task_control_client;
ros::ServiceClient set_joystick_mode_client;
ros::ServiceClient joystick_action_client;

bool moveByPosOffset(FlightTaskControl& task,const JoystickCommand &offsetDesired,
                     float posThresholdInM = 0.8,
                     float yawThresholdInDeg = 1.0);

void velocityAndYawRateCtrl(const JoystickCommand &offsetDesired, uint32_t timeMs);

int main(int argc, char** argv)
{
  ros::init(argc, argv, "flight_control_node");
  ros::NodeHandle nh;
  task_control_client = nh.serviceClient<FlightTaskControl>("/flight_task_control");
  auto set_go_home_altitude_client = nh.serviceClient<SetGoHomeAltitude>("/set_go_home_altitude");
  auto get_go_home_altitude_client = nh.serviceClient<GetGoHomeAltitude>("get_go_home_altitude");
  auto set_current_point_as_home_client = nh.serviceClient<SetCurrentAircraftLocAsHomePoint>("/set_current_aircraft_point_as_home");
  auto enable_horizon_avoid_client  = nh.serviceClient<SetAvoidEnable>("/set_horizon_avoid_enable");
  auto enable_upward_avoid_client   = nh.serviceClient<SetAvoidEnable>("/set_upwards_avoid_enable");
  auto get_avoid_enable_client      = nh.serviceClient<GetAvoidEnable>("get_avoid_enable_status");
  auto obtain_ctrl_authority_client = nh.serviceClient<dji_osdk_ros::ObtainControlAuthority>("obtain_release_control_authority");
  auto emergency_brake_client       = nh.serviceClient<dji_osdk_ros::EmergencyBrake>("emergency_brake");

  set_joystick_mode_client = nh.serviceClient<SetJoystickMode>("set_joystick_mode");
  joystick_action_client   = nh.serviceClient<JoystickAction>("joystick_action");
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
  std::cout << "| [d] Monitored Takeoff + Velocity Control + Landing |"
            << std::endl;

  std::cout << "Please select command: ";
  char inputChar;
  std::cin >> inputChar;
  EmergencyBrake emergency_brake;
  FlightTaskControl control_task;
  ObtainControlAuthority obtainCtrlAuthority;
  
  obtainCtrlAuthority.request.enable_obtain = true;
  obtain_ctrl_authority_client.call(obtainCtrlAuthority);

  switch (inputChar)
  {
    case 'a':
      {
        control_task.request.task = FlightTaskControl::Request::TASK_TAKEOFF;
        ROS_INFO_STREAM("Takeoff request sending ...");
        task_control_client.call(control_task);
        if(control_task.response.result == true)
        {
          ROS_INFO_STREAM("Takeoff task successful");
          ros::Duration(2.0).sleep();

          ROS_INFO_STREAM("Land request sending ...");
          control_task.request.task = FlightTaskControl::Request::TASK_LAND;
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
        control_task.request.task = FlightTaskControl::Request::TASK_TAKEOFF;
        ROS_INFO_STREAM("Takeoff request sending ...");
        task_control_client.call(control_task);
        if(control_task.response.result == false)
        {
          ROS_ERROR_STREAM("Takeoff task failed");
          break;
        }

        if(control_task.response.result == true)
        {
          ROS_INFO_STREAM("Takeoff task successful");
          ros::Duration(2.0).sleep();

          ROS_INFO_STREAM("Move by position offset request sending ...");
          moveByPosOffset(control_task, {0.0, 6.0, 6.0, 30.0}, 0.8, 1);
          ROS_INFO_STREAM("Step 1 over!");
          moveByPosOffset(control_task, {6.0, 0.0, -3, -30.0}, 0.8, 1);
          ROS_INFO_STREAM("Step 2 over!");
          moveByPosOffset(control_task, {-6.0, -6.0, 0.0, 0.0}, 0.8, 1);
          ROS_INFO_STREAM("Step 3 over!");

          control_task.request.task = FlightTaskControl::Request::TASK_LAND;
          ROS_INFO_STREAM("Landing request sending ...");
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
        control_task.request.task = FlightTaskControl::Request::TASK_TAKEOFF;
        ROS_INFO_STREAM("Takeoff request sending ...");
        task_control_client.call(control_task);
        if(control_task.response.result == false)
        {
          ROS_ERROR_STREAM("Takeoff task failed");
          break;
        }

        if (control_task.response.result == true)
        {
          ROS_INFO_STREAM("Takeoff task successful");
          ros::Duration(2.0).sleep();

          ROS_INFO_STREAM("turn on Horizon_Collision-Avoidance-Enabled");
          SetAvoidEnable horizon_avoid_req;
          horizon_avoid_req.request.enable = true;
          enable_horizon_avoid_client.call(horizon_avoid_req);
          if(horizon_avoid_req.response.result == false)
          {
            ROS_ERROR_STREAM("Enable Horizon Avoid FAILED");
          }

          ROS_INFO_STREAM("turn on Upwards-Collision-Avoidance-Enabled");
          SetAvoidEnable upward_avoid_req;
          upward_avoid_req.request.enable = true;
          enable_upward_avoid_client.call(upward_avoid_req);
          if(upward_avoid_req.response.result == false)
          {
            ROS_ERROR_STREAM("Enable Upward Avoid FAILED");
          }

          GetAvoidEnable getAvoidEnable;
          get_avoid_enable_client.call(getAvoidEnable);
          if (getAvoidEnable.response.result)
          {
            ROS_INFO("get horizon avoid enable status:%d, get upwards avoid enable status:%d",
                     getAvoidEnable.response.horizon_avoid_enable_status,
                     getAvoidEnable.response.upwards_avoid_enable_status);
          }

          ROS_INFO_STREAM("Move by position offset request sending ...");
          ROS_INFO_STREAM("Move to higher altitude");
          moveByPosOffset(control_task, {0.0, 0.0, 30.0, 0.0}, 0.8, 1);
          ROS_INFO_STREAM("Move a short distance");
          moveByPosOffset(control_task, {10.0, 0.0, 0.0, 0.0}, 0.8, 1);

          ROS_INFO_STREAM("Set aircraft current position as new home location");
          SetCurrentAircraftLocAsHomePoint home_set_req;
          set_current_point_as_home_client.call(home_set_req);
          if(home_set_req.response.result == false)
          {
            ROS_ERROR_STREAM("Set current position as Home, FAILED");
            break;
          }


          ROS_INFO_STREAM("Get current go home altitude");
          GetGoHomeAltitude current_go_home_altitude;
          get_go_home_altitude_client.call(current_go_home_altitude);
          if(current_go_home_altitude.response.result == false)
          {
            ROS_ERROR_STREAM("Get altitude for go home FAILED");
            break;
          }
          ROS_INFO("Current go home altitude is :%d m", current_go_home_altitude.response.altitude);

          ROS_INFO_STREAM("Set new go home altitude");
          SetGoHomeAltitude altitude_go_home;
          altitude_go_home.request.altitude = 50;
          set_go_home_altitude_client.call(altitude_go_home);
          if(altitude_go_home.response.result == false)
          {
            ROS_ERROR_STREAM("Set altitude for go home FAILED");
            break;
          }

          get_go_home_altitude_client.call(current_go_home_altitude);
          if(current_go_home_altitude.response.result == false)
          {
            ROS_ERROR_STREAM("Get altitude for go home FAILED");
            break;
          }
          ROS_INFO("Current go home altitude is :%d m", current_go_home_altitude.response.altitude);

          ROS_INFO_STREAM("Move to another position");
          moveByPosOffset(control_task, {50.0, 0.0, 0.0, 0.0} , 0.8, 1);

          ROS_INFO_STREAM("Shut down Horizon_Collision-Avoidance-Enabled");
          horizon_avoid_req.request.enable = false;
          enable_horizon_avoid_client.call(horizon_avoid_req);
          if(horizon_avoid_req.response.result == false)
          {
            ROS_ERROR_STREAM("Disable Horizon Avoid FAILED");
          }

          ROS_INFO_STREAM("Shut down Upwards-Collision-Avoidance-Enabled");
          upward_avoid_req.request.enable = false;
          enable_upward_avoid_client.call(upward_avoid_req);
          if(upward_avoid_req.response.result == false)
          {
            ROS_ERROR_STREAM("Enable Upward Avoid FAILED");
          }

          get_avoid_enable_client.call(getAvoidEnable);
          if (getAvoidEnable.response.result)
          {
            ROS_INFO("get horizon avoid enable status:%d, get upwards avoid enable status:%d",
                     getAvoidEnable.response.horizon_avoid_enable_status,
                     getAvoidEnable.response.upwards_avoid_enable_status);
          }

          ROS_INFO_STREAM("Go home...");

          control_task.request.task = FlightTaskControl::Request::TASK_GOHOME_AND_CONFIRM_LANDING;
          task_control_client.call(control_task);
          if(control_task.response.result == false)
          {
            ROS_ERROR_STREAM("GO HOME FAILED");
          }
          break;
        }
      }
    case 'd':
      {
        control_task.request.task = FlightTaskControl::Request::TASK_TAKEOFF;
        ROS_INFO_STREAM("Takeoff request sending ...");
        task_control_client.call(control_task);
        if(control_task.response.result == false)
        {
          ROS_ERROR_STREAM("Takeoff task failed");
          break;
        }

        if(control_task.response.result == true)
        {
          ROS_INFO_STREAM("Takeoff task successful");
          ros::Duration(2).sleep();

          velocityAndYawRateCtrl( {0, 0, 5.0, 0}, 2000);
          ROS_INFO_STREAM("Step 1 over!EmergencyBrake for 2s\n");
          emergency_brake_client.call(emergency_brake);
          ros::Duration(2).sleep();
          velocityAndYawRateCtrl({-1.5, 2, 0, 0}, 2000);
          ROS_INFO_STREAM("Step 2 over!EmergencyBrake for 2s\n");
          emergency_brake_client.call(emergency_brake);
          ros::Duration(2).sleep();
          velocityAndYawRateCtrl({3, 0, 0, 0}, 2500);
          ROS_INFO_STREAM("Step 3 over!EmergencyBrake for 2s\n");
          emergency_brake_client.call(emergency_brake);
          ros::Duration(2).sleep();
          velocityAndYawRateCtrl({-1.6, -2, 0, 0}, 2200);
          ROS_INFO_STREAM("Step 4 over!EmergencyBrake for 2s\n");
          emergency_brake_client.call(emergency_brake);
          ros::Duration(2).sleep();

          control_task.request.task = FlightTaskControl::Request::TASK_LAND;
          ROS_INFO_STREAM("Landing request sending ...");
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
    default:
      break;
  }

  ROS_INFO_STREAM("Finished. Press CTRL-C to terminate the node");

  ros::spin();
  return 0;
}


bool moveByPosOffset(FlightTaskControl& task,const JoystickCommand &offsetDesired,
                    float posThresholdInM,
                    float yawThresholdInDeg)
{
  task.request.task = FlightTaskControl::Request::TASK_POSITION_AND_YAW_CONTROL;
  task.request.joystickCommand.x = offsetDesired.x;
  task.request.joystickCommand.y = offsetDesired.y;
  task.request.joystickCommand.z = offsetDesired.z;
  task.request.joystickCommand.yaw = offsetDesired.yaw;
  task.request.posThresholdInM   = posThresholdInM;
  task.request.yawThresholdInDeg = yawThresholdInDeg;

  task_control_client.call(task);
  return task.response.result;
}

void velocityAndYawRateCtrl(const JoystickCommand &offsetDesired, uint32_t timeMs)
{
  double originTime  = 0;
  double currentTime = 0;
  uint64_t elapsedTimeInMs = 0;
  
  SetJoystickMode joystickMode;
  JoystickAction joystickAction;

  joystickMode.request.horizontal_mode = joystickMode.request.HORIZONTAL_VELOCITY;
  joystickMode.request.vertical_mode = joystickMode.request.VERTICAL_VELOCITY;
  joystickMode.request.yaw_mode = joystickMode.request.YAW_RATE;
  joystickMode.request.horizontal_coordinate = joystickMode.request.HORIZONTAL_GROUND;
  joystickMode.request.stable_mode = joystickMode.request.STABLE_ENABLE;
  set_joystick_mode_client.call(joystickMode);

  joystickAction.request.joystickCommand.x = offsetDesired.x;
  joystickAction.request.joystickCommand.y = offsetDesired.y;
  joystickAction.request.joystickCommand.z = offsetDesired.z;
  joystickAction.request.joystickCommand.yaw = offsetDesired.yaw;

  originTime  = ros::Time::now().toSec();
  currentTime = originTime;
  elapsedTimeInMs = (currentTime - originTime)*1000;

  while(elapsedTimeInMs <= timeMs)
  {
    currentTime = ros::Time::now().toSec();
    elapsedTimeInMs = (currentTime - originTime) * 1000;
    joystick_action_client.call(joystickAction);
  }
}
