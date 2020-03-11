/**
  * @Copyright (C) 2020 All rights reserved.
  * @date: 2020
  * @file: mfio_control_node.cc
  * @version: v0.0.1
  * @author: kevin.hoo@dji.com
  * @create_date: 2020-03-04 23:56:34
  * @last_modified_date: 2020-03-11 23:51:38
  * @brief: TODO
  * @details: TODO
  */

#include <ros/ros.h>
#include <dji_osdk_ros/MFIO.h>

using namespace dji_osdk_ros;

void start_info();
int main(int argc, char** argv)
{
  ros::init(argc, argv, "mfio_control_node");
  ros::NodeHandle nh;
  auto mfio_set_client = nh.serviceClient<MFIO>("mfio_control");

  start_info();
  char inputChar;
  std::cin >> inputChar;

  MFIO mfio_settings;
  switch (inputChar)
  {
    case 'a':
    {
      // Turn On PWM Output(Blocking)
      mfio_settings.request.action = MFIO::Request::TURN_ON;
      mfio_settings.request.mode = MFIO::Request::MODE_PWM_OUT;
      mfio_settings.request.block = true;
      mfio_settings.request.channel = MFIO::Request::CHANNEL_3;
      mfio_settings.request.init_on_time_us = 5000; // 25% PWM
      mfio_settings.request.pwm_freq = 50;           // 50 Hz

      ROS_INFO_STREAM("Outputting PWM, last 5 seconds");
      mfio_set_client.call(mfio_settings);
      ros::Duration(5.0).sleep();

      // Turn Off PWM Output
      mfio_settings.request.action = MFIO::Request::TURN_OFF;
      mfio_set_client.call(mfio_settings);
      ROS_INFO_STREAM("Stop PWM output");

      break;
    }
    case 'b':
    {
      // Turn On PWM Output(Non-Blocking)
      mfio_settings.request.action = MFIO::Request::TURN_ON;
      mfio_settings.request.mode = MFIO::Request::MODE_PWM_OUT;
      mfio_settings.request.block = false;
      mfio_settings.request.channel = MFIO::Request::CHANNEL_3;
      mfio_settings.request.init_on_time_us = 5000; // 25% PWM
      mfio_settings.request.pwm_freq = 50;           // 50 Hz

      ROS_INFO_STREAM("Outputting PWM, last 5 seconds");
      mfio_set_client.call(mfio_settings);
      ros::Duration(5.0).sleep();

      // Turn Off PWM Output
      mfio_settings.request.action = MFIO::Request::TURN_OFF;
      mfio_set_client.call(mfio_settings);
      ROS_INFO_STREAM("Stop PWM output");

      break;
    }
    case 'c':
    {
      // Enable GPIO Output(Blocking)
      mfio_settings.request.action = MFIO::Request::TURN_ON;
      mfio_settings.request.mode = MFIO::Request::MODE_GPIO_OUT;
      mfio_settings.request.block = true;
      mfio_settings.request.channel = MFIO::Request::CHANNEL_3;
      mfio_settings.request.gpio_value = 1; // OUTPUT High

      ROS_INFO_STREAM("Set GPIO with 1, last 5 seconds");
      mfio_set_client.call(mfio_settings);
      ros::Duration(5.0).sleep();

      // Turn Off PWM Output
      mfio_settings.request.action = MFIO::Request::TURN_OFF;
      mfio_set_client.call(mfio_settings);
      ROS_INFO_STREAM("Stop GPIO output");

      break;
    }
    case 'd':
    {
      // Enable GPIO Output(Blocking)
      mfio_settings.request.action = MFIO::Request::TURN_ON;
      mfio_settings.request.mode = MFIO::Request::MODE_GPIO_OUT;
      mfio_settings.request.block = false;
      mfio_settings.request.channel = MFIO::Request::CHANNEL_3;
      mfio_settings.request.gpio_value = 1; // OUTPUT High

      ROS_INFO_STREAM("Set GPIO with 1, last 5 seconds");
      mfio_set_client.call(mfio_settings);
      ros::Duration(5.0).sleep();

      // Turn Off PWM Output
      mfio_settings.request.action = MFIO::Request::TURN_OFF;
      mfio_set_client.call(mfio_settings);
      ROS_INFO_STREAM("Stop GPIO output");

      break;
    }
    case 'e':
    {
      // GPIO input(Blocking)
      mfio_settings.request.action = MFIO::Request::TURN_ON;
      mfio_settings.request.mode = MFIO::Request::MODE_GPIO_IN;
      mfio_settings.request.block = true;
      mfio_settings.request.channel = MFIO::Request::CHANNEL_3;

      ROS_INFO_STREAM("Request to read from GPIO");
      mfio_set_client.call(mfio_settings);
      ros::Duration(5.0).sleep();

      // Turn Off PWM Output
      mfio_settings.request.action = MFIO::Request::TURN_OFF;
      mfio_set_client.call(mfio_settings);
      ROS_INFO_STREAM("Input Read Value: " << mfio_settings.response.read_value);

      break;
    }
    case 'f':
    {
      // ADC input(Blocking)
      mfio_settings.request.action = MFIO::Request::TURN_ON;
      mfio_settings.request.mode = MFIO::Request::MODE_ADC;
      mfio_settings.request.block = true;
      mfio_settings.request.channel = MFIO::Request::CHANNEL_3;
      mfio_settings.request.gpio_value = 1; // OUTPUT High

      ROS_INFO_STREAM("Request to read from ADC");
      mfio_set_client.call(mfio_settings);
      ros::Duration(5.0).sleep();

      // Turn Off PWM Output
      mfio_settings.request.action = MFIO::Request::TURN_OFF;
      mfio_set_client.call(mfio_settings);
      ROS_INFO_STREAM("Input Read Value: " << mfio_settings.response.read_value);

      break;
    }
    default:
      break;
  }
  return 0;
}

void start_info()
{
  std::cout
    << "To run the MFIO sample, please first do the following steps:\n"
       "1. Open DJI Assistant 2 and go to the Tools page.\n"
       "2. Click on the Function Channels tab.\n"
       "3. Select SDK4 for channel F3.\n"
       "4. Connect a logic analyzer/oscilloscope to this channel.\n"
       "   The pin diagram, from top to bottom, is Gnd, 5V, Signal.\n"
       "   You will only need to connect to the Gnd and Signal pins.\n\n";
  std::cout << "All loopback demos use SDK4 as output and SDK5 as input \n\n";
  std::cout << "ADC demo uses SDK5 as input \n\n";

  // Display interactive prompt
  std::cout
    << "| Available commands:                                            |"
    << std::endl;
  std::cout
    << "| [a] Set PWM output (block api)                                 |"
    << std::endl;
  std::cout
    << "| [b] Set PWM output (non-block api)                             |"
    << std::endl;
  std::cout
    << "| [c] Set GPIO loopback (blocking)                               |"
    << std::endl;
  std::cout
    << "| [d] Set GPIO loopback (non-blocking)                           |"
    << std::endl;
  std::cout
    << "| [e] Read From GPIO (blocking)                                         |"
    << std::endl;
  std::cout
    << "| [f] Read From ADC (blocking)                                         |"
    << std::endl;
}
