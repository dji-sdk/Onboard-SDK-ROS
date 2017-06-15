/** @file demo_mfio.cpp
 *  @version 3.3
 *  @date May, 2017
 *
 *  @brief
 *  demo sample of how to use MFIO APIs
 *
 *  @copyright 2017 DJI. All rights reserved.
 *
 */

#include <dji_sdk_demo/demo_mfio.h>

// global variables
ros::ServiceClient      drone_activation_service;
ros::ServiceClient      mfio_config_service;
ros::ServiceClient      mfio_set_value_service;

int
main(int argc, char** argv)
{
  ros::init(argc, argv, "sdk_demo_mfio");
  ros::NodeHandle nh;

  // ROS stuff
  drone_activation_service  = nh.serviceClient<dji_sdk::Activation>
    ("dji_sdk/activation");
  mfio_config_service       = nh.serviceClient<dji_sdk::MFIOConfig>
    ("dji_sdk/mfio_config");
  mfio_set_value_service    = nh.serviceClient<dji_sdk::MFIOSetValue>
    ("dji_sdk/mfio_set_value");

  // Activate
  if (activate().result) {
    ROS_INFO("Activated successfully");
  } else {
    ROS_WARN("Failed activation");
    return -1;
  }

  ROS_INFO("To run the MFIO sample, please first do the following steps:");
  ROS_INFO("1. Open DJI Assistant 2 and go to the Tools page.");
  ROS_INFO("2. Click on the Function Channels tab.");
  ROS_INFO("3. Select SDK1 for channel F3.");
  ROS_INFO("4. Connect a logic analyzer/oscilloscope to this channel.");
  ROS_INFO("   The pin diagram, from top to bottom, is Gnd, 5V, Signal.");
  ROS_INFO("   You will only need to connect to the Gnd and Signal pins.");

  // Display interactive prompt
  std::cout
    << "| Available commands:                                            |"
    << std::endl;
  std::cout
    << "| [a] Set PWM values                                             |"
    << std::endl;
  char inputChar;
  std::cin >> inputChar;

  switch (inputChar) {
    case 'a':
      pwmDemo();
      break;
    default:
      break;
  }

  ros::spin();

  return 0;
}

bool pwmDemo() {

  int responseTimeout = 1;

  // Set SDK1 to output PWM at 50Hz
  // Parameters: initialValue - duty cycle
  //             freq - PWM freq
  uint32_t initOnTimeUs = 10000; // us
  uint16_t pwmFreq = 50; //Hz

  // Setup the channel
  ROS_INFO("Configuring channel");
  dji_sdk::MFIOConfig mfio_config;
  mfio_config.request.mode = MODE::MODE_PWM_OUT;
  mfio_config.request.channel = CHANNEL::CHANNEL_0;
  mfio_config.request.init_on_time_us = initOnTimeUs;
  mfio_config.request.pwm_freq = pwmFreq;
  mfio_config_service.call(mfio_config);
  ROS_INFO("Channel configured to output 50Hz PWM with 50 percent duty cycle.");
  sleep(5);

  // After 10s, change the duty cycle from 30%, then 70%.
  ROS_INFO("Setting 30 percent duty cycle");
  initOnTimeUs = 6000; //us, 30 percent duty cycle
  dji_sdk::MFIOSetValue mfio_set_value;
  mfio_set_value.request.channel = CHANNEL::CHANNEL_0;
  mfio_set_value.request.init_on_time_us = initOnTimeUs;
  mfio_set_value_service.call(mfio_set_value);
  sleep(5);

  ROS_INFO("Setting 70 percent duty cycle");
  initOnTimeUs = 14000; //us, 70 percent duty cycle
  mfio_set_value.request.init_on_time_us = initOnTimeUs;
  mfio_set_value_service.call(mfio_set_value);
  sleep(5);

  ROS_INFO("Turning off the PWM signal");
  uint32_t digitalValue = 0;
  uint16_t digitalFreq = 0; //Does not matter for digital I/O
  mfio_config.request.mode = MODE::MODE_GPIO_OUT;
  mfio_config.request.channel = CHANNEL::CHANNEL_0;
  mfio_config.request.init_on_time_us = digitalValue;
  mfio_config.request.pwm_freq = digitalFreq;
  mfio_config_service.call(mfio_config);

  return true;
}

ServiceAck
activate()
{
  dji_sdk::Activation activation;
  drone_activation_service.call(activation);
  if(!activation.response.result) {
    ROS_WARN("ack.info: set = %i id = %i", activation.response.cmd_set, activation.response.cmd_id);
    ROS_WARN("ack.data: %i", activation.response.ack_data);
  }
  return {activation.response.result, activation.response.cmd_set,
          activation.response.cmd_id, activation.response.ack_data};
}
