/** @file demo_mfio.h
 *  @version 3.3
 *  @date May, 2017
 *
 *  @brief
 *  demo sample of how to use MFIO APIs
 *
 *  @copyright 2017 DJI. All rights reserved.
 *
 */

#ifndef DEMO_MFIO_H
#define DEMO_MFIO_H

// System includes
#include "unistd.h"
#include <cstdint>
#include <iostream>

// DJI SDK includes
#include <dji_sdk/Activation.h>
#include <dji_sdk/MFIOConfig.h>
#include <dji_sdk/MFIOSetValue.h>

// ROS includes
#include <ros/ros.h>

typedef enum MODE {
  MODE_PWM_OUT  = 0,
  MODE_PWM_IN   = 1,
  MODE_GPIO_OUT = 2,
  MODE_GPIO_IN  = 3,
  MODE_ADC      = 4
} MODE;

typedef enum CHANNEL {
  CHANNEL_0 = 0,
  CHANNEL_1 = 1,
  CHANNEL_2 = 2,
  CHANNEL_3 = 3,
  CHANNEL_4 = 4,
  CHANNEL_5 = 5,
  CHANNEL_6 = 6,
  CHANNEL_7 = 7,
} CHANNEL;

typedef struct ServiceAck
{
  bool         result;
  int          cmd_set;
  int          cmd_id;
  unsigned int ack_data;
  ServiceAck(bool res, int set, int id, unsigned int ack)
    : result(res)
    , cmd_set(set)
    , cmd_id(id)
    , ack_data(ack)
  {
  }
  ServiceAck()
  {
  }
} ServiceAck;

bool pwmDemo();

ServiceAck activate();

#endif // DEMO_MFIO_H
