/** @file demo_camera_gimbal.h
 *  @version 3.3
 *  @date May, 2017
 *
 *  @brief
 *  demo sample of how to use camera and gimbal APIs
 *
 *  @copyright 2017 DJI. All rights reserved.
 *
 */

#ifndef DEMO_CAMERA_GIMBAL_H
#define DEMO_CAMERA_GIMBAL_H

// System includes
#include "unistd.h"
#include <cstdint>
#include <iostream>

// DJI SDK includes
#include <dji_sdk/Activation.h>
#include <dji_sdk/CameraAction.h>
#include <dji_sdk/Gimbal.h>

// ROS includes
#include <ros/ros.h>
#include <geometry_msgs/Vector3Stamped.h>

#define C_PI (double)3.141592653589793
#define DEG2RAD(DEG) ((DEG) * ((C_PI) / (180.0)))
#define RAD2DEG(RAD) ((RAD) * (180.0) / (C_PI))

typedef struct ServiceAck{
  bool result;
  int cmd_set;
  int cmd_id;
  unsigned int ack_data;
  ServiceAck(bool res, int set, int id, unsigned int ack):
    result(res), cmd_set(set), cmd_id(id), ack_data(ack) {};
  ServiceAck() {};
}ServiceAck;

// Be precise here
struct RotationAngle{
  float roll;
  float pitch;
  float yaw;
};

struct GimbalContainer{
  int roll = 0;
  int pitch = 0;
  int yaw = 0;
  int duration = 0;
  int isAbsolute = 0;
  bool yaw_cmd_ignore = false;
  bool pitch_cmd_ignore = false;
  bool roll_cmd_ignore = false;
  RotationAngle initialAngle;
  RotationAngle currentAngle;
  GimbalContainer( int roll = 0,
                   int pitch = 0,
                   int yaw = 0,
                   int duration = 0,
                   int isAbsolute = 0,
                   RotationAngle initialAngle = {},
                   RotationAngle currentAngle = {}):
    roll(roll), pitch(pitch), yaw(yaw),
    duration(duration),isAbsolute(isAbsolute),
    initialAngle(initialAngle), currentAngle(currentAngle){}
};

struct ResultContainer{
  int angle[3] = {0};
  int error[3] = {0};
};

void doSetGimbalAngle(GimbalContainer *gimbal);

bool gimbalCameraControl();

void displayResult(RotationAngle *currentAngle);

ServiceAck activate();

bool takePicture();

bool startVideo();

bool stopVideo();

void gimbalAngleCallback(const geometry_msgs::Vector3Stamped::ConstPtr& msg);

#endif //DEMO_CAMERA_GIMBAL_H
