/** @file demo_time_sync.h
 *  @version 3.8.1
 *  @date May, 2019
 *
 *  @brief
 *  demo sample of how to use time sync APIs
 *
 *  @copyright 2019 DJI. All rights reserved.
 *
 */

#ifndef DEMO_TIME_SYNC_H
#define DEMO_TIME_SYNC_H

// System includes
#include "unistd.h"
#include <cstdint>
#include <iostream>

// DJI SDK includes
#include <dji_sdk/FCTimeInUTC.h>
#include <dji_sdk/GPSUTC.h>

// ROS includes
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <nmea_msgs/Sentence.h>

void NMEADataCallback(const nmea_msgs::Sentence::ConstPtr& msg);

void GPSUTCTimeDataCallback(const dji_sdk::GPSUTC::ConstPtr& msg);

void FCUTCTimeDataCallback(const dji_sdk::FCTimeInUTC::ConstPtr& msg);

void PPSSourceDataCallback(const std_msgs::String::ConstPtr& msg);

#endif //DEMO_TIME_SYNC_H
