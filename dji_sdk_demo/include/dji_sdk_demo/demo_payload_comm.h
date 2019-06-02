/** @file demo_mobile_comm.h
 *  @version 3.3
 *  @date May, 2017
 *
 *  @brief
 *  demo sample of how to use mobile communication APIs
 *
 *  @copyright 2017 DJI. All rights reserved.
 *
 */

#ifndef DEMO_PAYLOAD_COMM_H
#define DEMO_PAYLOAD_COMM_H

// msgs
#include <dji_sdk/PayloadData.h>
#include <dji_sdk/SendPayloadData.h>

// ROS includes
#include <ros/ros.h>

// SDK library
#include <djiosdk/dji_vehicle.hpp>

void fromPayloadDataSubscriberCallback(const dji_sdk::PayloadData::ConstPtr& from_mobile_data);

bool sendToPayload(dji_sdk::SendPayloadData &payload_data);

#endif //DEMO_PAYLOAD_COMM_H
