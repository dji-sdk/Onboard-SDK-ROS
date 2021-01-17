/** @file mobile_device_node.h
 *  @version 4.0
 *  @date May 2020
 *
 *  @brief sample node of mobile device.
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
#ifndef SRC_MOBILE_DEVICE_NODE_H
#define SRC_MOBILE_DEVICE_NODE_H

#include <ros/ros.h>
#include <dji_osdk_ros/common_type.h>
#include <dji_osdk_ros/dji_vehicle_node.h>

using namespace dji_osdk_ros;

#pragma pack(1)
typedef struct AckReturnToMobile{
    uint16_t cmdID;
    bool     ackResult;
} AckReturnToMobile;
#pragma pack()

/*! service */
ros::ServiceClient send_to_mobile_data_client;
ros::ServiceClient flight_control_client;
ros::ServiceClient gimbal_control_client;
ros::ServiceClient camera_start_shoot_single_photo_client;
ros::ServiceClient camera_record_video_action_client;
ros::ServiceClient obtain_ctrl_authority_client;

/*! Subscriber */
ros::Subscriber fromMobileDataSub;

/*! function */
static void DisplayMainMenu(void);
bool sendToMobile(AckReturnToMobile returnAckMobile);
void fromMobileDataSubCallback(const dji_osdk_ros::MobileData::ConstPtr& fromMobileData);

/*! action */
bool takeoff();
bool land();
bool gohome();
bool gohomeAndConfirmLanding();
bool moveByPosOffset(MoveOffset&& move_offset);
bool localPositionCtrl();
bool resetGimbal();
bool Rotategimbal();
bool takePicture();
bool recordVideo();
bool obtainJoystickControlAuthority();

#endif //SRC_MOBILE_DEVICE_NODE_H
