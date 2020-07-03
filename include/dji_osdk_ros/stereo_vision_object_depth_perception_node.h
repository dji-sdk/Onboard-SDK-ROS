/** @file stereo_vision_object_depth_perception_node.h
 *  @version 4.0
 *  @date June 2020
 *
 *  @brief node of stereo_vision_object_depth_perception.
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
#ifndef STEREO_VISION_OBJECT_DEPTH_PERCEPTION_NODE_H
#define STEREO_VISION_OBJECT_DEPTH_PERCEPTION_NODE_H
// System includes
#include "chrono"
#include <signal.h>

// DJI SDK includes
#include <dji_osdk_ros/common_type.h>
#include <dji_osdk_ros/StereoVGASubscription.h>
#include <dji_osdk_ros/GetDroneType.h>
#include <dji_osdk_ros/GetM300StereoParams.h>

// ROS includes
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <visualization_msgs/MarkerArray.h>

// Utility includes
#include "dji_osdk_ros/stereo_utility/stereo_frame.hpp"

// YOLO includes
#include "darknet_ros_msgs/BoundingBoxes.h"

typedef std::chrono::time_point<std::chrono::high_resolution_clock> timer;
typedef std::chrono::duration<float> duration;

void objectDetectionCallback(const darknet_ros_msgs::BoundingBoxesConstPtr &msg,
                             M210_STEREO::StereoFrame::Ptr stereo_frame_ptr);

void displayObjectPtCloudCallback(const sensor_msgs::ImageConstPtr &img_left,
                                  const sensor_msgs::ImageConstPtr &img_right,
                                  const darknet_ros_msgs::BoundingBoxesConstPtr &b_box,
                                  M210_STEREO::StereoFrame::Ptr stereo_frame_ptr);

void visualizeRectImgHelper(M210_STEREO::StereoFrame::Ptr stereo_frame_ptr);

void visualizeDisparityMapHelper(M210_STEREO::StereoFrame::Ptr stereo_frame_ptr);

bool imgSubscriptionHelper(dji_sdk::StereoVGASubscription &service);

void shutDownHandler(int s);


#endif // STEREO_VISION_OBJECT_DEPTH_PERCEPTION_NODE_H