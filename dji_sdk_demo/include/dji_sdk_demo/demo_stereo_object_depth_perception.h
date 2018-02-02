#ifndef PROJECT_DEMO_ADVANCED_SENSING_OBJECT_DEPTH_PERCEPTION_H
#define PROJECT_DEMO_ADVANCED_SENSING_OBJECT_DEPTH_PERCEPTION_H

// System includes
#include "chrono"
#include <signal.h>

// DJI SDK includes
#include "dji_sdk/StereoVGASubscription.h"

// ROS includes
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <visualization_msgs/MarkerArray.h>

// Utility includes
#include "stereo_utility/stereo_frame.hpp"

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

#endif //PROJECT_DEMO_ADVANCED_SENSING_OBJECT_DEPTH_PERCEPTION_H
