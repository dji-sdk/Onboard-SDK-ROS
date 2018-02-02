#ifndef DEMO_ADVANCED_SENSING_DEPTH_PERCEPTION_H
#define DEMO_ADVANCED_SENSING_DEPTH_PERCEPTION_H

// System includes
#include "chrono"
#include <signal.h>

// DJI SDK includes
#include "dji_sdk/StereoVGASubscription.h"

// ROS includes
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include "sensor_msgs/PointCloud2.h"
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>

// Utility includes
#include "stereo_utility/stereo_frame.hpp"

typedef std::chrono::time_point<std::chrono::high_resolution_clock> timer;
typedef std::chrono::duration<float> duration;


void displayStereoRectImgCallback(const sensor_msgs::ImageConstPtr &img_left,
                                  const sensor_msgs::ImageConstPtr &img_right,
                                  M210_STEREO::StereoFrame::Ptr stereo_frame_ptr);

void displayStereoDisparityCallback(const sensor_msgs::ImageConstPtr &img_left,
                                    const sensor_msgs::ImageConstPtr &img_right,
                                    M210_STEREO::StereoFrame::Ptr stereo_frame_ptr);

void displayStereoFilteredDisparityCallback(const sensor_msgs::ImageConstPtr &img_left,
                                            const sensor_msgs::ImageConstPtr &img_right,
                                            M210_STEREO::StereoFrame::Ptr stereo_frame_ptr);

void displayStereoPtCloudCallback(const sensor_msgs::ImageConstPtr &img_left,
                                  const sensor_msgs::ImageConstPtr &img_right,
                                  M210_STEREO::StereoFrame::Ptr stereo_frame_ptr);

void visualizeRectImgHelper(M210_STEREO::StereoFrame::Ptr stereo_frame_ptr);

void visualizeDisparityMapHelper(M210_STEREO::StereoFrame::Ptr stereo_frame_ptr);

bool imgSubscriptionHelper(dji_sdk::StereoVGASubscription &service);

void shutDownHandler(int s);

#endif //DEMO_ADVANCED_SENSING_DEPTH_PERCEPTION_H
