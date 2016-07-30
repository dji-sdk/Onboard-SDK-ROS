/** @file dji_sdk_node.cpp
 *  @version 3.1.8
 *  @date July 29th, 2016
 *
 *  @brief
 *  Main function for ROS Node
 *
 *  @copyright 2016 DJI. All rights reserved.
 *
 */

#include <iostream>
#include <cstdio>
#include <cstring>
#include <cmath>
#include <dji_sdk/dji_sdk_node.h>

DJI::onboardSDK::ROSAdapter *rosAdapter;

int main(int argc, char **argv) {
    ros::init(argc, argv, "dji_sdk");
    ros::NodeHandle nh;
    ros::NodeHandle nh_private("~");

    //new an object of adapter
    rosAdapter = new DJI::onboardSDK::ROSAdapter;

    DJISDKNode* dji_sdk_node = new DJISDKNode(nh, nh_private);

    ros::AsyncSpinner spinner(4); // Use 4 threads
    spinner.start();
    ros::waitForShutdown();

    //clear
    delete rosAdapter;
    rosAdapter = NULL;
    delete dji_sdk_node;
    dji_sdk_node = NULL;

    return 0;
}
