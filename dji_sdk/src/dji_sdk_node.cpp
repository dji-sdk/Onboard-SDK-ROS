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

//TODO
    rosAdapter = new DJI::onboardSDK::ROSAdapter;

    DJISDKNode* dji_sdk_node = new DJISDKNode(nh, nh_private);

    ros::AsyncSpinner spinner(4); // Use 4 threads
    spinner.start();
    ros::waitForShutdown();

//TODO:
    delete rosAdapter;
    rosAdapter = NULL;

    return 0;
}
