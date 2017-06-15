/** @file main.cpp
 *  @version 3.3
 *  @date May, 2017
 *
 *  @brief
 *  DJISDKNode
 *
 *  @copyright 2017 DJI. All rights reserved.
 *
 */

#include <dji_sdk/dji_sdk_node.h>

int main(int argc, char **argv) {
  ros::init(argc, argv, "dji_sdk");
  ros::NodeHandle nh;
  ros::NodeHandle nh_private("~");

  DJISDKNode* dji_sdk_node = new DJISDKNode(nh, nh_private);

  ros::AsyncSpinner spinner(4); // Use 4 threads
  spinner.start();
  ros::waitForShutdown();

  delete dji_sdk_node;
  dji_sdk_node = NULL;

  return 0;
}
