#include <iostream>
#include <cstdio>
#include <cstring>
#include <cmath>
#include <dji_sdk/dji_sdk_node.h>

int main(int argc, char **argv) {
	ros::init(argc, argv, "dji_sdk");

	DJISDKNode* dji_sdk_node = new DJISDKNode();
	dji_sdk_node->run();
	
	ros::waitForShutdown();

	return 0;
}
