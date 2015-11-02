#include <ros/ros.h>

#include "dji2mav/mavHandler.h"


int main(int argc, char* argv[]) {

    ros::init(argc, argv, "dji2mav_bringup");
    ros::NodeHandle nh("~");

    dji2mav::MavHandler* mavHandler = dji2mav::MavHandler::getInstance();
    mavHandler->setupMav(1);
    mavHandler->establish("10.60.23.122", 14550, 14551);

    while(ros::ok()) {
        mavHandler->sendHB_Once();
        ros::Durantion(1.0).sleep();

        mavlink_message_t recvMsg;
        mavlink_status_t recvStatus;
        if( mavHandler->receive(recvMsg, recvStatus) ) {
            ROS_STREAM(recvMsg << "\n" << recvStatus);
        };
    }

    return 0;
}

