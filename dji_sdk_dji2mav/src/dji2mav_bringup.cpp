/****************************************************************************
 * @Brief   A bringup node for dji2mav. Using dji2mav interface v1.x
 * @Version 1.1
 * @Author  Chris Liu
 * @Create  2015/11/02
 * @Modify  2015/11/17
 ****************************************************************************/

#include <pthread.h>
#include <string>

#include <ros/ros.h>
#include <dji_sdk/LocalPosition.h>
#include <dji_sdk/Velocity.h>
#include <dji_sdk/AttitudeQuaternion.h>

#include "dji2mav/config.h"
#include "dji2mav/modules/heartbeat/mavHeartbeat.h"

int main(int argc, char* argv[]) {

    ros::init(argc, argv, "dji2mav_bringup");
    ros::NodeHandle nh("~");

    dji2mav::Config* config = dji2mav::Config::getInstance();

/*
    int thread_ret = pthread_create(&tid, NULL, sendHB_Period, (void*)&t_s);
    if(0 != thread_ret) {
        ROS_ERROR("Create pthread for sending heartbeat fail! Error code: %d", thread_ret);
    }
*/

    // set the sysid "1" and the number of GCS is also "1"
    config->setup(1, 1);
    // The index of first GCS is "0"
    config->start(0, "10.60.23.185", 14550, 14551);

    //should get its instance after the config set up!
    dji2mav::MavHeartbeat* mavHb = dji2mav::MavHeartbeat::getInstance();
    dji2mav::MavDistributor* mavDstb = dji2mav::MavDistributor::getInstance();

    while( ros::ok() ) {
        mavHb->sendHeartbeat();
        ros::Duration(1.0).sleep();
        mavDstb->distribute();
    }

    return 0;
}

