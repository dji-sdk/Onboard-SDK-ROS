/****************************************************************************
 * @Brief   A bringup node for dji2mav. Using dji2mav interface v1.x
 * @Version 1.1
 * @Author  Chris Liu
 * @Create  2015/11/02
 * @Modify  2015/11/19
 ****************************************************************************/

#include <pthread.h>
#include <string>

#include <ros/ros.h>
#include <dji_sdk/dji_drone.h>
#include <dji_sdk/LocalPosition.h>
#include <dji_sdk/Velocity.h>
#include <dji_sdk/AttitudeQuaternion.h>

#include "dji2mav/config.h"
#include "dji2mav/modules/heartbeat/mavHeartbeat.h"

DJIDrone* drone;

/* A thread for sending heartbeat */
void* sendHB_Period(void* args) {
    int t_s = *((int*) args);
    while( ros::ok() ) {
        dji2mav::MavHeartbeat::getInstance()->sendHeartbeat();
        sleep(t_s);
    }
}

void locPosCB(const dji_sdk::LocalPosition &msg) {
    dji2mav::MavSensors::getInstance()->setLocalPosition(&msg.ts, &msg.x, 
            &msg.y, &msg.z);
}

void velCB(const dji_sdk::Velocity &msg) {
    dji2mav::MavSensors::getInstance()->setVelocity(&msg.ts, &msg.vx, 
            &msg.vy, &msg.vz);
}

void attCB(const dji_sdk::AttitudeQuaternion &msg) {
    dji2mav::MavSensors::getInstance()->setAttitudeQuaternion(&msg.ts, 
            &msg.q0, &msg.q1, &msg.q2, &msg.q3, &msg.wx, &msg.wy, &msg.wz);
}

void respondToHeartbeat() {
    printf("\n");
}

void respondToMissionRequestList() {
    ROS_INFO("Mission request list get\n");
}

void respondToMissionRequest(uint16_t param) {
    ROS_INFO("--- Finally! Mission Request get %d! ---", param);
}

void respondToMissionAck() {
    ROS_INFO("Mission ack get\n");
}

void respondToMissionCount(uint16_t param) {
    ROS_INFO("*** Get mission count %d! ***", param);
}

void respondToMissionItem(uint16_t param) {
    ROS_INFO("+++ Get mission item %d +++", param);
}


void respondToMissionClearAll() {
    ROS_INFO("Get mission clear all");
}

void respondToMissionSetCurrent(uint16_t param) {
    ROS_INFO("Get mission set current %u", param);
}

void respondToTarget(const float mission[][3], uint16_t beginIdx, uint16_t endIdx) {
    dji_sdk::WaypointList wpl;
    dji_sdk::Waypoint wp;
    for(int i = beginIdx; i < endIdx; ++i) {
        // WARNNING: convert from float to double! QGround Station use float
        wp.latitude = mission[i][0];
        wp.longitude = mission[i][1];
        wp.altitude = mission[i][2];
        wpl.waypoint_list.push_back(wp);
    }
    ROS_INFO("Size of the wpl: %d", wpl.waypoint_list.size());
    ROS_INFO("Going to wait for server...");
    drone->waypoint_navigation_wait_server();
    drone->waypoint_navigation_send_request(wpl);
    ROS_INFO("Going to sleep 10 seconds");
    ros::Duration(10).sleep();
    ROS_INFO("Finish sleeping!");
    if(drone->waypoint_navigation_wait_for_result()) {
        ROS_INFO("Succeed to execute current task!");
    } else {
        ROS_INFO("Fail to execute current task in 10 seconds!");
    }
}

int main(int argc, char* argv[]) {

    ros::init(argc, argv, "dji2mav_bringup");
    ros::NodeHandle nh;
    DJIDrone drone_(nh);
    drone = &drone_;
    drone->request_sdk_permission_control();


    dji2mav::Config* config = dji2mav::Config::getInstance();
    /* set the sysid "1" and the number of GCS is also "1" */
    config->setup(1, 1);
    /* The index of first GCS is "0" */
    config->start(0, "10.60.23.185", 14550, 14551);

    /* Register Subscribers */
    ros::Subscriber sub1 = nh.subscribe("/dji_sdk/local_position", 1, locPosCB);
    ros::Subscriber sub2 = nh.subscribe("/dji_sdk/velocity", 1, velCB);
    ros::Subscriber sub3 = nh.subscribe("/dji_sdk/attitude_quaternion", 1, attCB);

    /* Heartbeat send thread */
    pthread_t tid;
    int t_s = 1;
    int thread_ret = pthread_create(&tid, NULL, sendHB_Period, (void*)&t_s);
    if(0 != thread_ret) {
        ROS_ERROR("Create pthread for sending heartbeat fail! Error code: %d", thread_ret);
    }

    /* Register rsp */
    dji2mav::MavHeartbeat::getInstance()->setHeartbeatRsp(respondToHeartbeat);
    dji2mav::MavWaypoint::getInstance()->setMissionRequestListRsp(respondToMissionRequestList);
    dji2mav::MavWaypoint::getInstance()->setMissionRequestRsp(respondToMissionRequest);
    dji2mav::MavWaypoint::getInstance()->setMissionAckRsp(respondToMissionAck);
    dji2mav::MavWaypoint::getInstance()->setMissionCountRsp(respondToMissionCount);
    dji2mav::MavWaypoint::getInstance()->setMissionItemRsp(respondToMissionItem);
    dji2mav::MavWaypoint::getInstance()->setMissionClearAllRsp(respondToMissionClearAll);
    dji2mav::MavWaypoint::getInstance()->setMissionSetCurrentRsp(respondToMissionSetCurrent);
    dji2mav::MavWaypoint::getInstance()->setTargetRsp(respondToTarget);

    while( ros::ok() ) {
        dji2mav::MavSensors::getInstance()->sendSensorsData();
        dji2mav::MavDistributor::getInstance()->distribute();

        ros::Duration(0.1).sleep();
        ros::spinOnce();
    }

    config->distructor();

    return 0;
}

