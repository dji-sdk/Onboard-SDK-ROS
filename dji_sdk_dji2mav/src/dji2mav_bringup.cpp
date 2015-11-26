/****************************************************************************
 * @Brief   A bringup node for dji2mav. Using dji2mav interface v1.x
 * @Version 0.2.1
 * @Author  Chris Liu
 * @Create  2015/11/02
 * @Modify  2015/11/24
 ****************************************************************************/

#include <pthread.h>
#include <string>

#include <ros/ros.h>
#include <dji_sdk/dji_drone.h>
#include <dji_sdk/LocalPosition.h>
#include <dji_sdk/Velocity.h>
#include <dji_sdk/AttitudeQuaternion.h>

#include "dji_sdk_dji2mav/config.h"



DJIDrone* drone;



/* A thread for sending heartbeat */
void* sendHb_Period(void* args) {
    int t_s = *((int*) args);
    while( ros::ok() ) {
        dji2mav::MavHeartbeat::getInstance()->sendHeartbeat();
        sleep(t_s);
    }
}

/* A thread for sending sensor data */
void* sendSs_Period(void* args) {
    int t_ms = *((int*) args);
    while( ros::ok() ) {
        dji2mav::MavSensors::getInstance()->sendSensorsData();
        usleep(t_ms);
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

void gloPosCB(const dji_sdk::GlobalPosition &msg) {
    dji2mav::MavSensors::getInstance()->setGlobalPosition(&msg.ts, 
            &msg.latitude, &msg.longitude, &msg.altitude, &msg.height);
}



void respondToHeartbeat() {
    ROS_INFO("Get heartbeat\n");
}

void respondToMissionRequestList() {
    ROS_INFO("Get mission request list\n");
}

void respondToMissionRequest(uint16_t param) {
    ROS_INFO("Get mission request %d", param);
}

void respondToMissionAck() {
    ROS_INFO("Mission ack get\n");
}

void respondToMissionCount(uint16_t param) {
    ROS_INFO("Get mission count %d", param);
}

void respondToMissionItem(uint16_t param) {
    ROS_INFO("Get mission item %d", param);
}

void respondToMissionClearAll() {
    ROS_INFO("Get mission clear all");
}

void respondToMissionSetCurrent(uint16_t param) {
    ROS_INFO("Get mission set current %u", param);
}



void respondToTarget(const float mission[][7], uint16_t beginIdx, 
        uint16_t endIdx) {

    dji_sdk::WaypointList wpl;
    dji_sdk::Waypoint wp;
    ROS_INFO("beginIdx %d, endIdx %d", beginIdx, endIdx);
    for(int i = beginIdx; i < endIdx; ++i) {
        wp.latitude = mission[i][4];
        wp.longitude = mission[i][5];
        wp.altitude = mission[i][6];
        wp.staytime = (uint16_t)mission[i][0];
        wp.heading = (int16_t)mission[i][3];
        wpl.waypoint_list.push_back(wp);
    }
    ROS_INFO("Size of the wpl: %d", wpl.waypoint_list.size());

    /**
     * Currently this is executed in main thread, So don't wait for server or 
     * result in case of blocking the distribution process. A new coming 
     * version will bring up a better architecture soon
     */
/*
    ROS_INFO("Going to wait for server...");
    drone->waypoint_navigation_wait_server();
*/

    drone->waypoint_navigation_send_request(wpl);

/*
    ROS_INFO("Going to wait for result");
    if(drone->waypoint_navigation_wait_for_result()) {
        ROS_INFO("Succeed to execute current task!");
    } else {
        ROS_INFO("Fail to execute current task in 10 seconds!");
    }
*/

}



int main(int argc, char* argv[]) {

    ros::init(argc, argv, "dji2mav_bringup");
    ros::NodeHandle nh;
    ros::NodeHandle nh_private("~");
    drone = new DJIDrone(nh);

    std::string targetIp1;
    int targetPort1;
    int srcPort;
    nh_private.param( "targetIp1", targetIp1, std::string("10.60.23.136") );
    nh_private.param("targetPort1", targetPort1, 14550);
    nh_private.param("srcPort", srcPort, 14551);

    drone->request_sdk_permission_control();

    dji2mav::Config* config = dji2mav::Config::getInstance();
    /* set the sysid "1" and the number of GCS is also "1" */
    config->setup(1, 1);
    /* The index of first GCS is "0" */
    config->start(0, targetIp1, (uint16_t)targetPort1, (uint16_t)srcPort);

    /* Register Subscribers */
    ros::Subscriber sub1 = nh.subscribe("/dji_sdk/local_position", 1, locPosCB);
    ros::Subscriber sub2 = nh.subscribe("/dji_sdk/velocity", 1, velCB);
    ros::Subscriber sub3 = nh.subscribe("/dji_sdk/attitude_quaternion", 1, attCB);
    ros::Subscriber sub4 = nh.subscribe("/dji_sdk/global_position", 1, gloPosCB);

    /* Heartbeat send thread */
    pthread_t hbTid;
    int hb_sec = 1;
    int hb_thread_ret = pthread_create(&hbTid, NULL, sendHb_Period, (void*)&hb_sec);
    if(0 != hb_thread_ret) {
        ROS_ERROR("Create pthread for sending heartbeat fail! Error code: %d", hb_thread_ret);
    }

    /* Sensors data send thread */
    pthread_t ssTid;
    int ss_sec = 20000;
    int ss_thread_ret = pthread_create(&ssTid, NULL, sendSs_Period, (void*)&ss_sec);
    if(0 != ss_thread_ret) {
        ROS_ERROR("Create pthread for sending sensors data fail! Error code: %d", ss_thread_ret);
    }

    /* Register responser */
//  dji2mav::MavHeartbeat::getInstance()->setHeartbeatRsp(respondToHeartbeat);
    dji2mav::MavWaypoint::getInstance()->setMissionRequestListRsp(respondToMissionRequestList);
    dji2mav::MavWaypoint::getInstance()->setMissionRequestRsp(respondToMissionRequest);
    dji2mav::MavWaypoint::getInstance()->setMissionAckRsp(respondToMissionAck);
    dji2mav::MavWaypoint::getInstance()->setMissionCountRsp(respondToMissionCount);
    dji2mav::MavWaypoint::getInstance()->setMissionItemRsp(respondToMissionItem);
    dji2mav::MavWaypoint::getInstance()->setMissionClearAllRsp(respondToMissionClearAll);
    dji2mav::MavWaypoint::getInstance()->setMissionSetCurrentRsp(respondToMissionSetCurrent);
    dji2mav::MavWaypoint::getInstance()->setTargetRsp(respondToTarget);

    while( ros::ok() ) {
        /* Do distribution in loop */
        dji2mav::MavDistributor::getInstance()->distribute();

        ros::Duration(0.1).sleep();
        ros::spinOnce();
    }

    ROS_INFO("Going to distruct the whole process...");
    config->distructor();
    delete drone;

    return 0;
}

