/****************************************************************************
 * @Brief   A bringup node for dji2mav. Using dji2mav interface v0.3.x
 * @Version 0.3.0
 * @Author  Chris Liu
 * @Create  2015/11/02
 * @Modify  2015/12/25
 ****************************************************************************/

#define DJI2MAV_LOG_INFO
//#define GCS_NUM 2
//#define MAVLINK_COMM_NUM_BUFFERS GCS_NUM

#include <string>

#include <ros/ros.h>
#include <dji_sdk/dji_drone.h>
#include <dji_sdk/LocalPosition.h>
#include <dji_sdk/Velocity.h>
#include <dji_sdk/AttitudeQuaternion.h>

#include "dji_sdk_dji2mav/mavHandler.h"
#include "dji_sdk_dji2mav/mavContainer.h"
#include "dji_sdk_dji2mav/modules/heartbeat/mavHeartbeat.h"
#include "dji_sdk_dji2mav/modules/sensors/mavSensors.h"
#include "dji_sdk_dji2mav/modules/waypoint/mavWaypoint.h"
#include "dji_sdk_dji2mav/modules/hotpoint/mavHotpoint.h"


DJIDrone* drone;
dji2mav::MavSensors* g_sensors;


void locPosCB(const dji_sdk::LocalPosition &msg) {
    g_sensors->setLocalPosition(&msg.ts, &msg.x, &msg.y, &msg.z);
}

void velCB(const dji_sdk::Velocity &msg) {
    g_sensors->setVelocity(&msg.ts, &msg.vx, 
            &msg.vy, &msg.vz);
}

void attCB(const dji_sdk::AttitudeQuaternion &msg) {
    g_sensors->setAttitudeQuaternion(&msg.ts, &msg.q0, &msg.q1, &msg.q2, 
            &msg.q3, &msg.wx, &msg.wy, &msg.wz);
}

void gloPosCB(const dji_sdk::GlobalPosition &msg) {
    g_sensors->setGlobalPosition(&msg.ts, &msg.latitude, &msg.longitude, 
            &msg.altitude, &msg.height);
}



void respondToHeartbeat() {
    ROS_INFO("Get heartbeat");
}

void respondToMissionRequestList() {
    ROS_INFO("Get mission request list");
}

void respondToMissionRequest(uint16_t param) {
    ROS_INFO("Get mission request %d", param);
}

void respondToMissionAck() {
    ROS_INFO("Mission ack get");
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



void respondToWpTarget(const float mission[][7], uint16_t beginIdx, 
        uint16_t endIdx) {

    dji_sdk::MissionWaypointTask task;
    memset(&task, 0, sizeof(dji_sdk::MissionWaypointTask));

    task.mission_exec_times = 0x01;
    task.yaw_mode = 0x03;
    task.velocity_range = 10.0; //must be set
    task.idle_velocity = 3.0; //must be set

    ROS_INFO("beginIdx %d, endIdx %d", beginIdx, endIdx);
    for(int i = beginIdx; i < endIdx; ++i) {
        dji_sdk::MissionWaypoint wp;
        memset(&wp, 0, sizeof(dji_sdk::MissionWaypoint));

        wp.latitude = mission[i][4];
        wp.longitude = mission[i][5];
        wp.altitude = mission[i][6];
        wp.target_yaw = (int16_t)mission[i][3];
        wp.has_action = 0x01; //true or false
        wp.action_time_limit = 0xffff;

        wp.waypoint_action.action_repeat = 0x01; //times!
        wp.waypoint_action.command_list[0] = 0x00;
        wp.waypoint_action.command_parameter[0] = (int16_t) (mission[i][0] * 1000.0);

        task.mission_waypoint.push_back(wp);
    }
    ROS_INFO("Size of the wpl: %d", task.mission_waypoint.size());

    ROS_INFO("Going to wait for Waypoint server...");
    drone->waypoint_navigation_wait_server();
    ROS_INFO("...get response!");

    drone->mission_waypoint_upload(task);
    ros::Duration(1.0).sleep();
    drone->mission_start();
    ROS_INFO("Start waypoint mission.");

/*
    ROS_INFO("Going to wait for Waypoint result...");
    if(drone->waypoint_navigation_wait_for_result()) {
        ROS_INFO("...succeed executing current task!");
    } else {
        ROS_INFO("...fail to execute current task!");
    }
*/

}


void respondToHpTarget(const float hp[], uint16_t size, uint16_t cmd) {
    dji_sdk::MissionHotpointTask task;
    memset(&task, 0, sizeof(dji_sdk::MissionHotpointTask));

    task.latitude = hp[4];
    task.longitude = hp[5];
    task.altitude = hp[6];

    if(hp[2] < 0)
        task.is_clockwise = 0x00;
    else
        task.is_clockwise = 0x01;
    task.radius = abs(hp[2]);

    //task.? = hp[3];//unused

    task.angular_speed = 5;
    task.start_point = 0x04;//frome current position to the nearest point
    task.yaw_mode = 0x01;//point to the center of the circle

    switch(cmd) {
        case 17:
            break;
        case 18:
            //turns
            break;
        case 19:
            //time
            break;
    }

    ROS_INFO("Going to wait for Hotpoint server...");
    drone->waypoint_navigation_wait_server();
    ROS_INFO("...get response!");

    drone->mission_hotpoint_upload(task);
    ros::Duration(1.0).sleep();
    drone->mission_start();
    ROS_INFO("Start hotpoint mission.");

/*
    ROS_INFO("Going to wait for Hotpoint result...");
    if( drone->waypoint_navigation_wait_for_result() )
        ROS_INFO("...succeed executing current task!");
    else
        ROS_INFO("...fail to execute current task!");
*/
}



int main(int argc, char* argv[]) {

    ros::init(argc, argv, "dji2mav_bringup");
    ros::NodeHandle nh;
    ros::NodeHandle nh_private("~");
    drone = new DJIDrone(nh);

    std::string targetIp1, targetIp2;
    int targetPort1, targetPort2;
    int srcPort1, srcPort2;
    nh_private.param( "targetIp1", targetIp1, std::string("10.60.23.136") );
    nh_private.param("targetPort1", targetPort1, 14550);
    nh_private.param( "targetIp2", targetIp2, std::string("10.60.23.136") );
    nh_private.param("targetPort2", targetPort2, 14548);
    nh_private.param("srcPort1", srcPort1, 14551);
    nh_private.param("srcPort2", srcPort2, 14549);

    drone->activate();
    ros::Duration(1.0).sleep();
    drone->request_sdk_permission_control();

    /* set the sysid "1" and the number of GCS is 2 */
    dji2mav::MavHandler handler(1, 2);
    handler.establish(0, targetIp1, (uint16_t)targetPort1, srcPort1);
    handler.establish(1, targetIp2, (uint16_t)targetPort2, srcPort2);

    /* set heartbeat module and employ senders for it on GCS 0 and 1 */
    dji2mav::MavHeartbeat heartbeat(handler, "heartbeat", 2, 0, 1);
    dji2mav::MavSensors sensors(handler, "sensors", 2, 0, 1);
    dji2mav::MavWaypoint waypoint(handler, "waypoint", 1, 0);
    dji2mav::MavHotpoint hotpoint(handler, "hotpoint", 1, 1);

    g_sensors = &sensors;

    /* Register responser */
//  heartbeat.setHeartbeatHook(respondToHeartbeat);

    waypoint.setMissionRequestListHook(respondToMissionRequestList);
    waypoint.setMissionRequestHook(respondToMissionRequest);
    waypoint.setMissionAckHook(respondToMissionAck);
    waypoint.setMissionCountHook(respondToMissionCount);
    waypoint.setMissionItemHook(respondToMissionItem);
    waypoint.setMissionClearAllHook(respondToMissionClearAll);
    waypoint.setMissionSetCurrentHook(respondToMissionSetCurrent);
    waypoint.setTargetHook(respondToWpTarget);

    hotpoint.setTargetHook(respondToHpTarget);

    /* set the module container and register module with its MOI */
    dji2mav::MavContainer container(handler);
    container.registerModule(heartbeat, 1, MAVLINK_MSG_ID_HEARTBEAT);
    container.registerModule(sensors, 0);
    container.registerModule(waypoint, 7, MAVLINK_MSG_ID_MISSION_REQUEST_LIST, 
            MAVLINK_MSG_ID_MISSION_REQUEST, MAVLINK_MSG_ID_MISSION_ACK, 
            MAVLINK_MSG_ID_MISSION_COUNT, MAVLINK_MSG_ID_MISSION_ITEM, 
            MAVLINK_MSG_ID_MISSION_CLEAR_ALL, MAVLINK_MSG_ID_MISSION_SET_CURRENT);
    container.registerModule(hotpoint, 7, MAVLINK_MSG_ID_MISSION_REQUEST_LIST, 
            MAVLINK_MSG_ID_MISSION_REQUEST, MAVLINK_MSG_ID_MISSION_ACK, 
            MAVLINK_MSG_ID_MISSION_COUNT, MAVLINK_MSG_ID_MISSION_ITEM, 
            MAVLINK_MSG_ID_MISSION_CLEAR_ALL, MAVLINK_MSG_ID_MISSION_SET_CURRENT);

    /* Register Subscribers */
    ros::Subscriber sub1 = nh.subscribe("/dji_sdk/local_position", 1, locPosCB);
    ros::Subscriber sub2 = nh.subscribe("/dji_sdk/velocity", 1, velCB);
    ros::Subscriber sub3 = nh.subscribe("/dji_sdk/attitude_quaternion", 1, attCB);
    ros::Subscriber sub4 = nh.subscribe("/dji_sdk/global_position", 1, gloPosCB);

    container.run();

    while( ros::ok() ) {
        ros::Duration(0.02).sleep();
        ros::spinOnce();
    }

    ROS_INFO("Going to destruct the ROS node...");
    drone->release_sdk_permission_control();
    delete drone;
    ROS_INFO("...finish. Exit...");

    return 0;
}

