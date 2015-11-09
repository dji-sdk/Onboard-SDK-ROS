/****************************************************************************
 * @brief   A bringup node for dji2mav. Using MavHandler interface v1.x
 * @version 1.0
 * @Date    2015/11/2
 ****************************************************************************/

#include <pthread.h>
#include <string>

#include <ros/ros.h>
#include <dji_sdk/LocalPosition.h>
#include <dji_sdk/Velocity.h>
#include <dji_sdk/AttitudeQuaternion.h>

#include "dji2mav/mavHandler.h"

dji_sdk::LocalPosition g_locPos;
dji_sdk::Velocity g_vel;
dji_sdk::AttitudeQuaternion g_att;
dji2mav::MavHandler* g_mavHandler;

/* A thread for sending heartbeat */
void* sendHB_Period(void* args) {
    int t_s = *((int*) args);
    while( ros::ok() ) {
        g_mavHandler->sendHB();
        sleep(t_s);
    }
}

void locPosCB(const dji_sdk::LocalPosition &msg) {
    g_locPos.ts = msg.ts;
    g_locPos.x = msg.x;
    g_locPos.y = msg.y;
    g_locPos.z = msg.z;
}

void velCB(const dji_sdk::Velocity &msg) {
    g_vel.ts = msg.ts;
    g_vel.vx = msg.vx;
    g_vel.vy = msg.vy;
    g_vel.vz = msg.vz;
}

void attCB(const dji_sdk::AttitudeQuaternion &msg) {
    g_att.ts = msg.ts;
    g_att.q0 = msg.q0;
    g_att.q1 = msg.q1;
    g_att.q2 = msg.q2;
    g_att.q3 = msg.q3;
    g_att.wx = msg.wx;
    g_att.wy = msg.wy;
    g_att.wz = msg.wz;
}

int main(int argc, char* argv[]) {

    ros::init(argc, argv, "dji2mav_bringup");
    ros::NodeHandle nh("~");

    /* Setup MavHandler */
    g_mavHandler = dji2mav::MavHandler::getInstance();
    g_mavHandler->setupMav(1);
    g_mavHandler->establish("10.60.23.186", 14550, 14551);

    /* Register Subscribers */
    ros::Subscriber sub1 = nh.subscribe("/dji_sdk/local_position", 1, locPosCB);
    ros::Subscriber sub2 = nh.subscribe("/dji_sdk/velocity", 1, velCB);
    ros::Subscriber sub3 = nh.subscribe("/dji_sdk/attitude_quaternion", 1, attCB);

    /* Send heartbeat once a second */
    //g_mavHandler->sendHB();
    pthread_t tid;
    int t_s = 1;
    int thread_ret = pthread_create(&tid, NULL, sendHB_Period, (void*)&t_s);
    if(0 != thread_ret) {
        ROS_ERROR("Create pthread for sending heartbeat fail! Error code: %d", thread_ret);
    }

    while(ros::ok()) {
        /* Send local position */
        g_mavHandler->updateLocPos(std::min(g_locPos.ts, g_vel.ts), 
                g_locPos.x, g_locPos.y, g_locPos.z, g_vel.vx, g_vel.vy, 
                g_vel.vz);
        g_mavHandler->sendLocPos();

        /* Send attitude, convert quaternion to eular */
        g_mavHandler->updateAtt(
                g_att.ts, 
                atan2( 2*(g_att.q0*g_att.q1+g_att.q2*g_att.q3), 
                        1-2*(g_att.q1*g_att.q1+g_att.q2*g_att.q2) ), 
                asin( 2*(g_att.q0*g_att.q2-g_att.q3*g_att.q1) ), 
                atan2( 2*(g_att.q0*g_att.q3+g_att.q1*g_att.q2), 
                        1-2*(g_att.q2*g_att.q2+g_att.q3*g_att.q3) ), 
                g_att.wx, 
                g_att.wy, 
                g_att.wz  );
        g_mavHandler->sendAtt();

        /* Remember to spin */
        ros::Duration(0.1).sleep();
        ros::spinOnce();

        /* Get received data */
        mavlink_message_t recvMsg;
        mavlink_status_t recvStatus;
        if( g_mavHandler->receive(recvMsg, recvStatus) ) {
            ROS_INFO("Get!");
        };
    }

    return 0;
}

