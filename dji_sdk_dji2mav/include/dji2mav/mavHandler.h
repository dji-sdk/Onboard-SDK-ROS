/*****************************************************************************
 * @Brief     Handle mavlink msg packing and unpacking. ROS-free singleton
 * @Version   1.0
 * @Author    Chris Liu
 * @Created   2015/10/31
 * @Modified  2015/11/04
 *****************************************************************************/

#ifndef _DJI2MAV_MAVHANDLER_H_
#define _DJI2MAV_MAVHANDLER_H_


#include "communicator.h"
#include "msgManager.h"
#include "mavResponser.h"

#include <mavlink/v1.0/common/mavlink.h>
#include <stdio.h>
#include <iostream>
#include <new>
#include <string>

#define DEFAULT_SENDER_LIST_SIZE 256

namespace dji2mav {

    class MavHandler {
        public:
            MavHandler() {
                //TODO: Better architecture?
                //-2 for having not registered yet
                m_hbIdx = -2;
                m_statusIdx = -2;
                m_locPosIdx = -2;
                m_attIdx = -2;
                m_wpIdx = -2;

                //TODO: temporary add sender for waypoint
                m_tempWpIdx = m_mng->registerSender();
                if(-1 == m_tempWpIdx) {
                    printf("Registering a temp waypoint sender fail!");
                }
            }


            ~MavHandler() {
            }


            /**
             * @brief  Set configurations for this mavlink device
             * @param  sysid        : ID of this mavlink system
             * @param  compid       : ID of this component
             * @param  mavType      : Default quadrotor type
             * @param  mavAutopilot : Default full supporting for every
             * @param  mavMode      : Default autocontrol and disarmed
             * @param  customMode   : Default 0. Defined by user
             * @param  mavStatus    : Default vehicle grounded and standby
             * @param  mavVersion   : Default 3
             */
            //TODO: MAV_COMP_ID_IMU?
            void setupMav(uint8_t sysid, uint8_t compid = MAV_COMP_ID_IMU, 
                    uint8_t mavType = MAV_TYPE_QUADROTOR, 
                    uint8_t mavAutopilot = MAV_AUTOPILOT_GENERIC, 
                    uint8_t mavMode = MAV_MODE_GUIDED_DISARMED, 
                    uint32_t customMode = 0, 
                    uint8_t mavStatus = MAV_STATE_STANDBY, 
                    uint8_t mavVersion = 3) {

                m_mavSys.sysid = sysid;
                m_mavSys.compid = compid;

                m_mavHB.custom_mode = customMode;
                m_mavHB.type = mavType;
                m_mavHB.autopilot = mavAutopilot;
                m_mavHB.base_mode = mavMode;
                m_mavHB.system_status = mavStatus;
                m_mavHB.mavlink_version = mavVersion;

            }


            /**
             * @brief  Establish connection to the GCS
             * @param  gcsIP          : The IP of GCS
             * @param  gcsPort        : Connection port of GCS
             * @param  locPort        : Localhost port
             * @param  senderListSize : Default 256
             * @param  recvBufSize    : Default 4096
             * @return True if succeed and false if fail
             */
            bool establish(std::string gcsIP, int gcsPort, int locPort, 
                    uint16_t senderListSize = DEFAULT_SENDER_LIST_SIZE, 
                    uint16_t recvBufSize = DEFAULT_RECV_BUF_SIZE) {

                m_comm = Communicator::getInstance();
                m_comm->setConf(gcsIP, gcsPort, locPort);
                if( m_comm->connect() == false ) {
                    printf("Connecting to GCS fail!");
                    return false;
                }

                m_mng = MsgManager::getInstance(senderListSize, recvBufSize);
                if(NULL == m_mng) {
                    printf("Get MsgManager instance fail!\n");
                    return false;
                }

                printf("Establishing Connection Succeed!\n");
                return true;

            }


            /**
             * @brief  Designed for send encoded package
             * @param  idx   : The index of sender
             * @param  msg_p : The pointer of msg that should be sent
             * @return True if succeed and false if fail
             */
            inline bool sendEncodedMsg(int idx, 
                    const mavlink_message_t *msg_p) {
                uint16_t len = mavlink_msg_to_send_buffer(
                        m_mng->getSendBuf(idx), msg_p);
                int bytes_sent = m_mng->send(idx, len);
                if(bytes_sent < 0)
                    return false;
                if( (uint16_t)bytes_sent != len ) {
                    std::cout << bytes_sent << " bytes were sent. But " 
                            << len << "-byte package was generated!" 
                            << std::endl;
                    return false;
                }
                return true;
            }


            /**
             * @brief  Send heartbeat once
             * @return True if succeed and false if fail
             */
            bool sendHB() {
                if(-2 == m_hbIdx) {
                    m_hbIdx = m_mng->registerSender();
                }
                if(-1 == m_hbIdx) {
                    printf("Registering a heartbeat sender fail!");
                    return false;
                }

                mavlink_message_t msg;
                mavlink_msg_heartbeat_encode(m_mavSys.sysid, m_mavSys.compid, 
                        &msg, &m_mavHB);

                if( sendEncodedMsg(m_hbIdx, &msg) ) {
                    return true;
                } else {
                    printf("Sending heartbeat pack fail!");
                    return false;
                }
            }


            /**
             * @brief  Send status
             * @return True if succeed and false if fail
             */
            bool sendStatus() {
                if(-2 == m_statusIdx) {
                    m_statusIdx = m_mng->registerSender();
                }
                if(-1 == m_statusIdx) {
                    printf("Registering a status sender fail!");
                    return false;
                }

                mavlink_message_t msg;
                mavlink_msg_sys_status_encode(m_mavSys.sysid, 
                        m_mavSys.compid, &msg, &m_sysStatus);

                if( sendEncodedMsg(m_statusIdx, &msg) ) {
                    return true;
                } else {
                    printf("Sending status pack fail!");
                    return false;
                }
            }


            /**
             * @brief  Send local position
             * @return True if succeed and false if fail
             */
            bool sendLocPos() {
                if(-2 == m_locPosIdx) {
                    m_locPosIdx = m_mng->registerSender();
                }
                if(-1 == m_locPosIdx) {
                    printf("Registering a local position sender fail!");
                    return false;
                }

                mavlink_message_t msg;
                mavlink_msg_local_position_ned_encode(m_mavSys.sysid, 
                        m_mavSys.compid, &msg, &m_locPos);

                if( sendEncodedMsg(m_locPosIdx, &msg) ) {
                    return true;
                } else {
                    printf("Sending local position pack fail!");
                    return false;
                }
            }


            /**
             * @brief  Send attitude
             * @return True if succeed and false if fail
             */
            bool sendAtt() {
                if(-2 == m_attIdx) {
                    m_attIdx = m_mng->registerSender();
                }
                if(-1 == m_attIdx) {
                    printf("Registering a attitude sender fail!");
                    return false;
                }

                mavlink_message_t msg;
                mavlink_msg_attitude_encode(m_mavSys.sysid, 
                        m_mavSys.compid, &msg, &m_att);

                if( sendEncodedMsg(m_attIdx, &msg) ) {
                    return true;
                } else {
                    printf("Sending attitude pack fail!");
                    return false;
                }
            }


            /**
             * @brief  Receive data and parse it from buffer to mavlink
             * @return True if succeed and false if fail
             */
            bool receive(mavlink_message_t &recvMsg, 
                    mavlink_status_t &recvStatus) {

                bool ret = false;
                int bytes_recv = m_mng->recv();
                if(bytes_recv < 0 && errno != EAGAIN) {
                    printf("Execution of receive process fail!");
                    return false;
                }
                if(bytes_recv > 0) {
                    printf("\nDatagram: "); //TODO: temporary
                    for(int i = 0; i < bytes_recv; ++i) {
                        uint8_t tmp = ( m_mng->getRecvBuf() )[i]; //TODO: temporary
                        printf("%02x", tmp); //TODO: temporary
                        if(mavlink_parse_char(MAVLINK_COMM_0, 
                                ( m_mng->getRecvBuf() )[i], 
                                &recvMsg, &recvStatus)) {

                            ret = true;
                            //m_rsp->decode(recvMsg);
                            ////printf("\nReceived packet: SYS: %d, COMP: %d, LEN: %d, MSG ID: %d\n", recvMsg.sysid, recvMsg.compid, recvMsg.len, recvMsg.msgid); //TODO: temporary

                        }
                    }
                    printf("\n"); //TODO: temporary
                }
                return ret;

            }


            /**
             * @brief Refer to mavlink/common/mavlink_msg_heartbeat.h
             */
            void updateHB(uint8_t type, uint8_t autopilot, uint8_t base_mode, 
                    uint32_t custom_mode, uint8_t system_status, 
                    uint8_t mavlink_version) {

                m_mavHB.type = type;
                m_mavHB.autopilot = autopilot;
                m_mavHB.base_mode = base_mode;
                m_mavHB.custom_mode = custom_mode;
                m_mavHB.system_status = system_status;
                m_mavHB.mavlink_version = mavlink_version;

            }


            /**
             * @brief Refer to mavlink/common/mavlink_msg_sys_status.h
             */
            void updateStatus(uint32_t onboard_control_sensors_present, 
                    uint32_t onboard_control_sensors_enabled, 
                    uint32_t onboard_control_sensors_health, 
                    uint16_t load, uint16_t voltage_battery, 
                    int16_t current_battery, int8_t battery_remaining, 
                    uint16_t drop_rate_comm, uint16_t errors_comm, 
                    uint16_t errors_count1, uint16_t errors_count2, 
                    uint16_t errors_count3, uint16_t errors_count4) {

                m_sysStatus.onboard_control_sensors_present;
                m_sysStatus.onboard_control_sensors_enabled;
                m_sysStatus.onboard_control_sensors_health;
                m_sysStatus.load;
                m_sysStatus.voltage_battery;
                m_sysStatus.current_battery;
                m_sysStatus.battery_remaining;
                m_sysStatus.drop_rate_comm;
                m_sysStatus.errors_comm;
                m_sysStatus.errors_count1;
                m_sysStatus.errors_count2;
                m_sysStatus.errors_count3;
                m_sysStatus.errors_count4;

            }


            /**
             * @brief Refer to mavlink/common/mavlink_msg_local_position_ned.h
             */
            void updateLocPos(uint32_t time_boot_ms, float x, float y, 
                    float z, float vx, float vy, float vz) {

                m_locPos.time_boot_ms = time_boot_ms;
                m_locPos.x = x;
                m_locPos.y = y;
                m_locPos.z = z;
                m_locPos.vx = vx;
                m_locPos.vy = vy;
                m_locPos.vz = vz;

            }


            /**
             * @brief Refer to mavlink/common/mavlink_msg_attitude.h
             */
            void updateAtt(uint32_t time_boot_ms, float roll, float pitch, 
                    float yaw, float rollspeed, float pitchspeed, 
                    float yawspeed) {

                m_att.time_boot_ms = time_boot_ms;
                m_att.roll = roll;
                m_att.pitch = pitch;
                m_att.yaw = yaw;
                m_att.rollspeed = rollspeed;
                m_att.pitchspeed = pitchspeed;
                m_att.yawspeed = yawspeed;

            }


        private:
            static MavHandler* m_instance;
            Communicator* m_comm;
            MsgManager* m_mng;

            mavlink_system_t m_mavSys;
            mavlink_heartbeat_t m_mavHB;
            mavlink_sys_status_t m_sysStatus;
            mavlink_local_position_ned_t m_locPos;
            mavlink_attitude_t m_att;

            //TODO: temporary?
            int m_hbIdx;
            int m_statusIdx;
            int m_locPosIdx;
            int m_attIdx;

            int m_tempWpIdx;
    };

    MavHandler* MavHandler::m_instance = NULL;

}


#endif
