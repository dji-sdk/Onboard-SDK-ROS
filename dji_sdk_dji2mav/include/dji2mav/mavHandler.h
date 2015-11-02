/****************************************************************************
 * @brief   Handle mavlink messages packing and unpacking. ROS-free singleton
 * @version 1.0
 * @Date    2014/10/31
 ****************************************************************************/

#ifndef _DJI2MAV_MAVHANDLER_H_
#define _DJI2MAV_MAVHANDLER_H_


#include "communicator.h"
#include "msgManager.h"

#include <mavlink/v1.0/common/mavlink.h>
#include <new>
#include <string>

#define DEFAULT_SENDER_LIST_SIZE 256
#define DEFAULT_RECV_BUF_SIZE 4096

namespace dji2mav {

    class MavHandler {
        public:
            /**
             * @brief   Lazy mode singleton
             * @return  The only instance of MavHandler
             * @warning UNSAFE FOR MULTI-THREAD!
             */
            MavHandler* getInstance() {
                if(NULL == m_instance) {
                    try {
                        m_instance = new MavHandler;
                    } catch(bad_alloc& m) {
                        perror( "Cannot new instance of MavHandler: " 
                                + m.what() );
                    }
                }
                return m_instance;
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
            bool establish(string gcsIP, int gcsPort, int locPort, 
                    uint16_t senderListSize = DEFAULT_SENDER_LIST_SIZE, 
                    uint16_t recvBufSize = DEFAULT_RECV_BUF_SIZE) {

                m_comm = Communicator::getInstance();
                m_comm.setConf(gcsIP, gcsPort, locPort);
                if( m_comm.connect() == false ) {
                    perror("Fail to connect to GCS!");
                    return false;
                }
                m_mng = MsgManager::getInstance(senderListSize, recvBufSize);
                return true;

            }


            //TODO: temporary
            //-1 for fail and -2 for having not registered yet
            int registerSender() {
                return m_mng->registerSender();
            }


            /**
             * @brief  Send heartbeat once
             * @return True if succeed and false if fail
             */
            bool sendHB_Once() {
                if(-2 == m_HBSender) {
                    m_HBSender = this.registerSender();
                }
                if(-1 == m_HBSender) {
                    perror("Fail to register a heartbeat sender!");
                    return false;
                }

                mavlink_message_t msg;
                mavlink_msg_heartbeat_pack(m_mavSys.sysid, m_mavSys.compid, 
                        &msg, m_mavHB.type, m_mavHB.autopilot, 
                        m_mavHB.base_mode, m_mavHB.custom_mode, 
                        m_mavHB.system_status);
                uint16_t len = mavlink_msg_to_send_buffer(
                        m_HBSender.getBuf(), &msg);
                int bytes_sent = m_mng.send(m_HBSender, len);
                if(bytes_sent < 0) {
                    perror("Fail to send heartbeat pack!");
                    return false;
                }
                if( (uint16_t)bytes_sent != len ) {
                    perror("%d bytes were sent, which didn't match %d bytes!", 
                            bytes_sent, len);
                    return false;
                }
                return true;
            }


            /**
             * @brief  Receive data and parse it from buffer to mavlink
             * @return True if succeed and false if fail
             */
            bool receive(mavlink_message_t &recvMsg, 
                    mavlink_status_t &recvStatus) {
                bool ret = false;
                int bytes_recv = m_mng->recv();
                if(bytes_recv < 0) {
                    perror("Fail to execute receive process!");
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
                            printf("\nReceived packet: SYS: %d, COMP: %d, LEN: %d, MSG ID: %d\n", recvMsg.sysid, recvMsg.compid, recvMsg.len, recvMsg.msgid); //TODO: temporary
                        }
                    }
                    printf("\n"); //TODO: temporary
                }
                return ret;
            }


        private:
            MavHandler() {
                //TODO: temporary
                m_HBSender = -2;
            }


            ~MavHandler() {
            }


            static MavHandler* m_instance;
            Communicator* m_comm;
            MsgManager* m_mng;
            mavlink_system_t m_mavSys;
            mavlink_heartbeat_t m_mavHB;

            //TODO: temporary
            int m_HBSender;
    };

}


#endif
