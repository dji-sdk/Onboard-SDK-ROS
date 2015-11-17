/****************************************************************************
 * @Brief   Contain various modules, such as waypoint. ROS-free singleton
 * @Version 1.1
 * @Author  Chris Liu
 * @Create  2015/11/12
 * @Modify  2015/11/16
 ****************************************************************************/

#ifndef _MAVCONTAINER_H_
#define _MAVCONTAINER_H_


#include "mngHandler.h"
#include "modules/heartbeat/mavHeartbeat.h"
#include "modules/sensors/mavSensors.h"
#include "modules/waypoint/mavWaypoint.h"

#include <mavlink/v1.0/common/mavlink.h>
#include <new>

namespace dji2mav {

    class MavContainer {
        public:
            /**
             * @brief   Get the only instance. Lazy mode singleton
             * @return  The only instance of MavContainer
             * @warning UNSAFE FOR MULTI-THREAD! Should be called BEFORE fork
             */
            static MavContainer* getInstance() {
                if(NULL == m_instance) {
                    try {
                        m_instance = new MavContainer;
                    } catch(std::bad_alloc &m) {
                        std::cerr << "New instance of MavContainer fail: " 
                                << "at line: " << __LINE__ << ", func: " 
                                << __func__ << ", file: " << __FILE__ 
                                << std::endl;
                        perror( m.what() );
                        exit(EXIT_FAILURE);
                    }
                }
                return m_instance;
            }


            /**
             * @brief  Alloc memory for senderTable
             * @param  num : The number of GCS(Ground Control Stations)
             * @return True if succeed or false if fail
             */
            bool setContainerConf(uint16_t num) {

                if(0 != m_gcsNum) {
                    printf("The GCS number has been set before!\n");
                    return false;
                }

                m_gcsNum = num;

                try {
                    m_senderTable = new int*[m_module.SIZE];
                    for(uint16_t i = 0; i < m_module.SIZE; ++i) {
                        m_senderTable[i] = new int[m_gcsNum];
                    }
                } catch(std::bad_alloc& m) {
                    std::cerr << "Failed to alloc memory for senderTable: " 
                            << "at line: " << __LINE__ << ", func: " 
                            << __func__ << ", file: " << __FILE__ 
                            << std::endl;
                    perror( m.what() );
                    exit(EXIT_FAILURE);
                }

                for(uint16_t i = 0; i < m_gcsNum; ++i) {

                    // send heartbeat to every GCS
                    m_senderTable[m_module.Heartbeat][i] 
                            = m_hdlr->registerSender(i);

                    // send sensor data to every GCS
                    m_senderTable[m_module.Sensors][i] 
                            = m_hdlr->registerSender(i);

                    // only execute waypoint command from GCS #0
                    if(0 == i) {
                        m_senderTable[m_module.Waypoint][0] 
                                = m_hdlr->getGeneralSenderIdx(0);
                    }

                }

                return true;

            }


            /**
             * @breif  Get the number of GCS
             * @return The number of GCS
             */
            inline uint16_t getGcsNum() {
                return m_gcsNum;
            }


            /**
             * @brief  Check whether the mng index is valid
             * @param  mngIdx : The index of mng that is to be checked
             * @return True if valid or false if invalid
             */
            inline bool isValidMngIdx(uint16_t mngIdx) {
                return m_hdlr->isValidMngIdx(mngIdx);
            }


            /**
             * @brief  Check whether the mng index and sender index are valid
             * @param  mngIdx    : The index of mng that is to be checked
             * @param  senderIdx : The index of sender that is to be checked
             * @return True if valid or false if invalid
             */
            inline bool isValidIdx(uint16_t mngIdx, uint16_t senderIdx) {
                return ( m_hdlr->isValidMngIdx(mngIdx)
                        && m_hdlr->isValidSenderIdx(mngIdx, senderIdx) );
            }


            /**
             * @brief Should be called in the global loop function
             */
            void execute() {
                mavlink_message_t recvMsg;
                mavlink_status_t status;
                if( m_hdlr->receive(recvMsg, status) ) {
                    printf(">>> GET ONE MESSAGE!\n");
                    mavlink_message_t sendMsg;
                    mavlink_mission_request_t req;
                    mavlink_mission_item_t item;
                    mavlink_mission_ack_t ack;
                    mavlink_mission_count_t cnt;
                    switch(recvMsg.msgid) {
                        int tmpLen;
                        //TODO: should consider target sysid and compid here
                        case MAVLINK_MSG_ID_HEARTBEAT:
                            if( m_rsp->isSetHeartbeatRsp() )
                                m_rsp->respondToHeartbeat();
                            break;
                        case MAVLINK_MSG_ID_PING:
                            if( m_rsp->isSetPingRsp() )
                                m_rsp->respondToPing();
                            break;
                        // waypoint application
                        case MAVLINK_MSG_ID_MISSION_REQUEST_LIST:
                            m_wp->displayMission();
                            //cnt->target_system = m_hdlr->
                            m_wp->getListLen(tmpLen);
                            mavlink_msg_mission_count_pack(1, 200, &sendMsg, recvMsg.sysid, recvMsg.compid, tmpLen);//TODO get sysid etc
                            m_hdlr->sendEncodedMsg(0, &sendMsg);
                            if( m_rsp->isSetMissionRequestListRsp() )
                                m_rsp->respondToMissionRequestList();
                            break;
                        case MAVLINK_MSG_ID_MISSION_REQUEST:
                            mavlink_msg_mission_request_decode(&recvMsg, &req);
                            item.target_system = recvMsg.sysid;
                            item.target_component = recvMsg.compid;
                            item.seq = req.seq;
                            m_wp->getWaypoint(req.seq, item.x, item.y, item.z);
                            item.x = item.x * 180.0 / M_PI;
                            item.y = item.y * 180.0 / M_PI;
                            mavlink_msg_mission_item_encode(1, 200, &sendMsg, &item);
                            m_hdlr->sendEncodedMsg(0, &sendMsg);
                            if( m_rsp->isSetMissionRequestRsp() )
                                m_rsp->respondToMissionRequest(&req.seq);
                            break;
                        case MAVLINK_MSG_ID_MISSION_ACK:
                            mavlink_msg_mission_ack_decode(&recvMsg, &ack);
                            printf("Mission ACK code: %d\n", ack.type);
                            if( m_rsp->isSetMissionAckRsp() )
                                m_rsp->respondToMissionAck();
                            break;
                        case MAVLINK_MSG_ID_MISSION_COUNT:
                            mavlink_msg_mission_count_decode(&recvMsg, &cnt);
                            m_wp->setListLen(cnt.count);
                            mavlink_msg_mission_request_pack(1, 200, &sendMsg, recvMsg.sysid, recvMsg.compid, 0);//seq 0
                            m_hdlr->sendEncodedMsg(0, &sendMsg);
                            if( m_rsp->isSetMissionCountRsp() )
                                m_rsp->respondToMissionCount(&cnt.count);
                            break;
                        case MAVLINK_MSG_ID_MISSION_ITEM:
                            mavlink_msg_mission_item_decode(&recvMsg, &item);
                            m_wp->setWaypoint(item.seq, item.x / 180.0 * M_PI, 
                                    item.y / 180.0 * M_PI, item.z);//have to use globle data
                            if( m_wp->getNextIdx() ) {
                                mavlink_msg_mission_request_pack(1, 200, &sendMsg, recvMsg.sysid, recvMsg.compid, item.seq + 1);
                                m_hdlr->sendEncodedMsg(0, &sendMsg);
                            }
                            else {
                                mavlink_msg_mission_ack_pack(1, 200, &sendMsg, recvMsg.sysid, recvMsg.compid, MAV_MISSION_ACCEPTED);//TODO
                                m_wp->displayMission();
                                m_hdlr->sendEncodedMsg(0, &sendMsg);
                            }
                            if( m_rsp->isSetMissionItemRsp() )
                                m_rsp->respondToMissionItem(&item.seq);
                            break;
                        default:
                            printf("--- Ehhhh, no match msgid %d! ---\n", recvMsg.msgid);
                    }
                }
            }


        private:
            MavContainer() {
                m_hdlr = MngHandler::getInstance();
                m_hb = MavHeartbeat::getInstance();
                m_wp = MavWaypoint::getInstance();
                m_gcsNum = 0;
            }


            ~MavContainer() {
            }


            enum Modules {
                Heartbeat = 0,
                Sensors,
                Waypoint,
                SIZE
            } m_module;


            static MavContainer* m_instance;
            MngHandler* m_hdlr;
            MavHeartbeat* m_hb;
            MavSensors* m_sensors;
            MavWaypoint* m_wp;
            int** senderTable;
            uint16_t m_gcsNum;

    };

    MavContainer* MavContainer::m_instance = NULL;

}


#endif
