/****************************************************************************
 * @Brief   Contain various applications, such as waypoint. ROS-free singleton
 * @Version 1.0
 * @Author  Chris Liu
 * @Create  2015/11/12
 * @Modify  2015/11/12
 ****************************************************************************/

#ifndef _MAVCONTAINER_H_
#define _MAVCONTAINER_H_


#include "mavWaypoint.h"
#include "mavResponser.h"

#include <mavlink/v1.0/common/mavlink.h>
#include <new>

namespace dji2mav {

    class MavContainer {
        public:
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
                m_hdlr = MavHandler::getInstance();
                m_wp = MavWaypoint::getInstance();
                m_rsp = MavResponser::getInstance();

                m_wpStatus = idle;
            }


            ~MavContainer() {
            }


            static MavContainer* m_instance;
            MavHandler* m_hdlr;
            MavWaypoint* m_wp;
            MavResponser* m_rsp;

            // waypoint state machine
            enum {idle, ready, executing, paused, error} m_wpStatus;

    };

    MavContainer* MavContainer::m_instance = NULL;

}


#endif
