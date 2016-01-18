/*****************************************************************************
 * @Brief     Waypoint module. Implement waypoint protocol here
 * @Version   0.3.0
 * @Author    Chris Liu
 * @Created   2015/11/18
 * @Modified  2015/12/25
 *****************************************************************************/

#ifndef _MAV2DJI_MAVWAYPOINT_H_
#define _MAV2DJI_MAVWAYPOINT_H_


#include <mavlink/common/mavlink.h>
#include <new>
#include <string>
#include <cstdarg>

#include "dji_sdk_dji2mav/modules/mavModule.h"
#include "waypointList.h"
#include "dji_sdk_dji2mav/log.h"

namespace dji2mav{

    class MavWaypoint : public MavModule{
        public:
            /**
             * @brief Constructor for Waypoint Module. Recv buf size 4096
             * @param handler : The reference of MavHandler Object
             * @param name    : The name of this module
             * @param gcsNum  : The number of GCS that the module employs
             * @param ...     : The indexes of GCS list, should be uint16_t
             */
            MavWaypoint(MavHandler &handler, std::string name, 
                    uint16_t gcsNum, ...) : MavModule(handler, name, 4096) {

                DJI2MAV_DEBUG("Going to construct Waypoint module with name " 
                        "%s and gcsNum %u...", name.c_str(), gcsNum);

                setMissionRequestListHook(NULL);
                setMissionRequestHook(NULL);
                setMissionAckHook(NULL);
                setMissionCountHook(NULL);
                setMissionItemHook(NULL);
                setMissionClearAllHook(NULL);
                setMissionSetCurrentHook(NULL);

                va_list arg;
                va_start(arg, gcsNum);
                if(1 == gcsNum) {
                    setMasterGcsIdx( (uint16_t)va_arg(arg, int) );
                } else {
                    for(uint16_t i = 0; i < gcsNum; ++i) {
                        employGcsSender( (uint16_t)va_arg(arg, int) );
                    }
                }
                va_end(arg);

                //TODO: Also set the MOI here

                DJI2MAV_DEBUG("...finish constructing Waypoint module.");

            }


            ~MavWaypoint() {
                DJI2MAV_DEBUG("Going to destruct Waypoint module...");
                DJI2MAV_DEBUG("...finish destructing Waypoint moduel.");
            }


            /**
             * @brief Implement the messages passively handling function
             * @param msg : The reference of received message
             */
            void passivelyReceive(mavlink_message_t &msg) {
                switch(msg.msgid) {
                    //TODO: shrink the processors?
                    case MAVLINK_MSG_ID_MISSION_REQUEST_LIST:
                        reactToMissionRequestList(getMasterGcsIdx(), msg);
                        break;
                    case MAVLINK_MSG_ID_MISSION_REQUEST:
                        reactToMissionRequest(getMasterGcsIdx(), msg);
                        break;
                    case MAVLINK_MSG_ID_MISSION_ACK:
                        reactToMissionAck(getMasterGcsIdx(), msg);
                        break;
                    case MAVLINK_MSG_ID_MISSION_COUNT:
                        reactToMissionCount(getMasterGcsIdx(), msg);
                        break;
                    case MAVLINK_MSG_ID_MISSION_ITEM:
                        reactToMissionItem(getMasterGcsIdx(), msg);
                        break;
                    case MAVLINK_MSG_ID_MISSION_CLEAR_ALL:
                        reactToMissionClearAll(getMasterGcsIdx(), msg);
                        break;
                    case MAVLINK_MSG_ID_MISSION_SET_CURRENT:
                        reactToMissionSetCurrent(getMasterGcsIdx(), msg);
                        break;
                    default:
                        DJI2MAV_WARN("No execution is defined for msgid #%u "
                                "in Hotpoint module.", msg.msgid);
                        break;
                }
                usleep(20000); //50Hz
            }


            /**
             * @brief Implement the messages actively sending function
             */
            void activelySend() {
            }


            void reactToMissionRequestList(uint16_t gcsIdx, 
                    const mavlink_message_t &recvMsg) {

                if(recvMsg.compid != MAV_COMP_ID_MISSIONPLANNER) {
                    DJI2MAV_DEBUG("In Waypoint the compid is %u.", 
                            recvMsg.compid);
                }

                DJI2MAV_DEBUG("In Waypoint mission request list with status: " 
                        "%d.", (int)m_status);

                switch(m_status) {
                    case idle:
                    case downloading:
                    case executing:
                    case paused:
                        return;
                    case loaded:
                        m_status = uploading;
                        break;
                    case uploading:
                    case error:
                        break;
                }

                mavlink_mission_count_t cntMsg;
                cntMsg.target_system = recvMsg.sysid;
                cntMsg.target_component = recvMsg.compid;
                cntMsg.count = m_wpl.getListSize();
                m_wpl.readyToUpload();

                mavlink_message_t sendMsg;
                mavlink_msg_mission_count_encode(getMySysid(), 
                        MAV_COMP_ID_MISSIONPLANNER, &sendMsg, &cntMsg);
                sendMsgToMaster(sendMsg);

                if(NULL != m_missionRequestListHook)
                    m_missionRequestListHook();

            }


            void reactToMissionRequest(uint16_t gcsIdx, 
                    const mavlink_message_t &recvMsg) {

                if(recvMsg.compid != MAV_COMP_ID_MISSIONPLANNER) {
                    DJI2MAV_DEBUG("In Waypoint the compid is %u.", 
                            recvMsg.compid);
                }

                DJI2MAV_DEBUG("In Waypoint mission request with status: " 
                        "%d.", (int)m_status);

                switch(m_status) {
                    case idle:
                    case downloading:
                    case loaded:
                    case executing:
                    case paused:
                        return;
                    case uploading:
                    case error:
                        break;
                }

                mavlink_mission_request_t reqMsg;
                mavlink_msg_mission_request_decode(&recvMsg, &reqMsg);

                mavlink_mission_item_t itemMsg;
                itemMsg.target_system = recvMsg.sysid;
                itemMsg.target_component = recvMsg.compid;
                itemMsg.seq = reqMsg.seq;
                if( m_wpl.isValidIdx(itemMsg.seq) ) {

                    m_wpl.getWaypointData(itemMsg.seq, itemMsg.command, 
                            itemMsg.param1, itemMsg.param2, itemMsg.param3, 
                            itemMsg.param4, itemMsg.x, itemMsg.y, itemMsg.z);

                } else {
                    DJI2MAV_ERROR("Invalid sequence %u of mission request in" 
                            "Waypoint!", reqMsg.seq);
                    return;
                }

                mavlink_message_t sendMsg;
                mavlink_msg_mission_item_encode(getMySysid(), 
                        MAV_COMP_ID_MISSIONPLANNER, &sendMsg, &itemMsg);
                sendMsgToMaster(sendMsg);

                if(NULL != m_missionRequestHook)
                    m_missionRequestHook(reqMsg.seq);

            }


            void reactToMissionAck(uint16_t gcsIdx, 
                    const mavlink_message_t &recvMsg) {

                if(recvMsg.compid != MAV_COMP_ID_MISSIONPLANNER) {
                    DJI2MAV_DEBUG("In Wotpoint the compid is %u.", 
                            recvMsg.compid);
                }

                DJI2MAV_DEBUG("In Waypoint mission ack with status: %d.", 
                        (int)m_status);

                switch(m_status) {
                    case idle:
                    case downloading:
                    case loaded:
                    case executing:
                    case paused:
                        return;
                    case uploading:
                        m_status = loaded;
                        break;
                    case error:
                        break;
                }

                mavlink_mission_ack_t ackMsg;
                mavlink_msg_mission_ack_decode(&recvMsg, &ackMsg);
                DJI2MAV_DEBUG("In Waypoint mission ACK code: %d.", ackMsg.type);
                m_status = loaded;
                m_wpl.finishUpload();
                m_wpl.displayMission();

                if(NULL != m_missionAckHook)
                    m_missionAckHook();

            }


            void reactToMissionCount(uint16_t gcsIdx, 
                    const mavlink_message_t &recvMsg) {

                if(recvMsg.compid != MAV_COMP_ID_MISSIONPLANNER) {
                    DJI2MAV_DEBUG("In Hotpoint the compid is %u.", 
                            recvMsg.compid);
                }

                DJI2MAV_DEBUG("In Waypoint mission count with status: %d.", 
                        (int)m_status);

                switch(m_status) {
                    case idle:
                    case loaded:
                        m_status = downloading;
                        break;
                    case downloading:
                    case uploading:
                    case executing:
                    case paused:
                    case error:
                        return;
                }

                mavlink_mission_count_t cntMsg;
                mavlink_msg_mission_count_decode(&recvMsg, &cntMsg);
                m_wpl.setListSize(cntMsg.count);
                m_wpl.readyToDownload();

                mavlink_mission_request_t reqMsg;
                reqMsg.target_system = recvMsg.sysid;
                reqMsg.target_component = recvMsg.compid;
                reqMsg.seq = 0;

                mavlink_message_t sendMsg;
                mavlink_msg_mission_request_encode(getMySysid(), 
                        MAV_COMP_ID_MISSIONPLANNER, &sendMsg, &reqMsg);
                sendMsgToMaster(sendMsg);

                if(NULL != m_missionCountHook)
                    m_missionCountHook(cntMsg.count);

            }


            void reactToMissionItem(uint16_t gcsIdx, 
                    const mavlink_message_t &recvMsg) {

                if(recvMsg.compid != MAV_COMP_ID_MISSIONPLANNER) {
                    DJI2MAV_DEBUG("In Waypoint the compid is %u.", 
                            recvMsg.compid);
                }

                DJI2MAV_DEBUG("In Hotpoint mission item with status: %d.", 
                        (int)m_status);

                switch(m_status) {
                    case loaded:
                    case executing:
                    case idle:
                    case uploading:
                    case paused:
                    case error:
                        return;
                    case downloading:
                        break;
                }

                mavlink_mission_item_t itemMsg;
                mavlink_msg_mission_item_decode(&recvMsg, &itemMsg);
                if( m_wpl.isValidIdx(itemMsg.seq) ) {

                    m_wpl.setWaypointData(itemMsg.seq, itemMsg.command, 
                            itemMsg.param1, itemMsg.param2, itemMsg.param3, 
                            itemMsg.param4, itemMsg.x, itemMsg.y, itemMsg.z);

                } else {
                    DJI2MAV_ERROR("Invalid sequence %u of mission item in " 
                            "Waypoint!", itemMsg.seq);
                    return;
                }

                mavlink_message_t sendMsg;
                if( m_wpl.isDownloadFinished() ) {
                    mavlink_msg_mission_ack_pack(getMySysid(), 
                            MAV_COMP_ID_MISSIONPLANNER, &sendMsg, 
                            recvMsg.sysid, recvMsg.compid, 
                            MAV_MISSION_ACCEPTED);
                    sendMsgToMaster(sendMsg);
                    m_status = loaded;
                    m_wpl.displayMission();
                } else {
                    mavlink_mission_request_t reqMsg;
                    reqMsg.target_system = recvMsg.sysid;
                    reqMsg.target_component = recvMsg.compid;
                    reqMsg.seq = m_wpl.getTargetIdx();
                    mavlink_msg_mission_request_encode(getMySysid(), 
                            MAV_COMP_ID_MISSIONPLANNER, &sendMsg, &reqMsg);
                    sendMsgToMaster(sendMsg);
                    DJI2MAV_TRACE("In Waypoint send request %u, %u, %u.", 
                            reqMsg.target_system, reqMsg.target_component, 
                            reqMsg.seq);
                }

                if(NULL != m_missionItemHook)
                    m_missionItemHook(itemMsg.seq);

            }


            void reactToMissionClearAll(uint16_t gcsIdx, 
                    const mavlink_message_t &recvMsg) {

                if(recvMsg.compid != MAV_COMP_ID_MISSIONPLANNER) {
                    DJI2MAV_DEBUG("In Waypoint the compid is %u\n", 
                            recvMsg.compid);
                }

                DJI2MAV_DEBUG("In Waypoint mission clear all with status: %d", 
                        (int)m_status);

                switch(m_status) {
                    case idle:
                    case uploading:
                    case downloading:
                    case loaded:
                    case error:
                        m_status = idle;
                        break;
                    case executing:
                    case paused:
                        return;
                }

                mavlink_mission_ack_t ackMsg;
                ackMsg.target_system = recvMsg.sysid;
                ackMsg.target_component = recvMsg.compid;
                ackMsg.type = MAV_MISSION_ACCEPTED;

                mavlink_message_t sendMsg;
                mavlink_msg_mission_ack_encode(getMySysid(), 
                        MAV_COMP_ID_MISSIONPLANNER, &sendMsg, &ackMsg);
                sendMsgToMaster(sendMsg);

                m_wpl.clearMission();

                if(NULL != m_missionClearAllHook)
                    m_missionClearAllHook();

            }


            void reactToMissionSetCurrent(uint16_t gcsIdx, 
                    const mavlink_message_t &recvMsg) {

                if(recvMsg.compid != MAV_COMP_ID_MISSIONPLANNER) {
                    DJI2MAV_DEBUG("In Waypoint the compid is %u.", 
                            recvMsg.compid);
                }

                DJI2MAV_DEBUG("In Waypoint mission set current with status: " 
                        "%d.", (int)m_status);

                switch(m_status) {
                    case idle:
                    case uploading:
                    case downloading:
                    case error:
                        return;
                    case loaded:
                    case paused:
                        m_status = executing;
                        break;
                    case executing:
                        break;
                }

                mavlink_mission_set_current_t setCurrMsg;
                mavlink_msg_mission_set_current_decode(&recvMsg, &setCurrMsg);
                if( m_wpl.isValidIdx(setCurrMsg.seq) ) {

                    m_wpl.setTargetIdx(setCurrMsg.seq);

                    if(NULL != m_targetHook) {
                        m_targetHook( m_wpl.getWaypointList(), 
                                setCurrMsg.seq, 
                                m_wpl.getListSize() );
                        //It is slow to send mission to FC. Now there should be
                        //duplicate cmd msg in the buffer. Clear them
                        clearBuf();
                    } else {
                        DJI2MAV_WARN("In Waypoint no target hook is set.");
                    }

                    mavlink_message_t sendMsg;
                    mavlink_msg_mission_current_pack( getMySysid(), 
                            MAV_COMP_ID_MISSIONPLANNER, &sendMsg, 
                            m_wpl.getTargetIdx() );
                    sendMsgToMaster(sendMsg);

                    m_status = loaded;//TODO finish the task

                } else {
                    m_status = paused;
                    DJI2MAV_ERROR( "The sequence %u of set current message is " 
                            "invalid in Waypoint! The size of list is %u!", 
                            setCurrMsg.seq, m_wpl.getListSize() );
                }

                if(NULL != m_missionSetCurrentHook)
                    m_missionSetCurrentHook(setCurrMsg.seq);

            }


            /**
             * @brief  React to sensors from all GCS and execute rsp
             * @return True if succeed or false if fail
             */
            inline void setMissionRequestListHook( void (*func)() ) {
                m_missionRequestListHook = func;
            }


            inline void setMissionRequestHook( void (*func)(uint16_t) ) {
                m_missionRequestHook = func;
            }


            inline void setMissionAckHook( void (*func)() ) {
                m_missionAckHook = func;
            }


            inline void setMissionCountHook( void (*func)(uint16_t) ) {
                m_missionCountHook = func;
            }


            inline void setMissionItemHook( void (*func)(uint16_t) ) {
                m_missionItemHook = func;
            }


            inline void setMissionClearAllHook( void (*func)() ) {
                m_missionClearAllHook = func;
            }


            inline void setMissionSetCurrentHook( void (*func)(uint16_t) ) {
                m_missionSetCurrentHook = func;
            }


            inline void setTargetHook( void (*func)(const float[][7], uint16_t, uint16_t) ) {
                m_targetHook = func;
            }


        private:
            WaypointList m_wpl;

            enum {
                idle, 
                uploading, 
                downloading, 
                loaded, 
                executing, 
                paused, 
                error
            } m_status;

            void (*m_missionRequestListHook)();
            void (*m_missionRequestHook)(uint16_t);
            void (*m_missionAckHook)();
            void (*m_missionCountHook)(uint16_t);
            void (*m_missionItemHook)(uint16_t);
            void (*m_missionClearAllHook)();
            void (*m_missionSetCurrentHook)(uint16_t);
            void (*m_targetHook)(const float[][7], uint16_t, uint16_t);


    };

} //namespace dji2mav


#endif
