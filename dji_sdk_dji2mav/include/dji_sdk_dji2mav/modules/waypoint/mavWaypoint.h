/*****************************************************************************
 * @Brief     Waypoint module. Implement waypoint protocol here
 * @Version   0.2.1
 * @Author    Chris Liu
 * @Created   2015/11/18
 * @Modified  2015/11/24
 *****************************************************************************/

#ifndef _MAV2DJI_MAVWAYPOINT_H_
#define _MAV2DJI_MAVWAYPOINT_H_


#include "../../mavHandler.h"
#include "waypointList.h"

#include <iostream>
#include <stdio.h>
#include <new>
#include <assert.h>
#include <limits.h>

namespace dji2mav{

    class MavWaypoint {
        public:
            /**
             * @brief   Get the only instance. Lazy mode singleton
             * @return  The only instance of MavWaypoint
             * @warning UNSAFE FOR MULTI-THREAD! Should be called BEFORE fork
             */
            static MavWaypoint* getInstance() {
                if(NULL == m_instance) {
                    try {
                        m_instance = new MavWaypoint();
                    } catch(std::bad_alloc &m) {
                        std::cerr << "New instance of MavWaypoint fail: " 
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
             * @brief  Get the senderRecord array address
             * @return The address of senderRecord
             */
            inline const int* getSenderRecord() {
                return m_senderRecord;
            }


            /**
             * @brief  Set the sender index of specific GCS
             * @param  gcsIdx    : The index of GCS
             * @param  senderIdx : The index of sender
             * @return True if succeed or false if fail
             */
            bool setSenderIdx(uint16_t gcsIdx, int senderIdx) {
                if( !m_hdlr->isValidIdx(gcsIdx, senderIdx) )
                    return false;
                m_senderRecord[gcsIdx] = senderIdx;
                return true;
            }


            /**
             * @brief  Set the master with its gcsIdx and senderIdx
             * @param  gcsIdx    : The index of GCS that is to be set
             * @param  senderIdx : The index of sender of GCS
             * @return True if succeed or false if fail
             */
            bool setMasterGcsIdx(uint16_t gcsIdx, uint16_t senderIdx) {
                if( m_hdlr->isValidIdx(gcsIdx, senderIdx) ) {
                    m_senderRecord[m_masterGcsIdx] = -1;
                    m_masterGcsIdx = gcsIdx;
                    m_senderRecord[m_masterGcsIdx] = senderIdx;
                    return true;
                } else {
                    printf("Invalid master GCS index %u "
                            "and sender index %u.\n", gcsIdx, senderIdx);
                    return false;
                }
            }


            /**
             * @brief  Register new sender and use it for specific GCS
             * @param  gcsIdx : The index of GCS
             * @return True if succeed or false if fail
             */
            bool applyNewSender(uint16_t gcsIdx) {
                int newSender = m_hdlr->registerSender(gcsIdx);
                if( newSender < 0 ) {
                    printf("Fail to regiser sender for waypoint in GCS #%u! "
                            "Did you set sender list too small?\n", gcsIdx);
                    return false;
                }
                m_senderRecord[gcsIdx] = newSender;
                return true;
            }


            inline bool applyNewSender() {
                return applyNewSender(m_masterGcsIdx);
            }


            /**
             * @brief Use the general sender
             * @param  gcsIdx : The index of GCS
             */
            inline void applyGeneralSender(uint16_t gcsIdx) {
                m_senderRecord[gcsIdx] = m_hdlr->getGeneralSenderIdx(gcsIdx);
            }


            inline void applyGeneralSender() {
                applyGeneralSender(m_masterGcsIdx);
            }


            void reactToMissionRequestList(uint16_t gcsIdx, 
                    const mavlink_message_t* recvMsgPtr) {

                if(gcsIdx != m_masterGcsIdx)
                    return;

                if(recvMsgPtr->compid != MAV_COMP_ID_MISSIONPLANNER) {
                    printf("WARNNING! The compid is %u\n", recvMsgPtr->compid);
                }

                printf("In mission request list with status: %d\n", (int)m_status);

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

                m_cntMsg.target_system = recvMsgPtr->sysid;
                m_cntMsg.target_component = recvMsgPtr->compid;
                m_cntMsg.count = m_wpl.getListSize();
                m_wpl.readyToUpload();
                mavlink_msg_mission_count_encode(m_hdlr->getSysid(), 
                        MAV_COMP_ID_MISSIONPLANNER, &m_sendMsg, &m_cntMsg);
                m_hdlr->sendEncodedMsg(m_masterGcsIdx, 
                        m_senderRecord[m_masterGcsIdx], &m_sendMsg);

                if(NULL != m_missionRequestListRsp)
                    m_missionRequestListRsp();

            }


            void reactToMissionRequest(uint16_t gcsIdx, 
                    const mavlink_message_t* recvMsgPtr) {

                if(gcsIdx != m_masterGcsIdx)
                    return;

                if(recvMsgPtr->compid != MAV_COMP_ID_MISSIONPLANNER) {
                    printf("WARNNING! The compid is %u\n", recvMsgPtr->compid);
                }

                printf("In mission request with status: %d\n", (int)m_status);

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

                mavlink_msg_mission_request_decode(recvMsgPtr, &m_reqMsg);

                m_itemMsg.target_system = recvMsgPtr->sysid;
                m_itemMsg.target_component = recvMsgPtr->compid;
                m_itemMsg.seq = m_reqMsg.seq;
                if( m_wpl.isValidIdx(m_itemMsg.seq) ) {

                    m_wpl.getWaypointData(m_itemMsg.seq, m_itemMsg.command, 
                            m_itemMsg.param1, m_itemMsg.param2, 
                            m_itemMsg.param3, m_itemMsg.param4, 
                            m_itemMsg.x, m_itemMsg.y, m_itemMsg.z);


                } else {
                    printf("Invalid index of waypoint in waypoint module!\n");
                    return;
                }

                mavlink_msg_mission_item_encode(m_hdlr->getSysid(), 
                        MAV_COMP_ID_MISSIONPLANNER, &m_sendMsg, &m_itemMsg);
                m_hdlr->sendEncodedMsg(m_masterGcsIdx, 
                        m_senderRecord[m_masterGcsIdx], &m_sendMsg);

                if(NULL != m_missionRequestRsp)
                    m_missionRequestRsp(m_reqMsg.seq);

            }


            void reactToMissionAck(uint16_t gcsIdx, 
                    const mavlink_message_t* recvMsgPtr) {

                if(gcsIdx != m_masterGcsIdx)
                    return;

                if(recvMsgPtr->compid != MAV_COMP_ID_MISSIONPLANNER) {
                    printf("WARNNING! The compid is %u\n", recvMsgPtr->compid);
                }

                printf("In mission ack with status: %d\n", (int)m_status);

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

                mavlink_msg_mission_ack_decode(recvMsgPtr, &m_ackMsg);
                printf("Mission ACK code: %d\n", m_ackMsg.type);
                m_status = loaded;
                m_wpl.finishUpload();
                m_wpl.displayMission();

                if(NULL != m_missionAckRsp)
                    m_missionAckRsp();

            }


            void reactToMissionCount(uint16_t gcsIdx, 
                    const mavlink_message_t* recvMsgPtr) {

                if(gcsIdx != m_masterGcsIdx)
                    return;

                if(recvMsgPtr->compid != MAV_COMP_ID_MISSIONPLANNER) {
                    printf("WARNNING! The compid is %u\n", recvMsgPtr->compid);
                }

                printf("In mission count with status: %d\n", (int)m_status);

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

                mavlink_msg_mission_count_decode(recvMsgPtr, &m_cntMsg);
                m_wpl.setListSize(m_cntMsg.count);
                m_wpl.readyToDownload();

                m_reqMsg.target_system = recvMsgPtr->sysid;
                m_reqMsg.target_component = recvMsgPtr->compid;
                m_reqMsg.seq = 0;

                mavlink_msg_mission_request_encode(m_hdlr->getSysid(), 
                        MAV_COMP_ID_MISSIONPLANNER, &m_sendMsg, &m_reqMsg);
                m_hdlr->sendEncodedMsg(m_masterGcsIdx, 
                        m_senderRecord[m_masterGcsIdx], &m_sendMsg);

                if(NULL != m_missionCountRsp)
                    m_missionCountRsp(m_cntMsg.count);

            }


            void reactToMissionItem(uint16_t gcsIdx, 
                    const mavlink_message_t* recvMsgPtr) {

                if(gcsIdx != m_masterGcsIdx)
                    return;

                if(recvMsgPtr->compid != MAV_COMP_ID_MISSIONPLANNER) {
                    printf("WARNNING! The compid is %u\n", recvMsgPtr->compid);
                }

                printf("In mission item with status: %d\n", (int)m_status);

                switch(m_status) {
                    case loaded:
                    case executing:
                        //TODO: Wrong designed in GCS of Win32-Stable-V2.7.1
                        reactToMissionSetCurrent(gcsIdx, recvMsgPtr);
                        m_status = loaded;
                        return;
                    case idle:
                    case uploading:
                    case paused:
                    case error:
                        return;
                    case downloading:
                        break;
                }

                mavlink_msg_mission_item_decode(recvMsgPtr, &m_itemMsg);
                if( m_wpl.isValidIdx(m_itemMsg.seq) ) {
                    m_wpl.setWaypointData(m_itemMsg.seq, m_itemMsg.command, 
                            m_itemMsg.param1, m_itemMsg.param2, 
                            m_itemMsg.param3, m_itemMsg.param4, m_itemMsg.x, 
                            m_itemMsg.y, m_itemMsg.z);
                } else {
                    printf("Invalid index of waypoint in waypoint module!\n");
                    return;
                }
printf(">>>  Mission Item: \ntarget_system: %u, \ntarget_component: %u, \nseq: %u, \nframe: %u, \ncommand: %u, \ncurrent: %u, \nautocontinue: %u, \nparam1: %f, \nparam2: %f, \nparam3: %f, \nparam4: %f, \nx: %f, \ny: %f, \nz: %f \n\n", m_itemMsg.target_system, m_itemMsg.target_component, m_itemMsg.seq, m_itemMsg.frame, m_itemMsg.command, m_itemMsg.current, m_itemMsg.autocontinue, m_itemMsg.param1, m_itemMsg.param2, m_itemMsg.param3, m_itemMsg.param4, m_itemMsg.x, m_itemMsg.y, m_itemMsg.z);

                if( m_wpl.isDownloadFinished() ) {
                    mavlink_msg_mission_ack_pack(m_hdlr->getSysid(), 
                            MAV_COMP_ID_MISSIONPLANNER, &m_sendMsg, 
                            recvMsgPtr->sysid, recvMsgPtr->compid, 
                            MAV_MISSION_ACCEPTED);
                    m_hdlr->sendEncodedMsg(m_masterGcsIdx, 
                            m_senderRecord[m_masterGcsIdx], &m_sendMsg);
                    m_status = loaded;
                    m_wpl.displayMission();
                } else {
                    m_reqMsg.target_system = recvMsgPtr->sysid;
                    m_reqMsg.target_component = recvMsgPtr->compid;
                    m_reqMsg.seq = m_wpl.getTargetIdx();
                    mavlink_msg_mission_request_encode(m_hdlr->getSysid(), 
                            MAV_COMP_ID_MISSIONPLANNER, &m_sendMsg, &m_reqMsg);
                    m_hdlr->sendEncodedMsg(m_masterGcsIdx, 
                            m_senderRecord[m_masterGcsIdx], &m_sendMsg);
printf("Send request %u, %u, %u\n", m_reqMsg.target_system, m_reqMsg.target_component, m_reqMsg.seq);
                }

                if(NULL != m_missionItemRsp)
                    m_missionItemRsp(m_itemMsg.seq);

            }


            void reactToMissionClearAll(uint16_t gcsIdx, 
                    const mavlink_message_t* recvMsgPtr) {

                if(gcsIdx != m_masterGcsIdx)
                    return;

                if(recvMsgPtr->compid != MAV_COMP_ID_MISSIONPLANNER) {
                    printf("WARNNING! The compid is %u\n", recvMsgPtr->compid);
                }

                printf("In mission clear all with status: %d\n", (int)m_status);

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

                m_ackMsg.target_system = recvMsgPtr->sysid;
                m_ackMsg.target_component = recvMsgPtr->compid;
                m_ackMsg.type = MAV_MISSION_ACCEPTED;
                mavlink_msg_mission_ack_encode(m_hdlr->getSysid(), 
                        MAV_COMP_ID_MISSIONPLANNER, &m_sendMsg, &m_ackMsg);
                m_hdlr->sendEncodedMsg(m_masterGcsIdx, 
                        m_senderRecord[m_masterGcsIdx], &m_sendMsg);

                m_wpl.clearMission();

                if(NULL != m_missionClearAllRsp)
                    m_missionClearAllRsp();

            }


            void reactToMissionSetCurrent(uint16_t gcsIdx, 
                    const mavlink_message_t* recvMsgPtr) {

                if(gcsIdx != m_masterGcsIdx)
                    return;

                if(recvMsgPtr->compid != MAV_COMP_ID_MISSIONPLANNER) {
                    printf("WARNNING! The compid is %u\n", recvMsgPtr->compid);
                }

                printf("In mission set current with status: %d\n", (int)m_status);

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

                mavlink_msg_mission_set_current_decode(recvMsgPtr, &m_setCurrMsg);
                if( m_wpl.isValidIdx(m_setCurrMsg.seq) ) {

                    m_wpl.setTargetIdx(m_setCurrMsg.seq);

                    if(NULL != m_targetRsp) {
                        m_targetRsp( m_wpl.getWaypointList(), 
                                m_setCurrMsg.seq, 
                                m_wpl.getListSize() );//TODO
                    }
                    mavlink_msg_mission_current_pack( m_hdlr->getSysid(), 
                            MAV_COMP_ID_MISSIONPLANNER, &m_sendMsg, 
                            m_wpl.getTargetIdx() );
                    m_hdlr->sendEncodedMsg(m_masterGcsIdx, 
                            m_senderRecord[m_masterGcsIdx], &m_sendMsg);

                    m_status = loaded;//TODO finish the task

                } else {
                    m_status = paused;
                    printf( "The current index %u is invalid! "
                            "The size of list is %u.\n", m_setCurrMsg.seq, 
                            m_wpl.getListSize() );
                }

                if(NULL != m_missionSetCurrentRsp)
                    m_missionSetCurrentRsp(m_setCurrMsg.seq);

            }


            /**
             * @brief  React to sensors from all GCS and execute rsp
             * @return True if succeed or false if fail
             */
            inline void setMissionRequestListRsp( void (*func)() ) {
                m_missionRequestListRsp = func;
            }


            inline void setMissionRequestRsp( void (*func)(uint16_t) ) {
                m_missionRequestRsp = func;
            }


            inline void setMissionAckRsp( void (*func)() ) {
                m_missionAckRsp = func;
            }


            inline void setMissionCountRsp( void (*func)(uint16_t) ) {
                m_missionCountRsp = func;
            }


            inline void setMissionItemRsp( void (*func)(uint16_t) ) {
                m_missionItemRsp = func;
            }


            inline void setMissionClearAllRsp( void (*func)() ) {
                m_missionClearAllRsp = func;
            }


            inline void setMissionSetCurrentRsp( void (*func)(uint16_t) ) {
                m_missionSetCurrentRsp = func;
            }


            inline void setTargetRsp( void (*func)(const float[][7], uint16_t, uint16_t) ) {
                m_targetRsp = func;
            }


            /**
             * @brief  Interface for targetRsp to get the waypoint data
             * @param  idx      : The index of specific waypoint in the list
             * @param  lat      : Latitude data, double in dji_sdk
             * @param  lon      : Longitude data, double in dji_sdk
             * @param  alt      : Altitude data, float in dji_sdk
             * @param  heading  : Heading data, int16 in dji_sdk
             * @param  staytime : Staytime data, uint16 in dji_sdk
             * @return True for valid index or false for invalid index
             */
            //TODO: Any better idea of sovling this?
            bool getWaypoint(uint16_t idx, double &lat, double &lon, 
                    float &alt, int16_t &heading, 
                    uint16_t &staytime) {

                if( m_wpl.isValidIdx(idx) ) {
                    lat = (double)m_wpl.getWaypointLat(idx);
                    lon = (double)m_wpl.getWaypointLon(idx);
                    alt = (float)m_wpl.getWaypointAlt(idx);
                    heading = (int16_t)m_wpl.getWpHeading(idx);
                    staytime = (uint16_t)m_wpl.getWpStaytime(idx);
                    return true;
                } else {
                    printf("Invalid index when getting waypoint!\n");
                    return false;
                }

            }


            void distructor() {
                delete m_instance;
            }


        private:
            MavWaypoint() {

                assert(CHAR_BIT * sizeof(float) == 32);
                assert(CHAR_BIT * sizeof(double) == 64);

                m_hdlr = MavHandler::getInstance();

                m_masterGcsIdx = 0;

                try {
                    m_senderRecord = new int[m_hdlr->getMngListSize()];
                    memset( m_senderRecord, 0, 
                            m_hdlr->getMngListSize() * sizeof(int) );
                } catch(std::bad_alloc& m) {
                    std::cerr << "Failed to alloc memory for senderRecord: " 
                            << "at line: " << __LINE__ << ", func: " 
                            << __func__ << ", file: " << __FILE__ 
                            << std::endl;
                    perror( m.what() );
                    exit(EXIT_FAILURE);
                }

                // default #0 GCS send the waypoint cmd to the vehicle
                setMasterGcsIdx(0, m_hdlr->getGeneralSenderIdx(0));

                setMissionRequestListRsp(NULL);
                setMissionRequestRsp(NULL);
                setMissionAckRsp(NULL);
                setMissionCountRsp(NULL);
                setMissionItemRsp(NULL);

                printf("Succeed to construct Waypoint module\n");

            }


            ~MavWaypoint() {
                if(m_senderRecord != NULL) {
                    delete []m_senderRecord;
                    m_senderRecord = NULL;
                }
                m_hdlr = NULL;
                printf("Finish destructing Waypoint module\n");
            }


            static MavWaypoint* m_instance;
            int* m_senderRecord;
            int m_masterGcsIdx;

            MavHandler* m_hdlr;
            WaypointList m_wpl;

            mavlink_message_t m_sendMsg;
            mavlink_mission_request_t m_reqMsg;
            mavlink_mission_count_t m_cntMsg;
            mavlink_mission_ack_t m_ackMsg;
            mavlink_mission_item_t m_itemMsg;
            mavlink_mission_set_current_t m_setCurrMsg;
            mavlink_mission_current_t m_currMsg;

            enum {
                idle, 
                uploading, 
                downloading, 
                loaded, 
                executing, 
                paused, 
                error
            } m_status;

            void (*m_missionRequestListRsp)();
            void (*m_missionRequestRsp)(uint16_t);
            void (*m_missionAckRsp)();
            void (*m_missionCountRsp)(uint16_t);
            void (*m_missionItemRsp)(uint16_t);
            void (*m_missionClearAllRsp)();
            void (*m_missionSetCurrentRsp)(uint16_t);
            void (*m_targetRsp)(const float[][7], uint16_t, uint16_t);


    };

    MavWaypoint* MavWaypoint::m_instance = NULL;

} //namespace dji2mav


#endif
