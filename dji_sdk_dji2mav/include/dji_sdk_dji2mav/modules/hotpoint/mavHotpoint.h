/*****************************************************************************
 * @Brief     Hotpoint module. Implement hotpoint protocol here 
 * @Version   0.3.0
 * @Author    Chris Liu
 * @Created   2015/12/11
 * @Modified  2015/12/25
 *****************************************************************************/

#ifndef _DJI2MAV_MAVHOTPOINT_H_
#define _DJI2MAV_MAVHOTPOINT_H_


#include <mavlink/common/mavlink.h>
#include <new>
#include <string>
#include <cstdarg>

#include "dji_sdk_dji2mav/modules/mavModule.h"
#include "hotpointData.h"
#include "dji_sdk_dji2mav/log.h"

namespace dji2mav{

    class MavHotpoint : public MavModule {
        public:
            /**
             * @brief Constructor for Hotpoint Module. Recv buf size 4096
             * @param handler : The reference of MavHandler Object
             * @param name    : The name of this module
             * @param gcsNum  : The number of GCS that the module employs
             * @param ...     : The indexes of GCS list, should be uint16_t
             */
            MavHotpoint(MavHandler &handler, std::string name, 
                    uint16_t gcsNum, ...) : MavModule(handler, name, 4096) {

                DJI2MAV_DEBUG("Going to construct Hotpoint module with name " 
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

                //TODO: Set the MOI here

                DJI2MAV_DEBUG("...finish constructing Hotpoint module.");

            }


            ~MavHotpoint() {
                DJI2MAV_DEBUG("Going to destruct Hotpoint module...");
                DJI2MAV_DEBUG("...finish destructing Hotpoint module.");
            }


            /**
             * @brief Implement the messages passive handling function
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
                    DJI2MAV_DEBUG("In Hotpoint the compid is %u.", 
                            recvMsg.compid);
                }

                DJI2MAV_DEBUG("In Hotpoint mission request list with status: " 
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
                cntMsg.count = 1;//one single unit in the list

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
                    DJI2MAV_DEBUG("In Hotpoint the compid is %u.", 
                            recvMsg.compid);
                }

                DJI2MAV_DEBUG("In Hotpoint mission request with status: %d.", 
                        (int)m_status);

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
                if(reqMsg.seq == 0) {
                    itemMsg.target_system = recvMsg.sysid;
                    itemMsg.target_component = recvMsg.compid;
                    itemMsg.seq = reqMsg.seq;
                    m_hp.getHotpointData(itemMsg.seq, itemMsg.command, 
                            itemMsg.param1, itemMsg.param2, itemMsg.param3, 
                            itemMsg.param4, itemMsg.x, itemMsg.y, itemMsg.z);
                } else {
                    DJI2MAV_ERROR("Invalid sequence %u of mission request in " 
                            "Hotpoint!", reqMsg.seq);
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
                    DJI2MAV_DEBUG("In Hotpoint the compid is %u.", 
                            recvMsg.compid);
                }

                DJI2MAV_DEBUG("In Hotpoint mission ack with status: %d.", 
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
                DJI2MAV_DEBUG("In Hotpoint mission ACK code: %d.", ackMsg.type);
                m_status = loaded;
                m_hp.display();

                if(NULL != m_missionAckHook)
                    m_missionAckHook();

            }


            void reactToMissionCount(uint16_t gcsIdx, 
                    const mavlink_message_t &recvMsg) {

                if(recvMsg.compid != MAV_COMP_ID_MISSIONPLANNER) {
                    DJI2MAV_DEBUG("In Hotpoint the compid is %u.", 
                            recvMsg.compid);
                }

                DJI2MAV_DEBUG("In Hotpoint mission count with status: %d.", 
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
                    DJI2MAV_DEBUG("In Hotpoint the compid is %u.", 
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
                if(itemMsg.seq == 0) {

                    m_hp.setHotpointData(itemMsg.seq, itemMsg.command, 
                            itemMsg.param1, itemMsg.param2, itemMsg.param3, 
                            itemMsg.param4, itemMsg.x, itemMsg.y, itemMsg.z);

                } else {
                    DJI2MAV_ERROR("Invalid sequence %u of mission item in " 
                            "Hotpoint!", itemMsg.seq);
                    return;
                }

                mavlink_message_t sendMsg;
                mavlink_msg_mission_ack_pack(getMySysid(), 
                        MAV_COMP_ID_MISSIONPLANNER, &sendMsg, 
                        recvMsg.sysid, recvMsg.compid, 
                        MAV_MISSION_ACCEPTED);
                sendMsgToMaster(sendMsg);
                m_status = loaded;
                m_hp.display();

                if(NULL != m_missionItemHook)
                    m_missionItemHook(itemMsg.seq);

            }


            void reactToMissionClearAll(uint16_t gcsIdx, 
                    const mavlink_message_t &recvMsg) {

                if(recvMsg.compid != MAV_COMP_ID_MISSIONPLANNER) {
                    DJI2MAV_DEBUG("In Hotpoint the compid is %u.", 
                            recvMsg.compid);
                }

                DJI2MAV_DEBUG("In Hotpoint mission clear all with status: %d.", 
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

                m_hp.clear();

                if(NULL != m_missionClearAllHook)
                    m_missionClearAllHook();

            }


            void reactToMissionSetCurrent(uint16_t gcsIdx, 
                    const mavlink_message_t &recvMsg) {

                if(recvMsg.compid != MAV_COMP_ID_MISSIONPLANNER) {
                    DJI2MAV_DEBUG("In Hotpoint the compid is %u.", 
                            recvMsg.compid);
                }

                DJI2MAV_DEBUG("In Hotpoint mission set current with status: " 
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
                if(0 == setCurrMsg.seq) {

                    if(NULL != m_targetHook) {
                        m_targetHook( m_hp.getHotpoint(), 7, m_hp.getCmd() );
                        DJI2MAV_DEBUG("Finish running target hook.");
                        //It is slow to send mission to FC. Now there should be
                        //duplicate cmd msg in the buffer. Clear them
                        clearBuf();
                    } else {
                        DJI2MAV_WARN("In Hotpoint no target hook is set.");
                    }

                    mavlink_message_t sendMsg;
                    mavlink_msg_mission_current_pack( getMySysid(), 
                            MAV_COMP_ID_MISSIONPLANNER, &sendMsg, 0 );
                    sendMsgToMaster(sendMsg);

                    m_status = loaded;

                } else {
                    m_status = idle;
                    DJI2MAV_ERROR("The sequence %u of set current message is " 
                            "invalid in Hotpoint!", setCurrMsg.seq);
                }

                if(NULL != m_missionSetCurrentHook)
                    m_missionSetCurrentHook(setCurrMsg.seq);

            }


            void setMissionRequestListHook(void (*func)()) {
                m_missionRequestListHook = func;
            }


            void setMissionRequestHook(void (*func)(uint16_t)) {
                m_missionRequestHook = func;
            }


            void setMissionAckHook(void (*func)()) {
                m_missionAckHook = func;
            }


            void setMissionCountHook(void (*func)(uint16_t)) {
                m_missionCountHook = func;
            }


            void setMissionItemHook(void (*func)(uint16_t)) {
                m_missionItemHook = func;
            }


            void setMissionClearAllHook(void (*func)()) {
                m_missionClearAllHook = func;
            }


            void setMissionSetCurrentHook(void (*func)(uint16_t)) {
                m_missionSetCurrentHook = func;
            }


            void setTargetHook(void (*func)(const float[], uint16_t, uint16_t)) {
                m_targetHook = func;
            }



        private:
            HotpointData m_hp;

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
            void (*m_targetHook)(const float[], uint16_t, uint16_t);


    };

} //namespace dji2mav


#endif
