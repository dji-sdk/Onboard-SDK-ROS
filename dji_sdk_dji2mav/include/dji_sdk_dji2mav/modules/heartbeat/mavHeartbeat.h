/*****************************************************************************
 * @Brief     Heartbeat module. Mav-depended and ROS-free
 * @Version   0.3.0
 * @Author    Chris Liu
 * @Created   2015/11/16
 * @Modified  2015/12/25
 *****************************************************************************/

#ifndef _DJI2MAV_MAVHEARTBEAT_H_
#define _DJI2MAV_MAVHEARTBEAT_H_


#include <mavlink/common/mavlink.h>
#include <new>
#include <string>
#include <cstdarg>

#include "dji_sdk_dji2mav/modules/mavModule.h"
#include "dji_sdk_dji2mav/log.h"

namespace dji2mav {

    class MavHeartbeat : public MavModule {
        public:
            /**
             * @brief Constructor for Heartbeat Module. Recv buf size 4096
             * @param handler : The reference of MavHandler Object
             * @param name    : The name of this module
             * @param gcsNum  : The number of GCS that the module employs
             * @param ...     : The indexes of GCS list, should be uint16_t
             */
            MavHeartbeat(MavHandler &handler, std::string name, 
                    uint16_t gcsNum, ...) : MavModule(handler, name, 4096) {

                DJI2MAV_DEBUG("Going to construct Heartbeat module with name " 
                        "%s and gcsNum %u...", name.c_str(), gcsNum);

                setHeartbeatData(MAV_TYPE_QUADROTOR, MAV_AUTOPILOT_GENERIC, 
                        MAV_MODE_GUIDED_DISARMED, 0, MAV_STATE_STANDBY, 3);

                setHeartbeatHook(NULL);

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

                DJI2MAV_DEBUG("...finish constructing Heartbeat module.");

            }


            ~MavHeartbeat() {
                DJI2MAV_DEBUG("Going to destruct Heartbeat module...");
                DJI2MAV_DEBUG("...finish destructing Heartbeat module.");
            }


            /**
             * @brief  Set the heartbeat package raw data
             * @param  mavType      : Default quadrotor type
             * @param  mavAutopilot : Default full supporting for every
             * @param  mavMode      : Default autocontrol and disarmed
             * @param  customMode   : Default 0. Defined by user
             * @param  mavStatus    : Default vehicle grounded and standby
             * @param  mavVersion   : Default 3
             */
            inline void setHeartbeatData(uint8_t mavType, 
                    uint8_t mavAutopilot, uint8_t mavMode, 
                    uint32_t customMode, uint8_t mavStatus, 
                    uint8_t mavVersion) {

                m_hbMsg.type = mavType;
                m_hbMsg.autopilot = mavAutopilot;
                m_hbMsg.base_mode = mavMode;
                m_hbMsg.custom_mode = customMode;
                m_hbMsg.system_status = mavStatus;
                m_hbMsg.mavlink_version = mavVersion;

            }


            /**
             * @brief  Set type
             * @param  mavType : Default quadrotor type
             */
            inline void setType(uint8_t mavType) {
                m_hbMsg.type = mavType;
            }


            /**
             * @brief  Set autopilot
             * @param  mavAutopilot : Default full supporting for everything
             */
            inline void setAutopilot(uint8_t mavAutopilot) {
                m_hbMsg.autopilot = mavAutopilot;
            }


            /**
             * @brief  Set base mode
             * @param  mavMode : Default autocontrol and disarmed
             */
            inline void setBaseMode(uint8_t mavMode) {
                m_hbMsg.base_mode = mavMode;
            }


            /**
             * @brief  Set custom mode
             * @param  customMode : Default 0. Defined by user
             */
            inline void setCustomMode(uint32_t customMode) {
                m_hbMsg.custom_mode = customMode;
            }


            /**
             * @brief  Set status
             * @param  mavStatus : Default vehicle grounded and standby
             */
            inline void setSystemStatus(uint8_t mavStatus) {
                m_hbMsg.system_status = mavStatus;
            }


            /**
             * @brief  Set mavlink version
             * @param  mavVersion : Default 3
             */
            inline void setMavlinkVersion(uint8_t mavVersion) {
                m_hbMsg.mavlink_version = mavVersion;
            }


            /**
             * @brief  Send heartbeat to specific GCS. Compid is set to ALL
             * @param  gcsIdx : The index of GCS
             * @return True if succeed or false if fail
             */
            bool sendHeartbeatToGcs(uint16_t gcsIdx) {

                mavlink_msg_heartbeat_encode(getMySysid(), 
                        MAV_COMP_ID_ALL, &m_sendMsg, &m_hbMsg);

                if( sendMsgToGcs(gcsIdx, m_sendMsg) ) {
                    return true;
                } else {
                    DJI2MAV_ERROR("Fail to send Heartbeat to GCS #%u!", gcsIdx);
                    return false;
                }

            }


            /**
             * @brief  Send heartbeat to all GCS
             * @return True if succeed or false if fail
             */
            bool sendHeartbeatToAll() {

                mavlink_msg_heartbeat_encode(getMySysid(),      
                        MAV_COMP_ID_ALL, &m_sendMsg, &m_hbMsg);

                if( sendMsgToAll(m_sendMsg) ) {
                    return true;
                } else {
                    DJI2MAV_ERROR("Fail to send Heartbeat to some GCS!");
                    return false;
                }

            }


            /**
             * @brief  React to heartbeat from specific GCS and execute hook
             * @param  gcsIdx : Get the index of GCS
             * @param  msg    : Get the received message
             */
            void reactToHeartbeat(uint16_t gcsIdx, const mavlink_message_t* msg) {
                if(NULL != m_hook) {
                    m_hook();
                }
            }


            /**
             * @brief Set the responser function pointer for the heartbeat
             * @param The function pointer that is to be set
             */
            inline void setHeartbeatHook( void (*func)() ) {
                m_hook = func;
            }


            /**
             * @brief Implement the messages passively handling function
             * @param msg : The reference of received message
             */
            void passivelyReceive(mavlink_message_t &msg) {
            }


            /**
             * @brief Implement the messages actively sending function
             */
            void activelySend() {
                sendHeartbeatToAll();
                sleep(1); //1Hz
            }


        private:
            mavlink_message_t m_sendMsg;
            mavlink_heartbeat_t m_hbMsg;

            void (*m_hook)();

    };

} //namespace dji2mav


#endif
