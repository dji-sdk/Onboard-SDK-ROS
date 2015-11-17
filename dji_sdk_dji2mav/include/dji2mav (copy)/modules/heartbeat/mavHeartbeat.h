/*****************************************************************************
 * @Brief     Heartbeat module. Mav-depended and ROS-free
 * @Version   1.1
 * @Author    Chris Liu
 * @Created   2015/11/16
 * @Modified  2015/11/16
 *****************************************************************************/

#ifndef _DJI2MAV_MAVHEARTBEAT_H_
#define _DJI2MAV_MAVHEARTBEAT_H_


#include "mavHandler.h"

#include <iostream>
#include <stdio.h>
#include <new>

namespace dji2mav {

    class MavHeartbeat {
        public:
            /**
             * @brief   Get the only instance. Lazy mode singleton
             * @return  The only instance of MavHeartbeat
             * @warning UNSAFE FOR MULTI-THREAD! Should be called BEFORE fork
             */
            static MavHeartbeat* getInstance() {
                if(NULL == m_instance) {
                    try {
                        m_instance = new MavHeartbeat();
                    } catch(std::bad_alloc &m) {
                        std::cerr << "New instance of MavHeartbeat fail: " 
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
                if( !m_ctnr->isValidIdx(gcsIdx, senderIdx) )
                    return false;
                m_senderRecord[gcsIdx] = senderIdx;
                return true;
            }


            /**
             * @brief  Register new sender and use it for specific GCS
             * @param  gcsIdx : The index of GCS
             * @return True if succeed or false if fail
             */
            bool applyNewSender(uint16_t gcsIdx) {
                int newSender = m_hdlr->registerSender(gcsIdx);
                if( newSender < 0 ) {
                    printf("Fail to regiser a sender for GCS #%u! "
                            "Did you sent sender list size 0 or 1?\n", i);
                    return false;
                }
                m_senderRecord[gcsIdx] = newSender;
                return true;
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

                m_mavHb.type = mavType;
                m_mavHb.autopilot = mavAutopilot;
                m_mavHb.base_mode = mavMode;
                m_mavHb.custom_mode = customMode;
                m_mavHb.system_status = mavStatus;
                m_mavHb.mavlink_version = mavVersion;

            }


            /**
             * @brief  Set type
             * @param  mavType : Default quadrotor type
             */
            inline void setType(uint8_t mavType) {
                m_mavHb.type = mavType;
            }


            /**
             * @brief  Set autopilot
             * @param  mavAutopilot : Default full supporting for everything
             */
            inline void setAutopilot(uint8_t mavAutopilot) {
                m_mavHb.autopilot = mavAutopilot;
            }


            /**
             * @brief  Set base mode
             * @param  mavMode : Default autocontrol and disarmed
             */
            inline void setBaseMode(uint8_t mavMode) {
                m_mavHb.base_mode = mavMode;
            }


            /**
             * @brief  Set custom mode
             * @param  customMode : Default 0. Defined by user
             */
            inline void setCustomMode(uint32_t customMode) {
                m_mavHb.custom_mode = customMode;
            }


            /**
             * @brief  Set status
             * @param  mavStatus : Default vehicle grounded and standby
             */
            inline void setSystemStatus(uint8_t mavStatus) {
                m_mavHb.system_status = mavStatus;
            }


            /**
             * @brief  Set mavlink version
             * @param  mavVersion : Default 3
             */
            inline void setMavlinkVersion(uint8_t mavVersion) {
                m_mavHb.mavlink_version = mavVersion;
            }


            /**
             * @brief  Send heartbeat to specific GCS
             * @param  gcsIdx : The index of GCS
             * @return True if succeed or false if fail
             */
            bool sendHeartbeat(uint16_t gcsIdx) {
                mavlink_msg_heartbeat_encode(m_hdlr->getSysid(), 
                        MAV_COPM_ID_ALL, &m_msg, &m_mavHb);

                if( m_hdlr->sendEncodedMsg(gcsIdx, m_senderRecord[gcsIdx], 
                        &m_msg) ) {
                    return true;
                } else {
                    printf("Sending heartbeat to GCS #%u fail!\n", gcsIdx);
                    return false;
                }
            }


            /**
             * @brief  Send heartbeat to all GCS
             * @return True if succeed or false if fail
             */
            inline bool sendHeartbeat() {
                bool ret = true;
                for(uint16_t i = 0; i < m_hdlr->getMngListSize(); ++i) {
                    ret &= sendHeatbeat(i);
                }
                return ret;
            }


        private:
            MavContainer() {

                m_ctnr = MavContainer::getInstance();

                try {
                    m_senderRecord = new int[m_hdlr->getMngListSize()];
                } catch(std::bad_alloc& m) {
                    std::cerr << "Failed to alloc memory for senderRecord: " 
                            << "at line: " << __LINE__ << ", func: " 
                            << __func__ << ", file: " << __FILE__ 
                            << std::endl;
                    perror( m.what() );
                    exit(EXIT_FAILURE);
                }
                for(uint16_t i = 0; i < m_hdlr->getMngListSize(); ++i) {
                    // register new sender for heartbeat in every GCS
                    applyNewSender(i);
                }

                setHeartbeatData(MAV_TYPE_QUADROTOR, MAV_AUTOPILOT_GENERIC, 
                        MAV_MODE_GUIDED_DISARMED, 0, MAV_STATE_STANDBY, 3);

            }


            ~MavContainer() {
            }


            static MavHeartbeat* m_instance;
            MavHandler* m_hdlr;
            int* m_senderRecord;

            mavlink_message_t m_msg;
            mavlink_heartbeat_t m_mavHb;

    };

} //namespace dji2mav


#endif
