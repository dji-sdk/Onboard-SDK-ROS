/*****************************************************************************
 * @Brief     Base class of dji2mav module. ROS-free and mav-depended
 * @Version   0.3.0
 * @Author    Chris Liu
 * @Created   2015/12/06
 * @Modified  2015/12/24
 *****************************************************************************/

#ifndef _MAV2DJI_MAVMODULE_H_
#define _MAV2DJI_MAVMODULE_H_


#include <mavlink/common/mavlink.h>
#include <new>
#include <string>
#include <pthread.h>

#include "moduleBuf.h"
#include "dji_sdk_dji2mav/mavHandler.h"
#include "dji_sdk_dji2mav/log.h"

namespace dji2mav{

    class MavModule {

        public:
            /**
             * @brief Constructor for base MavModule class
             * @param handler : The reference of MavHandler Object
             * @param name    : The string name of this module
             * @param buf     : The buffer size of received message
             */
            MavModule(MavHandler &handler, std::string name, uint16_t bufSize) {
                m_hdlr = &handler;
                m_masterGcsIdx = -1;
                m_name = name;

                try {
                    m_senderRecord = new int[m_hdlr->getMaxListSize()];
                    //set all sender record to -1
                    memset( m_senderRecord, 0xFF, 
                            m_hdlr->getMaxListSize() * sizeof(int) );
                    m_moduleBuf = new ModuleBuf(bufSize);
                } catch(std::bad_alloc& m) {
                    DJI2MAV_FATAL( "Failed to allocate memory for mavModule! " 
                            "Exception: %s!", m.what() );
                    exit(EXIT_FAILURE);
                }

                DJI2MAV_DEBUG("Construct base MavModule.");
            }


            virtual ~MavModule() {
                DJI2MAV_DEBUG("Going to destruct base MavModule...");
                if(NULL != m_senderRecord) {
                    delete []m_senderRecord;
                    m_senderRecord = NULL;
                }
                if(NULL != m_moduleBuf)
                    delete m_moduleBuf;
                m_hdlr = NULL;
                DJI2MAV_DEBUG("...finish destructing base Mavmodule.");
            }


            /**
             * @brief  Get the name of module
             * @return The name of module
             */
            inline std::string getName() {
                return m_name;
            }


            /**
             * @brief  Get the sysid of this device
             * @return The sysid of this device
             */
            inline uint8_t getMySysid() {
                return m_hdlr->getMySysid();
            }


            /**
             * @brief  Get the index of master GCS
             * @return The index of master GCS
             */
            inline int getMasterGcsIdx() {
                return m_masterGcsIdx;
            }


            /**
             * @brief  Get the sender index of master GCS
             * @return SenderIdx of master, -2 for no master or -1 for no sender
             */
            int getMasterGcsSenderIdx() {
                if(-1 == m_masterGcsIdx)
                    return -2;
                else
                    return m_senderRecord[m_masterGcsIdx];
            }


            /**
             * @brief  Set the master GCS with its index
             * @param  gcsIdx    : The index of GCS that is to be set
             * @return True if succeed or false if fail
             */
            bool setMasterGcsIdx(uint16_t gcsIdx) {
                if( !activateSender(gcsIdx) ) {
                    DJI2MAV_ERROR("Fail to set master GCS to #%u because it " 
                            "is not activated", gcsIdx);
                    return false;
                }
                if(-1 != m_masterGcsIdx) {
                    DJI2MAV_INFO("Switch master GCS from #%u to #%u.", 
                            m_masterGcsIdx, gcsIdx);
                }
                //at last, turn on master mode
                m_masterGcsIdx = gcsIdx;
                return true;
            }


            /**
             * @brief  Get the senderRecord array address
             * @return The address of senderRecord
             */
            inline const int* getSenderRecord() {
                return m_senderRecord;
            }


            /**
             * @brief  Get sender index for specific GCS
             * @param  gcsIdx : The index of GCS
             * @return The sender index of the GCS or -2 for invalid input
             */
            int getGcsSenderIdx(uint16_t gcsIdx) {
                if( !m_hdlr->isValidMngIdx(gcsIdx) )
                    return -2;
                return m_senderRecord[gcsIdx];
            }


            /**
             * @brief  Employ a sender for the specific GCS
             * @param  gcsIdx : The index of GCS
             * @return True if succeed or false if fail
             */
            bool employGcsSender(uint16_t gcsIdx) {
                if( !activateSender(gcsIdx) ) {
                    DJI2MAV_ERROR("Fail to employ a sender for GCS #%u!", gcsIdx);
                    return false;
                }
                //turn off master mode
                m_masterGcsIdx = -1;
                DJI2MAV_DEBUG("Succeed in employing sender for GCS #%u.", gcsIdx);
                return true;
            }


            /**
             * @brief  Activate sender of the specific GCS
             * @param  gcsIdx : The index of GCS
             * @return True if succeed or false if fail
             */
            bool activateSender(uint16_t gcsIdx) {
                if( !m_hdlr->isValidMngIdx(gcsIdx) ) {
                    DJI2MAV_ERROR("Fail to activate sender for invalid GCS " 
                            "index %u!", gcsIdx);
                    return false;
                }
                int sender = m_senderRecord[gcsIdx];
                if( -1 == sender ) {
                    sender = m_hdlr->registerSender(gcsIdx);
                    if(0 > sender) {
                        DJI2MAV_ERROR("Fail to register sender for GCS #%u!", 
                                gcsIdx);
                        return false;
                    }
                    m_senderRecord[gcsIdx] = sender;
                }
                DJI2MAV_DEBUG("Succeed in activating sender with index %u for " 
                        "GCS #%u.", m_senderRecord[gcsIdx], gcsIdx);
                return true;
            }


            /**
             * @brief  Send message to specific GCS
             * @param  gcsIdx : The index of specific GCS
             * @param  msg    : The reference of encoded mavlink message
             * @return True if succeed or false if fail
             */
            bool sendMsgToGcs(uint16_t gcsIdx, mavlink_message_t& msg) {
                if(0 > m_senderRecord[gcsIdx]) {
                    DJI2MAV_ERROR("Fail to send message to GCS #%u because no " 
                            "sender is employed for it!", gcsIdx);
                    return false;
                }
                bool ret = m_hdlr->sendEncodedMsgToMng(gcsIdx, 
                        (uint16_t)m_senderRecord[gcsIdx], msg);
                if(ret) {
                    DJI2MAV_TRACE("Succeed in sending message to GCS #%u " 
                            "using sender with index %u.", gcsIdx, 
                            m_senderRecord[gcsIdx]);
                } else {
                    DJI2MAV_ERROR("Fail to send message to GCS #%u!", gcsIdx);
                }
                return ret;
            }


            /**
             * @brief  Send message to corresponding GCS with sysid
             * @param  sysid : The sysid of target device
             * @param  msg   : The reference of encoded mavlink message
             * @return True if succeed or false if fail
             */
            bool sendMsgToSys(uint8_t sysid, mavlink_message_t& msg) {
                int idx = m_hdlr->findMngIdx(sysid);
                if( idx <= 0 || idx > m_hdlr->getMaxListSize() ) {
                    DJI2MAV_ERROR("Fail to send message to GCS because no " 
                            "valid index is matched sysid %u! Return value: " 
                            "%d!", sysid, idx);
                    return false;
                }
                //TODO: Cannot call sendEncodedMsgToSys for passing senderIdx
                return sendMsgToGcs( (uint8_t)idx, msg);
            }


            /**
             * @brief  Send message to master GCS
             * @param  msg : The reference of encoded mavlink message
             * @return True if succeed or false if fail
             */
            inline bool sendMsgToMaster(mavlink_message_t& msg) {
                return sendMsgToGcs(m_masterGcsIdx, msg);
            }


            /**
             * @brief  Send message to all activated GCS
             * @param  msg : The reference of encoded mavlink message
             * @return True if succeed or false if fail
             */
            bool sendMsgToAll(mavlink_message_t& msg) {
                bool ret = true;
                if(-1 != m_masterGcsIdx) {
                    return sendMsgToMaster(msg);
                } else {
                    bool ret = true;
                    for(uint16_t i = 0; i < m_hdlr->getMaxListSize(); ++i) {
                        if( -1 != m_senderRecord[i] && !sendMsgToGcs(i, msg) ) {
                            DJI2MAV_ERROR("Fail to send message to GCS #%u!", i);
                            ret = false;
                        }
                    }
                    if(!ret)
                        DJI2MAV_ERROR("Fail to send the message to some GCS!");
                    return ret;
                }
            }


            /**
             * @brief  Push mavlink message to the buffer
             * @param  srcMsg : The source of message
             * @return True if succeed or false if fail
             */
            inline bool pushMsg(uint16_t gcsIdx, 
                    const mavlink_message_t &srcMsg) {

                return m_moduleBuf->writeBuf( (uint8_t*)&srcMsg, 
                        sizeof(mavlink_message_t) );

            }


            /**
             * @brief  Pull mavlink message from the buffer
             * @param  destMsg : The destination of message
             * @return True if succeed or false if fail
             */
            inline bool pullMsg(mavlink_message_t &destMsg) {

                return m_moduleBuf->readBuf( (uint8_t*)&destMsg, 
                        sizeof(mavlink_message_t) );

            }


            /**
             * @brief Clear buffer content
             */
            inline void clearBuf() {
                m_moduleBuf->clear();
            }


            /**
             * @brief  Run a thread for the module
             * @return True if succeed to create the thread or false if fail
             */
            bool run() {
                int ret = pthread_create( &m_tid, NULL, thread_call, 
                        (void*)this );
                if(0 != ret) {
                    printf("Fail to create thread for the module.\n");
                    return false;
                }
                return true;
            }


            /**
             * @brief A thread calling function for the module
             * @param param : The pointer to the module object
             */
            static void* thread_call(void* param) {
                mavlink_message_t msg;
                while(true) {
                    //call passivelyReceive(msg) inside helper
                    ( (MavModule*)param )->receiveHelper(msg);

                    ( (MavModule*)param )->activelySend();
                }
            }


            /**
             * @brief The helper of thread passively reveive
             * @param msg : A reference of message type that is used to recv
             */
            void receiveHelper(mavlink_message_t &msg) {
                if( pullMsg(msg) ) {
                    if( -1 != m_masterGcsIdx && msg.sysid != 
                            m_hdlr->findMngSysid(m_masterGcsIdx) ) {
                        return;
                    }
                    passivelyReceive(msg);
                }
            }


            /**
             * @brief Handle the received messages process for the module
             * @param msg : The reference of received message
             */
            virtual void passivelyReceive(mavlink_message_t &msg) = 0;


            /**
             * @brief Handle the messages sending process for the module
             */
            virtual void activelySend() = 0;


        private:
            int* m_senderRecord;
            ModuleBuf* m_moduleBuf;
            int m_masterGcsIdx;
            uint8_t m_masterSysid;

            MavHandler* m_hdlr;

            pthread_t m_tid;
            std::string m_name;

    };

} //namespace dji2mav


#endif
