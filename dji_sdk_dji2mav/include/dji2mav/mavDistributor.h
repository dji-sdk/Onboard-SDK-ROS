/*****************************************************************************
 * @Brief     Distribute received msg to modules. ROS-free singleton
 * @Version   1.1
 * @Author    Chris Liu
 * @Created   2015/11/17
 * @Modified  2015/11/17
 *****************************************************************************/

#ifndef _DJI2MAV_DISTRIBUTOR_H_
#define _DJI2MAV_DISTRIBUTOR_H_


#include "mavHandler.h"
#include "modules/heartbeat/mavHeartbeat.h"

#include <iostream>
#include <stdio.h>
#include <new>

namespace dji2mav{

    class MavDistributor{
        public:
            /**
             * @brief   Get the only instance. Lazy mode singleton
             * @return  The only instance of MavDistributor
             * @warning UNSAFE FOR MULTI-THREAD! Should be called BEFORE fork
             */
            static MavDistributor* getInstance() {
                if(NULL == m_instance) {
                    try {
                        m_instance = new MavDistributor();
                    } catch(std::bad_alloc &m) {
                        std::cerr << "New instance of MavDistributor fail: " 
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
             * @brief  Set GCS number and alloc memory for lists
             * @param  gcsNum : The number of GCS
             * @return True if succeed or false if fail
             */
            bool setDistributorConf(uint16_t gcsNum) {
                if(0 != m_listSize) {
                    printf("The recv lists size has been set before!\n");
                    return false;
                }

                m_listSize = gcsNum;

                try {
                    m_recvMsgList = new mavlink_message_t[m_listSize];
                    m_recvStatusList = new mavlink_status_t[m_listSize];
                    memset(m_recvMsgList, 0, 
                            m_listSize * sizeof(mavlink_message_t));
                    memset(m_recvStatusList, 0, 
                            m_listSize * sizeof(mavlink_status_t));
                } catch(std::bad_alloc& m) {
                    std::cerr << "Failed to alloc memory for recv lists: " 
                            << "at line: " << __LINE__ << ", func: " 
                            << __func__ << ", file: " << __FILE__ 
                            << std::endl;
                    perror( m.what() );
                    exit(EXIT_FAILURE);
                }

                startModules();

                return true;

            }


            /**
             * @brief    Initialize all the modules and get their instance
             * @warnning Should be sed AFTER the handler is setup!
             */
            inline void startModules() {
                  m_moduleHb = MavHeartbeat::getInstance();
            }


            /**
             * @brief  Distribute received message to specific GCS
             * @param  gcsIdx : The index of GCS
             * @return True if succeed or false if fail
             */
            bool distribute(uint16_t gcsIdx) {
                bool ret = m_hdlr->receive(gcsIdx, m_recvMsgList[gcsIdx], 
                        m_recvStatusList[gcsIdx]);
                if(ret) {
                    switch(m_recvMsgList[gcsIdx].msgid) {
                        case MAVLINK_MSG_ID_HEARTBEAT:
                            m_moduleHb->reactToHeartbeat(gcsIdx, 
                                    m_recvMsgList[gcsIdx]);
                            break;
                        default:
                            printf("Undistributed message with msgid %u!\n", 
                                    m_recvMsgList[gcsIdx].msgid);
                    }
                }
                return ret;
            }


            /**
             * @brief  Distribute received message to all GCS
             * @return True if succeed or false if fail
             */
            inline bool distribute() {
                bool ret = true;
                for(uint16_t i = 0; i < m_listSize; ++i)
                    ret &= distribute(i);
                return ret;
            }


        private:
            MavDistributor() {
                m_listSize = 0;
                m_hdlr = MavHandler::getInstance();
            }


            ~MavDistributor() {
                if(NULL != m_recvMsgList) {
                    delete []m_recvMsgList;
                    m_recvMsgList = NULL;
                }
                if(NULL != m_recvStatusList) {
                    delete []m_recvStatusList;
                    m_recvStatusList = NULL;
                }
            }


            static MavDistributor* m_instance;

            mavlink_message_t* m_recvMsgList;
            mavlink_status_t* m_recvStatusList;
            uint16_t m_listSize;

            MavHandler* m_hdlr;

            MavHeartbeat* m_moduleHb;

    };

    MavDistributor* MavDistributor::m_instance = NULL;

} //namespace dji2mav


#endif
