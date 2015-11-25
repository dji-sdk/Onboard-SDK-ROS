/*****************************************************************************
 * @Brief     Handle mng list. A mng is related to a GCS. ROS-free singleton
 * @Version   0.2.1
 * @Author    Chris Liu
 * @Created   2015/10/31
 * @Modified  2015/11/16
 *****************************************************************************/

#ifndef _DJI2MAV_MAVHANDLER_H_
#define _DJI2MAV_MAVHANDLER_H_


#include "msgManager.h"

#include <mavlink/v1.0/common/mavlink.h>
#include <stdio.h>
#include <iostream>
#include <new>
#include <string>

namespace dji2mav {

    class MavHandler {
        public:
            /**
             * @brief   Get the only instance. Lazy mode singleton
             * @return  The only instance of MavHandler
             * @warning UNSAFE FOR MULTI-THREAD! Should be called BEFORE fork
             */
            static MavHandler* getInstance() {
                if(NULL == m_instance) {
                    try {
                        m_instance = new MavHandler();
                    } catch(std::bad_alloc &m) {
                        std::cerr << "New instance of MavHandler fail: " 
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
             * @brief  Check whether the mng index is valid
             * @param  mngIdx : The index of mng that is to be checked
             * @return True if valid or false if invalid
             */
            inline bool isValidMngIdx(uint16_t mngIdx) {
                if(mngIdx >= m_mngListSize) {
                    printf("The mngIdx %u exceeds the mng list size %u!\n", 
                            mngIdx, m_mngListSize);
                    return false;
                }

                if(NULL == m_mngList[mngIdx]) {
                    printf("Mng #%u haven't been setup yet!\n", mngIdx);
                    return false;
                }

                return true;
            }


            /**
             * @brief  Check whether the mng and sender indexes are valid
             * @param mngIdx    : The MsgManager index that is to be checked
             * @param senderIdx : The sender index that is to be checked
             * @return True if valid or false if invalid
             */
            inline bool isValidIdx(uint16_t mngIdx, uint16_t senderIdx) {
                return ( isValidMngIdx(mngIdx)
                        && (m_mngList[mngIdx]->getCurrSenderListSize() 
                        > senderIdx ? true : false) );
            }


            /**
             * @brief  Register a messager sender with specifc buf size
             * @param  mngIdx  : The index the MsgManager
             * @param  bufSize : The size of sender buf that is to be allocated
             * @return Index of the sender. Return -1 if the list is full
             */
            inline int registerSender(uint16_t mngIdx, uint16_t bufSize) {
                return m_mngList[mngIdx]->registerSender(bufSize);
            }


            /**
             * @brief  Register a messager sender with default buf size
             * @param  mngIdx : The index the MsgManager
             * @return Index of the sender. Return -1 if the list is full
             */
            inline int registerSender(uint16_t mngIdx) {
                return m_mngList[mngIdx]->registerSender();
            }


            /**
             * @brief  Get general sender index
             * @param  mngIdx : The index the MsgManager
             * @return The index of general sender
             */
            inline uint16_t getGeneralSenderIdx(uint16_t mngIdx) {
                return m_mngList[mngIdx]->getGeneralSenderIdx();
            }


            /**
             * @brief  Set system id and mng list size
             * @param  sysid       : The system id of this vehicle
             * @param  mngListSize : The size of MsgManager list
             * @return True if succeed or false if fail
             */
            bool setHandlerConf(uint8_t sysid, uint16_t mngListSize) {

                if(0 != m_mngListSize) {
                    printf("The mng list size has been set before!\n");
                    return false;
                }

                m_sysid = sysid;
                m_mngListSize = mngListSize;

                try {
                    m_mngList = new MsgManager*[m_mngListSize];
                    memset(m_mngList, 0, m_mngListSize * sizeof(MsgManager*));
                } catch(std::bad_alloc& m) {
                    std::cerr << "Failed to alloc memory for mng list: " 
                            << "at line: " << __LINE__ << ", func: " 
                            << __func__ << ", file: " << __FILE__ 
                            << std::endl;
                    perror( m.what() );
                    exit(EXIT_FAILURE);
                }

                printf("Succeed to setup Handler\n");
                return true;

            }


            /**
             * @brief  Set the configurations of MavHandler
             * @param  mngIdx            : The index of mng that is to be set
             * @param  senderListSize : The size of sender list
             * @param  sendBufSize    : The default value of send buf size
             * @param  recvBufSize    : The recv buf size
             * @return True if succeed or false if fail
             */
            bool setMngConf(uint16_t mngIdx, uint16_t senderListSize, 
                    uint16_t sendBufSize, uint16_t recvBufSize) {

                if(mngIdx >= m_mngListSize) {
                    printf("The index %u exceeds the mng list size %u.", 
                            mngIdx, m_mngListSize);
                    return false;
                }

                if(NULL != m_mngList[mngIdx]) {
                    printf("The mng of index %u has been set before!\n", 
                            mngIdx);
                    return false;
                }

                try {
                    m_mngList[mngIdx] = new MsgManager(senderListSize, 
                            sendBufSize, recvBufSize);
                } catch(std::bad_alloc& m) {
                    std::cerr << "Failed to alloc memory for mng " 
                            << mngIdx << ": at line: " << __LINE__ 
                            << ", func: " << __func__ << ", file: " 
                            << __FILE__ << std::endl;
                    perror( m.what() );
                    exit(EXIT_FAILURE);
                }

                return true;

            }


            /**
             * @brief  Get the size of mngList(aka, GCS number)
             * @return The size of mngList(aka, GCS number)
             */
            inline uint16_t getMngListSize() {
                return m_mngListSize;
            }


            /**
             * @brief  Establish UDP connection of the mng
             * @param  mngIdx     : The index of mng
             * @param  targetIP   : The IP of target
             * @param  targetPort : The connection port of target
             * @param  srcPort    : The connection port of source
             * @return True if succeed or false if fail
             */
            bool establish(uint16_t mngIdx, std::string targetIP, 
                    uint16_t targetPort, uint16_t srcPort) {

                if( !isValidMngIdx(mngIdx) )
                    return false;

                return m_mngList[mngIdx]->establish(targetIP, targetPort, 
                        srcPort);

            }


            /**
             * @brief  Get the system id of this vehicle
             * @return The system id of this vehicle
             */
            inline uint8_t getSysid() {
                return m_sysid;
            }


            /**
             * @brief  Send msg to specific mng using specific sender
             * @param  mngIdx    : The index of Ground Control Station
             * @param  senderIdx : The index of sender
             * @param  msg_p     : The pointer of msg that should be sent
             * @return True if succeed and false if fail
             */
            bool sendEncodedMsg(uint16_t mngIdx, uint16_t senderIdx, 
                    const mavlink_message_t *msg_p) {

                if( !isValidMngIdx(mngIdx) )
                    return false;

                uint16_t len = mavlink_msg_to_send_buffer(
                        m_mngList[mngIdx]->getSendBuf(senderIdx), msg_p);
                int bytes_sent = m_mngList[mngIdx]->send(senderIdx, len);
                if(bytes_sent < 0)
                    return false;
                if( (uint16_t)bytes_sent != len ) {
                    std::cout << "For mng " << mngIdx << ", " << bytes_sent 
                            << " bytes were sent. But " << len 
                            << "-byte package was generated!" << std::endl;
                    return false;
                }
                return true;

            }


            /**
             * @brief  Send msg to specific mng using general sender
             * @param  mngIdx    : The index of Ground Control Station
             * @param  msg_p     : The pointer of msg that should be sent
             * @return True if succeed and false if fail
             */
            inline bool sendEncodedMsg(uint16_t mngIdx, 
                    const mavlink_message_t *msg_p) {

                return sendEncodedMsg( mngIdx, 
                        m_mngList[mngIdx]->getGeneralSenderIdx(), msg_p );

            }


            /**
             * @brief  Receive data and parse it from buffer to mavlink
             * @param  mngIdx     : The index of manager
             * @param  recvMsg    : The reference of recv msg variable
             * @pram   recvStatus : The reference of recv status variable
             * @return True if succeed and false if fail
             */
            bool receive(uint16_t mngIdx, mavlink_message_t &recvMsg, 
                    mavlink_status_t &recvStatus) {

                if( !isValidMngIdx(mngIdx) )
                    return false;

                bool ret = false;
                int bytes_recv = m_mngList[mngIdx]->recv();
                if(bytes_recv < 0 && errno != EAGAIN) {
                    printf("Execution of mng #%u receive process fail!\n", 
                            mngIdx);
                    return false;
                }
                if(bytes_recv > 0) {
//                  printf("Datagram: ");
                    uint8_t* bufPtr = m_mngList[mngIdx]->getRecvBuf();
                    for(int i = 0; i < bytes_recv; ++i, ++bufPtr) {
//                      printf("%02x", *bufPtr);
                        if( mavlink_parse_char(MAVLINK_COMM_0, *bufPtr, 
                                &recvMsg, &recvStatus) ) {
                            ret = true;
                        }
                    }
//                  printf("\n");
                }
                return ret;

            }


            /**
             * @brief  Receive data from every mng(aka, GCS)
             * @param  recvMsgList    : The pointer to recv msg list
             * @pram   recvStatusList : The pointer to recv status list
             * @return True if succeed and false if fail
             */
            inline bool receive(mavlink_message_t* recvMsgList, 
                    mavlink_status_t* recvStatusList) {

                bool ret = true;
                for(uint16_t i = 0; i < m_mngListSize; ++i) {
                    ret &= receive(i, recvMsgList[i], recvStatusList[i]);
                }
                return ret;

            }


            void distructor() {
                delete m_instance;
            }


        private:
            MavHandler() {
                m_mngListSize = 0;
            }


            ~MavHandler() {
                printf("Going to destruct Handler...\n");

                if(NULL != m_mngList) {
                    for(uint16_t i = 0; i < m_mngListSize; ++i) {
                        if(NULL != m_mngList[i]) {
                            delete m_mngList[i];
                            m_mngList[i] = NULL;
                        }
                    }
                    delete []m_mngList;
                    m_mngList = NULL;
                }

                printf("Finish destructing Handler\n");
            }


            static MavHandler* m_instance;
            MsgManager** m_mngList;
            uint16_t m_mngListSize;

            uint8_t m_sysid;
    };

    MavHandler* MavHandler::m_instance = NULL;

} //namespace dji2mav


#endif
