/*****************************************************************************
 * @Brief     Manage msg senders and receiver. ROS-free and mav-free singleton
 * @Version   1.0
 * @Author    Chris Liu
 * @Created   2015/10/30
 * @Modified  2015/11/03
 *****************************************************************************/

#ifndef _DJI2MAV_MSGMANAGER_H_
#define _DJI2MAV_MSGMANAGER_H_


#include "msgSender.h"
#include "msgReceiver.h"

#include <new>
#include <stdio.h>
#include <iostream>

#define DEFAULT_SENDER_LIST_SIZE 256

namespace dji2mav{

    class MsgManager {
        public:
            /**
             * @brief   Lazy mode singleton
             * @param   senderListSize : Default 256. Valid on first call
             * @param   recvBufSize    : Default 4096. Valid on first call
             * @return  The only instance of MsgManager
             * @warning UNSAFE FOR MULTI-THREAD!
             */
            static MsgManager* getInstance(uint16_t senderListSize 
                    = DEFAULT_SENDER_LIST_SIZE, uint16_t recvBufSize
                    = DEFAULT_RECV_BUF_SIZE) {

                if(NULL == m_instance) {
                    try {
                        m_instance = new MsgManager(senderListSize, 
                                recvBufSize);
                    } catch(std::bad_alloc &m) {
                        std::cerr << "Cannot new instance of MsgManager: " 
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
             * @brief   Register a message sender and get its index
             * @param   bufSize : Set the buf size of sender. Default 1024
             * @return  Index of the sender. Return -1 if the list is full
             * @warning UNSAFE FOR MULTI-THREAD!
             */
            int registerSender(uint16_t bufSize = DEFAULT_SEND_BUF_SIZE) {
                int idx = m_currListSize;
                if(m_currListSize >= m_maxListSize)
                    return -1;
                try {
                    m_senderList[idx] = new MsgSender(bufSize);
                } catch(std::bad_alloc &m) {
                    std::cerr << "Failed to new MsgSender object: "
                            << "at line: " << __LINE__ << ", func: " 
                            << __func__ << ", file: " << __FILE__ 
                            << std::endl;
                    perror( m.what() );
                    exit(EXIT_FAILURE);
                }
                ++m_currListSize;
                return idx;
            }


            /**
             * @brief  Get the buffer of a sender
             * @param  idx : The index of the sender
             * @return A pointer to send buf. Return NULL for invalid input
             */
            uint8_t* getSendBuf(int idx) {
                if(idx < 0 || idx > m_currListSize + 1) {
                    perror("Invalid sender index");
                    return NULL;
                }
                return m_senderList[idx]->getBuf();
            }


            /**
             * @brief  Get the buffer size of a sender
             * @param  idx : The index of the sender
             * @return The size of send buf. Return -1 for invalid input
             */
            uint16_t getSendBufSize(int idx) {
                if(idx < 0 || idx > m_currListSize + 1) {
                    perror("Invalid sender index");
                    return -1;
                }
                return m_senderList[idx]->getBufSize();
            }


            /**
             * @brief  Send mavlink message
             * @param  idx : The index of the sender
             * @param  len : The length that should be sent
             * @return Bytes that is sent. Return -2 for invalid, -1 for fail
             */
            int send(int idx, int len) {
                if(idx < 0 || idx > m_currListSize + 1) {
                    perror("Invalid sender index");
                    return -2;
                }
                if(len < 0 || m_senderList[idx]->getBufSize() < len) {
                    perror("Invalid length value");
                    return -3;
                }
                return m_senderList[idx]->send(len);
            }


            /**
             * @brief   Get the buffer of receiver
             * @return  The pointer to the receiver buffer
             */
            uint8_t* getRecvBuf() {
                return m_receiver->getBuf();
            }


            /**
             * @brief   Get the buffer size of receiver
             * @return  The size of the receiver buffer
             */
            uint16_t getRecvBufSize() {
                return m_receiver->getBufSize();
            }


            /**
             * @brief   Receive mavlink message
             * @return  Bytes that is received. Return -1 if it fails
             * @warning UNSAFE FOR MULTI_THREAD!
             */
            int recv() {
                return m_receiver->recv();
            }


        private:
            MsgManager(uint16_t senderListSize, uint16_t recvBufSize) {
                m_maxListSize = senderListSize;
                m_currListSize = 0;
                try {
                    m_senderList = new MsgSender*[m_maxListSize];
                    memset(m_senderList, 0, m_maxListSize);
                } catch(std::bad_alloc& m) {
                    std::cerr << "Failed to alloc memory for sender list: " 
                            << "at line: " << __LINE__ << ", func: " 
                            << __func__ << ", file: " << __FILE__ 
                            << std::endl;
                    perror( m.what() );
                    exit(EXIT_FAILURE);
                }
                m_receiver = MsgReceiver::getInstance(recvBufSize);
            }


            ~MsgManager() {
                if(NULL != m_senderList) {
                    for(int i = 0; i < m_maxListSize; ++i) {
                        delete m_senderList[i];
                        m_senderList[i] = NULL;
                    }
                }
                m_receiver = NULL;
            }


            static MsgManager* m_instance;
            MsgSender** m_senderList;
            uint16_t m_maxListSize;
            uint16_t m_currListSize;
            MsgReceiver* m_receiver;
    };

    MsgManager* MsgManager::m_instance = NULL;

}


#endif
