/*****************************************************************************
 * @Brief     Manage msg senders and receiver. ROS-free and mav-free
 * @Version   0.2.1
 * @Author    Chris Liu
 * @Created   2015/10/30
 * @Modified  2015/11/16
 *****************************************************************************/

#ifndef _DJI2MAV_MSGMANAGER_H_
#define _DJI2MAV_MSGMANAGER_H_


#include "msgSender.h"
#include "msgReceiver.h"

#include <new>
#include <stdio.h>
#include <iostream>

namespace dji2mav{

    class MsgManager {
        public:
            /**
             * @brief   Constructor for MsgManager
             * @param   senderListSize : Used to alloc senders list
             * @param   sendBufSize    : Used to alloc buf for sender
             * @param   recvBufSize    : Used to alloc buf for receiver
             */
            MsgManager(uint16_t senderListSize, uint16_t sendBufSize, 
                    uint16_t recvBufSize) : m_maxListSize(senderListSize), 
                    m_defaultSendBufSize(sendBufSize) {

                try {
                    m_senderList = new MsgSender*[m_maxListSize];
                    memset(m_senderList, 0, m_maxListSize * sizeof(MsgSender*));
                    m_currListSize = 0;
                    m_comm = new SocketComm();
                    m_receiver = new MsgReceiver(recvBufSize);
                } catch(std::bad_alloc& m) {
                    std::cerr << "Failed to alloc memory for manager: " 
                            << "at line: " << __LINE__ << ", func: " 
                            << __func__ << ", file: " << __FILE__ 
                            << std::endl;
                    perror( m.what() );
                    exit(EXIT_FAILURE);
                }

                m_generalSenderIdx = registerSender();
                if(-1 == m_generalSenderIdx) {
                    printf("Fail to register a general sender. "
                            "Did you set sender list size 0?\n");
                    exit(EXIT_FAILURE);
                }

            }


            ~MsgManager() {
                if(NULL != m_senderList) {
                    for(uint16_t i = 0; i < m_currListSize; ++i) {
                        delete m_senderList[i];
                        m_senderList[i] = NULL;
                    }
                    delete []m_senderList;
                    m_senderList = NULL;
                }
                if(NULL != m_receiver) {
                    delete m_receiver;
                    m_receiver = NULL;
                }
                if(NULL != m_comm) {
                    delete m_comm;
                    m_comm = NULL;
                }

                printf("Finish destructing Manager\n");
            }


            /**
             * @brief   Register a message sender and get its index
             * @param   bufSize : Set the buf size of sender. Default 1024
             * @return  Index of the sender. Return -1 if the list is full
             * @warning UNSAFE FOR MULTI-THREAD!
             */
            int registerSender(uint16_t bufSize) {
                if(m_currListSize >= m_maxListSize)
                    return -1;
                try {
                    m_senderList[m_currListSize] = new MsgSender(bufSize);
                } catch(std::bad_alloc &m) {
                    std::cerr << "Failed to new MsgSender object: "
                            << "at line: " << __LINE__ << ", func: " 
                            << __func__ << ", file: " << __FILE__ 
                            << std::endl;
                    perror( m.what() );
                    exit(EXIT_FAILURE);
                }
                return m_currListSize++;
            }


            /**
             * @brief  Register a messager sender with default buf size
             * @return Refer to "int registerSender(uint16_t bufSize)"
             */
            inline int registerSender() {
                return registerSender(m_defaultSendBufSize);
            }


            /**
             * @brief  Get the buffer of a sender
             * @param  idx : The index of the sender
             * @return A pointer to send buf. Return NULL for invalid input
             */
            inline uint8_t* getSendBuf(uint16_t idx) {
                if(idx >= m_currListSize) {
                    printf("Invalid sender index %u!\n", idx);
                    return NULL;
                }
                return m_senderList[idx]->getBuf();
            }


            /**
             * @brief  Get the buffer size of a sender
             * @param  idx : The index of the sender
             * @return The size of send buf. Return -1 for invalid input
             */
            inline uint16_t getSendBufSize(uint16_t idx) {
                if(idx >= m_currListSize) {
                    printf("Invalid sender index %u!\n", idx);
                    return -1;
                }
                return m_senderList[idx]->getBufSize();
            }


            /**
             * @brief  Get general sender index
             * @return The index of general sender
             */
            inline uint16_t getGeneralSenderIdx() {
                return m_generalSenderIdx;
            }


            /**
             * @brief  Send mavlink message
             * @param  idx : The index of the sender
             * @param  len : The length that should be sent
             * @return Bytes that is sent. Return -2 for invalid, -1 for fail
             */
            int send(uint16_t idx, int len) {
                if(idx >= m_currListSize) {
                    printf("Send fail! Invalid sender index %u!\n", idx);
                    return -2;
                }
                if(len < 0 || m_senderList[idx]->getBufSize() < len) {
                    printf("Send fail! Invalid length value %d!\n", len);
                    return -3;
                }
                return m_senderList[idx]->send(m_comm, len);
            }


            /**
             * @brief   Get the buffer of receiver
             * @return  The pointer to the receiver buffer
             */
            inline uint8_t* getRecvBuf() {
                return m_receiver->getBuf();
            }


            /**
             * @brief   Get the buffer size of receiver
             * @return  The size of the receiver buffer
             */
            inline uint16_t getRecvBufSize() {
                return m_receiver->getBufSize();
            }


            /**
             * @brief   Receive mavlink message
             * @return  Bytes that is received. Return -1 if it fails
             * @warning UNSAFE FOR MULTI_THREAD!
             */
            inline int recv() {
                return m_receiver->recv(m_comm);
            }


            /**
             * @brief  Establish the UDP connection to the target
             * @param  targetIP   : The IP of target
             * @param  targetPort : The connection port of target
             * @param  srcPort    : The connection port of source
             * @return True if succeed or false if fail
             */
            inline bool establish(std::string targetIP, uint16_t targetPort, 
                    uint16_t srcPort) {

                m_comm->setConf(targetIP, targetPort, srcPort);
                return m_comm->connect();

            }


            /**
             * @brief  Get current sender list size
             * @return The size of current sender list
             */
            inline uint16_t getCurrSenderListSize() {
                return m_currListSize;
            }


            /**
             * @brief  Get max sender list size
             * @return The size of max sender list
             */
            inline uint16_t getMaxSenderListSize() {
                return m_maxListSize;
            }


        private:
            MsgSender** m_senderList;
            uint16_t m_maxListSize;
            uint16_t m_currListSize;
            uint16_t m_defaultSendBufSize;
            uint16_t m_generalSenderIdx;
            MsgReceiver* m_receiver;
            SocketComm* m_comm;
    };

} //namespace dji2mav


#endif
