/*****************************************************************************
 * @Brief     Manage msg senders and receiver. ROS-free and mav-free
 * @Version   0.3.0
 * @Author    Chris Liu
 * @Created   2015/10/30
 * @Modified  2015/12/28
 *****************************************************************************/

#ifndef _DJI2MAV_MSGMANAGER_H_
#define _DJI2MAV_MSGMANAGER_H_


#include <new>
#include <string>
#include <mutex>

#include "msgSender.h"
#include "msgReceiver.h"
#include "log.h"

namespace dji2mav{

    class MsgManager {
        public:
            /**
             * @brief Constructor for MsgManager
             * @param senderListSize : Used to allocate senders list
             * @param sendBufSize    : Used to allocate buf for sender
             * @param recvBufSize    : Used to allocate buf for receiver
             */
            MsgManager(uint16_t senderListSize, uint16_t sendBufSize, 
                    uint16_t recvBufSize) : m_maxListSize(senderListSize), 
                    m_defaultSendBufSize(sendBufSize) {

                DJI2MAV_DEBUG("Going to construct MsgManager with max " 
                        "senderListSize %u, default sendBufSize %u and " 
                        "recvBufSize %u...", senderListSize, sendBufSize, 
                        recvBufSize);

                try {
                    m_senderList = new MsgSender*[m_maxListSize];
                    memset(m_senderList, 0, m_maxListSize * sizeof(MsgSender*));
                    m_currListSize = 0;
                    m_comm = new SocketComm();
                    m_receiver = new MsgReceiver(recvBufSize);
                } catch(std::bad_alloc& m) {
                    DJI2MAV_FATAL( "Fail to alloc memory for MsgManager! " 
                            "Exception: %s!", m.what() );
                    exit(EXIT_FAILURE);
                }

                DJI2MAV_DEBUG("...finish constructing MsgManager.");

            }


            ~MsgManager() {
                DJI2MAV_DEBUG("Going to construct MsgManager...");
                if(NULL != m_senderList) {
                    for(uint16_t i = 0; i < m_currListSize; ++i) {
                        if(NULL != m_senderList[i]) {
                            delete m_senderList[i];
                            m_senderList[i] = NULL;
                        }
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
                DJI2MAV_DEBUG("...finish destructing MsgManager.");
            }


            /**
             * @brief  Check if a sender index is valid
             * @paran  senderIdx : The index of sender
             * @return True if valid or false if invalid
             */
            inline bool isValidSenderIdx(uint16_t senderIdx) {
                return (senderIdx < m_currListSize);
            }


            /**
             * @brief   Register a message sender and get its index
             * @param   bufSize : Set the buf size of sender. Default 1024
             * @return  Index of the sender. Return -1 if the list is full
             */
            int registerSender(uint16_t bufSize) {
                DJI2MAV_DEBUG("Register a sender of %u bytes buf...", bufSize);
                m_registerMutex.lock();
                if(m_currListSize >= m_maxListSize) {
                    DJI2MAV_ERROR("The sender list with %u sender is full!", 
                            m_currListSize);
                    return -1;
                }
                try {
                    m_senderList[m_currListSize] = new MsgSender(bufSize);
                } catch(std::bad_alloc &m) {
                    DJI2MAV_FATAL( "Fail to new MsgSender object! " 
                            "Exception: %s!", m.what() );
                    exit(EXIT_FAILURE);
                }
                m_registerMutex.unlock();
                DJI2MAV_DEBUG("...succeed in registering the sender with "
                        "index %u", m_currListSize);
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
                if( !isValidSenderIdx(idx) ) {
                    DJI2MAV_ERROR("Invalid sender index %u!", idx);
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
                if( !isValidSenderIdx(idx) ) {
                    DJI2MAV_ERROR("Invalid sender index %u!", idx);
                    return -1;
                }
                return m_senderList[idx]->getBufSize();
            }


            /**
             * @brief  Send mavlink message
             * @param  idx : The index of the sender
             * @param  len : The length that should be sent
             * @return Bytes sent. Return -2 or -3 for invalid, -1 for fail
             */
            int send(uint16_t idx, int len) {
                if( !isValidSenderIdx(idx) ) {
                    DJI2MAV_ERROR("Send fail! Invalid sender index %u!", idx);
                    return -2;
                }
                if(len < 0 || m_senderList[idx]->getBufSize() < len) {
                    DJI2MAV_ERROR("Send fail! Invalid length value %d!", len);
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
             * @brief  Receive mavlink message
             * @return Bytes received. Return -1 for no datagram or -2 for fail
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
            bool establish(std::string targetIP, uint16_t targetPort, 
                    uint16_t srcPort) {

                DJI2MAV_INFO("Going to establish connection with %s:%u " 
                        "using port %u...", targetIP.c_str(), targetPort, 
                        srcPort);

                m_comm->setConf(targetIP, targetPort, srcPort);
                bool ret = m_comm->connect();

                if(ret)
                    DJI2MAV_INFO("...succeed in establishing connection.");
                else
                    DJI2MAV_ERROR("...fail to establish connection!");

                return ret;

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
            MsgReceiver* m_receiver;
            SocketComm* m_comm;

            std::mutex m_registerMutex;
    };

} //namespace dji2mav


#endif
