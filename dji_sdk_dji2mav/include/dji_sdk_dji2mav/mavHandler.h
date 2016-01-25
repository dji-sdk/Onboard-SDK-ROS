/*****************************************************************************
 * @Brief     Handle mng(aka, GCS, Ground Control Station). ROS-free singleton
 * @Version   0.3.0
 * @Author    Chris Liu
 * @Created   2015/10/31
 * @Modified  2015/12/24
 *****************************************************************************/

#ifndef _DJI2MAV_MAVHANDLER_H_
#define _DJI2MAV_MAVHANDLER_H_


#ifndef DEFAULT_SENDER_LIST_SIZE
#define DEFAULT_SENDER_LIST_SIZE 16
#endif

#ifndef DEFAULT_SEND_BUF_SIZE
#define DEFAULT_SEND_BUF_SIZE 512
#endif

#ifndef DEFAULT_RECV_BUF_SIZE
#define DEFAULT_RECV_BUF_SIZE 4096
#endif

#include <mavlink/common/mavlink.h>
#include <new>
#include <string>

#include "msgManager.h"
#include "log.h"

namespace dji2mav {

    class MavHandler {
        public:
            /**
             * @brief Constructor for MavHandler
             * @param mySysid   : The mavlink sysid of the device
             * @param maxMngNum : The max number of Manager
             */
            MavHandler(uint8_t mySysid, uint16_t maxMngNum) {
                DJI2MAV_DEBUG("Going to construct Handler with maxMngNum " 
                        "%u...", maxMngNum);

                m_mySysid = mySysid;
                m_maxListSize = maxMngNum;

                try {
                    m_mngList = new MsgManager*[m_maxListSize];
                    memset(m_mngList, 0, m_maxListSize * sizeof(MsgManager*));
                    m_sysidRecord = new uint8_t[m_maxListSize];
                    memset(m_sysidRecord, 0, m_maxListSize * sizeof(uint8_t));
                } catch(std::bad_alloc &m) {
                    DJI2MAV_FATAL( "Fail to allocate memory for mngList! " 
                            "Exception: %s!", m.what() );
                    exit(EXIT_FAILURE);
                }

                DJI2MAV_DEBUG("...finish constructing Handler.");
            }


            ~MavHandler() {
                DJI2MAV_DEBUG("Going to destruct Handler...");

                if(NULL != m_mngList) {
                    for(uint16_t i = 0; i < m_maxListSize; ++i) {
                        if(NULL != m_mngList[i]) {
                            delete m_mngList[i];
                            m_mngList[i] = NULL;
                        }
                    }
                    delete []m_mngList;
                    m_mngList = NULL;
                }

                if(NULL != m_sysidRecord) {
                    delete []m_sysidRecord;
                    m_sysidRecord = NULL;
                }

                DJI2MAV_DEBUG("...finish destructing Handler.");
            }


            /**
             * @brief  Check whether the manager index is valid
             * @param  mngIdx : The index of manager that is to be checked
             * @return True if valid or false if invalid
             */
            inline bool isValidMngIdx(uint16_t mngIdx) {
                return (mngIdx < m_maxListSize && NULL != m_mngList[mngIdx]);
            }


            /**
             * @brief  Get the sysid of the device
             * @return The sysid of the device
             */
            inline uint8_t getMySysid() {
                return m_mySysid;
            }


            /**
             * @brief  Get the max size of mngList(aka, GCS number)
             * @return The max size of mngList(aka, GCS number)
             */
            inline uint16_t getMaxListSize() {
                return m_maxListSize;
            }


            /**
             * @brief  Find the sysid of specific Manager
             * @param  mngIdx : The index of Manager
             * @return The sysid of mng. Return 0 if invalid input or no record
             */
            uint8_t findMngSysid(uint16_t mngIdx) {
                if( isValidMngIdx(mngIdx) ) {
                    return m_sysidRecord[mngIdx];
                } else {
                    DJI2MAV_ERROR("Fail to get sysid because Manager index %u " 
                             "is not matched!", mngIdx);
                    return 0;
                }
            }


            /**
             * @brief  Set the sysid of specific Manager
             * @param  mngIdx : The index of Manager
             * @param  sysid  : The sysid that is to be set
             * @return True if succeed or false if fail
             */
            bool setMngSysid(uint16_t mngIdx, uint8_t sysid) {
                if( isValidMngIdx(mngIdx) ) {
                    if(0 != sysid) {
                        m_sysidRecord[mngIdx] = sysid;
                        return true;
                    } else {
                        DJI2MAV_ERROR("Fail to set sysid %u! The sysid should " 
                                "in range 1~255!", sysid);
                        return false;
                    }
                } else {
                    DJI2MAV_ERROR("Fail to set sysid because Manager index %u " 
                            "is not matched!", mngIdx);
                    return false;
                }
            }


            /**
             * @brief  Get the index of Manager by its sysid
             * @param  sysid : The sysid that is used to match
             * @return Idx of Manager, -1 for not found or -2 for invalid input
             */
            int findMngIdx(uint8_t sysid) {
                if(0 == sysid) {
                    DJI2MAV_ERROR("Fail to get mngIdx for invalid input %u!", 
                            sysid);
                    return -2;
                }
                for(uint16_t i = 0; i < m_maxListSize; ++i) {
                    if(sysid == m_sysidRecord[i])
                        return i;
                }
                DJI2MAV_ERROR("Cannot find out the Manager index using sysid " 
                        "%u!", sysid);
                return -1;
            }


            /**
             * @brief Implement update sysid strategy. Currently it is simple
             * @param mngIdx : The index of Manager
             * @param sysid  : The sysid that is going to be update
             */
            void updateSysid(uint16_t mngIdx, uint8_t sysid) {
                if(m_sysidRecord[mngIdx] != sysid) {
                    DJI2MAV_WARN("Switch sysid of Manager with index %u from " 
                            "%u to %u.", mngIdx, m_sysidRecord[mngIdx], sysid);
                    m_sysidRecord[mngIdx] = sysid;
                }
            }


            /**
             * @brief  Register a messager sender with specifc buf size
             * @param  mngIdx  : The index the Manager
             * @param  bufSize : The size of sender buf that is to be allocated
             * @return Idx of the sender. -1 for full list, -2 for invalid input
             */
            int registerSender(uint16_t mngIdx, uint16_t bufSize) {
                if( !isValidMngIdx(mngIdx) ) {
                    DJI2MAV_ERROR("Invalid mngIdx %u while maxListSize is %u!", 
                            mngIdx, m_maxListSize);
                    return -2;
                } else {
                    return m_mngList[mngIdx]->registerSender(bufSize);
                }
            }


            /**
             * @brief  Register a messager sender with default buf size
             * @param  mngIdx  : The index the Manager
             * @return Idx of the sender. -1 for full list, -2 for invalid input
             */
            int registerSender(uint16_t mngIdx) {
                if( !isValidMngIdx(mngIdx) ) {
                    DJI2MAV_ERROR("Invalid mngIdx %u while maxListSize is %u!", 
                            mngIdx, m_maxListSize);
                    return -2;
                } else {
                    return m_mngList[mngIdx]->registerSender();
                }
            }


            /**
             * @brief  Establish UDP connection of the mng
             * @param  mngIdx         : The index of Manager
             * @param  targetIP       : The IP of target
             * @param  targetPort     : The connection port of target
             * @param  srcPort        : The connection port of source
             * @param  senderListSize : The size of sender list
             * @param  sendBufSize    : The default value of send buf size
             * @param  recvBufSize    : The recv buf size
             * @return True if succeed or false if fail
             */
            bool establish(uint16_t mngIdx, std::string targetIP, 
                    uint16_t targetPort, uint16_t srcPort, 
                    uint16_t senderListSize = DEFAULT_SENDER_LIST_SIZE, 
                    uint16_t sendBufSize = DEFAULT_SEND_BUF_SIZE, 
                    uint16_t recvBufSize = DEFAULT_RECV_BUF_SIZE) {

                DJI2MAV_INFO("Going to establish connection for Manager " 
                        "with index %u. The size of sender list is set to " 
                        "%u, the size of send buf and recv buf are set to " 
                        "%u and %u...", mngIdx, senderListSize, sendBufSize, 
                        recvBufSize);

                if(mngIdx >= m_maxListSize) {
                    DJI2MAV_ERROR("The manager index %u exceeds the max " 
                            "manager list size %u!", mngIdx, m_maxListSize);
                    return false;
                }

                if(NULL != m_mngList[mngIdx]) {
                    DJI2MAV_ERROR("The manager with index %u has been set " 
                            "before!", mngIdx);
                    return false;
                }

                try {
                    m_mngList[mngIdx] = new MsgManager(senderListSize, 
                            sendBufSize, recvBufSize);
                } catch(std::bad_alloc &m) {
                    DJI2MAV_FATAL( "Fail to allocate memory for msgManager " 
                            "with index %u! Exception: %s!", mngIdx, m.what() );
                    exit(EXIT_FAILURE);
                }

                bool ret = m_mngList[mngIdx]->establish(targetIP, targetPort, 
                        srcPort);
                if(ret) {
                    DJI2MAV_INFO("...succeed in establishing connection " 
                            "for Manager %u.", mngIdx);
                } else {
                    DJI2MAV_ERROR("...fail to establish connection for " 
                            "Manager with index %u!", mngIdx);
                }

                return ret;
            }


            /**
             * @brief  Send msg to specific mng using specific sender
             * @param  mngIdx    : The index of Manager
             * @param  senderIdx : The index of sender
             * @param  msg       : The reference of msg that is to be sent
             * @return True if succeed and false if fail
             */
            bool sendEncodedMsgToMng(uint16_t mngIdx, uint16_t senderIdx, 
                    const mavlink_message_t &msg) {

                DJI2MAV_TRACE("Going to send encoded message to Manager " 
                        "with index %u...", mngIdx);

                if( !isValidMngIdx(mngIdx) ) {
                    DJI2MAV_ERROR("Fail to send encoded message because " 
                            "invalid mngIdx %u is met while mngListSize " 
                            "is %u!", mngIdx, m_maxListSize);
                    return false;
                }

                uint16_t len = mavlink_msg_to_send_buffer(
                        m_mngList[mngIdx]->getSendBuf(senderIdx), &msg);
                int bytes_sent = m_mngList[mngIdx]->send(senderIdx, len);

                if(bytes_sent < 0) {
                    DJI2MAV_ERROR("Fail to send encoded message while using " 
                            "Manager with index %u!", mngIdx);
                    return false;
                }
                if(bytes_sent != len) {
                    DJI2MAV_ERROR("For Manager #%u, a %u-bytes message was " 
                            "generated but only %d-bytes were sent!", mngIdx, 
                            len, bytes_sent);
                    return false;
                }

                DJI2MAV_TRACE("...succeed in sending encoded message to " 
                        "Manager with index %u.", mngIdx);
                return true;

            }


            /**
             * @brief  Send msg with specific sysid using specific sender
             * @param  sysid     : The target sysid
             * @param  senderIdx : The index of sender
             * @param  msg       : The reference of msg that is to be sent
             * @return True if succeed or false if fail
             */
            bool sendEncodedMsgToSys(uint8_t sysid, uint16_t senderIdx, 
                    const mavlink_message_t &msg) {

                uint16_t idx = findMngIdx(sysid);
                DJI2MAV_TRACE("Convert sysid %u to Manager index %u for "
                        "sending encoded message.", sysid, idx);

                if(0 != idx) {
                    return sendEncodedMsgToMng(idx, senderIdx, msg);
                } else {
                    DJI2MAV_ERROR("Fail to send encoded msg because of " 
                            "%u sysid is met after convertion!", idx);
                    return false;
                }

            }


            /**
             * @brief  Receive data and parse it from buffer to mavlink message
             * @param  mngIdx         : The index of manager
             * @param  recvMsgList    : The pointer to a recv msg array
             * @param  recvStatusList : The pointer to a recv status array
             * @param  listSize       : The size of list
             * @return Num of msg received, -2 for invalid input or -1 for error
             */
            int recvFromMng(uint16_t mngIdx, mavlink_message_t *recvMsgList, 
                    mavlink_status_t *recvStatusList, uint16_t listSize) {

                DJI2MAV_TRACE("Going to fetch message from Manager with " 
                        "index %u to a list of size %u...", mngIdx, listSize);

                if( !isValidMngIdx(mngIdx) ) {
                    DJI2MAV_ERROR("Fail to receive message because invalid" 
                            "mngIdx %u is met while mngListSize is %u!", 
                            mngIdx, m_maxListSize);
                    return -2;
                }

                int bytes_recv = m_mngList[mngIdx]->recv();
                if(bytes_recv > 0) {
                    DJI2MAV_TRACE("Datagram: ");
                    uint16_t cnt = 0;
                    uint8_t* bufPtr = m_mngList[mngIdx]->getRecvBuf();
                    for(int i = 0; i < bytes_recv; ++i, ++bufPtr) {
#ifdef DJI2MAV_LOG_TRACE
                        printf("%02x", *bufPtr);
#endif
                        if(cnt == listSize) {
                            DJI2MAV_ERROR("Fail to parse all datagram because " 
                                    "recvMsgList of size %u is not enough!", 
                                    listSize);
                            break;
                        }

                        if( mavlink_parse_char(MAVLINK_COMM_0, *bufPtr, 
                                &recvMsgList[cnt], &recvStatusList[cnt]) ) {
#ifdef DJI2MAV_LOG_TRACE
                            if(i != bytes_recv) {
                                printf("\n");
                                DJI2MAV_TRACE("Another datagram: ");
                            }
#endif
                            ++cnt;
                        }
                    }
#ifdef DJI2MAV_LOG_TRACE
                    printf("\n");
#endif
                    if(cnt > 0)
                        updateSysid(mngIdx, recvMsgList[0].sysid);

                    DJI2MAV_TRACE("Totally %u messages are received from " 
                            "Manager of index %u.", cnt, mngIdx);
                    return cnt;
                } else if(-1 == bytes_recv) {
                    DJI2MAV_TRACE("No datagram received from Manager with " 
                            "index %d!", mngIdx);
                    return 0;
                } else {
                    DJI2MAV_TRACE("Receiving process of Manager with index %u " 
                            "fail! Return value: %d!", mngIdx, bytes_recv);
                    return -1;
                }

            }


            /**
             * @brief  Receive data from all Manager and parse it
             * @param  recvMsgList    : The pointer to a recv msg list
             * @param  recvStatusList : The pointer to a recv status list
             * @param  listSize       : The size of list
             * @return Num of msg received, -1 for error
             */
            int recvFromAll(mavlink_message_t* recvMsgList, 
                    mavlink_status_t* recvStatusList, uint16_t listSize) {

                int cnt = 0;
                bool err = false;
                for(uint16_t i = 0; i < m_maxListSize; ++i) {
                    if(NULL != m_mngList[i]) {
                        int r = recvFromMng(i, recvMsgList + cnt, 
                                recvStatusList + cnt, listSize - cnt);
                        DJI2MAV_TRACE("Execute receiving process on Manager " 
                                "with index %u and listSize %u. Return value " 
                                "is %d.", i, listSize - cnt, r);
                        if(r >= 0) {
                            DJI2MAV_TRACE("Receive %d msg from Manager with " 
                                    "index %u.", r, i);
                            cnt += r;
                        } else {
                            DJI2MAV_ERROR("Fail to recv msg from Manager " 
                                    "with index %u!", i);
                            err = true;
                        }
                    }
                }
                if(err) {
                    DJI2MAV_ERROR("Fail to execute receiving process on some " 
                            "Manager!");
                    return -1;
                } else {
                    return cnt;
                }

            }


        private:
            MsgManager** m_mngList;
            uint16_t m_maxListSize;
            uint8_t* m_sysidRecord;

            uint8_t m_mySysid;


    };

} //namespace dji2mav


#endif
