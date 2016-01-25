/*****************************************************************************
 * @Brief     Deal with modules message transportation. ROS-free singleton
 * @Version   0.3.0
 * @Author    Chris Liu
 * @Created   2015/12/21
 * @Modified  2015/12/22
 *****************************************************************************/

#ifndef _DJI2MAV_MAVCONTAINER_H_
#define _DJI2MAV_MAVCONTAINER_H_


#ifndef DEFAULT_RECV_LIST_SIZE
#define DEFAULT_RECV_LIST_SIZE 4
#endif

#include <mavlink/common/mavlink.h>
#include <new>
#include <string>
#include <cstdarg>
#include <algorithm>

#include "mavHandler.h"
#include "modules/mavModule.h"
#include "log.h"

namespace dji2mav{

    class MavContainer{
        public:
            /**
             * @brief  Constructor for the MavContainer
             * @param  handler      : The reference of MavHandler
             * @param  maxModuleNum : The max number of modules
             */
            MavContainer(MavHandler &handler, uint16_t maxModuleNum = 8) {
                DJI2MAV_DEBUG("Going to construct Container with moduleNum " 
                        "%u...", maxModuleNum);

                m_hdlr = &handler;
                m_maxListSize = maxModuleNum;
                m_currListSize = 0;

                try {
                    m_moduleList = new MavModule*[m_maxListSize];
                    memset(m_moduleList, 0, m_maxListSize * sizeof(MavModule*));
                    m_moduleMOI = new uint8_t*[m_maxListSize];
                    memset(m_moduleMOI, 0, m_maxListSize * sizeof(uint8_t*));
                } catch(std::bad_alloc &m) {
                    DJI2MAV_FATAL("Fail to allocate memory for moduleList and " 
                            "moduleMIO! Exception: %s!", m.what());
                    exit(EXIT_FAILURE);
                }

                DJI2MAV_DEBUG("...finish constructing Container.");
            }


            ~MavContainer() {
                DJI2MAV_DEBUG("Going to destruct Container...");
                if(NULL != m_moduleList) {
                    for(uint16_t i = 0; i < m_maxListSize; ++i) {
                        if(NULL != m_moduleList[i]) {
                            //delete m_moduleList[i];
                            m_moduleList[i] = NULL;
                        }
                    }
                    delete []m_moduleList;
                    m_moduleList = NULL;
                }
                if(NULL != m_moduleMOI) {
                    for(uint16_t i = 0; i < m_maxListSize; ++i) {
                        if(NULL != m_moduleMOI[i]) {
                            delete m_moduleMOI[i];
                            m_moduleMOI[i] = NULL;
                        }
                    }
                    delete []m_moduleMOI;
                    m_moduleMOI = NULL;
                }
                DJI2MAV_DEBUG("...finish destructing Container.");
            }


            /**
             * @brief  Inject the module object and set its msgid of interest
             * @param  module : The reference of module object
             * @param  size   : The size of interested msgid list
             * @param  ...    : The msgid list of interest, should be uint8_t
             * @return True if succeed or false if fail
             */
            bool registerModule(MavModule &module, uint16_t size, ...) {
                if(m_currListSize == m_maxListSize) {
                    DJI2MAV_ERROR("Fail to register module %s because " 
                            "currListSize %u reaches the limit %u!", 
                            module.getName().c_str(), m_currListSize, 
                            m_maxListSize);
                    return false;
                }

                /*if(size >= 64) {
                    DJI2MAV_WARN("The size %u of MOI seems too long. This " 
                            "affects the performance of the program.", size);
                }*/

                try {
                    //one more element for storing the list size
                    m_moduleMOI[m_currListSize] = new uint8_t[size + 1];
                    memset( m_moduleMOI[m_currListSize], 0, 
                            (size + 1) * sizeof(uint8_t) );
                } catch(std::bad_alloc &m) {
                    DJI2MAV_FATAL( "Fail to allocate memory for moduleMOI! " 
                            "Exception: %s!", m.what() );
                    exit(EXIT_FAILURE);
                }
                //first position is used to store the size
                m_moduleMOI[m_currListSize][0] = size;

                va_list arg;
                va_start(arg, size);

                m_moduleList[m_currListSize] = &module;
                for(uint16_t i = 1; i <= size; ++i) {
                    uint8_t msgid = (uint8_t)va_arg(arg, int);
                    m_moduleMOI[m_currListSize][i] = msgid;
                    DJI2MAV_DEBUG( "Add msgid #%u to MOI of module %s.", msgid, 
                            module.getName().c_str() );
                }
                std::sort( m_moduleMOI[m_currListSize] + 1, 
                        m_moduleMOI[m_currListSize] + size + 1 );

                ++m_currListSize;
                va_end(arg);
                DJI2MAV_INFO( "Succeed in registering module %s.", 
                        module.getName().c_str() );
                return true;
            }


            /**
             * @brief  Get the index of module by its name
             * @param  moduleName : The string name of module
             * @return The index of module or -1 for not matched
             */
            int getModuleIdx(std::string moduleName) {
                for(uint16_t i = 0; i < m_currListSize; ++i) {
                    if( moduleName == m_moduleList[i]->getName() ) {
                        return i;
                    }
                }
                DJI2MAV_ERROR( "No module is named %s!", moduleName.c_str() );
                return -1;
            }


            /**
             * @brief  Judge whether a msgid is in the MOI list of module
             * @param  moduleIdx : The index of module
             * @param  msgid     : The msgid that is to be checked
             * @return True if exists or false if not
             */
            bool hasMOI(uint16_t moduleIdx, uint8_t msgid) {
                if(moduleIdx >= m_currListSize) {
                    DJI2MAV_ERROR("The index of module %u exceeds the current " 
                            "list size %u!", moduleIdx, m_currListSize);
                    return false;
                }

                //binary search
                uint8_t *begin = m_moduleMOI[moduleIdx] + 1;
                uint8_t *end = m_moduleMOI[moduleIdx] + 
                        m_moduleMOI[moduleIdx][0] + 1;
                uint8_t *mid;
                while(begin <= end) {
                    mid = begin + (end - begin) / 2;
                    if(msgid < *mid)
                        end = mid - 1;
                    else if(msgid > *mid)
                        begin = mid +1;
                    else
                        return true;
                }

                DJI2MAV_TRACE( "Cannot find out msgid %u in module %s.", msgid, 
                        m_moduleList[moduleIdx]->getName().c_str() );
                return false;
            }


            /**
             * @brief  Judge whether a msgid is in the MOI list of module
             * @param  moduleName : The name of the module
             * @param  msgid      : The msgid that is to be checked
             * @return True if exists or false if not
             */
            bool hasMOI(std::string moduleName, uint8_t msgid) {
                int idx = getModuleIdx(moduleName);
                if(-1 == idx) {
                    DJI2MAV_ERROR( "Fail to execute MOI check process because " 
                            "module name %s does not exist!", 
                            moduleName.c_str() );
                    return false;
                }
                return hasMOI( (uint16_t)idx, msgid );
            }


            /**
             * @brief  Fetch messages from Handler specific Manager
             * @param  mngIdx         : The index of specific Manager
             * @param  recvMsgList    : The pointer of a recv msg list
             * @param  recvStatusList : The pointer of a recv status list
             * @param  listSize       : The size of the list
             * @return Num of msg received, -2 for invalid input or -1 for error
             */
            inline int fetchMsgFromMng(uint16_t mngIdx, 
                    mavlink_message_t* recvMsgList, 
                    mavlink_status_t* recvStatusList, uint16_t listSize) {

                return m_hdlr->recvFromMng(mngIdx, recvMsgList, recvStatusList, 
                        listSize);

            }


            /**
             * @brief  Fetch messages from Handler
             * @param  recvMsgList    : The pointer of a recv msg list
             * @param  recvStatusList : The pointer of a recv status list
             * @param  listSize       : The size of the list
             * @return Num of msg received, -1 for error
             */
            inline int fetchMsgFromAll(mavlink_message_t* recvMsgList, 
                    mavlink_status_t* recvStatusList, uint16_t listSize) {

                return m_hdlr->recvFromAll(recvMsgList, recvStatusList, 
                        listSize);

            }


            /**
             * @brief  Deliver received message to specific module
             * @param  moduleIdx : The index of the module
             * @param  msg       : The const pointer to msg
             * @return True if succeed or false if fail
             */
            bool deliverMsg(uint16_t moduleIdx, 
                    const mavlink_message_t &msg) {

                if( hasMOI(moduleIdx, msg.msgid) ) {
                    m_moduleList[moduleIdx]->pushMsg(moduleIdx, msg);
                    return true;
                }
                DJI2MAV_TRACE( "Fail to deliver message to module %s!", 
                        m_moduleList[moduleIdx]->getName().c_str() );
                return false;

            }


            /**
             * @brief  Deliver received message to specific module
             * @param  moduleName : The name of the module
             * @param  msg        : The const pointer to msg
             * @return True if succeed or false if fail
             */
            bool deliverMsg(std::string moduleName, 
                    const mavlink_message_t &msg) {

                int idx = getModuleIdx(moduleName);
                if(-1 == idx) {
                    DJI2MAV_ERROR( "Module name %s is not matched!", 
                            moduleName.c_str() );
                    return false;
                }
                if( hasMOI(idx, msg.msgid) ) {
                    m_moduleList[idx]->pushMsg(idx, msg);
                    return true;
                }
                return false;

            }


            /**
             * @brief  Deliver received message to all modules
             * @param  msg : The const pointer to msg that is to be delivered
             * @return True if succeed or false if fail
             */
            bool deliverMsgToAll(const mavlink_message_t &msg) {
                bool ret = true;
                for(uint8_t i = 0; i < m_currListSize; ++i) {
                    if(NULL != m_moduleList[i] && !deliverMsg(i, msg)) {
                        DJI2MAV_ERROR( "Fail to deliver message to module %s!", 
                                m_moduleList[i]->getName().c_str() );
                        ret = false;
                    }
                }
                if(!ret)
                    DJI2MAV_ERROR("Fail to deliver message to some module!");
                return ret;
            }


            /**
             * @brief A thread calling function for delivering received msg
             * @param param : The pointer to the Container object
             */
            static void* thread_call(void* param) {
                ( (MavContainer*)param )->deliverHelper();
            }


            /**
             * @brief A message recv helper for the thread
             */
            void deliverHelper() {
                mavlink_message_t recvMsgList[DEFAULT_RECV_LIST_SIZE];
                mavlink_status_t recvStatusList[DEFAULT_RECV_LIST_SIZE];
                while(true) {
                    for(uint16_t i = 0; i < m_hdlr->getMaxListSize(); ++i) {
                        int msgCnt = fetchMsgFromMng(i, recvMsgList, 
                                recvStatusList, DEFAULT_RECV_LIST_SIZE);
                        for(uint16_t j = 0; j < msgCnt; ++j) {
                            for(uint16_t k = 0; k < m_currListSize; ++k) {
                                deliverMsg(k, recvMsgList[j]);
                            }
                        }
                    }
                }
            }


            /**
             * @brief  Run threads for delivering messages and modules
             * @return True if succeed to create the thread or false if fail
             */
            bool run() {
                int ret = pthread_create( &m_tid, NULL, thread_call, 
                        (void*)this );
                if(0 != ret) {
                    DJI2MAV_FATAL("Fail to create thread for the Container!");
                    //TODO: should exit and return tid?
                    return false;
                }

                for(uint16_t i = 0; i < m_currListSize; ++i) {
                    ret &= m_moduleList[i]->run();
                }
                return ret;
            }


        private:
            MavHandler* m_hdlr;

            MavModule** m_moduleList;
            uint8_t** m_moduleMOI;
            uint16_t m_maxListSize;
            uint16_t m_currListSize;

            pthread_t m_tid;


    };

} //namespace dji2mav


#endif
