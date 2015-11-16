/*****************************************************************************
 * @Brief     Handle user-defined funcs. Mav-depended and ROS-free singleton
 * @Version   1.0
 * @Author    Chris Liu
 * @Created   2015/11/9
 * @Modified  2015/11/10
 *****************************************************************************/

#ifndef _DJI2MAV_MAVRESPONSER_H_
#define _DJI2MAV_MAVRESPONSER_H_


#include "mavWaypoint.h"

#include <mavlink/v1.0/common/mavlink.h>
#include <new>

namespace dji2mav {

    class MavResponser {
        public:
            /**
             * @brief   Lazy mode singleton
             * @return  The only instance of MavResponser
             * @warning UNSAFE FOR MULTI-THREAD!
             */
            static MavResponser* getInstance() {
                if(NULL == m_instance) {
                    try {
                        m_instance = new MavResponser;
                    } catch(std::bad_alloc &m) {
                        std::cerr << "New instance of MavResponser fail: " 
                                << "at line: " << __LINE__ << ", func: " 
                                << __func__ << ", file: " << __FILE__ 
                                << std::endl;
                        perror( m.what() );
                        exit(EXIT_FAILURE);
                    }
                }
                return m_instance;
            }


            void setHeartbeatRsp( void (*func)() ) {
                m_respondToHeartbeat = func;
            }


            void setPingRsp( void (*func)() ) {
                m_respondToHeartbeat = func;
            }


            void setMissionRequestListRsp( void (*func)() ) {
                m_respondToMissionRequestList = func;
            }


            void setMissionRequestRsp( void (*func)(uint16_t*) ) {
                m_respondToMissionRequest = func;
            }


            void setMissionAckRsp( void (*func)() ) {
                m_respondToMissionAck = func;
            }


            void setMissionCountRsp( void (*func)(uint16_t*) ) {
                m_respondToMissionCount = func;
            }


            void setMissionItemRsp( void (*func)(uint16_t*) ) {
                m_respondToMissionItem = func;
            }


            bool isSetHeartbeatRsp() {
                if(m_respondToHeartbeat)
                    return true;
                else
                    return false;
            }


            bool isSetPingRsp() {
                if(m_respondToHeartbeat)
                    return true;
                else
                    return false;
            }


            bool isSetMissionRequestListRsp() {
                if(m_respondToMissionRequestList)
                    return true;
                else
                    return false;
            }


            bool isSetMissionRequestRsp() {
                if(m_respondToMissionRequest)
                    return true;
                else
                    return false;
            }


            bool isSetMissionAckRsp() {
                if(m_respondToMissionAck)
                    return true;
                else
                    return false;
            }


            bool isSetMissionCountRsp() {
                if(m_respondToMissionCount)
                    return true;
                else
                    return false;
            }


            bool isSetMissionItemRsp() {
                if(m_respondToMissionItem)
                    return true;
                else
                    return false;
            }


            void respondToHeartbeat() {
                m_respondToHeartbeat();
            }


            void respondToPing() {
                m_respondToPing();
            }


            void respondToMissionRequestList() {
                m_respondToMissionRequestList();
            }


            void respondToMissionRequest(uint16_t* param) {
                m_respondToMissionRequest(param);
            }


            void respondToMissionAck() {
                m_respondToMissionAck();
            }


            void respondToMissionCount(uint16_t* param) {
                m_respondToMissionCount(param);
            }


            void respondToMissionItem(uint16_t* param) {
                m_respondToMissionItem(param);
            }


        private:
            MavResponser() {
                m_respondToHeartbeat = NULL;
                m_respondToPing = NULL;
                m_respondToMissionRequestList = NULL;
                m_respondToMissionRequest = NULL;
                m_respondToMissionAck = NULL;
                m_respondToMissionCount = NULL;
                m_respondToMissionItem = NULL;
            }


            ~MavResponser() {
            }


            static MavResponser* m_instance;
            // function pointers
            void (*m_respondToHeartbeat)();
            void (*m_respondToPing)();
            void (*m_respondToMissionRequestList)();
            void (*m_respondToMissionRequest)(uint16_t*);
            void (*m_respondToMissionAck)();
            void (*m_respondToMissionCount)(uint16_t*);
            void (*m_respondToMissionItem)(uint16_t*);

    };

    MavResponser* MavResponser::m_instance = NULL;

}


#endif
