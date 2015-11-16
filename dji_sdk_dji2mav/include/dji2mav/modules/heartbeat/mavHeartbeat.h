/*****************************************************************************
 * @Brief     Heartbeat module. Mav-depended and ROS-free
 * @Version   1.1
 * @Author    Chris Liu
 * @Created   2015/11/16
 * @Modified  2015/11/16
 *****************************************************************************/

#ifndef _DJI2MAV_MAVHEARTBEAT_H_
#define _DJI2MAV_MAVHEARTBEAT_H_


#include "mavContainer.h"

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
             * @brief Set the senderRecord array address
             * @param senderRecord : The array address
             */
            inline void setSenderRecord(int* senderRecord) {
                m_senderRecord = senderRecord;
            }


            /**
             * @brief  Get the senderRecord array address
             * @return The address og senderRecord
             */
            inline int* getSenderRecord() {
                return m_senderRecord;
            }


            /**
             * @brief  Set the sender index of specific GCS
             * @param  gcsIdx    : The index of GCS
             * @param  senderIdx : The index of sender
             * @return True if succeed or false if fail
             */
            inline bool setSenderIdx(uint16_t gcsIdx, int senderIdx) {
                if( !m_ctnr->isValidIdx(gcsIdx, senderIdx) )
                    return false;
                m_senderRecord[gcsIdx] = senderIdx;
                return true;
            }


        private:
            MavContainer() {
                m_ctnr = MavContainer::getInstance();
            }


            ~MavContainer() {
            }


            static MavHeartbeat* m_instance;
            MavHandler* m_hdlr;
            int* m_senderRecord;

    };

} //namespace dji2mav


#endif
