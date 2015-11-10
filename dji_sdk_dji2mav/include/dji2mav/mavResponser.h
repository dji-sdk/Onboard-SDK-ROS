/****************************************************************************
 * @Brief   Response to received msg. Mavlink-depended and ROS-free singleton
 * @Version 1.0
 * @Author  Chris Liu
 * @Create  2015/11/9
 * @Modify  2015/11/10
 ****************************************************************************/

#ifndef _DJI2MAV_MAVRESPONSER_H_
#define _DJI2MAV_MAVRESPONSER_H_


#include <mavlink/v1.0/common/mavlink.h>

namespace dji2mav {

    class MavResponser {
        public:
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


            void decode(mavlink_message_t &msg) {
                //TODO:checksum?
                switch(msg.msgid) {
                    case MAVLINK_MSG_ID_HEARTBEAT:
                        m_respondToHeartbeat();
                        break;
                    case MAVLINK_MSG_ID_PING:
                        m_respondToPing();
                        break;
                }
                //TODO:return?
            }


            void registerHeartbeat(void (*func)()) {
                m_respondToHeartbeat = func;
            }


        private:
            MavResponser() {
            }


            ~MavResponser() {
            }


            static MavResponser* m_instance;
            void (*m_respondToHeartbeat)();
            void (*m_respondToPing)();

    };

    MavResponser* MavResponser::m_instance = NULL;

}


#endif
