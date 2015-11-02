/****************************************************************************
 * @brief   Singleton msg receiver. Alloc receive buffer here and manager it.
 * @version 1.0
 * @Date    2014/10/30
 ****************************************************************************/

#ifndef _MSGRECEIVER_H_
#define _MSGRECEIVER_H_


#include <stdlib.h>
#include <string>
#include <new>

namespace dji2mav {

    class MsgReceiver {
        public:
            /* Lazy Mode Singleton. WARNING: Unsafe for multi-thread */
            // Can set buffer size at the first call of getInstance(size)
            static MsgReceiver* getInstance(uint16_t bufSize) {

                if(NULL == m_instance) {
                    try {
                        m_instance = new MsgReceiver(bufSize);
                    } catch(bad_alloc& m) {
                        perror( "Cannot new instance of MsgReceiver: " 
                                + m.what() );
                    }
                }
                return m_instance;

            }


            uint16_t getBufSize() {
                return m_bufSize;
            }


            void* getBuf() {
                return m_recvBuf;
            }


            int recv() {
                return m_comm->recv(m_recvBuf, m_bufSize);
            }


        private:
            MsgReceiver(uint16_t bufSize) : m_bufSize(bufSize) {
                try {
                    m_recvBuf = (void*) new uint8_t[m_bufSize];
                } catch(bad_alloc& m) {
                    //cerr<<
                    perror( "Failed to alloc memory for MsgReceiver buffer: " 
                            + m.what() );
                }
                m_comm = Communicator::getInstance();
            }


            ~MsgReceiver() {
                delete []m_recvBuf;
                m_recvBuf = NULL;
                m_comm = NULL;
            }


            static MsgReceiver* m_instance;
            void* m_recvBuf;
            uint16_t m_bufSize;
            Communicator* m_comm;
    };

}


#endif
