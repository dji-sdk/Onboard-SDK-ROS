/****************************************************************************
 * @Brief   Singleton msg receiver. Alloc receive buffer here and manager it.
 * @Version 1.0
 * @Author  Chris Liu
 * @Create  2015/10/30
 * @Modify  2015/11/10
 ****************************************************************************/

#ifndef _MSGRECEIVER_H_
#define _MSGRECEIVER_H_


#include <stdlib.h>
#include <stdio.h>
#include <iostream>
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
                    } catch(std::bad_alloc &m) {
                        std::cerr << "Cannot new instance of MsgReceiver: " 
                                << "at line: " << __LINE__ << ", func: " 
                                << __func__ << ", file: " << __FILE__ 
                                << std::endl;
                        perror( m.what() );
                        exit(EXIT_FAILURE);
                    }
                }
                return m_instance;

            }


            uint16_t getBufSize() {
                return m_bufSize;
            }


            uint8_t* getBuf() {
                return m_recvBuf;
            }


            int recv() {
                return m_comm->recv( (void *)m_recvBuf, m_bufSize );
            }


        private:
            MsgReceiver(uint16_t bufSize) : m_bufSize(bufSize) {
                try {
                    m_recvBuf = new uint8_t[m_bufSize];
                } catch(std::bad_alloc &m) {
                    std::cerr << "Fail to alloc memory for MsgReceiver buf: " 
                            << "at line: " << __LINE__ << ", func: " 
                            << __func__ << ", file: " << __FILE__ 
                            << std::endl;
                    perror( m.what() );
                    exit(EXIT_FAILURE);
                }
                m_comm = Communicator::getInstance();
            }


            ~MsgReceiver() {
                delete []m_recvBuf;
                m_recvBuf = NULL;
                m_comm = NULL;
            }


            static MsgReceiver* m_instance;
            uint8_t* m_recvBuf;
            uint16_t m_bufSize;
            Communicator* m_comm;
    };

    MsgReceiver* MsgReceiver::m_instance = NULL;

}


#endif
