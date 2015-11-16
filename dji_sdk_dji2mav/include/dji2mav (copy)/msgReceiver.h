/*****************************************************************************
 * @Brief     Singleton msg receiver. Alloc receive buffer here and manager it
 * @Version   1.0
 * @Author    Chris Liu
 * @Created   2015/10/30
 * @Modified  2015/11/10
 *****************************************************************************/

#ifndef _MSGRECEIVER_H_
#define _MSGRECEIVER_H_


#include <stdlib.h>
#include <stdio.h>
#include <iostream>
#include <string>
#include <new>

#define DEFAULT_RECV_BUF_SIZE 4096

namespace dji2mav {

    class MsgReceiver {
        public:
            /* Lazy Mode Singleton. WARNING: Unsafe for multi-thread */
            // Can set buffer size at the first call of getInstance(size)
            static MsgReceiver* getInstance() {

                if(NULL == m_instance) {
                    try {
                        m_instance = new MsgReceiver();
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
            MsgReceiver() {
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
                m_comm = SocketComm::getInstance();
            }


            ~MsgReceiver() {
                delete []m_recvBuf;
                m_recvBuf = NULL;
                m_comm = NULL;
            }


            static MsgReceiver* m_instance;
            static uint16_t m_bufSize;
            uint8_t* m_recvBuf;
            SocketComm* m_comm;
    };

    MsgReceiver* MsgReceiver::m_instance = NULL;
    uint16_t m_bufSize = DEFAULT_RECV_BUF_SIZE;

}


#endif
