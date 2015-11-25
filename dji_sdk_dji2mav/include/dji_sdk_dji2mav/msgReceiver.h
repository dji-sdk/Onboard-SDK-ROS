/*****************************************************************************
 * @Brief     Singleton msg receiver. Alloc receive buffer here and manager it
 * @Version   0.2.1
 * @Author    Chris Liu
 * @Created   2015/10/30
 * @Modified  2015/11/15
 *****************************************************************************/

#ifndef _MSGRECEIVER_H_
#define _MSGRECEIVER_H_


#include "socketComm.h"

#include <stdlib.h>
#include <stdio.h>
#include <iostream>
#include <string>
#include <new>

namespace dji2mav {

    class MsgReceiver {
        public:
            MsgReceiver(uint16_t bufSize) : m_bufSize(bufSize) {

                try {
                    m_recvBuf = new uint8_t[m_bufSize];
                    memset(m_recvBuf, 0, m_bufSize * sizeof(uint8_t));
                } catch(std::bad_alloc &m) {
                    std::cerr << "Fail to alloc memory for MsgReceiver buf: " 
                            << "at line: " << __LINE__ << ", func: " 
                            << __func__ << ", file: " << __FILE__ 
                            << std::endl;
                    perror( m.what() );
                    exit(EXIT_FAILURE);
                }

            }


            ~MsgReceiver() {
                delete []m_recvBuf;
                m_recvBuf = NULL;
                printf("Finish destructing Receiver\n");
            }


            inline uint16_t getBufSize() {
                return m_bufSize;
            }


            inline uint8_t* getBuf() {
                return m_recvBuf;
            }


            inline int recv(SocketComm* comm_p) {
                return comm_p->recv( (void *)m_recvBuf, m_bufSize );
            }


        private:
            uint16_t m_bufSize;
            uint8_t* m_recvBuf;
    };

} //namespace dji2mav


#endif
