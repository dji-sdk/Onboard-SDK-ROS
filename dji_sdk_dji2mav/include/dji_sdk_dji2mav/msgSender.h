/*****************************************************************************
 * @Brief     Messages sender. Alloc send buffer here and manager it.
 * @Version   0.2.1
 * @Author    Chris Liu
 * @Created   2015/10/30
 * @Modified  2015/11/15
 *****************************************************************************/

#ifndef _MSGSENDER_H_
#define _MSGSENDER_H_


#include "socketComm.h"

#include <stdlib.h>
#include <stdio.h>
#include <iostream>
#include <string>
#include <new>

namespace dji2mav {

    class MsgSender {
        public:
            MsgSender(uint16_t bufSize) : m_bufSize(bufSize) {

                try {
                    m_sendBuf = new uint8_t[m_bufSize];
                    memset(m_sendBuf, 0, m_bufSize * sizeof(uint8_t));
                } catch(std::bad_alloc &m) {
                    std::cerr << "Fail to alloc memory for MsgSender buf: " 
                            << "at line: " << __LINE__ << ", func: " 
                            << __func__ << ", file: " << __FILE__ 
                            << std::endl;
                    perror( m.what() );
                    exit(EXIT_FAILURE);
                }

            }


            ~MsgSender() {
                delete []m_sendBuf;
                m_sendBuf = NULL;
                printf("Finish destructing Sender\n");
            }


            inline uint16_t getBufSize() {
                return m_bufSize;
            }


            inline uint8_t* getBuf() {
                return m_sendBuf;
            }


            inline int send(SocketComm* comm_p, uint16_t len) {
                return comm_p->send(m_sendBuf, len);
            }


        private:
            uint8_t* m_sendBuf;
            uint16_t m_bufSize;
    };

} //namespace dji2mav


#endif
