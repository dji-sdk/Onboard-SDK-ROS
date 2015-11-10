/****************************************************************************
 * @Brief   Messages sender. Alloc send buffer here and manager it.
 * @Version 1.0
 * @Author  Chris Liu
 * @Create  2015/10/30
 * @Modify  2015/11/10
 ****************************************************************************/

#ifndef _MSGSENDER_H_
#define _MSGSENDER_H_


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
                } catch(std::bad_alloc &m) {
                    std::cerr << "Fail to alloc memory for MsgSender buf: " 
                            << "at line: " << __LINE__ << ", func: " 
                            << __func__ << ", file: " << __FILE__ 
                            << std::endl;
                    perror( m.what() );
                    exit(EXIT_FAILURE);
                }
                m_comm = Communicator::getInstance();

            }


            ~MsgSender() {
                delete []m_sendBuf;
                m_sendBuf = NULL;
                m_comm = NULL;
            }


            uint16_t getBufSize() {
                return m_bufSize;
            }


            uint8_t* getBuf() {
                return m_sendBuf;
            }


            int send(uint16_t len) {
                return m_comm->send(m_sendBuf, len);
            }


        private:
            uint8_t* m_sendBuf;
            uint16_t m_bufSize;
            Communicator* m_comm;
    };

}


#endif
