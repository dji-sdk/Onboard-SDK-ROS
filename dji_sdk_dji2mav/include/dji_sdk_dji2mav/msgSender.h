/*****************************************************************************
 * @Brief     Messages sender. Alloc send buffer here and manager it.
 * @Version   0.3.0
 * @Author    Chris Liu
 * @Created   2015/10/30
 * @Modified  2015/12/16
 *****************************************************************************/

#ifndef _DJI2MAV_MSGSENDER_H_
#define _DJI2MAV_MSGSENDER_H_


#include <string>
#include <new>

#include "socketComm.h"
#include "log.h"

namespace dji2mav {

    class MsgSender {
        public:
            MsgSender(uint16_t bufSize) : m_bufSize(bufSize) {

                DJI2MAV_DEBUG("Going to construct MsgSender...");

                try {
                    m_sendBuf = new uint8_t[m_bufSize];
                    memset(m_sendBuf, 0, m_bufSize * sizeof(uint8_t));
                } catch(std::bad_alloc &m) {
                    DJI2MAV_FATAL( "Fail to alloc memory for MsgSender buf! " 
                            "Exception: %s!", m.what() );
                    exit(EXIT_FAILURE);
                }

                DJI2MAV_DEBUG("...finish constructing MsgSender.");

            }


            ~MsgSender() {
                DJI2MAV_DEBUG("Going to destruct MsgSender...");
                delete []m_sendBuf;
                m_sendBuf = NULL;
                DJI2MAV_DEBUG("...finish destructing MsgSender.");
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
