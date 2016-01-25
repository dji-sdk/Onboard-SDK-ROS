/*****************************************************************************
 * @Brief     Singleton msg receiver. Alloc receive buffer here and manager it
 * @Version   0.3.0
 * @Author    Chris Liu
 * @Created   2015/10/30
 * @Modified  2015/12/16
 *****************************************************************************/

#ifndef _DJI2MAV_MSGRECEIVER_H_
#define _DJI2MAV_MSGRECEIVER_H_


#include <string>
#include <new>

#include "socketComm.h"
#include "log.h"

namespace dji2mav {

    class MsgReceiver {
        public:
            MsgReceiver(uint16_t bufSize) : m_bufSize(bufSize) {

                DJI2MAV_DEBUG("Going to construct MsgReceiver...");

                try {
                    m_recvBuf = new uint8_t[m_bufSize];
                    memset(m_recvBuf, 0, m_bufSize * sizeof(uint8_t));
                } catch(std::bad_alloc &m) {
                    DJI2MAV_FATAL("Fail to alloc memory for MsgReceiver buf! " 
                            "Exception: %s!", m.what());
                    exit(EXIT_FAILURE);
                }

                DJI2MAV_DEBUG("...finish constructing MsgReceiver");

            }


            ~MsgReceiver() {
                DJI2MAV_DEBUG("Going to destruct MsgReceiver...");
                delete []m_recvBuf;
                m_recvBuf = NULL;
                DJI2MAV_DEBUG("...finish destructing MsgReceiver.");
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
