/*****************************************************************************
 * @Brief     ROS-free and MavLink-free low-level socket communicator class
 * @Version   0.3.0
 * @Author    Chris Liu
 * @Created   2015/10/29
 * @Modified  2015/12/16
 *****************************************************************************/

#ifndef _DJI2MAV_SOCKETCOMM_H_
#define _DJI2MAV_SOCKETCOMM_H_


#include <sys/socket.h>
#include <sys/types.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <fcntl.h>
#include <unistd.h>
#include <time.h>
#include <errno.h>

#include "log.h"

namespace dji2mav {

    class SocketComm {
        public:
            SocketComm() {
                DJI2MAV_DEBUG("Construct SocketComm.");
                m_sock = -1;
            }


            ~SocketComm() {
                DJI2MAV_DEBUG("Going to destruct SocketComm...");
                disconnect();
                DJI2MAV_DEBUG("...finish destructing SocketComm.");
            }


            /* Set the Connection Configuration */
            void setConf(std::string gcsIP, uint16_t gcsPort, 
                    uint16_t locPort) {

                DJI2MAV_DEBUG("Set SocketComm config IP: %s, destPort: %u, " 
                        "srcPort: %u", gcsIP.c_str(), gcsPort, locPort);

                memset(&m_locAddr, 0, sizeof(m_locAddr));
                m_locAddr.sin_family = AF_INET;
                m_locAddr.sin_addr.s_addr = INADDR_ANY;
                m_locAddr.sin_port = htons(locPort);

                memset(&m_gcsAddr, 0, sizeof(m_gcsAddr));
                m_gcsAddr.sin_family = AF_INET;
                m_gcsAddr.sin_addr.s_addr = inet_addr(gcsIP.c_str());
                m_gcsAddr.sin_port = htons(gcsPort);

            }


            /* Connect to the GCS */
            bool connect() {

                DJI2MAV_INFO("Attempt to connect to %s:%u " 
                        "using local port %u...", 
                        inet_ntoa(m_gcsAddr.sin_addr), 
                        ntohs(m_gcsAddr.sin_port), 
                        ntohs(m_locAddr.sin_port) );

                // Set socket properties. Using UDP
                if((m_sock = socket(PF_INET, SOCK_DGRAM, IPPROTO_UDP)) < 0) {
                    DJI2MAV_FATAL("...socket invalid!");
                    return false;
                }

                // Set the Communication Nonblocking
                int flag = fcntl(m_sock, F_GETFL, 0);
                if(fcntl(m_sock, F_SETFL, flag | O_NONBLOCK) < 0) {
                    DJI2MAV_FATAL("...set nonblocking socket fail!");
                    close(m_sock);
                    return false;
                }

                // Bind socket
                if( -1 == bind(m_sock, (struct sockaddr *)&m_locAddr, 
                        sizeof(struct sockaddr_in)) ) {
                    DJI2MAV_FATAL("...bind socket fail!");
                    close(m_sock);
                    return false;
                }

                DJI2MAV_INFO("...connection succeed.");
                m_timer = time(NULL);
                return true;

            }


            /* Disconnect to the GCS */
            bool disconnect() {

                DJI2MAV_DEBUG("Going to disconnect to %s:%u " 
                        "on local port %u...", 
                        inet_ntoa(m_gcsAddr.sin_addr), 
                        ntohs(m_gcsAddr.sin_port), 
                        ntohs(m_locAddr.sin_port) );

                if(m_sock != -1) {
                    close(m_sock);
                    m_sock = -1;
                    DJI2MAV_DEBUG("...finish disconnecting.");
                    return true;
                } else {
                    DJI2MAV_DEBUG("...no socke was binded.");
                    return false;
                }

            }


            /**
             * @brief  Send Message Nonblocked
             * @param  sendBuf : pointer to the send buffer
             * @param  msglen  : length of message that is going to be sent
             * @return Bytes that is sent. Return -1 if it fails
             */
            int send(uint8_t* sendBuf, uint16_t msglen) {

                DJI2MAV_TRACE( "Send to %s:%u using port %u with %u bytes " 
                        "message.", inet_ntoa(m_gcsAddr.sin_addr), 
                        ntohs(m_gcsAddr.sin_port), ntohs(m_locAddr.sin_port), 
                        msglen );

                // Using Nonblocking flag "MSG_DONTWAIT"
                int ret = sendto( m_sock, sendBuf, msglen, MSG_DONTWAIT, 
                        (struct sockaddr *)&m_gcsAddr, 
                        sizeof(struct sockaddr_in) );
                if(ret == -1)
                    DJI2MAV_ERROR("Fail to send message!");
                else if(ret < msglen)
                    DJI2MAV_ERROR("Partial msg failed to send!");

                return ret;

            }


            /**
             * @brief  Receive Message Nonblocked
             * @param  recvBuf   : pointer to the receive buffer
             * @param  maxBufLen : max length of of the buffer
             * @return Bytes received. Return -1 for no datagram or -2 for fail
             */
            int recv(void* recvBuf, uint16_t maxBufLen) {
                socklen_t l = sizeof(m_gcsAddr);
                // Using Nonblocking flag "MSG_DONTWAIT"
                int ret = recvfrom( m_sock, recvBuf, maxBufLen, MSG_DONTWAIT, 
                        (struct sockaddr *)&m_gcsAddr, &l );
                if(ret == -1) {
                    if(errno != EAGAIN) {
                        DJI2MAV_ERROR("Fail to receive message!");
                        return -2;
                    }
                    if( m_timer + 10 < time(NULL) ) {
                        DJI2MAV_INFO("No datagram received in last 10 sec.");
                        m_timer = time(NULL);
                        return -1;
                    }
                } else if(ret > 0) {
                    DJI2MAV_TRACE("Received %d bytes msg.", ret);
                    m_timer = time(NULL);
                    return ret;
                } else if(0 == ret) {
                    DJI2MAV_ERROR( "The connection to %s:%u is closed!", 
                            inet_ntoa(m_gcsAddr.sin_addr), 
                            ntohs(m_gcsAddr.sin_port) );
                    return -2;
                } else {
                    DJI2MAV_ERROR("Undefined return value %d while receiving " 
                            "datagram!", ret);
                    return -2;
                }
            }


        private:
            struct sockaddr_in m_gcsAddr;
            struct sockaddr_in m_locAddr;
            int m_sock;
            long m_timer;
    };

} //namespace dji2mav


#endif
