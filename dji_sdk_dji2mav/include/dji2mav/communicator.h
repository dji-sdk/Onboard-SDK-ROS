/****************************************************************************
 * @brief   ROS-free and MavLink-free low-level socket communicator class
 * @version 1.0
 * @Date    2014/10/29
 ****************************************************************************/

#ifndef _DJI2MAV_COMMUNICATOR_H_
#define _DJI2MAV_COMMUNICATOR_H_


#include <sys/socket.h>
#include <sys/types.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <fcntl.h>
#include <errno.h>
#include <stdio.h>
#include <string>

namespace dji2mav {

    class Communicator {
        public:
            /* Lazy Mode Singleton. WARNING: Unsafe for multi-thread */
            static Communicator* getInstance() {
                if(NULL == m_instance) {
                    try {
                        m_instance = new Communicator();
                    } catch(std::bad_alloc &m) {
                        std::cerr << "Cannot new instance of Communicator: " 
                                << "At line: " << __LINE__ << ", func: " 
                                << __func__ << ", file: " << __FILE__ 
                                << std::endl;
                        perror( m.what() );
                        exit(EXIT_FAILURE);
                    }
                }
                return m_instance;
            }


            /* Set the Connection Configuration */
            void setConf(std::string gcsIP, int gcsPort, int locPort) {
                memset(&m_locAddr, 0, sizeof(m_locAddr));
                m_locAddr.sin_family = AF_INET;
                m_locAddr.sin_addr.s_addr = INADDR_ANY;
                m_locAddr.sin_port = htons(locPort);

                memset(&m_gcsAddr, 0, sizeof(m_gcsAddr));
                m_gcsAddr.sin_family = AF_INET;
                m_gcsAddr.sin_addr.s_addr = inet_addr(gcsIP.c_srt());
                m_gcsAddr.sin_port = htons(gcsPort);
            }


            /* Connect to the GCS */
            bool connect() {
                // Set socket properties. Using UDP
                if((m_sock = socket(PF_INET, SOCK_DGRAM, IPPROTO_UDP)) < 0) {
                    perror("Socket invalid: " + strerror(errno));
                    return false;
                }

                // Set the Communication Nonblocking
                int flag = fcntl(m_sock, F_GETFL, 0);
                if(fcntl(m_sock, F_SETFL, flag | O_NONBLOCK) < 0) {
                    perror("Set nonblocking socket fail: " + strerror(errno));
                    close(m_sock);
                    return false;
                }

                // Bind socket
                if( -1 == bind(m_sock, (struct sockaddr *)&m_locAddr, 
                        sizeof(struct sockaddr_in)) ) {
                    perror("Bind socket fail: " + strerror(errno));
                    close(m_sock);
                    return false;
                }

                return true;
            }


            /* Disconnect to the GCS */
            bool disconnect() {
                if(m_sock != INVALID_SOCKET) {
                    close(m_sock);
                    m_sock = INVALID_SOCKET;
                    return true;
                } else {
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
                // Using Nonblocking flag "MSG_DONTWAIT"
                int ret = sendto( m_sock, sendBuf, msglen, MSG_DONTWAIT, 
                        (struct sockaddr *)&m_gcsAddr, 
                        sizeof(struct sockaddr_in) );
                if(ret == -1)
                    perror("Fail to send message: " + strerror(errno));
                else if(ret < msglen)
                    perror("Partial msg failed to send: " + strerror(errno));
                return ret;
            }


            /**
             * @brief  Receive Message Nonblocked
             * @param  recvBuf   : pointer to the receive buffer
             * @param  maxBufLen : max length of of the buffer
             * @return Bytes that is received. Return -1 if it fails
             */
            int recv(void* recvBuf, uint16_t maxBufLen) {
                // Using Nonblocking flag "MSG_DONTWAIT"
                int ret = recvfrom( m_sock, recvBuf, maxBufLen, MSG_DONTWAIT, 
                        (struct sockaddr *)&m_gcsAddr, 
                        sizeof(struct sockaddr_in) );
                if(ret == -1)
                    perror("Fail to receive message: " + strerror(errno));
                return ret;
            }


        private:
            Communicator() {
                m_sock = -1;
            }


            ~Communicator() {
                disconnect();
            }


            static Communicator* m_instance;
            struct sockaddr_in m_gcsAddr;
            struct sockaddr_in m_locAddr;
            int m_sock;
    };

}


#endif
