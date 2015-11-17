/*****************************************************************************
 * @Brief     Manage configuration of dji2mav. ROS-free singleton
 * @Version   1.1
 * @Author    Chris Liu
 * @Created   2015/11/14
 * @Modified  2015/11/16
 *****************************************************************************/

#ifndef _DJI2MAV_CONFIG_H_
#define _DJI2MAV_CONFIG_H_


#include "mngHandler.h"
#include "mavContainer.h"

#include <iostream>
#include <string>
#include <new>

#define DEFAULT_SENDER_LIST_SIZE 256
#define DEFAULT_SEND_BUF_SIZE 1024
#define DEFAULT_RECV_BUF_SIZE 4096

namespace dji2mav {

    class Config {
        public:
            /**
             * @brief   Get the only instance. Lazy mode singleton
             * @return  The only instance of Config
             * @warning UNSAFE FOR MULTI-THREAD! Should be called BEFORE fork
             */
            static Config* getInstance() {
                if(NULL == m_instance) {
                    try {
                        m_instance = new Config();
                    } catch(std::bad_alloc &m) {
                        std::cerr << "New instance of Config fail: " 
                                << "at line: " << __LINE__ << ", func: " 
                                << __func__ << ", file: " << __FILE__ 
                                << std::endl;
                        perror( m.what() );
                        exit(EXIT_FAILURE);
                    }
                }
                return m_instance;
            }


            /**
             * @brief  Set the mavlink system id and the size of GCS list
             * @param  mavSysid : The system id of this vehicle
             * @param  num      : The number of Ground Control Stations
             * @return True if succeed or false if fail
             */
            inline bool setup(uint8_t mavSysid, uint16_t num) {
                return ( m_hdlr->setHandlerConf(mavSysid, num) );
            }


            /**
             * @brief  Establish the connection of specific GCS
             * @param  gcsIdx         : The index of the GCS. Begin from 0
             * @param  gcsIP          : The IP of GCS
             * @param  gcsPort        : Connection port of GCS
             * @param  locPort        : Localhost port
             * @param  senderListSize : Default 256
             * @param  sendBufSize    : Default 1024
             * @param  recvBufSize    : Default 4096
             * @return True if succeed or false if fail
             */
            inline bool start(uint16_t gcsIdx, std::string gcsIP, 
                    uint16_t gcsPort, uint16_t locPort, 
                    uint16_t senderListSize = DEFAULT_SENDER_LIST_SIZE, 
                    uint16_t sendBufSize = DEFAULT_SEND_BUF_SIZE, 
                    uint16_t recvBufSize = DEFAULT_RECV_BUF_SIZE) {

                return ( m_hdlr->setMngConf(gcsIdx, senderListSize, 
                        sendBufSize, recvBufSize) 
                        && m_hdlr->establish(mngIdx, gcsIP, gcsPort, 
                        locPort) );

            }


        private:
            Config() {
                m_hdlr = MngHandler::getInstance();
            }


            ~Config() {
            }


            static Config* m_instance;
            MngHandler* m_hdlr;

    };

    Config* Config::m_instance = NULL;

} //namespace dji2mav


#endif
