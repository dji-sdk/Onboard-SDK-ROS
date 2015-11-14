/*****************************************************************************
 * @Brief     ROS-free and MavLink-free low-level socket communicator class
 * @Version   1.0
 * @Author    Chris Liu
 * @Created   2015/10/29
 * @Modified  2015/11/14
 *****************************************************************************/

#ifndef _DJI2MAV_CONFIG_H_
#define _DJI2MAV_CONFIG_H_


#include "mavHandler.h"

#include <string>

namespace dji2mav {

    class Config {
        public:
            /**
             * @brief   Lazy mode singleton
             * @return  The only instance of MavWaypoint
             * @warning UNSAFE FOR MULTI-THREAD!
             */
            static MavWaypoint* getInstance() {
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
             * @brief  Set the size of Ground Control Station list
             * @param  num : The number of Ground Control Stations
             * @return The real size of Ground Control Station list
             */
            uint8_t setGCSNum(uint8_t num) {
                if(NULL != m_gcsList) {
                    return m_gcsListSize;
                }

                for(uint8_t i = 0; i < num; ++i) {
                    try {
                        m_gcsList[i] = new MavHandler();
                    } catch(std::bad_alloc &m) {
                        std::cerr << "Cannot new instance of MavHanler " 
                                << i << ": at line: " << __LINE__ 
                                << ", func: " << __func__ << ", file: " 
                                << __FILE__ << std::endl;
                        perror( m.what() );
                        exit(EXIT_FAILURE);
                    }
                }
                return m_gcsListSize = num;
            }


            /**
             * @brief  Setup the connection of specific Ground Control Station
             * @param  idx     : The index of the GCS. Begin from 0
             * @param  gcsIP   : The IP of GCS
             * @param  gcsPort : Connection port of GCS
             * @param  locPort : Localhost port
             * @return True if succeed or false if fail
             */
            inline bool setup(uint8_t idx, std::string gcsIP, int gcsPort, 
                    int locPort) {
                if(idx >= m_gcsListSize)
                    return false;
                return m_gcsList[idx]->setup(gcsIP, gcsPort, locPort);
            }


            /**
             * @brief Set the size of sender buffer. Not necessary to be called
             * @param idx  : The index of the GCS. Begin from 0
             * @param size : The size of sender buffer
             */
            inline void setSendBufSize(uint8_t idx, uint16_t size) {
                m_gcsList[idx]->setSendBufSize(size);
            }


            /**
             * @brief Set the size of receiver buf. Not necessary to be called
             * @param idx  : The index of the GCS. Begin from 0
             * @param size : The size of receiver buffer
             */
            inline void setRecvBufSize(uint8_t idx, uint16_t size) {
                m_gcsList[idx]->setRecvBufSize(size);
            }


            /**
             * @brief Set the size of sender list. Not necessary to be called
             * @param idx  : The index of the GCS. Begin from 0
             * @param size : The size of sender list
             */
            inline void setSendBufListSize(uint8_t idx, uint16_t size) {
                m_gcsList[idx]->setSendBufListSize(size);
            }


        private:
            Config() {
                m_gcsListSize = 0;
            }


            ~Config() {
            }


            static Config* m_instance;
            MavHandler** m_gcsList;
            uint8_t m_gcsListSize;

    };

    Config* Config::m_instance = NULL;

}


#endif
