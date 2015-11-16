/*****************************************************************************
 * @Brief     Manage configuration of dji2mav. ROS-free singleton
 * @Version   1.0
 * @Author    Chris Liu
 * @Created   2015/11/14
 * @Modified  2015/11/15
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
             * @brief Set system id for the vehicle. Unnecssary to be called
             * @param sysid : The system id to be set
             */
            inline void setSysid(uint8_t sysid) {
                m_sysid = sysid;
            }


            /**
             * @brief  Get system id for the whole device
             * @return The system id of this device
             */
            inline uint8_t getSysid() {
                return m_sysid;
            }


            /**
             * @brief  Establish the connection of specific GCS
             * @param  gcsIdx  : The index of the GCS. Begin from 0
             * @param  gcsIP   : The IP of GCS
             * @param  gcsPort : Connection port of GCS
             * @param  locPort : Localhost port
             * @return True if succeed or false if fail
             */
            inline bool establish(uint8_t gcsIdx, std::string gcsIP, 
                    int gcsPort, int locPort) {
                if(gcsIdx >= m_gcsListSize)
                    return false;
                return m_gcsList[gcsIdx]->establish(gcsIP, gcsPort, locPort);
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
                m_sysid = 1;
            }


            ~Config() {
            }


            static Config* m_instance;
            uint8_t m_gcsListSize;
            MavHandler** m_gcsList;
            uint8_t m_sysid;

    };

    Config* Config::m_instance = NULL;

}


#endif
