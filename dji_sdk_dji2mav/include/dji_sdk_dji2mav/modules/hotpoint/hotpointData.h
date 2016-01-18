/*****************************************************************************
 * @Brief     Data class for hotpoint. Mav-free and ROS-free
 * @Version   0.3.0
 * @Author    Chris Liu
 * @Created   2015/12/14
 * @Modified  2015/12/25
 *****************************************************************************/

#ifndef _DJI2MAV_HOTPOINTDATA_H_
#define _DJI2MAV_HOTPOINTDATA_H_


#include <iostream>
#include <new>
#include <stdio.h>

namespace dji2mav {

    class HotpointData {
        public:
            HotpointData() {
                m_isEmpty = true;
            }


            ~HotpointData() {
            }


            void getHotpointData(uint16_t seq, uint16_t& cmd, float& param1, 
                    float& param2, float& param3, float& param4, float& x, 
                    float& y, float& z) {

                if(false == m_isEmpty && 0 == seq) {
                    param1 = m_data[0];
                    param2 = m_data[1];
                    param3 = m_data[2];
                    param4 = m_data[3];
                    x = m_data[4];
                    y = m_data[5];
                    z = m_data[6];
                    cmd = m_cmd;
                }

            }


            void setHotpointData(uint16_t seq, uint16_t cmd, float param1, 
                    float param2, float param3, float param4, float x, 
                    float y, float z) {

                if(0 == seq) {
                    m_data[0] = param1;
                    m_data[1] = param2;
                    m_data[2] = param3;
                    m_data[3] = param4;
                    m_data[4] = x;
                    m_data[5] = y;
                    m_data[6] = z;
                    m_cmd = cmd;
                    m_isEmpty = false;
                }

            }


            void display() {
                DJI2MAV_INFO("Display the full hotpoint mission:");
                DJI2MAV_INFO("param1: %f, param2: %f, param3: %f, param4: %f, "
                        "lat: %f, lon: %f, alt: %f", m_data[0], m_data[1], 
                        m_data[2], m_data[3], m_data[4], m_data[5], m_data[6]);
                DJI2MAV_INFO("--- End of display ---");
            }


            inline const float *getHotpoint() {
                return m_data;
            }


            inline uint16_t getCmd() {
                return m_cmd;
            }


            void clear() {
                m_isEmpty = true;
            }


        private:
            float m_data[7];
            uint16_t m_cmd;
            bool m_isEmpty;


    };

}


#endif
