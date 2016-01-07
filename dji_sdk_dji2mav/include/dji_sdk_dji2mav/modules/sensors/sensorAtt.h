/*****************************************************************************
 * @Brief     Storage attitude data and provide setter and getter methods
 * @Version   0.3.0
 * @Author    Chris Liu
 * @Created   2015/11/17
 * @Modified  2015/12/25
 *****************************************************************************/

#ifndef _DJI2MAV_SENSORATT_H_
#define _DJI2MAV_SENSORATT_H_


#include "math.h"

namespace dji2mav {

    class SensorAtt {
        public:
            SensorAtt() {
            }


            ~SensorAtt() {
            }


            inline mavlink_attitude_t* getDataPtr() {
                return &m_data;
            }


            inline void setTimeBootMs(const int32_t* ts) {
                m_data.time_boot_ms = *ts;
            }


            inline void setRoll(const float* roll) {
                m_data.roll = *roll;
            }


            inline void setRoll(const float* q0, const float* q1, 
                    const float* q2, const float* q3) {

                m_data.roll = atan2(  2*( (*q0)*(*q1)+(*q2)*(*q3) ), 
                        1-2*( (*q1)*(*q1)+(*q2)*(*q2) )  );

            }


            inline void setPitch(const float* pitch) {
                m_data.pitch = *pitch;
            }


            inline void setPitch(const float* q0, const float* q1, 
                    const float* q2, const float* q3) {

                m_data.pitch = asin(  2*( (*q0)*(*q2)-(*q3)*(*q1) )  );

            }


            inline void setYaw(const float* yaw) {
                m_data.yaw = *yaw;
            }


            inline void setYaw(const float* q0, const float* q1, 
                    const float* q2, const float* q3) {

                m_data.yaw = atan2(  2*( (*q0)*(*q3)+(*q1)*(*q2) ), 
                        1-2*( (*q2)*(*q2)+(*q3)*(*q3) )  );

            }


            inline void setRollSpeed(const float* wx) {
                m_data.rollspeed = *wx;
            }


            inline void setPitchSpeed(const float* wy) {
                m_data.pitchspeed = *wy;
            }


            inline void setYawSpeed(const float* wz) {
                m_data.yawspeed = *wz;
            }


        private:
            mavlink_attitude_t m_data;


    };

} //namespace dji2mav


#endif
