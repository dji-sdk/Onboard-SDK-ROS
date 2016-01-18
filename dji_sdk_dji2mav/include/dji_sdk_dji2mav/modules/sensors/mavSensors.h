/*****************************************************************************
 * @Brief     Sensors module. Handler all the sensors date. ROS-free singleton
 * @Version   0.3.0
 * @Author    Chris Liu
 * @Created   2015/11/17
 * @Modified  2015/12/25
 *****************************************************************************/

#ifndef _MAV2DJI_MAVSENSORS_H_
#define _MAV2DJI_MAVSENSORS_H_


#include <mavlink/common/mavlink.h>
#include <new>
#include <string>
#include <cstdarg>

#include "dji_sdk_dji2mav/modules/mavModule.h"
#include "sensorLocPosNed.h"
#include "sensorAtt.h"
#include "sensorGloPosInt.h"
#include "dji_sdk_dji2mav/log.h"

namespace dji2mav{

    class MavSensors : public MavModule {
        public:
            /**
             * @brief Constructor for Sensor Module. Recv buf size 1024
             * @param handler : The reference of MavHandler Object
             * @param name    : The name of this module
             * @param gcsNum  : The number of GCS that the module employs
             * @param ...     : The indexes of GCS list, should be uint16_t
             */
            MavSensors(MavHandler &handler, std::string name, 
                    uint16_t gcsNum, ...) : MavModule(handler, name, 1024) { 

                DJI2MAV_DEBUG("Going to construct Sensors module with name "
                        "%s and gcsNum %u...", name.c_str(), gcsNum);

                setSensorsHook(NULL);

                va_list arg;
                va_start(arg, gcsNum);
                if(1 == gcsNum) {
                    setMasterGcsIdx( (uint16_t)va_arg(arg, int) );
                } else {
                    for(uint16_t i = 0; i < gcsNum; ++i) {
                        employGcsSender( (uint16_t)va_arg(arg, int) );
                    }
                }
                va_end(arg);

                //TODO: Also set the MOI here

                DJI2MAV_DEBUG("...finish constructing Sensors module.");

            }


            ~MavSensors() {
                DJI2MAV_DEBUG("Going to destruct Sensors module...");
                DJI2MAV_DEBUG("...finish destructing Sensors module.");
            }


            /**
             * @brief Update the local position data
             * @param ts : Pointer of timestamp from boot
             * @param x  : Pointer of local position x value
             * @param y  : Pointer of local position y value
             * @param z  : Pointer of local position z value
             */
            void setLocalPosition(const int32_t* ts, const float* x, 
                    const float* y, const float* z) {

                m_locPos.setTimeBootMs(ts);
                m_locPos.setX(x);
                m_locPos.setY(y);
                m_locPos.setZ(z);

            }


            /**
             * @brief Update the velocity data
             * @param ts : Pointer of timestamp from boot
             * @param vx : Pointer of x axis velocity value
             * @param vy : Pointer of y axis velocity value
             * @param vz : Pointer of z axis velocity value
             */
            void setVelocity(const int32_t* ts, const float* vx, 
                    const float* vy, const float* vz) {

                m_gloPos.setTimeBootMs(ts);
                m_gloPos.setVx(vx);
                m_gloPos.setVy(vy);
                m_gloPos.setVz(vz);

            }


            /**
             * @brief Update the velocity data
             * @param ts : Pointer of timestamp from boot
             * @param vx : Pointer of x axis velocity value
             * @param vy : Pointer of y axis velocity value
             * @param vz : Pointer of z axis velocity value
             */
            void setAttitudeQuaternion(const int32_t* ts, const float* q0, 
                    const float* q1, const float* q2, const float* q3, 
                    const float* wx, const float* wy, const float* wz) {

                m_att.setTimeBootMs(ts);
                m_att.setRoll(q0, q1, q2, q3);
                m_att.setPitch(q0, q1, q2, q3);
                m_att.setYaw(q0, q1, q2, q3);
                m_att.setRollSpeed(wx);
                m_att.setPitchSpeed(wy);
                m_att.setYawSpeed(wz);

            }


            /**
             */
            void setGlobalPosition(const int32_t* ts, const double* lat, 
                    const double* lon, const float* alt, const float* height) {

                m_gloPos.setTimeBootMs(ts);
                m_gloPos.setLat(lat);
                m_gloPos.setLon(lon);
                m_gloPos.setAlt(alt);
                m_gloPos.setRelativeAlt(height);

            }


            /**
             * @brief  Send sensors data to specific GCS. Compid is set to IMU
             * @param  gcsIdx : The index of GCS
             * @return True if succeed or false if fail
             */
            bool sendSensorsDataToGcs(uint16_t gcsIdx) {
                bool ret = true;
                mavlink_msg_local_position_ned_encode( getMySysid(), 
                        MAV_COMP_ID_IMU, &m_sendLocPosMsg, 
                        m_locPos.getDataPtr() );
                if( !sendMsgToGcs(gcsIdx, m_sendLocPosMsg) ) {
                    DJI2MAV_ERROR("Fail to send Sensor data locPosNed to GCS " 
                            "#%u!", gcsIdx);
                    ret = false;
                }

                mavlink_msg_attitude_encode( getMySysid(), 
                        MAV_COMP_ID_IMU, &m_sendAttMsg, m_att.getDataPtr() );
                if( !sendMsgToGcs(gcsIdx, m_sendAttMsg) ) {
                    DJI2MAV_ERROR("Fail to send Sensor data attitude to GCS " 
                            "#%u!", gcsIdx);
                    ret = false;
                }

                mavlink_msg_global_position_int_encode( getMySysid(), 
                        MAV_COMP_ID_IMU, &m_sendGloPosMsg, 
                        m_gloPos.getDataPtr() );
                if( !sendMsgToGcs(gcsIdx, m_sendGloPosMsg) ) {
                    DJI2MAV_ERROR("Fail to send Sensor data GloPosInt to GCS " 
                            "#%u!", gcsIdx);
                    ret = false;
                }

                return ret;
            }


            /**
             * @brief  Send sensors data to all GCS
             * @return True if succeed or false if fail
             */
            bool sendSensorsDataToAll() {
                bool ret = true;
                mavlink_msg_local_position_ned_encode( getMySysid(), 
                        MAV_COMP_ID_IMU, &m_sendLocPosMsg, 
                        m_locPos.getDataPtr() );
                if( !sendMsgToAll(m_sendLocPosMsg) ) {
                    DJI2MAV_ERROR("Fail to send Sensor data locPosNed to some " 
                            "GCS!");
                    ret = false;
                }

                mavlink_msg_attitude_encode( getMySysid(), 
                        MAV_COMP_ID_IMU, &m_sendAttMsg, m_att.getDataPtr() );
                if( !sendMsgToAll(m_sendAttMsg) ) {
                    DJI2MAV_ERROR("Fail to send Sensor data attitude to some " 
                            "GCS!");
                    ret = false;
                }

                mavlink_msg_global_position_int_encode( getMySysid(), 
                        MAV_COMP_ID_IMU, &m_sendGloPosMsg, 
                        m_gloPos.getDataPtr() );
                if( !sendMsgToAll(m_sendGloPosMsg) ) {
                    DJI2MAV_ERROR("Fail to send Sensor data GloPosInt to some " 
                            "GCS!");
                    ret = false;
                }

                return ret;
            }
            

            /**
             * @brief  React to sensor data from specific GCS and execute rsp
             * @param  gcsIdx : Get the index of GCS
             * @param  msg    : Get the received message
             */
            void reactToSensors(uint16_t gcsIdx, mavlink_message_t &msg) {
                if(NULL != m_hook) {
                    m_hook();
                }
            }


            /**
             * @brief Set the responser function pointer for the sensors
             * @param The function pointer that is to be set
             */
            inline void setSensorsHook( void (*func)() ) {
                m_hook = func;
            }


            /**
             * @brief Implement the messages passively handling function
             * @param msg : The reference of received message
             */
            void passivelyReceive(mavlink_message_t &msg) {
            }


            /**
             * @brief Implement the messages actively sending function
             */
            void activelySend() {
                sendSensorsDataToAll();
                usleep(20000); //50Hz
            }


        private:
            mavlink_message_t m_sendLocPosMsg;
            mavlink_message_t m_sendAttMsg;
            mavlink_message_t m_sendGloPosMsg;

            SensorLocPosNed m_locPos;
            SensorAtt m_att;
            SensorGloPosInt m_gloPos;

            void (*m_hook)();


    };

} //namespace dji2mav


#endif
