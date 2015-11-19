/*****************************************************************************
 * @Brief     Sensors module. Handler all the sensors date. ROS-free singleton
 * @Version   1.1
 * @Author    Chris Liu
 * @Created   2015/11/17
 * @Modified  2015/11/17
 *****************************************************************************/

#ifndef _MAV2DJI_MAVSENSORS_H_
#define _MAV2DJI_MAVSENSORS_H_


#include "../../mavHandler.h"
#include "sensorLocPosNed.h"
#include "sensorAtt.h"

#include <iostream>
#include <stdio.h>
#include <new>

namespace dji2mav{

    class MavSensors {
        public:
            /**
             * @brief   Get the only instance. Lazy mode singleton
             * @return  The only instance of MavSensors
             * @warning UNSAFE FOR MULTI-THREAD! Should be called BEFORE fork
             */
            static MavSensors* getInstance() {
                if(NULL == m_instance) {
                    try {
                        m_instance = new MavSensors();
                    } catch(std::bad_alloc &m) {
                        std::cerr << "New instance of MavSensors fail: " 
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
             * @brief  Get the senderRecord array address
             * @return The address of senderRecord
             */
            inline const int* getSenderRecord() {
                return m_senderRecord;
            }


            /**
             * @brief  Set the sender index of specific GCS
             * @param  gcsIdx    : The index of GCS
             * @param  senderIdx : The index of sender
             * @return True if succeed or false if fail
             */
            bool setSenderIdx(uint16_t gcsIdx, int senderIdx) {
                if( !m_hdlr->isValidIdx(gcsIdx, senderIdx) )
                    return false;
                m_senderRecord[gcsIdx] = senderIdx;
                return true;
            }


            /**
             * @brief  Register new sender and use it for specific GCS
             * @param  gcsIdx : The index of GCS
             * @return True if succeed or false if fail
             */
            bool applyNewSender(uint16_t gcsIdx) {
                int newSender = m_hdlr->registerSender(gcsIdx);
                if( newSender < 0 ) {
                    printf("Fail to regiser sender for sensors in GCS #%u! "
                            "Did you set sender list too small?\n", gcsIdx);
                    return false;
                }
                m_senderRecord[gcsIdx] = newSender;
                return true;
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

                m_locPos->setTimeBootMs(ts);
                m_locPos->setX(x);
                m_locPos->setY(y);
                m_locPos->setZ(z);

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

                m_locPos->setTimeBootMs(ts);
                m_locPos->setVx(vx);
                m_locPos->setVy(vy);
                m_locPos->setVz(vz);

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

                m_att->setTimeBootMs(ts);
                m_att->setRoll(q0, q1, q2, q3);
                m_att->setPitch(q0, q1, q2, q3);
                m_att->setYaw(q0, q1, q2, q3);
                m_att->setRollSpeed(wx);
                m_att->setPitchSpeed(wy);
                m_att->setYawSpeed(wz);

            }


            /**
             * @brief  Send sensors data to specific GCS. Compid is set to IMU
             * @param  gcsIdx : The index of GCS
             * @return True if succeed or false if fail
             */
            bool sendSensorsData(uint16_t gcsIdx) {
                mavlink_msg_local_position_ned_encode( m_hdlr->getSysid(), 
                        MAV_COMP_ID_IMU, &m_sendMsg, m_locPos->getDataPtr() );
                if( !m_hdlr->sendEncodedMsg(gcsIdx, m_senderRecord[gcsIdx], 
                        &m_sendMsg) ) {
                    printf("Sending locPosNed to GCS #%u fail!\n", gcsIdx);
                    return false;
                }

                mavlink_msg_attitude_encode( m_hdlr->getSysid(), 
                        MAV_COMP_ID_IMU, &m_sendMsg, m_att->getDataPtr() );
                if( !m_hdlr->sendEncodedMsg(gcsIdx, m_senderRecord[gcsIdx], 
                        &m_sendMsg) ) {
                    printf("Sending attitude to GCS #%u fail!\n", gcsIdx);
                    return false;
                }

                return true;

            }


            /**
             * @brief  Send sensors data to all GCS
             * @return True if succeed or false if fail
             */
            inline bool sendSensorsData() {
                bool ret = true;
                for(uint16_t i = 0; i < m_hdlr->getMngListSize(); ++i) {
                    ret &= sendSensorsData(i);
                }
                return ret;
            }
            

            /**
             * @brief  React to sensor data from specific GCS and execute rsp
             * @param  gcsIdx : Get the index of GCS
             * @param  msg    : Get the received message
             */
            void reactToSensors(uint16_t gcsIdx, mavlink_message_t &msg) {
                printf("Ehhh... Generally this should not be called... O_o\n");
                if(NULL != m_rsp) {
                    m_rsp();
                }
            }


            /**
             * @brief Set the responser function pointer for the sensors
             * @param The function pointer that is to be set
             */
            inline void setSensorsRsp( void (*func)() ) {
                m_rsp = func;
            }


            void distructor() {
                delete m_instance;
            }


        private:
            MavSensors() {

                assert(CHAR_BIT * sizeof(float) == 32);
                assert(CHAR_BIT * sizeof(double) == 64);

                m_hdlr = MavHandler::getInstance();

                try {
                    m_senderRecord = new int[m_hdlr->getMngListSize()];
                } catch(std::bad_alloc& m) {
                    std::cerr << "Failed to alloc memory for senderRecord: " 
                            << "at line: " << __LINE__ << ", func: " 
                            << __func__ << ", file: " << __FILE__ 
                            << std::endl;
                    perror( m.what() );
                    exit(EXIT_FAILURE);
                }
                for(uint16_t i = 0; i < m_hdlr->getMngListSize(); ++i) {
                    // register new sender for sensor data in every GCS
                    if( applyNewSender(i) < 0)
                        printf("Fail to register new sender for sensors!\n");
                }

                setSensorsRsp(NULL);


                try {
                    m_locPos = new SensorLocPosNed();
                    m_att = new SensorAtt();
                } catch(std::bad_alloc& m) {
                    std::cerr << "Failed to alloc memory for sensor data : " 
                            << "at line: " << __LINE__ << ", func: " 
                            << __func__ << ", file: " << __FILE__ 
                            << std::endl;
                    perror( m.what() );
                    exit(EXIT_FAILURE);
                }

                printf("Succeed to construct Sensors module\n");

            }


            ~MavSensors() {
                if(NULL != m_locPos) {
                    delete m_locPos;
                    m_locPos = NULL;
                }
                if(NULL != m_att) {
                    delete m_att;
                    m_att = NULL;
                }
                m_hdlr = NULL;
                printf("Finish to destruct Sensors module\n");
            }


            static MavSensors* m_instance;
            int* m_senderRecord;

            MavHandler* m_hdlr;

            mavlink_message_t m_sendMsg;
            mavlink_message_t m_recvMsg;
            mavlink_status_t m_recvStatus;

            SensorLocPosNed* m_locPos;
            SensorAtt* m_att;

            void (*m_rsp)();


    };

    MavSensors* MavSensors::m_instance = NULL;

} //namespace dji2mav


#endif
