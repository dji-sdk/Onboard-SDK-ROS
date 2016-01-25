/*****************************************************************************
 * @Brief     Storage locPosNed data and provide setter and getter methods
 * @Version   0.3.0
 * @Author    Chris Liu
 * @Created   2015/11/17
 * @Modified  2015/12/25
 *****************************************************************************/

#ifndef _DJI2MAV_SENSORLOCPOSNED_H_
#define _DJI2MAV_SENSORLOCPOSNED_H_


namespace dji2mav {

    class SensorLocPosNed {
        public:
            SensorLocPosNed() {
            }


            ~SensorLocPosNed() {
            }


            inline const mavlink_local_position_ned_t* getDataPtr() {
                return &m_data;
            }


            inline void setTimeBootMs(const int32_t* ts) {
                m_data.time_boot_ms = *ts;
            }


            inline void setX(const float* x) {
                m_data.x = *x;
            }


            inline void setY(const float* y) {
                m_data.y = *y;
            }


            inline void setZ(const float* z) {
                m_data.z = *z;
            }


            inline void setVx(const float* vx) {
                m_data.vx = *vx;
            }


            inline void setVy(const float* vy) {
                m_data.vy = *vy;
            }


            inline void setVz(const float* vz) {
                m_data.vz = *vz;
            }


        private:
            mavlink_local_position_ned_t m_data;


    };

} //namespace dji2mav


#endif
