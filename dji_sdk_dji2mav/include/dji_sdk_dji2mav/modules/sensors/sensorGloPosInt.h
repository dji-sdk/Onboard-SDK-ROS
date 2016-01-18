/*****************************************************************************
 * @Brief     Storage GPS data and provide setter and getter methods
 * @Version   0.3.0
 * @Author    Chris Liu
 * @Created   2015/11/20
 * @Modified  2015/12/25
 *****************************************************************************/

#ifndef _DJI2MAV_SENSORGLOPOSINT_H_
#define _DJI2MAV_SENSORGLOPOSINT_H_


namespace dji2mav {

    class SensorGloPosInt {
        public:
            SensorGloPosInt() {
            }


            ~SensorGloPosInt() {
            }


            inline mavlink_global_position_int_t* getDataPtr() {
                return &m_data;
            }


            inline void setTimeBootMs(const int32_t* ts) {
                m_data.time_boot_ms = *ts;
            }


            inline void setLat(const double* lat) {
                m_data.lat = (int) (*lat * 1e7);
            }


            inline void setLat(const int32_t* lat) {
                m_data.lat = *lat;
            }


            inline void setLon(const double* lon) {
                m_data.lon = (int) (*lon * 1e7);
            }


            inline void setLon(const int32_t* lon) {
                m_data.lon = *lon;
            }


            inline void setAlt(const float* alt) {
                m_data.alt = (int) (*alt * 1e7);
            }


            inline void setAlt(const int32_t* alt) {
                m_data.alt = *alt;
            }


            inline void setRelativeAlt(const float* relativeAlt) {
                m_data.relative_alt = (int) (*relativeAlt * 1e7);
            }


            inline void setRelativeAlt(const int32_t* relativeAlt) {
                m_data.relative_alt = *relativeAlt;
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
            mavlink_global_position_int_t m_data;


    };

} //namespace dji2mav


#endif
