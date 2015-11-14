/*****************************************************************************
 * @Brief     Functional class for waypoint. Mav-free and ROS-free singleton
 * @Version   1.0
 * @Author    Chris Liu
 * @Created   2015/11/11
 * @Modified  2015/11/11
 *****************************************************************************/

#ifndef _MAVWAYPOINT_H_
#define _MAVWAYPOINT_H_


#include <new>

namespace dji2mav {

    class MavWaypoint {
        public:
            /**
             * @brief   Lazy mode singleton
             * @return  The only instance of MavWaypoint
             * @warning UNSAFE FOR MULTI-THREAD!
             */
            static MavWaypoint* getInstance() {
                if(NULL == m_instance) {
                    try {
                        m_instance = new MavWaypoint;
                    } catch(std::bad_alloc &m) {
                        std::cerr << "New instance of MavWaypoint fail: " 
                                << "at line: " << __LINE__ << ", func: " 
                                << __func__ << ", file: " << __FILE__ 
                                << std::endl;
                        perror( m.what() );
                        exit(EXIT_FAILURE);
                    }
                }
                return m_instance;
            }


            bool getListLen(int &len) {
                len = m_listLen;
                return true;
            }


            bool setListLen(const int len) {
                m_listLen = len;
                m_targetIdx = 0;
                return true;
            }


            bool getTargetIdx(int &idx) {
                idx = m_targetIdx;
                return true;
            }


            bool getWaypoint(int idx, float &x, float &y, float &z) {
                x = m_waypointList[idx][0];
                y = m_waypointList[idx][1];
                z = m_waypointList[idx][2];
                return true;
            }


            bool setWaypoint(int idx, float x, float y, float z) {
                m_waypointList[idx][0] = x;
                m_waypointList[idx][1] = y;
                m_waypointList[idx][2] = z;
                m_targetIdx++;
                return true;
            }


            bool getNextIdx() {
                if(m_targetIdx >= m_listLen)
                    return false;
                else
                    return true;
            }


            bool clearMission() {
                m_targetIdx = -1;
            }


            void displayMission() {
                printf("Display the full mission:\n");
                for(int i = 0; i < m_listLen; ++i)
                    printf("%d: %f, %f, %f\n", i, m_waypointList[i][0], m_waypointList[i][1], m_waypointList[i][2]);
                printf("--- End of display ---\n\n");
            }


        private:
            MavWaypoint() {
                m_listLen = 0;
                m_targetIdx = -1;
                m_status = idle;
            }


            ~MavWaypoint() {
            }


            static MavWaypoint* m_instance;
            //TODO: should dynamically alloc memory?
            double m_waypointList[100][3];
            int m_listLen;
            int m_targetIdx;
            enum {idle, read, written, ready, executing, paused, error} 
                    m_status;

    };

    MavWaypoint* MavWaypoint::m_instance = NULL;

}


#endif
