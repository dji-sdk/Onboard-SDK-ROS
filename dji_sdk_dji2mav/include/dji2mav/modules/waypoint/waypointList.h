/*****************************************************************************
 * @Brief     Functional class for waypoint. Mav-free and ROS-free singleton
 * @Version   1.1
 * @Author    Chris Liu
 * @Created   2015/11/18
 * @Modified  2015/11/18
 *****************************************************************************/

#ifndef _DJI2MAV_WAYPOINTLIST_H_
#define _DJI2MAV_WAYPOINTLIST_H_


#include "waypointType.h"

#include <iostream>
#include <new>
#include <stdio.h>

namespace dji2mav {

    class WaypointList {
        public:
            WaypointList() {
                m_listSize = 0;
                m_targetIdx = -1;
            }


            ~WaypointList() {
            }


            inline const float ( *getWaypointListDeg() )[3] {
                return m_wpList_deg;
            }


            inline uint16_t getListSize() {
                return m_listSize;
            }


            inline void setListSize(const uint16_t size) {
                m_listSize = size;
            }


            inline uint16_t getTargetIdx() {
                return m_targetIdx;
            }


            inline void setTargetIdx(uint16_t targetIdx) {
                m_targetIdx = targetIdx;
            }


            inline void readyToUpload() {
                m_targetIdx = 0;
            }


            inline void readyToDownload() {
                m_targetIdx = 0;
            }


            inline void finishUpload() {
                if(m_targetIdx != m_listSize) {
                    printf("Uploaded waypoint list size doesn't matched!\n");
                }
                m_targetIdx = -1;
            }


            inline bool isDownloadFinished() {
                if(m_targetIdx == m_listSize) {
                    m_targetIdx = -1;
                    return true;
                } else {
                    return false;
                }
            }


            inline bool isValidIdx(uint16_t idx) {
                return (idx < m_listSize) ? true : false;
            }


            bool getWaypointRad(uint16_t idx, float &x, float &y, float &z) {
                if( isValidIdx(idx) ) {
                    x = m_wpList_deg[idx][0] / 180.0 * M_PI;
                    y = m_wpList_deg[idx][1] / 180.0 * M_PI;
                    z = m_wpList_deg[idx][2];
                    ++m_targetIdx;
                    return true;
                } else {
                    printf("Invalid waypoint index %u\n", idx);
                    return false;
                }
            }


            bool getWaypointDeg(uint16_t idx, float &x, float &y, float &z) {
                if( isValidIdx(idx) ) {
                    x = m_wpList_deg[idx][0];
                    y = m_wpList_deg[idx][1];
                    z = m_wpList_deg[idx][2];
                    ++m_targetIdx;
                    return true;
                } else {
                    printf("Invalid waypoint index %u\n", idx);
                    return false;
                }
            }


            bool setWaypointRad(uint16_t idx, float x, float y, float z) {
                if( isValidIdx(idx) ) {
                    m_wpList_deg[idx][0] = x * 180.0 / M_PI;
                    m_wpList_deg[idx][1] = y * 180.0 / M_PI;
                    m_wpList_deg[idx][2] = z;
                    ++m_targetIdx;
                    return true;
                } else {
                    printf("Invalid waypoint index %u\n", idx);
                    return false;
                }
            }


            bool setWaypointDeg(uint16_t idx, float x, float y, float z) {
                if( isValidIdx(idx) ) {
                    m_wpList_deg[idx][0] = x;
                    m_wpList_deg[idx][1] = y;
                    m_wpList_deg[idx][2] = z;
                    ++m_targetIdx;
                    return true;
                } else {
                    printf("Invalid waypoint index %u\n", idx);
                    return false;
                }
            }


            bool setCmdTakeoff(uint16_t idx) {
                if( isValidIdx(idx) ) {
                    m_cmd = takeoff;
                    return true;
                } else {
                    printf("Invalid waypoint index %u\n", idx);
                    return false;
                }
            }


            bool setCmdLand(uint16_t idx) {
                if( isValidIdx(idx) ) {
                    m_cmd = land;
                    return true;
                } else {
                    printf("Invalid waypoint index %u\n", idx);
                    return false;
                }
            }


            bool setCmdGohome(uint16_t idx) {
                if( isValidIdx(idx) ) {
                    m_cmd = gohome;
                    return true;
                } else {
                    printf("Invalid waypoint index %u\n", idx);
                    return false;
                }
            }


            bool setCmdWaypoint(uint16_t idx) {
                if( isValidIdx(idx) ) {
                    m_cmd = waypoint;
                    return true;
                } else {
                    printf("Invalid waypoint index %u\n", idx);
                    return false;
                }
            }


            bool isCmdTakeoff(uint16_t idx) {
                if( isValidIdx(idx) ) {
                    return (m_cmd[idx] == takeoff) ? true : false;
                } else {
                    printf("Invalid waypoint index %u\n", idx);
                    return false;
                }
            }


            bool isCmdLand(uint16_t idx) {
                if( isValidIdx(idx) ) {
                    return (m_cmd[idx] == land) ? true : false;
                } else {
                    printf("Invalid waypoint index %u\n", idx);
                    return false;
                }
            }


            bool isCmdGohome(uint16_t idx) {
                if( isValidIdx(idx) ) {
                    return (m_cmd[idx] == gohome) ? true : false;
                } else {
                    printf("Invalid waypoint index %u\n", idx);
                    return false;
                }
            }


            bool isCmdWaypoint(uint16_t idx) {
                if( isValidIdx(idx) ) {
                    return (m_cmd[idx] == waypoint) ? true : false;
                } else {
                    printf("Invalid waypoint index %u\n", idx);
                    return false;
                }
            }


            bool clearMission() {
                m_listSize = 0;
                m_targetIdx = -1;
            }


            void displayMissionRad() {
                printf("Display the full mission:\n");
                for(uint16_t i = 0; i < m_listSize; ++i) {
                    printf("%d: %f, %f, %f\n", i, 
                            m_wpList_deg[i][0] / 180.0 * M_PI, 
                            m_wpList_deg[i][1] / 180.0 * M_PI, 
                            m_wpList_deg[i][2]);
                }
                printf("--- End of display ---\n\n");
            }


            void displayMissionDeg() {
                printf("Display the full mission:\n");
                for(uint16_t i = 0; i < m_listSize; ++i) {
                    printf("%d: %f, %f, %f\n", i, 
                            m_wpList_deg[i][0], m_wpList_deg[i][1], 
                            m_wpList_deg[i][2]);
                }
                printf("--- End of display ---\n\n");
            }


        private:
            WaypointType m_wpList_deg[100];
            uint16_t m_listSize;
            int m_targetIdx;

    };

}


#endif
