/*****************************************************************************
 * @Brief     Functional class for waypoint. Mav-free and ROS-free singleton
 * @Version   0.2.1
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


            inline const WaypointType* getWaypointListDeg() {
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


/*            bool getWaypointRad(uint16_t idx, float &x, float &y, float &z) {
                if( isValidIdx(idx) ) {
                    x = m_wpList_deg[idx].lat / 180.0 * M_PI;
                    y = m_wpList_deg[idx].lon / 180.0 * M_PI;
                    z = m_wpList_deg[idx].alt;
                    ++m_targetIdx;
                    return true;
                } else {
                    printf("Invalid waypoint index %u\n", idx);
                    return false;
                }
            }


            bool getWaypointDeg(uint16_t idx, float &x, float &y, float &z) {
                if( isValidIdx(idx) ) {
                    x = m_wpList_deg[idx].lat;
                    y = m_wpList_deg[idx].lon;
                    z = m_wpList_deg[idx].alt;
                    ++m_targetIdx;
                    return true;
                } else {
                    printf("Invalid waypoint index %u\n", idx);
                    return false;
                }
            }


            bool setWaypointRad(uint16_t idx, float x, float y, float z) {
                if( isValidIdx(idx) ) {
                    m_wpList_deg[idx].lat = x * 180.0 / M_PI;
                    m_wpList_deg[idx].lon = y * 180.0 / M_PI;
                    m_wpList_deg[idx].alt = z;
                    ++m_targetIdx;
                    return true;
                } else {
                    printf("Invalid waypoint index %u\n", idx);
                    return false;
                }
            }


            bool setWaypointDeg(uint16_t idx, float x, float y, float z) {
                if( isValidIdx(idx) ) {
                    m_wpList_deg[idx].lat = x;
                    m_wpList_deg[idx].lon = y;
                    m_wpList_deg[idx].alt = z;
                    ++m_targetIdx;
                    return true;
                } else {
                    printf("Invalid waypoint index %u\n", idx);
                    return false;
                }
            }
*/

            void getWaypointData(uint16_t idx, WaypointType &cmd,
                    float &lat, float &lon, float &alt, float &heading, 
                    float &staytime) {

                cmd = m_wpl_deg[idx].cmd;
		lat = m_wpl_deg[idx].lat;
                lon = m_wpl_deg[idx].lon;
                alt = m_wpl_deg[idx].alt;
                heading = m_wpl_deg[idx].heading;
                staytime = m_wpl_deg[idx].staytime;

            }


            void setWaypointData(uint16_t idx, WaypointType cmd, 
                    float lat, float lon, float alt, float heading, 
                    float staytime) {

                m_wpList_deg[idx].cmd = cmd;
                m_wpList_deg[idx].lat = lat;
                m_wpList_deg[idx].lon = lon;
                m_wpList_deg[idx].alt = alt;
                m_wpList_deg[idx].heading = heading;
                m_wpList_deg[idx].staytime = staytime;

            }


            inline WaypointType getWaypointCmd(uint16_t idx) {
                return m_wpList_deg[idx].cmd;
            }


            inline void setWaypointCmd(uint16_t idx, WaypointCmd cmd) {
                m_wpList_deg[idx].cmd = cmd;
            }


            inline float getWaypointLat(uint16_t idx) {
                return m_wpList_deg[idx].lat;
            }


            inline void setWaypointLat(uint16_t idx, float lat) {
                m_wpList_deg[idx].lat = lat;
            }


            inline float getWaypointLon(uint16_t idx) {
                return m_wpList_deg[idx].lon;
            }


            inline void setWaypointLon(uint16_t idx, float lon) {
                m_wpList_deg[idx].lon = lon;
            }


            inline float getWaypointAlt(uint16_t idx) {
                return m_wpList_deg[idx].alt;
            }


            inline void setWaypointAlt(uint16_t idx, float alt) {
                m_wpList_deg[idx].alt = alt;
            }


            inline float getWpHeading(uint16_t idx) {
                return (int16_t)m_wpList_deg[idx].heading;
            }


            inline void setWpHeading(uint16_t idx, float heading) {
                m_wpList_deg[idx].heading = heading;
            }


            inline float getWpStaytime(uint16_t idx) {
                return (uint16_t)m_wpList_deg[idx].staytime;
            }


            inline void setWpStaytime(uint16_t idx, float staytime) {
                m_wpList_deg[idx].staytime = staytime;
            }


            bool clearMission() {
                m_listSize = 0;
                m_targetIdx = -1;
            }


            void displayMissionRad() {
                printf("Display the full mission:\n");
                for(uint16_t i = 0; i < m_listSize; ++i) {
                    printf("%d: %f, %f, %f\n", i, 
                            m_wpList_deg[i].lat / 180.0 * M_PI, 
                            m_wpList_deg[i].lon / 180.0 * M_PI, 
                            m_wpList_deg[i].alt);
                }
                printf("--- End of display ---\n\n");
            }


            void displayMissionDeg() {
                printf("Display the full mission:\n");
                for(uint16_t i = 0; i < m_listSize; ++i) {
                    printf("%d: %f, %f, %f\n", i, 
                            m_wpList_deg[i].lat, m_wpList_deg[i].lon, 
                            m_wpList_deg[i].alt);
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
