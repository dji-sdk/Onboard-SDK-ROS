#ifndef _DJI_LIB_ROS_ADAPTER_H_
#define _DJI_LIB_ROS_ADAPTER_H_


#include "DJI_HardDriver_Manifold.h"
#include <dji_sdk_lib/DJI_API.h>
#include <dji_sdk_lib/DJI_Flight.h>
#include <dji_sdk_lib/DJI_Camera.h>
#include <dji_sdk_lib/DJI_VirtualRC.h>
#include <dji_sdk_lib/DJI_WayPoint.h>
#include <dji_sdk_lib/DJI_HotPoint.h>
#include <dji_sdk_lib/DJI_Follow.h>

#include <ros/ros.h>
#include <stdlib.h>
#include <string>
#include <pthread.h>
#include <functional>

namespace DJI {

namespace onboardSDK {


class ROSAdapter {
    public:
        ROSAdapter() {
        }


        ~ROSAdapter() {
            delete flight;
            delete camera;
            delete virtualRC;
            delete waypoint;
            delete hotpoint;
            delete followme;
            delete coreAPI;
            delete m_hd;
        }


        static void* APIRecvThread(void* param) {
            CoreAPI* p_coreAPI = (CoreAPI*)param;
            while(true) {
                p_coreAPI->readPoll();
                p_coreAPI->sendPoll();
                usleep(1000);
            }
        }


        static void broadcastCallback(CoreAPI *coreAPI, Header *header, void *userData) {
            ( (ROSAdapter*)userData )->m_broadcastCallback();
        }

		static void fromMobileCallback(CoreAPI *coreAPI, Header *header, void *userData) {
			uint8_t *data = ((unsigned char*) header) + sizeof(Header) + SET_CMD_SIZE;
			uint8_t len = header->length - SET_CMD_SIZE - EXC_DATA_SIZE;
            ( (ROSAdapter*)userData )->m_fromMobileCallback(data, len);
		}

		static void missionStatusCallback(CoreAPI *coreAPI, Header *header, void *userData) {
			uint8_t *data = ((unsigned char*) header) + sizeof(Header) + SET_CMD_SIZE;
			uint8_t len = header->length - SET_CMD_SIZE - EXC_DATA_SIZE;
            ( (ROSAdapter*)userData )->m_missionStatusCallback(data, len);
		}

		static void missionEventCallback(CoreAPI *coreAPI, Header *header, void *userData) {
			uint8_t *data = ((unsigned char*) header) + sizeof(Header) + SET_CMD_SIZE;
			uint8_t len = header->length - SET_CMD_SIZE - EXC_DATA_SIZE;
            ( (ROSAdapter*)userData )->m_missionEventCallback(data, len);
		}

        void init(std::string device, unsigned int baudrate) {
            printf("--- Connection Info ---\n");
            printf("Serial port: %s\n", device.c_str());
            printf("Baudrate: %u\n", baudrate);
            printf("-----\n");

            m_hd = new HardDriver_Manifold(device, baudrate);
            m_hd->init();

            coreAPI = new CoreAPI( (HardDriver*)m_hd );
            //no log output while running hotpoint mission
            coreAPI->setHotPointData(false);

            flight = new Flight(coreAPI);
            camera = new Camera(coreAPI);
            virtualRC = new VirtualRC(coreAPI);
            waypoint = new WayPoint(coreAPI);
            hotpoint = new HotPoint(coreAPI);
            followme = new Follow(coreAPI);

            int ret;
            ret = pthread_create(&m_recvTid, 0, APIRecvThread, (void *)coreAPI);
            if(0 != ret)
                ROS_FATAL("Cannot create new thread for readPoll!");
            else
                ROS_INFO("Succeed to create thread for readPoll");

            coreAPI->getSDKVersion();
        }


        void activate(ActivateData *data, CallBack callback) {
            coreAPI->activate(data, callback);
        }


        template<class T>
        void setBroadcastCallback( void (T::*func)(), T *obj ) {
            m_broadcastCallback = std::bind(func, obj);
            coreAPI->setBroadcastCallback(&ROSAdapter::broadcastCallback, (UserData)this);
        }

		template<class T>
		void setFromMobileCallback( void (T::*func)(uint8_t *, uint8_t), T *obj) {
		m_fromMobileCallback = std::bind(func, obj, std::placeholders::_1, std::placeholders::_2);
		coreAPI->setFromMobileCallback(&ROSAdapter::fromMobileCallback, (UserData)this);
		}

		template<class T>
		void setMissionStatusCallback( void (T::*func)(uint8_t *, uint8_t), T *obj) {
		m_missionStatusCallback= std::bind(func, obj, std::placeholders::_1, std::placeholders::_2);
		coreAPI->setWayPointCallback(&ROSAdapter::missionStatusCallback, (UserData)this);
		}

		template<class T>
		void setMissionEventCallback( void (T::*func)(uint8_t *, uint8_t), T *obj) {
		m_missionEventCallback = std::bind(func, obj, std::placeholders::_1, std::placeholders::_2);
		coreAPI->setWayPointEventCallback(&ROSAdapter::missionEventCallback, (UserData)this);
		}

/*
        BroadcastData getBroadcastData() {
            return coreAPI->getBroadcastData();
        }
*/

		void sendToMobile(uint8_t *data, uint8_t len){
			coreAPI->sendToMobile(data, len, NULL, NULL);
		}

        void usbHandshake(std::string &device) {
            m_hd->usbHandshake(device);
        }


        //pointer to the lib API
        CoreAPI *coreAPI;
        Flight *flight;
        Camera *camera;
        VirtualRC *virtualRC;
        WayPoint *waypoint;
        HotPoint *hotpoint;
        Follow *followme;


    private:
        HardDriver_Manifold *m_hd;

        pthread_t m_recvTid;

        std::function<void()> m_broadcastCallback;

		std::function<void(uint8_t*, uint8_t)> m_fromMobileCallback;

		std::function<void(uint8_t*, uint8_t)> m_missionStatusCallback;

		std::function<void(uint8_t*, uint8_t)> m_missionEventCallback;
};


} //namespace onboardSDK

} //namespace dji


#endif
