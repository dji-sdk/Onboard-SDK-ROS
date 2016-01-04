#ifndef _DJI_LIB_ROS_ADAPTER_H_
#define _DJI_LIB_ROS_ADAPTER_H_


#include "DJI_HardDriver_Manifold.h"
#include "lib/inc/DJI_API.h"
#include "lib/inc/DJI_Flight.h"
#include "lib/inc/DJI_Camera.h"
#include "lib/inc/DJI_VirtualRC.h"
#include "lib/inc/DJI_WayPoint.h"
#include "lib/inc/DJI_HotPoint.h"
#include "lib/inc/DJI_Follow.h"

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
            }
        }


        static void broadcastCallback(CoreAPI *coreAPI, Header *header, void *userData) {
            ( (ROSAdapter*)userData )->m_broadcastCallback();
        }


        void init(std::string device, unsigned int baudrate) {
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

            coreAPI->getVersion();
        }


/*
        void activate(ActivateData *data, CallBack callback) {
            coreAPI->activate(data, callback);
        }
*/


        template<class T>
        void setBroadcastCallback( void (T::*func)(), T *obj ) {
            m_broadcastCallback = std::bind(func, obj);
            coreAPI->setBroadcastCallback(&ROSAdapter::broadcastCallback, (UserData)this);
        }


/*
        BroadcastData getBroadcastData() {
            return coreAPI->getBroadcastData();
        }
*/


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


};


} //namespace onboardSDK

} //namespace dji


#endif
