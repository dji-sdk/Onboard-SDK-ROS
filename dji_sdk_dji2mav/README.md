# DJI SDK DJI2MAV 0.2.1
##### A package for connecting onboard computer with ground control station using mavlink protocal

---
Version: 0.2.1

Author: Chris Liu

Updated Date: 2015/11/26

---


## Introduction
This package is designed as a library and can be included in various platforms. It is implemented in C++ and depended on mavlink library. A simple ROS node is provided to bringup the dji2mav package.

Dji2mav connects the onboard computer with ground control station using UDP. And it is required to call functions to send heartbeat and sensors data. All the mavlink status and message encode/decode will be handled inside the dji2mav package. So far it is tested on Ubuntu 14.0 with ROS indigo.


## Quick Start
Since a ROS node is provided inside this package, it is easy to put the dji2mav to use.

#### 1. Connection
Connect onboard computer with your ground control station. Any ground control station using mavlink protocol is fitted. Please do sufficient tests on simulator before a real flight. We will be appreciated if you put forward any bug.

Make sure the onboard computer and ground control station are in the same LAN and can successfully ping each other.

#### 2. Launch
Firstly, fire up dji sdk main launch:
```
roslaunch dji_sdk sdk_manifold.launch
```
Record the IP and port of ground control station, modified the corresponding param in *dji2mav_bringup.launch*.
Then run:
```
roslaunch dji_sdk_dji2mav dji2mav_bringup.launch
```
If the output of this ROS node prompts that every thing goes okay, you can see your ground control station has received the heartbeat and some sensors data from the vehicle. Now you can do some simple waypoint test on simulator.


## Functional Specification
In waypoint function module, users can set latitude, longitude, altitude, heading and staytime for the waypoint. Users can upload waypoint list to the vehicle. The vehicle will wait till a current target is set by the users in the ground control station. The vehicle will directly go to that target waypoint and automatically carry out the rest of the waypoint list mission.


## Architecture
The dji2mav is designed to meet multi ground control stations and easy expansibility demands. Some classes are designed as lazy-mode singleton.

![dji2mav architecture](/dji_sdk_dji2mav/doc/img/arch.png?raw=true)

#### 1. Setup
It is easy to access dji2mav interface by getting instance of Config. There are two important methods in config:
```
/**
 * @brief  Set the mavlink system id and the size of GCS list
 * @param  mavSysid : The system id of this vehicle
 * @param  num      : The number of Ground Control Stations
 * @return True if succeed or false if fail
 */
bool setup(uint8_t mavSysid, uint16_t num);

/**
 * @brief  Establish the connection of specific GCS
 * @param  gcsIdx         : The index of the GCS. Begin from 0
 * @param  gcsIP          : The IP of GCS
 * @param  gcsPort        : Connection port of GCS
 * @param  locPort        : Localhost port
 * @param  senderListSize : Default 256
 * @param  sendBufSize    : Default 1024
 * @param  recvBufSize    : Default 4096
 * @return True if succeed or false if fail
 */
bool start(uint16_t gcsIdx, std::string gcsIP, 
        uint16_t gcsPort, uint16_t locPort, 
        uint16_t senderListSize = DEFAULT_SENDER_LIST_SIZE, 
        uint16_t sendBufSize = DEFAULT_SEND_BUF_SIZE, 
        uint16_t recvBufSize = DEFAULT_RECV_BUF_SIZE);
```
This two methods must be called **BEFORE** any data transporting between ground control station and onboard computer. They can be called like this:
```
/* Set the ID of system "1". There is only one ground control system so the number of GCS is also "1" */
dji2mav::Config::getInstance()->setup(1, 1);
/* The index of first GCS is "0". Set the first GCS IP and port */
dji2mav::Config::getInstance()->start(0, targetIp1, (uint16_t)targetPort1, (uint16_t)srcPort);
```

#### 2. Send Heartbeat
To send heartbeat to all GCS(ground control station), use:
```
dji2mav::MavHeartbeat::getInstance()->sendHeartbeat();
```
To send heartbeat to specific GCS, use the corresponding GCS index:
```
dji2mav::MavHeartbeat::getInstance()->sendHeartbeat(gcsIdx);
```
The send buffer of heartbeat module is particular. So it is safe to be called in a thread that periodically send heartbeat and it is recommanded to do so.

#### 3. Update Sensors Data
The sending process of sensors data is similar to the heartbeat module. To update the data, please use setters. All the data are stored inside the relative sensor class.

Take ROS platform as an example. Since all the sensors data are published to specific topics, we can update data in the callback function:
```
void locPosCB(const dji_sdk::LocalPosition &msg) {
    dji2mav::MavSensors::getInstance()->setLocalPosition(&msg.ts, &msg.x, 
            &msg.y, &msg.z);
}

void velCB(const dji_sdk::Velocity &msg) {
    dji2mav::MavSensors::getInstance()->setVelocity(&msg.ts, &msg.vx, 
            &msg.vy, &msg.vz);
}

void attCB(const dji_sdk::AttitudeQuaternion &msg) {
    dji2mav::MavSensors::getInstance()->setAttitudeQuaternion(&msg.ts, 
            &msg.q0, &msg.q1, &msg.q2, &msg.q3, &msg.wx, &msg.wy, &msg.wz);
}

void gloPosCB(const dji_sdk::GlobalPosition &msg) {
    dji2mav::MavSensors::getInstance()->setGlobalPosition(&msg.ts, 
            &msg.latitude, &msg.longitude, &msg.altitude, &msg.height);
}
```

#### 4. Use Waypoint Module
Once waypoint module has been launched, it is ready to receive command from ground control station and reply automatically. All you need to do is keeping the distribution process running(an example of ROS platform):
```
while( ros::ok() ) {
    dji2mav::MavDistributor::getInstance()->distribute();
    ros::Duration(0.1).sleep();
    ros::spinOnce(); // callback function spin
}
```


