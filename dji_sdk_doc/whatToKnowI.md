#DJI SDK Challenge: Onboard SDK Part I

## Overview
Onboard SDK is a protocol provided by DJI, which make it possible for developers to get the current state of the drone and send commands to it in order to control drone’s movement.

This document is a brief introduction of the structure and usage of SDK’s ROS package. The package structure and usage will be introduced first, then followed by some tips of SDK challenges. At last, there will be an appendix of all msgs and srcs, together with the detailed parameters of them.

## ROS Package
The ROS packages in this repository is one kind of implementation of DJI Onboard SDK protocol and many applications based on it.

- dji\_sdk\_lib: The implementation of protocol communication. It is the core part of protocol implementation outside ROS. The code is almost the same as the one in [official github repository](https://github.com/dji-sdk/Onboard-SDK/tree/3.1/lib) but in ROS format.
- dji\_sdk: This package is a group of packed APIs from dji\_sdk\_lib. All received data from the drone are published into topics and all protocols of sending commands have become ROS services. Other than the logic of processing topics and services, we provided a header file `dji_drone.h` in the include folder, which is a packed class able to access all topics and services. Developers can implement their logic in another package then include this header file to access all data.
- dji\_sdk\_demo: A demo of how to use `dji_drone.h` in dji\_sdk package.
- dji\_sdk\_dji2mav: An extended package of dji\_sdk, which converts messages between DJI protocol and MAVLink protocol. It may be useful to developers who wants do work on the QGroundControl.  Please refer to its [readme](https://github.com/dji-sdk/Onboard-SDK-ROS/blob/3.1/dji_sdk_dji2mav/README.md) for more details.
- dji\_sdk\_web\_groundstation: A grundstation package using ROS Javascript API (ROS bridge suite) and Baidu Map API.
- dji\_sdk\_read\_cam: A specified ROS package for Manifold. It converts the video stream from X3 gimbal into the ROS sensor\_msgs/Image and publish the video out.

All these packages are designed as a reference purpose, which means they are not suitable for an application usage, they are just various demos as your reference.
For example, the dji\_sdk package implements all protocols as ROS element, however, some of them may be not necessary in a specified task. Developers should remove unnecessary topics or services (like everything related to groundstation) if they want to reduce the CPU usage. It is very important because the Manifold has a limited performance.

## Important Notes
As for the SDK challenge details, there are several points developers need to pay attention.

1. The drone is not controllable by SDK during the taking off procedure until its height reaches the 1.2 meter, and the landing procedure will give you the same condition. Therefore, it is better to arm/disarm the drone by yourself, then with the help of attitude control making it take off or land with SDK control ability.
2. There are three control modes in horizontal direction when doing attitude control (regardless the frame). The angle of pitch/roll, the velocity in x/y direction and the position offset in x/y direction. The position control needs a stable GPS signal (health flag \> 3), the velocity control needs a stable velocity feedback (from Guidance or GPS), the pitch/roll angle control is the only way to achieve attitude control when working in weak GPS signal condition with no Guidance installed. 
3. The video transmission in ROS network may suffer latency problem. It is a better idea to read and process the video stream in the same node then publish the final result out.

## Appendix
We provided an [appendix](http://github.com/dji-sdk/Onboard-SDK-ROS/blob/3.1/dji_sdk_doc/Appendix.md) for developers to check the details of all published topics and services, together with the definition of corresponding msgs and srvs with parameter description.