#DJI SDK Challenge: Onboard SDK Part I

## Overview
Onboard SDK is a protocol provided by DJI, which makes it possible for developers to get current state of Matrice 100 and send commands to it.

This document is a brief introduction of the structure and usage of SDK’s ROS packages. The structure and usage of these packages will be introduced at first and then followed by some tips of SDK challenges. At last, there will be an appendix showing all topics and services, together with the correspondsing msgs and srvs.

## ROS Package
The ROS packages in this repository is one kind of implementation of DJI Onboard SDK protocol and many applications based on it.

- dji\_sdk\_lib: The implementation of protocol communication. This package is a protocol implementation outside ROS. The code is almost the same as the one in [official github repository](https://github.com/dji-sdk/Onboard-SDK/tree/3.1/lib) but catkinized.
- dji\_sdk: This package is a group of packed APIs from dji\_sdk\_lib. All received data from the drone are published into ROS topics and all command sending APIs have become ROS services. Other than the logic of processing topics and services, we provided a header file `dji_drone.h` in the include folder as a packed class of subscriber andd service client. Developers are able to access all topics and services simply by this header file.
- dji\_sdk\_demo: A demo of how to use the `dji_drone.h` mentioned above.
- dji\_sdk\_dji2mav: An extended package of dji\_sdk, which converts messages between DJI protocol and MAVLink protocol. It may be useful to developers who want do work on the QGroundControl. Please refer to its [readme](https://github.com/dji-sdk/Onboard-SDK-ROS/blob/3.1/dji_sdk_dji2mav/README.md) for more details.
- dji\_sdk\_web\_groundstation: A grundstation package using ROS Javascript API (ROS bridge suite) and Baidu Map API.
- dji\_sdk\_read\_cam: A specified ROS package for Manifold. It converts the video stream from X3 gimbal into the ROS sensor\_msgs/Image and publish the video out.

All these packages are designed as a reference purpose, which means they are not suitable for an application usage, developers should treat them as demos instead.
For example, the dji\_sdk package implements all protocols as ROS elements, however, some of them may be not necessary in a specified task. Developers should remove unnecessary topics or services (like everything related to groundstation) for reducing CPU usage purpose.

## Important Notes
As for the SDK challenge details, there are several points developers need to pay attention.

1. The drone is not controllable by SDK during the taking off procedure until its reaches the 1.2 meters high, and the landing procedure will bring you the same condition. Therefore, it is better to arm/disarm the drone by yourself, then doing attitude control in order to make it take off and land properly with SDK control ability and your control logic.
2. There are three control modes in horizontal direction when doing attitude control (regardless the frame). The angle of pitch/roll, the velocity in x/y direction and the position offset in x/y direction. The position control needs a stable GPS signal (health flag \> 3), the velocity control needs a stable velocity feedback (from Guidance or GPS), the pitch/roll angle control is the only way to achieve attitude control when working in a weak GPS signal condition with no Guidance installed. 
3. The video transmission in ROS network may suffer latency problem. It is better to read and process the video stream in the same node then publish the final result out.

## Appendix
We provided an [appendix](http://github.com/dji-sdk/Onboard-SDK-ROS/blob/3.1/dji_sdk_doc/Appendix.md) for developers to check the details of all published topics and services, together with the definition of corresponding msgs and srvs with parameter description.
