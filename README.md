##DJI Onboard SDK ROS Packages

----

This is a ROS package for DJI OnBoard SDK.

It helps users handle the following commands and actions.

* The activation
* The flight control obtainment
* The flight control release
* The take off procedure
* The landing procedure
* The go home procedure
* Example for the gimbal control
* Example for the attitude control
* The photo taking procedure
* The start/stop video recording procedure
* Local navigation (fly into a certain (X,Y,Z))
* GPS navigation (fly into a certain GPS coordinate)
* Waypoint navigation (fly through a series of GPS coordinates)
* Using WebSocket together with Baidu Map for navigation 
* Using MAVLink protocol and QGroundStation (TODO)

We also provides a ROS package 

###How to use
1. Install and configure your hardware correctly.
2. Enter the following info into *dji_sdk/launch/sdk_manifold.launch*.
	* APP ID
	* APP Level
	* Communication Key
	* uart device name
	* baudrate
3. Use `roslaunch dji_sdk sdk_manifold.launch` to start the core node.
4. Include the `dji_drone.h` from `dji_sdk/include/dji_sdk` into your package and run it. (there also provides a python version `dji_drone.py` in `dji_sdk/src/dji_sdk`)


###System Structure
* [dji_sdk](dji_sdk): the core package handling the communication with Matrice 100, which provides a header file `dji_drone.h` for future use
* [dji_sdk_demo](dji_sdk_demo): an example package of using `dji_drone.h` to control the Matrice 100
* [dji_sdk_web_groundstation](dji_sdk_web_groundstation): a WebSocket example using ROS-bridge-suite, where a webpage groundstatino is provided
* [dji_sdk_manifold_read_cam](dji_sdk_manifold_read_cam): a specifed X3 video reading package for Manifold, video stream will be published out in RGB, CATKIN_IGNOREd by defualt
* [dji_sdk_manifold_read_cam_nv](dji_sdk_manifold_read_cam_nv): same as the previous one, but use hardware decoding method and in Grayscale, CATKIN_IGNOREd by defualt
* [dji_sdk_doc](dji_sdk_doc): all documents

![image](../dji_sdk_doc/structure.jpg)

###System Environment
The below environment has been tested.
* Operating System: Ubuntu 14.04, Manifold
* ROS version: ROS Indigo