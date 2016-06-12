##DJI Onboard SDK ROS Core Package

This package is the core package handling the requests and responses of Matrice 100.

It also wrappered a `dji_drone.h` class, which user can include and use it directly for his own purpose.
Also, there is a python version `dji_drone.py` in `src/dji_sdk`

Please check the [Appendix](../dji_sdk_doc/Appendix.md) for the detail of all published topic, services and actions.

###Directory Structure
* include/dji_sdk: header files
* include/dji_sdk/DJI_LIB: DJI Onboard SDK API library
* action: ROS action files
* launch: ROS launch files
* msg: ROS message files
* src: source code
* src/module: implementation of action/service/publisher
* srv: ROS service files

###How to use
1. Install and configure your hardware correctly.
2. Enter the following info into *dji_sdk/launch/sdk_manifold.launch*.
	* Drone Version ("M100" or "A3")
	* APP ID
	* APP Level
	* Communication Key
	* UART device name
	* Baudrate
3. Use `roslaunch dji_sdk sdk_manifold.launch` to start the core node.

