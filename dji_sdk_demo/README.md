##DJI Onboard SDK ROS Demo Client Package

This package is a demonstration of how to use dji_sdk package and its `dji_drone.h` class as a custom client.

###How to use
1. Install and configure your hardware correctly.
2. Enter the following info into *dji_sdk/launch/sdk_manifold.launch*.
	* APP ID
	* APP Level
	* Communication Key
	* uart device name
	* baudrate
3. Use `roslaunch dji_sdk sdk_manifold.launch` to start the core node.
4. `rosrun dji_sdk_demo dji_sdk_client`

