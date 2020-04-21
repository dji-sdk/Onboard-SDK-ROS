# DJI Onboard SDK ROS 4.0.0Beta

## Latest Update

OSDK-ROS 4.0.0(beta) was released on 16 April 2020.You need to read newest update below the readme to get update information. 

## newest update  
### feature  
This 4.0(Beta) version releases a feature package: dji_osdk_ros. The package contains two different framework's interface to adapt to two different version of onboard-sdk.  
(__note:We will cancel support for the OSDK-ROS-obsoleted in the next version.__)

|                     | **OSDK-ROS4.0**            | **OSDK-ROS-obsoleted**                |  
|---------------------|----------------------------|---------------------------------------|
| **onboard-sdk4.0**  |files belows in dji_osdk_ros| not support                           |
| **onboard-sdk3.9**  | not support                | files belows in dji_osdk_ros_obsoleted|

The update mainly includes:  
1. Redesigned the 4.0 version of the framework (ROS side interacts with the OSDK side through the wrapper layer, business-related interfaces are fully encapsulated into the wrapper layer, and the ROS side provides all services and topics);  
2. Version 4.0 opens a main node (dji_vehicle_node), which completes the activation of the drone, the acquisition of control rights, and the initialization of all services and topics;  

|  **services**                                    | **topics**                            | **nodes(demo)**                | 
|--------------------------------------------------|-------------------------------------- |--------------------------------|
|flight_task_control                               |cameraData                             |flight_control_node             |
|gimbal_task_control(need update,may not work now) |waypoint(will support in next version) |advanced_sensing_node(not test) |
|camera_task_control                               |                                       |gimbal_camera_control_node      |
|advanced_sensing                                  |                                       |mfio_conrol_node(not test)      |
|set_go_home_altitude                              |
|set_current_point_as_home                         |
|mfio_control(not test)                            |
|enable_avoid(not test)                            |
|waypoint(will support in next version)            |
 
3. At the same time, we keeped all services and topics of osdk-ros 3.8.1 which compatible with onboard-sdk3.9. If you want to use these interfaces,you need to run dji_sdk_node and use it's services and topics. 
(__note: they will not be supported in next osdk-ros version.__)
### Prerequisites
We have tested the library in Ubuntu 18.04, but it should be easy to compile in other platforms.
#### Ros  
you need to install ros first.Install instruction can be found at: http://wiki.ros.org/ROS/Installation. We just test ROS Melodic Morenia version.  
#### C++14 Compiler
We compile with C + + 14 Standard
#### djiosdk-core
you need to download onboard-sdk,and install osdk-core.
>$mkdir build  
>$cd build  
>$cmake..  
>$sudo make -j7 install
#### nema-comms
> $sudo apt install ros-{release}-nmea-comms  

__note:we only test on melodic,but it should be support on other version.__
#### ffmpeg
> $sudo apt install ffmpeg  
#### libusb-1.0-0-dev
> $sudo apt install libusb-1.0-0-dev
#### opencv3.x
We use OpenCV to show images from camera stream. Dowload and install instructions can be found at: http://opencv.org. Tested with OpenCV 3.2.0.

###  Building dji_osdk_ros pkg
#### create workspace
If you don't have a catkin workspace, create one as follows:
>$mkdir catkin_ws  
>$cd catkin_ws  
>$mkdir src  
>$cd src  
>$catkin_init_workspace
#### add osdk-ros 4.0 
Download osdk-ros 4.0 and put it into src.
#### Build the dji_osdk_ros ROS package
>$cd ..
>$catkin_make
#### Configuration
1.Remember to source your setup.bash:
>$source devel/setup.bash  

2.Edit the launch file and enter your App ID, Key, Baudrate and Port name in the designated places:  
(__note:there are two launch file.  
dji_sdk_node.launch is for dji_sdk_node.(3.8.1)  
dji_vehicle_node is for dji_vehicle_node(4.0.0)__)
> $rosed dji_osdk_ros dji_sdk_node.launch  
> $rosed dji_osdk_ros dji_vehicle_node.launch  

3.Remember to add UserConfig.txt to correct path.(in the current work directory))  
>If you want to run dji_sdk_node.launch, you need to put UserConfig.txt into /home/{user}/.ros.
>dji_vehicle_node.launch does not need UserConfig.txt.
#### Running the Samples
1.Start up the dji_osdk_ros ROS node:  
if you want to try 4.0's feature:
>$roslaunch dji_osdk_ros dji_vehicle_node.launch  

if you want to try 3.8.1's feature:
>$roslaunch dji_osdk_ros dji_sdk_node.launch    
>
2.Open up another terminal and cd to your catkin_ws location, and start up a sample (e.g. flight control sample):
>$source devel/setup.bash  
>$rosrun dji_osdk_ros flight_control_node  
>
__note:if you want to rosrun dji_sdk_node,you need to put UserConfig.txt into current work directory.__  
3.Follow the prompt on screen to choose an action for the drone to do.
