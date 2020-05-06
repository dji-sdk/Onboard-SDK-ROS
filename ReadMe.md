# DJI Onboard SDK ROS 4.0.0

## Latest Update

OSDK-ROS 4.0.0 was released on 8 May 2020.You need to read newest update below the readme to get update information. 

## newest update  
### 1. feature  
This 4.0 version releases a feature package: dji_osdk_ros. The package contains two different framework's interface. OSDK-ROS-obsoleted kept ros3.8.1's interface.
(__note:We will cancel support for the OSDK-ROS-obsoleted interface in the next version.__)

| **OSDK-ROS4.0 interface**            | **OSDK-ROS-obsoleted interface**            |
|--------------------------------------|---------------------------------------------|
|files below in dji_osdk_ros folder    | files below in dji_osdk_ros_obsoleted folder|

This update mainly includes:  
1. Redesigned the 4.0 version of the framework and interface (ROS side interacts with the OSDK side through the wrapper layer, business-related interfaces are fully encapsulated into the wrapper layer, and the ROS side provides all services and topics);  
2. Version 4.0 provides a main node (dji_vehicle_node), which completes the activation of the drone, the acquisition of control rights, and the initialization of all services and topics;  

| **nodes**                            |  **services's name**                             | **topics's name**                     |
|--------------------------------------|--------------------------------------------------|---------------------------------------|
|flight_control_node                   |flight_task_control                               |                                       |
|                                      |set_go_home_altitude                              |                                       |
|                                      |set_current_point_as_home                         |                                       |
|                                      |enable_avoid                                      |                                       |
|advanced_sensing_node                 |advanced_sensing                                  |cameradata                             | 
|gimbal_camera_control_node            |gimbal_task_control                               |                                       |
|                                      |camera_task_set_EV                                |                                       |
|                                      |camera_task_set_shutter_speed                     |                                       |
|                                      |camera_task_set_aperture                          |                                       |
|                                      |camera_task_set_ISO                               |                                       |
|                                      |camera_task_set_focus_point                       |                                       |
|                                      |camera_task_tap_zoom_point                        |                                       |
|                                      |camera_task_zoom_ctrl                             |                                       |
|                                      |camera_start_shoot_single_photo                   |                                       |
|                                      |camera_start_shoot_aeb_photo                      |                                       |
|                                      |camera_start_shoot_burst_photo                    |                                       |
|                                      |camera_start_shoot_interval_photo                 |                                       |
|                                      |camera_stop_shoot_photo                           |                                       |
|                                      |camera_record_video_action                        |                                       |
|mfio_conrol_node(not test)            |mfio_control                                      |                                       |
|wapoint(will be supported in next version) |waypoint(will be supported in next version)       |waypoint(will be supported in next version) | 
 
3. At the same time, we kept all services and topics of osdk-ros 3.8.1. If you want to use these interfaces,you need to run dji_sdk_node and use it's services and topics. 
(__note: they will not be supported in next osdk-ros version.__)
### 2. Prerequisites
The system environment we have tested is in the table below.

|                            |                                             |
|----------------------------|-------------------------------------------- |  
| **system version**         | ubuntu 16.04                                |
| **processor architecture** | x86(mainfold2-c),armv8(mainfold2-c)           |

#### Ros  
you need to install ros first.Install instruction can be found at: http://wiki.ros.org/ROS/Installation. We just tested ROS kinetic version.  
#### C++11 Compiler
We compile with C + + 11 Standard.
#### onboard-sdk
you need to download newest onboard-sdk,and install it.
>$mkdir build  
>$cd build  
>$cmake..  
>$sudo make -j7 install
#### nema-comms
> $sudo apt install ros-{release}-nmea-comms  

__note:we only test on kinetic,but it should be support on other version.__
#### ffmpeg
> $sudo apt install ffmpeg  
#### libusb-1.0-0-dev
> $sudo apt install libusb-1.0-0-dev
#### opencv3.x
We use OpenCV to show images from camera stream. Download and install instructions can be found at: http://opencv.org. Tested with OpenCV 3.2.0.

### 3.Permission
#### uart permission
You need to add your user to the dialout group to obtain read/write permissions for the uart communication.
>$sudo usermod -a -G dialout ${USER}  
>
Then log out of your user account and log in again for the permissions to take effect.

#### usb permission
You will need to add an udev file to allow your system to obtain permission and to identify DJI USB port.
>$cd /etc/udev/rules.d/  
>$sudo vi DJIDevice.rules

Then add these content into DJIDevice.rules.
>$SUBSYSTEM=="usb", ATTRS{idVendor}=="2ca3", MODE="0666"

At last,you need to reboot your computer to make sure it works.

### 4. Building dji_osdk_ros pkg
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
1.Remember to source your setup.bash.
>$source devel/setup.bash  

2.Edit the launch file and enter your App ID, Key, Baudrate and Port name in the designated places.  
(__note:there are two launch file.  
dji_sdk_node.launch is for dji_sdk_node.(3.8.1's interface)  
dji_vehicle_node is for dji_vehicle_node(4.0.0's interface)__)
> $rosed dji_osdk_ros dji_sdk_node.launch  
> $rosed dji_osdk_ros dji_vehicle_node.launch  

3.Remember to add UserConfig.txt to correct path.(in the current work directory)  
>If you want to run dji_sdk_node.launch, you need to put UserConfig.txt into /home/{user}/.ros.
>dji_vehicle_node.launch does not need UserConfig.txt.
#### Running the Samples
1.Start up the dji_osdk_ros ROS node.  
if you want to use OSDK ROS 4.0.0's services and topics:
>$roslaunch dji_osdk_ros dji_vehicle_node.launch  

if you want to adapt to OSDK ROS 3.8.1's services and topics:
>$roslaunch dji_osdk_ros dji_sdk_node.launch    
>
2.Open up another terminal and cd to your catkin_ws location, and start up a sample (e.g. flight control sample).
>$source devel/setup.bash  
>$rosrun dji_osdk_ros flight_control_node  
>
__note:if you want to rosrun dji_sdk_node,you need to put UserConfig.txt into current work directory.__  
3.Follow the prompt on screen to choose an action for the drone to do.
