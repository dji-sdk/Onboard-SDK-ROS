# This is a fork of DJI's Onboard-SDK-ROS.
# Created by Masahiro Ogawa
#

# DJI Onboard SDK ROS 4.0.1

## Latest Update

OSDK-ROS 4.0.1 was released on 24 August 2020.You need to read newest update below to get update information. Please see the [release notes](https://developer.dji.com/onboard-sdk/downloads/) and [ROS sample setup](https://developer.dji.com/cn/onboard-sdk/documentation/development-workflow/environment-setup.html#linux-with-ros) for more information.And We will update [ROS Wiki](http://wiki.ros.org/dji_sdk/) later.

### 1. feature  
This 4.0.1 version releases a feature package: dji_osdk_ros. The package contains two different framework's interface. OSDK-ROS-obsoleted kept ros3.8.1's interface.  
(__note:We will cancel support for the OSDK-ROS-obsoleted's interface in the next version.__)

| **OSDK-ROS4.0.1 interface**          | **OSDK-ROS-obsoleted interface**            |
|--------------------------------------|---------------------------------------------|
|files below in dji_osdk_ros folder    | files below in dji_osdk_ros_obsoleted folder|

This update mainly includes:  
1. Data subscription interface and sample;  
2. Mobile device interface and sample;  
3. Payload device interface and sample;  
4. Add the top obstacle avoidance enable interface;
5. Waypoint1.0/2.0 interface and sample;  
6. Advanced sensing interface and sample perfection;  
   include:  
   > camera-h264  
   > camera-stream  
   > stereo-vision-depth-perception(rectified stereo images．disparity mapfiltered disparity map (Optional)．point cloud)  
  (__note: If you roslaunch dji_vehicle_node.launch to get m300_stereo_param.yaml, the yaml file will be prouduced in .ros folder(/home/${user}/.ros).And you need to copy it to the directory which you need to rosrun.__)  
7. we also kept all services and topics of osdk-ros 3.8.1. If you want to use these interfaces,you need to run dji_sdk_node and use it's services and topics.   
(__note: These interfaces are not fully compatible with onboard-sdk4.0.1.And they will not be supported in next osdk-ros version.__)

| **nodes**                            |  **services's name**                             | **topics's name**                     |
|--------------------------------------|--------------------------------------------------|---------------------------------------|
|dji_vehicle_node                      |get_drone_type                                    |                                       |
|flight_control_node                   |flight_task_control                               |                                       |
|                                      |set_go_home_altitude                              |                                       |
|                                      |set_current_point_as_home                         |                                       |
|                                      |enable_avoid                                      |                                       |
|                                      |enable_upwards_avoid                              |                                       |
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
|telemetry_node                        |                                                  |dji_osdk_ros/attitude                  |
|                                      |                                                  |dji_osdk_ros/battery_state             |
|                                      |                                                  |dji_osdk_ros/imu                       |
|                                      |                                                  |dji_osdk_ros/flight_status             |
|                                      |                                                  |dji_osdk_ros/gps_health                |
|                                      |                                                  |dji_osdk_ros/gps_position              |
|                                      |                                                  |dji_osdk_ros/vo_position               |
|                                      |                                                  |dji_osdk_ros/height_above_takeoff      |
|                                      |                                                  |dji_osdk_ros/velocity                  |
|                                      |                                                  |dji_osdk_ros/from_mobile_data          |
|                                      |                                                  |dji_osdk_ros/from_payload_data         |
|                                      |                                                  |dji_osdk_ros/gimbal_angle              |
|                                      |                                                  |dji_osdk_ros/rc                        |
|                                      |                                                  |dji_osdk_ros/local_position            |
|                                      |                                                  |dji_osdk_ros/local_frame_ref           |
|                                      |                                                  |dji_osdk_ros/time_sync_nmea_msg        |
|                                      |                                                  |dji_osdk_ros/time_sync_gps_utc         |
|                                      |                                                  |dji_osdk_ros/time_sync_fc_time_utc     |
|                                      |                                                  |dji_osdk_ros/time_sync_pps_source      |
|                                      |                                                  |dji_osdk_ros/main_camera_images        |
|                                      |                                                  |dji_osdk_ros/fpv_camera_images         |
|                                      |                                                  |dji_osdk_ros/camera_h264_stream        |
|                                      |                                                  |dji_osdk_ros/stereo_240p_front_left_images|
|                                      |                                                  |dji_osdk_ros/stereo_240p_front_right_images|
|                                      |                                                  |dji_osdk_ros/stereo_240p_down_front_images |
|                                      |                                                  |dji_osdk_ros/stereo_240p_down_back_images  |
|                                      |                                                  |dji_osdk_ros/stereo_240p_front_depth_images|
|                                      |                                                  |dji_osdk_ros/stereo_vga_front_left_images  |
|                                      |                                                  |dji_osdk_ros/stereo_vga_front_right_images |
|time_sync_node                        |                                                  |                                       |
|mission_node                          |dji_osdk_ros/mission_waypoint_upload              |                                       |
|                                      |dji_osdk_ros/mission_waypoint_action              |                                       |
|                                      |dji_osdk_ros/mission_waypoint_getInfo             |                                       |
|                                      |dji_osdk_ros/mission_waypoint_getSpeed            |                                       |
|                                      |dji_osdk_ros/mission_waypoint_setSpeed            |                                       |
|                                      |dji_osdk_ros/mission_hotpoint_upload              |                                       |
|                                      |dji_osdk_ros/mission_hotpoint_action              |                                       |
|                                      |dji_osdk_ros/mission_hotpoint_getInfo             |                                       |
|                                      |dji_osdk_ros/mission_hotpoint_updateYawRate       |                                       |
|                                      |dji_osdk_ros/mission_hotpoint_resetYaw            |                                       |
|                                      |dji_osdk_ros/mission_hotpoint_updateRadius        |                                       |
|                                      |dji_osdk_ros/mission_status                       |                                       |
|camera_stream_node                    |setup_camera_stream                               |                                       |
|camera_h264_node                      |setup_camera_h264                                 |                                       |
|stereo_vision_depth_perception_node   |get_m300_stereo_params                            |                                       |
|                                      |stereo_240p_subscription                          |                                       |
|                                      |stereo_depth_subscription                         |                                       |
|                                      |stereo_vga_subscription                           |                                       |
|mobile_device_node                    |send_data_to_mobile_device                        |                                       |
|payload_device_node                   |send_data_to_payload_device_server                |                                       |
|waypointV2_node                       |dji_osdk_ros/waypointV2_initSetting               |                                       |
|                                      |dji_osdk_ros/waypointV2_uploadMission             |                                       |
|                                      |dji_osdk_ros/waypointV2_downloadMission           |                                       |
|                                      |dji_osdk_ros/waypointV2_uploadAction              |                                       |
|                                      |dji_osdk_ros/waypointV2_startMission              |                                       |
|                                      |dji_osdk_ros/waypointV2_stopMission               |                                       |
|                                      |dji_osdk_ros/waypointV2_pauseMission              |                                       |
|                                      |dji_osdk_ros/waypointV2_resumeMission             |                                       |
|                                      |dji_osdk_ros/waypointV2_generateActions           |                                       |
|                                      |dji_osdk_ros/waypointV2_setGlobalCruisespeed      |                                       |
|                                      |dji_osdk_ros/waypointV2_getGlobalCruisespeed      |                                       |
|                                      |dji_osdk_ros/waypointV2_subscribeMissionEvent     |dji_osdk_ros/waypointV2_mission_event  |
|                                      |dji_osdk_ros/waypointV2_subscribeMissionState     |dji_osdk_ros/swaypointV2_mission_state |
### 2. Prerequisites
The system environment we have tested is in the table below.

|                            |                                             |
|----------------------------|-------------------------------------------- |  
| **system version**         | ubuntu 16.04                                |
| **processor architecture** | x86(mainfold2-c),armv8(mainfold2-g)         |
#### Firmware Compatibility
OSDK-ROS 4.0.1's firmware compatibility depends on onboard-sdk 4.0.1's. you can get more information [here](https://developer.dji.com/cn/document/0c2b2d75-d019-480c-9241-8c8e7209692d);
#### Ros  
you need to install ros first.Install instruction can be found at: http://wiki.ros.org/ROS/Installation. We just tested ROS kinetic version.  
#### C++11 Compiler
We compile with C + + 11 Standard.
#### onboard-sdk
you need to download onboard-sdk4.0.1,and install it.
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
We use OpenCV to show images from camera stream. Download and install instructions can be found at: http://opencv.org. Tested with OpenCV 3.3.0.Suggest using 3.3.0+.
#### stereo-vision function
Follow the instruction of [here](https://developer.dji.com/onboard-sdk/documentation/sample-doc/advanced-sensing-stereo-depth-perception.html).

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
#### add osdk-ros 4.0.1 
Download osdk-ros 4.0.1 and put it into src.
#### Build the dji_osdk_ros ROS package
>$cd ..  
>$catkin_make
#### Configuration
1.Remember to source your setup.bash.
>$source devel/setup.bash  

2.Edit the launch file and enter your App ID, Key, Baudrate and Port name in the designated places.  
(__note:there are two launch file.  
dji_sdk_node.launch is for dji_sdk_node.(3.8.1's interface)  
dji_vehicle_node is for dji_vehicle_node(4.0.1's interface)__)
> $rosed dji_osdk_ros dji_sdk_node.launch  
> $rosed dji_osdk_ros dji_vehicle_node.launch  

3.Remember to add UserConfig.txt to correct path.(in the current work directory)  
>If you want to run dji_sdk_node.launch, you need to put UserConfig.txt into /home/{user}/.ros.
>dji_vehicle_node.launch does not need UserConfig.txt.
#### Running the Samples
1.Start up the dji_osdk_ros ROS node.  
if you want to use OSDK ROS 4.0.1's services and topics:
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

## Support
You can get support from DJI and the community with the following methods:
* Email to dev@dji.com
* Report issue on github
