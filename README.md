#DJI Onboard SDK ROS Example
##Intro
This example aims to help you understand and play with the basic flight procedures using ROS. The following procedures are included:
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
* Example for local navigation (fly into a certain (X,Y,Z))
* Example for GPS navigation (fly into a certain GPS coordinate)
* Example for waypoint navigation (fly through a series of GPS coordinates)
* Example of using MAVLink protocol and QGroundStation (TODO)
* Example of using Websocket together with Google/Baidu Map for navigation purpose 

Developers can play with this example via the ROS interaction interface.

##Directory Structure
* include: header files
* include/DJI_LIB: DJI Onboard SDK API library
* action: ROS action files
* launch: ROS launch files
* msg: ROS message files
* src: source code
* srv: ROS service files
* README.md: this file

##Compile & Run Environment
The below environment has been tested.
* Operating System: Ubuntu 14.04
* ROS version: ROS Indigo

## Hardware Installation
* In order to communicate with the N1 Autopilot via the DJI OPEN protocal, a physical connection between your Onboard Device and the N1 Autopilot is required with a USB to TTL Serial cable (SOLD Seperately).
* In order to monitor & control the flight, a remote controller connects to the mobile device (with the DJI GO APP running) is needed.

##Configs
Enter the following info into *launch/sdk_demo.launch* when using `roslaunch` OR "src/djiMain.cpp" when using `rosrun` directly.
* APP ID
* APP Level
* Communication Key
* uart device name
* baudrate

>Note: the 'baudrate' needs to be consistent with the setting in the DJI N1 PC assistant.

##Compile
Catkin workspace is required in order to compile and run this ROS example.
Please refer to [ROS catkin tutorial](http://wiki.ros.org/catkin/Tutorials) if you haven't been with it before.

~~~bash
cd `your catkin workspace`
catkin_make
~~~

##Run
We recommend you first run this example in the simulator then move to the real flight test. Also, please be aware that you will need sudo privilege to manipulate the linux serial port. You may need to enter the following command to gain the access privilege.

~~~bash
sudo -s
~~~

Run the ROS server node by entering the following command

~~~bash
roslauch dji_sdk sdk_demo.launch
~~~

OR 

~~~bash
roscore
rosrun dji_sdk dji_sdk
~~~

Note: The above command selection depends on the way you store the activation infomation.

The ROS server node will first check your activation data. If the check of your activation information is successful, it will start running and been able to accept commands.

---
Please make sure you have started the server node and then run the example client node by entering the following command:

~~~bash
rosrun dji_sdk dji_sdk_client
~~~

also we have a navigation example client:

~~~bash
rosrun dji_sdk dji_sdk_wp_client
~~~

The server node handles the communication between onboard device and N1. It processes the data broadcasted from N1, reorganized and published the received stream into different topics. The server node also holds `srv` and `action`, which can be used by other nodes to control the movement and action of M100. Also, the activation procedure is processed in the server node when start running.

The client node is a sample node communicating with server node, which allows user to run several funcions to control the drone.

The wp_client is a special client node for navigation purpose, where user can ask the drone fly into a certain position or fly throught a given waypoint lists.

What's more, we also provides a webpage client node for navigation purpose. Users can draw the [webpage navigation part](webpage_waypoint_client/README.md) for more detail.

![webpageClient](../../Onboard_SDK_Doc/en/Images/ROS.png)


---

The status of the UAV are published following our customized message type together with the [odometry](http://docs.ros.org/api/nav_msgs/html/msg/Odometry.html) message type. 


You can find the topic you want by `rostopic list` and query the data inside using either `rostopic echo [topic name]` OR subscribe to the topic by your own ROS node.


##What You Can Expect
* You can learn how to use ROS to wrap the DJI SDK API Library and use it as a ROS service.
* You can see the flight simulation on screen if you are using the DJI PC simulator. Otherwise, real flight happens.
* You can see the actual 'gimbal and camera' movement.
* You can see the image/video you capture from you mobile device.

ENJOY your flight!

##Appendix

####Topic list:

```xml
/dji_sdk/acceleration
	info: acceleration data
	type: dji_sdk/acc

/dji_sdk/activation_result
	info: acitvation result
	type: std_msgs/UInt8

/dji_sdk/attitude_quad
	info: quaternion data
	type: dji_sdk/attitude_quad

/dji_sdk/battery_status
	info: remaining battery percent(s)
	type: std_msgs/UInt8

/dji_sdk/compass_info
	info: compass data
	type: dji_sdk/compass

/dji_sdk/ctrl_info
	info: current control device of drone
	type: dji_sdk/ctrl_info

/dji_sdk/flight_status
	info: current flight status (please check the document appendix for detailed meaning)
	type: std_msgs/UInt8

/dji_sdk/gimbal_info
	info: gimbal roll/yaw/pitch angle
	type: dji_sdk/gimbal

/dji_sdk/global_position
	info: current GPS coordinate 
	type: dji_sdk/global_position

/dji_sdk/gps_navigation_action/cancel
	info: gps_navigation_action action cancel 
	type: actionlib_msgs/GoalID

/dji_sdk/gps_navigation_action/feedback
	info: gps_navigation_action action progress
	type: dji_sdk/gps_navigationActionFeedback

/dji_sdk/gps_navigation_action/goal
	info: gps_navigation_action action goal
	type: dji_sdk/gps_navigationActionGoal

/dji_sdk/gps_navigation_action/result
	info: gps_navigation_action action result
	type: dji_sdk/gps_navigationActionResult

/dji_sdk/gps_navigation_action/status
	info: gps_navigation_action action status
	type: actionlib_msgs/GoalStatusArray

/dji_sdk/local_navigation_action/cancel
	info: local_navigation_action action cancel
	type: actionlib_msgs/GoalID

/dji_sdk/local_navigation_action/feedback
	info: local_navigation_action action progress
	type: dji_sdk/local_navigationActionFeedback

/dji_sdk/local_navigation_action/goal
	info: local_navigation_action action goal
	type: dji_sdk/local_navigationActionGoal

/dji_sdk/local_navigation_action/result
	info: local_navigation_action action result
	type: dji_sdk/local_navigationActionResult

/dji_sdk/local_navigation_action/status
	info: local_navigation_action action status
	type: actionlib_msgs/GoalStatusArray

/dji_sdk/local_position
	info: current local position((X,Y,Z) coordinate, origin is the first recorded GPS point when server node start)
	type: dji_sdk/local_position

/dji_sdk/obtained_control
	info: current onboard device control ability
	type: std_msgs/UInt8

/dji_sdk/odom
	info: current odometry data
	type: nav_msgs/Odometry

/dji_sdk/rc_channels
	info: current channel values from RC controller
	type: dji_sdk/rc_channels
	
/dji_sdk/velocity
	info: current velocity
	type: dji_sdk/velocity

/dji_sdk/waypoint_navigation_action/cancel
	info: waypoint_navigation_action action cancel
	type: actionlib_msgs/GoalID

/dji_sdk/waypoint_navigation_action/feedback
	info: waypoint_navigation_action action progress
	type: dji_sdk/waypoint_navigationActionFeedback

/dji_sdk/waypoint_navigation_action/goal
	info: waypoint_navigation_action action goal
	type: dji_sdk/waypoint_navigationActionGoal

/dji_sdk/waypoint_navigation_action/result
	info: waypoint_navigation_action action result
	type: dji_sdk/waypoint_navigationActionResult

/dji_sdk/waypoint_navigation_action/status
	info: waypoint_navigation_action action statsu
	type: actionlib_msgs/GoalStatusArray
```

####Msg List
```xml
dji_sdk/gps_navigationAction
dji_sdk/gps_navigationActionFeedback
dji_sdk/gps_navigationActionGoal
dji_sdk/gps_navigationActionResult
dji_sdk/gps_navigationFeedback
dji_sdk/gps_navigationGoal
dji_sdk/gps_navigationResult
	auto generated by GPS navigation action

dji_sdk/local_navigationAction
dji_sdk/local_navigationActionFeedback
dji_sdk/local_navigationActionGoal
dji_sdk/local_navigationActionResult
dji_sdk/local_navigationFeedback
dji_sdk/local_navigationGoal
dji_sdk/local_navigationResult
	auto generated by local navigation action

dji_sdk/taskAction
dji_sdk/taskActionFeedback
dji_sdk/taskActionGoal
dji_sdk/taskActionResult
dji_sdk/taskFeedback
dji_sdk/taskGoal
dji_sdk/taskResult
	auto generated by task action

dji_sdk/waypoint_navigationAction
dji_sdk/waypoint_navigationActionFeedback
dji_sdk/waypoint_navigationActionGoal
dji_sdk/waypoint_navigationActionResult
dji_sdk/waypoint_navigationFeedback
dji_sdk/waypoint_navigationGoal
dji_sdk/waypoint_navigationResult
	auto generate by waypoint navigation action

dji_sdk/acc
	std_msgs/Header header
	  uint32 seq
	  time stamp
	  string frame_id
	int32 ts
	float32 ax
	float32 ay
	float32 az

dji_sdk/attitude_quad
	std_msgs/Header header
	  uint32 seq
	  time stamp
	  string frame_id
	int32 ts
	float32 q0
	float32 q1
	float32 q2
	float32 q3
	float32 wx
	float32 wy
	float32 wz

dji_sdk/compass
	std_msgs/Header header
	  uint32 seq
	  time stamp
	  string frame_id
	int32 ts
	int8 x
	int8 y
	int8 z

dji_sdk/ctrl_info
	int8 cur_ctrl_dev_in_navi_mode
	int8 serial_req_status

dji_sdk/gimbal
	std_msgs/Header header
	  uint32 seq
	  time stamp
	  string frame_id
	int32 ts
	float32 pitch
	float32 yaw
	float32 roll

dji_sdk/global_position
	std_msgs/Header header
	  uint32 seq
	  time stamp
	  string frame_id
	int32 ts
	#latitude is in radian
	float64 latitude
	#longitude is in radian
	float64 longitude
	float32 altitude
	float32 height
	#reliablity [0,5]
	int8 health

dji_sdk/local_position
	#    North(x)
	#   /
	#  /
	# /______East(y)
	# |
	# |
	# Donw (-z)
	std_msgs/Header header
	  uint32 seq
	  time stamp
	  string frame_id
	int32 ts
	float32 x
	float32 y
	float32 z

dji_sdk/rc_channels
	# RC Map
	#
	#  mode:
	#
	# +8000 <--->  0  <---> -8000
	#  API  <---> ATT <--->  POS
	#
	# CH3(throttle) +10000            CH1(pitch) +10000
	#             ^                            ^
	#             |                            |                  / -10000
	#  CH2(yaw)   |                CH0(roll)   |                 /
	# -10000 <-----------> +10000 -10000 <-----------> +10000   H(gear_up)
	#             |                            |                 \
	#             |                            |                  \ -4545
	#             V                            V
	#          -10000                       -10000

	std_msgs/Header header
	  uint32 seq
	  time stamp
	  string frame_id
	int32 ts
	float32 roll
	float32 pitch
	float32 yaw
	float32 throttle
	float32 mode
	float32 gear_up

dji_sdk/velocity
	std_msgs/Header header
	  uint32 seq
	  time stamp
	  string frame_id
	int32 ts
	float32 velx
	float32 vely
	float32 velz
	uint8 health_flag
	uint8 feedback_sensor_id


dji_sdk/waypoint
	#latitude is in degree
	float64 latitude
	#longitude is in degree
	float64 longitude
	float32 altitude
	#heading is in [-180,180]
	int16 heading
	#stay time is in second
	uint16 staytime


dji_sdk/waypointList
	dji_sdk/waypoint[] waypointList


```

####Service List
```xml
/dji_sdk/camera_action_service
	info: camera action control service
	type: dji_sdk/camera_action

/dji_sdk/drone_action_control
	info: drone action control service
	type: dji_sdk/action

/dji_sdk/drone_attitude_control
	info: drone attitude control service
	type: dji_sdk/attitude

/dji_sdk/gimbal_angle_control
	info: gimbal angle control service
	type: dji_sdk/gimbal_angle

/dji_sdk/gimbal_speed_control
	info: gimbal angular speed control service
	type: dji_sdk/gimbal_speed

/dji_sdk/obtain_release_control
	info: obtain / release control abilitiy service
	type: dji_sdk/control_manager

```

####Srv List
```xml
dji_sdk/action
	#takeoff: 4
	#landing: 6
	#gohome:  1
	uint8 action
	---
	bool result

dji_sdk/attitude
	uint8 flag
	float32 x
	float32 y
	float32 z
	float32 yaw
	---
	bool result 

dji_sdk/camera_action
	#takePicture: 0
	#startRecord: 1
	#stopRecord:  2
	uint8 camera_action
	---
	bool result

dji_sdk/control_manager
	#requestControl: 1
	#releaseControl: 0
	uint8 control_ability 
	---
	bool result

dji_sdk/gimbal_angle
	#moveByUnits: 0
	#moveToUnits: 1
	uint8 flag
	int16 yaw
	int16 x
	int16 y
	int16 duration
	---
	bool result 

dji_sdk/gimbal_speed
	int16 yaw_rate
	int16 x_rate
	int16 y_rate
	---
	bool result 
```

####Action Server List
```xml

dji_sdk/gps_navigation_action
	info: gps navigation action
	type: dji_sdk/gps_navigation.action

dji_sdk/local_navigation_action
	info: local navigation action
	type: dji_sdk/local_navigation.action

dji_sdk/task_action [DEPRECATED]
	info: same to drone action control service
	type: dji_sdk/task.action

dji_sdk/waypoint_navigation_action
	info: waypoint navigation action
	type: dji_sdk/waypoint_navigation.action
```

####Action File List
```xml
gps_navigation.action
	#latitude is in degree
	float64 latitude
	#longitude is in degree
	float64 longitude
	float32 altitude
	---
	bool result
	---
	#progress is in percent
	uint8 latitude_progress
	uint8 longitude_progress
	uint8 altitude_progress


local_navigation.action
	#x,y,z are in meters
	float32 x
	float32 y
	float32 z
	---
	bool result
	---
	#progress is in percent
	uint8 x_progress 
	uint8 y_progress 
	uint8 z_progress 

task.action
	uint8 task 
	---
	bool result
	---
	uint8 progress 

waypoint_navigation.action
	dji_sdk/waypointList waypointList
	---
	bool result
	---
	#progress is in percent
	uint8 latitude_progress
	uint8 longitude_progress
	uint8 altitude_progress
	uint8 index_progress

