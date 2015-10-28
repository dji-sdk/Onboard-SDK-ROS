#!/usr/bin/env python
#encoding = utf-8
import rospy
import std_msgs.msg 
import dji_sdk.msg 
import nav_msgs.msg 
import dji_sdk.srv 
import math
import time
import actionlib
import roslib

class DJISDKConnection:
    
    def init_subscribers(self):
        self.acceleration_subscriber = rospy.Subscriber("dji_sdk/acceleration", dji_sdk.msg.Acceleration)
        self.attitude_quaternion_subscriber = rospy.Subscriber("dji_sdk/attitude_quaternion",dji_sdk.msg.AttitudeQuaternion)
        self.compass_subscriber = rospy.Subscriber("dji_sdk/compass", dji_sdk.msg.Compass)
        self.flight_control_info_subscriber = rospy.Subscriber("dji_sdk/flight_control_info", dji_sdk.msg.FlightControlInfo)
        self.flight_status_subscriber = rospy.Subscriber("dji_sdk/flight_status", std_msgs.msg.UInt8)
        self.gimbal_subscriber = rospy.Subscriber("dji_sdk/gimbal", dji_sdk.msg.Gimbal)
        self.global_position_subscriber = rospy.Subscriber("dji_sdk/global_position", dji_sdk.msg.GlobalPosition)
        self.local_position_subscriber = rospy.Subscriber("dji_sdk/local_position", dji_sdk.msg.LocalPosition)
        self.power_status_subscriber = rospy.Subscriber("dji_sdk/power_status", dji_sdk.msg.PowerStatus)
        self.rc_channels_subscriber = rospy.Subscriber("dji_sdk/rc_channels", dji_sdk.msg.RCChannels)
        self.velocity_subscriber = rospy.Subscriber("dji_sdk/velocity", dji_sdk.msg.Velocity)
        self.activation_subscriber = rospy.Subscriber("dji_sdk/activation", std_msgs.msg.UInt8)
        self.odometry_subscriber = rospy.Subscriber("dji_sdk/odometry", nav_msgs.msg.Odometry)
        self.sdk_permission_subscriber = rospy.Subscriber("dji_sdk/sdk_permission", std_msgs.msg.UInt8)

    def init_services(self):
        rospy.wait_for_service("dji_sdk/attitude_control")
        rospy.wait_for_service("dji_sdk/camera_action_control")
        rospy.wait_for_service("dji_sdk/drone_task_control")
        rospy.wait_for_service("dji_sdk/gimbal_angle_control")
        rospy.wait_for_service("dji_sdk/gimbal_speed_control")
        rospy.wait_for_service("dji_sdk/global_position_control")
        rospy.wait_for_service("dji_sdk/local_position_control")
        rospy.wait_for_service("dji_sdk/sdk_permission_control")
        rospy.wait_for_service("dji_sdk/velocity_control")

        self.attitude_control_service = rospy.ServiceProxy("dji_sdk/attitude_control", dji_sdk.srv.AttitudeControl)
        self.camera_action_control_service = rospy.ServiceProxy("dji_sdk/camera_action_control", dji_sdk.srv.CameraActionControl)
        self.drone_task_control_service = rospy.ServiceProxy("dji_sdk/drone_task_control", dji_sdk.srv.DroneTaskControl)
        self.gimbal_angle_control_service = rospy.ServiceProxy("dji_sdk/gimbal_angle_control", dji_sdk.srv.GimbalAngleControl)
        self.gimbal_speed_control_service = rospy.ServiceProxy("dji_sdk/gimbal_speed_control", dji_sdk.srv.GimbalSpeedControl)
        self.global_position_control_service = rospy.ServiceProxy("dji_sdk/global_position_control", dji_sdk.srv.GlobalPositionControl)
        self.local_position_control_service = rospy.ServiceProxy("dji_sdk/local_position_control", dji_sdk.srv.LocalPositionControl)
        self.sdk_permission_control_service = rospy.ServiceProxy("dji_sdk/sdk_permission_control", dji_sdk.srv.SDKPermissionControl)
        self.velocity_control_service = rospy.ServiceProxy("dji_sdk/velocity_control", dji_sdk.srv.VelocityControl)

    def init_actions(self):
        self.local_position_navigation_action_client = actionlib.SimpleActionClient("dji_sdk/local_position_navigation_action", dji_sdk.msg.LocalPositionNavigationAction)
        self.local_position_navigation_action_client.wait_for_server()
        self.global_position_navigation_action_client = actionlib.SimpleActionClient("dji_sdk/global_position_navigation_action", dji_sdk.msg.GlobalPositionNavigationAction)
        self.global_position_navigation_action_client.wait_for_server()
        self.waypoint_navigation_action_client = actionlib.SimpleActionClient("dji_sdk/waypoint_navigation_action", dji_sdk.msg.WaypointNavigationAction)
        self.waypoint_navigation_action_client.wait_for_server()

    def navigate_local_action(self, x, y, z):
        self.local_navigation_action_client.wait_for_server()
        goal = dji_sdk.msg.LocalPositionControlGoal(x = x, y = y, z = z)
        local_navigation_action_client.send_goal(goal)

    def navigate_global_action(self, latitude, longitude, altitude):
        self.gps_navigation_action_client.wait_for_server()
        goal = dji_sdk.msg.gps_navigationGoal(latitude = latitude, longitude = longitude, altitude = altitude)
        gps_navigation_action_client.send_goal(goal)

    def navigate_waypoint_action(self, waypoint_list):
        self.waypoint_navigation_action_client.wait_for_server()
        goal = dji_sdk.msg.waypointList(waypoint_list=waypoint_list)
        waypoint_navigation_action_client.send_goal(goal)

    def takeoff(self):
        self.drone_task_control_service(task=4)

    def landing(self):
        self.drone_task_control_service(task=6)

    def gohome(self):
        self.drone_task_control_service(task=1)

    def take_picture(self):
        self.camera_action_control_service(camera_action=0)

    def start_video(self):
        self.camera_action_control_service(camera_action=1)

    def stop_video(self):
        self.camera_action_control_service(camera_action=2)

    def set_gimbal_angle(self, flag, yaw, roll, pitch, duration):
        self.gimbal_angle_control_service(flag = flag, yaw = yaw, roll = roll, pitch = pitch, duration = duration)

    def gimbal_speed_control(self, yaw_rate, roll_rate, pitch_rate):
        self.gimbal_speed_control_service(yaw_rate = yaw_rate, roll_rate = roll_rate, pitch_rate = pitch_rate)

    def request_sdk_permission_control(self):
        sdk_permission_control_service.call(control_enable = 1)

    def release_sdk_permission_control(self):
        sdk_permission_control_service.call(control_enable = 0)

    def attitude_control(flag, x, y, z, yaw):
        attitude_control_service.call(flag = flag, x = x, y = y, z = z, yaw = yaw)

    def velocity_control(frame, vx, vy, vz, yawAngle):
        velocity_control_service.call(frame = frame, vx = vx, vy = vy, vz = vz, yawAngle = yawAngle)

    def local_position_control(x, y, z, yaw):
        local_position_control_service.call(x = x, y = y, z = z, yaw = yaw)

    def global_position_control(latitude, longitude, altitude, yaw):
        global_position_control_service.call(latitude = latitude, longitude = longitude, altitude = altitude, yaw = yaw)

    def __init__(self, namespace=''):
        rospy.init_node('dji_sdk_connector')
        self.namespace = namespace

        self.need_open = True

        self.acceleration = dji_sdk.msg.Acceleration()
        self.attitude_quaternion = dji_sdk.msg.AttitudeQuaternion()
        self.compass = dji_sdk.msg.Compass()
        self.flight_control_info = dji_sdk.msg.FlightControlInfo()
        self.flight_status = std_msgs.msg.UInt8()
        self.gimbal = dji_sdk.msg.Gimbal()
        self.global_position = dji_sdk.msg.GlobalPosition()
        self.global_position_ref = dji_sdk.msg.GlobalPosition()
        self.local_position = dji_sdk.msg.LocalPosition()
        self.local_position_ref = dji_sdk.msg.LocalPosition()
        self.power_status = dji_sdk.msg.PowerStatus()
        self.rc_channels = dji_sdk.msg.rc_channels() 
        self.velocity = dji_sdk.msg.velocity()
        self.odometry = nav_msgs.msg.Odometry()
        self.sdk_permission_opened = False
        self.activation = False
        self.localposbase_use_height = True

        self.init_actions()
        self.init_subscribers()
        self.init_services()

        self.update_time = rospy.Timer(rospy.Duration(0.02), self.update)

    def update_modes(self):
        if self.acted == 0:
            # print "try act"
            self.send_activation()

        if self.need_open:
            self.send_open_close_control()
            pass

if __name__ == "__main__":
    drone = DJIDrone('drone1')
    drone.takeoff()
    for i in range(1000):
        drone.
        time.sleep(0.02)
    drone.land()

    rospy.spin()
