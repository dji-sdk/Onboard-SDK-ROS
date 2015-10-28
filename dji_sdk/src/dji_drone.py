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
        self.acceleration_publisher = nh.advertise<dji_sdk::Acceleration>("dji_sdk/acceleration", 10);
        self.attitude_quaternion_publisher = nh.advertise<dji_sdk::AttitudeQuaternion>("dji_sdk/attitude_quaternion", 10);
        self.compass_publisher = nh.advertise<dji_sdk::Compass>("dji_sdk/compass", 10);
        self.flight_control_info_publisher = nh.advertise<dji_sdk::FlightControlInfo>("dji_sdk/flight_control_info", 10);
        self.flight_status_publisher = nh.advertise<std_msgs::UInt8>("dji_sdk/flight_status", 10);
        self.gimbal_publisher = nh.advertise<dji_sdk::Gimbal>("dji_sdk/gimbal", 10);
        self.global_position_publisher = nh.advertise<dji_sdk::GlobalPosition>("dji_sdk/global_position", 10);
        self.local_position_publisher = nh.advertise<dji_sdk::LocalPosition>("dji_sdk/local_position", 10);
        self.power_status_publisher = nh.advertise<dji_sdk::PowerStatus>("dji_sdk/power_status", 10);
        self.rc_channels_publisher = nh.advertise<dji_sdk::RCChannels>("dji_sdk/rc_channels", 10);
        self.velocity_publisher = nh.advertise<dji_sdk::Velocity>("dji_sdk/velocity", 10);
        self.activation_publisher = nh.advertise<std_msgs::UInt8>("dji_sdk/activation", 10);
        self.odometry_publisher = nh.advertise<nav_msgs::Odometry>("dji_sdk/odometry",10);
        self.sdk_permission_publisher = nh.advertise<std_msgs::UInt8>("dji_sdk/sdk_permission", 10);

        self.acceleration_sub = rospy.Subscriber(self.ns + "/dji_sdk/acceleration", dji_sdk.msg.acc, self.handle_acceleration)
        self.attitude_quaternion_sub = rospy.Subscriber(self.ns + "/dji_sdk/attitude_quad", dji_sdk.msg.attitude_quad, self.handle_attitude_quad)
        self.power_status_sub = rospy.Subscriber(self.ns + "/dji_sdk/battery_status", std_msgs.msg.UInt8, self.handle_battery_status)
        self.gimbal_sub = rospy.Subscriber(self.ns + "/dji_sdk/gimbal_info", dji_sdk.msg.gimbal, self.handle_gimbal_info)
        self.flight_status_sub = rospy.Subscriber(self.ns + "/dji_sdk/flight_status", std_msgs.msg.UInt8, self.handle_flight_status)
        self.global_position_sub = rospy.Subscriber(self.ns + "/dji_sdk/global_position", dji_sdk.msg.global_position, self.handle_global_position)
        self.local_position_sub = rospy.Subscriber(self.ns + "/dji_sdk/local_position", dji_sdk.msg.local_position, self.handle_local_position)
        self.odom_sub = rospy.Subscriber(self.ns + "/dji_sdk/odom", nav_msgs.msg.Odometry, self.handle_odom)
        self.vel_sub = rospy.Subscriber(self.ns + "/dji_sdk/velocity", dji_sdk.msg.velocity, self.handle_velocity)
        self.rc_channels_sub = rospy.Subscriber(self.ns + "/dji_sdk/rc_channels", dji_sdk.msg.rc_channels, self.handle_rc_channels)
        self.control_sub = rospy.Subscriber(self.ns + "/dji_sdk/obtained_control", std_msgs.msg.UInt8, self.handle_obtained_control)
        self.activation_sub = rospy.Subscriber(self.ns + "/dji_sdk/activation_result", std_msgs.msg.UInt8, self.handle_activation_result)
        self.ctrl_info_sub = rospy.Subscriber(self.ns + "/dji_sdk/ctrl_info", dji_sdk.msg.ctrl_info, self.handle_ctrl_info)
        self.compass_sub = rospy.Subscriber(self.ns + "/dji_sdk/compass_info", dji_sdk.msg.compass, self.handle_compass_info)

    def init_actions(self):
        self.local_navigation_action_client = actionlib.SimpleActionClient(self.ns + "/dji_sdk/local_navigation_action", dji_sdk.msg.local_navigationAction)
        self.local_navigation_action_client.wait_for_server()
        self.gps_navigation_action_client = actionlib.SimpleActionClient(self.ns + "/dji_sdk/gps_navigation_action", dji_sdk.msg.gps_navigationAction)
        self.gps_navigation_action_client.wait_for_server()
        self.waypoint_navigation_action_client = actionlib.SimpleActionClient(self.ns + "/dji_sdk/waypoint_navigation_action", dji_sdk.msg.waypoint_navigationAction)
        self.waypoint_navigation_action_client.wait_for_server()

    def navigate_local_action(self, x, y, z):
        self.local_navigation_action_client.wait_for_server()
        goal = dji_sdk.msg.local_navigationGoal(x = x, y = y, z = z)
        local_navigation_action_client.send_goal(goal)
        local_navigation_action_client.wait_for_result()
        return local_navigation_action_client.get_result()

    def navigate_global_action(self, latitude, longitude, altitude):
        self.gps_navigation_action_client.wait_for_server()
        goal = dji_sdk.msg.gps_navigationGoal(latitude = latitude, longitude = longitude, altitude = altitude)
        gps_navigation_action_client.send_goal(goal)
        gps_navigation_action_client.wait_for_result()
        return gps_navigation_action_client.get_result()

    def navigate_waypoint_action(self, waypointList):
        self.waypoint_navigation_action_client.wait_for_server()
        goal = dji_sdk.msg.waypointList(waypointList=waypointList)
        waypoint_navigation_action_client.send_goal(goal)
        waypoint_navigation_action_client.wait_for_result()
        return waypoint_navigation_action_client.get_result()

    def request_control(self):
        self.control_service(control_ability=1)

    def release_control(self):
        self.control_service(control_ability=0)

    def takeoff(self):
        self.action_service(action=4)

    def land(self):
        self.action_service(action=6)

    def go_home(self):
        self.action_service(action=1)

    def take_picture(self):
        self.camera_service(camera_action=0)

    def start_video(self):
        self.camera_service(camera_action=1)

    def stop_video(self):
        self.camera_service(camera_action=2)

    def set_gimbal_angle(self, flag, yaw, x, y, duration):
        self.gimbal_angle_service(flag, yaw, x, y, duration)

    def control_gimbal_speed(self, yaw_rate, x_rate, y_rate):
        self.gimbal_speed_service(yaw_rate, x_rate, y_rate)

    def control_attitude(self, flag, x, y, yaw):
        self.attitude_service(flag, x, y, yaw)

    def init_services(self):
        print "initing service..."

        rospy.wait_for_service(self.ns + "/dji_sdk/obtain_release_control")
        self.control_service = rospy.ServiceProxy(self.ns + "/dji_sdk/obtain_release_control",  dji_sdk.srv.control_manager)

        rospy.wait_for_service(self.ns + "/dji_sdk/drone_action_control")
        self.action_service = rospy.ServiceProxy(self.ns + "/dji_sdk/drone_action_control",  dji_sdk.srv.action)

        rospy.wait_for_service(self.ns + "/dji_sdk/camera_action_control")
        self.camera_service = rospy.ServiceProxy(self.ns + "/dji_sdk/camera_action_control",  dji_sdk.srv.camera_action)

        rospy.wait_for_service(self.ns + "/dji_sdk/gimbal_angle_control")
        self.gimbal_angle_service = rospy.ServiceProxy(self.ns + "/dji_sdk/gimbal_angle_control",  dji_sdk.srv.gimbal_angle)

        rospy.wait_for_service(self.ns + "/dji_sdk/gimbal_speed_control")
        self.gimbal_speed_service = rospy.ServiceProxy(self.ns + "/dji_sdk/gimbal_speed_control",  dji_sdk.srv.gimbal_speed)

        rospy.wait_for_service(self.ns + "/dji_sdk/drone_attitude_control")
        self.attitude_service = rospy.ServiceProxy(self.ns + "/dji_sdk/drone_attitude_control",  dji_sdk.srv.attitude)

        rospy.wait_for_service(self.ns + "/dji_sdk/local_navigation_control")
        self.local_navigation_service = rospy.ServiceProxy(self.ns + "/dji_sdk/local_navigation_control",  dji_sdk.srv.local_navigation)

        rospy.wait_for_service(self.ns + "/dji_sdk/gps_navigation_control")
        self.gps_navigation_service = rospy.ServiceProxy(self.ns + "/dji_sdk/gps_navigation_control",  dji_sdk.srv.gps_navigation)

    def __init__(self, namespace=''):
        rospy.init_node('dji_sdk_connector')
        self.ns = namespace
        self.count = 0

        self.elapsed_time = 0

        self.cmd_sp = 0
        self.status = 0

        self.open = 0
        self.need_open = True

        self.att_quad = attitude_quad()
        self.global_position = dji_sdk.msg.global_position()
        self.local_position = dji_sdk.msg.local_position()
        self.vel = velocity()
        self.acc = acc()
        self.gimbal_sp = gimbal()
        self.rc_channels = rc_channels()

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
