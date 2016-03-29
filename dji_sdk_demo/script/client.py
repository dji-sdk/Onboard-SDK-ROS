#!/usr/bin/env python

from dji_sdk.dji_drone import DJIDrone
import dji_sdk.msg 
import time
import sys
import math

def display_main_menu():
    print "----------- < Main menu > ----------"
    print "[a] Request to obtain control"
    print "[b] Release control"
    print "[c] Takeoff"
    print "[d] Landing"
    print "[e] Go home"
    print "[f] Gimbal control sample"
    print "[g] Attitude control sample"
    print "[h] Draw circle sample"
    print "[i] Draw square sample"
    print "[j] Take a picture"
    print "[k] Start video"
    print "[l] Stop video"
    print "[m] Local Navi Test"
    print "[n] GPS Navi Test"
    print "[o] Waypoint List Test"
    print "[p] Arm Test"
    print "[q] Disarm Test"
    print "[r] Vrc Test"
    print "[s] Sync Test"
    print "[t] Exit"
    print "\ninput a/b/c etc..then press enter key"
    print "\nuse `rostopic echo` to query drone status"
    print "----------------------------------------"
    print "input: "

def main():
    drone = DJIDrone()
    display_main_menu()
    while True:
        main_operate_code = sys.stdin.read(1)
        if main_operate_code == 'a':
            drone.request_sdk_permission_control()
        elif main_operate_code == 'b':
            drone.release_sdk_permission_control()
        elif main_operate_code == 'c':
            drone.takeoff()
        elif main_operate_code == 'd':
            drone.landing()
        elif main_operate_code =='e':
            drone.gohome()
        elif main_operate_code == 'f':
            drone.gimbal_angle_control(0, 0, 0, 20)
            time.sleep(2)
            drone.gimbal_angle_control(0, 0, 1800, 20)
            time.sleep(2)
            drone.gimbal_angle_control(0, 0, -1800, 20)
            time.sleep(2)
            drone.gimbal_angle_control(300, 0, 0, 20)
            time.sleep(2)
            drone.gimbal_angle_control(-300, 0, 0, 20)
            time.sleep(2)
            drone.gimbal_angle_control(0, 300, 0, 20)
            time.sleep(2)
            drone.gimbal_angle_control(0, -300, 0, 20)
            time.sleep(2)
            drone.gimbal_speed_control(100, 0, 0)
            time.sleep(2)
            drone.gimbal_speed_control(-100, 0, 0)
            time.sleep(2)
            drone.gimbal_speed_control(0, 0, 200)
            time.sleep(2)
            drone.gimbal_speed_control(0, 0, -200)
            time.sleep(2)
            drone.gimbal_speed_control(0, 200, 0)
            time.sleep(2)
            drone.gimbal_speed_control(0, -200, 0)
            time.sleep(2)
            drone.gimbal_angle_control(0, 0, 0, 20)
        elif main_operate_code == 'g':
            drone.takeoff()
            time.sleep(5)

            for i in range(100):
                if i < 90:
                    drone.attitude_control(0x40, 0, 2, 0, 0)
                else:
                    drone.attitude_control(0x40, 0, 0, 0, 0)
                time.sleep(0.02)
            time.sleep(1)

            for i in range(200):
                if i < 180:
                    drone.attitude_control(0x40, 2, 0, 0, 0)
                else:
                    drone.attitude_control(0x40, 0, 0, 0, 0)
                time.sleep(0.02)
            time.sleep(1)


            for i in range(200):
                if i < 180:
                    drone.attitude_control(0x40, -2, 0, 0, 0)
                else:
                    drone.attitude_control(0x40, 0, 0, 0, 0)
                time.sleep(0.02)
            time.sleep(1)

            for i in range(200):
                if i < 180:
                    drone.attitude_control(0x40, 0, 2, 0, 0)
                else:
                    drone.attitude_control(0x40, 0, 0, 0, 0)
                time.sleep(0.02)
            time.sleep(1)

            for i in range(200):
                if i < 180:
                    drone.attitude_control(0x40, 0, -2, 0, 0)
                else:
                    drone.attitude_control(0x40, 0, 0, 0, 0)
                time.sleep(0.02)
            time.sleep(1)

            for i in range(200):
                if i < 180:
                    drone.attitude_control(0x40, 0, 0, 0.5, 0)
                else:
                    drone.attitude_control(0x40, 0, 0, 0, 0)
                time.sleep(0.02)
            time.sleep(1)

            for i in range(200):
                if i < 180:
                    drone.attitude_control(0x40, 0, 0, -0.5, 0)
                else:
                    drone.attitude_control(0x40, 0, 0, 0, 0)
                time.sleep(0.02)
            time.sleep(1)

            for i in range(200):
                if i < 180:
                    drone.attitude_control(0x40, 0, 0, 0, 90)
                else:
                    drone.attitude_control(0x40, 0, 0, 0, 0)
                time.sleep(0.02)
            time.sleep(1)

            for i in range(200):
                if i < 180:
                    drone.attitude_control(0x40, 0, 0, 0, -90)
                else:
                    drone.attitude_control(0x40, 0, 0, 0, 0)
                time.sleep(0.02)
            time.sleep(1)

            drone.landing()

        elif main_operate_code == 'h':
            R = 2
            V = 2
            # start to draw circle 
            for i in range(300):
                vx = V * math.sin((V/R)*i/50.0)
                vy = V * math.cos((V/R)*i/50.0)
    
                drone.attitude_control(DJIDrone.HORIZ_POS|DJIDrone.VERT_VEL|DJIDrone.YAW_ANG|DJIDrone.HORIZ_BODY|DJIDrone.STABLE_ON, vx, vy, 0, 0)
                time.sleep(0.02)
        elif main_operate_code == 'i':
            # draw square sample
            for i in range(60):
                drone.attitude_control(DJIDrone.HORIZ_POS|DJIDrone.VERT_VEL|DJIDrone.YAW_ANG|DJIDrone.HORIZ_BODY|DJIDrone.STABLE_ON, 3, 3, 0, 0)
                time.sleep(0.02)
            for i in range(60):
                drone.attitude_control(DJIDrone.HORIZ_POS|DJIDrone.VERT_VEL|DJIDrone.YAW_ANG|DJIDrone.HORIZ_BODY|DJIDrone.STABLE_ON, -3, 3, 0, 0)
                time.sleep(0.02)
            for i in range(60):
                drone.attitude_control(DJIDrone.HORIZ_POS|DJIDrone.VERT_VEL|DJIDrone.YAW_ANG|DJIDrone.HORIZ_BODY|DJIDrone.STABLE_ON, -3, -3, 0, 0)
                time.sleep(0.02)
            for i in range(60):
                drone.attitude_control(DJIDrone.HORIZ_POS|DJIDrone.VERT_VEL|DJIDrone.YAW_ANG|DJIDrone.HORIZ_BODY|DJIDrone.STABLE_ON, 3, -3, 0, 0)
                time.sleep(0.02)
        elif main_operate_code == 'j':
            # take a picture
            drone.take_picture()
        elif main_operate_code == 'k':
            # start video
            drone.start_video()
        elif main_operate_code == 'l':
            # stop video
            drone.stop_video()
        elif main_operate_code == 'm':
            # Local Navi Test 
            drone.local_position_navigation_send_request(-100, -100, 100)
        elif main_operate_code == 'n':
            # GPS Navi Test 
            drone.global_position_navigation_send_request(22.535, 113.95, 100)
        elif main_operate_code == 'o':
            # Waypoint List Navi Test 
            newWaypointList = [
                dji_sdk.msg.Waypoint(latitude = 22.535, longitude = 113.95, altitude = 100, staytime = 5, heading = 0),
                dji_sdk.msg.Waypoint(latitude = 22.535, longitude = 113.96, altitude = 100, staytime = 0, heading = 90),
                dji_sdk.msg.Waypoint(latitude = 22.545, longitude = 113.96, altitude = 100, staytime = 4, heading = -90),
                dji_sdk.msg.Waypoint(latitude = 22.545, longitude = 113.96, altitude = 10, staytime = 2, heading = 180),
                dji_sdk.msg.Waypoint(latitude = 22.525, longitude = 113.93, altitude = 50, staytime = 0, heading = -180)]
            drone.waypoint_navigation_send_request(newWaypointList)
        elif main_operate_code == 'p':
            drone.arm_drone()
        elif main_operate_code == 'q':
            drone.disarm_drone()
        elif main_operate_code == 'r':
            drone.vrc_start();
            for i in range(100):
                drone.vrc_control(1024-660, 1024-660, 1024-660, 1024+660)
                time.sleep(0.02)	
            for i in range(1000):
                drone.vrc_control(1024, 1024, 1024+660, 1024+660)
                time.sleep(0.02)	
            for i in range(1000):
                drone.vrc_control(1024-660, 1024-660)
                time.sleep(0.02)	
            drone.vrc_stop()
        elif main_operate_code == 's':
            drone.sync_timestamp(50)
        elif main_operate_code == 't':
            return
        else:
            display_main_menu()

if __name__ == "__main__":
    main()
