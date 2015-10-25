#!/usr/bin/env python
#encoding = utf-8
import rospy
from std_msgs.msg import String,Float32
from dji_sdk.msg import *
from dji_sdk.srv import *
import math
import time

status = {
    'UNKNOWN':0,
    'STANDBY':1,
    'TAKINGOFF':2,
    'INAIR':3,
    'LANDING':4,
    'POSTLANGING':5
}

# fx = open("/Users/xuhao/aircraft/DJI/data/log_x_y_circle.txt","w")
# fy = open("/Users/xuhao/aircraft/DJI/log_x_y_circle_sp.txt","w")

fx = open("/data/log_x_y_circle.txt","w")
fgps = open('/data/gps.txt',"w")
global_log = open('/data/global_pos.txt', "w")

class dji_conn:
    def send_activation(self):
        # print "try to activation"
        req = Float32()
        req.data = 1
        self.act_pub.publish(req)

    def handle_activation(self,e):
        data = e.data
        if data < 1:
            print "activate successful"
            self.acted = 1
        else:
            print "activate unsuccessful {0}".format(data)
            pass

    def send_open_close_control(self):
        print "send_open_close_control"
        req = Float32()
        req.data = 1
        self.open_close_pub.publish(req)

    def handle_open_close_control(self,data):
	print "handle_open_close_control", data.data
        if data.data == 2:
            self.need_open = False
            if self.seted_ref == 0:
                print "Get control"
                self.services["set_local_position_ref"](self.global_pos,True)
                print "set base point to {0} {1}".format(
                    self.global_pos.lat,self.global_pos.lon
                )
                print >>fgps,"zero point lat {0},lon {1}".format(
                    self.global_pos.lat,
                    self.global_pos.lon
                )

                self.seted_ref = 1
	    print "open = 1"
            self.open = 1
        else:
            # print "lose control"
            self.open = 0
            pass

    def handle_attitude_quat(self,data):
        self.att_quad = data

    def handle_global_pos(self,data):
        self.global_pos = data

        print >>global_log,"{0},{1},{2},{3}".format(
            self.elapsed_time,
            data.lat,data.lon,data.height
        )


    def handle_local_position(self,data):
        self.local_pos = data

        print >>fx,"{0},{1},{2},{3}".format(
            self.elapsed_time,
            data.x,data.y,data.height
        )

    def handle_velocity(self,data):
        self.vel = data
        vx = data.velx
        vy = data.vely
        vz = data.velz
        #print "vx {0} vy {1} vz {2}".format(vx,vy,vz)
        if self.elapsed_time > 0.01:
            #print "{0},{1},{2}".format(self.time,vx,self.vel_sp.velx)
            pass


    def handle_acceleration(self,data):
        self.acc.ax = data.ax
        self.acc.ay = data.ay
        self.acc.az = data.az
        if self.elapsed_time > 0.01:
            return
        pass

    def handle_rc_channels(self,data):
        self.rc_channels.roll = data.roll
        self.rc_channels.pitch = data.pitch
        self.rc_channels.yaw = data.yaw
        self.rc_channels.throttle = data.throttle
        self.rc_channels.mode = data.mode
        self.rc_channels.gear = data.gear

    def send_cmd(self,a):
        req = Float32()
        req.data = a
        # print "send cmd takeoff"
        self.cmd_pub.publish(req)

    def handle_status(self,a):
        self.status = math.floor(a.data)

    def set_takeoff(self):
        self.cmd_sp = 4

    def battery_check_callback(self,data):
        #print "battery is {0}".format(data)
        pass

    def battery_check_test(self):
        rospy.Subscriber("battery_status", Float32, self.battery_check_callback)

    def init_subscribers(self):
        self.act_sub = rospy.Subscriber(self.ns + '/activation_status',Float32,self.handle_activation)
        self.open_close_sub = rospy.Subscriber(self.ns + '/nav_open_close_status',Float32,self.handle_open_close_control)
        self.status_sub = rospy.Subscriber(self.ns + '/flight_status',Float32,self.handle_status)
        self.att_sub = rospy.Subscriber(self.ns + '/attitude_quad',attitude_quad,self.handle_attitude_quat)
        self.global_pos_sub = rospy.Subscriber(self.ns + '/global_position',global_position,self.handle_global_pos)
        self.handle_vel_sub = rospy.Subscriber(self.ns + '/velocity',velocity,self.handle_velocity)
        self.acc_sub = rospy.Subscriber(self.ns + '/acceleration',acc,self.handle_acceleration)
        self.local_pos_sub = rospy.Subscriber(self.ns + '/local_position',local_position,self.handle_local_position)
        self.rc_channels_sub = rospy.Subscriber(self.ns + '/rc_channels',rc_channels,self.handle_rc_channels)

    def init_publishers(self):
        self.cmd_pub = rospy.Publisher(self.ns + '/sdk_request_cmd',Float32)
        self.act_pub = rospy.Publisher(self.ns + '/sdk_request_activation',Float32)
        self.open_close_pub = rospy.Publisher(self.ns + '/nav_open_close_request',Float32)

        self.vel_pub = rospy.Publisher(self.ns + '/velocity_setpoint',velocity)
        self.acc_pub = rospy.Publisher(self.ns + '/acc_setpoint',acc)
        self.gimbal_pub = rospy.Publisher(self.ns + '/gimbal_setpoint',gimbal)
        self.local_pos_pub = rospy.Publisher(self.ns + '/local_position_setpoint',local_position)
        self.lookat_cam_pub = rospy.Publisher(self.ns + '/lookat_setpoint',local_position)

    def init_a_service(self,name):
        rospy.wait_for_service(self.ns + '/' + name)
        service_func = rospy.ServiceProxy(self.ns + '/' + name, eval(name))
        self.services[name] = service_func
        print "inited service {0}".format(name)
        pass

    def init_services(self):
        print "initing service..."
        self.services = dict()
        service_list = [
            "set_local_position_ref",
            "set_lookat_local",
            "fly_to_local",
            "set_velocity",
            "set_gimbal_angles"
            ]
        map(self.init_a_service,service_list)

    def __init__(self, namespace=''):
        rospy.init_node('dji_connector_test')
        self.ns=namespace
        self.count = 0

        self.elapsed_time = 0

        self.cmd_sp = 0
        self.status = 0

        self.open = 0
        self.need_open = True

        self.acted = 0
        self.att_quad = attitude_quad()
        self.global_pos = global_position()
        self.local_pos = local_position()
        self.vel = velocity()
        self.acc = acc()
        self.gimbal_sp = gimbal()
        self.rc_channels = rc_channels()
        self.sp_mode = -1

        self.acc_sp = acc()

        self.lookat_enable = False


        self.init_publishers()
        self.init_subscribers()
        self.init_services()
        self.seted_ref = 0

        self.update_time = rospy.Timer(rospy.Duration(0.02), self.update)


    def landed(self):
        return self.status == status['STANDBY'] \
               or self.status == status['LANDING'] \
               or self.status == status['POSTLANGING']

    def takedoff(self):
        return self.status == status['TAKINGOFF'] \
        or self.status == status['INAIR']

    def need_cmd(self):
        if self.cmd_sp == 4:
            return not self.takedoff()
        if self.cmd_sp == 6:
            return not self.landed()
        if self.cmd_sp == 1:
            return not self.landed()

        return True

    def update_modes(self):
        if self.acted == 0:
            # print "try act"
            self.send_activation()

        if self.need_open:
            self.send_open_close_control()
            pass
        if self.need_cmd():
            self.send_cmd(self.cmd_sp)
            pass

    def update_sp(self):
        if self.sp_mode == 4:
            self.vel_pub.publish(self.vel_sp)
        elif self.sp_mode == 1:
            self.acc_pub.publish(self.acc_sp)
        elif self.sp_mode == 12:
            self.local_pos_pub.publish(self.local_pos_sp)
            data = self.local_pos_sp
            """
            print >> fy,"{0},{1},{2}".format(
                self.local_pos_sp.x,
                self.local_pos_sp.y,
                self.local_pos_sp.height
            )
            """
        if self.lookat_enable:
            # self.lookat_cam_pub.publish(self.lookat_cam_sp)
            pass
        else:
            self.gimbal_pub.publish(self.gimbal_sp)
        pass

    def update(self,e):
        self.count += 1
        if self.count % 5 == 0:
            self.update_modes()
        self.update_sp()

    def set_return2home(self):
        self.cmd_sp = 1

    def set_land(self):
        self.cmd_sp = 6

    def set_stop(self):
        self.sp_mode = -1

    def set_acc(self,ax = 0,ay = 0 ,vz = 0, yaw = 0):
        self.sp_mode = 1
        self.acc_sp.ax = ax
        self.acc_sp.ay = ay
        self.acc_sp.vz = vz
        self.acc_sp.yaw = yaw

def splinetool():
    dt = 0.04
    data = [i for i in range(10/dt)]

if __name__ == "__main__":
    yqx = dji_conn()
    yqx.set_takeoff()
    for i in range(1000):
        #yqx.set_local_ref()
        a = velocity(velx = 10,vely = 10,velz = 0)
        yqx.services["set_velocity"](a)
        time.sleep(0.02)

    rospy.spin()
