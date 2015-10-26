#include "djiPublisher.h"
#include "std_msgs/UInt8.h"
#include <dji_sdk/attitude_quad.h>
#include <dji_sdk/global_position.h>
#include <dji_sdk/local_position.h>
#include <dji_sdk/velocity.h>
#include <dji_sdk/acc.h>
#include <dji_sdk/gimbal.h>
#include <dji_sdk/rc_channels.h>
#include <nav_msgs/Odometry.h>

namespace publishers
{
	ros::Publisher battery_status_pub;
	ros::Publisher ctrl_info_pub;
	ros::Publisher flight_status_pub; 
	ros::Publisher acc_pub;
	ros::Publisher gimbal_info_pub;
	ros::Publisher gps_pub;
	ros::Publisher att_quad_pub;
	ros::Publisher compass_pub;
	ros::Publisher vel_pub;
	ros::Publisher local_pos_pub;
	ros::Publisher rc_channels_pub;
	ros::Publisher control_pub;
	ros::Publisher activation_pub;
	ros::Publisher odom_pub;

	int init_publishers(ros::NodeHandle &nh)
	{
		// start ros publisher
		publishers::acc_pub = nh.advertise<dji_sdk::acc>("dji_sdk/acceleration", 10);
		publishers::att_quad_pub = nh.advertise<dji_sdk::attitude_quad>("dji_sdk/attitude_quaternion", 10);
		publishers::battery_status_pub = nh.advertise<std_msgs::UInt8>("dji_sdk/battery_status", 10);
		publishers::gimbal_info_pub = nh.advertise<dji_sdk::gimbal>("dji_sdk/gimbal_info", 10);
		publishers::flight_status_pub = nh.advertise<std_msgs::UInt8>("dji_sdk/flight_status", 10);
		publishers::gps_pub = nh.advertise<dji_sdk::global_position>("dji_sdk/global_position", 10);
		publishers::local_pos_pub = nh.advertise<dji_sdk::local_position>("dji_sdk/local_position", 10);
		publishers::odom_pub = nh.advertise<nav_msgs::Odometry>("dji_sdk/odometry",10);
		publishers::vel_pub = nh.advertise<dji_sdk::velocity>("dji_sdk/velocity", 10);
		publishers::rc_channels_pub = nh.advertise<dji_sdk::rc_channels>("dji_sdk/rc_channels",10);
        publishers::control_pub = nh.advertise<std_msgs::UInt8>("dji_sdk/obtained_control",10);
        publishers::activation_pub = nh.advertise<std_msgs::UInt8>("dji_sdk/activation_result",10);
		publishers::ctrl_info_pub = nh.advertise<dji_sdk::ctrl_info>("dji_sdk/ctrl_info", 10);
		publishers::compass_pub = nh.advertise<dji_sdk::compass>("dji_sdk/compass_info", 10);
		return 0;
	}
};
