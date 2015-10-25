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
	ros::Publisher battery_pub, ctrl_info_pub,
		flight_status_pub, acc_pub, gimbal_info_pub;
	ros::Publisher gps_pub, att_quad_pub, compass_pub,
		vel_pub, local_pos_pub,rc_channels_pub,control_publisher, activation_publisher;
	ros::Publisher odem_publisher;
	int init_publishers(ros::NodeHandle &nh)
	{
		// start ros publisher
		publishers::acc_pub = nh.advertise<dji_sdk::acc>("dji_sdk/acceleration", 10);
		publishers::att_quad_pub = nh.advertise<dji_sdk::attitude_quad>("dji_sdk/attitude_quad", 10);
		publishers::battery_pub = nh.advertise<std_msgs::UInt8>("dji_sdk/battery_status", 10);
		publishers::gimbal_info_pub = nh.advertise<dji_sdk::gimbal>("dji_sdk/gimbal_info", 10);
		publishers::flight_status_pub = nh.advertise<std_msgs::UInt8>("dji_sdk/flight_status", 10);
		publishers::gps_pub = nh.advertise<dji_sdk::global_position>("dji_sdk/global_position", 10);
		publishers::local_pos_pub = nh.advertise<dji_sdk::local_position>("dji_sdk/local_position", 10);
		publishers::odem_publisher = nh.advertise<nav_msgs::Odometry>("dji_sdk/odom",10);
		publishers::vel_pub = nh.advertise<dji_sdk::velocity>("dji_sdk/velocity", 10);
		publishers::rc_channels_pub = nh.advertise<dji_sdk::rc_channels>("dji_sdk/rc_channels",10);
        publishers::control_publisher = nh.advertise<std_msgs::UInt8>("dji_sdk/obtained_control",10);
        publishers::activation_publisher = nh.advertise<std_msgs::UInt8>("dji_sdk/activation_result",10);
		publishers::ctrl_info_pub = nh.advertise<dji_sdk::ctrl_info>("dji_sdk/ctrl_info", 10);
		publishers::compass_pub = nh.advertise<dji_sdk::compass>("dji_sdk/compass_info", 10);
		return 0;
	}
};
