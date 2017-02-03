/** @file dji_sdk_node_main.cpp
 *  @version 3.1.8
 *  @date July 29th, 2016
 *
 *  @brief
 *  Broadcast and Mobile callbacks are implemented here. 
 *
 *  @copyright 2016 DJI. All rights reserved.
 *
 */


#include <dji_sdk/dji_sdk_node.h>
#include <functional>
#include <dji_sdk/DJI_LIB_ROS_Adapter.h>
//----------------------------------------------------------
// timer spin_function 50Hz
//----------------------------------------------------------

void DJISDKNode::transparent_transmission_callback(uint8_t *buf, uint8_t len)
{
	dji_sdk::TransparentTransmissionData transparent_transmission_data;
	transparent_transmission_data.data.resize(len);
	memcpy(&transparent_transmission_data.data[0], buf, len);
	data_received_from_remote_device_publisher.publish(transparent_transmission_data);

}
void DJISDKNode::broadcast_callback()
{
    DJI::onboardSDK::BroadcastData bc_data = rosAdapter->coreAPI->getBroadcastData();

    DJI::onboardSDK::Version version = rosAdapter->coreAPI->getFwVersion();
    unsigned short msg_flags = bc_data.dataFlag;

    static int frame_id = 0;
    frame_id ++;
   
    auto current_time = ros::Time::now();

    if(msg_flags & HAS_TIME){
        time_stamp.header.frame_id = "/time";
        time_stamp.header.stamp = current_time;
        time_stamp.time = bc_data.timeStamp.time;
        time_stamp.time_ns = bc_data.timeStamp.nanoTime;
        time_stamp.sync_flag = bc_data.timeStamp.syncFlag;
        time_stamp_publisher.publish(time_stamp);
    }

    //update attitude msg
    if ( (msg_flags & HAS_Q) && (msg_flags & HAS_W) ) {
        attitude_quaternion.header.frame_id = "/world";
        attitude_quaternion.header.stamp = current_time;
        attitude_quaternion.q0 = bc_data.q.q0;
        attitude_quaternion.q1 = bc_data.q.q1;
        attitude_quaternion.q2 = bc_data.q.q2;
        attitude_quaternion.q3 = bc_data.q.q3;
        attitude_quaternion.wx = bc_data.w.x;
        attitude_quaternion.wy = bc_data.w.y;
        attitude_quaternion.wz = bc_data.w.z;
        attitude_quaternion.ts = bc_data.timeStamp.time;
        attitude_quaternion_publisher.publish(attitude_quaternion);
    }

    //update global_position msg
    if (msg_flags & HAS_POS) {
        global_position.header.frame_id = "/world";
        global_position.header.stamp = current_time;
        global_position.ts = bc_data.timeStamp.time;
        global_position.latitude = bc_data.pos.latitude * 180.0 / C_PI;
        global_position.longitude = bc_data.pos.longitude * 180.0 / C_PI;
        global_position.height = bc_data.pos.height;
        global_position.altitude = bc_data.pos.altitude;
        global_position.health = bc_data.pos.health;
        global_position_publisher.publish(global_position);

        //TODO:
        // FIX BUG about flying at lat = 0
        if (global_position.ts != 0 && global_position_ref_seted == 0 && global_position.latitude != 0 && global_position.health > 3) {
            global_position_ref = global_position;
            global_position_ref_seted = 1;
        }

        //update local_position msg
        local_position.header.frame_id = "/world";
        local_position.header.stamp = current_time;
        gps_convert_ned(
                local_position.x,
                local_position.y,
                global_position.longitude,
                global_position.latitude,
                global_position_ref.longitude,
                global_position_ref.latitude
                );
        local_position.z = global_position.height;
        local_position.ts = global_position.ts;
        local_position_ref = local_position;
        local_position_publisher.publish(local_position);
    }


    //update velocity msg
    if (msg_flags & HAS_V) {
        velocity.header.frame_id = "/world";
        velocity.header.stamp = current_time;
        velocity.ts = bc_data.timeStamp.time;
        velocity.vx = bc_data.v.x;
        velocity.vy = bc_data.v.y;
        velocity.vz = bc_data.v.z;
        velocity.health_flag = bc_data.v.health;
        velocity_publisher.publish(velocity);
    }

    //update acceleration msg
    if (msg_flags & HAS_A) {
        acceleration.header.frame_id = "/world";
        acceleration.header.stamp = current_time;
        acceleration.ts = bc_data.timeStamp.time;
        acceleration.ax = bc_data.a.x;
        acceleration.ay = bc_data.a.y;
        acceleration.az = bc_data.a.z;
        acceleration_publisher.publish(acceleration);
    }
    

    //update odom msg
    if ( (msg_flags & HAS_POS) && (msg_flags & HAS_Q) && (msg_flags & HAS_W) && (msg_flags & HAS_V) ) {
        odometry.header.frame_id = "/world";
        odometry.header.stamp = current_time;
        odometry.pose.pose.position.x = local_position.x;
        odometry.pose.pose.position.y = local_position.y;
        odometry.pose.pose.position.z = local_position.z;
        odometry.pose.pose.orientation.w = attitude_quaternion.q0;
        odometry.pose.pose.orientation.x = attitude_quaternion.q1;
        odometry.pose.pose.orientation.y = attitude_quaternion.q2;
        odometry.pose.pose.orientation.z = attitude_quaternion.q3;
        odometry.twist.twist.angular.x = attitude_quaternion.wx;
        odometry.twist.twist.angular.y = attitude_quaternion.wy;
        odometry.twist.twist.angular.z = attitude_quaternion.wz;
        odometry.twist.twist.linear.x = velocity.vx;
        odometry.twist.twist.linear.y = velocity.vy;
        odometry.twist.twist.linear.z = velocity.vz;
        odometry_publisher.publish(odometry);
    }

/******************************************************************
****************************If using A3/N3/M600********************
******************************************************************/

    if(version > MAKE_VERSION(3,1,10,0)) {

    	//update gimbal msg
    	if (msg_flags & A3_HAS_GIMBAL) {
        	gimbal.header.frame_id = "/gimbal";
        	gimbal.header.stamp= current_time;
        	gimbal.ts = bc_data.timeStamp.time;
        	gimbal.roll = bc_data.gimbal.roll;
       	 	gimbal.pitch = bc_data.gimbal.pitch;
        	gimbal.yaw = bc_data.gimbal.yaw;
        	gimbal_publisher.publish(gimbal);
    	}

   	 //update rc_channel msg
    	if (msg_flags & A3_HAS_RC) {
        	rc_channels.header.frame_id = "/rc";
        	rc_channels.header.stamp = current_time;
        	rc_channels.ts = bc_data.timeStamp.time;
       	 	rc_channels.pitch = bc_data.rc.pitch;
        	rc_channels.roll = bc_data.rc.roll;
        	rc_channels.mode = bc_data.rc.mode;
        	rc_channels.gear = bc_data.rc.gear;
        	rc_channels.throttle = bc_data.rc.throttle;
        	rc_channels.yaw = bc_data.rc.yaw;
        	rc_channels_publisher.publish(rc_channels);
    	}


	if (msg_flags & A3_HAS_GPS){
		A3_GPS.date = bc_data.gps.date;
		A3_GPS.time = bc_data.gps.time;
		A3_GPS.longitude = bc_data.gps.longitude;
		A3_GPS.latitude = bc_data.gps.latitude;
		A3_GPS.height_above_sea = bc_data.gps.Hmsl;
		A3_GPS.velocity_north = bc_data.gps.velocityNorth;
		A3_GPS.velocity_east= bc_data.gps.velocityEast;
		A3_GPS.velocity_ground = bc_data.gps.velocityGround;
		A3_GPS_info_publisher.publish(A3_GPS);
	}
	if (msg_flags & A3_HAS_RTK)
		A3_RTK.date = bc_data.rtk.date;
		A3_RTK.time = bc_data.rtk.time;
		A3_RTK.longitude_RTK = bc_data.rtk.longitude;
		A3_RTK.latitude_RTK = bc_data.rtk.latitude;
		A3_RTK.height_above_sea_RTK = bc_data.rtk.Hmsl;
		A3_RTK.velocity_north = bc_data.rtk.velocityNorth;
		A3_RTK.velocity_east = bc_data.rtk.velocityEast;
		A3_RTK.velocity_ground = bc_data.rtk.velocityGround;
		A3_RTK.yaw = bc_data.rtk.yaw;
		A3_RTK.position_flag = bc_data.rtk.posFlag;
		A3_RTK.yaw_flag = bc_data.rtk.yawFlag;
		A3_RTK_info_publisher.publish(A3_RTK);

    	//update compass msg
    	if (msg_flags & A3_HAS_MAG) {
        	compass.header.frame_id = "/world";
        	compass.header.stamp = current_time;
        	compass.ts = bc_data.timeStamp.time;
        	compass.x = bc_data.mag.x;
        	compass.y = bc_data.mag.y;
        	compass.z = bc_data.mag.z;
        	compass_publisher.publish(compass);
    	}

    	//update flight_status
    	if (msg_flags & A3_HAS_STATUS) {
        	std_msgs::UInt8 msg;
        	flight_status = bc_data.status;
        	msg.data = flight_status;
        	flight_status_publisher.publish(msg);
    	}

    	//update battery msg
    	if (msg_flags & A3_HAS_BATTERY) {
        	power_status.percentage = bc_data.battery;
        	power_status_publisher.publish(power_status);
    	}

    	//update flight control info
    	if (msg_flags & A3_HAS_DEVICE) {
		flight_control_info.control_mode = bc_data.ctrlInfo.mode;
        	flight_control_info.cur_ctrl_dev_in_navi_mode = bc_data.ctrlInfo.deviceStatus;
        	flight_control_info.serial_req_status = bc_data.ctrlInfo.flightStatus;
		flight_control_info.virtual_rc_status = bc_data.ctrlInfo.vrcStatus;
        	flight_control_info_publisher.publish(flight_control_info);
    	}

    }

/******************************************************************
***************************If using M100***************************
******************************************************************/

    else {

     	if (msg_flags & HAS_GIMBAL) {
        	gimbal.header.frame_id = "/gimbal";
        	gimbal.header.stamp= current_time;
        	gimbal.ts = bc_data.timeStamp.time;
        	gimbal.roll = bc_data.gimbal.roll;
        	gimbal.pitch = bc_data.gimbal.pitch;
        	gimbal.yaw = bc_data.gimbal.yaw;
        	gimbal_publisher.publish(gimbal);
    	}

    	//update rc_channel msg
    	if (msg_flags & HAS_RC) {
       	 	rc_channels.header.frame_id = "/rc";
        	rc_channels.header.stamp = current_time;
        	rc_channels.ts = bc_data.timeStamp.time;
        	rc_channels.pitch = bc_data.rc.pitch;
        	rc_channels.roll = bc_data.rc.roll;
        	rc_channels.mode = bc_data.rc.mode;
        	rc_channels.gear = bc_data.rc.gear;
        	rc_channels.throttle = bc_data.rc.throttle;
        	rc_channels.yaw = bc_data.rc.yaw;
        	rc_channels_publisher.publish(rc_channels);
    	}



    	//update compass msg
    	if (msg_flags & HAS_MAG) {
        	compass.header.frame_id = "/world";
        	compass.header.stamp = current_time;
        	compass.ts = bc_data.timeStamp.time;
        	compass.x = bc_data.mag.x;
        	compass.y = bc_data.mag.y;
        	compass.z = bc_data.mag.z;
        	compass_publisher.publish(compass);
    	}	

    	//update flight_status
    	if (msg_flags & HAS_STATUS) {
        	std_msgs::UInt8 msg;
        	flight_status = bc_data.status;
        	msg.data = flight_status;
        	flight_status_publisher.publish(msg);
    	}

    	//update battery msg
    	if (msg_flags & HAS_BATTERY) {
        	power_status.percentage = bc_data.battery;
        	power_status_publisher.publish(power_status);
    	}

    	//update flight control info
    	if (msg_flags & HAS_DEVICE) {
		flight_control_info.control_mode = bc_data.ctrlInfo.mode;
        	flight_control_info.cur_ctrl_dev_in_navi_mode = bc_data.ctrlInfo.deviceStatus;
        	flight_control_info.serial_req_status = bc_data.ctrlInfo.flightStatus;
		flight_control_info.virtual_rc_status = bc_data.ctrlInfo.vrcStatus;
        	flight_control_info_publisher.publish(flight_control_info);
    	}

    }



    //update obtaincontrol msg
	std_msgs::UInt8 msg;
	activation_result = bc_data.activation;
	msg.data = bc_data.activation;
	activation_publisher.publish(msg);

}



int DJISDKNode::init_parameters(ros::NodeHandle& nh_private)
{
    std::string drone_version;
    std::string serial_name;
    int baud_rate;
    int app_id;
    std::string app_bundle_id; //reserved
    std::string enc_key;
    int uart_or_usb;
    

    nh_private.param("serial_name", serial_name, std::string("/dev/ttyTHS1"));
    nh_private.param("baud_rate", baud_rate, 230400);
    nh_private.param("app_id", app_id, 1022384);
    nh_private.param("enc_key", enc_key,
            std::string("e7bad64696529559318bb35d0a8c6050d3b88e791e1808cfe8f7802150ee6f0d"));

    nh_private.param("uart_or_usb", uart_or_usb, 0);//chosse uart as default
    nh_private.param("drone_version", drone_version, std::string("M100"));//chosse M100 as default

    // activation
    user_act_data.ID = app_id;
    user_act_data.encKey = app_key;
    strcpy(user_act_data.encKey, enc_key.c_str());

    printf("=================================================\n");
    printf("app id   : %d\n", user_act_data.ID);
    printf("app key  : %s\n", user_act_data.encKey);
    printf("=================================================\n");

    if(uart_or_usb) //use usb port for SDK
    {
        rosAdapter->usbHandshake(serial_name);
    }

    rosAdapter->init(serial_name, baud_rate);
    rosAdapter->coreAPI->getDroneVersion();
    //usleep(1000000);
    ros::Duration(1.0).sleep();
    printf("=================================================\n");
    printf("Hardware : %s\n", rosAdapter->coreAPI->getHwVersion());
    printf("Firmware : 0x0%X\n", rosAdapter->coreAPI->getFwVersion());
    printf("=================================================\n");

    rosAdapter->activate(&user_act_data, NULL);
    rosAdapter->setBroadcastCallback(&DJISDKNode::broadcast_callback, this);
    rosAdapter->setFromMobileCallback(&DJISDKNode::transparent_transmission_callback,this);
   
    return 0;
}



DJISDKNode::DJISDKNode(ros::NodeHandle& nh, ros::NodeHandle& nh_private) : dji_sdk_mission(nullptr)
{

    init_publishers(nh);
    init_services(nh);
    init_actions(nh);


    init_parameters(nh_private);

    int groundstation_enable;
    nh_private.param("groundstation_enable", groundstation_enable, 1);
    if(groundstation_enable)
    {
        dji_sdk_mission = new DJISDKMission(nh);
    }

}


void DJISDKNode::gps_convert_ned(float &ned_x, float &ned_y,
		double gps_t_lon, double gps_t_lat,
		double gps_r_lon, double gps_r_lat)
{
	double d_lon = gps_t_lon - gps_r_lon;
	double d_lat = gps_t_lat - gps_r_lat;
	ned_x = DEG2RAD(d_lat) * C_EARTH;
	ned_y = DEG2RAD(d_lon) * C_EARTH * cos(DEG2RAD(gps_t_lat));
};

dji_sdk::LocalPosition DJISDKNode::gps_convert_ned(dji_sdk::GlobalPosition loc)
{
    dji_sdk::LocalPosition local;
    gps_convert_ned(local.x, local.y,
        loc.longitude, loc.latitude,
        global_position_ref.longitude, global_position_ref.latitude
    );
    local.z = loc.height;
    return local;
}

