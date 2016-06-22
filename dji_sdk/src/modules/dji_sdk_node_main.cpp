#include <dji_sdk/dji_sdk_node.h>
#include <functional>
#define DEG2RAD(DEG) ((DEG)*((C_PI)/(180.0)))

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

#define MAX_UINT32 0xffffffff

double DJISDKNode::deltaTimeStamp(DJI::onboardSDK::TimeStampData& current_time, DJI::onboardSDK::TimeStampData& prev_time) {
    unsigned long delta;

    if (current_time.nanoTime < prev_time.nanoTime)
    {
        delta  = (MAX_UINT32 - prev_time.nanoTime) + current_time.nanoTime;
    }
    else
    {
        delta = current_time.nanoTime - prev_time.nanoTime;
    }

    return (double)delta / 1000000000.0;
}

void DJISDKNode::broadcast_callback()
{
    DJI::onboardSDK::BroadcastData bc_data = rosAdapter->coreAPI->getBroadcastData();
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
        LOG_MSG_STAMP("/dji_sdk/attitude_quaternion", attitude_quaternion, current_time, 2);
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
        LOG_MSG_STAMP("/dji_sdk/global_position", global_position, current_time, 1);

        //TODO:
        // FIX BUG about flying at lat = 0
        if (global_position.ts != 0 && global_position_ref_seted == 0 && global_position.latitude != 0 && global_position.health > 3) {
            global_position_ref = global_position;
            global_position_ref_seted = 1;
        }

        //update local_position msg
        if (!localpos_odometry) {
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
            LOG_MSG_STAMP("/dji_sdk/local_position", local_position, current_time, 1);
        }
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
        LOG_MSG_STAMP("/dji_sdk/velocity", velocity, current_time, 2);

        if (localpos_odometry && localpos_init) {
            ros::Duration d = prev_time - ros::Time::now();
            double dt = deltaTimeStamp(bc_data.timeStamp, localpos_prevtime);
//            printf("#### VEL[%0.5f %0.5f %0.5f]  time[%lu  %lu : %lu  : %0.9f  %0.9f]\n", velocity.vx, velocity.vy, velocity.vz,
//                    cnt++, (unsigned long)bc_data.timeStamp.time, (unsigned long)bc_data.timeStamp.nanoTime, d.toSec(), dt);
            prev_time = ros::Time::now();
            localpos_prevtime = bc_data.timeStamp;
            local_position.header.frame_id = "/world";
            local_position.header.stamp = current_time;
            local_position.x += velocity.vx * dt;
            local_position.y += velocity.vy * dt;
            local_position.z += velocity.vz * dt;

//            gps_convert_ned(
//                    local_position.x,
//                    local_position.y,
//                    global_position.longitude,
//                    global_position.latitude,
//                    global_position_ref.longitude,
//                    global_position_ref.latitude
//            );
//            local_position.z = global_position.height;
            local_position.ts = global_position.ts;
            local_position_ref = local_position;
            local_position_publisher.publish(local_position);
            LOG_MSG_STAMP("/dji_sdk/local_position", local_position, current_time, 1);
        }
        else
        {
            localpos_init = true;
            localpos_prevtime = bc_data.timeStamp;
            prev_time = ros::Time::now();
        }
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
        LOG_MSG_STAMP("/dji_sdk/acceleration", acceleration, current_time, 2);
    }

    //update gimbal msg
    if (msg_flags & HAS_GIMBAL) {
        gimbal.header.frame_id = "/gimbal";
        gimbal.header.stamp= current_time;
        gimbal.ts = bc_data.timeStamp.time;
        gimbal.roll = bc_data.gimbal.roll;
        gimbal.pitch = bc_data.gimbal.pitch;
        gimbal.yaw = bc_data.gimbal.yaw;
        gimbal_publisher.publish(gimbal);
        LOG_MSG_STAMP("/dji_sdk/gimbal", gimbal, current_time, 1);
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
        LOG_MSG_STAMP("/dji_sdk/odometry", odometry, current_time, 1);
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
        LOG_MSG_STAMP("/dji_sdk/rc_channels", rc_channels, current_time, 2);
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
        LOG_MSG_STAMP("/dji_sdk/compass", compass, current_time, 2);
    }

    //update flight_status 
    if (msg_flags & HAS_STATUS) {
        std_msgs::UInt8 msg;
        flight_status = bc_data.status;
        msg.data = flight_status;
        flight_status_publisher.publish(msg);
        LOG_MSG_STAMP("/dji_sdk/flight_status", msg, current_time, 1);
    }

    //update battery msg
    if (msg_flags & HAS_BATTERY) {
        power_status.percentage = bc_data.battery;
        power_status_publisher.publish(power_status);
        LOG_MSG_STAMP("/dji_sdk/power_status", power_status, current_time, 1);
    }

    //update flight control info
    if (msg_flags & HAS_DEVICE) {
		flight_control_info.control_mode = bc_data.ctrlInfo.data;
        flight_control_info.cur_ctrl_dev_in_navi_mode = bc_data.ctrlInfo.device;
        flight_control_info.serial_req_status = bc_data.ctrlInfo.signature;
		flight_control_info.virtual_rc_status = bc_data.ctrlInfo.virtualrc;
        flight_control_info_publisher.publish(flight_control_info);
        LOG_MSG_STAMP("/dji_sdk/flight_control_info", flight_control_info, current_time, 2);
    }

    //update obtaincontrol msg
	std_msgs::UInt8 msg;
	sdk_permission_opened = bc_data.ctrlInfo.data;
	msg.data = bc_data.ctrlInfo.data;
	sdk_permission_publisher.publish(msg);

	activation_result = bc_data.activation;
	msg.data = bc_data.activation;
	activation_publisher.publish(msg);

}



int DJISDKNode::init_parameters(ros::NodeHandle& nh_private)
{
    std::string serial_name;
    int baud_rate;
    int app_id;
    int app_version;
    std::string app_bundle_id; //reserved
    std::string enc_key;

    int uart_or_usb;
    int A3_or_M100;

    nh_private.param("serial_name", serial_name, std::string("/dev/ttyTHS1"));
    nh_private.param("baud_rate", baud_rate, 230400);
    nh_private.param("app_id", app_id, 1022384);
    nh_private.param("app_version", app_version, 1);
    nh_private.param("enc_key", enc_key,
            std::string("e7bad64696529559318bb35d0a8c6050d3b88e791e1808cfe8f7802150ee6f0d"));

    nh_private.param("uart_or_usb", uart_or_usb, 0);//chosse uart as default
    nh_private.param("A3_or_M100", A3_or_M100, 1);//chosse M100 as default

    // activation
    user_act_data.ID = app_id;

    if((uart_or_usb)&&(A3_or_M100))
    {
        printf("M100 does not support USB API.\n");
        return -1;
    }

    if(A3_or_M100)
    {
        user_act_data.version = 0x03010a00;
    }
    else
    {
        user_act_data.version = 0x03016400;
    }
    user_act_data.encKey = app_key;
    strcpy(user_act_data.encKey, enc_key.c_str());

    printf("=================================================\n");
    printf("app id: %d\n", user_act_data.ID);
    printf("app version: 0x0%X\n", user_act_data.version);
    printf("app key: %s\n", user_act_data.encKey);
    printf("=================================================\n");

    if(uart_or_usb) //use usb port for SDK
    {
        rosAdapter->usbHandshake(serial_name);
    }

    rosAdapter->init(serial_name, baud_rate);

    rosAdapter->activate(&user_act_data, NULL);
    rosAdapter->setBroadcastCallback(&DJISDKNode::broadcast_callback, this);
	rosAdapter->setFromMobileCallback(&DJISDKNode::transparent_transmission_callback,this);

	printf("\n");
    // Check for odometry local location mode
    nh_private.getParam("/Location/odometry", localpos_odometry);
    ROS_INFO("DJI_NODE: Using velocity odometry for local positioning[%s]", localpos_odometry ? "TRUE" : "FALSE");

    // Check for reference lat lon coordiantes
	if (nh_private.hasParam("/World/Origin/latitude") && nh_private.hasParam("/World/Origin/longitude"))
	{
        nh_private.param("/World/Origin/longitude", global_position_ref.longitude, 1.1);
        nh_private.param("/World/Origin/latitude", global_position_ref.latitude, 2.2);
        if (global_position_ref.longitude == 1.1 || global_position_ref.latitude == 2.2)
        {
            ROS_ERROR("DJI_NODE: Illegal or missing origin latitude longitude coordinates");
        }
        else
        {
            ROS_INFO("DJI_NODE: Origin latitude[%f]  longitude[%f]",
                    global_position_ref.latitude, global_position_ref.longitude);
            global_position_ref_seted = 1;
        }
	}

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



inline void DJISDKNode::gps_convert_ned(float &ned_x, float &ned_y,
            double gps_t_lon, double gps_t_lat,
            double gps_r_lon, double gps_r_lat)
{
    double d_lon = gps_t_lon - gps_r_lon;
    double d_lat = gps_t_lat - gps_r_lat;
    ned_x = DEG2RAD(d_lat) * C_EARTH;
    ned_y = DEG2RAD(d_lon) * C_EARTH * cos(DEG2RAD(gps_t_lat));
}



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

void DJISDKNode::logControlCB(const dji_sdk::LogControl::ConstPtr& msg)
{
    ROS_INFO("DJI: STARTING THE LOGGER TO %s, level %d", msg->name.c_str(), msg->level);
    if (msg->enable)
    {
        BagLogger::instance()->startLogging(msg->name, msg->level);
    }
    else
    {
        BagLogger::instance()->stopLogging();
    }
}

