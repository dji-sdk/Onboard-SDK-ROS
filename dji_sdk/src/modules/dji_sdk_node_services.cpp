#include <dji_sdk/dji_sdk_node.h>

bool DJISDKNode::activation_callback(dji_sdk::Activation::Request& request, dji_sdk::Activation::Response& response)
{
	////DJI_Pro_Activate_API(&user_act_data, NULL);
//TODO: [__CHRIS__a8]
	rosAdapter->coreAPI->activate(&user_act_data, NULL);
	response.result = true;
	return true;
}

bool DJISDKNode::attitude_control_callback(dji_sdk::AttitudeControl::Request& request, dji_sdk::AttitudeControl::Response& response)
{
///
/*
    attitude_data_t user_ctrl_data;

    user_ctrl_data.ctrl_flag = request.flag;
    user_ctrl_data.roll_or_x = request.x;
    user_ctrl_data.pitch_or_y = request.y;
    user_ctrl_data.thr_z = request.z;
    user_ctrl_data.yaw = request.yaw;
*/
///

    ////DJI_Pro_Attitude_Control(&user_ctrl_data);
//TODO: [__CHRIS__a9]
    DJI::onboardSDK::FlightData flight_ctrl_data;
    flight_ctrl_data.ctrl_flag = request.flag;
    flight_ctrl_data.roll_or_x = request.x;
    flight_ctrl_data.pitch_or_y = request.y;
    flight_ctrl_data.thr_z = request.z;
    flight_ctrl_data.yaw = request.yaw;
    rosAdapter->flight->setFlight(&flight_ctrl_data);

    response.result = true;
    return true;
}

bool DJISDKNode::camera_action_control_callback(dji_sdk::CameraActionControl::Request& request, dji_sdk::CameraActionControl::Response& response)
{
    if (request.camera_action == 0) {
        ////DJI_Pro_Camera_Control(API_CAMERA_SHOT);
//TODO: [__CHRIS__a10]
	rosAdapter->camera->setCamera(DJI::onboardSDK::Camera::CAMERA_CODE::CODE_CAMERA_SHOT);
        response.result = true;
    }
    else if (request.camera_action == 1) {
        ////DJI_Pro_Camera_Control(API_CAMERA_VIDEO_START);
	rosAdapter->camera->setCamera(DJI::onboardSDK::Camera::CAMERA_CODE::CODE_CAMERA_VIDEO_START);
        response.result = true;
    }
    else if (request.camera_action == 2) {
        ////DJI_Pro_Camera_Control(API_CAMERA_VIDEO_STOP);
	rosAdapter->camera->setCamera(DJI::onboardSDK::Camera::CAMERA_CODE::CODE_CAMERA_VIDEO_STOP);
        response.result = true;
    }
    else {
        response.result = false;
    }
    return true;
}

bool DJISDKNode::drone_task_control_callback(dji_sdk::DroneTaskControl::Request& request, dji_sdk::DroneTaskControl::Response& response)
{
    if (request.task== 4) {
        //takeoff
        ////DJI_Pro_Status_Ctrl(4, 0);
//TODO: [__CHRIS__a11]
	rosAdapter->flight->task(DJI::onboardSDK::Flight::TASK::TASK_TAKEOFF);
        response.result = true;
    }
    else if (request.task == 6) {
        //landing
        ////DJI_Pro_Status_Ctrl(6, 0);
	rosAdapter->flight->task(DJI::onboardSDK::Flight::TASK::TASK_LANDING);
        response.result = true;
    }
    else if (request.task == 1) {
        //gohome
        ////DJI_Pro_Status_Ctrl(1, 0);
	rosAdapter->flight->task(DJI::onboardSDK::Flight::TASK::TASK_GOHOME);
        response.result = true;
    }
    else
        response.result = false;
    return true;
}

bool DJISDKNode::gimbal_angle_control_callback(dji_sdk::GimbalAngleControl::Request& request, dji_sdk::GimbalAngleControl::Response& response) 
{
////
/*
    gimbal_custom_control_angle_t gimbal_angle = {0};
    gimbal_angle.yaw_angle = request.yaw;
    gimbal_angle.roll_angle = request.roll;
    gimbal_angle.pitch_angle = request.pitch;
    gimbal_angle.duration = request.duration;
    gimbal_angle.ctrl_byte.base = request.absolute_or_incremental;
    gimbal_angle.ctrl_byte.yaw_cmd_ignore = request.yaw_cmd_ignore;
    gimbal_angle.ctrl_byte.roll_cmd_ignore = request.roll_cmd_ignore;
    gimbal_angle.ctrl_byte.pitch_cmd_ignore = request.pitch_cmd_ignore;
*/
////

    ////DJI_Pro_Gimbal_Angle_Control(&gimbal_angle);
//TODO: [__CHRIS__a12]
    DJI::onboardSDK::GimbalAngleData gimbal_angle;
    gimbal_angle.yaw_angle = request.yaw;
    gimbal_angle.roll_angle = request.roll;
    gimbal_angle.pitch_angle = request.pitch;
    gimbal_angle.duration = request.duration;
    gimbal_angle.ctrl_byte = 0xF0;
    gimbal_angle.ctrl_byte &= request.absolute_or_incremental ? 0xFF : 0x7F;
    gimbal_angle.ctrl_byte &= request.yaw_cmd_ignore ? 0xFF : 0xBF;
    gimbal_angle.ctrl_byte &= request.roll_cmd_ignore ? 0xFF : 0xDF;
    gimbal_angle.ctrl_byte &= request.pitch_cmd_ignore ? 0xFF : 0xEF;
    rosAdapter->camera->setGimbalAngle(&gimbal_angle);

    response.result = true;
    return true;
}

bool DJISDKNode::gimbal_speed_control_callback(dji_sdk::GimbalSpeedControl::Request& request, dji_sdk::GimbalSpeedControl::Response& response)
{
////
/*
    gimbal_custom_speed_t gimbal_speed = {0};
    gimbal_speed.yaw_angle_rate = request.yaw_rate;
    gimbal_speed.roll_angle_rate = request.roll_rate;
    gimbal_speed.pitch_angle_rate = request.pitch_rate;
    gimbal_speed.ctrl_byte.ctrl_switch = 1; //enable
*/
////

    ////DJI_Pro_Gimbal_Speed_Control(&gimbal_speed);
//TODO: [__CHRIS__a13]
    DJI::onboardSDK::GimbalSpeedData gimbal_speed;
    gimbal_speed.yaw_angle_rate = request.yaw_rate;
    gimbal_speed.roll_angle_rate = request.roll_rate;
    gimbal_speed.pitch_angle_rate = request.pitch_rate;
    gimbal_speed.reserved = 0x80; //little endian. enable
    rosAdapter->camera->setGimbalSpeed(&gimbal_speed);


    response.result = true;
    return true;
}

bool DJISDKNode::global_position_control_callback(dji_sdk::GlobalPositionControl::Request& request, dji_sdk::GlobalPositionControl::Response& response)
{
    float dst_x;
    float dst_y;
    float dst_z = request.altitude;

    gps_convert_ned(dst_x, 
        dst_y,
        request.longitude, request.latitude,
        global_position.longitude,  global_position.latitude);

////
/*
    attitude_data_t user_ctrl_data;
    user_ctrl_data.ctrl_flag = 0x90;
    user_ctrl_data.roll_or_x = dst_x - local_position.x;
    user_ctrl_data.pitch_or_y = dst_y - local_position.y;
    user_ctrl_data.thr_z = dst_z;
    user_ctrl_data.yaw = request.yaw;
*/
////
    ////DJI_Pro_Attitude_Control(&user_ctrl_data);
//TODO: [__CHRIS__a14]
    DJI::onboardSDK::FlightData flight_ctrl_data;
    flight_ctrl_data.ctrl_flag = 0x90;
    flight_ctrl_data.roll_or_x = dst_x - local_position.x;
    flight_ctrl_data.pitch_or_y = dst_y - local_position.y;
    flight_ctrl_data.thr_z = dst_z;
    flight_ctrl_data.yaw = request.yaw;
    rosAdapter->flight->setFlight(&flight_ctrl_data);


    response.result = true;
    return true;
}

bool DJISDKNode::local_position_control_callback(dji_sdk::LocalPositionControl::Request& request, dji_sdk::LocalPositionControl::Response& response)
{
    float dst_x = request.x;
    float dst_y = request.y;
    float dst_z = request.z;

////
/*
    attitude_data_t user_ctrl_data;
    user_ctrl_data.ctrl_flag = 0x90;
    user_ctrl_data.roll_or_x = dst_x - local_position.x;
    user_ctrl_data.pitch_or_y = dst_y - local_position.y;
    user_ctrl_data.thr_z = dst_z;
    user_ctrl_data.yaw = request.yaw;
*/
////
    ////DJI_Pro_Attitude_Control(&user_ctrl_data);
//TODO: [__CHRIS__a15]
    DJI::onboardSDK::FlightData flight_ctrl_data;
    flight_ctrl_data.ctrl_flag = 0x90;
    flight_ctrl_data.roll_or_x = dst_x - local_position.x;
    flight_ctrl_data.pitch_or_y = dst_y - local_position.y;
    flight_ctrl_data.thr_z = dst_z;
    flight_ctrl_data.yaw = request.yaw;
    rosAdapter->flight->setFlight(&flight_ctrl_data);


    response.result = true;
    return true;
}

bool DJISDKNode::sdk_permission_control_callback(dji_sdk::SDKPermissionControl::Request& request, dji_sdk::SDKPermissionControl::Response& response)
{
    if (request.control_enable == 1) {
        printf("Request Control");
        ////DJI_Pro_Control_Management(1, NULL);
//TODO: [__CHRIS__a16]
	rosAdapter->coreAPI->setControl(1);
        response.result = true;
    }
    else if (request.control_enable == 0) {
        printf("Release Control");
        ////DJI_Pro_Control_Management(0, NULL);
	rosAdapter->coreAPI->setControl(0);
        response.result = true;
    }
    else
        response.result = false;

    return true;
}

bool DJISDKNode::velocity_control_callback(dji_sdk::VelocityControl::Request& request, dji_sdk::VelocityControl::Response& response)
{
////
/*
    attitude_data_t user_ctrl_data;
	if (request.frame)
		//world frame 
		user_ctrl_data.ctrl_flag = 0x40;
	else
		//body frame
		user_ctrl_data.ctrl_flag = 0x42;
    user_ctrl_data.roll_or_x = request.vx;
    user_ctrl_data.pitch_or_y = request.vy;
    user_ctrl_data.thr_z = request.vz;
    user_ctrl_data.yaw = request.yawAngle;
*/
////
    ////DJI_Pro_Attitude_Control(&user_ctrl_data);
//TODO: [__CHRIS__a17]
    DJI::onboardSDK::FlightData flight_ctrl_data;
    if (request.frame)
        //world frame 
        flight_ctrl_data.ctrl_flag = 0x40;
    else
        //body frame
        flight_ctrl_data.ctrl_flag = 0x42;
    flight_ctrl_data.roll_or_x = request.vx;
    flight_ctrl_data.pitch_or_y = request.vy;
    flight_ctrl_data.thr_z = request.vz;
    flight_ctrl_data.yaw = request.yawAngle;
    rosAdapter->flight->setFlight(&flight_ctrl_data);


    response.result = true;

    return true;
}

bool DJISDKNode::version_check_callback(dji_sdk::VersionCheck::Request& request, dji_sdk::VersionCheck::Response& response)
{
	////DJI_Pro_Get_API_Version(NULL);
//TODO: [__CHRIS__a18]
	rosAdapter->coreAPI->getVersion();
	response.result = true;
	return true;
}

bool DJISDKNode::virtual_rc_enable_control_callback(dji_sdk::VirtualRCEnableControl::Request& request, dji_sdk::VirtualRCEnableControl::Response& response)
{
////
/*
	virtual_rc_manager_t virtual_rc_manager;
	virtual_rc_manager.enable = request.enable;
	virtual_rc_manager.if_back_to_real = request.if_back_to_real;
*/
////
	////DJI_Pro_Virtual_RC_Manage(&virtual_rc_manager);
//TODO: [__CHRIS__a19]
	DJI::onboardSDK::VirtualRCSetting vrc_setting;
        vrc_setting.enable = request.enable;
        vrc_setting.cutoff = request.if_back_to_real;
        rosAdapter->virtualRC->setControl((bool)request.enable, (DJI::onboardSDK::VirtualRC::CutOff)request.if_back_to_real);


	response.result = true;
	return true;
}

bool DJISDKNode::virtual_rc_data_control_callback(dji_sdk::VirtualRCDataControl::Request& request, dji_sdk::VirtualRCDataControl::Response& response)
{
////
/*
	virtual_rc_data_t virtual_rc_data;
	std::copy(request.channel_data.begin(), request.channel_data.end(), virtual_rc_data.channel_data);
*/
////
	////DJI_Pro_Virtual_RC_Send_Value(&virtual_rc_data);
//TODO: [__CHRIS__a20]
	DJI::onboardSDK::VirtualRCData vrc_data;
	vrc_data.roll = request.channel_data[0];
	vrc_data.pitch = request.channel_data[1];
	vrc_data.throttle = request.channel_data[2];
	vrc_data.yaw = request.channel_data[3];
	vrc_data.gear = request.channel_data[4];
	vrc_data.reserved = request.channel_data[5];
	vrc_data.mode = request.channel_data[6];
	vrc_data.Channel_07 = request.channel_data[7];
	vrc_data.Channel_08 = request.channel_data[8];
	vrc_data.Channel_09 = request.channel_data[9];
	vrc_data.Channel_10 = request.channel_data[10];
	vrc_data.Channel_11 = request.channel_data[11];
	vrc_data.Channel_12 = request.channel_data[12];
	vrc_data.Channel_13 = request.channel_data[13];
	vrc_data.Channel_14 = request.channel_data[14];
	vrc_data.Channel_15 = request.channel_data[15];
	rosAdapter->virtualRC->sendData(vrc_data);


	response.result = true;
	return true;
}

bool DJISDKNode::drone_arm_control_callback(dji_sdk::DroneArmControl::Request& request, dji_sdk::DroneArmControl::Response& response)
{
	uint8_t arm = request.arm;
	////DJI_Pro_Arm_Control(arm);
//TODO: [__CHRIS__a21]
	rosAdapter->flight->setArm((bool)arm);

	response.result = true;
	return true;
}

bool DJISDKNode::sync_flag_control_callback(dji_sdk::SyncFlagControl::Request& request, dji_sdk::SyncFlagControl::Response& response)
{
	uint32_t frequency = request.frequency;
	////DJI_Pro_Send_Sync_Flag(frequency);
//TODO: [__CHRIS__a22]
	rosAdapter->coreAPI->setSyncFreq(frequency);

	response.result = true;
	return true;
}

bool DJISDKNode::message_frequency_control_callback(dji_sdk::MessageFrequencyControl::Request& request, dji_sdk::MessageFrequencyControl::Response& response)
{
////
/*
	sdk_msgs_frequency_data_t message_frequency;
	std::copy(request.frequency.begin(), request.frequency.end(), message_frequency.std_freq);
*/
////
	////DJI_Pro_Set_Msgs_Frequency(&message_frequency);
//TODO: [__CHRIS__a23]
	uint8_t message_frequency[16];
	std::copy(request.frequency.begin(), request.frequency.end(), message_frequency);
	rosAdapter->coreAPI->setBroadcastFreq(message_frequency);

	response.result = true;
	return true;
}
