/** @file dji_sdk_node_services.cpp
 *  @version 3.1.8
 *  @date July 29th, 2016
 *
 *  @brief
 *  All the purchase callbacks are implemented here.
 *
 *  @copyright 2016 DJI. All rights reserved.
 *
 */

#include <dji_sdk/dji_sdk_node.h>


bool DJISDKNode::activation_callback(dji_sdk::Activation::Request& request, dji_sdk::Activation::Response& response)
{
	rosAdapter->coreAPI->activate(&user_act_data);
	response.result = true;
	return true;
}


bool DJISDKNode::attitude_control_callback(dji_sdk::AttitudeControl::Request& request, dji_sdk::AttitudeControl::Response& response)
{
    rosAdapter->flight->setMovementControl(request.flag, request.x, request.y, request.z, request.yaw);
    response.result = true;
    return true;
}


bool DJISDKNode::camera_action_control_callback(dji_sdk::CameraActionControl::Request& request, dji_sdk::CameraActionControl::Response& response)
{
    if (request.camera_action == 0) {
        rosAdapter->camera->setCamera(DJI::onboardSDK::Camera::CAMERA_CODE::CODE_CAMERA_SHOT);
        response.result = true;
    }
    else if (request.camera_action == 1) {
        rosAdapter->camera->setCamera(DJI::onboardSDK::Camera::CAMERA_CODE::CODE_CAMERA_VIDEO_START);
        response.result = true;
    }
    else if (request.camera_action == 2) {
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
        rosAdapter->flight->task(DJI::onboardSDK::Flight::TASK::TASK_TAKEOFF);
        response.result = true;
    }
    else if (request.task == 6) {
        //landing
        rosAdapter->flight->task(DJI::onboardSDK::Flight::TASK::TASK_LANDING);
        response.result = true;
    }
    else if (request.task == 1) {
        //gohome
        rosAdapter->flight->task(DJI::onboardSDK::Flight::TASK::TASK_GOHOME);
        response.result = true;
    }
    else
        response.result = false;
    return true;
}


bool DJISDKNode::gimbal_angle_control_callback(dji_sdk::GimbalAngleControl::Request& request, dji_sdk::GimbalAngleControl::Response& response) 
{
    DJI::onboardSDK::GimbalAngleData gimbal_angle;
    gimbal_angle.yaw = request.yaw;
    gimbal_angle.roll = request.roll;
    gimbal_angle.pitch = request.pitch;
    gimbal_angle.duration = request.duration;
    gimbal_angle.mode = 0;
    gimbal_angle.mode |= request.absolute_or_incremental;
    gimbal_angle.mode |= request.yaw_cmd_ignore << 1;
    gimbal_angle.mode |= request.roll_cmd_ignore << 2;
    gimbal_angle.mode |= request.pitch_cmd_ignore << 3;

    rosAdapter->camera->setGimbalAngle(&gimbal_angle);

    response.result = true;
    return true;
}


bool DJISDKNode::gimbal_speed_control_callback(dji_sdk::GimbalSpeedControl::Request& request, dji_sdk::GimbalSpeedControl::Response& response)
{
    DJI::onboardSDK::GimbalSpeedData gimbal_speed;
    gimbal_speed.yaw = request.yaw_rate;
    gimbal_speed.roll = request.roll_rate;
    gimbal_speed.pitch = request.pitch_rate;
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

    if(global_position_ref_seted == 0)
    {
        printf("Cannot run global position navigation because home position haven't set yet!");
        response.result = false;
        return false;
    }

    gps_convert_ned(dst_x, 
            dst_y,
            request.longitude, request.latitude,
            global_position.longitude,  global_position.latitude);

    uint8_t flag = 0x90;
    rosAdapter->flight->setMovementControl(flag, dst_x - local_position.x, dst_y - local_position.y, dst_z, request.yaw);
    response.result = true;
    return true;
}


bool DJISDKNode::local_position_control_callback(dji_sdk::LocalPositionControl::Request& request, dji_sdk::LocalPositionControl::Response& response)
{
    float dst_x = request.x;
    float dst_y = request.y;
    float dst_z = request.z;

    if(global_position_ref_seted == 0)
    {
        printf("Cannot run local position navigation because home position haven't set yet!");
        response.result = false;
        return false;
    }

    DJI::onboardSDK::FlightData flight_ctrl_data;
    uint8_t flag = 0x90;
    rosAdapter->flight->setMovementControl(flag, dst_x - local_position.x, dst_y - local_position.y, dst_z, request.yaw);
    response.result = true;
    return true;
}


bool DJISDKNode::sdk_permission_control_callback(dji_sdk::SDKPermissionControl::Request& request, dji_sdk::SDKPermissionControl::Response& response)
{
    if (request.control_enable == 1) {
        printf("Request Control");
        rosAdapter->coreAPI->setControl(1);
        response.result = true;
    }
    else if (request.control_enable == 0) {
        printf("Release Control");
        rosAdapter->coreAPI->setControl(0);
        response.result = true;
    }
    else
        response.result = false;

    return true;
}


bool DJISDKNode::velocity_control_callback(dji_sdk::VelocityControl::Request& request, dji_sdk::VelocityControl::Response& response)
{
    DJI::onboardSDK::FlightData flight_ctrl_data;
    if (request.frame)
        //world frame 
        flight_ctrl_data.flag = 0x49;
    else
        //body frame
        flight_ctrl_data.flag = 0x4B;

    flight_ctrl_data.x = request.vx;
    flight_ctrl_data.y = request.vy;
    flight_ctrl_data.z = request.vz;
    flight_ctrl_data.yaw = request.yawRate;

    rosAdapter->flight->setFlight(&flight_ctrl_data);

    response.result = true;
    return true;
}


bool DJISDKNode::version_check_callback(dji_sdk::VersionCheck::Request& request, dji_sdk::VersionCheck::Response& response)
{
    int sdkVer;
    sdkVer = rosAdapter->coreAPI->getFwVersion();
    std::cout << std::hex << sdkVer << '\n';
	response.result = true;
	return true;
}


bool DJISDKNode::virtual_rc_enable_control_callback(dji_sdk::VirtualRCEnableControl::Request& request, dji_sdk::VirtualRCEnableControl::Response& response)
{
    DJI::onboardSDK::VirtualRCSetting vrc_setting;
    vrc_setting.enable = request.enable;
    vrc_setting.cutoff = request.if_back_to_real;
    rosAdapter->virtualRC->setControl((bool)request.enable, (DJI::onboardSDK::VirtualRC::CutOff)request.if_back_to_real);

    response.result = true;
    return true;
}


bool DJISDKNode::virtual_rc_data_control_callback(dji_sdk::VirtualRCDataControl::Request& request, dji_sdk::VirtualRCDataControl::Response& response)
{
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
	rosAdapter->flight->setArm((bool)arm);

	response.result = true;
	return true;
}


bool DJISDKNode::sync_flag_control_callback(dji_sdk::SyncFlagControl::Request& request, dji_sdk::SyncFlagControl::Response& response)
{
	uint32_t frequency = request.frequency;
	rosAdapter->coreAPI->setSyncFreq(frequency);

	response.result = true;
	return true;
}


bool DJISDKNode::message_frequency_control_callback(dji_sdk::MessageFrequencyControl::Request& request, dji_sdk::MessageFrequencyControl::Response& response)
{
	uint8_t message_frequency[16];
	std::copy(request.frequency.begin(), request.frequency.end(), message_frequency);
	rosAdapter->coreAPI->setBroadcastFreq(message_frequency);

	response.result = true;
	return true;
}

bool DJISDKNode::send_data_to_remote_device_callback(dji_sdk::SendDataToRemoteDevice::Request& request, dji_sdk::SendDataToRemoteDevice::Response& response)
{
	memcpy(transparent_transmission_data, &request.data[0], request.data.size());
	rosAdapter->sendToMobile(transparent_transmission_data, request.data.size());
	response.result = true;
	return true;
}
