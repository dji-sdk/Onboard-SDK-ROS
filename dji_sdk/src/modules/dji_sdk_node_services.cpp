#include "dji_sdk_node.h"

void DJI_Gimbal_SpeedCtrl(signed short yaw_angle_rate,
            signed short roll_angle_rate,
            signed short pitch_angle_rate)
{
    gimbal_custom_speed_t gimbal_speed = {0};
    gimbal_speed.yaw_angle_rate = yaw_angle_rate;
    gimbal_speed.roll_angle_rate = roll_angle_rate;
    gimbal_speed.pitch_angle_rate = pitch_angle_rate;
    gimbal_speed.ctrl_byte.ctrl_switch = 1;
    DJI_Pro_App_Send_Data(0,
            1, MY_CTRL_CMD_SET,
            API_GIMBAL_CTRL_SPEED_REQUEST,
            (uint8_t*)&gimbal_speed,
            sizeof(gimbal_speed),
            NULL,
            0,
            0
            );
}

void DJI_Gimbal_AngleCtrl(int16_t yaw_angle,
        int16_t roll_angle,
        int16_t pitch_angle,
        uint8_t baseflag,
        uint8_t duration)
{
    gimbal_custom_control_angle_t gimbal_angle = {0};
    gimbal_angle.yaw_angle = yaw_angle;
    gimbal_angle.roll_angle = roll_angle;
    gimbal_angle.pitch_angle = pitch_angle;
    gimbal_angle.ctrl_byte.base = baseflag;
    gimbal_angle.ctrl_byte.yaw_cmd_ignore = 0;
    gimbal_angle.ctrl_byte.roll_cmd_ignore = 0;
    gimbal_angle.ctrl_byte.pitch_cmd_ignore = 0;
    gimbal_angle.duration = duration;
    DJI_Pro_App_Send_Data(0,
            0,
            MY_CTRL_CMD_SET,
            API_GIMBAL_CTRL_ANGLE_REQUEST,
            (uint8_t*)&gimbal_angle,
            sizeof(gimbal_angle),
            NULL,
            0,
            0
            );
}

bool DJISDKNode::attitude_control_callback(dji_sdk::AttitudeControl::Request& request, dji_sdk::AttitudeControl::Response& response)
{
    attitude_data_t user_ctrl_data;

    user_ctrl_data.ctrl_flag = request.flag;
    user_ctrl_data.roll_or_x = request.x;
    user_ctrl_data.pitch_or_y = request.y;
    user_ctrl_data.thr_z = request.z;
    user_ctrl_data.yaw = request.yaw;

    DJI_Pro_Attitude_Control(&user_ctrl_data);

    response.result = true;
    return true;
}

bool DJISDKNode::camera_action_control_callback(dji_sdk::CameraActionControl::Request& request, dji_sdk::CameraActionControl::Response& response)
{
    if (request.camera_action == 0) {
        DJI_Pro_Camera_Control(API_CAMERA_SHOT);
        response.result = true;
    }
    else if (request.camera_action == 1) {
        DJI_Pro_Camera_Control(API_CAMERA_VIDEO_START);
        response.result = true;
    }
    else if (request.camera_action == 2) {
        DJI_Pro_Camera_Control(API_CAMERA_VIDEO_STOP);
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
        DJI_Pro_Status_Ctrl(4, 0);
        response.result = true;
    }
    else if (request.task == 6) {
        //landing
        DJI_Pro_Status_Ctrl(6, 0);
        response.result = true;
    }
    else if (request.task == 1) {
        //gohome
        DJI_Pro_Status_Ctrl(1, 0);
        response.result = true;
    }
    else
        response.result = false;
    return true;
}

bool DJISDKNode::gimbal_angle_control_callback(dji_sdk::GimbalAngleControl::Request& request, dji_sdk::GimbalAngleControl::Response& response) 
{
    uint8_t flag = request.flag;
    int16_t x = request.x;
    int16_t y = request.y;
    int16_t yaw = request.yaw;
    uint8_t duration = request.duration;

    DJI_Gimbal_AngleCtrl(yaw, x, y, flag, duration);
    response.result = true;
    return true;
}

bool DJISDKNode::gimbal_speed_control_callback(dji_sdk::GimbalSpeedControl::Request& request, dji_sdk::GimbalSpeedControl::Response& response)
{
    signed short yaw_rate = request.yaw_rate;
    signed short x_rate = request.x_rate;
    signed short y_rate = request.y_rate;

    DJI_Gimbal_SpeedCtrl(yaw_rate, x_rate, y_rate);

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

    attitude_data_t user_ctrl_data;
    user_ctrl_data.ctrl_flag = 0x90;
    user_ctrl_data.roll_or_x = dst_x - local_position.x;
    user_ctrl_data.pitch_or_y = dst_y - local_position.y;
    user_ctrl_data.thr_z = dst_z;
    user_ctrl_data.yaw = request.yaw;
    DJI_Pro_Attitude_Control(&user_ctrl_data);

    response.result = true;
    return true;
}

bool DJISDKNode::local_position_control_callback(dji_sdk::LocalPositionControl::Request& request, dji_sdk::LocalPositionControl::Response& response)
{
    float dst_x = request.x;
    float dst_y = request.y;
    float dst_z = request.z;

    attitude_data_t user_ctrl_data;
    user_ctrl_data.ctrl_flag = 0x90;
    user_ctrl_data.roll_or_x = dst_x - local_position.x;
    user_ctrl_data.pitch_or_y = dst_y - local_position.y;
    user_ctrl_data.thr_z = dst_z;
    user_ctrl_data.yaw = request.yaw;
    DJI_Pro_Attitude_Control(&user_ctrl_data);

    response.result = true;
    return true;
}

bool DJISDKNode::sdk_permission_control_callback(dji_sdk::SDKPermissionControl::Request& request, dji_sdk::SDKPermissionControl::Response& response)
{
    if (request.control_enable == 1) {
        printf("Request Control");
        DJI_Pro_Control_Management(1, NULL);
        response.result = true;
    }
    else if (request.control_enable == 0) {
        printf("Release Control");
        DJI_Pro_Control_Management(0, NULL);
        response.result = true;
    }
    else
        response.result = false;

    return true;
}

bool DJISDKNode::velocity_control_callback(dji_sdk::VelocityControl::Request& request, dji_sdk::VelocityControl::Response& response)
{
    attitude_data_t user_ctrl_data;
    user_ctrl_data.ctrl_flag = 0x40;
    user_ctrl_data.roll_or_x = request.vx;
    user_ctrl_data.pitch_or_y = request.vy;
    user_ctrl_data.thr_z = request.vz;
    user_ctrl_data.yaw = request.yaw;
    DJI_Pro_Attitude_Control(&user_ctrl_data);

    response.result = true;

    return true;
}


