#include <dji_sdk/dji_sdk_node.h>

bool DJISDKNode::attitude_control_callback(dji_sdk::AttitudeControl::Request& request, dji_sdk::AttitudeControl::Response& response)
{
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


