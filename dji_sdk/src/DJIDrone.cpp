#include "dji_sdk.h"

class DJIDrone
{
	std::string ns;
	ros::NodeHandle nh;

	actionlib::SimpleActionClient<dji_sdk::LocalPositionNavigationAction> local_navigation_action_client("dji_sdk/local_navigation_action", true);
	actionlib::SimpleActionClient<dji_sdk::GlobalPositionNavigationAction> gps_navigation_action_client("dji_sdk/gps_navigation_action", true);
	actionlib::SimpleActionClient<dji_sdk::WaypointNavigationAction> waypoint_navigation_action_client("dji_sdk/waypoint_navigation_action", true);

    ros::ServiceClient attitude_control_service = nh.serviceClient<dji_sdk::AttitudeControl>("dji_sdk/attitude_control");
    ros::ServiceClient camera_action_control_service = nh.serviceClient<dji_sdk::CameraActionControl>("dji_sdk/camera_action_control");
    ros::ServiceClient drone_task_control_service = nh.serviceClient<dji_sdk::DroneTaskControl>("dji_sdk/drone_task_control");
    ros::ServiceClient gimbal_angle_control_service = nh.serviceClient<dji_sdk::GimbalAngleControl>("dji_sdk/gimbal_angle_control");
    ros::ServiceClient gimbal_speed_control_service = nh.serviceClient<dji_sdk::GimbalSpeedControl>("dji_sdk/gimbal_speed_control");
    ros::ServiceClient global_position_control_service = nh.serviceClient<dji_sdk::GlobalPositionControl>("dji_sdk/global_position_control");
    ros::ServiceClient local_position_control_service = nh.serviceClient<dji_sdk::LocalPositionControl>("dji_sdk/local_position_control");
    ros::ServiceClient sdk_permission_control_service = nh.serviceClient<dji_sdk::SDKPermissionControl>("dji_sdk/sdk_permission_control");
    ros::ServiceClient velocity_control_service = nh.serviceClient<dji_sdk::VelocityControl>("dji_sdk/velocity_control");


	DJIDrone(std::string ns): ns(ns), nh(ns)
	{

	}

	bool takeoff()
	{
		dji_sdk::DroneTaskControl drone_task_control;
		drone_task_control.request.task = 4;
		return drone_task_control_service.call(drone_task_control) && drone_task_control.response.result;
	}

	bool land()
	{
		dji_sdk::DroneTaskControl drone_task_control;
		drone_task_control.request.task = 6;
		return drone_task_control_service.call(drone_task_control) && drone_task_control.response.result;
	}

	bool gohome()
	{
		dji_sdk::DroneTaskControl drone_task_control;
		drone_task_control.request.task = 1;
		return drone_task_control_service.call(drone_task_control) && drone_task_control.response.result;
	}

	bool take_picture()
	{
		dji_sdk::CameraActionControl camera_action_control;
		camera_action_control.request.action = 0;
		return camera_action_control_service.call(camera_action_control) && camera_action_control.response.result;
	}

	bool record_video()
	{
		dji_sdk::CameraActionControl camera_action_control;
		camera_action_control.request.action = 1;
		return camera_action_control_service.call(camera_action_control) && camera_action_control.response.result;
	}
	
	bool stop_video()
	{
		dji_sdk::CameraActionControl camera_action_control;
		camera_action_control.request.action = 2;
		return camera_action_control_service.call(camera_action_control) && camera_action_control.response.result;
	}

};