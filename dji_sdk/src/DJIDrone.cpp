#include "dji_sdk.h"

class DJIDrone
{
	std::string ns;
	ros::NodeHandle nh;

	actionlib::SimpleActionClient<dji_sdk::DroneTaskAction> drone_task_action_client;
	actionlib::SimpleActionClient<dji_sdk::LocalPositionNavigationAction> local_position_navigation_action_client;
	actionlib::SimpleActionClient<dji_sdk::GlobalPositionNavigationAction> global_position_navigation_action_client;
	actionlib::SimpleActionClient<dji_sdk::WaypointNavigationAction> waypoint_navigation_action_client;

	ros::ServiceClient attitude_control_service;
    ros::ServiceClient camera_action_control_service;
    ros::ServiceClient drone_task_control_service;
    ros::ServiceClient gimbal_angle_control_service;
    ros::ServiceClient gimbal_speed_control_service;
    ros::ServiceClient global_position_control_service;
    ros::ServiceClient local_position_control_service;
    ros::ServiceClient sdk_permission_control_service;
    ros::ServiceClient velocity_control_service;

	DJIDrone(std::string ns): ns(ns), nh(ns)
	{
		drone_task_action_client = actionlib::SimpleActionClient<dji_sdk::DroneTaskAction>(nh, "dji_sdk/drone_task_action", true);
		local_position_navigation_action_client = actionlib::SimpleActionClient<dji_sdk::LocalPositionNavigationAction>(nh, "dji_sdk/local_position_navigation_action", true);
		global_position_navigation_action_client = actionlib::SimpleActionClient<dji_sdk::GlobalPositionNavigationAction>(nh, "dji_sdk/global_position_navigation_action", true);
		waypoint_navigation_action_client = actionlib::SimpleActionClient<dji_sdk::WaypointNavigationAction>(nh, "dji_sdk/waypoint_navigation_action", true);

	    attitude_control_service = nh.serviceClient<dji_sdk::AttitudeControl>("dji_sdk/attitude_control");
	    camera_action_control_service = nh.serviceClient<dji_sdk::CameraActionControl>("dji_sdk/camera_action_control");
	    drone_task_control_service = nh.serviceClient<dji_sdk::DroneTaskControl>("dji_sdk/drone_task_control");
	    gimbal_angle_control_service = nh.serviceClient<dji_sdk::GimbalAngleControl>("dji_sdk/gimbal_angle_control");
	    gimbal_speed_control_service = nh.serviceClient<dji_sdk::GimbalSpeedControl>("dji_sdk/gimbal_speed_control");
	    global_position_control_service = nh.serviceClient<dji_sdk::GlobalPositionControl>("dji_sdk/global_position_control");
	    local_position_control_service = nh.serviceClient<dji_sdk::LocalPositionControl>("dji_sdk/local_position_control");
	    sdk_permission_control_service = nh.serviceClient<dji_sdk::SDKPermissionControl>("dji_sdk/sdk_permission_control");
	    velocity_control_service = nh.serviceClient<dji_sdk::VelocityControl>("dji_sdk/velocity_control");
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

	bool gimbal_speed_control()
	{

	}

	bool gimbal_speed_control()
	{
		
	}

	bool gimbal_speed_control()
	{
		
	}

};