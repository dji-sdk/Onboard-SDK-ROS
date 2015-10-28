#include <dji_sdk/dji_sdk.h>
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/UInt8.h>
#include <actionlib/client/simple_action_client.h> 
#include <actionlib/client/terminal_state.h> 
#include <string>

class DJIDrone
{
private:
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

    ros::Subscriber acceleration_subscriber;
    ros::Subscriber attitude_quaternion_subscriber;
    ros::Subscriber compass_subscriber;
    ros::Subscriber flight_control_info_subscriber;
    ros::Subscriber flight_status_subscriber;
    ros::Subscriber gimbal_subscriber;
    ros::Subscriber global_position_subscriber;
    ros::Subscriber local_position_subscriber;
    ros::Subscriber power_status_subscriber;
    ros::Subscriber rc_channels_subscriber;
    ros::Subscriber velocity_subscriber;
    ros::Subscriber activation_subscriber;
    ros::Subscriber odometry_subscriber;
    ros::Subscriber sdk_permission_subscriber;

public:
    dji_sdk::Acceleration acceleration;
    dji_sdk::AttitudeQuaternion attitude_quaternion;
    dji_sdk::Compass compass;
    dji_sdk::FlightControlInfo flight_control_info;
    uint8_t flight_status;
    dji_sdk::Gimbal gimbal;
    dji_sdk::GlobalPosition global_position;
    dji_sdk::GlobalPosition global_position_ref;
    dji_sdk::LocalPosition local_position;
    dji_sdk::LocalPosition local_position_ref;
    dji_sdk::PowerStatus power_status;
    dji_sdk::RCChannels rc_channels;
    dji_sdk::Velocity velocity;
    nav_msgs::Odometry odometry;
    bool sdk_permission_opened = false;
    bool activation = false;
    bool localposbase_use_height = true;

private:
	void acceleration_subscriber_callback(dji_sdk::Acceleration acceleration) {
		this->acceleration = acceleration;
	}

	void attitude_quaternion_subscriber_callback(dji_sdk::AttitudeQuaternion attitude_quaternion) {
		this->attitude_quaternion = attitude_quaternion;
	}

	void compass_subscriber_callback(dji_sdk::Compass compass) {
		this->compass = compass;
	}

	void flight_control_info_subscriber_callback(dji_sdk::FlightControlInfo flight_control_info) {
		this->flight_control_info = flight_control_info;
	}

	void flight_status_subscriber_callback(std_msgs::UInt8 flight_status) {
		this->flight_status = flight_status.data;
	}

	void gimbal_subscriber_callback(dji_sdk::Gimbal gimbal) {
		this->gimbal = gimbal;
	}

	void global_position_subscriber_callback(dji_sdk::GlobalPosition global_position) {
		this->global_position = global_position;
	}

	void local_position_subscriber_callback(dji_sdk::LocalPosition local_position) {
		this->local_position = local_position;
	}

	void power_status_subscriber_callback(dji_sdk::PowerStatus power_status) {
		this->power_status = power_status;
	}

	void rc_channels_subscriber_callback(dji_sdk::RCChannels rc_channels) {
		this->rc_channels = rc_channels;
	}

	void velocity_subscriber_callback(dji_sdk::Velocity velocity) {
		this->velocity = velocity;
	}

	void activation_subscriber_callback(std_msgs::UInt8 activation) {
		this->activation = activation.data;
	}

	void odometry_subscriber_callback(nav_msgs::Odometry odometry) {
		this->odometry = odometry;
	}

	void sdk_permission_subscriber_callback(std_msgs::UInt8 sdk_permission) {
		this->sdk_permission_opened = sdk_permission.data;
	}

public:
	DJIDrone(std::string ns = ""): 
		ns(ns), 
		nh(ns), 
		drone_task_action_client(nh, "dji_sdk/drone_task_action", true),
		local_position_navigation_action_client(nh, "dji_sdk/local_position_navigation_action", true),
		global_position_navigation_action_client(nh, "dji_sdk/global_position_navigation_action", true),
		waypoint_navigation_action_client(nh, "dji_sdk/waypoint_navigation_action", true)
	{
	    attitude_control_service = nh.serviceClient<dji_sdk::AttitudeControl>("dji_sdk/attitude_control");
	    camera_action_control_service = nh.serviceClient<dji_sdk::CameraActionControl>("dji_sdk/camera_action_control");
	    drone_task_control_service = nh.serviceClient<dji_sdk::DroneTaskControl>("dji_sdk/drone_task_control");
	    gimbal_angle_control_service = nh.serviceClient<dji_sdk::GimbalAngleControl>("dji_sdk/gimbal_angle_control");
	    gimbal_speed_control_service = nh.serviceClient<dji_sdk::GimbalSpeedControl>("dji_sdk/gimbal_speed_control");
	    global_position_control_service = nh.serviceClient<dji_sdk::GlobalPositionControl>("dji_sdk/global_position_control");
	    local_position_control_service = nh.serviceClient<dji_sdk::LocalPositionControl>("dji_sdk/local_position_control");
	    sdk_permission_control_service = nh.serviceClient<dji_sdk::SDKPermissionControl>("dji_sdk/sdk_permission_control");
	    velocity_control_service = nh.serviceClient<dji_sdk::VelocityControl>("dji_sdk/velocity_control");

        acceleration_subscriber = nh.subscribe<dji_sdk::Acceleration>("dji_sdk/acceleration", 10, &DJIDrone::acceleration_subscriber_callback, this);
        attitude_quaternion_subscriber = nh.subscribe<dji_sdk::AttitudeQuaternion>("dji_sdk/attitude_quaternion", 10, &DJIDrone::attitude_quaternion_subscriber_callback, this);
        compass_subscriber = nh.subscribe<dji_sdk::Compass>("dji_sdk/compass", 10, &DJIDrone::compass_subscriber_callback, this);
        flight_control_info_subscriber = nh.subscribe<dji_sdk::FlightControlInfo>("dji_sdk/flight_control_info", 10, &DJIDrone::flight_control_info_subscriber_callback, this);
        flight_status_subscriber = nh.subscribe<std_msgs::UInt8>("dji_sdk/flight_status", 10, &DJIDrone::flight_status_subscriber_callback, this);
        gimbal_subscriber = nh.subscribe<dji_sdk::Gimbal>("dji_sdk/gimbal", 10, &DJIDrone::gimbal_subscriber_callback, this);
        global_position_subscriber = nh.subscribe<dji_sdk::GlobalPosition>("dji_sdk/global_position", 10, &DJIDrone::global_position_subscriber_callback, this);
        local_position_subscriber = nh.subscribe<dji_sdk::LocalPosition>("dji_sdk/local_position", 10, &DJIDrone::local_position_subscriber_callback, this);
        power_status_subscriber = nh.subscribe<dji_sdk::PowerStatus>("dji_sdk/power_status", 10, &DJIDrone::power_status_subscriber_callback, this);
        rc_channels_subscriber = nh.subscribe<dji_sdk::RCChannels>("dji_sdk/rc_channels", 10, &DJIDrone::rc_channels_subscriber_callback, this);
        velocity_subscriber = nh.subscribe<dji_sdk::Velocity>("dji_sdk/velocity", 10, &DJIDrone::velocity_subscriber_callback, this);
        activation_subscriber = nh.subscribe<std_msgs::UInt8>("dji_sdk/activation", 10, &DJIDrone::activation_subscriber_callback, this);
        odometry_subscriber = nh.subscribe<nav_msgs::Odometry>("dji_sdk/odometry",10, &DJIDrone::odometry_subscriber_callback, this);
        sdk_permission_subscriber = nh.subscribe<std_msgs::UInt8>("dji_sdk/sdk_permission", 10, &DJIDrone::sdk_permission_subscriber_callback, this);
	}

	bool takeoff()
	{
		dji_sdk::DroneTaskControl drone_task_control;
		drone_task_control.request.task = 4;
		return drone_task_control_service.call(drone_task_control) && drone_task_control.response.result;
	}

	bool landing()
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
		camera_action_control.request.camera_action = 0;
		return camera_action_control_service.call(camera_action_control) && camera_action_control.response.result;
	}

	bool record_video()
	{
		dji_sdk::CameraActionControl camera_action_control;
		camera_action_control.request.camera_action = 1;
		return camera_action_control_service.call(camera_action_control) && camera_action_control.response.result;
	}
	
	bool stop_video()
	{
		dji_sdk::CameraActionControl camera_action_control;
		camera_action_control.request.camera_action = 2;
		return camera_action_control_service.call(camera_action_control) && camera_action_control.response.result;
	}

	bool gimbal_speed_control(int roll_rate, int pitch_rate, int yaw_rate)
	{
		dji_sdk::GimbalSpeedControl gimbal_speed_control;
		gimbal_speed_control.request.roll_rate = roll_rate;
		gimbal_speed_control.request.pitch_rate = pitch_rate;
		gimbal_speed_control.request.yaw_rate = yaw_rate;
		return gimbal_speed_control_service.call(gimbal_speed_control) && gimbal_speed_control.response.result;

	}

	bool gimbal_angle_control(unsigned char ctrl_flag, int roll, int pitch, int yaw, int duration)
	{
		dji_sdk::GimbalAngleControl gimbal_angle_control;
		gimbal_angle_control.request.flag = ctrl_flag;
		gimbal_angle_control.request.roll = roll;
		gimbal_angle_control.request.pitch = pitch;
		gimbal_angle_control.request.yaw = yaw;
		gimbal_angle_control.request.duration= duration;
		return gimbal_angle_control_service.call(gimbal_angle_control) && gimbal_angle_control.response.result;
	}
	
	bool request_sdk_permission_control()
	{
		return sdk_permission_control(1);
	}

	bool release_sdk_permission_control()
	{
		return sdk_permission_control(0);
	}

	bool sdk_permission_control(unsigned char request)
	{
		dji_sdk::SDKPermissionControl sdk_permission_control;
		sdk_permission_control.request.control_enable = request;
		
		return sdk_permission_control_service.call(sdk_permission_control) && sdk_permission_control.response.result;

	}

	bool attitude_control(unsigned char ctrl_flag, float x, float y, float z, float yaw)
	{
		dji_sdk::AttitudeControl attitude_control;
		attitude_control.request.flag = ctrl_flag;
		attitude_control.request.x = x;
		attitude_control.request.y = y;
		attitude_control.request.z = z;
		attitude_control.request.yaw = yaw;

		return attitude_control_service.call(attitude_control) && attitude_control.response.result;
	}

	bool velocity_control(int frame, float x, float y, float z, float yaw)
	{
		dji_sdk::VelocityControl velocity_control;
		velocity_control.request.frame = frame;
		velocity_control.request.vx = x;
		velocity_control.request.vy = y;
		velocity_control.request.vz = z;
		velocity_control.request.yawAngle = yaw;
	
		return velocity_control_service.call(velocity_control) && velocity_control.response.result;
	}

	bool local_position_control(float x, float y, float z, float yaw)
	{
		dji_sdk::LocalPositionControl local_position_control;
		local_position_control.request.x = x;
		local_position_control.request.y = y;
		local_position_control.request.z = z;
		local_position_control.request.yaw = yaw;
		
		return local_position_control_service.call(local_position_control) && local_position_control.response.result;

	}

	bool global_position_control(double latitude, double longitude, float altitude, float yaw)
	{
		dji_sdk::GlobalPositionControl global_position_control;
		global_position_control.request.latitude = latitude;
		global_position_control.request.longitude = longitude;
		global_position_control.request.altitude = altitude;
		global_position_control.request.yaw = yaw;

		return global_position_control_service.call(global_position_control) && global_position_control.response.result;
	}


	bool local_position_navigation(float x, float y, float z)
	{
		local_position_navigation_action_client.waitForServer();

		dji_sdk::LocalPositionNavigationGoal local_position_navigation_goal;
		local_position_navigation_goal.x = x;
		local_position_navigation_goal.y = y;
		local_position_navigation_goal.z = z;
		local_position_navigation_action_client.sendGoal(local_position_navigation_goal);

		return true;//pls check Feedback topic for progress detail
	}

	bool global_position_navigation(double latitude, double longitude, float altitude)
	{
		global_position_navigation_action_client.waitForServer();

		dji_sdk::GlobalPositionNavigationGoal global_position_navigation_goal;
		global_position_navigation_goal.latitude = latitude;
		global_position_navigation_goal.longitude = longitude;
		global_position_navigation_goal.altitude = altitude;
		global_position_navigation_action_client.sendGoal(global_position_navigation_goal);

		bool finished_before_timeout = global_position_navigation_action_client.waitForResult(ros::Duration(300.0));
		if (finished_before_timeout)
		{
			actionlib::SimpleClientGoalState state = global_position_navigation_action_client.getState();
			ROS_INFO("Action finished: %s",state.toString().c_str());
		}
		else {
			ROS_INFO("Action did not finish before the time out.");
		}

		return finished_before_timeout;//pls check Feedback topic for progress detail

	}

	bool waypoint_navigation(dji_sdk::WaypointList waypoint_data)
	{
		waypoint_navigation_action_client.waitForServer();

		dji_sdk::WaypointNavigationGoal waypoint_navigation_goal;
		waypoint_navigation_goal.waypoint_list = waypoint_data;
		waypoint_navigation_action_client.sendGoal(waypoint_navigation_goal);

		bool finished_before_timeout = waypoint_navigation_action_client.waitForResult(ros::Duration(300.0));
		if (finished_before_timeout)
		{
			actionlib::SimpleClientGoalState state = waypoint_navigation_action_client.getState();
			ROS_INFO("Action finished: %s",state.toString().c_str());
		}
		else {
			ROS_INFO("Action did not finish before the time out.");
		}

		return finished_before_timeout;//pls check Feedback topic for progress detail

	}

};

