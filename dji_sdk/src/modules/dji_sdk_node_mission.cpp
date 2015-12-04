#include <dji_sdk/dji_sdk_mission.h>

cmd_mission_hotpoint_setting_t new_hotpoint = {0};
cmd_mission_follow_setting_t new_follow = {0};

void DJISDKMission::mission_state_callback()
{
	cmd_mission_common_data_t mission_state_data;
	dji_sdk::MissionPushInfo mission_state;
	DJI_Pro_Get_Mission_State_Data(&mission_state_data);
	mission_state.type = mission_state_data.type;
	mission_state.data_1 = mission_state_data.data_1;
	mission_state.data_2 = mission_state_data.data_2;
	mission_state.data_3 = mission_state_data.data_3;
	mission_state.data_4 = mission_state_data.data_4;
	mission_state.data_5 = mission_state_data.data_5;
	mission_state_publisher.publish(mission_state);
}

void DJISDKMission::mission_event_callback()
{
	cmd_mission_common_data_t mission_event_data;
	dji_sdk::MissionPushInfo mission_event;
	DJI_Pro_Get_Mission_Event_Data(&mission_event_data);
	mission_event.data_1 = mission_event_data.data_1;
	mission_event.data_2 = mission_event_data.data_2;
	mission_event.data_3 = mission_event_data.data_3;
	mission_event.data_4 = mission_event_data.data_4;
	mission_event.data_5 = mission_event_data.data_5;
	mission_event_publisher.publish(mission_event);

}

bool DJISDKMission::mission_start_callback(dji_sdk::MissionStart::Request& request, dji_sdk::MissionStart::Response& response)
{
	switch(current_type)
	{	
		case MissionType::WAYPOINT:
			//send start command 
			DJI_Pro_Mission_Waypoint_Start(0);
			break;
		
		case MissionType::HOTPOINT:
			//upload hp task info

			new_hotpoint.latitude = hotpoint_task.latitude;
			new_hotpoint.longitude = hotpoint_task.longitude;
			new_hotpoint.altitude = hotpoint_task.altitude;
			new_hotpoint.radius = hotpoint_task.radius;
			new_hotpoint.angular_rate = hotpoint_task.angular_speed;
			new_hotpoint.is_clockwise = hotpoint_task.is_clockwise;
			new_hotpoint.start_point = hotpoint_task.start_point;
			new_hotpoint.yaw_mode = hotpoint_task.yaw_mode;
		
			DJI_Pro_Mission_Hotpoint_Start(&new_hotpoint);
			break;

		case MissionType::FOLLOWME:
			//upload fm task info

			new_follow.mode = followme_task.mode;
			new_follow.yaw_mode = followme_task.yaw_mode;
			new_follow.initial_latitude = followme_task.initial_latitude;
			new_follow.initial_longitude = followme_task.initial_longitude;
			new_follow.initial_altitude = followme_task.initial_altitude;
			new_follow.sensitivity = followme_task.sensitivity;

			DJI_Pro_Mission_Followme_Start(&new_follow);
			break;

		default:
			//empty
			return false;

	}
	return true;

}
bool DJISDKMission::mission_pause_callback(dji_sdk::MissionPause::Request& request, dji_sdk::MissionPause::Response& response)
{

	switch(current_type)
	{	
		//different cmd id
		case MissionType::WAYPOINT:
			DJI_Pro_Mission_Waypoint_Pause(request.pause);
			break;
		
		case MissionType::HOTPOINT:
			DJI_Pro_Mission_Hotpoint_Pause(request.pause);
			break;

		case MissionType::FOLLOWME:
			DJI_Pro_Mission_Followme_Pause(request.pause);
			break;
		default:
			return false;

	}

	return true;

}
bool DJISDKMission::mission_cancel_callback(dji_sdk::MissionCancel::Request& request, dji_sdk::MissionCancel::Response& response)
{

	switch(current_type)
	{	
		//different cmd id
		case MissionType::WAYPOINT:
			DJI_Pro_Mission_Waypoint_Start(1);
			break;
		
		case MissionType::HOTPOINT:
			DJI_Pro_Mission_Hotpoint_Stop();
			break;

		case MissionType::FOLLOWME:
			DJI_Pro_Mission_Followme_Stop();
			break;

		default:
			return false;

	}
	return true;
}

bool DJISDKMission::mission_download_callback(dji_sdk::MissionDownload::Request& request, dji_sdk::MissionDownload::Response& response)
{

	switch(current_type)
	{	
		//different callback
		case MissionType::WAYPOINT:
			DJI_Pro_Mission_Waypoint_Download_Task();
			//TODO download (for each waypoint in task)
			break;
		
		case MissionType::HOTPOINT:
			DJI_Pro_Mission_Hotpoint_Download();
			break;

		case MissionType::FOLLOWME:
			//No download for Followme
			break;

		default:
			return false;
	}

	return true;

}
bool DJISDKMission::mission_wp_upload_callback(dji_sdk::MissionWpUpload::Request& request, dji_sdk::MissionWpUpload::Response& response)
{

	waypoint_task = request.waypoint_task;

	cmd_mission_wp_task_info_comm_t new_task = {0};
	cmd_mission_wp_waypoint_info_comm_t new_waypoint = {0};
	cmd_mission_wp_action_comm_t new_action = {0};
	cmd_mission_wp_waypoint_upload_comm_t new_upload = {0};
	
	new_task.length = waypoint_task.mission_waypoint.size();
	new_task.vel_cmd_range = waypoint_task.velocity_range;
	new_task.idle_vel = waypoint_task.idle_velocity;
	new_task.action_on_finish = waypoint_task.action_on_finish;
	new_task.mission_exec_times = waypoint_task.mission_exec_times;
	new_task.yaw_mode = waypoint_task.yaw_mode;
	new_task.trace_mode = waypoint_task.trace_mode;
	new_task.action_on_rc_lost = waypoint_task.action_on_rc_lost;
	new_task.gimbal_pitch_mode = waypoint_task.gimbal_pitch_mode;
	
	DJI_Pro_Mission_Waypoint_Upload_Task(&new_task);
	printf("uploaded the task with %d waypoints\n", new_task.length);

	sleep(2);

	int i = 0;
	for (auto waypoint:waypoint_task.mission_waypoint)
	{
		new_waypoint.latitude = waypoint.latitude*C_PI/180;
		new_waypoint.longitude = waypoint.longitude*C_PI/180;
		new_waypoint.altitude = waypoint.altitude;
		new_waypoint.damping_dis = waypoint.damping_distance;
		new_waypoint.tgt_yaw = waypoint.target_yaw;
		new_waypoint.tgt_gimbal_pitch = waypoint.target_gimbal_pitch;
		new_waypoint.turn_mode = waypoint.turn_mode;
		new_waypoint.has_action = waypoint.has_action;
		new_waypoint.action_time_limit = waypoint.action_time_limit;

		new_action.action_num = 15;
		new_action.action_rpt = waypoint.waypoint_action.action_repeat;
		std::copy(waypoint.waypoint_action.command_list.begin(), waypoint.waypoint_action.command_list.end(), new_action.command_list);
		std::copy(waypoint.waypoint_action.command_parameter.begin(),waypoint.waypoint_action.command_parameter.end(), new_action.command_param);

		new_waypoint.action = new_action;

		new_upload.waypoint_index = i;
		new_upload.waypoint = new_waypoint;

		DJI_Pro_Mission_Waypoint_Upload_Waypoint(&new_upload);
		printf("uploaded the %dth waypoint\n", new_upload.waypoint_index);
		i+=1;
		sleep(2);
	}
	
	current_type = MissionType::WAYPOINT;
	printf("current_type -> WP\n");
	return true;

}
bool DJISDKMission::mission_wp_get_speed_callback(dji_sdk::MissionWpGetSpeed::Request& request, dji_sdk::MissionWpGetSpeed::Response& response)
{
	if (current_type != MissionType::WAYPOINT)
	{
		printf("Not in Waypoint Mode!\n");
		return false;
	}
	DJI_Pro_Mission_Waypoint_Get_Idle_Speed();
	return true;

}
bool DJISDKMission::mission_wp_set_speed_callback(dji_sdk::MissionWpSetSpeed::Request& request, dji_sdk::MissionWpSetSpeed::Response& response)
{
	if (current_type != MissionType::WAYPOINT)
	{
		printf("Not in Waypoint Mode!\n");
		return false;
	}
	DJI_Pro_Mission_Waypoint_Set_Idle_Speed(request.speed);
	return true;

}
bool DJISDKMission::mission_hp_upload_callback(dji_sdk::MissionHpUpload::Request& request, dji_sdk::MissionHpUpload::Response& response)
{
	hotpoint_task = request.hotpoint_task;
	hotpoint_task.latitude = hotpoint_task.latitude*C_PI/180;
	hotpoint_task.longitude = hotpoint_task.longitude*C_PI/180;
	current_type = MissionType::HOTPOINT;
	printf("current_type -> HP\n");
	return true;

}
bool DJISDKMission::mission_hp_set_speed_callback(dji_sdk::MissionHpSetSpeed::Request& request, dji_sdk::MissionHpSetSpeed::Response& response)
{
	if (current_type != MissionType::HOTPOINT)
	{
		printf("Not in Hotpoint Mode!\n");
		return false;
	}
	cmd_mission_hotpoint_velocity_t new_velocity;
	new_velocity.is_clockwise = request.direction;
	new_velocity.speed = request.speed;
	DJI_Pro_Mission_Hotpoint_Set_Speed(&new_velocity);
	return true;

}
bool DJISDKMission::mission_hp_set_radius_callback(dji_sdk::MissionHpSetRadius::Request& request, dji_sdk::MissionHpSetRadius::Response& response)
{
	if (current_type != MissionType::HOTPOINT)
	{
		printf("Not in Hotpoint Mode!\n");
		return false;
	}
	DJI_Pro_Mission_Hotpoint_Set_Radius(request.radius);
	return true;

}
bool DJISDKMission::mission_hp_reset_yaw_callback(dji_sdk::MissionHpResetYaw::Request& request, dji_sdk::MissionHpResetYaw::Response& response)
{
	if (current_type != MissionType::HOTPOINT)
	{
		printf("Not in Hotpoint Mode!\n");
		return false;
	}
	DJI_Pro_Mission_Hotpoint_Reset_Yaw();
	return true;

}
bool DJISDKMission::mission_fm_upload_callback(dji_sdk::MissionFmUpload::Request& request, dji_sdk::MissionFmUpload::Response& response)
{
	followme_task = request.followme_task;
	followme_task.initial_latitude = followme_task.initial_latitude*C_PI/180;
	followme_task.initial_longitude = followme_task.initial_longitude*C_PI/180;
	current_type = MissionType::FOLLOWME;
	printf("current_type -> FM\n");
	return true;

}
bool DJISDKMission::mission_fm_set_target_callback(dji_sdk::MissionFmSetTarget::Request& request, dji_sdk::MissionFmSetTarget::Response& response)
{
	if (current_type != MissionType::FOLLOWME)
	{
		printf("Not in Followme Mode!\n");
		return false;
	}
	cmd_mission_follow_target_t target_info;
	target_info.latitude = request.followme_target.latitude;
	target_info.longitude = request.followme_target.longitude;
	target_info.altitude = request.followme_target.altitude;
	DJI_Pro_Mission_Followme_Update_Target(&target_info);
	return true;

}

DJISDKMission::DJISDKMission(ros::NodeHandle& nh)
{
	init_missions(nh);
	DJI_Pro_Register_Mission_State_Callback(std::bind(&DJISDKMission::mission_state_callback,this));
	DJI_Pro_Register_Mission_Event_Callback(std::bind(&DJISDKMission::mission_event_callback,this));
}


