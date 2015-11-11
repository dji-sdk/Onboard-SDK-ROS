#include <dji_sdk/dji_sdk_mission.h>

bool DJISDKMission::mission_start_callback(dji_sdk::MissionStart::Request& request, dji_sdk::MissionStart::Response& response)
{
	//the start cmd should run while ready
	if (current_state != ServerState::READY)
		return false;

	switch(current_type)
	{	
		case MissionType::WAYPOINT:
			//send start command 

			break;
		
		case MissionType::HOTPOINT:
			//upload hp task info
			
			break;

		case MissionType::FOLLOWME:
			//upload fm task info

			break;

		default:
			//empty
			break;

	}
	current_state = ServerState::RUNNING;
	

}
bool DJISDKMission::mission_pause_callback(dji_sdk::MissionPause::Request& request, dji_sdk::MissionPause::Response& response)
{
	//the pause cmd should run while running
	if (current_state != ServerState::RUNNING)
		return false;

	switch(current_type)
	{	
		//different cmd id
		case MissionType::WAYPOINT:

			break;
		
		case MissionType::HOTPOINT:
			
			break;

		case MissionType::FOLLOWME:

			break;

		default:

			break;

	}
 	current_state = ServerState::PAUSED;

}
bool DJISDKMission::mission_cancel_callback(dji_sdk::MissionCancel::Request& request, dji_sdk::MissionCancel::Response& response)
{
	//the cancel cmd cannot run while ready
	if (current_state == ServerState::READY)
		return false;

	switch(current_type)
	{	
		//different cmd id
		case MissionType::WAYPOINT:

			break;
		
		case MissionType::HOTPOINT:
			
			break;

		case MissionType::FOLLOWME:

			break;

		default:

			break;

	}
	current_state == ServerState::READY;

}
bool DJISDKMission::mission_download_callback(dji_sdk::MissionDownload::Request& request, dji_sdk::MissionDownload::Response& response)
{
	//the download cmd cannot run while ready
	if (current_state == ServerState::READY)
		return false;
	switch(current_type)
	{	
		//different callback
		case MissionType::WAYPOINT:

			break;
		
		case MissionType::HOTPOINT:
			
			break;

		case MissionType::FOLLOWME:

			break;

		default:

			break;

	}

}
bool DJISDKMission::mission_wp_upload_callback(dji_sdk::MissionWpUpload::Request& request, dji_sdk::MissionWpUpload::Response& response)
{
	//the upload cmd should run while ready
	if (current_state != ServerState::READY)
		return false;

	waypoint_task = request.waypoint_task;
	current_type = MissionType::WAYPOINT;

}
bool DJISDKMission::mission_wp_get_speed_callback(dji_sdk::MissionWpGetSpeed::Request& request, dji_sdk::MissionWpGetSpeed::Response& response)
{
	if (current_type != MissionType::WAYPOINT)
		return false;

}
bool DJISDKMission::mission_wp_set_speed_callback(dji_sdk::MissionWpSetSpeed::Request& request, dji_sdk::MissionWpSetSpeed::Response& response)
{
	if (current_type != MissionType::WAYPOINT)
		return false;

}
bool DJISDKMission::mission_hp_upload_callback(dji_sdk::MissionHpUpload::Request& request, dji_sdk::MissionHpUpload::Response& response)
{
	//the upload cmd should run while ready
	if (current_state != ServerState::READY)
		return false;
	hotpoint_task = request.hotpoint_task;
	current_type = MissionType::HOTPOINT;

}
bool DJISDKMission::mission_hp_set_speed_callback(dji_sdk::MissionHpSetSpeed::Request& request, dji_sdk::MissionHpSetSpeed::Response& response)
{
	if (current_type != MissionType::HOTPOINT)
		return false;

}
bool DJISDKMission::mission_hp_set_radiu_callback(dji_sdk::MissionHpSetRadiu::Request& request, dji_sdk::MissionHpSetRadiu::Response& response)
{
	if (current_type != MissionType::HOTPOINT)
		return false;

}
bool DJISDKMission::mission_hp_reset_yaw_callback(dji_sdk::MissionHpResetYaw::Request& request, dji_sdk::MissionHpResetYaw::Response& response)
{
	if (current_type != MissionType::HOTPOINT)
		return false;

}
bool DJISDKMission::mission_fm_upload_callback(dji_sdk::MissionFmUpload::Request& request, dji_sdk::MissionFmUpload::Response& response)
{
	//the upload cmd should run while ready
	if (current_state != ServerState::READY)
		return false;
	followme_task = request.followme_task;
	current_type = MissionType::FOLLOWME;

}
bool DJISDKMission::mission_fm_set_target_callback(dji_sdk::MissionFmSetTarget::Request& request, dji_sdk::MissionFmSetTarget::Response& response)
{
	if (current_type != MissionType::FOLLOWME)
		return false;

}

DJISDKMission::DJISDKMission(ros::NodeHandle& nh)
{
	init_missions(nh);
}
