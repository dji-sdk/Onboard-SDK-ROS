#include <dji_sdk/dji_sdk_mission.h>

bool DJISDKMission::mission_start_callback(dji_sdk::MissionStart::Request& request, dji_sdk::MissionStart::Response& response)
{
	//the start cmd should run while ready
	if (current_state != ServerState::READY)
		return false;
	current_state = ServerState::RUNNING;

}
bool DJISDKMission::mission_pause_callback(dji_sdk::MissionPause::Request& request, dji_sdk::MissionPause::Response& response)
{
	//the pause cmd should run while running
	if (current_state != ServerState::RUNNING)
		return false;
 	current_state = ServerState::PAUSED;

}
bool DJISDKMission::mission_cancel_callback(dji_sdk::MissionCancel::Request& request, dji_sdk::MissionCancel::Response& response)
{
	//the cancel cmd cannot run while ready
	if (current_state == ServerState::READY)
		return false;
	current_state == ServerState::READY;

}
bool DJISDKMission::mission_download_callback(dji_sdk::MissionDownload::Request& request, dji_sdk::MissionDownload::Response& response)
{
	//the download cmd cannot run while ready
	if (current_state == ServerState::READY)
		return false;

}
bool DJISDKMission::mission_wp_upload_callback(dji_sdk::MissionWpUpload::Request& request, dji_sdk::MissionWpUpload::Response& response)
{
	//the upload cmd should run while ready
	if (current_state != ServerState::READY)
		return false;

}
bool DJISDKMission::mission_wp_get_speed_callback(dji_sdk::MissionWpGetSpeed::Request& request, dji_sdk::MissionWpGetSpeed::Response& response)
{

}
bool DJISDKMission::mission_wp_set_speed_callback(dji_sdk::MissionWpSetSpeed::Request& request, dji_sdk::MissionWpSetSpeed::Response& response)
{

}
bool DJISDKMission::mission_hp_upload_callback(dji_sdk::MissionHpUpload::Request& request, dji_sdk::MissionHpUpload::Response& response)
{
	//the upload cmd should run while ready
	if (current_state != ServerState::READY)
		return false;

}
bool DJISDKMission::mission_hp_set_speed_callback(dji_sdk::MissionHpSetSpeed::Request& request, dji_sdk::MissionHpSetSpeed::Response& response)
{

}
bool DJISDKMission::mission_hp_set_radiu_callback(dji_sdk::MissionHpSetRadiu::Request& request, dji_sdk::MissionHpSetRadiu::Response& response)
{

}
bool DJISDKMission::mission_hp_reset_yaw_callback(dji_sdk::MissionHpResetYaw::Request& request, dji_sdk::MissionHpResetYaw::Response& response)
{

}
bool DJISDKMission::mission_fm_upload_callback(dji_sdk::MissionFmUpload::Request& request, dji_sdk::MissionFmUpload::Response& response)
{
	//the upload cmd should run while ready
	if (current_state != ServerState::READY)
		return false;

}
bool DJISDKMission::mission_fm_set_target_callback(dji_sdk::MissionFmSetTarget::Request& request, dji_sdk::MissionFmSetTarget::Response& response)
{

}

DJISDKMission::DJISDKMission(ros::NodeHandle& nh)
{
	init_missions(nh);
}
