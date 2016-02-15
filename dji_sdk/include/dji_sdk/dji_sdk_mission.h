#ifndef __DJI_MISSION_NODE_H__
#define __DJI_MISSION_NODE_H__

#include <ros/ros.h>
#include <dji_sdk/dji_sdk.h>

#define C_PI (double) 3.141592653589793

extern DJI::onboardSDK::ROSAdapter *rosAdapter;

enum class MissionType 
{
	EMPTY,
	WAYPOINT,
	HOTPOINT,
	FOLLOWME
};

class DJISDKMission
{
private:
	dji_sdk::MissionPushInfo mission_status;
	dji_sdk::MissionPushInfo mission_event;

	dji_sdk::MissionWaypointTask waypoint_task;
	dji_sdk::MissionHotpointTask hotpoint_task;
	dji_sdk::MissionFollowmeTask followme_task;
	
public:
	DJISDKMission(ros::NodeHandle& nh);
	void mission_status_callback(uint8_t *buf, uint8_t len);
	void mission_event_callback(uint8_t *buf, uint8_t len);
private:
	//mission data publisher, processing data from N1
	ros::Publisher mission_status_publisher;
	ros::Publisher mission_event_publisher;

	//common mission request subscriber
	ros::ServiceServer mission_start_service;
	ros::ServiceServer mission_pause_service;
	ros::ServiceServer mission_cancel_service;

	//subscriber running when operating waypoint
	ros::ServiceServer mission_wp_upload_service;
	ros::ServiceServer mission_wp_download_service;
	ros::ServiceServer mission_wp_set_speed_service;
	ros::ServiceServer mission_wp_get_speed_service;

	//subscriber running when operating hotpoint
	ros::ServiceServer mission_hp_upload_service;
	ros::ServiceServer mission_hp_download_service;
	ros::ServiceServer mission_hp_set_speed_service;
	ros::ServiceServer mission_hp_set_radius_service;
	ros::ServiceServer mission_hp_reset_yaw_service;

	//subscriber running when operating followme
	ros::ServiceServer mission_fm_upload_service;
	ros::ServiceServer mission_fm_set_target_service;
	
	MissionType current_type = MissionType::EMPTY;

	bool mission_start_callback(dji_sdk::MissionStart::Request& request, dji_sdk::MissionStart::Response& response);
	bool mission_pause_callback(dji_sdk::MissionPause::Request& request, dji_sdk::MissionPause::Response& response);
	bool mission_cancel_callback(dji_sdk::MissionCancel::Request& request, dji_sdk::MissionCancel::Response& response);
	bool mission_wp_upload_callback(dji_sdk::MissionWpUpload::Request& request, dji_sdk::MissionWpUpload::Response& response);
	bool mission_wp_download_callback(dji_sdk::MissionWpDownload::Request& request, dji_sdk::MissionWpDownload::Response& response);
	bool mission_wp_get_speed_callback(dji_sdk::MissionWpGetSpeed::Request& request, dji_sdk::MissionWpGetSpeed::Response& response);
	bool mission_wp_set_speed_callback(dji_sdk::MissionWpSetSpeed::Request& request, dji_sdk::MissionWpSetSpeed::Response& response);
	bool mission_hp_upload_callback(dji_sdk::MissionHpUpload::Request& request, dji_sdk::MissionHpUpload::Response& response);
	bool mission_hp_download_callback(dji_sdk::MissionHpDownload::Request& request, dji_sdk::MissionHpDownload::Response& response);
	bool mission_hp_set_speed_callback(dji_sdk::MissionHpSetSpeed::Request& request, dji_sdk::MissionHpSetSpeed::Response& response);
	bool mission_hp_set_radius_callback(dji_sdk::MissionHpSetRadius::Request& request, dji_sdk::MissionHpSetRadius::Response& response);
	bool mission_hp_reset_yaw_callback(dji_sdk::MissionHpResetYaw::Request& request, dji_sdk::MissionHpResetYaw::Response& response);
	bool mission_fm_upload_callback(dji_sdk::MissionFmUpload::Request& request, dji_sdk::MissionFmUpload::Response& response);
	bool mission_fm_set_target_callback(dji_sdk::MissionFmSetTarget::Request& request, dji_sdk::MissionFmSetTarget::Response& response);


	void init_missions(ros::NodeHandle& nh)
	{
		mission_status_publisher = nh.advertise<dji_sdk::MissionPushInfo>("dji_sdk/mission_status", 10);
		mission_event_publisher = nh.advertise<dji_sdk::MissionPushInfo>("dji_sdk/mission_event", 10);

		mission_start_service = nh.advertiseService("dji_sdk/mission_start", &DJISDKMission::mission_start_callback ,this);
		mission_pause_service = nh.advertiseService("dji_sdk/mission_pause", &DJISDKMission::mission_pause_callback ,this);
		mission_cancel_service = nh.advertiseService("dji_sdk/mission_cancel", &DJISDKMission::mission_cancel_callback ,this);
		mission_wp_upload_service = nh.advertiseService("dji_sdk/mission_waypoint_upload", &DJISDKMission::mission_wp_upload_callback,this);
		mission_wp_download_service = nh.advertiseService("dji_sdk/mission_waypoint_download", &DJISDKMission::mission_wp_download_callback,this);
		mission_wp_set_speed_service = nh.advertiseService("dji_sdk/mission_waypoint_set_speed", &DJISDKMission::mission_wp_set_speed_callback ,this);
		mission_wp_get_speed_service = nh.advertiseService("dji_sdk/mission_waypoint_get_speed", &DJISDKMission::mission_wp_get_speed_callback ,this);
		mission_hp_upload_service = nh.advertiseService("dji_sdk/mission_hotpoint_upload", &DJISDKMission::mission_hp_upload_callback ,this);
		mission_hp_download_service = nh.advertiseService("dji_sdk/mission_hotpoint_download", &DJISDKMission::mission_hp_download_callback,this);
		mission_hp_set_speed_service = nh.advertiseService("dji_sdk/mission_hotpoint_set_speed", &DJISDKMission::mission_hp_set_speed_callback ,this);
		mission_hp_set_radius_service = nh.advertiseService("dji_sdk/mission_hotpoint_set_radius", &DJISDKMission::mission_hp_set_radius_callback ,this);
		mission_hp_reset_yaw_service = nh.advertiseService("dji_sdk/mission_hotpoint_reset_yaw", &DJISDKMission::mission_hp_reset_yaw_callback ,this);
		mission_fm_upload_service = nh.advertiseService("dji_sdk/mission_followme_upload", &DJISDKMission::mission_fm_upload_callback ,this);
		mission_fm_set_target_service = nh.advertiseService("dji_sdk/mission_followme_set_target", &DJISDKMission::mission_fm_set_target_callback ,this);

	}
};

#endif
