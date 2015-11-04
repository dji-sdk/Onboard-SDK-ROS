#ifndef __DJI_MISSION_NODE_H__
#define __DJI_MISSION_NODE_H__

#include <ros/ros.h>
#include <dji_sdk/dji_sdk.h>

class DJIMission
{
private:
	//mission data publisher, processing data from N1
	ros::Publisher mission_event_pub;
	ros::Publisher mission_state_pub;

	//common mission request subscriber
	ros::Subscriber mission_start_request;
	ros::Subscriber mission_pause_request;
	ros::Subscriber mission_download_request;
	ros::Subscriber mission_cancel_request;

	//subscriber running when operating waypoint
	ros::Subscriber mission_wp_upload;
	ros::Subscriber mission_wp_download_wp;
	ros::Subscriber mission_wp_set_speed;
	ros::Subscriber mission_wp_get_speed;

	//subscriber running when operating hotpoint
	ros::Subscriber mission_hp_set_speed;
	ros::Subscriber mission_hp_set_radiu;
	ros::Subscriber mission_hp_reset_yaw;

	//subscriber running when operating followme
	ros::Subscriber mission_fm_set_target;

	enum serverState {
		READY,
		RUNNING,
		PAUSED
	};

	enum missionType {
		WAYPOINT,
		HOTPOINT,
		FOLLOWME
	};


	int init_missionServer(ros::NodeHandle &nh);

		

};

#endif
