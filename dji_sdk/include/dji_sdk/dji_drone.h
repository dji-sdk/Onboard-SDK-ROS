/** @file dji_drone.h
 *  @version 3.1.8
 *  @date July 29th, 2016
 *
 *  @brief
 *  Contains client side ROS code. Including this header
 *  as a part of your project will allow your project to be used as a custom client. 
 *
 *  @copyright 2016 DJI. All rights reserved.
 *
 */

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

    typedef actionlib::SimpleActionClient<dji_sdk::DroneTaskAction> DroneTaskActionClient;
    typedef actionlib::SimpleActionClient<dji_sdk::LocalPositionNavigationAction> LocalPositionNavigationActionClient;
    typedef actionlib::SimpleActionClient<dji_sdk::GlobalPositionNavigationAction> GlobalPositionNavigationActionClient;
    typedef actionlib::SimpleActionClient<dji_sdk::WaypointNavigationAction> WaypointNavigationActionClient;

	DroneTaskActionClient drone_task_action_client;
	LocalPositionNavigationActionClient local_position_navigation_action_client;
	GlobalPositionNavigationActionClient global_position_navigation_action_client;
	WaypointNavigationActionClient waypoint_navigation_action_client;

	ros::ServiceClient activation_service;
	ros::ServiceClient attitude_control_service;
	ros::ServiceClient camera_action_control_service;
	ros::ServiceClient drone_task_control_service;
	ros::ServiceClient gimbal_angle_control_service;
	ros::ServiceClient gimbal_speed_control_service;
	ros::ServiceClient global_position_control_service;
	ros::ServiceClient local_position_control_service;
	ros::ServiceClient sdk_permission_control_service;
	ros::ServiceClient velocity_control_service;
	ros::ServiceClient version_check_service;

	ros::ServiceClient virtual_rc_enable_control_service;
	ros::ServiceClient virtual_rc_data_control_service;
	ros::ServiceClient drone_arm_control_service;
	ros::ServiceClient sync_flag_control_service;
	ros::ServiceClient message_frequency_control_service;
	ros::ServiceClient mission_start_service;
	ros::ServiceClient mission_pause_service;
	ros::ServiceClient mission_cancel_service;
	ros::ServiceClient mission_wp_upload_service;
	ros::ServiceClient mission_wp_download_service;
	ros::ServiceClient mission_wp_set_speed_service;
	ros::ServiceClient mission_wp_get_speed_service;
	ros::ServiceClient mission_hp_upload_service;
	ros::ServiceClient mission_hp_download_service;
	ros::ServiceClient mission_hp_set_speed_service;
	ros::ServiceClient mission_hp_set_radius_service;
	ros::ServiceClient mission_hp_reset_yaw_service;
	ros::ServiceClient mission_fm_upload_service;
	ros::ServiceClient mission_fm_set_target_service;
	//ros::ServiceClient mobile_commands_service;

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

	ros::Subscriber time_stamp_subscriber;
	ros::Subscriber mission_status_subscriber;
	ros::Subscriber mission_event_subscriber;
	ros::Subscriber mobile_data_subscriber;

public:

	  dji_sdk::Acceleration acceleration;
		dji_sdk::AttitudeQuaternion attitude_quaternion;
		dji_sdk::Compass compass;
		dji_sdk::FlightControlInfo flight_control_info;
		uint8_t flight_status;
		uint8_t mobile_new_data;
		dji_sdk::Gimbal gimbal;
		dji_sdk::GlobalPosition global_position;
		dji_sdk::GlobalPosition global_position_ref;
		dji_sdk::LocalPosition local_position;
		dji_sdk::LocalPosition local_position_ref;
		dji_sdk::PowerStatus power_status;
		dji_sdk::TransparentTransmissionData mobile_data;
		dji_sdk::RCChannels rc_channels;
   	dji_sdk::Velocity velocity;
   	nav_msgs::Odometry odometry;
   	dji_sdk::TimeStamp time_stamp;
   	bool activation = false;
   	bool localposbase_use_height = true;

	uint8_t mission_type;

	uint8_t incident_type;

	dji_sdk::MissionStatusWaypoint waypoint_mission_push_info;
	dji_sdk::MissionStatusHotpoint hotpoint_mission_push_info;
	dji_sdk::MissionStatusFollowme followme_mission_push_info;
	dji_sdk::MissionStatusOther other_mission_push_info;

	dji_sdk::MissionEventWpUpload waypoint_upload_result;
	dji_sdk::MissionEventWpAction waypoint_action_result;
	dji_sdk::MissionEventWpReach waypoint_reached_result;

typedef void *UserData; 
typedef void (*CallBack)(DJIDrone *);

typedef struct CallBackHandler
{
  CallBack callback;
  UserData userData;
} CallBackHandler;

private:
	void acceleration_subscriber_callback(dji_sdk::Acceleration acceleration)
	{
		this->acceleration = acceleration;
	}

	void attitude_quaternion_subscriber_callback(dji_sdk::AttitudeQuaternion attitude_quaternion)
	{
		this->attitude_quaternion = attitude_quaternion;
	}

	void compass_subscriber_callback(dji_sdk::Compass compass)
	{
		this->compass = compass;
	}

	void flight_control_info_subscriber_callback(dji_sdk::FlightControlInfo flight_control_info)
	{
		this->flight_control_info = flight_control_info;
	}

	void flight_status_subscriber_callback(std_msgs::UInt8 flight_status)
	{
		this->flight_status = flight_status.data;
	}

	void gimbal_subscriber_callback(dji_sdk::Gimbal gimbal)
	{
		this->gimbal = gimbal;
	}

	void global_position_subscriber_callback(dji_sdk::GlobalPosition global_position)
	{
		this->global_position = global_position;
	}

	void local_position_subscriber_callback(dji_sdk::LocalPosition local_position)
	{
		this->local_position = local_position;
	}

	void power_status_subscriber_callback(dji_sdk::PowerStatus power_status)
	{
		this->power_status = power_status;
	}

	void rc_channels_subscriber_callback(dji_sdk::RCChannels rc_channels)
	{
		this->rc_channels = rc_channels;
	}

	void velocity_subscriber_callback(dji_sdk::Velocity velocity)
	{
		this->velocity = velocity;
	}

	void activation_subscriber_callback(std_msgs::UInt8 activation)
	{
		this->activation = activation.data;
	}

	void odometry_subscriber_callback(nav_msgs::Odometry odometry)
	{
		this->odometry = odometry;
	}

	void time_stamp_subscriber_callback(dji_sdk::TimeStamp time_stamp)
	{
		this->time_stamp = time_stamp;
	}

	void mission_status_push_info_callback(dji_sdk::MissionPushInfo status_push_info)
	{
		this->mission_type = status_push_info.type;
		switch(status_push_info.type)
		{
			case 1:
				this->waypoint_mission_push_info.mission_type = status_push_info.type;
				this->waypoint_mission_push_info.target_waypoint = status_push_info.data_1;
				this->waypoint_mission_push_info.current_status = status_push_info.data_2;
				this->waypoint_mission_push_info.error_code = status_push_info.data_3;
				break;

			case 2:
				this->hotpoint_mission_push_info.mission_type = status_push_info.type;
				this->hotpoint_mission_push_info.mission_status= status_push_info.data_1;
				this->hotpoint_mission_push_info.hotpoint_radius = status_push_info.data_2 << 8 | status_push_info.data_3;
				this->hotpoint_mission_push_info.error_code = status_push_info.data_4;
				this->hotpoint_mission_push_info.hotpoint_velocity = status_push_info.data_5;
				break;

			case 3:
				this->followme_mission_push_info.mission_type = status_push_info.type;
				break;

			case 0:
			case 4:
				this->other_mission_push_info.mission_type = status_push_info.type;
				this->other_mission_push_info.last_mission_type = status_push_info.data_1;
				this->other_mission_push_info.is_broken = status_push_info.data_2;
				this->other_mission_push_info.error_code = status_push_info.data_3;
				break;
			default:
				break;
		}
	
	}

    //! Callback Handler functions for Mobile data commands
	CallBackHandler obtainControlCallback;
	CallBackHandler releaseControlCallback;
	CallBackHandler takeOffCallback;
	CallBackHandler landingCallback;
	CallBackHandler getSDKVersionCallback;
	CallBackHandler armCallback;
	CallBackHandler disArmCallback;
	CallBackHandler goHomeCallback;
	CallBackHandler takePhotoCallback;
	CallBackHandler startVideoCallback;
	CallBackHandler stopVideoCallback;
	CallBackHandler drawCircleDemoCallback;
	CallBackHandler drawSquareDemoCallback;
	CallBackHandler attitudeControlDemoCallback;
	CallBackHandler waypointNavigationTestCallback;
	CallBackHandler localNavigationTestCallback;
	CallBackHandler globalNavigationTestCallback;
	CallBackHandler virtualRCTestCallback;
	CallBackHandler gimbalControlDemoCallback;
	CallBackHandler startMapLASLoggingCallback;
	CallBackHandler stopMapLASLoggingCallback;
	CallBackHandler startCollisionAvoidanceCallback;
	CallBackHandler stopCollisionAvoidanceCallback;


	void mobile_data_push_info_callback(dji_sdk::TransparentTransmissionData information)
	{
		this->mobile_data = information;
		mobile_new_data = 1;
		int cmdID = mobile_data.data[0];
		printf("Command ID code is %d \n", cmdID);

		switch(cmdID)
		{
			case 2:
			if (obtainControlCallback.callback)
			{
			 obtainControlCallback.callback(this);
				}
				break;

				case 3:
				if (releaseControlCallback.callback)
			{
			 releaseControlCallback.callback(this);
				}
				break;

				case 4:
				//if (obtainControlCallback.callback)
			//{
			// obtainControlCallback.callback();
				//}
				break;

				case 5:
				if (armCallback.callback)
			{
			 armCallback.callback(this);
				}
				break;

				case 6:
				if (disArmCallback.callback)
			{
			 disArmCallback.callback(this);
				}
				break;

				case 7:
				if (takeOffCallback.callback)
			{
			 takeOffCallback.callback(this);
				}
				break;

				case 8:
				if (landingCallback.callback)
			{
			 landingCallback.callback(this);
				}
				break;

				case 9:
				if (goHomeCallback.callback)
			{
			 goHomeCallback.callback(this);
				}
				break;

				case 10:
				if (takePhotoCallback.callback)
			{
			 takePhotoCallback.callback(this);
				}
				break;

				case 11:
				if (startVideoCallback.callback)
			{
			 startVideoCallback.callback(this);
				}
				break;

				case 13:
				if (stopVideoCallback.callback)
			{
			 stopVideoCallback.callback(this);
				}
				break;

			case 20:
				if (startMapLASLoggingCallback.callback)
				{
					startMapLASLoggingCallback.callback(this);
				}

				break;

			case 21:
				if (stopMapLASLoggingCallback.callback)
				{
					stopMapLASLoggingCallback.callback(this);
				}
				break;

			case 22:
				if (startCollisionAvoidanceCallback.callback)
				{
					startCollisionAvoidanceCallback.callback(this);
				}

				break;

			case 23:
				if (stopCollisionAvoidanceCallback.callback)
				{
					stopCollisionAvoidanceCallback.callback(this);
				}
				break;


				case 61:
				if (drawCircleDemoCallback.callback)
			{
			 drawCircleDemoCallback.callback(this);
				}
				break;


				case 62:
				if (drawSquareDemoCallback.callback)
			{
			 drawSquareDemoCallback.callback(this);
				}
				break;

				case 63:
				if (attitudeControlDemoCallback.callback)
			{
			 attitudeControlDemoCallback.callback(this);
				}
				break;

				case 64:
				if (gimbalControlDemoCallback.callback)
			{
			 gimbalControlDemoCallback.callback(this);
				}
				break;

				case 65:
				if (waypointNavigationTestCallback.callback)
			{
			 waypointNavigationTestCallback.callback(this);
				}
				break;

				case 66:
				if (localNavigationTestCallback.callback)
			{
			 localNavigationTestCallback.callback(this);
				}
				break;

				case 67:
				if (globalNavigationTestCallback.callback)
			{
			 globalNavigationTestCallback.callback(this);
				}
				break;

				case 68:
				if (virtualRCTestCallback.callback)
			{
			 virtualRCTestCallback.callback(this);
				}
				break;


		}
	}

	void mission_event_push_info_callback(dji_sdk::MissionPushInfo event_push_info)
	{
		this->incident_type = event_push_info.type;
		switch(event_push_info.type)
		{
			case 0:
				this->waypoint_upload_result.incident_type = event_push_info.type;
				this->waypoint_upload_result.mission_valid = event_push_info.data_1;
				this->waypoint_upload_result.estimated_runtime = event_push_info.data_2 << 8 | event_push_info.data_3;
				break;
			case 1:
				this->waypoint_action_result.incident_type = event_push_info.type;
				this->waypoint_action_result.repeat = event_push_info.data_1;
				break;
			case 2:
				this->waypoint_reached_result.incident_type = event_push_info.type;
				this->waypoint_reached_result.waypoint_index = event_push_info.data_1;
				this->waypoint_reached_result.current_status = event_push_info.data_2;
				break;
			default:
				break;
		}
	}

public:
	DJIDrone(ros::NodeHandle& nh):
		drone_task_action_client(nh, "dji_sdk/drone_task_action", true),
		local_position_navigation_action_client(nh, "dji_sdk/local_position_navigation_action", true),
		global_position_navigation_action_client(nh, "dji_sdk/global_position_navigation_action", true),
		waypoint_navigation_action_client(nh, "dji_sdk/waypoint_navigation_action", true)
	{
		activation_service = nh.serviceClient<dji_sdk::Activation>("dji_sdk/activation");
	    	attitude_control_service = nh.serviceClient<dji_sdk::AttitudeControl>("dji_sdk/attitude_control");
	    	camera_action_control_service = nh.serviceClient<dji_sdk::CameraActionControl>("dji_sdk/camera_action_control");
	    	drone_task_control_service = nh.serviceClient<dji_sdk::DroneTaskControl>("dji_sdk/drone_task_control");
	    	gimbal_angle_control_service = nh.serviceClient<dji_sdk::GimbalAngleControl>("dji_sdk/gimbal_angle_control");
	    	gimbal_speed_control_service = nh.serviceClient<dji_sdk::GimbalSpeedControl>("dji_sdk/gimbal_speed_control");
	    	global_position_control_service = nh.serviceClient<dji_sdk::GlobalPositionControl>("dji_sdk/global_position_control");
	    	local_position_control_service = nh.serviceClient<dji_sdk::LocalPositionControl>("dji_sdk/local_position_control");
	    	sdk_permission_control_service = nh.serviceClient<dji_sdk::SDKPermissionControl>("dji_sdk/sdk_permission_control");
	    	velocity_control_service = nh.serviceClient<dji_sdk::VelocityControl>("dji_sdk/velocity_control");
		version_check_service = nh.serviceClient<dji_sdk::VersionCheck>("dji_sdk/version_check");
		virtual_rc_enable_control_service = nh.serviceClient<dji_sdk::VirtualRCEnableControl>("dji_sdk/virtual_rc_enable_control");
		virtual_rc_data_control_service = nh.serviceClient<dji_sdk::VirtualRCDataControl>("dji_sdk/virtual_rc_data_control");
		drone_arm_control_service = nh.serviceClient<dji_sdk::DroneArmControl>("dji_sdk/drone_arm_control");
		sync_flag_control_service = nh.serviceClient<dji_sdk::SyncFlagControl>("dji_sdk/sync_flag_control");
		message_frequency_control_service = nh.serviceClient<dji_sdk::MessageFrequencyControl>("dji_sdk/message_frequency_control");
		//mobile_commands_service = nh.serviceClient<dji_sdk::mobileCommandsl>("dji_sdk/mobile_commands");

		mission_start_service = nh.serviceClient<dji_sdk::MissionStart>("dji_sdk/mission_start");
		mission_pause_service = nh.serviceClient<dji_sdk::MissionPause>("dji_sdk/mission_pause");
		mission_cancel_service = nh.serviceClient<dji_sdk::MissionCancel>("dji_sdk/mission_cancel");
		mission_wp_upload_service = nh.serviceClient<dji_sdk::MissionWpUpload>("dji_sdk/mission_waypoint_upload");
		mission_wp_download_service = nh.serviceClient<dji_sdk::MissionWpDownload>("dji_sdk/mission_waypoint_download");
		mission_wp_set_speed_service = nh.serviceClient<dji_sdk::MissionWpSetSpeed>("dji_sdk/mission_waypoint_set_speed");
		mission_wp_get_speed_service = nh.serviceClient<dji_sdk::MissionWpGetSpeed>("dji_sdk/mission_waypoint_get_speed");
		mission_hp_upload_service = nh.serviceClient<dji_sdk::MissionHpUpload>("dji_sdk/mission_hotpoint_upload");
		mission_hp_download_service = nh.serviceClient<dji_sdk::MissionHpDownload>("dji_sdk/mission_hotpoint_download");
		mission_hp_set_speed_service = nh.serviceClient<dji_sdk::MissionHpSetSpeed>("dji_sdk/mission_hotpoint_set_speed");
		mission_hp_set_radius_service = nh.serviceClient<dji_sdk::MissionHpSetRadius>("dji_sdk/mission_hotpoint_set_radius");
		mission_hp_reset_yaw_service = nh.serviceClient<dji_sdk::MissionHpResetYaw>("dji_sdk/mission_hotpoint_reset_yaw");
		mission_fm_upload_service = nh.serviceClient<dji_sdk::MissionFmUpload>("dji_sdk/mission_followme_upload");
		mission_fm_set_target_service = nh.serviceClient<dji_sdk::MissionFmSetTarget>("dji_sdk/mission_followme_set_target");

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
		time_stamp_subscriber = nh.subscribe<dji_sdk::TimeStamp>("dji_sdk/time_stamp", 10, &DJIDrone::time_stamp_subscriber_callback,this);
		mission_status_subscriber = nh.subscribe<dji_sdk::MissionPushInfo>("dji_sdk/mission_status", 10, &DJIDrone::mission_status_push_info_callback, this);  
		mission_event_subscriber = nh.subscribe<dji_sdk::MissionPushInfo>("dji_sdk/mission_event", 10, &DJIDrone::mission_event_push_info_callback, this);
		mobile_data_subscriber = nh.subscribe<dji_sdk::TransparentTransmissionData>("dji_sdk/data_received_from_remote_device", 10, &DJIDrone::mobile_data_push_info_callback, this);
	}

	void setObtainControlMobileCallback(DJIDrone::CallBack userCallback, UserData userData)
	{
  		obtainControlCallback.callback = userCallback;
  		obtainControlCallback.userData = userData;
	}


	void setReleaseControlMobileCallback(DJIDrone::CallBack userCallback, UserData userData)
	{
  		releaseControlCallback.callback = userCallback;
  		releaseControlCallback.userData = userData;
	}

	void setTakeOffMobileCallback(DJIDrone::CallBack userCallback, UserData userData)
	{
  		takeOffCallback.callback = userCallback;
  		takeOffCallback.userData = userData;
	}


	void setLandingMobileCallback(DJIDrone::CallBack userCallback, UserData userData)
	{
  		landingCallback.callback = userCallback;
  		landingCallback.userData = userData;
	}


	void setGetSDKVersionMobileCallback(DJIDrone::CallBack userCallback, UserData userData)
	{
  	    getSDKVersionCallback.callback = userCallback;
  		getSDKVersionCallback.userData = userData;
	}


	void setArmMobileCallback(DJIDrone::CallBack userCallback, UserData userData)
	{
  		armCallback.callback = userCallback;
  		armCallback.userData = userData;
	}

	void setDisarmMobileCallback(DJIDrone::CallBack userCallback, UserData userData)
	{
  		disArmCallback.callback = userCallback;
  		disArmCallback.userData = userData;
	}



	void setGoHomeMobileCallback(DJIDrone::CallBack userCallback, UserData userData)
	{
  		goHomeCallback.callback = userCallback;
  		goHomeCallback.userData = userData;
	}


	void setTakePhotoMobileCallback(DJIDrone::CallBack userCallback, UserData userData)
	{
  		takePhotoCallback.callback = userCallback;
  		takePhotoCallback.userData = userData;
	}


	void setStartVideoMobileCallback(DJIDrone::CallBack userCallback, UserData userData)
	{
  		startVideoCallback.callback = userCallback;
  		startVideoCallback.userData = userData;
	}


	void setStopVideoMobileCallback(DJIDrone::CallBack userCallback, UserData userData)
	{
  		stopVideoCallback.callback = userCallback;
  		stopVideoCallback.userData = userData;
	}

	void setDrawCircleDemoMobileCallback(DJIDrone::CallBack userCallback, UserData userData)
	{
		drawCircleDemoCallback.callback = userCallback;
		drawCircleDemoCallback.userData = userData;
	}

	void setDrawSquareDemoMobileCallback(DJIDrone::CallBack userCallback, UserData userData)
	{
		drawSquareDemoCallback.callback = userCallback;
		drawSquareDemoCallback.userData = userData;
	}
    
    void setAttitudeControlDemoMobileCallback(DJIDrone::CallBack userCallback, UserData userData)
    {
    	attitudeControlDemoCallback.callback = userCallback;
		attitudeControlDemoCallback.userData = userData;
    }

      void setLocalNavigationTestMobileCallback(DJIDrone::CallBack userCallback, UserData userData)
    {
    	localNavigationTestCallback.callback = userCallback;
		localNavigationTestCallback.userData = userData;
    }

      void setGlobalNavigationTestMobileCallback(DJIDrone::CallBack userCallback, UserData userData)
    {
    	globalNavigationTestCallback.callback = userCallback;
		globalNavigationTestCallback.userData = userData;
    }

      void setWaypointNavigationTestMobileCallback(DJIDrone::CallBack userCallback, UserData userData)
    {
    	waypointNavigationTestCallback.callback = userCallback;
		waypointNavigationTestCallback.userData = userData;
    }

      void setVirtuaRCTestMobileCallback(DJIDrone::CallBack userCallback, UserData userData)
    {
    	virtualRCTestCallback.callback = userCallback;
		virtualRCTestCallback.userData = userData;
    }

      void setGimbalControlDemoMobileCallback(DJIDrone::CallBack userCallback, UserData userData)
    {
    	gimbalControlDemoCallback.callback = userCallback;
		gimbalControlDemoCallback.userData = userData;
    }

	void setStartMapLASLoggingMobileCallback(DJIDrone::CallBack userCallback, UserData userData)
	{
		startMapLASLoggingCallback.callback = userCallback;
		startMapLASLoggingCallback.userData = userData;
	}

	void setStopMapLASLoggingMobileCallback(DJIDrone::CallBack userCallback, UserData userData)
	{
		stopMapLASLoggingCallback.callback = userCallback;
		stopMapLASLoggingCallback.userData = userData;
	}

	void setStartCollisionAvoidanceCallback(DJIDrone::CallBack userCallback, UserData userData)
	{
		startCollisionAvoidanceCallback.callback = userCallback;
		startCollisionAvoidanceCallback.userData = userData;
	}

	void setStopCollisionAvoidanceCallback(DJIDrone::CallBack userCallback, UserData userData)
	{
		stopCollisionAvoidanceCallback.callback = userCallback;
		stopCollisionAvoidanceCallback.userData = userData;
	}

	bool activate()
	{
		dji_sdk::Activation activate;
		return activation_service.call(activate) && activate.response.result;
	}
	
	bool check_version()
	{
		dji_sdk::VersionCheck version_check;
		return version_check_service.call(version_check) && version_check.response.result;
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

	bool start_video()
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

	bool gimbal_speed_control(int roll_rate = 0, int pitch_rate = 0, int yaw_rate = 0)
	{
		dji_sdk::GimbalSpeedControl gimbal_speed_control;
		gimbal_speed_control.request.roll_rate = roll_rate;
		gimbal_speed_control.request.pitch_rate = pitch_rate;
		gimbal_speed_control.request.yaw_rate = yaw_rate;

		return gimbal_speed_control_service.call(gimbal_speed_control) && gimbal_speed_control.response.result;
	}

	bool gimbal_angle_control(int roll = 0, int pitch = 0, int yaw = 0, int duration = 0, bool absolute_or_incremental = 1, bool yaw_cmd_ignore = 0, bool roll_cmd_ignore = 0, bool pitch_cmd_ignore = 0)
	{
		dji_sdk::GimbalAngleControl gimbal_angle_control;
		gimbal_angle_control.request.roll = roll;
		gimbal_angle_control.request.pitch = pitch;
		gimbal_angle_control.request.yaw = yaw;
		gimbal_angle_control.request.duration = duration;
		gimbal_angle_control.request.absolute_or_incremental = absolute_or_incremental;
		gimbal_angle_control.request.yaw_cmd_ignore = yaw_cmd_ignore;
		gimbal_angle_control.request.roll_cmd_ignore = roll_cmd_ignore;
		gimbal_angle_control.request.pitch_cmd_ignore = pitch_cmd_ignore;

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
		velocity_control.request.yawRate = yaw;
	
		return velocity_control_service.call(velocity_control) && velocity_control.response.result;
	}

	bool virtual_rc_enable()
	{
		dji_sdk::VirtualRCEnableControl virtual_rc_enable_control;
		virtual_rc_enable_control.request.enable = 1;
		virtual_rc_enable_control.request.if_back_to_real = 1;
		
		return virtual_rc_enable_control_service.call(virtual_rc_enable_control) && virtual_rc_enable_control.response.result;
	}

	bool virtual_rc_disable()
	{
		dji_sdk::VirtualRCEnableControl virtual_rc_enable_control;
		virtual_rc_enable_control.request.enable = 0;
		virtual_rc_enable_control.request.if_back_to_real = 1;
		
		return virtual_rc_enable_control_service.call(virtual_rc_enable_control) && virtual_rc_enable_control.response.result;
	}

	bool virtual_rc_control(uint32_t channel_data[16])
	{
		dji_sdk::VirtualRCDataControl virtual_rc_data_control;

		for (int i = 0; i < 16; i ++) 
		{
			virtual_rc_data_control.request.channel_data[i] = channel_data[i];
		}

		return virtual_rc_data_control_service.call(virtual_rc_data_control) && virtual_rc_data_control.response.result;
	}

	bool drone_arm()
	{
		dji_sdk::DroneArmControl drone_arm_control;
		drone_arm_control.request.arm = 1;
		return drone_arm_control_service.call(drone_arm_control) && drone_arm_control.response.result;
	}

	bool drone_disarm()
	{
		dji_sdk::DroneArmControl drone_arm_control;
		drone_arm_control.request.arm = 0;
		return drone_arm_control_service.call(drone_arm_control) && drone_arm_control.response.result;
	}

	bool sync_flag_control(float frequency)
	{
		dji_sdk::SyncFlagControl sync_flag_control;
		sync_flag_control.request.frequency = frequency;
		return sync_flag_control_service.call(sync_flag_control) && sync_flag_control.response.result;
	}

	bool set_message_frequency(uint8_t frequency_data[16])
	{
		dji_sdk::MessageFrequencyControl message_frequency_control;

		for (int i = 0; i < 16; i++)
		{
			message_frequency_control.request.frequency[i] = frequency_data[i];
		}

		return message_frequency_control_service.call(message_frequency_control) && message_frequency_control.response.result;
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

	void local_position_navigation_cancel_current_goal()
	{
		local_position_navigation_action_client.cancelGoal();
	}

	void local_position_navigation_cancel_all_goals()
	{
		local_position_navigation_action_client.cancelAllGoals();
	}

	void local_position_navigation_cancel_goals_at_and_before_time(const ros::Time time)
	{
		local_position_navigation_action_client.cancelGoalsAtAndBeforeTime(time);
	}

	dji_sdk::LocalPositionNavigationResultConstPtr local_position_navigation_get_result()
	{
		return local_position_navigation_action_client.getResult();
	}

	actionlib::SimpleClientGoalState local_position_navigation_get_state()
	{
		return local_position_navigation_action_client.getState();
	}

	bool local_position_navigation_is_server_connected() 
	{
		return local_position_navigation_action_client.isServerConnected();
	}

	void local_position_navigation_send_request(float x, float y, float z, 
		LocalPositionNavigationActionClient::SimpleDoneCallback done_callback = LocalPositionNavigationActionClient::SimpleDoneCallback(), 
		LocalPositionNavigationActionClient::SimpleActiveCallback active_callback = LocalPositionNavigationActionClient::SimpleActiveCallback(), 
		LocalPositionNavigationActionClient::SimpleFeedbackCallback feedback_callback = LocalPositionNavigationActionClient::SimpleFeedbackCallback())
	{
		dji_sdk::LocalPositionNavigationGoal local_position_navigation_goal;
		local_position_navigation_goal.x = x;
		local_position_navigation_goal.y = y;
		local_position_navigation_goal.z = z;
		local_position_navigation_action_client.sendGoal(local_position_navigation_goal, done_callback, active_callback, feedback_callback);
	}

	bool local_position_navigation_wait_for_result(const ros::Duration duration = ros::Duration(0))
	{
		return local_position_navigation_action_client.waitForResult(duration);
	}

	bool local_position_navigation_stop_tracking_goal()
	{
		local_position_navigation_action_client.stopTrackingGoal();
		return true;
	}
	
	bool local_position_navigation_wait_server(const ros::Duration duration = ros::Duration(0))
	{
		return local_position_navigation_action_client.waitForServer(duration);
	}
	
	void global_position_navigation_cancel_current_goal()
	{
		global_position_navigation_action_client.cancelGoal();
	}

	void global_position_navigation_cancel_all_goals()
	{
		global_position_navigation_action_client.cancelAllGoals();
	}

	void global_position_navigation_cancel_goals_at_and_before_time(const ros::Time time)
	{
		global_position_navigation_action_client.cancelGoalsAtAndBeforeTime(time);
	}

	dji_sdk::GlobalPositionNavigationResultConstPtr global_position_navigation_get_result()
	{
		return global_position_navigation_action_client.getResult();
	}

	actionlib::SimpleClientGoalState global_position_navigation_get_state()
	{
		return global_position_navigation_action_client.getState();
	}

	bool global_position_navigation_is_server_connected() 
	{
		return global_position_navigation_action_client.isServerConnected();
	}

	void global_position_navigation_send_request(double latitude, double longitude, float altitude, 
		GlobalPositionNavigationActionClient::SimpleDoneCallback done_callback = GlobalPositionNavigationActionClient::SimpleDoneCallback(), 
		GlobalPositionNavigationActionClient::SimpleActiveCallback active_callback = GlobalPositionNavigationActionClient::SimpleActiveCallback(), 
		GlobalPositionNavigationActionClient::SimpleFeedbackCallback feedback_callback = GlobalPositionNavigationActionClient::SimpleFeedbackCallback())
	{
		dji_sdk::GlobalPositionNavigationGoal global_position_navigation_goal;
		global_position_navigation_goal.latitude = latitude;
		global_position_navigation_goal.longitude = longitude;
		global_position_navigation_goal.altitude = altitude;
		global_position_navigation_action_client.sendGoal(global_position_navigation_goal, done_callback, active_callback, feedback_callback);
	}

	bool global_position_navigation_wait_for_result (const ros::Duration duration = ros::Duration(0))
	{
		return global_position_navigation_action_client.waitForResult(duration);
	}

	bool global_position_navigation_stop_tracking_goal()
	{
		global_position_navigation_action_client.stopTrackingGoal();
		return true;
	}
	
	bool global_position_navigation_wait_server(const ros::Duration duration = ros::Duration(0))
	{
		return global_position_navigation_action_client.waitForServer(duration);
	}
	

	void waypoint_navigation_cancel_current_goal()
	{
		waypoint_navigation_action_client.cancelGoal();
	}

	void waypoint_navigation_cancel_all_goals()
	{
		waypoint_navigation_action_client.cancelAllGoals();
	}

	void waypoint_navigation_cancel_goals_at_and_before_time(const ros::Time time)
	{
		waypoint_navigation_action_client.cancelGoalsAtAndBeforeTime(time);
	}

	dji_sdk::WaypointNavigationResultConstPtr waypoint_navigation_get_result()
	{
		return waypoint_navigation_action_client.getResult();
	}

	actionlib::SimpleClientGoalState waypoint_navigation_get_state()
	{
		return waypoint_navigation_action_client.getState();
	}

	bool waypoint_navigation_is_server_connected() 
	{
		return waypoint_navigation_action_client.isServerConnected();
	}

	void waypoint_navigation_send_request(dji_sdk::WaypointList waypoint_data, 
		WaypointNavigationActionClient::SimpleDoneCallback done_callback = WaypointNavigationActionClient::SimpleDoneCallback(), 
		WaypointNavigationActionClient::SimpleActiveCallback active_callback = WaypointNavigationActionClient::SimpleActiveCallback(), 
		WaypointNavigationActionClient::SimpleFeedbackCallback feedback_callback = WaypointNavigationActionClient::SimpleFeedbackCallback())
	{
		dji_sdk::WaypointNavigationGoal waypoint_navigation_goal;
		waypoint_navigation_goal.waypoint_list = waypoint_data;
		waypoint_navigation_action_client.sendGoal(waypoint_navigation_goal, done_callback, active_callback, feedback_callback);
	}

	bool waypoint_navigation_wait_for_result(const ros::Duration duration = ros::Duration(0))
	{
		return waypoint_navigation_action_client.waitForResult(duration);
	}

	bool waypoint_navigation_stop_tracking_goal()
	{
		waypoint_navigation_action_client.stopTrackingGoal();
		return true;
	}
	
	bool waypoint_navigation_wait_server(const ros::Duration duration = ros::Duration(0))
	{
		return waypoint_navigation_action_client.waitForServer(duration);
	}

	bool mission_start()
	{
		dji_sdk::MissionStart mission_start;
		return mission_start_service.call(mission_start)&&mission_start.response.result;
	}

	bool mission_pause()
	{
		dji_sdk::MissionPause mission_pause;
		mission_pause.request.pause = 0;
		return mission_pause_service.call(mission_pause)&&mission_pause.response.result;
	}

	bool mission_resume()
	{
		dji_sdk::MissionPause mission_pause;
		mission_pause.request.pause = 1;
		return mission_pause_service.call(mission_pause)&&mission_pause.response.result;
	}

	bool mission_cancel()
	{
		dji_sdk::MissionCancel mission_cancel;
		return mission_cancel_service.call(mission_cancel)&&mission_cancel.response.result;
	}

	bool mission_waypoint_upload(dji_sdk::MissionWaypointTask waypoint_task)
	{
		dji_sdk::MissionWpUpload mission_waypoint_task;
		mission_waypoint_task.request.waypoint_task = waypoint_task;
		return mission_wp_upload_service.call(mission_waypoint_task)&&mission_waypoint_task.response.result;
	}

	dji_sdk::MissionWaypointTask mission_waypoint_download()
	{
		dji_sdk::MissionWpDownload mission_waypoint_download;
		mission_wp_download_service.call(mission_waypoint_download);
		return mission_waypoint_download.response.waypoint_task;
	}

	bool mission_waypoint_set_speed(float speed)
	{
		dji_sdk::MissionWpSetSpeed mission_waypoint_set_speed;
		mission_waypoint_set_speed.request.speed = speed;
		return mission_wp_set_speed_service.call(mission_waypoint_set_speed)&&mission_waypoint_set_speed.response.result;
	}

	float mission_waypoint_get_speed()
	{
		dji_sdk::MissionWpGetSpeed mission_waypoint_get_speed;
		mission_wp_get_speed_service.call(mission_waypoint_get_speed);
		return mission_waypoint_get_speed.response.speed;
	}

	bool mission_hotpoint_upload(dji_sdk::MissionHotpointTask hotpoint_task)
	{
		dji_sdk::MissionHpUpload mission_hotpoint_upload;
		mission_hotpoint_upload.request.hotpoint_task = hotpoint_task;
		return mission_hp_upload_service.call(mission_hotpoint_upload)&&mission_hotpoint_upload.response.result;
	}

	dji_sdk::MissionHotpointTask mission_hotpoint_download()
	{
		dji_sdk::MissionHpDownload mission_hotpoint_download;
		mission_hp_download_service.call(mission_hotpoint_download);
		return mission_hotpoint_download.response.hotpoint_task;
	}

	bool mission_hotpoint_set_speed(float speed, uint8_t direction)
	{
		dji_sdk::MissionHpSetSpeed mission_hotpoint_set_speed;
		mission_hotpoint_set_speed.request.speed = speed;
		mission_hotpoint_set_speed.request.direction = direction;
		return mission_hp_set_speed_service.call(mission_hotpoint_set_speed)&&mission_hotpoint_set_speed.response.result;
	}

	bool mission_hotpoint_set_radius(float radius)
	{
		dji_sdk::MissionHpSetRadius mission_hotpoint_set_radius;
		mission_hotpoint_set_radius.request.radius = radius;
		return mission_hp_set_radius_service.call(mission_hotpoint_set_radius)&&mission_hotpoint_set_radius.response.result;
	}

	bool mission_hotpoint_reset_yaw()
	{
		dji_sdk::MissionHpResetYaw mission_hotpoint_reset_yaw;
		return mission_hp_reset_yaw_service.call(mission_hotpoint_reset_yaw)&&mission_hotpoint_reset_yaw.response.result;
	}

	bool mission_followme_upload(dji_sdk::MissionFollowmeTask followme_task)
	{
		dji_sdk::MissionFmUpload mission_followme_task;
		mission_followme_task.request.followme_task = followme_task;
		return mission_fm_upload_service.call(mission_followme_task)&&mission_followme_task.response.result;
	}

	bool mission_followme_update_target(dji_sdk::MissionFollowmeTarget followme_target)
	{
		dji_sdk::MissionFmSetTarget mission_followme_set_target;
		mission_followme_set_target.request.followme_target = followme_target;
		return mission_fm_set_target_service.call(mission_followme_set_target)&&mission_followme_set_target.response.result;
	}

};
