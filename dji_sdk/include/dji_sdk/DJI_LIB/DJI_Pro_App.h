/*
 * DJI_Pro_App.cpp
 *
 *  Created on: Sep 8, 2015
 *      Author: wuyuwei
 */
#ifndef __DJI_PRO_APP_H__
#define __DJI_PRO_APP_H__

#include <stdint.h>
#include <functional>
#include "DJI_Pro_Link.h"

#define MY_DEV_ID               0x00
#define MY_ACTIVATION_SET       0x00
#define MY_CTRL_CMD_SET         0x01
#define MY_BROADCAST_CMD_SET    0x02
#define MY_MISSION_CMD_SET		0x03
#define MY_SYNC_CMD_SET			0x04
#define MY_VIRTUAL_RC_CMD_SET   0x05

// cmd_id for ACTIVATION_SET
#define API_VER_QUERY       			0x00
#define API_USER_ACTIVATION     		0x01
#define API_INFO_QUERY          		0x02
#define API_SET_FREQUENCY				0x10
#define API_TRANSPARENT_DATA_TO_MOBILE  0xFE

// cmd_id for CTRL_CMD_SET
#define API_CTRL_MANAGEMENT     		0x00
#define API_CMD_REQUEST         		0x01
#define API_CMD_STATUS_REQUEST  		0x02
#define API_CTRL_REQUEST        		0x03
#define API_CTRL_ARM					0x05
#define API_GIMBAL_CTRL_SPEED_REQUEST   0x1A
#define API_GIMBAL_CTRL_ANGLE_REQUEST   0x1B
#define API_CAMERA_SHOT         		0x20
#define API_CAMERA_VIDEO_START  		0x21
#define API_CAMERA_VIDEO_STOP   		0X22

// cmd_id for BROADCAST_CMD_SET
#define API_STD_DATA            		0x00
#define API_TRANSPARENT_DATA_TO_OBOARD  0x02
#define API_MISSION_DATA				0x03
#define API_WAYPOINT_DATA				0x04

// cmd_id for MISSION_CMD_SET
#define API_MISSION_TASK_UPLOAD		0x10
#define API_MISSION_WP_UPLOAD		0x11
#define API_MISSION_WP_START		0x12
#define API_MISSION_WP_PAUSE		0x13
#define API_MISSION_TASK_DOWNLOAD	0x14
#define API_MISSION_WP_DOWNLOAD		0x15
#define API_MISSION_SET_IDLE_SPEED	0x16
#define API_MISSION_GET_IDLE_SPEED	0x17

#define API_MISSION_HP_START    	0x20
#define API_MISSION_HP_STOP			0x21
#define API_MISSION_HP_PAUSE		0x22
#define API_MISSION_HP_SET_SPEED	0x23
#define API_MISSION_HP_SET_RADIU	0x24
#define API_MISSION_HP_RESET_YAW	0x25
#define API_MISSION_HP_DOWNLOAD		0x26

#define API_MISSION_FM_START		0x30
#define API_MISSION_FM_STOP			0x31
#define API_MISSION_FM_PAUSE		0x32
#define API_MISSION_FM_TARGET		0x33


// cmd_id for SYNC_CMD_SET
#define API_SYNC_TIMESTAMP			0x00

// cmd_id for VIRTUAL_RC_CMD_SET
#define API_VIRTUAL_RC_MANAGER  	0x00
#define API_VIRTUAL_RC_DATA	  		0x01

#define API_OPEN_SERIAL    	0x00
#define API_SIM_ECHO       	0xFF

#define HORIZ_ATT		0x00
#define HORIZ_VEL		0x40
#define HORIZ_POS		0x80

#define VERT_VEL		0x00
#define VERT_POS		0x10
#define VERT_TRU		0x20

#define YAW_ANG        	0x00
#define YAW_RATE       	0x08

#define HORIZ_GND      	0x00
#define HORIZ_BODY     	0x02

#define STABLE_FLAG_DISABLE	0x00
#define STABLE_FLAG_ENABLE 	0x01

#define MAKE_VERSION(a,b,c,d) (((a << 24)&0xff000000) | ((b << 16)&0x00ff0000) | ((c << 8)&0x0000ff00) | (d&0x000000ff))
#define SDK_VERSION           (MAKE_VERSION(3,0,10,0))

// data_type
typedef float 	fp32;
typedef double	fp64;

//----------------------------------------------------------------------
// uav std_msgs reciever
//----------------------------------------------------------------------
#define MSG_ENABLE_FLAG_LEN		2

#define ENABLE_MSG_TIME			0x0001
#define ENABLE_MSG_Q			0x0002
#define ENABLE_MSG_A			0x0004
#define ENABLE_MSG_V			0x0008
#define ENABLE_MSG_W			0x0010
#define ENABLE_MSG_POS			0x0020
#define ENABLE_MSG_MAG			0x0040
#define ENABLE_MSG_RC			0x0080
#define ENABLE_MSG_GIMBAL		0x0100
#define ENABLE_MSG_STATUS		0x0200
#define ENABLE_MSG_BATTERY		0x0400
#define ENABLE_MSG_DEVICE		0x0800

#pragma  pack(1)

/*
 *struct of gimbal contorl
 */

typedef struct
{
    signed short yaw_angle;
    signed short roll_angle;
    signed short pitch_angle;
    struct
    {
        unsigned char base : 1;//decide increment mode or absolute mode
        unsigned char yaw_cmd_ignore : 1;
        unsigned char roll_cmd_ignore : 1;
        unsigned char pitch_cmd_ignore : 1;
        unsigned char reserve : 4;
    }ctrl_byte;
    unsigned char duration;
}gimbal_custom_control_angle_t;

typedef struct
{
    signed short yaw_angle_rate;
    signed short roll_angle_rate;
    signed short pitch_angle_rate;
    struct
    {
        unsigned char reserve : 7; unsigned char ctrl_switch : 1;//enable or disable
    }ctrl_byte;
}gimbal_custom_speed_t;

/*
 *struct of quaternion data
 */

typedef struct
{
    fp32 q0;
    fp32 q1;
    fp32 q2;
    fp32 q3;
}api_quaternion_data_t;

typedef struct
{
    fp32 x;
    fp32 y;
    fp32 z;
}api_common_data_t;

/*
 *struct of vellocity data
 */

typedef struct
{
    fp32 x;
    fp32 y;
    fp32 z;
    unsigned char health_flag         :1;
    unsigned char feedback_sensor_id  :4;
    unsigned char reserve             :3;
}api_vel_data_t;

/*
 *struct of GPS data
 */
typedef struct
{
    fp64 lati;
    fp64 longti;
    fp32 alti;
    fp32 height;
    unsigned char health_flag;
}api_pos_data_t;

/*
 * struct of RC data
 */
typedef struct
{
    signed short roll;
    signed short pitch;
    signed short yaw;
    signed short throttle;
    signed short mode;
    signed short gear;
}api_rc_data_t;

/*
 * struct of compass data
 */
typedef struct
{
    signed short x;
    signed short y;
    signed short z;
}api_mag_data_t;

/*
 * struct of ctrl device data
 */
typedef struct
{
	unsigned char cur_api_ctrl_mode;
    unsigned char cur_ctrl_dev_in_navi_mode   :3;/*0->rc  1->app  2->serial*/
    unsigned char serial_req_status           :1;/*1->opensd  0->close*/
    unsigned char reserved                    :4;
}api_ctrl_info_data_t;

/*
 * struct of gimbal data 
 */
typedef struct 
{
	fp32 roll;
	fp32 pitch;
	fp32 yaw;
	unsigned char is_pitch_limit	:1;
	unsigned char is_roll_limit	:1;
	unsigned char is_yaw_limit		:1;
	unsigned char reserved			:5;
}api_gimbal_data_t;

/*
 * struct of time stamp data
 */
typedef struct 
{
	uint32_t time;
	uint32_t asr_ts;
	unsigned char sync_flag;
}api_time_stamp_t;

/*
 * struct of broadcast data
 */
typedef struct
{
    api_time_stamp_t time_stamp;
    api_quaternion_data_t q;
    api_common_data_t a;
    api_vel_data_t v;
    api_common_data_t w;
    api_pos_data_t pos;
    api_mag_data_t mag;
    api_rc_data_t rc;
    api_gimbal_data_t gimbal;
    unsigned char status;
    unsigned char battery_remaining_capacity;
    api_ctrl_info_data_t ctrl_info;
    uint8_t obtained_control;
	uint8_t activation;
}sdk_std_msg_t;

typedef struct 
{
	uint8_t mission_type;
	uint8_t target_waypoint;
	uint8_t current_state;
	uint8_t error_code;
}waypoint_mission_push_info_t;

typedef struct
{
	uint8_t mission_type;
	uint8_t mission_state;
	uint16_t hotpoint_radius;
	uint8_t error_code;
	uint8_t hotpoint_velocity;
}hotpoint_mission_push_info_t;

typedef struct
{
	uint8_t mission_type;
}followme_mission_push_info_t;

typedef struct 
{
	uint8_t mission_type;
	uint8_t last_mission_type;
	uint8_t is_broken:1;
	uint8_t reserved: 7;
	uint8_t error_code;
}other_mission_push_info_t;

typedef struct
{
	uint8_t incident_type;
	uint8_t mission_valid;
	uint16_t estimated_runtime;
}waypoint_upload_push_info_t;

typedef struct
{
	uint8_t incident_type;
	uint8_t repeat;
}waypoint_finish_action_push_info_t;

typedef struct
{
	uint8_t incident_type;
	uint8_t waypoint_index;
	uint8_t current_state;
}waypoint_reached_push_info_t;


#pragma  pack()

//----------------------------------------------------------------------
// App layer function
//----------------------------------------------------------------------
typedef struct 
{
    unsigned short	sequence_number;
    unsigned char	session_id 	: 5;
    unsigned char	need_encrypt	: 1;
    unsigned char	reserve	   	: 2;
}req_id_t;

#define DATA_MAX_SIZE 	(1000u)
#define ERR_INDEX       (0xff)
#define EXC_DATA_SIZE	(16u)
#define SET_CMD_SIZE	(2u)

//----------------------------------------------------------------------
// for cmd agency
//----------------------------------------------------------------------
#define	REQ_TIME_OUT                0x0000
#define REQ_REFUSE                  0x0001
#define CMD_RECIEVE                 0x0002
#define STATUS_CMD_EXECUTING		0x0003
#define STATUS_CMD_EXE_FAIL         0x0004
#define STATUS_CMD_EXE_SUCCESS		0x0005

/*
 *struct of cmd agency data
 */

typedef struct
{
    unsigned char cmd_sequence;
    unsigned char cmd_data;
}cmd_agency_data_t;

//----------------------------------------------------------------------
// for activation 
//----------------------------------------------------------------------

/*
 *  code re-construction according onboard api protocol
 */

#define SDK_ERR_SUCCESS                     0x0000
#define SDK_ERR_COMMAND_NOT_SUPPORTED       0xFF00
#define SDK_ERR_NO_AUTHORIZED               0xFF01
#define SDK_ERR_NO_RIGHTS                   0xFF02
#define SDK_ERR_NO_RESPONSE                 0xFFFF

#define SDK_ACTIVATE_SUCCESS                0x0000
#define SDK_ACTIVATE_PARAM_ERROR            0x0001
#define SDK_ACTIVATE_DATA_ENC_ERROR         0x0002
#define SDK_ACTIVATE_NEW_DEVICE             0x0003
#define SDK_ACTIVATE_DJI_APP_NOT_CONNECT    0x0004
#define SDK_ACTIVATE_DIJ_APP_NO_INTERNET    0x0005
#define SDK_ACTIVATE_SERVER_REFUSED         0x0006
#define SDK_ACTIVATE_LEVEL_ERROR            0x0007
#define SDK_ACTIVATE_SDK_VERSION_ERROR      0x0008

#define PARSE_STD_MSG(_flag, _enable, _data, _buf, _datalen)\
    if((_flag & _enable))\
    {\
        memcpy((unsigned char *)&(_data),(unsigned char *)(_buf)+(_datalen), sizeof(_data));\
        _datalen += sizeof(_data);\
    }

#pragma  pack(1)

/*
 *struct of activate data
 */

typedef struct
{
    unsigned int	app_id;
    unsigned int	app_api_level;
    unsigned int	app_ver;
    unsigned char	app_bundle_id[32];
    char *app_key;
}activate_data_t;

/*
 *struct of version query data
 */

typedef struct
{
    unsigned short	version_ack;
    unsigned int	version_crc;
    char     	version_name[32];
}version_query_data_t;

/*
 * struct of set frequency
 */

typedef struct
{
	unsigned char std_freq[16];
}sdk_msgs_frequency_data_t;

/*
 *struct of attitude data
 */

typedef struct
{
    unsigned char ctrl_flag;
    fp32 	roll_or_x;
    fp32	pitch_or_y;
    fp32	thr_z;
    fp32	yaw;
}attitude_data_t;

/*
 * struct of virtual rc manager
 */
typedef struct 
{
	unsigned char enable:	1; 
	unsigned char if_back_to_real: 1;
	unsigned char reserve: 	6;
}virtual_rc_manager_t;

/*
 * struct of virtual rc data
 */
typedef struct
{
	uint32_t channel_data[16];
}virtual_rc_data_t;

/*
 * struct of waypoint mission info
 */
typedef struct
{
	unsigned char length;
	fp32	vel_cmd_range;
	fp32	idle_vel;
	unsigned char action_on_finish;
	unsigned char mission_exec_times;
	unsigned char yaw_mode;
	unsigned char trace_mode;
	unsigned char action_on_rc_lost;
	unsigned char gimbal_pitch_mode;
	fp64	hp_lati;
	fp64	hp_longti;
	fp32	hp_altitude;
	unsigned char reserve[16];
}cmd_mission_wp_task_info_comm_t;


/*
 * struct of waypoint action
 */
typedef struct
{
	unsigned char action_num 	:4;
	unsigned char action_rpt	:4;
	unsigned char command_list [16];
	int16_t command_param[16];
}cmd_mission_wp_action_comm_t;

/*
 * struct of waypoint detail
 */
typedef struct
{
	fp64	latitude;
	fp64	longitude;
	fp32	altitude;
	fp32	damping_dis;
	int16_t	tgt_yaw;
	int16_t tgt_gimbal_pitch;
	unsigned char turn_mode;
	unsigned char reserve[8];
	unsigned char has_action;
	uint16_t action_time_limit;
	cmd_mission_wp_action_comm_t action;
}cmd_mission_wp_waypoint_info_comm_t;

/* 
 * struct of waypoint upload
 */
typedef struct
{
	unsigned char waypoint_index;
	cmd_mission_wp_waypoint_info_comm_t waypoint;
}cmd_mission_wp_waypoint_upload_comm_t;


/*
 * struct of hot point start
 */
typedef struct
{
	unsigned char version;
	
	fp64 latitude;
	fp64 longitude;
	fp64 altitude;
	
	fp64 radius;
	fp32 angular_rate;
	unsigned char is_clockwise;
	unsigned char start_point;
	unsigned char yaw_mode;

	unsigned char reserve[11];
}cmd_mission_hotpoint_setting_t;

/* 
 * struct of set hp vel
 */
typedef struct 
{
	unsigned char is_clockwise;
	fp32 speed;
}cmd_mission_hotpoint_velocity_t;

/*
 * struct of start fm
 */
typedef struct
{
	unsigned char mode;
	unsigned char yaw_mode;

	fp64 initial_latitude;
	fp64 initial_longitude;
	uint16_t	initial_altitude;
	uint16_t	initial_mag_angle;
	
	unsigned char sensitivity;	
}cmd_mission_follow_setting_t;

/*
 * struct of fw get target info
 */
typedef struct
{
	fp64	latitude;
	fp64	longitude;
	uint16_t altitude;
	uint16_t mag_angle;
}cmd_mission_follow_target_t;

/*
 * struct of API_MISSION broadcast
 */
typedef struct
{
	uint8_t type;
	uint8_t data_1;
	uint8_t data_2;
	uint8_t data_3;
	uint8_t data_4;
	uint8_t data_5;
}cmd_mission_common_data_t;

typedef struct 
{
	uint8_t ack;
	uint8_t index;
}cmd_mission_wp_upload_ack_t;

typedef struct
{
	uint8_t ack;
	cmd_mission_wp_task_info_comm_t wp_task;
}cmd_mission_wp_task_download_ack_t;

typedef struct
{
	uint8_t ack;
	uint8_t index;
	cmd_mission_wp_waypoint_info_comm_t wp_info;
}cmd_mission_wp_info_download_ack_t;

typedef struct
{
	uint8_t ack;
	fp32 idle_vel;
}cmd_mission_wp_velocity_ack_t;

	
typedef struct
{
	uint8_t ack;
	fp32 max_radius;
}cmd_mission_hp_start_ack_t;

typedef struct
{
	uint8_t ack;
	cmd_mission_hotpoint_setting_t hotpoint_data;
}cmd_mission_hp_download_ack_t;

	
#pragma  pack()

typedef std::function<void(unsigned short)> Command_Result_Notify;
typedef std::function<void(version_query_data_t *)> Get_API_Version_Notify;
typedef std::function<void(ProHeader *)> User_Handler_Func;
typedef std::function<void()> User_Broadcast_Handler_Func;
typedef std::function<void()> Mission_State_Handler_Func;
typedef std::function<void()> Mission_Event_Handler_Func;
typedef std::function<void(unsigned char *, unsigned char)> Transparent_Transmission_Func;

void DJI_Pro_App_Send_Data(unsigned char session_mode, unsigned char is_enc, unsigned char  cmd_set, unsigned char cmd_id,
                   unsigned char *pdata,int len,ACK_Callback_Func ack_callback, int timeout, int retry_time);
void DJI_Pro_App_Send_Ack(req_id_t req_id, unsigned char *ack, int len);

int DJI_Pro_Status_Ctrl(unsigned char cmd,Command_Result_Notify user_notice_entrance);
int DJI_Pro_Get_API_Version(Get_API_Version_Notify user_notice_entrance);
int DJI_Pro_Activate_API(activate_data_t *p_user_data,
                         Command_Result_Notify user_notice_entrance);
int DJI_Pro_Send_To_Mobile_Device(unsigned char *data,unsigned char len,
                                  Command_Result_Notify user_notice_entrance);
int DJI_Pro_Control_Management(unsigned char cmd,
                               Command_Result_Notify user_notice_entrance);
int DJI_Pro_Attitude_Control(attitude_data_t *p_user_data);
int DJI_Pro_Gimbal_Angle_Control(gimbal_custom_control_angle_t *p_user_data);
int DJI_Pro_Gimbal_Speed_Control(gimbal_custom_speed_t *p_user_data);
int DJI_Pro_Camera_Control(unsigned char camera_cmd);

int DJI_Pro_Arm_Control(unsigned char arm_cmd);
int DJI_Pro_Send_Sync_Flag(uint32_t frequency);
int DJI_Pro_Set_Msgs_Frequency(sdk_msgs_frequency_data_t *p_frequency_data);
int DJI_Pro_Virtual_RC_Manage(virtual_rc_manager_t *p_rc_manager_data);
int DJI_Pro_Virtual_RC_Send_Value(virtual_rc_data_t *p_rc_value);

int DJI_Pro_Mission_Waypoint_Upload_Task(cmd_mission_wp_task_info_comm_t *p_task_info);
int DJI_Pro_Mission_Waypoint_Upload_Waypoint(cmd_mission_wp_waypoint_upload_comm_t *p_waypoint_upload);
int DJI_Pro_Mission_Waypoint_Start(unsigned char start_cmd);
int DJI_Pro_Mission_Waypoint_Pause(unsigned char pause_cmd);
int DJI_Pro_Mission_Waypoint_Download_Task(void);
int DJI_Pro_Mission_Waypoint_Download_Waypoint(unsigned char index);
int DJI_Pro_Mission_Waypoint_Set_Idle_Speed(fp32 vel);
int DJI_Pro_Mission_Waypoint_Get_Idle_Speed(void);
int DJI_Pro_Mission_Hotpoint_Start(cmd_mission_hotpoint_setting_t *p_hotpoint_data);
int DJI_Pro_Mission_Hotpoint_Stop(void);
int DJI_Pro_Mission_Hotpoint_Pause(unsigned char pause_action);
int DJI_Pro_Mission_Hotpoint_Set_Speed(cmd_mission_hotpoint_velocity_t *p_hp_velocity);
int DJI_Pro_Mission_Hotpoint_Set_Radius(fp32 radius);
int DJI_Pro_Mission_Hotpoint_Reset_Yaw(void);
int DJI_Pro_Mission_Hotpoint_Download(void);
int DJI_Pro_Mission_Followme_Start(cmd_mission_follow_setting_t *p_follow_data);
int DJI_Pro_Mission_Followme_Stop(void);
int DJI_Pro_Mission_Followme_Pause(unsigned char pause);
int DJI_Pro_Mission_Followme_Update_Target(cmd_mission_follow_target_t *target_update);

int DJI_Pro_Get_Broadcast_Data(sdk_std_msg_t *p_user_buf, unsigned short *msg_flags);
int DJI_Pro_Get_Mission_State_Data(cmd_mission_common_data_t *p_user_buf);
int DJI_Pro_Get_Mission_Event_Data(cmd_mission_common_data_t *p_user_buf);
unsigned char DJI_Pro_Get_CmdSet_Id(ProHeader *header);
unsigned char DJI_Pro_Get_CmdCode_Id(ProHeader *header);
int DJI_Pro_Register_Transparent_Transmission_Callback(Transparent_Transmission_Func user_rec_handler_entrance);
int DJI_Pro_Register_Mission_State_Callback(Mission_State_Handler_Func mission_state_handler_entrance);
int DJI_Pro_Register_Mission_Event_Callback(Mission_Event_Handler_Func mission_event_handler_entrance);
int DJI_Pro_Register_Broadcast_Callback(User_Broadcast_Handler_Func user_broadcast_handler_entrance);
int DJI_Pro_Setup(User_Handler_Func user_cmd_handler_entrance);

#endif
