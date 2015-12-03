/*
 * DJI_Pro_App.cpp
 *
 *  Created on: Sep 8, 2015
 *      Author: wuyuwei
 */

#include <string.h>
#include <stdio.h>
#include <pthread.h>
#include <sys/time.h>
#include <unistd.h>
#include "DJI_Pro_Link.h"
#include "DJI_Pro_App.h"

static unsigned char Pro_Encode_Data[1024];
static unsigned char Pro_Encode_ACK[10];
static sdk_std_msg_t std_broadcast_data;
static pthread_mutex_t std_msg_lock = PTHREAD_MUTEX_INITIALIZER;
static pthread_mutex_t mission_state_lock = PTHREAD_MUTEX_INITIALIZER;
static pthread_mutex_t mission_event_lock = PTHREAD_MUTEX_INITIALIZER;
static cmd_mission_common_data_t mission_state_data;
static cmd_mission_common_data_t mission_event_data;


void DJI_Pro_App_Send_Data(unsigned char session_mode, unsigned char is_enc, unsigned char  cmd_set, unsigned char cmd_id,
                   unsigned char *pdata,int len,ACK_Callback_Func ack_callback, int timeout ,int retry_time)
{
	ProSendParameter param;
	unsigned char *ptemp = (unsigned char *)Pro_Encode_Data;
	*ptemp++ = cmd_set;
	*ptemp++ = cmd_id;

	memcpy(Pro_Encode_Data + SET_CMD_SIZE,pdata,len);

	param.ack_callback = ack_callback;
    param.session_mode = session_mode;
	param.length = len + SET_CMD_SIZE;
	param.buf = Pro_Encode_Data;
    param.retry_time = retry_time;

	param.ack_timeout = timeout; 
    param.need_encrypt = is_enc;
	
	Pro_Send_Interface(&param);
}

void DJI_Pro_App_Send_Ack(req_id_t req_id, unsigned char *ack, int len)
{
	ProAckParameter param;

	memcpy(Pro_Encode_ACK,ack,len);
	
	param.session_id = req_id.session_id;
	param.seq_num = req_id.sequence_number;
	param.need_encrypt = req_id.need_encrypt;
	param.buf = Pro_Encode_ACK;
	param.length = len;
	Pro_Ack_Interface(&param);
}

static int DJI_Pro_Create_Thread(void *(* func)(void *), void *arg)
{
    pthread_t A_ARR;

    if(pthread_create(&A_ARR,0,func,arg) != 0)
    {
        return -1;
    }
    return 0;
}

/*
 *  interface: drone status control
 */

static int status_ctrl_lock = -1;
static Command_Result_Notify p_status_ctrl_interface = 0;
static unsigned short status_ctrl_return_code = SDK_ERR_NO_RESPONSE;
static cmd_agency_data_t status_ctrl_cmd_data = {0,0};

static void Save_Status_Ctrl_Return_Code(unsigned short ret_code)
{
    status_ctrl_return_code = ret_code;
}

static unsigned short Get_Status_Ctrl_Return_Code(void)
{
    return status_ctrl_return_code;
}

static void DJI_Pro_Status_Ctrl_CallBack(ProHeader *header)
{
    unsigned short ack_data;
    if(header->length - EXC_DATA_SIZE <= 2)
    {
        memcpy((unsigned char *)&ack_data,(unsigned char *)&header->magic, (header->length - EXC_DATA_SIZE));
        Save_Status_Ctrl_Return_Code(ack_data);
    }
    else
    {
        printf("%s,line %d:ERROR,ACK is exception,seesion id %d,sequence %d\n",
               __func__,__LINE__,header->session_id,header->sequence_number);
    }
}

static void * Status_Ctrl_Thread_Func(void * arg)
{
    unsigned char *cmd = (unsigned char *)arg;
    unsigned cmd_timeout = 100;    //unit is ms
    unsigned retry_time = 3;
    unsigned short ack_data = SDK_ERR_NO_RESPONSE;
    static const char err[4][16] = {{"timeout"},{"cmd refused"},{"cmd not support"},{"didn't get ack"}};

    while(1)
    {
        status_ctrl_cmd_data.cmd_sequence ++;
        status_ctrl_cmd_data.cmd_data = *cmd;
        Save_Status_Ctrl_Return_Code(SDK_ERR_NO_RESPONSE);
        DJI_Pro_App_Send_Data(2,1,MY_CTRL_CMD_SET, API_CMD_REQUEST,(unsigned char*)&status_ctrl_cmd_data,
                  sizeof(status_ctrl_cmd_data),DJI_Pro_Status_Ctrl_CallBack,cmd_timeout, retry_time);
        /* first stage poll */
        usleep(cmd_timeout * retry_time * 1000);
        ack_data = Get_Status_Ctrl_Return_Code();
        if(ack_data == SDK_ERR_NO_RESPONSE || ack_data == SDK_ERR_COMMAND_NOT_SUPPORTED
              || ack_data == REQ_TIME_OUT || ack_data == REQ_REFUSE)
        {
            printf("%s,line %d,Status Ctrl %d: %s. Return error 0x%X\n",__func__,__LINE__,
            status_ctrl_cmd_data.cmd_data, 
				&err[((ack_data&0x100)>>7)|(ack_data&0x1)][0],	
				ack_data);
            if(p_status_ctrl_interface)
            {
                p_status_ctrl_interface(ack_data);
            }
            break;
        }
        else if(ack_data == 0x0002)
        {
             /* second stage poll */
            retry_time = 8;
            while(retry_time --)
            {
                sleep(1);
                DJI_Pro_App_Send_Data(2, 1,MY_CTRL_CMD_SET, API_CMD_STATUS_REQUEST,(unsigned char*)&status_ctrl_cmd_data.cmd_sequence,
                          1,DJI_Pro_Status_Ctrl_CallBack, cmd_timeout, 1);

                usleep(cmd_timeout * 1000);
                ack_data = Get_Status_Ctrl_Return_Code();
                if(ack_data == 0x0003)
                {
                    printf("%s,line %d,Command is running\n",__func__,__LINE__);
                    continue;
                }
                else
                    break;
            }
            if(ack_data == 0x0005)
            {
                /* do some delay here to make sure drone being
                 * in final status before call user notice interface*/
                sleep(5);
                printf("%s,line %d,Status Ctrl Successfully\n",__func__,__LINE__);
            }
            else
            {
               printf("%s,line %d,Status Ctrl Failed,Return 0x%X",__func__,__LINE__,ack_data);
            }

            if(p_status_ctrl_interface)
            {
                p_status_ctrl_interface(ack_data);
            }
            break;
        }
        else
        {
            if(p_status_ctrl_interface)
            {
                p_status_ctrl_interface(ack_data);
            }
            printf("%s,line %d,Unknown ERROR,Return 0x%X",__func__,__LINE__,ack_data);
            break;
        }
    }
    status_ctrl_lock = -1;
    return (void*) NULL;
}

/*
 * cmd: 1->go home;4->take-off;6->landing
 * return:-1->parameter error or previous cmd is not finish,otherwise 0
 */

int DJI_Pro_Status_Ctrl(unsigned char cmd,Command_Result_Notify user_notice_entrance)
{
    static unsigned char cur_cmd = 0;

    if(status_ctrl_lock == 0)
    {
        return -1;
    }
    status_ctrl_lock = 0;

    if(cmd != 1 && cmd != 4 && cmd != 6)
    {
        return -1;
    }
    p_status_ctrl_interface = user_notice_entrance ? user_notice_entrance : 0;
    cur_cmd = cmd;

    if(DJI_Pro_Create_Thread(Status_Ctrl_Thread_Func,&cur_cmd) != 0)
    {
        status_ctrl_lock = 0;
        return -1;
    }
    return 0;
}


/*
 *  interface: get api version
 */
static int get_api_ver_lock = -1;
static Get_API_Version_Notify p_get_api_ver_interface = 0;
static version_query_data_t to_user_version_data;

static void DJI_Pro_Get_API_Version_CallBack(ProHeader *header)
{
    unsigned char *ptemp = (unsigned char *)&header->magic;
    char *ptemp2;
    int count = 31;
    version_query_data_t *p_version_data = &to_user_version_data;
    Get_API_Version_Notify p_temp_interface;

    p_version_data->version_ack = ptemp[0] + (ptemp[1] << 8);
    ptemp += 2;
    p_version_data->version_crc = ptemp[0] + (ptemp[1] << 8) +
                                (ptemp[2] << 16) + (ptemp[3] << 24);
    ptemp += 4;
    ptemp2 = p_version_data->version_name;
    while(*ptemp && count)
    {
        *ptemp2 ++ = (char)*ptemp ++;
        count --;
    }
    *ptemp2 = 0;

    if(p_get_api_ver_interface)
    {
        p_temp_interface = p_get_api_ver_interface;
        p_get_api_ver_interface = 0;
        p_temp_interface(&to_user_version_data);
    }
    else
    {
        printf("%s,version ack=%d\n",__func__,p_version_data->version_ack);
        printf("%s,version crc=0x%X\n",__func__,p_version_data->version_crc);
        printf("%s,version name=%s\n",__func__,p_version_data->version_name);
    }
}

static void * Get_API_Version_Thread_Func(void * arg)
{
    version_query_data_t *p_version_data = (version_query_data_t*)arg;

    unsigned cmd_timeout = 100;    //unit is ms
    unsigned retry_time = 3;
    unsigned char cmd_data = 0;

    DJI_Pro_App_Send_Data(2,0,MY_ACTIVATION_SET, API_VER_QUERY,(unsigned char*)&cmd_data,
              1,DJI_Pro_Get_API_Version_CallBack,cmd_timeout, retry_time);

    usleep((cmd_timeout + 50) * retry_time * 1000);
    /* delay more 50ms to avoid call user notice interface at the same time*/
    if(p_get_api_ver_interface)
    {
        p_get_api_ver_interface(p_version_data);
    }

    get_api_ver_lock = -1;
    return (void*)NULL;
}

int DJI_Pro_Get_API_Version(Get_API_Version_Notify user_notice_entrance)
{
    if(get_api_ver_lock == 0)
    {
        return -1;
    }
    get_api_ver_lock = 0;

    p_get_api_ver_interface = user_notice_entrance ? user_notice_entrance : 0;
    to_user_version_data.version_ack = 0xFFFF;
    to_user_version_data.version_crc = 0x0;
    to_user_version_data.version_name[0] = 0;

    if(DJI_Pro_Create_Thread(Get_API_Version_Thread_Func,&to_user_version_data) != 0)
    {
        get_api_ver_lock = -1;
        return -1;
    }
    return 0;
}

/*
 *  interface: activation interface
 */

static int activate_api_lock = -1;
static Command_Result_Notify p_activate_api_interface = 0;
static activate_data_t from_user_account_data;
static unsigned short to_user_activation_result;

static void DJI_Pro_Activate_API_CallBack(ProHeader *header)
{
    unsigned short ack_data;
    if(header->length - EXC_DATA_SIZE <= 2)
    {
        memcpy((unsigned char *)&ack_data,(unsigned char *)&header->magic, (header->length - EXC_DATA_SIZE));
        to_user_activation_result = ack_data;
        if(ack_data == SDK_ACTIVATE_NEW_DEVICE)
        {

        }
        else
        {
            if(ack_data == SDK_ACTIVATE_SUCCESS)
            {
                printf("Activation Successfully\n");

					 pthread_mutex_lock(&std_msg_lock);
					 std_broadcast_data.activation= 1;
					 pthread_mutex_unlock(&std_msg_lock);

                if(from_user_account_data.app_key)
                    Pro_Config_Comm_Encrypt_Key(from_user_account_data.app_key);
            }
            else
            {
               printf("%s,line %d,activate ERR code:0x%X\n",__func__,__LINE__,ack_data);

					pthread_mutex_lock(&std_msg_lock);
					std_broadcast_data.activation= 0;
					pthread_mutex_unlock(&std_msg_lock);

            }
            if(p_activate_api_interface)
            {
                p_activate_api_interface(ack_data);
            }
            activate_api_lock = -1;
        }

    }
    else
    {
        printf("%s,line %d:ERROR,ACK is exception,seesion id %d,sequence %d\n",
               __func__,__LINE__,header->session_id,header->sequence_number);
        activate_api_lock = -1;
    }
}

static void * Activate_API_Thread_Func(void * arg)
{
    int retry = 12;
    activate_data_t *temp_data_t = (activate_data_t *)arg;
    from_user_account_data = *temp_data_t;
    while(1)
    {
        DJI_Pro_App_Send_Data( 2, 0, MY_ACTIVATION_SET, API_USER_ACTIVATION,
                   (unsigned char*)&from_user_account_data,
                   sizeof(from_user_account_data) - sizeof(char *),
                   DJI_Pro_Activate_API_CallBack,1000,1);

        usleep(50000);
        sleep(1);
        if(to_user_activation_result == SDK_ERR_NO_RESPONSE)
        {
            printf("--- NO RESPONSE: %d ---\n",__LINE__);

				pthread_mutex_lock(&std_msg_lock);
				std_broadcast_data.activation= 2;
				pthread_mutex_unlock(&std_msg_lock);

            if(p_activate_api_interface)
            {
                p_activate_api_interface(SDK_ERR_NO_RESPONSE);
            }
            activate_api_lock = -1;
            break;
        }
        else if(to_user_activation_result == SDK_ACTIVATE_NEW_DEVICE)
        {
            if(--retry)
            {
                printf("Activate try again\n");
                sleep(1);
                continue;
            }
            else
            {
                printf("--- NO RESPONSE:%d ---\n",__LINE__);

					 pthread_mutex_lock(&std_msg_lock);
					 std_broadcast_data.activation= 2;
					 pthread_mutex_unlock(&std_msg_lock);

                if(p_activate_api_interface)
                {
                    p_activate_api_interface(SDK_ERR_NO_RESPONSE);
                }
                activate_api_lock = -1;
                break;
            }
        }
        else
        {
            break;
        }
    }
    return (void*)NULL;
}

int DJI_Pro_Activate_API(activate_data_t *p_user_data,
                         Command_Result_Notify user_notice_entrance)
{
    if(activate_api_lock == 0)
    {
        return -1;
    }
    activate_api_lock = 0;
    p_activate_api_interface = user_notice_entrance ? user_notice_entrance : 0;
    to_user_activation_result = SDK_ERR_NO_RESPONSE;
    if(DJI_Pro_Create_Thread(Activate_API_Thread_Func,p_user_data) != 0)
    {
        printf("%s,line %d,ERROR\n",__func__,__LINE__);
        activate_api_lock = -1;
        return -1;
    }
    return 0;
}

static void DJI_Pro_User_Activate_Callback(unsigned short ack)
{
    printf("%s,ack=0x%X\n",__func__,ack);
}

/*
 *  interface: transparent transmission interface
 */

static Command_Result_Notify p_transparent_data_interface = 0;

static void DJI_Pro_Send_To_Mobile_Device_CallBack(ProHeader *header)
{
    unsigned short ack_data = 0xFFFF;
    if(header->length - EXC_DATA_SIZE <= 2)
    {
        memcpy((unsigned char *)&ack_data,(unsigned char *)&header->magic, (header->length - EXC_DATA_SIZE));
        if(p_transparent_data_interface)
            p_transparent_data_interface(ack_data);
    }
    else
    {
        printf("%s,line %d:ERROR,ACK is exception,seesion id %d,sequence %d\n",
               __func__,__LINE__,header->session_id,header->sequence_number);
    }
}

int DJI_Pro_Send_To_Mobile_Device(unsigned char *data,unsigned char len,
                                  Command_Result_Notify user_notice_entrance)
{
    if(len > 100)
    {
        return -1;
    }

    p_transparent_data_interface = user_notice_entrance ? user_notice_entrance : 0;
    DJI_Pro_App_Send_Data(2, 0, MY_ACTIVATION_SET, API_TRANSPARENT_DATA_TO_MOBILE,
               data,len,DJI_Pro_Send_To_Mobile_Device_CallBack,500,2);

    return 0;
}

/*
 *  interface: request obtain control interface
 */

static Command_Result_Notify p_control_management_interface = 0;

static void DJI_Pro_Control_Management_CallBack(ProHeader *header)
{
    unsigned short ack_data = 0xFFFF;
    if(header->length - EXC_DATA_SIZE <= 2)
    {
        memcpy((unsigned char *)&ack_data,(unsigned char *)&header->magic, (header->length - EXC_DATA_SIZE));
        if(p_control_management_interface)
            p_control_management_interface(ack_data);

    }
    else
    {
        printf("%s,line %d:ERROR,ACK is exception,seesion id %d,sequence %d\n",
               __func__,__LINE__,header->session_id,header->sequence_number);
    }

    switch(ack_data)
    {
    case 0x0001:
        printf("%s,line %d, release control successfully\n",__func__,__LINE__);
			 pthread_mutex_lock(&std_msg_lock);
            std_broadcast_data.obtained_control= 0;
			 pthread_mutex_unlock(&std_msg_lock);
        break;
    case 0x0002:
        printf("%s,line %d, obtain control successfully\n",__func__,__LINE__);
			 pthread_mutex_lock(&std_msg_lock);
            std_broadcast_data.obtained_control = 1;
			 pthread_mutex_unlock(&std_msg_lock);
        break;
    case 0x0003:
        printf("%s,line %d, obtain control failed\n",__func__,__LINE__);
			 pthread_mutex_lock(&std_msg_lock);
            std_broadcast_data.obtained_control = 2;
			 pthread_mutex_unlock(&std_msg_lock);
        break;
    default:
        printf("%s,line %d, there is unkown error,ack=0x%X\n",__func__,__LINE__,ack_data);
			 pthread_mutex_lock(&std_msg_lock);
			 std_broadcast_data.obtained_control = 3;
			 pthread_mutex_unlock(&std_msg_lock);
        break;
    }
}

int DJI_Pro_Control_Management(unsigned char cmd,Command_Result_Notify user_notice_entrance)
{
    unsigned char data = cmd & 0x1;
    DJI_Pro_App_Send_Data(2,1, MY_CTRL_CMD_SET, API_CTRL_MANAGEMENT,
               &data,1,NULL,500,1);
    usleep(50000);
    p_control_management_interface = user_notice_entrance ? user_notice_entrance : 0;
    DJI_Pro_App_Send_Data(2,1, MY_CTRL_CMD_SET, API_CTRL_MANAGEMENT,
               &data,1,DJI_Pro_Control_Management_CallBack,500,1);
    return 0;
}

/*
 *  interface: attitude control interface
 */

int DJI_Pro_Attitude_Control(attitude_data_t *p_user_data)
{
    DJI_Pro_App_Send_Data(0,1, MY_CTRL_CMD_SET, API_CTRL_REQUEST,
               (unsigned char *)p_user_data,sizeof(attitude_data_t),
                  0,0,1);
    return 0;
}

/*
 *  interface: gimbal angle control interface
 */
int DJI_Pro_Gimbal_Angle_Control(gimbal_custom_control_angle_t *p_user_data)
{
    DJI_Pro_App_Send_Data(0,1, MY_CTRL_CMD_SET, API_GIMBAL_CTRL_ANGLE_REQUEST,
               (unsigned char *)p_user_data,sizeof(gimbal_custom_control_angle_t),
                  0,0,1);
    return 0;
}

/*
 *  interface: gimbal angle speed control interface
 */
int DJI_Pro_Gimbal_Speed_Control(gimbal_custom_speed_t *p_user_data)
{
    DJI_Pro_App_Send_Data(0,1, MY_CTRL_CMD_SET, API_GIMBAL_CTRL_SPEED_REQUEST,
               (unsigned char *)p_user_data,sizeof(gimbal_custom_speed_t),
                  0,0,1);
    return 0;
}

/*
 *  interface: set sdk std msgs frequency interface
 */
static void DJI_Pro_Msgs_Frequency_Set_CallBack(ProHeader *header) {
   unsigned short ack_data = 0xFFFF;
   if (header->length - EXC_DATA_SIZE<= 2) {
      memcpy((unsigned char*)&ack_data, (unsigned char*) &header -> magic, (header -> length -EXC_DATA_SIZE));
	if(ack_data == 0x00)
		printf("Set Frequency Successfully!\n");
	else if (ack_data == 0x01)
      printf("Set Frequency Failed\n");
	else 
		printf("Command Failed with Unknown error 0x%x\n", ack_data);
   }
   else
   {
	   printf("%s,line %d:ERROR,ACK is exception,seesion id %d,sequence %d\n",
			   __func__,__LINE__,header->session_id,header->sequence_number);
   }
}

int DJI_Pro_Set_Msgs_Frequency(sdk_msgs_frequency_data_t *p_frequency_data)
{
	DJI_Pro_App_Send_Data(2,1, MY_ACTIVATION_SET, API_SET_FREQUENCY,
					(unsigned char*)p_frequency_data, sizeof(sdk_msgs_frequency_data_t),
						DJI_Pro_Msgs_Frequency_Set_CallBack,500,1);
	return 0;
}


/*
 * interface: arm/disarm control interface
 */

static void DJI_Pro_Arm_Control_CallBack(ProHeader *header) {
   unsigned short ack_data = 0xFFFF;
   if (header->length - EXC_DATA_SIZE<= 2) {
      memcpy((unsigned char*)&ack_data, (unsigned char*) &header -> magic, (header -> length -EXC_DATA_SIZE));
	if(ack_data == 0x0)
		printf("Command Success!\n");
	else
      printf("Failed with error 0x%x\n", ack_data);
   }
   else
   {
	   printf("%s,line %d:ERROR,ACK is exception,seesion id %d,sequence %d\n",
			   __func__,__LINE__,header->session_id,header->sequence_number);
   }
}

int DJI_Pro_Arm_Control(unsigned char arm_cmd)
{
	DJI_Pro_App_Send_Data(2, 1, MY_CTRL_CMD_SET, API_CTRL_ARM, 
						&arm_cmd, sizeof(arm_cmd), DJI_Pro_Arm_Control_CallBack, 100, 1 );
	return 0;

}

/*
 * interface: sync timestamp interface
 */
int DJI_Pro_Send_Sync_Flag(uint32_t frequency)
{
	DJI_Pro_App_Send_Data(0, 1, MY_SYNC_CMD_SET, API_SYNC_TIMESTAMP,
						(unsigned char*)&frequency, sizeof(frequency), 0, 0, 1);
	return 0;

}

/*
 * interface: start/stop virtual rc control interface
 */

static void DJI_Pro_Virtual_RC_Manage_CallBack(ProHeader *header) {
   unsigned short ack_data = 0xFFFF;
   if (header->length - EXC_DATA_SIZE<= 2) {
	   memcpy((unsigned char*)&ack_data, (unsigned char*) &header -> magic, (header -> length -EXC_DATA_SIZE));
	   printf("ACK of virtual RC manager: %x\n", ack_data);
   }
	else
    {
        printf("%s,line %d:ERROR,ACK is exception,seesion id %d,sequence %d\n",
               __func__,__LINE__,header->session_id,header->sequence_number);
    }
}

int DJI_Pro_Virtual_RC_Manage(virtual_rc_manager_t *p_rc_manager_data)
{
	DJI_Pro_App_Send_Data(2,1, MY_VIRTUAL_RC_CMD_SET, API_VIRTUAL_RC_MANAGER,
					(unsigned char*)p_rc_manager_data, sizeof(virtual_rc_manager_t), 
						DJI_Pro_Virtual_RC_Manage_CallBack, 500, 1);
	return 0;
}

/*
 * interface: send virtual rc value interface
 */

int DJI_Pro_Virtual_RC_Send_Value(virtual_rc_data_t *p_rc_value)
{
	DJI_Pro_App_Send_Data(0,1, MY_VIRTUAL_RC_CMD_SET, API_VIRTUAL_RC_DATA,
					(unsigned char*)p_rc_value, sizeof(virtual_rc_data_t),
						0,0,1);
	return 0;
}


/* 
 * interface: init waypoint interface
 */
static void DJI_Pro_Mission_Waypoint_Upload_Task_CallBack(ProHeader *header) {
	unsigned short ack_data = 0xFF;
	if (header->length - EXC_DATA_SIZE<= 2) {
		memcpy((unsigned char*)&ack_data, (unsigned char*) &header -> magic, (header -> length -EXC_DATA_SIZE));
		printf("ACK of waypoint init: %x\n", ack_data);
	}
	else
	{
		printf("%s,line %d:ERROR,ACK is exception,seesion id %d,sequence %d\n",
				__func__,__LINE__,header->session_id,header->sequence_number);
	}

}

int DJI_Pro_Mission_Waypoint_Upload_Task(cmd_mission_wp_task_info_comm_t *p_task_info)
{
	DJI_Pro_App_Send_Data(2,1, MY_MISSION_CMD_SET, API_MISSION_TASK_UPLOAD,
			(unsigned char*)p_task_info, sizeof(cmd_mission_wp_task_info_comm_t),
			DJI_Pro_Mission_Waypoint_Upload_Task_CallBack, 500, 1);
	printf("waypoint task uploaded\n");
	return 0;
}

/* 
 * interface: upload waypoint interface
 */

static void DJI_Pro_Mission_Waypoint_Upload_Waypoint_CallBack(ProHeader *header) {
	cmd_mission_wp_upload_ack_t upload_ack;
	if (header->length - EXC_DATA_SIZE<= sizeof(cmd_mission_wp_upload_ack_t)) {
		memcpy((unsigned char*)&upload_ack, (unsigned char*) &header -> magic, (header -> length -EXC_DATA_SIZE));
		printf("ACK of waypoint upload: %x with index %d\n", upload_ack.ack, upload_ack.index);
	}
	else
	{
		printf("%s,line %d:ERROR,ACK is exception,seesion id %d,sequence %d\n",
				__func__,__LINE__,header->session_id,header->sequence_number);
	}

}

int DJI_Pro_Mission_Waypoint_Upload_Waypoint(cmd_mission_wp_waypoint_upload_comm_t *p_waypoint_upload){
	DJI_Pro_App_Send_Data(2,1, MY_MISSION_CMD_SET, API_MISSION_WP_UPLOAD,
						(unsigned char*)p_waypoint_upload, sizeof(cmd_mission_wp_waypoint_upload_comm_t),
						DJI_Pro_Mission_Waypoint_Upload_Waypoint_CallBack,500,1);
	printf("waypoint uploaded\n");
	return 0;
}
/* 
 * interface: start/stop waypoint interface
 */

static void DJI_Pro_Mission_Waypoint_Start_CallBack(ProHeader *header) {
   unsigned short ack_data = 0xFF;
   if (header->length - EXC_DATA_SIZE<= 2) {
	   memcpy((unsigned char*)&ack_data, (unsigned char*) &header -> magic, (header -> length -EXC_DATA_SIZE));
	   printf("ACK of start waypoint: %x\n", ack_data);
   }
	else
    {
        printf("%s,line %d:ERROR,ACK is exception,seesion id %d,sequence %d\n",
               __func__,__LINE__,header->session_id,header->sequence_number);
    }
}

int DJI_Pro_Mission_Waypoint_Start(unsigned char start_cmd)
{
	DJI_Pro_App_Send_Data(2,1,MY_MISSION_CMD_SET, API_MISSION_WP_START,
						&start_cmd, sizeof(start_cmd), 
						DJI_Pro_Mission_Waypoint_Start_CallBack, 100, 1);
    return 0;
}

/*
 * interface: pause/resume waypoint interface
 */
static void DJI_Pro_Mission_Waypoint_Pause_CallBack(ProHeader *header) {
   unsigned short ack_data = 0xFF;
   if (header->length - EXC_DATA_SIZE<= 2) {
	   memcpy((unsigned char*)&ack_data, (unsigned char*) &header -> magic, (header -> length -EXC_DATA_SIZE));
	   printf("ACK of pause waypoint: %x\n", ack_data);
   }
	else
    {
        printf("%s,line %d:ERROR,ACK is exception,seesion id %d,sequence %d\n",
               __func__,__LINE__,header->session_id,header->sequence_number);
    }
}

int DJI_Pro_Mission_Waypoint_Pause(unsigned char pause_cmd)
{
	DJI_Pro_App_Send_Data(2,1,MY_MISSION_CMD_SET, API_MISSION_WP_PAUSE,
						&pause_cmd, sizeof(pause_cmd), 
						DJI_Pro_Mission_Waypoint_Pause_CallBack, 100, 1);
    return 0;
}
/*
 * interface: download task info interface
 */

static void DJI_Pro_Mission_Waypoint_Download_Task_CallBack(ProHeader *header) {
	cmd_mission_wp_task_download_ack_t task_ack;
   unsigned short ack_data = 0xFF;
   if (header->length - EXC_DATA_SIZE<= sizeof(cmd_mission_wp_task_download_ack_t)) {
		memcpy((unsigned char*)&task_ack, (unsigned char*) &header -> magic, (header -> length -EXC_DATA_SIZE));
	    printf("ACK of download task: %x\n", task_ack.ack);
   }
	else
    {
        printf("%s,line %d:ERROR,ACK is exception,seesion id %d,sequence %d\n",
               __func__,__LINE__,header->session_id,header->sequence_number);
    }
}

int DJI_Pro_Mission_Waypoint_Download_Task()
{
	unsigned char temp = 1;
	DJI_Pro_App_Send_Data(2,1,MY_MISSION_CMD_SET, API_MISSION_TASK_DOWNLOAD,
						&temp, sizeof(temp), 
						DJI_Pro_Mission_Waypoint_Download_Task_CallBack, 100, 1);
    return 0;
}
/*
 * interface: download waypoint info interface
 */

static void DJI_Pro_Mission_Waypoint_Download_Waypoint_CallBack(ProHeader *header) {
	cmd_mission_wp_info_download_ack_t waypoint_ack;
   if (header->length - EXC_DATA_SIZE<= sizeof(cmd_mission_wp_info_download_ack_t)) {
	   memcpy((unsigned char*)&waypoint_ack, (unsigned char*) &header -> magic, (header -> length -EXC_DATA_SIZE));
		printf("ACK of download wp: %x\n", waypoint_ack.ack);
   }
	else
    {
        printf("%s,line %d:ERROR,ACK is exception,seesion id %d,sequence %d\n",
               __func__,__LINE__,header->session_id,header->sequence_number);
    }
}

int DJI_Pro_Mission_Waypoint_Download_Waypoint(unsigned char index)
{
	DJI_Pro_App_Send_Data(2,1,MY_MISSION_CMD_SET, API_MISSION_WP_DOWNLOAD,
						&index, sizeof(index), 
						DJI_Pro_Mission_Waypoint_Download_Waypoint_CallBack, 100, 1);
    return 0;
}
/*
 * interface: set idle speed interface
 */

static void DJI_Pro_Mission_Waypoint_Set_Idle_Speed_CallBack(ProHeader *header) {
	cmd_mission_wp_velocity_ack_t velocity_ack;
   if (header->length - EXC_DATA_SIZE<= sizeof(cmd_mission_wp_velocity_ack_t)) {
	   memcpy((unsigned char*)&velocity_ack, (unsigned char*) &header -> magic, (header -> length -EXC_DATA_SIZE));
	   printf("ACK of set speed: %x with value %f\n", velocity_ack.ack, velocity_ack.idle_vel);
   }
	else
    {
        printf("%s,line %d:ERROR,ACK is exception,seesion id %d,sequence %d\n",
               __func__,__LINE__,header->session_id,header->sequence_number);
    }
}

int DJI_Pro_Mission_Waypoint_Set_Idle_Speed(fp32 vel)
{
	DJI_Pro_App_Send_Data(2,1,MY_MISSION_CMD_SET, API_MISSION_SET_IDLE_SPEED,
						(unsigned char*)&vel, sizeof(vel), 
						DJI_Pro_Mission_Waypoint_Set_Idle_Speed_CallBack, 100, 1);
    return 0;
}
/*
 * interface: get idle speed interface
 */ 

static void DJI_Pro_Mission_Waypoint_Get_Idle_Speed_CallBack(ProHeader *header) {
	cmd_mission_wp_velocity_ack_t velocity_ack;
   if (header->length - EXC_DATA_SIZE<= sizeof(cmd_mission_wp_velocity_ack_t)) {
	   memcpy((unsigned char*)&velocity_ack, (unsigned char*) &header -> magic, (header -> length -EXC_DATA_SIZE));
		
	   printf("ACK of get speed: %x with value: %f\n", velocity_ack.ack,velocity_ack.idle_vel);
   }
	else
    {
        printf("%s,line %d:ERROR,ACK is exception,seesion id %d,sequence %d\n",
               __func__,__LINE__,header->session_id,header->sequence_number);
    }
}

int DJI_Pro_Mission_Waypoint_Get_Idle_Speed()
{
	unsigned char temp = 1;
	DJI_Pro_App_Send_Data(2,1,MY_MISSION_CMD_SET, API_MISSION_GET_IDLE_SPEED,
						&temp, sizeof(temp), 
						DJI_Pro_Mission_Waypoint_Get_Idle_Speed_CallBack, 100, 1);
    return 0;
}
/*
 * interface: start hot point interface
 */
static void DJI_Pro_Mission_Hotpoint_Start_CallBack(ProHeader *header) {
	cmd_mission_hp_start_ack_t hp_start_ack;
   if (header->length - EXC_DATA_SIZE<= sizeof(cmd_mission_hp_start_ack_t)) {
	   memcpy((unsigned char*)&hp_start_ack, (unsigned char*) &header -> magic, (header -> length -EXC_DATA_SIZE));
	   printf("ACK of start hotpoint: %x\n", hp_start_ack.ack);
   }
	else
    {
        printf("%s,line %d:ERROR,ACK is exception,seesion id %d,sequence %d\n",
               __func__,__LINE__,header->session_id,header->sequence_number);
    }
}

int DJI_Pro_Mission_Hotpoint_Start(cmd_mission_hotpoint_setting_t *p_hotpoint_data)
{
	DJI_Pro_App_Send_Data(2,1,MY_MISSION_CMD_SET, API_MISSION_HP_START,
						(unsigned char*)p_hotpoint_data, sizeof(cmd_mission_hotpoint_setting_t), 
						DJI_Pro_Mission_Hotpoint_Start_CallBack, 100, 1);
    return 0;
}
/*
 * interface: stop hot point interface 
 */ 

static void DJI_Pro_Mission_Hotpoint_Stop_CallBack(ProHeader *header) {
   unsigned short ack_data = 0xFF;
   if (header->length - EXC_DATA_SIZE<= 2) {
	   memcpy((unsigned char*)&ack_data, (unsigned char*) &header -> magic, (header -> length -EXC_DATA_SIZE));
	   printf("ACK of stop hotpoint: %x\n", ack_data);
   }
	else
    {
        printf("%s,line %d:ERROR,ACK is exception,seesion id %d,sequence %d\n",
               __func__,__LINE__,header->session_id,header->sequence_number);
    }
}

int DJI_Pro_Mission_Hotpoint_Stop()
{
	unsigned char temp = 1;
	DJI_Pro_App_Send_Data(2,1,MY_MISSION_CMD_SET, API_MISSION_HP_STOP,
						&temp, sizeof(temp), 
						DJI_Pro_Mission_Hotpoint_Stop_CallBack, 100, 1);
    return 0;
}
/*
 * interface: pause/resume hot point interface
 */

static void DJI_Pro_Mission_Hotpoint_Pause_CallBack(ProHeader *header) {
   unsigned short ack_data = 0xFF;
   if (header->length - EXC_DATA_SIZE<= 2) {
	   memcpy((unsigned char*)&ack_data, (unsigned char*) &header -> magic, (header -> length -EXC_DATA_SIZE));
	   printf("ACK of pause hotpoint: %x\n", ack_data);
   }
	else
    {
        printf("%s,line %d:ERROR,ACK is exception,seesion id %d,sequence %d\n",
               __func__,__LINE__,header->session_id,header->sequence_number);
    }
}

int DJI_Pro_Mission_Hotpoint_Pause(unsigned char pause_action)
{
	DJI_Pro_App_Send_Data(2,1,MY_MISSION_CMD_SET, API_MISSION_HP_PAUSE,
						&pause_action, sizeof(pause_action), 
						DJI_Pro_Mission_Hotpoint_Pause_CallBack, 100, 1);
    return 0;
}
/*
 * interface: set default velocity interface
 */
static void DJI_Pro_Mission_Hotpoint_Set_Speed_CallBack(ProHeader *header) {
   unsigned short ack_data = 0xFF;
   if (header->length - EXC_DATA_SIZE<= 2) {
	   memcpy((unsigned char*)&ack_data, (unsigned char*) &header -> magic, (header -> length -EXC_DATA_SIZE));
	   printf("ACK of set hp speed: %x\n", ack_data);
   }
	else
    {
        printf("%s,line %d:ERROR,ACK is exception,seesion id %d,sequence %d\n",
               __func__,__LINE__,header->session_id,header->sequence_number);
    }
}

int DJI_Pro_Mission_Hotpoint_Set_Speed(cmd_mission_hotpoint_velocity_t *p_hp_velocity)
{
	DJI_Pro_App_Send_Data(2,1,MY_MISSION_CMD_SET, API_MISSION_HP_SET_SPEED,
						(unsigned char*)p_hp_velocity, sizeof(cmd_mission_hotpoint_velocity_t), 
						DJI_Pro_Mission_Hotpoint_Set_Speed_CallBack, 100, 1);
    return 0;
}
/*
 * interface: set radius interface
 */

static void DJI_Pro_Mission_Hotpoint_Set_Radius_CallBack(ProHeader *header) {
   unsigned short ack_data = 0xFF;
   if (header->length - EXC_DATA_SIZE<= 2) {
	   memcpy((unsigned char*)&ack_data, (unsigned char*) &header -> magic, (header -> length -EXC_DATA_SIZE));
	   printf("ACK of set hp radius: %x\n", ack_data);
   }
	else
    {
        printf("%s,line %d:ERROR,ACK is exception,seesion id %d,sequence %d\n",
               __func__,__LINE__,header->session_id,header->sequence_number);
    }
}

int DJI_Pro_Mission_Hotpoint_Set_Radius(fp32 radius)
{
	DJI_Pro_App_Send_Data(2,1,MY_MISSION_CMD_SET, API_MISSION_HP_SET_RADIU,
						(unsigned char*)&radius, sizeof(radius), 
						DJI_Pro_Mission_Hotpoint_Set_Radius_CallBack, 100, 1);
    return 0;
}
/*
 * interface: reset yaw interface
 */

static void DJI_Pro_Mission_Hotpoint_Reset_Yaw_CallBack(ProHeader *header) {
   unsigned short ack_data = 0xFF;
   if (header->length - EXC_DATA_SIZE<= 2) {
	   memcpy((unsigned char*)&ack_data, (unsigned char*) &header -> magic, (header -> length -EXC_DATA_SIZE));
	   printf("ACK of set hp reset yaw: %x\n", ack_data);
   }
	else
    {
        printf("%s,line %d:ERROR,ACK is exception,seesion id %d,sequence %d\n",
               __func__,__LINE__,header->session_id,header->sequence_number);
    }
}

int DJI_Pro_Mission_Hotpoint_Reset_Yaw()
{
	unsigned char temp = 1;
	DJI_Pro_App_Send_Data(2,1,MY_MISSION_CMD_SET, API_MISSION_HP_RESET_YAW,
						&temp, sizeof(temp), 
						DJI_Pro_Mission_Hotpoint_Reset_Yaw_CallBack, 100, 1);
    return 0;
}
/*
 * interface: download hotpoint
 */

static void DJI_Pro_Mission_Hotpoint_Download_CallBack(ProHeader *header) {
	cmd_mission_hp_download_ack_t hp_download_ack;
   if (header->length - EXC_DATA_SIZE<= sizeof(cmd_mission_hp_download_ack_t)) {
	    memcpy((unsigned char*)&hp_download_ack, (unsigned char*) &header -> magic, (header -> length -EXC_DATA_SIZE));
	    printf("ACK of hp download: %x\n", hp_download_ack.ack); 
   }
	else
    {
        printf("%s,line %d:ERROR,ACK is exception,seesion id %d,sequence %d\n",
               __func__,__LINE__,header->session_id,header->sequence_number);
    }
}

int DJI_Pro_Mission_Hotpoint_Download()
{
	unsigned char temp = 1;
	DJI_Pro_App_Send_Data(2,1,MY_MISSION_CMD_SET, API_MISSION_HP_DOWNLOAD,
						&temp, sizeof(temp), 
						DJI_Pro_Mission_Hotpoint_Download_CallBack, 100, 1);
    return 0;
}
/*
 * interface: start follow me interface
 */

static void DJI_Pro_Mission_Followme_Start_CallBack(ProHeader *header) {
   unsigned short ack_data = 0xFF;
   if (header->length - EXC_DATA_SIZE<= 2) {
	   memcpy((unsigned char*)&ack_data, (unsigned char*) &header -> magic, (header -> length -EXC_DATA_SIZE));
	   printf("ACK of set followme start: %x\n", ack_data);
   }
	else
    {
        printf("%s,line %d:ERROR,ACK is exception,seesion id %d,sequence %d\n",
               __func__,__LINE__,header->session_id,header->sequence_number);
    }
}

int DJI_Pro_Mission_Followme_Start(cmd_mission_follow_setting_t *p_follow_data)
{
	DJI_Pro_App_Send_Data(2,1,MY_MISSION_CMD_SET, API_MISSION_FM_START,
						(unsigned char*)p_follow_data, sizeof(cmd_mission_follow_setting_t), 
						DJI_Pro_Mission_Followme_Start_CallBack, 100, 1);
    return 0;
}
/* 
 * interface: stop follow me interface
 */

static void DJI_Pro_Mission_Followme_Stop_CallBack(ProHeader *header) {
   unsigned short ack_data = 0xFF;
   if (header->length - EXC_DATA_SIZE<= 2) {
	   memcpy((unsigned char*)&ack_data, (unsigned char*) &header -> magic, (header -> length -EXC_DATA_SIZE));
	   printf("ACK of set followme stop: %x\n", ack_data);
   }
	else
    {
        printf("%s,line %d:ERROR,ACK is exception,seesion id %d,sequence %d\n",
               __func__,__LINE__,header->session_id,header->sequence_number);
    }
}

int DJI_Pro_Mission_Followme_Stop()
{
	unsigned char temp = 1;
	DJI_Pro_App_Send_Data(2,1,MY_MISSION_CMD_SET, API_MISSION_FM_STOP,
						&temp, sizeof(temp), 
						DJI_Pro_Mission_Followme_Stop_CallBack, 100, 1);
    return 0;
}
/*
 * interface: pause/resume follow me interface
 */

static void DJI_Pro_Mission_Followme_Pause_CallBack(ProHeader *header) {
   unsigned short ack_data = 0xFF;
   if (header->length - EXC_DATA_SIZE<= 2) {
	   memcpy((unsigned char*)&ack_data, (unsigned char*) &header -> magic, (header -> length -EXC_DATA_SIZE));
	   printf("ACK of set followme pause: %x\n", ack_data);
   }
	else
    {
        printf("%s,line %d:ERROR,ACK is exception,seesion id %d,sequence %d\n",
               __func__,__LINE__,header->session_id,header->sequence_number);
    }
}

int DJI_Pro_Mission_Followme_Pause(unsigned char pause)
{
	DJI_Pro_App_Send_Data(2,1,MY_MISSION_CMD_SET, API_MISSION_FM_PAUSE,
						&pause, sizeof(pause), 
						DJI_Pro_Mission_Followme_Pause_CallBack, 100, 1);
    return 0;
}
/*
 * interface: follow me get target info interface
 */

static void DJI_Pro_Mission_Followme_Update_Target_CallBack(ProHeader *header) {
   unsigned short ack_data = 0xFF;
   if (header->length - EXC_DATA_SIZE<= 2) {
		memcpy((unsigned char*)&ack_data, (unsigned char*) &header -> magic, (header -> length -EXC_DATA_SIZE));

		unsigned char ack;
	    memcpy((unsigned char*)&ack, (unsigned char*) &ack_data, sizeof(ack));
		cmd_mission_follow_setting_t follow_info;
		memcpy((unsigned char*)&follow_info, (unsigned char*) &ack_data, sizeof(follow_info));

		printf("ACK of set followme target: %x\n", ack_data); //TODO test
   }
	else
    {
        printf("%s,line %d:ERROR,ACK is exception,seesion id %d,sequence %d\n",
               __func__,__LINE__,header->session_id,header->sequence_number);
    }
}

int DJI_Pro_Mission_Followme_Update_Target(cmd_mission_follow_target_t *target_update)
{
	DJI_Pro_App_Send_Data(2,1,MY_MISSION_CMD_SET, API_MISSION_FM_TARGET,
						(unsigned char*)target_update, sizeof(cmd_mission_follow_target_t), 
						DJI_Pro_Mission_Followme_Update_Target_CallBack, 100, 1);
    return 0;
}

/*
 *  interface: camera control
 */
int DJI_Pro_Camera_Control(unsigned char camera_cmd)
{
    unsigned char send_data = 0;

    if(camera_cmd != API_CAMERA_SHOT && camera_cmd != API_CAMERA_VIDEO_START
            && camera_cmd != API_CAMERA_VIDEO_STOP)
    {
        printf("%s,line %d,Param ERROR\n",__func__,__LINE__);
        return -1;
    }

    DJI_Pro_App_Send_Data(0,1, MY_CTRL_CMD_SET, camera_cmd,
               (unsigned char*)&send_data,sizeof(send_data),0,0,0);

    return 0;
}

/*
 * interface: get cmd set id
 */
unsigned char DJI_Pro_Get_CmdSet_Id(ProHeader *header)
{
    unsigned char *ptemp = (unsigned char *)&header->magic;
    return *ptemp;
}

/*
 * interface: get cmd code id
 */
unsigned char DJI_Pro_Get_CmdCode_Id(ProHeader *header)
{
    unsigned char *ptemp = (unsigned char *)&header->magic;
    ptemp ++;
    return *ptemp;
}

static unsigned short std_msg_flag = 0;

/*
 * interface: get broadcast data
 */

int DJI_Pro_Get_Broadcast_Data(sdk_std_msg_t *p_user_buf, unsigned short *msg_flags) {
	pthread_mutex_lock(&std_msg_lock); 	
	*p_user_buf = std_broadcast_data; 	
	*msg_flags = std_msg_flag;
	pthread_mutex_unlock(&std_msg_lock); 	
	return 0; 	
}

int DJI_Pro_Get_Mission_State_Data(cmd_mission_common_data_t *p_user_buf)
{
	pthread_mutex_lock(&mission_state_lock);
	*p_user_buf = mission_state_data;
	pthread_mutex_unlock(&mission_state_lock);
	return 0;
}

int DJI_Pro_Get_Mission_Event_Data(cmd_mission_common_data_t *p_user_buf)
{
	pthread_mutex_lock(&mission_event_lock);
	*p_user_buf = mission_event_data;
	pthread_mutex_unlock(&mission_event_lock);
	return 0;
}
	

static User_Broadcast_Handler_Func p_user_broadcast_handler_func = 0;
static Mission_State_Handler_Func p_mission_state_handler_func = 0;
static Mission_Event_Handler_Func p_mission_event_handler_func = 0;
static User_Handler_Func p_user_handler_func = 0;
static Transparent_Transmission_Func p_user_rec_func = 0;

static void DJI_Pro_Parse_Broadcast_Data(ProHeader *header)
{
    unsigned char *pdata = (unsigned char *)&header->magic;
    unsigned short *msg_enable_flag;
    unsigned short data_len = MSG_ENABLE_FLAG_LEN;
    pthread_mutex_lock(&std_msg_lock);
    pdata += 2;
    msg_enable_flag = (unsigned short *)pdata;
    std_msg_flag = *msg_enable_flag;
    PARSE_STD_MSG( *msg_enable_flag, ENABLE_MSG_TIME	,std_broadcast_data.time_stamp      , pdata, data_len);
    PARSE_STD_MSG( *msg_enable_flag, ENABLE_MSG_Q		,std_broadcast_data.q				, pdata, data_len);
    PARSE_STD_MSG( *msg_enable_flag, ENABLE_MSG_A		,std_broadcast_data.a				, pdata, data_len);
    PARSE_STD_MSG( *msg_enable_flag, ENABLE_MSG_V		,std_broadcast_data.v				, pdata, data_len);
    PARSE_STD_MSG( *msg_enable_flag, ENABLE_MSG_W		,std_broadcast_data.w				, pdata, data_len);
    PARSE_STD_MSG( *msg_enable_flag, ENABLE_MSG_POS     ,std_broadcast_data.pos				, pdata, data_len);
    PARSE_STD_MSG( *msg_enable_flag, ENABLE_MSG_MAG     ,std_broadcast_data.mag				, pdata, data_len);
    PARSE_STD_MSG( *msg_enable_flag, ENABLE_MSG_RC		,std_broadcast_data.rc				, pdata, data_len);
    PARSE_STD_MSG( *msg_enable_flag, ENABLE_MSG_GIMBAL	,std_broadcast_data.gimbal			, pdata, data_len);
    PARSE_STD_MSG( *msg_enable_flag, ENABLE_MSG_STATUS	,std_broadcast_data.status			, pdata, data_len);
    PARSE_STD_MSG( *msg_enable_flag, ENABLE_MSG_BATTERY ,std_broadcast_data.battery_remaining_capacity	, pdata, data_len);
    PARSE_STD_MSG( *msg_enable_flag, ENABLE_MSG_DEVICE	,std_broadcast_data.ctrl_info			, pdata, data_len);
    pthread_mutex_unlock(&std_msg_lock);

	if (p_user_broadcast_handler_func)
		p_user_broadcast_handler_func();
}

/*
 * interface: protocol initialization
 */
static void DJI_Pro_App_Recv_Req_Data(ProHeader *header)
{
    unsigned char buf[100] = {0,0};
    unsigned char len = 0;
    switch(header->session_id)
    {
    case 0:
        if(DJI_Pro_Get_CmdSet_Id(header) == MY_BROADCAST_CMD_SET
                && DJI_Pro_Get_CmdCode_Id(header) == API_STD_DATA)
        {
            DJI_Pro_Parse_Broadcast_Data(header);
        }
        else if(DJI_Pro_Get_CmdSet_Id(header) == MY_BROADCAST_CMD_SET
                && DJI_Pro_Get_CmdCode_Id(header) == API_TRANSPARENT_DATA_TO_OBOARD)
        {
            if(p_user_rec_func)
            {
                len = (header->length - EXC_DATA_SIZE -2) > 100 ? 100 :
                          (header->length - EXC_DATA_SIZE -2);
                memcpy(buf,(unsigned char*)&header->magic + 2,len);
                p_user_rec_func(buf,len);
            }
        }
		else if (DJI_Pro_Get_CmdSet_Id(header) == MY_BROADCAST_CMD_SET
				&& DJI_Pro_Get_CmdCode_Id(header) == API_MISSION_DATA)
		{
			pthread_mutex_lock(&mission_state_lock);
			memcpy((unsigned char*)&mission_state_data, (unsigned char*)header->magic,(header->length - EXC_DATA_SIZE));
			pthread_mutex_unlock(&mission_state_lock);
			if (p_mission_state_handler_func)
				p_mission_state_handler_func();
		}
	
		else if (DJI_Pro_Get_CmdSet_Id(header) == MY_BROADCAST_CMD_SET
				&& DJI_Pro_Get_CmdCode_Id(header) == API_WAYPOINT_DATA)
		{
			pthread_mutex_lock(&mission_event_lock);
			memcpy((unsigned char*)&mission_event_data, (unsigned char*)header->magic,(header->length - EXC_DATA_SIZE));
			pthread_mutex_unlock(&mission_event_lock);
			if (p_mission_event_handler_func)
				p_mission_event_handler_func();
		}

        else
        {
            if(p_user_handler_func)
            {
                p_user_handler_func(header);
            }
        }
        break;
    case 1:
    case 2:
        if(p_user_handler_func)
        {
            p_user_handler_func(header);
        }
        else
        {
            ProAckParameter param;
            printf("%s:Recv request,session id=%d,seq_num=%d\n",
                    __func__,header->session_id,header->sequence_number);
            if(header->session_id > 0)
            {
                buf[0] = buf[1] = 0;
                param.session_id = header->session_id;
                param.seq_num = header->sequence_number;
                param.need_encrypt = header->enc_type;
                param.buf = buf;
                param.length = 2;
                Pro_Ack_Interface(&param);
            }
        }
        break;
    }
}

int DJI_Pro_Register_Transparent_Transmission_Callback(Transparent_Transmission_Func user_rec_handler_entrance)
{
    p_user_rec_func = user_rec_handler_entrance;
    return 0;
}

int DJI_Pro_Register_Broadcast_Callback(User_Broadcast_Handler_Func user_broadcast_handler_entrance)
{
	p_user_broadcast_handler_func = user_broadcast_handler_entrance;
	return 0;
}

int DJI_Pro_Register_Mission_State_Callback(Mission_State_Handler_Func mission_state_handler_entrance)
{
	p_mission_state_handler_func = mission_state_handler_entrance;
	return 0;
}

int DJI_Pro_Register_Mission_Event_Callback(Mission_Event_Handler_Func mission_event_handler_entrance)
{
	p_mission_event_handler_func = mission_event_handler_entrance;
	return 0;
}

int DJI_Pro_Setup(User_Handler_Func user_cmd_handler_entrance)
{
    Pro_Link_Setup();
    Pro_App_Recv_Set_Hook(DJI_Pro_App_Recv_Req_Data);
    p_user_handler_func = user_cmd_handler_entrance ? user_cmd_handler_entrance : 0;
    return 0;
}
