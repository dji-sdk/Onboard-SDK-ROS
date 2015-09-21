#include <ros/ros.h>
#include "dji_services.h"
#include <cstdlib>
static void Display_Main_Menu(void)
{
	printf("\r\n");
	printf("----------- < Main menu > ----------\r\n\r\n");
	printf("[a] Request to obtain control\n");
	printf("[b] Release control\n");
	printf("[c] Takeoff\n");
	printf("[d] Landing\n");
	printf("[e] Go home\n");
	printf("[f] Attitude control sample\n");
	printf("[g] Gimbal control sample\n");
	printf("\ninput a/b/c etc..then press enter key\r\n");
	printf("----------------------------------------\r\n");
	printf("input: ");
}
int main(int argc, char **argv)
{
	int main_operate_code = 0;
	int temp32;
	bool valid_flag = false;
	bool err_flag = false;
	ros::init(argc, argv, "sdk_client");
	ROS_INFO("sdk_service_client_test");
	ros::NodeHandle n;
	ros::ServiceClient drone_control_manager = n.serviceClient<dji_ros::control_manager>("DJI_ROS/obtain_release_control");
	ros::ServiceClient drone_action_client = n.serviceClient<dji_ros::action>("DJI_ROS/drone_action_control");
	ros::ServiceClient drone_attitude_client= n.serviceClient<dji_ros::attitude>("DJI_ROS/drone_attitude_control");
	ros::ServiceClient gimbal_angle_client= n.serviceClient<dji_ros::gimbal_angle>("DJI_ROS/gimbal_angle_control");
	ros::ServiceClient gimbal_speed_client= n.serviceClient<dji_ros::gimbal_speed>("DJI_ROS/gimbal_speed_control");
	dji_ros::control_manager 	srv_control;
	dji_ros::action 				srv_action;
	dji_ros::attitude 			srv_attitude;
	dji_ros::gimbal_angle 		srv_gimbal_angle;
	dji_ros::gimbal_speed 		srv_gimbal_speed;
	Display_Main_Menu();
	while(1)
	{
		temp32 = getchar();
		if(temp32 != 10)
		{
			if(temp32 >= 'a' && temp32 <= 'j' && valid_flag == false)
			{
				main_operate_code = temp32;
				valid_flag = true;
			}
			else
			{
				err_flag = true;
			}
			continue;
		}
		else
		{
			if(err_flag == true)
			{
				printf("input: ERROR\n");
				Display_Main_Menu();
				err_flag = valid_flag = false;
				continue;
			}
		}
		switch(main_operate_code)
		{
			case 'a':
				/* request control ability*/
				srv_control.request.action=1;
				drone_control_manager.call(srv_control);
				break;
			case 'b':
				/* release control ability*/
				srv_control.request.action=0;
				drone_control_manager.call(srv_control);
				break;
			case 'c':
				/* take off */
				srv_action.request.action=4;
				drone_action_client.call(srv_action);
				break;
			case 'd':
				/* landing*/
				srv_action.request.action=6;
				drone_action_client.call(srv_action);
				break;
			case 'e':
				/* go home*/
				srv_action.request.action=4;
				drone_action_client.call(srv_action);
				break;
			case 'f':
				/* attitude control sample
				srv_tgc.request.flag=1;
				srv_tgc.request.yaw=1000;
				srv_tgc.request.x=0;
				srv_tgc.request.y=0;
				test_gimbal_control_client.call(srv_tgc);
*/
				break;
			case 'g':
				/* gimbal control sample
				srv_tgc.request.flag=2;
				srv_tgc.request.yaw=0;
				srv_tgc.request.x=0;
				srv_tgc.request.y=-1000;
				test_gimbal_control_client.call(srv_tgc);
*/
				break;
			default:
				break;
		}
		main_operate_code = -1;
		err_flag = valid_flag = false;
		Display_Main_Menu();
	}
	return 0;
}
