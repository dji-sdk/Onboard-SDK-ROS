#include <ros/ros.h>
#include "SDK.h"
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
	printf("[f] Local Navi Test\n");
	printf("[g] Exit\n");
	printf("\ninput a/b/c etc..then press enter key\r\n");
	printf("\nuse `rostopic echo` to query drone status\r\n");
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
	ros::ServiceClient local_navigation_client = n.serviceClient<dji_ros::local_navigation>("DJI_ROS/local_navigation_service");

	dji_ros::control_manager 	srv_control;
	dji_ros::action 				srv_action;
	dji_ros::local_navigation 	srv_local_nav;

	Display_Main_Menu();
	while(1)
	{
		temp32 = getchar();
		if(temp32 != 10)
		{
			if(temp32 >= 'a' && temp32 <= 'g' && valid_flag == false)
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
				srv_control.request.control_ability=1;
				drone_control_manager.call(srv_control);
				break;
			case 'b':
				/* release control ability*/
				srv_control.request.control_ability=0;
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
				srv_action.request.action=1;
				drone_action_client.call(srv_action);
				break;
			case 'f':
				srv_local_nav.request.x = 10;
				srv_local_nav.request.y = 10;
				srv_local_nav.request.z = 10;
				local_navigation_client.call(srv_local_nav);
				break;
			case 'g':
				return 0;

			default:
				break;
		}
		main_operate_code = -1;
		err_flag = valid_flag = false;
		Display_Main_Menu();
	}
	return 0;
}
