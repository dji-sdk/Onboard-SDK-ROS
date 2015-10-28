#include <ros/ros.h>
#include <dji_sdk/dji_drone.h>
#include <cstdlib>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>

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
	printf("[g] GPS Navi Test\n");
	printf("[h] Waypoint List Test\n");
	printf("[i] Exit\n");
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
	bool finished_before_timeout;

	ros::init(argc, argv, "sdk_wp_client");

	DJIDrone* drone = new DJIDrone("drone_demo");

	dji_sdk::WaypointList newWaypointList;
	dji_sdk::Waypoint waypoint0;
	dji_sdk::Waypoint waypoint1;
	dji_sdk::Waypoint waypoint2;
	dji_sdk::Waypoint waypoint3;
	dji_sdk::Waypoint waypoint4;

	Display_Main_Menu();

	while(1)
	{
		temp32 = getchar();
		if(temp32 != 10)
		{
			if(temp32 >= 'a' && temp32 <= 'i' && valid_flag == false)
			{
				main_operate_code = temp32;
				valid_flag = true;
			}
			else {
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
				drone->request_sdk_permission_control();
				break;
			case 'b':
				/* release control ability*/
				drone->release_sdk_permission_control();
				break;
			case 'c':
				/* take off */
				drone->takeoff();
				break;
			case 'd':
				/* landing*/
				drone->landing();
				break;
			case 'e':
				/* go home*/
				drone->gohome();
				break;
			case 'f':
				drone->local_position_navigation_send_request(-100, -100, 100);

				break;
			case 'g':
				drone->global_position_navigation_send_request(22.535, 113.95, 100);

				break;

			case 'h':
				{
					waypoint0.latitude = 22.535;
					waypoint0.longitude = 113.95;
					waypoint0.altitude = 100;
					waypoint0.staytime = 5;
					waypoint0.heading = 0;
				}
				newWaypointList.waypoint_list.push_back(waypoint0);

				{
					waypoint1.latitude = 22.535;
					waypoint1.longitude = 113.96;
					waypoint1.altitude = 100;
					waypoint1.staytime = 0;
					waypoint1.heading = 90;
				}
				newWaypointList.waypoint_list.push_back(waypoint1);

				{
					waypoint2.latitude = 22.545;
					waypoint2.longitude = 113.96;
					waypoint2.altitude = 100;
					waypoint2.staytime = 4;
					waypoint2.heading = -90;
				}
				newWaypointList.waypoint_list.push_back(waypoint2);

				{
					waypoint3.latitude = 22.545;
					waypoint3.longitude = 113.96;
					waypoint3.altitude = 10;
					waypoint3.staytime = 2;
					waypoint3.heading = 180;
				}
				newWaypointList.waypoint_list.push_back(waypoint3);

				{
					waypoint4.latitude = 22.525;
					waypoint4.longitude = 113.93;
					waypoint4.altitude = 50;
					waypoint4.staytime = 0;
					waypoint4.heading = -180;
				}
				newWaypointList.waypoint_list.push_back(waypoint4);

				drone->waypoint_navigation_send_request(newWaypointList);

				break;

			case 'i':
				return 0;
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
