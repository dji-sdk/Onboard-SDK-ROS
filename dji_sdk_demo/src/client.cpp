#include <ros/ros.h>
#include <dji_sdk/dji_drone.h>
#include <cstdlib>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>

static void Display_Main_Menu(void)
{
    printf("\r\n");
    printf("----------- < Main menu > ----------\r\n\r\n");
    printf("[a] Exit\n");
    printf("[b] SDK Version Query\n");
    printf("[c] Request to obtain control\n");
    printf("[d] Release control\n");
    printf("[e] Takeoff\n");
    printf("[f] Landing\n");
    printf("[g] Go home\n");
    printf("[h] Gimbal control sample\n");
    printf("[i] Attitude control sample\n");
    printf("[j] Draw circle sample\n");
    printf("[k] Draw square sample\n");
    printf("[l] Take a picture\n");
    printf("[m] Start video\n");
    printf("[n] Stop video\n");
    printf("[o] Local Navi Test\n");
    printf("[p] GPS Navi Test\n");
    printf("[q] Waypoint List Test\n");
    printf("[r] Drone Arm\n");
    printf("[s] Drone Disarm\n");
    printf("[t] Virtual RC Test\n");
    printf("[u] Sync Flag Test\n");
    printf("[v] Set Msg Frequency Test\n");
    printf("[w] Mission Start\n");
    printf("[x] Mission Pause\n");
    printf("[y] Mission Resume\n");
    printf("[z] Mission Cancel\n");
    printf("[1] Mission Download\n");
    printf("[2] Mission Wp Upload\n");
    printf("[3] Mission Hp Upload\n");
    printf("[4] Mission Fm Upload\n");
    printf("[5] Mission Wp Set Speed\n");
    printf("[6] Mission Wp Get Speed\n");
    printf("[7] Mission Hp Set Speed\n");
    printf("[8] Mission Hp Set Radius\n");
    printf("[9] Mission Hp Reset Yaw\n");
    printf("[0] Mission Fm Set Target\n");
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
    ros::NodeHandle nh;
    DJIDrone* drone = new DJIDrone(nh);

	//virtual RC test data
	uint32_t virtual_rc_data[16];
	//set frequency test data
	uint8_t  msg_frequency_data[16] = {2};
	//waypoint action test data
    dji_sdk::WaypointList newWaypointList;
    dji_sdk::Waypoint waypoint0;
    dji_sdk::Waypoint waypoint1;
    dji_sdk::Waypoint waypoint2;
    dji_sdk::Waypoint waypoint3;
    dji_sdk::Waypoint waypoint4;

	//groundstation test data
	dji_sdk::MissionWaypointTask waypoint_task;
	dji_sdk::MissionWaypoint 	 waypoint;
	dji_sdk::MissionHotpointTask hotpoint_task;
	dji_sdk::MissionFollowmeTask followme_task;
	dji_sdk::MissionFollowmeTarget followme_target;
	
    Display_Main_Menu();
    while(1)
    {
        temp32 = getchar();
        if(temp32 != 10)
        {
            if(valid_flag == false)
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
				//drone->activate();
				return 0;
			case 'b':
				/* SDK version query*/
				drone->check_version();
				break;
            case 'c':
                /* request control ability*/
                drone->request_sdk_permission_control();
                break;
            case 'd':
                /* release control ability*/
                drone->release_sdk_permission_control();
                break;
            case 'e':
                /* take off */
                drone->takeoff();
                break;
            case 'f':
                /* landing*/
                drone->landing();
                break;
            case 'g':
                /* go home*/
                drone->gohome();
                break;
            case 'h':
                /*gimbal test*/

                drone->gimbal_angle_control(0, 0, 1800, 20);
                sleep(2);
                drone->gimbal_angle_control(0, 0, -1800, 20);
                sleep(2);
                drone->gimbal_angle_control(300, 0, 0, 20);
                sleep(2);
                drone->gimbal_angle_control(-300, 0, 0, 20);
                sleep(2);
                drone->gimbal_angle_control(0, 300, 0, 20);
                sleep(2);
                drone->gimbal_angle_control(0, -300, 0, 20);
                sleep(2);
                drone->gimbal_speed_control(100, 0, 0);
                sleep(2);
                drone->gimbal_speed_control(-100, 0, 0);
                sleep(2);
                drone->gimbal_speed_control(0, 0, 200);
                sleep(2);
                drone->gimbal_speed_control(0, 0, -200);
                sleep(2);
                drone->gimbal_speed_control(0, 200, 0);
                sleep(2);
                drone->gimbal_speed_control(0, -200, 0);
                sleep(2);
                drone->gimbal_angle_control(0, 0, 0, 20);
                break;

            case 'i':
                /* attitude control sample*/
                drone->takeoff();
                sleep(8);


                for(int i = 0; i < 100; i ++)
                {
                    if(i < 90)
                        drone->attitude_control(0x40, 0, 2, 0, 0);
                    else
                        drone->attitude_control(0x40, 0, 0, 0, 0);
                    usleep(20000);
                }
                sleep(1);

                for(int i = 0; i < 200; i ++)
                {
                    if(i < 180)
                        drone->attitude_control(0x40, 2, 0, 0, 0);
                    else
                        drone->attitude_control(0x40, 0, 0, 0, 0);
                    usleep(20000);
                }
                sleep(1);

                for(int i = 0; i < 200; i ++)
                {
                    if(i < 180)
                        drone->attitude_control(0x40, -2, 0, 0, 0);
                    else
                        drone->attitude_control(0x40, 0, 0, 0, 0);
                    usleep(20000);
                }
                sleep(1);

                for(int i = 0; i < 200; i ++)
                {
                    if(i < 180)
                        drone->attitude_control(0x40, 0, 2, 0, 0);
                    else
                        drone->attitude_control(0x40, 0, 0, 0, 0);
                    usleep(20000);
                }
                sleep(1);

                for(int i = 0; i < 200; i ++)
                {
                    if(i < 180)
                        drone->attitude_control(0x40, 0, -2, 0, 0);
                    else
                        drone->attitude_control(0x40, 0, 0, 0, 0);
                    usleep(20000);
                }
                sleep(1);

                for(int i = 0; i < 200; i ++)
                {
                    if(i < 180)
                        drone->attitude_control(0x40, 0, 0, 0.5, 0);
                    else
                        drone->attitude_control(0x40, 0, 0, 0, 0);
                    usleep(20000);
                }
                sleep(1);

                for(int i = 0; i < 200; i ++)
                {
                    if(i < 180)
                        drone->attitude_control(0x40, 0, 0, -0.5, 0);
                    else
                        drone->attitude_control(0x40, 0, 0, 0, 0);
                    usleep(20000);
                }
                sleep(1);

                for(int i = 0; i < 200; i ++)
                {
                    if(i < 180)
                        drone->attitude_control(0xA, 0, 0, 0, 90);
                    else
                        drone->attitude_control(0xA, 0, 0, 0, 0);
                    usleep(20000);
                }
                sleep(1);

                for(int i = 0; i < 200; i ++)
                {
                    if(i < 180)
                        drone->attitude_control(0xA, 0, 0, 0, -90);
                    else
                        drone->attitude_control(0xA, 0, 0, 0, 0);
                    usleep(20000);
                }
                sleep(1);

                drone->landing();

                break;

            case 'j':
                /*draw circle sample*/
                static float time = 0;
                static float R = 2;
                static float V = 2;
                static float vx;
                static float vy;
                /* start to draw circle */
                for(int i = 0; i < 300; i ++)
                {
                    vx = V * sin((V/R)*time/50.0f);
                    vy = V * cos((V/R)*time/50.0f);
        
                    drone->attitude_control(HORIZ_POS|VERT_VEL|YAW_ANG|HORIZ_BODY|STABLE_FLAG_ENABLE, vx, vy, 0, 0);
                    usleep(20000);
                    time++;
                }
                break;

            case 'k':
                /*draw square sample*/
                for(int i = 0;i < 60;i++)
                {
                    drone->attitude_control(HORIZ_POS|VERT_VEL|YAW_ANG|HORIZ_BODY|STABLE_FLAG_ENABLE, 3, 3, 0, 0);
                    usleep(20000);
                }
                for(int i = 0;i < 60;i++)
                {
                    drone->attitude_control(HORIZ_POS|VERT_VEL|YAW_ANG|HORIZ_BODY|STABLE_FLAG_ENABLE, -3, 3, 0, 0);
                    usleep(20000);
                }
                for(int i = 0;i < 60;i++)
                {
                    drone->attitude_control(HORIZ_POS|VERT_VEL|YAW_ANG|HORIZ_BODY|STABLE_FLAG_ENABLE, -3, -3, 0, 0);
                    usleep(20000);
                }
                for(int i = 0;i < 60;i++)
                {
                    drone->attitude_control(HORIZ_POS|VERT_VEL|YAW_ANG|HORIZ_BODY|STABLE_FLAG_ENABLE, 3, -3, 0, 0);
                    usleep(20000);
                }
                break;
            case 'l':
                /*take a picture*/
                drone->take_picture();
                break;
            case 'm':
                /*start video*/
                drone->start_video();
                break;
            case 'n':
                /*stop video*/
                drone->stop_video();
                break;
            case 'o':
                /* Local Navi Test */
                drone->local_position_navigation_send_request(-100, -100, 100);
                break;
            case 'p':
                /* GPS Navi Test */
                drone->global_position_navigation_send_request(22.535, 113.95, 100);
                break;
            case 'q':
                /* Waypoint List Navi Test */
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
			case 'r':
				//drone arm
				drone->drone_arm();
                break;
			case 's':
				//drone disarm
				drone->drone_disarm();
                break;
			case 't':
				//virtual rc test 1: arm & disarm
				drone->virtual_rc_enable();
				virtual_rc_data[0] = 1024+600;	//0-> roll     	[1024-600,1024+600] 
				virtual_rc_data[1] = 1024-600;	//1-> pitch    	[1024-600,1024+600]
				virtual_rc_data[2] = 1024-600;	//2-> throttle 	[1024-600,1024+600]
				virtual_rc_data[3] = 1024-600;	//3-> yaw      	[1024-600,1024+600]
				virtual_rc_data[6] = 496;    	//6-> mode     	{1552(P), 1024(A), 496(F)}
				virtual_rc_data[4] = 1684;	 	//4-> gear		{1684(UP), 1324(DOWN)}

				for(int i = 0; i < 10; i++) {
					drone->virtual_rc_control(virtual_rc_data);
					usleep(20000);
				}

				//virtual rc test 2: yaw 
				drone->virtual_rc_enable();
				virtual_rc_data[0] = 1024;		//0-> roll     	[1024-600,1024+600] 
				virtual_rc_data[1] = 1024;		//1-> pitch    	[1024-600,1024+600]
				virtual_rc_data[2] = 1024;		//2-> throttle 	[1024-600,1024+600]
				virtual_rc_data[3] = 1024-600;	//3-> yaw      	[1024-600,1024+600]
				virtual_rc_data[6] = 496;    	//6-> mode     	{1552(P), 1024(A), 496(F)}
				virtual_rc_data[4] = 1684;	 	//4-> gear		{1684(UP), 1324(DOWN)}

				for(int i = 0; i < 10; i++) {
					drone->virtual_rc_control(virtual_rc_data);
					usleep(20000);
				}
				break;
			case 'u':
				//sync flag
				drone->sync_flag_control(1);
				break;
			case 'v':
				//set msg frequency
				drone->set_message_frequency(msg_frequency_data);
				break;
			case 'w':
				//mission start
				drone->mission_start();
				break;
			case 'x':
				//mission pause
				drone->mission_pause();
				break;
			case 'y':
				//mission resume
				drone->mission_resume();
				break;
			case 'z':
				//mission cancel
				drone->mission_cancel();
				break;
			case '1':
				//mission download
				drone->mission_download();
				break;
			case '2':
				//mission waypoint upload
				waypoint_task.velocity_range = 15;
				waypoint_task.idle_velocity = 10;
				waypoint_task.action_on_finish = 0;
				waypoint_task.mission_exec_times = 2;
				waypoint_task.yaw_mode = 1;
				waypoint_task.trace_mode = 0;
				waypoint_task.action_on_rc_lost = 0;
				waypoint_task.gimbal_pitch_mode = 0;

				waypoint.latitude = 22.540091;
				waypoint.longitude = 113.946593;
				waypoint.altitude = 100;
				waypoint.damping_distance = 2;
				waypoint.target_yaw = 0;
				waypoint.target_gimbal_pitch = 0;
				waypoint.turn_mode = 0;
				waypoint.has_action = 1;
				waypoint.action_time_limit = 10;
				waypoint.waypoint_action.action_repeat = 1;
				waypoint.waypoint_action.command_list[0] = 1;
				waypoint.waypoint_action.command_parameter[0] = 1;

				waypoint_task.mission_waypoint.push_back(waypoint);

				waypoint.latitude = 22.540015;
				waypoint.longitude = 113.94659;
				waypoint.altitude = 120;
				waypoint.damping_distance = 2;
				waypoint.target_yaw = 180;
				waypoint.target_gimbal_pitch = 0;
				waypoint.turn_mode = 0;
				waypoint.has_action = 1;
				waypoint.action_time_limit = 10;
				waypoint.waypoint_action.action_repeat = 1;
				waypoint.waypoint_action.command_list[0] = 1;
				waypoint.waypoint_action.command_parameter[0] = 1;

				waypoint_task.mission_waypoint.push_back(waypoint);

				drone->mission_waypoint_upload(waypoint_task);
				break;
			case '3':
				//mission hotpoint upload
				hotpoint_task.latitude = 22.540091;
				hotpoint_task.longitude = 113.946593;
				hotpoint_task.altitude = 20;
				hotpoint_task.radius = 10;
				hotpoint_task.angular_speed = 10;
				hotpoint_task.is_clockwise = 0;
				hotpoint_task.start_point = 0;
				hotpoint_task.yaw_mode = 0;

				drone->mission_hotpoint_upload(hotpoint_task);
				break;
			case '4':
				//mission followme upload
				followme_task.mode = 0;
				followme_task.yaw_mode = 0;
				followme_task.initial_latitude = 23.540091;
				followme_task.initial_longitude = 113.946593;
				followme_task.initial_altitude = 10;
				followme_task.sensitivity = 1;

				drone->mission_followme_upload(followme_task);
				break;
			case '5':
				//mission waypoint set speed
				drone->mission_waypoint_set_speed((float)5);
				break;
			case '6':
				//mission waypoint get speed
				printf("%f", drone->mission_waypoint_get_speed());
				break;
			case '7':
				//mission hotpoint set speed
				drone->mission_hotpoint_set_speed((float)5,(uint8_t)1);
				break;
			case '8':
				//mission hotpoint set radius
				drone->mission_hotpoint_set_radius((float)5);
				break;
			case '9':
				//mission hotpoint reset yaw
				drone->mission_hotpoint_reset_yaw();
				break;
			case '0':
				//mission followme update target
				for (int i = 0; i < 20; i++)
				{
					followme_target.latitude = 22.540091 + i*0.000001;
					followme_target.longitude = 113.946593 + i*0.000001;
					followme_target.altitude = 100;
					drone->mission_followme_update_target(followme_target);
					usleep(20000);
				}
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
