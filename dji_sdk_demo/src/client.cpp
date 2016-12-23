/** @file client.cpp
 *  @version 3.1.8
 *  @date July 29th, 2016
 *
 *  @brief
 *  All the exampls for ROS are implemented here. 
 *
 *  @copyright 2016 DJI. All rights reserved.
 *
 */



#include <ros/ros.h>
#include <stdio.h>
#include <dji_sdk/dji_drone.h>
#include <cstdlib>
#include <stdlib.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>

using namespace DJI::onboardSDK;

//! Function Prototypes for Mobile command callbacks - Core Functions
void ObtainControlMobileCallback(DJIDrone *drone);
void ReleaseControlMobileCallback(DJIDrone *drone);
void TakeOffMobileCallback(DJIDrone *drone);
void LandingMobileCallback(DJIDrone *drone);
void GetSDKVersionMobileCallback(DJIDrone *drone);
void ArmMobileCallback(DJIDrone *drone);
void DisarmMobileCallback(DJIDrone *drone);
void GoHomeMobileCallback(DJIDrone *drone);
void TakePhotoMobileCallback(DJIDrone *drone);
void StartVideoMobileCallback(DJIDrone *drone);
void StopVideoMobileCallback(DJIDrone *drone);
//! Function Prototypes for Mobile command callbacks - Custom Missions
void DrawCircleDemoMobileCallback(DJIDrone *drone);
void DrawSquareDemoMobileCallback(DJIDrone *drone);
void GimbalControlDemoMobileCallback(DJIDrone *drone);
void AttitudeControlDemoMobileCallback(DJIDrone *drone);
void LocalNavigationTestMobileCallback(DJIDrone *drone);
void GlobalNavigationTestMobileCallback(DJIDrone *drone);
void WaypointNavigationTestMobileCallback(DJIDrone *drone);
void VirtuaRCTestMobileCallback(DJIDrone *drone);

//! For LAS logging
void StartMapLASLoggingMobileCallback(DJIDrone *drone);
void StopMapLASLoggingMobileCallback(DJIDrone *drone);
void StartCollisionAvoidanceCallback(DJIDrone *drone);
void StopCollisionAvoidanceCallback(DJIDrone *drone);


static void Display_Main_Menu(void)
{
    printf("\r\n");
    printf("+-------------------------- < Main menu > ------------------------+\n");
	printf("| [1]  SDK Version Query        | [20] Set Sync Flag Test          |\n");
	printf("| [2]  Request Control          | [21] Set Msg Frequency Test      |\n");	
	printf("| [3]  Release Control          | [22] Waypoint Mission Upload     |\n");	
	printf("| [4]  Takeoff                  | [23] Hotpoint Mission Upload     |\n");	
	printf("| [5]  Landing                  | [24] Followme Mission Upload     |\n");	
	printf("| [6]  Go Home                  | [25] Mission Start               |\n");	
	printf("| [7]  Gimbal Control Sample    | [26] Mission Pause               |\n");	
	printf("| [8]  Attitude Control Sample  | [27] Mission Resume              |\n");	
	printf("| [9]  Draw Circle Sample       | [28] Mission Cancel              |\n");	
	printf("| [10] Draw Square Sample       | [29] Mission Waypoint Download   |\n");	
	printf("| [11] Take a Picture           | [30] Mission Waypoint Set Speed  |\n");	
	printf("| [12] Start Record Video       | [31] Mission Waypoint Get Speed  |\n");	 
	printf("| [13] Stop Record Video        | [32] Mission Hotpoint Set Speed  |\n");	
	printf("| [14] Local Navigation Test    | [33] Mission Hotpoint Set Radius |\n");	
	printf("| [15] Global Navigation Test   | [34] Mission Hotpoint Reset Yaw  |\n");	
	printf("| [16] Waypoint Navigation Test | [35] Mission Followme Set Target |\n");	
	printf("| [17] Arm the Drone            | [36] Mission Hotpoint Download   |\n");	
	printf("| [18] Disarm the Drone         | [37] Enter Mobile commands mode  |\n");
    printf("| [19] Virtual RC Test           \n");
    printf("+-----------------------------------------------------------------+\n");
    printf("input 1/2/3 etc..then press enter key\r\n");
    printf("use `rostopic echo` to query drone status\r\n");
    printf("----------------------------------------\r\n");
}

   
int main(int argc, char *argv[])
{
    int main_operate_code = 0;
    int temp32;
    int circleRadius;
    int circleHeight;
    float Phi, circleRadiusIncrements;
    int x_center, y_center, yaw_local;
    bool valid_flag = false;
    bool err_flag = false;
    ros::init(argc, argv, "sdk_client");
    ROS_INFO("sdk_service_client_test");
    ros::NodeHandle nh;
    DJIDrone* drone = new DJIDrone(nh);

	//virtual RC test data
	uint32_t virtual_rc_data[16];

	//set frequency test data
	uint8_t msg_frequency_data[16] = {1,2,3,4,3,2,1,2,3,4,3,2,1,2,3,4};
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
    uint8_t userData = 0;
    ros::spinOnce();
    
    //! Setting functions to be called for Mobile App Commands mode 
    drone->setObtainControlMobileCallback(ObtainControlMobileCallback, &userData);
    drone->setReleaseControlMobileCallback(ReleaseControlMobileCallback, &userData);
    drone->setTakeOffMobileCallback(TakeOffMobileCallback, &userData);
    drone->setLandingMobileCallback(LandingMobileCallback, &userData);
    drone->setGetSDKVersionMobileCallback(GetSDKVersionMobileCallback, &userData);
    drone->setArmMobileCallback(ArmMobileCallback, &userData);
    drone->setDisarmMobileCallback(DisarmMobileCallback, &userData);
    drone->setGoHomeMobileCallback(GoHomeMobileCallback, &userData);
    drone->setTakePhotoMobileCallback(TakePhotoMobileCallback, &userData);
    drone->setStartVideoMobileCallback(StartVideoMobileCallback,&userData);
    drone->setStopVideoMobileCallback(StopVideoMobileCallback,&userData);
    drone->setDrawCircleDemoMobileCallback(DrawCircleDemoMobileCallback, &userData);
    drone->setDrawSquareDemoMobileCallback(DrawSquareDemoMobileCallback, &userData);
    drone->setGimbalControlDemoMobileCallback(GimbalControlDemoMobileCallback, &userData);
    drone->setAttitudeControlDemoMobileCallback(AttitudeControlDemoMobileCallback, &userData);
    drone->setLocalNavigationTestMobileCallback(LocalNavigationTestMobileCallback, &userData);
    drone->setGlobalNavigationTestMobileCallback(GlobalNavigationTestMobileCallback, &userData);
    drone->setWaypointNavigationTestMobileCallback(WaypointNavigationTestMobileCallback, &userData);
    drone->setVirtuaRCTestMobileCallback(VirtuaRCTestMobileCallback, &userData);

    drone->setStartMapLASLoggingMobileCallback(StartMapLASLoggingMobileCallback, &userData);
    drone->setStopMapLASLoggingMobileCallback(StopMapLASLoggingMobileCallback, &userData);
    drone->setStartCollisionAvoidanceCallback(StartCollisionAvoidanceCallback, &userData);
    drone->setStopCollisionAvoidanceCallback(StopCollisionAvoidanceCallback, &userData);

	
    Display_Main_Menu();
    while(1)
    {
        ros::spinOnce();
        std::cout << "Enter Input Val: ";
        while(!(std::cin >> temp32)){
        std::cin.clear();
        std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
        std::cout << "Invalid input.  Try again: ";
	}

        if(temp32>0 && temp32<38)
        {
            main_operate_code = temp32;         
        }
        else
        {
            printf("ERROR - Out of range Input \n");
            Display_Main_Menu();
            continue;
        }
        switch(main_operate_code)
        {
			case 1:
				/* SDK version query*/
				drone->check_version();
				break;
            case 2:
                /* request control ability*/
                drone->request_sdk_permission_control();
                break;
            case 3:
                /* release control ability*/
                drone->release_sdk_permission_control();
                break;
            case 4:
                /* take off */
                drone->takeoff();
                break;
            case 5:
                /* landing*/
                drone->landing();
                break;
            case 6:
                /* go home*/
                drone->gohome();
                break;
            case 7:
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

            case 8:
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

            case 9:
                /*draw circle sample*/
                static float R = 2;
                static float V = 2;
                static float x;
                static float y;
                Phi = 0;
                std::cout<<"Enter the radius of the circle in meteres (10m > x > 4m)\n";
                std::cin>>circleRadius;   

                std::cout<<"Enter height in meteres (Relative to take off point. 15m > x > 5m) \n";
                std::cin>>circleHeight;  

                 if (circleHeight < 5)
                {
                    circleHeight = 5;
                }
                else if (circleHeight > 15)
                {
                    circleHeight = 15;
                }           
                if (circleRadius < 4)
                {
                    circleRadius = 4;
                }
                else if (circleRadius > 10)
                {
                    circleRadius = 10;
                } 
		
                x_center = drone->local_position.x;
                y_center = drone->local_position.y;
                circleRadiusIncrements = 0.01;
		
    		    for(int j = 0; j < 1000; j ++)
                {   
                  if (circleRadiusIncrements < circleRadius)
    			   {
    		        x =  x_center + circleRadiusIncrements;
    		        y =  y_center;
    			    circleRadiusIncrements = circleRadiusIncrements + 0.01;
    		        drone->local_position_control(x ,y ,circleHeight, 0);
    		        usleep(20000);
    			  }
                   else
    			   {
                    break;
                   }
                }

                /* start to draw circle */
                for(int i = 0; i < 1890; i ++)
                {   
                    x =  x_center + circleRadius*cos((Phi/300));
                    y =  y_center + circleRadius*sin((Phi/300));
                    Phi = Phi+1;
                    drone->local_position_control(x ,y ,circleHeight, 0);
                    usleep(20000);
                }
                break;

            case 10:
                /*draw square sample*/
                for(int i = 0;i < 60;i++)
                {
                    drone->attitude_control( Flight::HorizontalLogic::HORIZONTAL_POSITION |
                            Flight::VerticalLogic::VERTICAL_VELOCITY |
                            Flight::YawLogic::YAW_ANGLE |
                            Flight::HorizontalCoordinate::HORIZONTAL_BODY |
                            Flight::SmoothMode::SMOOTH_ENABLE,
                            3, 3, 0, 0 );
                    usleep(20000);
                }
                for(int i = 0;i < 60;i++)
                {
                    drone->attitude_control( Flight::HorizontalLogic::HORIZONTAL_POSITION |
                            Flight::VerticalLogic::VERTICAL_VELOCITY |
                            Flight::YawLogic::YAW_ANGLE |
                            Flight::HorizontalCoordinate::HORIZONTAL_BODY |
                            Flight::SmoothMode::SMOOTH_ENABLE,
                            -3, 3, 0, 0);
                    usleep(20000);
                }
                for(int i = 0;i < 60;i++)
                {
                    drone->attitude_control( Flight::HorizontalLogic::HORIZONTAL_POSITION |
                            Flight::VerticalLogic::VERTICAL_VELOCITY |
                            Flight::YawLogic::YAW_ANGLE |
                            Flight::HorizontalCoordinate::HORIZONTAL_BODY |
                            Flight::SmoothMode::SMOOTH_ENABLE,
                            -3, -3, 0, 0);
                    usleep(20000);
                }
                for(int i = 0;i < 60;i++)
                {
                    drone->attitude_control( Flight::HorizontalLogic::HORIZONTAL_POSITION |
                            Flight::VerticalLogic::VERTICAL_VELOCITY |
                            Flight::YawLogic::YAW_ANGLE |
                            Flight::HorizontalCoordinate::HORIZONTAL_BODY |
                            Flight::SmoothMode::SMOOTH_ENABLE,
                            3, -3, 0, 0);
                    usleep(20000);
                }
                break;
            case 11:
                /*take a picture*/
                drone->take_picture();
                break;
            case 12:
                /*start video*/
                drone->start_video();
                break;
            case 13:
                /*stop video*/
                drone->stop_video();
                break;
            case 14:
                /* Local Navi Test */
                drone->local_position_navigation_send_request(-100, -100, 100);
                break;
            case 15:
                /* GPS Navi Test */
                drone->global_position_navigation_send_request(22.5420, 113.9580, 10);
                break;
            case 16:
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
			case 17:
				//drone arm
				drone->drone_arm();
                break;
			case 18:
				//drone disarm
				drone->drone_disarm();
                break;
			case 19:
				//virtual rc test 1: arm & disarm
				drone->virtual_rc_enable();
				usleep(20000);

				virtual_rc_data[0] = 1024;	//0-> roll     	[1024-660,1024+660] 
				virtual_rc_data[1] = 1024;	//1-> pitch    	[1024-660,1024+660]
				virtual_rc_data[2] = 1024+660;	//2-> throttle 	[1024-660,1024+660]
				virtual_rc_data[3] = 1024;	//3-> yaw      	[1024-660,1024+660]
				virtual_rc_data[4] = 1684;	 	//4-> gear		{1684(UP), 1324(DOWN)}
				virtual_rc_data[6] = 1552;    	//6-> mode     	{1552(P), 1024(A), 496(F)}

				for (int i = 0; i < 100; i++){
					drone->virtual_rc_control(virtual_rc_data);
					usleep(20000);
				}

				//virtual rc test 2: yaw 
				drone->virtual_rc_enable();
				virtual_rc_data[0] = 1024;		//0-> roll     	[1024-660,1024+660] 
				virtual_rc_data[1] = 1024;		//1-> pitch    	[1024-660,1024+660]
				virtual_rc_data[2] = 1024-200;	//2-> throttle 	[1024-660,1024+660]
				virtual_rc_data[3] = 1024;		//3-> yaw      	[1024-660,1024+660]
				virtual_rc_data[4] = 1324;	 	//4-> gear		{1684(UP), 1324(DOWN)}
				virtual_rc_data[6] = 1552;    	//6-> mode     	{1552(P), 1024(A), 496(F)}

				for(int i = 0; i < 100; i++) {
					drone->virtual_rc_control(virtual_rc_data);
					usleep(20000);
				}
				drone->virtual_rc_disable();
				break;
			case 20:
				//sync flag
				drone->sync_flag_control(1);
				break;
			case 21:
				//set msg frequency
				drone->set_message_frequency(msg_frequency_data);
				break;

			case 22:

                // Clear the vector of previous waypoints 
                waypoint_task.mission_waypoint.clear();
                
				//mission waypoint upload
				waypoint_task.velocity_range = 10;
				waypoint_task.idle_velocity = 3;
				waypoint_task.action_on_finish = 0;
				waypoint_task.mission_exec_times = 1;
				waypoint_task.yaw_mode = 4;
				waypoint_task.trace_mode = 0;
				waypoint_task.action_on_rc_lost = 0;
				waypoint_task.gimbal_pitch_mode = 0;

                static int num_waypoints = 4; 
                static int altitude = 80;
                // Currently hard coded, should be dynamic
                static float orig_lat = 22.5401;
                static float orig_long = 113.9468;
                for(int i = 0; i < num_waypoints; i++)
                {
                    
                    // Careens in zig-zag pattern
    				waypoint.latitude = (orig_lat+=.0001);
                    if (i % 2 == 1){
    				    waypoint.longitude = orig_long+=.0001;
                    } else {
    				    waypoint.longitude = orig_long;
                    }
    				waypoint.altitude = altitude-=10;
    				waypoint.damping_distance = 0;
    				waypoint.target_yaw = 0;
    				waypoint.target_gimbal_pitch = 0;
    				waypoint.turn_mode = 0;
    				waypoint.has_action = 0;
    				/*
    				waypoint.action_time_limit = 10;
    				waypoint.waypoint_action.action_repeat = 1;
    				waypoint.waypoint_action.command_list[0] = 1;
    				waypoint.waypoint_action.command_parameter[0] = 1;
    				*/
    
    				waypoint_task.mission_waypoint.push_back(waypoint);
                } 
                
                /* 

				waypoint.latitude = 22.540015;
				waypoint.longitude = 113.94659;
				waypoint.altitude = 120;
				waypoint.damping_distance = 2;
				waypoint.target_yaw = 180;
				waypoint.target_gimbal_pitch = 0;
				waypoint.turn_mode = 0;
				waypoint.has_action = 0;
				waypoint.action_time_limit = 10;
				waypoint.waypoint_action.action_repeat = 1;
				waypoint.waypoint_action.command_list[0] = 1;
				waypoint.waypoint_action.command_list[1] = 1;
				waypoint.waypoint_action.command_parameter[0] = 1;
				waypoint.waypoint_action.command_parameter[1] = 1;


				waypoint_task.mission_waypoint.push_back(waypoint);

                */

				drone->mission_waypoint_upload(waypoint_task);
				break;
			case 23:
				//mission hotpoint upload
				hotpoint_task.latitude = 22.542813;
				hotpoint_task.longitude = 113.958902;
				hotpoint_task.altitude = 20;
				hotpoint_task.radius = 10;
				hotpoint_task.angular_speed = 10;
				hotpoint_task.is_clockwise = 0;
				hotpoint_task.start_point = 0;
				hotpoint_task.yaw_mode = 0;

				drone->mission_hotpoint_upload(hotpoint_task);
				break;
			case 24:
				//mission followme upload
				followme_task.mode = 0;
				followme_task.yaw_mode = 0;
				followme_task.initial_latitude = 23.540091;
				followme_task.initial_longitude = 113.946593;
				followme_task.initial_altitude = 10;
				followme_task.sensitivity = 1;

				drone->mission_followme_upload(followme_task);
				break;
			case 25:
				//mission start
				drone->mission_start();
				break;
			case 26:
				//mission pause
				drone->mission_pause();
				break;
			case 27:
				//mission resume
				drone->mission_resume();
				break;
			case 28:
				//mission cancel
				drone->mission_cancel();
				break;
			case 29:
				//waypoint mission download
				waypoint_task = drone->mission_waypoint_download();
				break;
			case 30:
				//mission waypoint set speed
				drone->mission_waypoint_set_speed((float)5);
				break;
			case 31:
				//mission waypoint get speed
				printf("%f", drone->mission_waypoint_get_speed());
				break;
			case 32:
				//mission hotpoint set speed
				drone->mission_hotpoint_set_speed((float)5,(uint8_t)1);
				break;
			case 33:
				//mission hotpoint set radius
				drone->mission_hotpoint_set_radius((float)5);
				break;
			case 34:
				//mission hotpoint reset yaw
				drone->mission_hotpoint_reset_yaw();
				break;
			case 35:
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
            case 37:
                printf("Mobile Data Commands mode entered. Use OSDK Mobile App to use this feature \n");
                printf("End program to exit this mode \n");
                while(1)
                {             
                ros::spinOnce();  
                }
			case 36:
				hotpoint_task = drone->mission_hotpoint_download();

            default:
                break;
        }
        main_operate_code = -1;
        Display_Main_Menu();
    }
    return 0;
}

//! Callback functions for Mobile Commands
    void ObtainControlMobileCallback(DJIDrone *drone)
    {
      drone->request_sdk_permission_control();
    }

    void ReleaseControlMobileCallback(DJIDrone *drone)
    {
      drone->release_sdk_permission_control();
    }

    void TakeOffMobileCallback(DJIDrone *drone)
    {
      drone->takeoff();
    }

    void LandingMobileCallback(DJIDrone *drone)
    {
      drone->landing();
    }

    void GetSDKVersionMobileCallback(DJIDrone *drone)
    {
      drone->check_version();
    }

    void ArmMobileCallback(DJIDrone *drone)
    {
      drone->drone_arm();
    }

    void DisarmMobileCallback(DJIDrone *drone)
    {
      drone->drone_disarm();
    }

    void GoHomeMobileCallback(DJIDrone *drone)
    {
      drone->gohome();
    }

    void TakePhotoMobileCallback(DJIDrone *drone)
    {
      drone->take_picture();
    }

    void StartVideoMobileCallback(DJIDrone *drone)
    {
      drone->start_video();
    }

    void StopVideoMobileCallback(DJIDrone *drone)
    {
      drone->stop_video();
    }

    void DrawCircleDemoMobileCallback(DJIDrone *drone)
    {
        static float R = 2;
        static float V = 2;
        static float x;
        static float y;
        int circleRadius;
        int circleHeight;
        float Phi =0, circleRadiusIncrements;
        int x_center, y_center, yaw_local; 

        circleHeight = 7;
        circleRadius = 7;

        x_center = drone->local_position.x;
        y_center = drone->local_position.y;
        circleRadiusIncrements = 0.01;

        for(int j = 0; j < 1000; j ++)
        {   
            if (circleRadiusIncrements < circleRadius)
            {
                x =  x_center + circleRadiusIncrements;
                y =  y_center;
                circleRadiusIncrements = circleRadiusIncrements + 0.01;
                drone->local_position_control(x ,y ,circleHeight, 0);
                usleep(20000);
            }
                else
            {
                break;
            }
        }
        

        /* start to draw circle */
        for(int i = 0; i < 1890; i ++)
        {   
            x =  x_center + circleRadius*cos((Phi/300));
            y =  y_center + circleRadius*sin((Phi/300));
            Phi = Phi+1;
            drone->local_position_control(x ,y ,circleHeight, 0);
            usleep(20000);
        }

    }
    void DrawSquareDemoMobileCallback(DJIDrone *drone)
    {
    /*draw square sample*/
        for(int i = 0;i < 60;i++)
        {
            drone->attitude_control( Flight::HorizontalLogic::HORIZONTAL_POSITION |
            Flight::VerticalLogic::VERTICAL_VELOCITY |
            Flight::YawLogic::YAW_ANGLE |
            Flight::HorizontalCoordinate::HORIZONTAL_BODY |
            Flight::SmoothMode::SMOOTH_ENABLE,
            3, 3, 0, 0 );
            usleep(20000);
        }
        for(int i = 0;i < 60;i++)
        {
            drone->attitude_control( Flight::HorizontalLogic::HORIZONTAL_POSITION |
            Flight::VerticalLogic::VERTICAL_VELOCITY |
            Flight::YawLogic::YAW_ANGLE |
            Flight::HorizontalCoordinate::HORIZONTAL_BODY |
            Flight::SmoothMode::SMOOTH_ENABLE,
            -3, 3, 0, 0);
            usleep(20000);
        }
        for(int i = 0;i < 60;i++)
        {
            drone->attitude_control( Flight::HorizontalLogic::HORIZONTAL_POSITION |
            Flight::VerticalLogic::VERTICAL_VELOCITY |
            Flight::YawLogic::YAW_ANGLE |
            Flight::HorizontalCoordinate::HORIZONTAL_BODY |
            Flight::SmoothMode::SMOOTH_ENABLE,
            -3, -3, 0, 0);
            usleep(20000);
        }
        for(int i = 0;i < 60;i++)
        {
            drone->attitude_control( Flight::HorizontalLogic::HORIZONTAL_POSITION |
            Flight::VerticalLogic::VERTICAL_VELOCITY |
            Flight::YawLogic::YAW_ANGLE |
            Flight::HorizontalCoordinate::HORIZONTAL_BODY |
            Flight::SmoothMode::SMOOTH_ENABLE,
            3, -3, 0, 0);
            usleep(20000);
        }
    }

     void GimbalControlDemoMobileCallback(DJIDrone *drone)
        {
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
        }

    void AttitudeControlDemoMobileCallback(DJIDrone *drone)
    {
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

    }
    void LocalNavigationTestMobileCallback(DJIDrone *drone)
    {

    }
    void GlobalNavigationTestMobileCallback(DJIDrone *drone)
    {

    }
    void WaypointNavigationTestMobileCallback(DJIDrone *drone)
    {
        
    }
    void VirtuaRCTestMobileCallback(DJIDrone *drone)
    {
        //virtual RC test data
        uint32_t virtual_rc_data[16];
        //virtual rc test 1: arm & disarm
        drone->virtual_rc_enable();
        usleep(20000);

        virtual_rc_data[0] = 1024;  //0-> roll      [1024-660,1024+660] 
        virtual_rc_data[1] = 1024;  //1-> pitch     [1024-660,1024+660]
        virtual_rc_data[2] = 1024+660;  //2-> throttle  [1024-660,1024+660]
        virtual_rc_data[3] = 1024;  //3-> yaw       [1024-660,1024+660]
        virtual_rc_data[4] = 1684;      //4-> gear      {1684(UP), 1324(DOWN)}
        virtual_rc_data[6] = 1552;      //6-> mode      {1552(P), 1024(A), 496(F)}

        for (int i = 0; i < 100; i++){
            drone->virtual_rc_control(virtual_rc_data);
            usleep(20000);
        }

        //virtual rc test 2: yaw 
        drone->virtual_rc_enable();
        virtual_rc_data[0] = 1024;      //0-> roll      [1024-660,1024+660] 
        virtual_rc_data[1] = 1024;      //1-> pitch     [1024-660,1024+660]
        virtual_rc_data[2] = 1024-200;  //2-> throttle  [1024-660,1024+660]
        virtual_rc_data[3] = 1024;      //3-> yaw       [1024-660,1024+660]
        virtual_rc_data[4] = 1324;      //4-> gear      {1684(UP), 1324(DOWN)}
        virtual_rc_data[6] = 1552;      //6-> mode      {1552(P), 1024(A), 496(F)}

        for(int i = 0; i < 100; i++) {
            drone->virtual_rc_control(virtual_rc_data);
            usleep(20000);
        }
        drone->virtual_rc_disable();
    }

void StartMapLASLoggingMobileCallback(DJIDrone *drone)
{
  system("roslaunch point_cloud_las start_velodyne_and_loam.launch &");
  system("rosrun point_cloud_las write _topic:=/laser_cloud_surround _folder_path:=. &");
}

void StopMapLASLoggingMobileCallback(DJIDrone *drone)
{
  system("rosnode kill /write_LAS /scanRegistration /laserMapping /transformMaintenance /laserOdometry  &");
}

void StartCollisionAvoidanceCallback(DJIDrone *drone)
{ 
  uint8_t freq[16];
  freq[0] = 1;    // 0 - Timestamp
  freq[1] = 4;    // 1 - Attitude Quaterniouns
  freq[2] = 1;    // 2 - Acceleration
  freq[3] = 4;    // 3 - Velocity (Ground Frame)
  freq[4] = 4;    // 4 - Angular Velocity (Body Frame)
  freq[5] = 3;    // 5 - Position
  freq[6] = 0;    // 6 - Magnetometer
  freq[7] = 3;    // 7 - M100:RC Channels Data, A3:RTK Detailed Information
  freq[8] = 0;    // 8 - M100:Gimbal Data, A3: Magnetometer
  freq[9] = 3;    // 9 - M100:Flight Status, A3: RC Channels
  freq[10] = 0;   // 10 - M100:Battery Level, A3: Gimble Data
  freq[11] = 2;   // 11 - M100:Control Information, A3: Flight Status

  drone->set_message_frequency(freq);
  usleep(1e4);
  system("roslaunch dji_collision_avoidance from_DJI_ros_demo.launch &");
}

void StopCollisionAvoidanceCallback(DJIDrone *drone)
{
  drone->release_sdk_permission_control();
  system("rosnode kill /drone_tf_builder /dji_occupancy_grid_node /dji_collision_detection_node /collision_velodyne_nodelet_manager /manual_fly");
  usleep(1e4);
  drone->request_sdk_permission_control();
}
