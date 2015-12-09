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
    printf("[f] Gimbal control sample\n");
    printf("[g] Attitude control sample\n");
    printf("[h] Draw circle sample\n");
    printf("[i] Draw square sample\n");
    printf("[j] Take a picture\n");
    printf("[k] Start video\n");
    printf("[l] Stop video\n");
    printf("[m] Local Navi Test\n");
    printf("[n] GPS Navi Test\n");
    printf("[o] Waypoint List Test\n");
    printf("[p] Exit\n");
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

    dji_sdk::WaypointList newWaypointList;
    dji_sdk::Waypoint waypoint0;
    dji_sdk::Waypoint waypoint1;
    dji_sdk::Waypoint waypoint2;
    dji_sdk::Waypoint waypoint3;
    dji_sdk::Waypoint waypoint4;

    Display_Main_Menu();
    while(1)
    {
		ros::spinOnce();
        temp32 = getchar();
        if(temp32 != 10)
        {
            if(temp32 >= 'a' && temp32 <= 'p' && valid_flag == false)
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

            case 'g':
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

            case 'h':
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
        
                    drone->attitude_control(HORIZ_POS|VERT_VEL|YAW_ANG|HORIZ_BODY|YAW_BODY, vx, vy, 0, 0);
                    usleep(20000);
                    time++;
                }
                break;

            case 'i':
                /*draw square sample*/
                for(int i = 0;i < 60;i++)
                {
                    drone->attitude_control(HORIZ_POS|VERT_VEL|YAW_ANG|HORIZ_BODY|YAW_BODY, 3, 3, 0, 0);
                    usleep(20000);
                }
                for(int i = 0;i < 60;i++)
                {
                    drone->attitude_control(HORIZ_POS|VERT_VEL|YAW_ANG|HORIZ_BODY|YAW_BODY, -3, 3, 0, 0);
                    usleep(20000);
                }
                for(int i = 0;i < 60;i++)
                {
                    drone->attitude_control(HORIZ_POS|VERT_VEL|YAW_ANG|HORIZ_BODY|YAW_BODY, -3, -3, 0, 0);
                    usleep(20000);
                }
                for(int i = 0;i < 60;i++)
                {
                    drone->attitude_control(HORIZ_POS|VERT_VEL|YAW_ANG|HORIZ_BODY|YAW_BODY, 3, -3, 0, 0);
                    usleep(20000);
                }
                break;
            case 'j':
                /*take a picture*/
                drone->take_picture();
                break;
            case 'k':
                /*start video*/
                drone->start_video();
                break;
            case 'l':
                /*stop video*/
                drone->stop_video();
                break;
            case 'm':
                /* Local Navi Test */
                drone->local_position_navigation_send_request(-100, -100, 100);
                break;
            case 'n':
                /* GPS Navi Test */
                drone->global_position_navigation_send_request(22.535, 113.95, 100);
                break;
            case 'o':
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
            case 'p':
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
