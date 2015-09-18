#include <iostream>
#include <stdio.h>
#include <string.h>
#include <thread>
#include "ros/ros.h"
#include "djiMain.h"

int DJI_Setup(std::string serial_port, int baudrate) {
	int ret;
	char uart_name[32];
	strcpy(uart_name, serial_port.c_str());
	printf("Serial port: %s\n", uart_name);
	printf("Baudrate: %d\n", baudrate);
	printf("=========================\n");
	
	//Serial Port Init
	ret = Pro_Hw_Setup(uart_name,baudrate);
	if(ret < 0)
		return ret;

	//Setup Other Things
	DJI_Pro_Setup(NULL);
	return 0;
}

int main(int argc,char **argv) {

	char temp_buf[65];

	ros::init(argc, argv, "DJI_ROS");
	ros::NodeHandle nh;
	ros::NodeHandle nh_private("~");

	nh_private.param("serial_name", serial_name, std::string("/dev/ttyTHS1"));
	nh_private.param("baud_rate", baud_rate, 230400);
	nh_private.param("app_id", app_id, 1022384);
	nh_private.param("app_api_level", app_api_level, 2);
	nh_private.param("app_version", app_version, 1);
	nh_private.param("app_bundle_id", app_bundle_id, std::string("12345678901234567890123456789012"));
	nh_private.param("enc_key", enc_key,
std::string("e7bad64696529559318bb35d0a8c6050d3b88e791e1808cfe8f7802150ee6f0d"));


	user_act_data.app_id = app_id;
	user_act_data.app_api_level =app_api_level;
	user_act_data.app_ver = SDK_VERSION;
	strcpy((char*)user_act_data.app_bundle_id, app_bundle_id.c_str());

	user_act_data.app_key = temp_buf;
	strcpy(user_act_data.app_key, enc_key.c_str());


	printf("=================================================\n");
	printf("app id: %d\n",user_act_data.app_id);
	printf("api level: %d\n",user_act_data.app_api_level);
	printf("app version: 0x0%X\n",user_act_data.app_ver);
	printf("app key: %s\n",user_act_data.app_key);
	printf("=================================================\n");

	if (DJI_Setup(serial_name.c_str(),baud_rate) < 0) {
		printf("Serial Port Cannot Open\n");
		return 0;
	}	

	DJI_Pro_Activate_API(&user_act_data,NULL);

	ros::spin();
	return 0;
}
