#include "ros/ros.h"
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <stdio.h>
#include <setjmp.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <getopt.h>
#include <strings.h>
#include <stdbool.h>
#include <assert.h>
#include "djiCam.h"


int main(int argc, char **argv)
{
	
	ros::init(argc,argv,"ImageTrans");

	ros::NodeHandle nh;

	image_transport::ImageTransport it(nh);
	image_transport::Publisher pub = it.advertise("dji_sdk/image_raw", 1);

	cv_bridge::CvImage out_Image;
	sensor_msgs::Image msg;

	ros::Time time=ros::Time::now();

	IplImage * pImg = cvCreateImage(cvSize(1280, 720),IPL_DEPTH_8U,3);

	int ret = djiCam_init();

	int nState = 1;

	while(nState && ros::ok())
	{
		nState = djiCam_loop(pImg); /*DISPLAY_MODE | SAVEPICTURE_MODE | TRANSFER_MODE*/
		
		time=ros::Time::now();
		out_Image.header.stamp = time;
		out_Image.header.frame_id = "image";
		out_Image.encoding = "bgr8";
		out_Image.image = pImg;
		out_Image.toImageMsg(msg);
		pub.publish(msg);

		ros::spinOnce();

	}

    /*Ctrl+c to break*/
    djiCam_exit();
    printf("System exit ok!\n");

    return 0;
}
