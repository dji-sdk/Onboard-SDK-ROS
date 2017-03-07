#include "ros/ros.h"
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/CameraInfo.h>
#include <sys/stat.h>
#include <sys/time.h>
#include <stdio.h>
#include <malloc.h>
#include <string.h>
#include <pthread.h>
#include <unistd.h>
#include <poll.h>
#include <signal.h>
#include <assert.h>
#include <sys/types.h>
#include <unistd.h>
#include "cv.h"
#include "highgui.h"
#include "djicam.h"
#include <opencv2/opencv.hpp>

#define IMAGE_W 1280
#define IMAGE_H 720
#define FRAME_SIZE              IMAGE_W*IMAGE_H*3

using namespace cv;

unsigned char buffer[FRAME_SIZE] = {0};

unsigned int nframe = 0;
FILE *fp;

int main(int argc, char **argv)
{
	ros::init(argc,argv,"image_raw");
	int ret,nKey;
	int nState = 1;
	int nCount = 1;

	int gray_or_rgb = 0;
	int to_mobile = 0;

	int mode = GETBUFFER_MODE;

	ros::NodeHandle nh_private("~");
	nh_private.param("gray_or_rgb", gray_or_rgb, 0);
	nh_private.param("to_mobile", to_mobile, 0);

	printf("%d\n",gray_or_rgb);

	if(to_mobile) {
		mode |= TRANSFER_MODE;
	}

	ros::NodeHandle node;
	image_transport::ImageTransport transport(node);
	image_transport::Publisher image_pub = transport.advertise("dji_sdk/image_raw", 1);
	ros::Publisher caminfo_pub = node.advertise<sensor_msgs::CameraInfo>("dji_sdk/camera_info",1);

	ros::Time time=ros::Time::now();

	cv_bridge::CvImage cvi;


	sensor_msgs::Image im;
	sensor_msgs::CameraInfo cam_info;

	cam_info.header.frame_id = "/camera";
	cam_info.height = IMAGE_H/2;
	cam_info.width = IMAGE_W/2;
	cam_info.distortion_model = "";
	cam_info.D.push_back(-0.1297646493949856);
	cam_info.D.push_back(0.0946885697670611);
	cam_info.D.push_back(-0.0002935002712265514);
	cam_info.D.push_back(-0.00022663675362156343);
	cam_info.D.push_back(0.0);
	cam_info.K = {388.40923066779754, 0.0, 318.06257844065226, 0.0, 518.1538449374815, 241.17339016626644, 0.0, 0.0, 1.0};
	cam_info.R = {1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0};
	cam_info.P = {373.5429992675781, 0.0, 317.51131336952494, 0.0, 0.0, 504.4360656738281, 240.6131009245937, 0.0, 0.0, 0.0, 1.0, 0.0};
	cam_info.binning_x = 0;
	cam_info.binning_x = 0;

	cam_info.roi.x_offset = 0;
	cam_info.roi.y_offset = 0;
	cam_info.roi.height = 0;
	cam_info.roi.width = 0;
	cam_info.roi.do_rectify = false;

	ret = manifold_cam_init(mode);
	if(ret == -1)
	{
		printf("manifold init error \n");
		return -1;
    }
    Mat imageCut,imageOri;
	while(1)
    {
		ret = manifold_cam_read(buffer, &nframe, 1);

		if(ret != -1)
		{

            imageOri = Mat(IMAGE_H * 3 / 2, IMAGE_W, CV_8UC1, buffer);
			if(gray_or_rgb){
                cvtColor(imageOri,imageOri,CV_YUV2BGR_NV12);
			}else{
                cvtColor(imageOri,imageOri,CV_YUV2GRAY_NV12);
            }
            //imageCut = imageOri(Range(0, 720), Range(160, 1120));
			time=ros::Time::now();
			cvi.header.stamp = time;
			cvi.header.frame_id = "image";
			if(gray_or_rgb){
				cvi.encoding = "bgr8";
			}else{
				cvi.encoding = "mono8";
			}
            cvi.image = imageOri;//imageCut;
			cvi.toImageMsg(im);
			cam_info.header.seq = nCount;
			cam_info.header.stamp = time;
			caminfo_pub.publish(cam_info);
			image_pub.publish(im);

			ros::spinOnce();
			nCount++;

		}
		else 
			break;
		usleep(1000);
	}
	while(!manifold_cam_exit())
	{
		sleep(1);
    }

	fclose(fp);
	sleep(1);
	return 0;
}
