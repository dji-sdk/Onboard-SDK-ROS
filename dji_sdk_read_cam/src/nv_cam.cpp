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

typedef unsigned char   BYTE;
#define IMAGE_W 1280
#define IMAGE_H 720
#define FRAME_SIZE              IMAGE_W*IMAGE_H*3


unsigned char buffer[FRAME_SIZE] = {0};
unsigned int frame_size = 0;
unsigned int nframe = 0;
FILE *fp;


struct sRGB{
	int r;
	int g;
	int b;
};

sRGB yuvTorgb(int Y, int U, int V)
{
	sRGB rgb;
	rgb.r = (int)(Y + 1.4075 * (V-128));
	rgb.g = (int)(Y - 0.3455 * (U-128) - 0.7169*(V-128));
	rgb.b = (int)(Y + 1.779 * (U-128));
	rgb.r =(rgb.r<0? 0: rgb.r>255? 255 : rgb.r);
	rgb.g =(rgb.g<0? 0: rgb.g>255? 255 : rgb.g);
	rgb.b =(rgb.b<0? 0: rgb.b>255? 255 : rgb.b);
	return rgb;
}

unsigned char * NV12ToRGB(unsigned char * src, unsigned char * rgb, int width, int height){
	int numOfPixel = width * height;
	int positionOfU = numOfPixel;
	int startY,step,startU,Y,U,V,index,nTmp;
	sRGB tmp;

	for(int i=0; i<height; i++){
		startY = i*width;
		step = i/2*width;
		startU = positionOfU + step;
		for(int j = 0; j < width; j++){
			Y = startY + j;
			if(j%2 == 0)
				nTmp = j;
			else
				nTmp = j - 1;
			U = startU + nTmp;
			V = U + 1;
			index = Y*3;
			tmp = yuvTorgb((int)src[Y], (int)src[U], (int)src[V]);
			rgb[index+0] = (char)tmp.b;
			rgb[index+1] = (char)tmp.g;
			rgb[index+2] = (char)tmp.r;
		}
	}
	return rgb;
}

bool YUV420_To_BGR24(unsigned char *puc_y, unsigned char *puc_u, unsigned char *puc_v, unsigned char *puc_rgb, int width_y, int height_y)
{
	if (!puc_y || !puc_u || !puc_v || !puc_rgb)
	{
		return false;
	}
	int baseSize = width_y * height_y;
	int rgbSize = baseSize * 3;

	BYTE* rgbData = new BYTE[rgbSize];
	memset(rgbData, 0, rgbSize);

	int temp = 0;

	BYTE* rData = rgbData; 
	BYTE* gData = rgbData + baseSize;
	BYTE* bData = gData + baseSize;

	int uvIndex =0, yIndex =0;


	for(int y=0; y < height_y; y++)
	{
		for(int x=0; x < width_y; x++)
		{
			uvIndex = (y>>1) * (width_y>>1) + (x>>1);
			yIndex = y * width_y + x;

			temp = (int)(puc_y[yIndex] + (puc_v[uvIndex] - 128) * 1.4022);
			rData[yIndex] = temp<0 ? 0 : (temp > 255 ? 255 : temp);

			temp = (int)(puc_y[yIndex] + (puc_u[uvIndex] - 128) * (-0.3456) +
					(puc_v[uvIndex] - 128) * (-0.7145));
			gData[yIndex] = temp < 0 ? 0 : (temp > 255 ? 255 : temp);

			temp = (int)(puc_y[yIndex] + (puc_u[uvIndex] - 128) * 1.771);
			bData[yIndex] = temp < 0 ? 0 : (temp > 255 ? 255 : temp);
		}
	}

	int widthStep = width_y*3;
	for (int y = 0; y < height_y; y++)
	{
		for (int x = 0; x < width_y; x++)
		{
			puc_rgb[y * widthStep + x * 3 + 2] = rData[y * width_y + x]; //R
			puc_rgb[y * widthStep + x * 3 + 1] = gData[y * width_y + x]; //G
			puc_rgb[y * widthStep + x * 3 + 0] = bData[y * width_y + x]; //B
		}
	}

	if (!puc_rgb)
	{
		return false;
	}
	delete [] rgbData;
	return true;
}

IplImage* YUV420_To_IplImage(unsigned char* pYUV420, int width, int height)
{
	if (!pYUV420)
	{
		return NULL;
	}

	int baseSize = width*height;
	int imgSize = baseSize*3;
	BYTE* pRGB24 = new BYTE[imgSize];
	memset(pRGB24, 0, imgSize);

	int temp = 0;

	BYTE* yData = pYUV420; 
	BYTE* uData = pYUV420 + baseSize; 
	BYTE* vData = uData + (baseSize>>2); 

	if(YUV420_To_BGR24(yData, uData, vData, pRGB24, width, height) == false || !pRGB24)
	{
		return NULL;
	}

	IplImage *image = cvCreateImage(cvSize(width, height), 8,3);
	memcpy(image->imageData, pRGB24, imgSize);

	if (!image)
	{
		return NULL;
	}

	delete [] pRGB24;
	return image;
}

int main(int argc, char **argv)
{
	ros::init(argc,argv,"image_raw");
	int ret,nKey;
	int nState = 1;
	int nCount = 1;

	int gray_or_rgb = 0;
	int to_mobile = 0;

	IplImage *pRawImg;
	IplImage *pImg;
	unsigned char *pData;

	int mode = GETBUFFER_MODE;


	ros::NodeHandle nh_private("~");
	nh_private.param("gray_or_rgb", gray_or_rgb, 0);
	nh_private.param("to_mobile", to_mobile, 0);

	printf("%d\n",gray_or_rgb);
	if(gray_or_rgb){
		pRawImg = cvCreateImage(cvSize(IMAGE_W, IMAGE_H),IPL_DEPTH_8U,3);
		pImg = cvCreateImage(cvSize(640, 480),IPL_DEPTH_8U,3);
		pData  = new unsigned char[1280 * 720 * 3];
	} else{
		pRawImg = cvCreateImage(cvSize(IMAGE_W, IMAGE_H),IPL_DEPTH_8U,1);
		pImg = cvCreateImage(cvSize(640, 480),IPL_DEPTH_8U,1);
	}
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

	while(1)
	{
		ret = manifold_cam_read(buffer, &nframe, 1);

		if(ret != -1)
		{

			if(gray_or_rgb){
				NV12ToRGB(buffer,pData,1280,720);
				memcpy(pRawImg->imageData,pData,FRAME_SIZE);
			}else{
				memcpy(pRawImg->imageData,buffer,FRAME_SIZE/3);
			}
			cvResize(pRawImg,pImg,CV_INTER_LINEAR);

			time=ros::Time::now();
			cvi.header.stamp = time;
			cvi.header.frame_id = "image";
			if(gray_or_rgb){
				cvi.encoding = "bgr8";
			}else{
				cvi.encoding = "mono8";
			}
			cvi.image = pImg;
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
