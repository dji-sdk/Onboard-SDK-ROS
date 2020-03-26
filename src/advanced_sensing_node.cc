/**
  * @Copyright (C) 2020 All rights reserved.
  * @date: 2020
  * @file: advanced_sensing_node.cc
  * @version: v0.0.1
  * @author: charles.huang@dji.com
  * @create_date: 2020-03-26
  * @brief: TODO
  * @details: TODO
  */

//INCLUDE
#include <ros/ros.h>
#include <dji_osdk_ros/common_type.hh>
#include <string>

#include <dji_osdk_ros/AdvancedSensing.h>
#include <dji_camera_image.hpp>

#ifdef OPEN_CV_INSTALLED
#include "opencv2/opencv.hpp"
#include "opencv2/highgui/highgui.hpp"
#endif


//CODE
using namespace dji_osdk_ros;

int writeH264StreamData(const char *fileName, const uint8_t *data, uint32_t len)
{
    FILE *fp = NULL;
    size_t size = 0;

    fp = fopen(fileName, "a+");
    if(fp == NULL) {
        printf("fopen failed!\n");
        return -1;
    }
    size = fwrite(data, 1, len, fp);
    if(size != len) {
        return -1;
    }

    fflush(fp);
    if(fp) {
        fclose(fp);
    }
    return 0;
}

void show_rgb(CameraRGBImage img, void *p)
{
    std::string name = std::string(reinterpret_cast<char *>(p));
    std::cout << "#### Got image from:\t" << name << std::endl;
    #ifdef OPEN_CV_INSTALLED
    cv::Mat mat(img.height, img.width, CV_8UC3, img.rawData.data(), img.width*3);
    cv::cvtColor(mat, mat, COLOR_RGB2BGR);
    cv::imshow(name,mat);
    cv::waitKey(1);
    #endif
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "advanced_sensing_node");
    ros::NodeHandle nh;
    auto advanced_sensing_client = nh.serviceClient<AdvancedSensing>("/advanced_sensing");

    std::cout
            << "| Available commands:                                            |"
            << std::endl;
    std::cout
            << "| [y] device is h264                                             |"
            << std::endl;
    std::cout
            << "| [n] device is not h264                                         |"
            << std::endl;

    char input_device_char;
    std::cin >> input_device_char;

    AdvancedSensing advanced_sensing;
    switch(input_device_char)
    {
        case 'y':
        {
            advanced_sensing.request.is_h264 = true;
            std::cout << std::endl;
            std::cout
                    << "| Available commands:                                            |"
                    << std::endl
                    << "| [a] Start getting FPV H264 stream sample                       |"
                    << std::endl
                    << "| [b] Start getting main camera H264 stream sample               |"
                    << std::endl
                    << "| [c] Start getting vice camera H264 stream sample               |"
                    << std::endl
                    << "| [d] Start getting top camera H264 stream sample                |"
                    << std::endl;

            char input_view_char;
            std::cin >> input_view_char;

            switch(input_view_char)
            {
                case 'a':
                {
                    advanced_sensing.request.request_view = AdvancedSensing::Request::FPV_CAMERA;
                    break;
                }
                case 'b':
                {
                    advanced_sensing.request.request_view = AdvancedSensing::Request::MAIN_CAMERA;
                    break;
                }
                case 'c':
                {
                    advanced_sensing.request.request_view = AdvancedSensing::Request::VICE_CAMERA;
                    break;
                }
                case 'd':
                {
                    advanced_sensing.request.request_view = AdvancedSensing::Request::TOP_CAMERA;
                    break;
                }
                default:
                {
                    std::cout << "No camera selected" << std::endl;
                    return 1;
                }
            }
            break;
        }
        case 'n':
        {
            advanced_sensing.request.is_h264 = true;
            std::cout << std::endl;
            std::cout
                    << "| Available commands:                                            |"
                    << std::endl
                    << "| [a] Start getting FPV stream sample                       |"
                    << std::endl
                    << "| [b] Start getting main camera stream sample               |"
                    << std::endl;

            char input_view_char;
            std::cin >> input_view_char;

            switch(input_view_char)
            {
                case 'a':
                {
                    advanced_sensing.request.request_view = AdvancedSensing::Request::FPV_CAMERA;
                    break;
                }
                case 'b':
                {
                    advanced_sensing.request.request_view = AdvancedSensing::Request::MAIN_CAMERA;
                    break;
                }
                default:
                {
                    std::cout << "No camera selected" << std::endl;
                    return 1;
                }
            }
            break;
        }
    }

    advanced_sensing.request.is_open = true;
    advanced_sensing_client.call(advanced_sensing);
    if (advanced_sensing.response.result)
    {
        ROS_INFO_STREAM("open advanced sensing task successful!");
        if(advanced_sensing.request.is_h264)
        {
            ros::Time begin_time = ros::Time::now();
            while (true)
            {
                uint8_t raw_data[advanced_sensing.response.raw_data_len];
                for (int i = 0; i < advanced_sensing.response.raw_data_len; i++)
                {
                    raw_data[i] = advanced_sensing.response.raw_data[i];
                }
                writeH264StreamData("H264View.h264", raw_data, advanced_sensing.response.raw_data_len);
                ros::Time current_time = ros::Time::now();
                if (current_time.sec - begin_time.sec >= 300)
                {
                    break;
                }
            }
        }
        else
        {
            CameraRGBImage img;
            char Name[] = "CAM";
            ros::Time begin_time = ros::Time::now();
            while(true)
            {
                for (int i = 0; i < advanced_sensing.response.raw_data_len; i++)
                {
                    img.rawData[i] = advanced_sensing.response.raw_data[i];
                }
                img.height = advanced_sensing.response.height;
                img.width  = advanced_sensing.response.width;
                show_rgb(img, &Name);

                ros::Time current_time = ros::Time::now();
                if (current_time.sec - begin_time.sec >= 300)
                {
                    break;
                }
            }
        }
    }

    advanced_sensing.request.is_open = false;
    advanced_sensing_client.call(advanced_sensing);
}