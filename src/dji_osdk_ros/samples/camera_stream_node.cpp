/** @file camera_stream_node.cpp
 *  @version 4.0
 *  @date June 2020
 *
 *  @brief sample node of camera stream.
 *
 *  @Copyright (c) 2020 DJI
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 *
 */

//INCLUDE
#include <ros/ros.h>
#include <dji_osdk_ros/common_type.h>
#include <sensor_msgs/Image.h>
#include <stdio.h>
#include <dji_camera_image.hpp>
#include <dji_osdk_ros/SetupCameraStream.h>

#ifdef OPEN_CV_INSTALLED
#include "opencv2/opencv.hpp"
#include "opencv2/highgui/highgui.hpp"
#endif

//CODE
using namespace dji_osdk_ros;

void show_rgb(CameraRGBImage img, void *p)
{
    std::string name = std::string(reinterpret_cast<char *>(p));
    std::cout << "#### Got image from:\t" << name << std::endl;
#ifdef OPEN_CV_INSTALLED
    cv::Mat mat(img.height, img.width, CV_8UC3, img.rawData.data(), img.width*3);
    cv::cvtColor(mat, mat, cv::COLOR_RGB2BGR);
    cv::imshow(name,mat);
    cv::waitKey(1);
#endif
}

void fpvCameraStreamCallBack(const sensor_msgs::Image& msg)
{
    CameraRGBImage img;
    img.rawData = msg.data;
    img.height  = msg.height;
    img.width   = msg.width;
    char Name[] = "FPV_CAM";
    show_rgb(img, &Name);
    std::cout<<"height is"<<msg.height<<std::endl;
    std::cout<<"width is"<<msg.width<<std::endl;
}

void mainCameraStreamCallBack(const sensor_msgs::Image& msg)
{
    CameraRGBImage img;
    img.rawData = msg.data;
    img.height  = msg.height;
    img.width   = msg.width;
    char Name[] = "MAIN_CAM";
    show_rgb(img, &Name);
    std::cout<<"height is"<<msg.height<<std::endl;
    std::cout<<"width is"<<msg.width<<std::endl;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "camera_stream_node");
    ros::NodeHandle nh;
    auto setup_camera_stream_client = nh.serviceClient<dji_osdk_ros::SetupCameraStream>("setup_camera_stream");
    auto fpv_camera_stream_sub = nh.subscribe("dji_osdk_ros/fpv_camera_images", 10, fpvCameraStreamCallBack);
    auto main_camera_stream_sub = nh.subscribe("dji_osdk_ros/main_camera_images", 10, mainCameraStreamCallBack);
    dji_osdk_ros::SetupCameraStream setupCameraStream_;

    bool f = false;
    bool m = false;
    char c = 0;
    std::cout << "Please enter the type of camera stream you want to view (M210 V2)\n"
              << "m: Main Camera\n"
              << "f: FPV  Camera" << std::endl;
    std::cin >> c;

    switch(c)
    {
        case 'm':
        m=true; break;
        case 'f':
        f=true; break;
        default:
        ROS_DEBUG("No camera selected");
        return 1;
    }

    if(f)
    {
        setupCameraStream_.request.cameraType = setupCameraStream_.request.FPV_CAM;
        setupCameraStream_.request.start = 1;
        setup_camera_stream_client.call(setupCameraStream_);
    }
    else if(m)
    {
        setupCameraStream_.request.cameraType = setupCameraStream_.request.MAIN_CAM;
        setupCameraStream_.request.start = 1;
        setup_camera_stream_client.call(setupCameraStream_);
    }

    //cameraZoomControl(vehicle);     //run camera zoom control test
    CameraRGBImage camImg;            //get the camera's image

    // main thread just sleep
    // callback function will be called whenever a new image is ready
    ros::AsyncSpinner spinner(1);
    spinner.start();
    ros::Duration(30).sleep();

    if(f)
    {
        setupCameraStream_.request.cameraType = setupCameraStream_.request.FPV_CAM;
        setupCameraStream_.request.start = 0;
        setup_camera_stream_client.call(setupCameraStream_);
    }
    else if(m)
    {
        setupCameraStream_.request.cameraType = setupCameraStream_.request.MAIN_CAM;
        setupCameraStream_.request.start = 0;
        setup_camera_stream_client.call(setupCameraStream_);
    }

    ROS_INFO_STREAM("Finished. Press CTRL-C to terminate the node");
    ros::waitForShutdown();
    return 0;
}