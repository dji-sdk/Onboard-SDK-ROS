/** @file camera_h264_node.cpp
 *  @version 4.0
 *  @date June 2020
 *
 *  @brief sample node of camera h264.
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
#include <iostream>

#include <dji_osdk_ros/SetupCameraH264.h>
#include <sensor_msgs/Image.h>

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

void cameraH264CallBack(const sensor_msgs::Image& msg)
{
    uint8_t tempData[msg.data.size()];
    memcpy(tempData, &msg.data[0], msg.data.size());
    writeH264StreamData("H264View.h264", tempData, msg.data.size());
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "camera_h264_node");
    ros::NodeHandle nh;
    auto setup_camera_h264_client = nh.serviceClient<dji_osdk_ros::SetupCameraH264>("setup_camera_h264");
    auto fpv_camera_h264_sub = nh.subscribe("dji_osdk_ros/camera_h264_stream", 10, cameraH264CallBack);
    dji_osdk_ros::SetupCameraH264 setupCameraH264_;

    char inputChar = 0;
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

    std::cin >> inputChar;

    struct timeval tv;
    struct tm tm;
    gettimeofday(&tv, NULL);
    localtime_r(&tv.tv_sec, &tm);

    switch (inputChar) 
    {
      case 'a':
      {
        setupCameraH264_.request.request_view = setupCameraH264_.request.FPV_CAMERA;
        setupCameraH264_.request.start        = 1;
        setup_camera_h264_client.call(setupCameraH264_);

        break;
      }
      case 'b':
      {
        setupCameraH264_.request.request_view = setupCameraH264_.request.MAIN_CAMERA;
        setupCameraH264_.request.start        = 1;
        setup_camera_h264_client.call(setupCameraH264_);

        break;
      }
      case 'c':
      {
        setupCameraH264_.request.request_view = setupCameraH264_.request.VICE_CAMERA;
        setupCameraH264_.request.start        = 1;
        setup_camera_h264_client.call(setupCameraH264_);

        break;
      }
      case 'd':
      {
        setupCameraH264_.request.request_view = setupCameraH264_.request.TOP_CAMERA;
        setupCameraH264_.request.start        = 1;
        setup_camera_h264_client.call(setupCameraH264_);

        break; 
      }
    }

    ros::AsyncSpinner spinner(1);
    spinner.start();
    ROS_INFO("Wait 10 second to record stream");
    ros::Duration(10).sleep();

    switch (inputChar)
    {
      case 'a':
      {
        setupCameraH264_.request.request_view = setupCameraH264_.request.FPV_CAMERA;
        setupCameraH264_.request.start        = 0;
        setup_camera_h264_client.call(setupCameraH264_);

        break;
      }
      case 'b':
      {
        setupCameraH264_.request.request_view = setupCameraH264_.request.MAIN_CAMERA;
        setupCameraH264_.request.start        = 0;
        setup_camera_h264_client.call(setupCameraH264_);

        break;
      }
      case 'c':
      {
        setupCameraH264_.request.request_view = setupCameraH264_.request.VICE_CAMERA;
        setupCameraH264_.request.start        = 0;
        setup_camera_h264_client.call(setupCameraH264_);

        break;
      }
      case 'd':
      {
        setupCameraH264_.request.request_view = setupCameraH264_.request.TOP_CAMERA;
        setupCameraH264_.request.start        = 0;
        setup_camera_h264_client.call(setupCameraH264_);

        break;
      }
    }

    ROS_INFO_STREAM("Finished. Press CTRL-C to terminate the node");
    ros::waitForShutdown();
    return 0;
}