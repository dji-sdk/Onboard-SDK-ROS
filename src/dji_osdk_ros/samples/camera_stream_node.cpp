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
#include <iostream>

#include <dji_osdk_ros/SetupCameraH264.h>
#include <sensor_msgs/Image.h>

extern "C"{
#include <libavcodec/avcodec.h>
#include <libavformat/avformat.h>
#include <libswscale/swscale.h>
}

#ifdef OPEN_CV_INSTALLED
#include "opencv2/opencv.hpp"
#include "opencv2/highgui/highgui.hpp"
#endif


//CODE
using namespace dji_osdk_ros;

AVCodecContext*       pCodecCtx;
AVCodec*              pCodec;
AVCodecParserContext* pCodecParserCtx;
SwsContext*           pSwsCtx;
AVFrame* pFrameYUV;
AVFrame* pFrameRGB;
uint8_t* rgbBuf;
size_t   bufSize;

bool ffmpeg_init()
{
    avcodec_register_all();
    pCodecCtx = avcodec_alloc_context3(NULL);
    if (!pCodecCtx)
    {
        return false;
    }

    pCodecCtx->thread_count = 4;
    pCodec = avcodec_find_decoder(AV_CODEC_ID_H264);
    if (!pCodec || avcodec_open2(pCodecCtx, pCodec, NULL) < 0)
    {
        return false;
    }

    pCodecParserCtx = av_parser_init(AV_CODEC_ID_H264);
    if (!pCodecParserCtx)
    {
        return false;
    }

    pFrameYUV = av_frame_alloc();
    if (!pFrameYUV)
    {
        return false;
    }

    pFrameRGB = av_frame_alloc();
    if (!pFrameRGB)
    {
        return false;
    }

    pSwsCtx = NULL;

    pCodecCtx->flags2 |= AV_CODEC_FLAG2_SHOW_ALL;
}

void show_rgb(uint8_t *rawData, int height, int width)
{
#ifdef OPEN_CV_INSTALLED
    cv::Mat mat(height, width, CV_8UC3, rawData, width*3);
    cv::cvtColor(mat, mat, cv::COLOR_RGB2BGR);
    cv::imshow("camera_stream_node", mat);
    cv::waitKey(1);
#endif
}

void decodeToDisplay(uint8_t *buf, int bufLen)
{
    uint8_t* pData   = buf;
    int remainingLen = bufLen;
    int processedLen = 0;

    AVPacket pkt;
    av_init_packet(&pkt);
    while (remainingLen > 0)
    {
        processedLen = av_parser_parse2(pCodecParserCtx, pCodecCtx,
                                        &pkt.data, &pkt.size,
                                        pData, remainingLen,
                                        AV_NOPTS_VALUE, AV_NOPTS_VALUE, AV_NOPTS_VALUE);
        remainingLen -= processedLen;
        pData        += processedLen;

        if (pkt.size > 0)
        {
            int gotPicture = 0;
            avcodec_decode_video2(pCodecCtx, pFrameYUV, &gotPicture, &pkt);

            if (!gotPicture)
            {
                //DSTATUS_PRIVATE("Got Frame, but no picture\n");
                continue;
            }
            else
            {
                int w = pFrameYUV->width;
                int h = pFrameYUV->height;
                //DSTATUS_PRIVATE("Got picture! size=%dx%d\n", w, h);

                if(NULL == pSwsCtx)
                {
                    pSwsCtx = sws_getContext(w, h, pCodecCtx->pix_fmt,
                                             w, h, AV_PIX_FMT_RGB24,
                                             4, NULL, NULL, NULL);
                }

                if(NULL == rgbBuf)
                {
                    bufSize = avpicture_get_size(AV_PIX_FMT_RGB24, w, h);
                    rgbBuf = (uint8_t*) av_malloc(bufSize);
                    avpicture_fill((AVPicture*)pFrameRGB, rgbBuf, AV_PIX_FMT_RGB24, w, h);
                }

                if(NULL != pSwsCtx && NULL != rgbBuf)
                {
                    sws_scale(pSwsCtx,
                              (uint8_t const *const *) pFrameYUV->data, pFrameYUV->linesize, 0, pFrameYUV->height,
                              pFrameRGB->data, pFrameRGB->linesize);

                    pFrameRGB->height = h;
                    pFrameRGB->width = w;

#ifdef OPEN_CV_INSTALLED
                    cv::Mat mat(pFrameRGB->height, pFrameRGB->width, CV_8UC3, pFrameRGB->data[0], pFrameRGB->width * 3);
                    cv::cvtColor(mat, mat, cv::COLOR_RGB2BGR);
                    cv::imshow("camera_stream_node", mat);
                    cv::waitKey(1);
#endif
                }
            }
        }
    }
    av_free_packet(&pkt);
}

void cameraH264CallBack(const sensor_msgs::Image& msg)
{
  printf("msg.data.size() = %lu\n", msg.data.size());
  decodeToDisplay((uint8_t *)&msg.data[0], msg.data.size());
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "camera_stream_node");
    ros::NodeHandle nh;
    auto setup_camera_h264_client = nh.serviceClient<dji_osdk_ros::SetupCameraH264>("setup_camera_h264");
    auto fpv_camera_h264_sub = nh.subscribe("dji_osdk_ros/camera_h264_stream", 10, cameraH264CallBack);
    dji_osdk_ros::SetupCameraH264 setupCameraH264_;
    ffmpeg_init();

    char inputChar = 0;
    std::cout << std::endl;
    std::cout
        << "| Available commands:                                          |"
        << std::endl
        << "| [a] Start to display FPV stream sample                       |"
        << std::endl
        << "| [b] Start to display main camera stream sample               |"
        << std::endl
        << "| [c] Start to display vice camera stream sample               |"
        << std::endl
        << "| [d] Start to display top camera stream sample                |"
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