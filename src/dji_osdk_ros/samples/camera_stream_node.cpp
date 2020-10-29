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
#include <dji_camera_image.hpp>
#include <dji_osdk_ros/SetupCameraStream.h>
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

#ifdef SDL2_INSTALLED
#include <SDL2/SDL.h>
void sdl_show_rgb(uint8_t *rgb24Buf, int width, int height) {
  static SDL_Renderer* sdlRenderer = NULL;
  static SDL_Texture* sdlTexture = NULL;
  static SDL_Window *screen = NULL;
  static int initFlag = 0;

  if (initFlag == 0) {
    initFlag = 1;
    if(SDL_Init(SDL_INIT_VIDEO)) {
      printf( "Could not initialize SDL - %s\n", SDL_GetError());
      return;
    }


    //SDL 2.0 Support for multiple windows
    screen = SDL_CreateWindow("camera_stream_node", SDL_WINDOWPOS_UNDEFINED, SDL_WINDOWPOS_UNDEFINED,
                              100, 100,SDL_WINDOW_OPENGL|SDL_WINDOW_SHOWN|SDL_WINDOW_RESIZABLE);
    if(!screen) {
      printf("SDL: could not create window - exiting:%s\n",SDL_GetError());
      return;
    }
    sdlRenderer = SDL_CreateRenderer(screen, -1, 0);
    uint32_t pixformat = SDL_PIXELFORMAT_RGB24;
    sdlTexture = SDL_CreateTexture(sdlRenderer,pixformat, SDL_TEXTUREACCESS_STREAMING, width, height);
  }

  if (!sdlRenderer || !sdlTexture || !screen) return;
  SDL_SetWindowSize(screen, width, height);

  SDL_Event event;
  event.type = (SDL_USEREVENT + 1);
  SDL_PushEvent(&event);
  if (SDL_WaitEventTimeout(&event, 5)) {
    SDL_Rect sdlRect;
    SDL_UpdateTexture(sdlTexture, NULL, rgb24Buf, width * 3);
    sdlRect.x = 0;
    sdlRect.y = 0;
    sdlRect.w = width;
    sdlRect.h = height;

    SDL_RenderClear(sdlRenderer);
    SDL_RenderCopy(sdlRenderer, sdlTexture, NULL, &sdlRect);
    SDL_RenderPresent(sdlRenderer);
  }
}
#endif

void show_rgb(CameraRGBImage img, char* name)
{
  std::cout << "#### Got image from:\t" << std::string(name) << std::endl;
#ifdef SDL2_INSTALLED
  sdl_show_rgb(img.rawData.data(), img.width, img.height);
#elif defined(OPEN_CV_INSTALLED)
  cv::Mat mat(img.height, img.width, CV_8UC3, img.rawData.data(), img.width*3);
  cvtColor(mat, mat, cv::COLOR_RGB2BGR);
  imshow(name,mat);
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
#ifdef SDL2_INSTALLED
                    sdl_show_rgb(pFrameRGB->data[0], pFrameRGB->width, pFrameRGB->height);
#elif defined(OPEN_CV_INSTALLED)
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


void fpvCameraStreamCallBack(const sensor_msgs::Image& msg)
{
  CameraRGBImage img;
  img.rawData = msg.data;
  img.height  = msg.height;
  img.width   = msg.width;
  char Name[] = "FPV_CAM";
  show_rgb(img, Name);
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
  show_rgb(img, Name);
  std::cout<<"height is"<<msg.height<<std::endl;
  std::cout<<"width is"<<msg.width<<std::endl;
}


void cameraH264CallBack(const sensor_msgs::Image& msg)
{
  decodeToDisplay((uint8_t *)&msg.data[0], msg.data.size());
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "camera_stream_node");
    ros::NodeHandle nh;

    /*! H264 flow init and H264 decoder init */
    auto setup_camera_h264_client = nh.serviceClient<dji_osdk_ros::SetupCameraH264>("setup_camera_h264");
    auto fpv_camera_h264_sub = nh.subscribe("dji_osdk_ros/camera_h264_stream", 10, cameraH264CallBack);
    dji_osdk_ros::SetupCameraH264 setupCameraH264_;
    ffmpeg_init();

    /*! RGB flow init */
    auto setup_camera_stream_client = nh.serviceClient<dji_osdk_ros::SetupCameraStream>("setup_camera_stream");
    auto fpv_camera_stream_sub = nh.subscribe("dji_osdk_ros/fpv_camera_images", 10, fpvCameraStreamCallBack);
    auto main_camera_stream_sub = nh.subscribe("dji_osdk_ros/main_camera_images", 10, mainCameraStreamCallBack);
    dji_osdk_ros::SetupCameraStream setupCameraStream_;

#ifndef SDL2_INSTALLED
  std::cout
      << "--Recommandation : It is found that using \"cv::imshow\" will cause more CPU resources and more processing "
         "time. Using SDL to display images can improve this situation. At present, SDL display is supported in this "
         "node, which can be used by installing SDL2 library and recompiling. \n"
      << "--Install SDL2 library in shell : \"sudo apt-get install libsdl2-dev\"."
      << std::endl;
#endif
#ifdef SDL2_INSTALLED
  std::cout << "Using SDL2 lib to display the images." << std::endl;
#elif defined(OPEN_CV_INSTALLED)
  std::cout << "Using Opencv to display the images." << std::endl;
#endif
    char inputChar = 0;
    std::cout << std::endl;
    std::cout
        << "| Input 'm' or 'f' to view the camera stream from the RGB flow |\n"
           "| of the topic \"dji_osdk_ros/fpv_camera_images\" and            |\n"
           "| \"dji_osdk_ros/main_camera_images\" :                          |"
        << std::endl
        << "| [m] Start to display main camera stream                      |"
        << std::endl
        << "| [f] Start to display fpv stream                              |"
        << std::endl;
    std::cout
        << "|                                                              |"
        << std::endl;
    std::cout
        << "| Input 'a' ~ 'b' to view the camera stream decoded by the h264|\n"
           "| flow from the topic \"dji_osdk_ros/camera_h264_stream\" :      |"
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

    switch (inputChar) 
    {
      case 'f':
      {
        setupCameraStream_.request.cameraType = setupCameraStream_.request.FPV_CAM;
        setupCameraStream_.request.start = 1;
        setup_camera_stream_client.call(setupCameraStream_);

        break;
      }
      case 'm':
      {
        setupCameraStream_.request.cameraType = setupCameraStream_.request.MAIN_CAM;
        setupCameraStream_.request.start = 1;
        setup_camera_stream_client.call(setupCameraStream_);

        break;
      }
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
    ros::Duration(20).sleep();

    switch (inputChar)
    {
      case 'f':
      {
        setupCameraStream_.request.cameraType = setupCameraStream_.request.FPV_CAM;
        setupCameraStream_.request.start = 0;
        setup_camera_stream_client.call(setupCameraStream_);

        break;
      }
      case 'm':
      {
        setupCameraStream_.request.cameraType = setupCameraStream_.request.MAIN_CAM;
        setupCameraStream_.request.start = 0;
        setup_camera_stream_client.call(setupCameraStream_);

        break;
      }
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
