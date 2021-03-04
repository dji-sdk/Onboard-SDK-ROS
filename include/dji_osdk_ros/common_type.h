/** @file common_type.hpp
 *  @version 4.0
 *  @date May 2020
 *
 *  @brief common type of osdk ros
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
#ifndef __COMMON_TYPE_HH__
#define __COMMON_TYPE_HH__

// Header include
#include <dji_type.hpp>
#include <vector>

// Declaration
namespace dji_osdk_ros
{
  struct MoveOffset
  {
    MoveOffset(double x_in, double y_in, double z_in,
               double yaw_in, double pos_threshold_in = 0.5, double yaw_threshold_in = 1.0)
      : x(x_in),
        y(y_in),
        z(z_in),
        yaw(yaw_in),
        pos_threshold(pos_threshold_in),
        yaw_threshold(yaw_threshold_in)
    {}
    double x;
    double y;
    double z;
    double yaw;
    double pos_threshold;
    double yaw_threshold;
  };

  struct JoystickMode
  {
    uint8_t horizontalLogic;
    uint8_t verticalLogic;
    uint8_t yawLogic;
    uint8_t horizontalCoordinate;
    uint8_t stableMode;
  };

  struct JoystickCommand
  {
    DJI::OSDK::float32_t x;   /*!< Control with respect to the x axis of the
                        DJI::OSDK::Control::HorizontalCoordinate.*/
    DJI::OSDK::float32_t y;   /*!< Control with respect to the y axis of the
                        DJI::OSDK::Control::HorizontalCoordinate.*/
    DJI::OSDK::float32_t z;   /*!< Control with respect to the z axis, up is positive. */
    DJI::OSDK::float32_t yaw; /*!< Yaw position/velocity control w.r.t. the ground frame.*/
  };

#pragma pack(1)
  /*! the type of error code list in HMS's raw pushing data*/
  typedef  struct ErrList{
      uint32_t alarmID;     /*! error code*/
      uint8_t  sensorIndex; /*! fault sensor's index*/
      uint8_t  reportLevel; /*! fault level ,0-4,0 is no error,4 is highest*/
  } ErrList;

  /*! the type of HMS's raw pushing data*/
  typedef struct HMSPushData {
      uint8_t msgVersion;           /*! version of message.default:0*/
      uint8_t globalIndex;          /*! cycle message sequence number*/
      uint8_t msgEnd : 1;           /*! end flag bit of message subcontract push. last package of subcontracting push is 1.*/
      uint8_t msgIndex : 7;         /*! Message serial number*/
      std::vector<ErrList> errList; /*! error code list in each pushing*/
  } HMSPushData;
#pragma pack()

  /*! the type of HMS's pushing data with a time stamp*/
  typedef struct HMSPushPacket
  {
    HMSPushData  hmsPushData; /*! HMS's raw pushing data*/
    uint32_t     timeStamp;   /*! timestamp of the packet*/
  } HMSPushPacket;

  struct RotationAngle
  {
    DJI::OSDK::float32_t roll;
    DJI::OSDK::float32_t pitch;
    DJI::OSDK::float32_t yaw;
  };

  enum class Dronetype
  {
      M100 = 0,
      M210V2 = 1,
      M300 = 2,
      M600 = 3,
      INVALID_TYPE = 0xFF,
  };

  enum class TelemetryType
  {
      USE_ROS_BROADCAST = 0,
      USE_ROS_SUBSCRIBE = 1,
  };

  enum class PayloadIndex {
      PAYLOAD_INDEX_0 = 0x00,
      PAYLOAD_INDEX_1 = 0x01,
      PAYLOAD_INDEX_2 = 0x02,
      PAYLOAD_INDEX_CNT = 0x03,
      PAYLOAD_INDEX_INVALID = 0x03,
  };

  enum class AlignStatus
  {
      UNALIGNED,
      ALIGNING,
      ALIGNED
  };

  typedef struct GimbalSingleData
  {
    DJI::OSDK::float32_t pitch;
    DJI::OSDK::float32_t roll;
    DJI::OSDK::float32_t yaw;
    uint32_t status;
    uint8_t mode;
  } GimbalSingleData;

  enum class SubscribePackgeIndex
  {
      TEMP_SUB_PACKAGE_INDEX                        = 0,
      BROADCAST_BUT_NEED_SUBSCRIBE                  = 1,
      PACKAGE_ID_5HZ   = 1,
      PACKAGE_ID_50HZ  = 2,
      PACKAGE_ID_100HZ = 3,
      PACKAGE_ID_400HZ = 4,
  };

  /*! for gimbal */
  #pragma pack(1)
  typedef struct GimbalRotationData {
      /*! rotation cooradiration
       *  0 = execute angle command based on the previously set reference point
       *  1 = execute angle command based on the current point
       */
      uint8_t rotationMode;
      /*! pitch angle in degree, unit : deg
       */
      float pitch;
      /*! roll angle in degree, unit : deg
       */
      float roll;
      /*! yaw angle in degree, unit : deg
       */
      float yaw;
      /*! execution time, unit : second
       */
      double time;
  } Rotation;
  #pragma pack()

  /*! for camera */
/*! @brief CameraModule exposure compensation.
*/
enum class ExposureCompensation {
    /*! The camera's exposure compensation is -5.0ev.*/
    N_5_0 = 1,
    /*! The camera's exposure compensation is -4.7ev.*/
    N_4_7 = 2,
    /*! The camera's exposure compensation is -4.3ev.*/
    N_4_3 = 3,
    /*! The camera's exposure compensation is -4.0ev.*/
    N_4_0 = 4,
    /*! The camera's exposure compensation is -3.7ev.*/
    N_3_7 = 5,
    /*! The camera's exposure compensation is -3.3ev.*/
    N_3_3 = 6,
    /*! The camera's exposure compensation is -3.0ev.*/
    N_3_0 = 7,
    /*! The camera's exposure compensation is -2.7ev.*/
    N_2_7 = 8,
    /*! The camera's exposure compensation is -2.3ev.*/
    N_2_3 = 9,
    /*! The camera's exposure compensation is -2.0ev.*/
    N_2_0 = 10,
    /*! The camera's exposure compensation is -1.7ev.*/
    N_1_7 = 11,
    /*! The camera's exposure compensation is -1.3ev.*/
    N_1_3 = 12,
    /*! The camera's exposure compensation is -1.0ev.*/
    N_1_0 = 13,
    /*! The camera's exposure compensation is -0.7ev.*/
    N_0_7 = 14,
    /*! The camera's exposure compensation is -0.3ev.*/
    N_0_3 = 15,
    /*! The camera's exposure compensation is 0.0ev.*/
    N_0_0 = 16,
    /*! The camera's exposure compensation is +0.3ev.*/
    P_0_3 = 17,
    /*! The camera's exposure compensation is +0.7ev.*/
    P_0_7 = 18,
    /*! The camera's exposure compensation is +1.0ev.*/
    P_1_0 = 19,
    /*! The camera's exposure compensation is +1.3ev.*/
    P_1_3 = 20,
    /*! The camera's exposure compensation is +1.7ev.*/
    P_1_7 = 21,
    /*! The camera's exposure compensation is +2.0ev.*/
    P_2_0 = 22,
    /*! The camera's exposure compensation is +2.3ev.*/
    P_2_3 = 23,
    /*! The camera's exposure compensation is +2.7ev.*/
    P_2_7 = 24,
    /*! The camera's exposure compensation is +3.0ev.*/
    P_3_0 = 25,
    /*! The camera's exposure compensation is +3.3ev.*/
    P_3_3 = 26,
    /*! The camera's exposure compensation is +3.7ev.*/
    P_3_7 = 27,
    /*! The camera's exposure compensation is +4.0ev.*/
    P_4_0 = 28,
    /*! The camera's exposure compensation is +4.3ev.*/
    P_4_3 = 29,
    /*! The camera's exposure compensation is +4.7ev.*/
    P_4_7 = 30,
    /*! The camera's exposure compensation is +5.0ev.*/
    P_5_0 = 31,
    /*! The camera's exposure compensation is fixed by the camera.*/
    FIXED = 0xFF,
    /*! The camera's exposure compensation is unknown. */
    UNKNOWN = 0xFFFF,
};

  /*! @brief the photo action of INTERVAL shooting photo mode
    */
  enum class ExposureMode {
      PROGRAM_AUTO = 1,       /*!< Program mode */
      SHUTTER_PRIORITY = 2,   /*!< Shutter priority mode */
      APERTURE_PRIORITY = 3,  /*!< Aperture priority mode */
      EXPOSURE_MANUAL = 4,    /*!< Manual mode */
      EXPOSURE_UNKNOWN = 0xFF /*!< The camera exposure mode is unknown. */
  };

  /*! @brief CameraModule ISO values.
  */
  enum class ISO {
      /*! The ISO value is automatically set. This cannot be used for all cameras
         when in Manual mode. */
      ISO_AUTO = 0,
      /*!  The ISO value is set to 100. */
      ISO_100 = 3,
      /*! The ISO value is set to 100. */
      ISO_200 = 4,
      /*! The ISO value is set to 100.*/
      ISO_400 = 5,
      /*! The ISO value is set to 100.*/
      ISO_800 = 6,
      /*! The ISO value is set to 100.*/
      ISO_1600 = 7,
      /*! The ISO value is set to 100.*/
      ISO_3200 = 8,
      /*! The ISO value is set to 100.*/
      ISO_6400 = 9,
      /*! The ISO value is set to 100.*/
      ISO_12800 = 10,
      /*! The ISO value is set to 100.*/
      ISO_25600 = 11,
      /*! ISO value is fixed by the camera firmware. When the camera color is set
       to D_LOG, camera will fix the ISO to a specific value in order to optimize
       the performance.
       */
      ISO_FIXED = 255,
      /*! The ISO value is set to Unknown value. */
      ISO_UNKNOWN = 0xFFFF,
  };

  /*! @brief CameraModule shutter speed values.
   */
  enum class ShutterSpeed {
      SHUTTER_SPEED_1_8000 = 0,     /*!< 1/8000 s */
      SHUTTER_SPEED_1_6400 = 1,     /*!< 1/6400 s */
      SHUTTER_SPEED_1_6000 = 2,     /*!< 1/6000 s */
      SHUTTER_SPEED_1_5000 = 3,     /*!< 1/5000 s */
      SHUTTER_SPEED_1_4000 = 4,     /*!< 1/4000 s */
      SHUTTER_SPEED_1_3200 = 5,     /*!< 1/3200 s */
      SHUTTER_SPEED_1_3000 = 6,     /*!< 1/3000 s */
      SHUTTER_SPEED_1_2500 = 7,     /*!< 1/2500 s */
      SHUTTER_SPEED_1_2000 = 8,     /*!< 1/2000 s */
      SHUTTER_SPEED_1_1600 = 9,     /*!< 1/1600 s */
      SHUTTER_SPEED_1_1500 = 10,    /*!< 1/1500 s */
      SHUTTER_SPEED_1_1250 = 11,    /*!< 1/1250 s */
      SHUTTER_SPEED_1_1000 = 12,    /*!< 1/1000 s */
      SHUTTER_SPEED_1_800 = 13,     /*!< 1/800 s */
      SHUTTER_SPEED_1_725 = 14,     /*!< 1/725 s */
      SHUTTER_SPEED_1_640 = 15,     /*!< 1/640 s */
      SHUTTER_SPEED_1_500 = 16,     /*!< 1/500 s */
      SHUTTER_SPEED_1_400 = 17,     /*!< 1/400 s */
      SHUTTER_SPEED_1_350 = 18,     /*!< 1/350 s */
      SHUTTER_SPEED_1_320 = 19,     /*!< 1/320 s */
      SHUTTER_SPEED_1_250 = 20,     /*!< 1/250 s */
      SHUTTER_SPEED_1_240 = 21,     /*!< 1/240 s */
      SHUTTER_SPEED_1_200 = 22,     /*!< 1/200 s */
      SHUTTER_SPEED_1_180 = 23,     /*!< 1/180 s */
      SHUTTER_SPEED_1_160 = 24,     /*!< 1/160 s */
      SHUTTER_SPEED_1_125 = 25,     /*!< 1/125 s */
      SHUTTER_SPEED_1_120 = 26,     /*!< 1/120 s */
      SHUTTER_SPEED_1_100 = 27,     /*!< 1/100 s */
      SHUTTER_SPEED_1_90 = 28,      /*!< 1/90 s */
      SHUTTER_SPEED_1_80 = 29,      /*!< 1/80 s */
      SHUTTER_SPEED_1_60 = 30,      /*!< 1/60 s */
      SHUTTER_SPEED_1_50 = 31,      /*!< 1/50 s */
      SHUTTER_SPEED_1_40 = 32,      /*!< 1/40 s */
      SHUTTER_SPEED_1_30 = 33,      /*!< 1/30 s */
      SHUTTER_SPEED_1_25 = 34,      /*!< 1/25 s */
      SHUTTER_SPEED_1_20 = 35,      /*!< 1/20 s */
      SHUTTER_SPEED_1_15 = 36,      /*!< 1/15 s */
      SHUTTER_SPEED_1_12DOT5 = 37,  /*!< 1/12.5 s */
      SHUTTER_SPEED_1_10 = 38,      /*!< 1/10 s */
      SHUTTER_SPEED_1_8 = 39,       /*!< 1/8 s */
      SHUTTER_SPEED_1_6DOT25 = 40,  /*!< 1/6.25 s */
      SHUTTER_SPEED_1_5 = 41,       /*!< 1/5 s */
      SHUTTER_SPEED_1_4 = 42,       /*!< 1/4 s */
      SHUTTER_SPEED_1_3 = 43,       /*!< 1/3 s */
      SHUTTER_SPEED_1_2DOT5 = 44,   /*!< 1/2.5 s */
      SHUTTER_SPEED_1_2 = 45,       /*!< 1/2 s */
      SHUTTER_SPEED_1_1DOT67 = 46,  /*!< 1/1.67 s */
      SHUTTER_SPEED_1_1DOT25 = 47,  /*!< 1/1.25 s */
      SHUTTER_SPEED_1 = 48,         /*!< 1.0 s */
      SHUTTER_SPEED_1DOT3 = 49,     /*!< 1.3 s */
      SHUTTER_SPEED_1DOT6 = 50,     /*!< 1.6 s */
      SHUTTER_SPEED_2 = 51,         /*!< 2.0 s */
      SHUTTER_SPEED_2DOT5 = 52,     /*!< 2.5 s */
      SHUTTER_SPEED_3 = 53,         /*!< 3.0 s */
      SHUTTER_SPEED_3DOT2 = 54,     /*!< 3.2 s */
      SHUTTER_SPEED_4 = 55,         /*!< 4.0 s */
      SHUTTER_SPEED_5 = 56,         /*!< 5.0 s */
      SHUTTER_SPEED_6 = 57,         /*!< 6.0 s */
      SHUTTER_SPEED_7 = 58,         /*!< 7.0 s */
      SHUTTER_SPEED_8 = 59,         /*!< 8.0 s */
      SHUTTER_SPEED_9 = 60,         /*!< 9.0 s */
      SHUTTER_SPEED_10 = 61,        /*!< 10.0 s */
      SHUTTER_SPEED_13 = 62,        /*!< 13.0 s */
      SHUTTER_SPEED_15 = 63,        /*!< 15.0 s */
      SHUTTER_SPEED_20 = 64,        /*!< 20.0 s */
      SHUTTER_SPEED_25 = 65,        /*!< 25.0 s */
      SHUTTER_SPEED_30 = 66,        /*!< 30.0 s */
      SHUTTER_SPEED_UNKNOWN = 0xFF, /*!< Unknown */
  };

  /*! @brief CameraModule aperture values.
  *  @note X5, X5R, Z30, Phantom 4 Pro camera, X4S and X5S support this
  * setting.
  */
  enum class Aperture {
      /*! 	The Aperture value is f/1.6. It is only supported by Z30
         camera.*/
      F_1_DOT_6 = 160,
      /*! The Aperture value is f/1.7.*/
      F_1_DOT_7 = 170,
      /*! The Aperture value is f/1.8.*/
      F_1_DOT_8 = 180,
      /*! The Aperture value is f/2.*/
      F_2 = 200,
      /*! The Aperture value is f/2.2.*/
      F_2_DOT_2 = 220,
      /*! The Aperture value is f/2.4. It is only supported by Z30 camera.*/
      F_2_DOT_4 = 240,
      /*! The Aperture value is f/2.5.*/
      F_2_DOT_5 = 250,
      /*! The Aperture value is f/2.6.*/
      F_2_DOT_6 = 260,
      /*! The Aperture value is f/2.8.*/
      F_2_DOT_8 = 280,
      /*! The Aperture value is f/3.2.*/
      F_3_DOT_2 = 320,
      /*! The Aperture value is f/3.4.*/
      F_3_DOT_4 = 340,
      /*! The Aperture value is f/3.5.*/
      F_3_DOT_5 = 350,
      /*! The Aperture value is f/4.*/
      F_4 = 400,
      /*! The Aperture value is f/4.5.*/
      F_4_DOT_5 = 450,
      /*! The Aperture value is f/4.8.*/
      F_4_DOT_8 = 480,
      /*! The Aperture value is f/5.*/
      F_5 = 500,
      /*! The Aperture value is f/5.6.*/
      F_5_DOT_6 = 560,
      /*! The Aperture value is f/6.3.*/
      F_6_DOT_3 = 630,
      /*! The Aperture value is f/6.8.*/
      F_6_DOT_8 = 680,
      /*! The Aperture value is f/7.1.*/
      F_7_DOT_1 = 710,
      /*! The Aperture value is f/8.*/
      F_8 = 800,
      /*! The Aperture value is f/9.*/
      F_9 = 900,
      /*! The Aperture value is f/9.6.*/
      F_9_DOT_6 = 960,
      /*! The Aperture value is f/10.*/
      F_10 = 1000,
      /*! The Aperture value is f/11.*/
      F_11 = 1100,
      /*! The Aperture value is f/13.*/
      F_13 = 1300,
      /*! The Aperture value is f/14.*/
      F_14 = 1400,
      /*! The Aperture value is f/16.*/
      F_16 = 1600,
      /*! The Aperture value is f/18.*/
      F_18 = 1800,
      /*! The Aperture value is f/19.*/
      F_19 = 1900,
      /*! The Aperture value is f/20.*/
      F_20 = 2000,
      /*! The Aperture value is f/22.*/
      F_22 = 2200,
      /*! The Aperture value is Unknown. */
      F_UNKNOWN = 0xFFFF,
  };

enum class PhotoBurstCount {
    /*! The number of pictures to continuously take each time in BURST mode is 2
     */
    BURST_COUNT_2 = 2,
    /*! The number of pictures to continuously take each time in BURST mode is 3
     */
    BURST_COUNT_3 = 3,
    /*! The number of pictures to continuously take each time in BURST mode is 5
     */
    BURST_COUNT_5 = 5,
    /*! The number of pictures to continuously take each time in BURST mode is 7
     */
    BURST_COUNT_7 = 7,
    /*! The number of pictures to continuously take at one time in BURST mode is
     * 10, Only supported by X4S camera, X5S camera and Phantom 4 Pro camera.
     */
    BURST_COUNT_10 = 10,
    /*! The number of pictures to continuously take at one time in BURST mode is
     * 14, Only supported by X4S camera, X5S camera and Phantom 4 Pro camera.
     */
    BURST_COUNT_14 = 14,
    /*!	The camera burst shoot count value is unknown.
     */
    BURST_COUNT_KNOWN = 0xFF,
};

  /*! @brief the photo action of INTERVAL shooting photo mode
   */
  enum class PhotoAEBCount {
      /*! The number of pictures to continuously take at one time in AEB mode is 3
       */
      AEB_COUNT_3 = 3,
      /*! The number of pictures to continuously take at one time in AEB mode is 5
       */
      AEB_COUNT_5 = 5,
      /*! The number of pictures to continuously take at one time in AEB mode is 7
       */
      AEB_COUNT_7 = 7,
      /*! The number of pictures to continuously take at one time in AEB mode is
       * unknown.
       */
      AEB_COUNT_KNOWN = 0xFF,
  };

  enum ZoomDirection {
      /*! Lens will zoom out.The focal length decreases,
          field of view becomes wider and magnification is lower. */
      ZOOM_OUT = 0,
      /*! Lens will zoom in.The focal length increases,
        field of view becomes narrower and magnification is
        higher */
      ZOOM_IN = 1,
  };

  enum ZoomSpeed {
      SLOWEST = 72,         /*!< slowest speed */
      SLOW = 73,            /*!< slow speed */
      MODERATELY_SLOW = 74, /*!< slightly slower than normal speed */
      NORMAL = 75,          /*!< normal speed */
      MODERATELY_FAST = 76, /*!< speed slightly faster than normal speed */
      FAST = 77,            /*!< fast speed */
      FASTEST = 78,         /*!< fastest speed */
  };

  #pragma pack(1)
  typedef struct PhotoIntervalData {
      uint8_t photoNumConticap; /*!< 0:reserve 1~254:number 255:keep capturing
                                   till stop */
      int16_t timeInterval;     /*!< time interval (second) */
  } PhotoIntervalData;
  #pragma pack()
// For zoom
  #pragma pack(1)
  typedef struct ZoomConfig
  {
    uint8_t digital_zoom_mode   : 2;  /* 0:step 1:position 2:continuous */
    uint8_t digital_reserve   : 1;  /* reserve */
    uint8_t digital_zoom_enable  : 1;  /* 0:not_ctrl 1:ctrl */
    uint8_t optical_zoom_mode    : 2;  /* 0:step 1:position 2:continuous */
    uint8_t optical_reserve    : 1;  /* reserve */
    uint8_t optical_zoom_enable  : 1;  /* 0:not_ctrl 1:ctrl */
  } zoom_config_t;

  typedef union zoom_param
  {
    struct
    {
      uint16_t zoom_cont_speed        : 8;  /* continuous speed 0~100 */
      uint16_t zoom_cont_direction    : 1;
      uint16_t zoom_cont_reserve      : 7;
    }cont_param;
    struct
    {
      uint16_t zoom_step_level    : 8;  /* level time * 100 = 1 times */
      uint16_t zoom_step_direction    : 1;
      uint16_t zoom_step_reserve      : 7;
    }step_param;
    struct
    {
      uint16_t zoom_pos_level;            /* 180 = 1.8times */
    }pos_param;
  } zoom_param;

  typedef struct camera_zoom_data_type
  {
    uint8_t func_index;
    uint8_t cam_index;
    zoom_config_t zoom_config;
    zoom_param optical_zoom_param;
    zoom_param digital_zoom_param;
  } CameraZoomDataType;
#pragma pack()

  //for advanced sensing
  #pragma pack(1)
  /*!
   * @brief This struct provides an interface for user to determine
   * what images to subscribe to
   */
  typedef struct ImageSelection {
    uint16_t front_left       : 1;
    uint16_t front_right      : 1;
    uint16_t down_front       : 1;
    uint16_t down_back        : 1;
    uint16_t back_left        : 1;
    uint16_t back_right       : 1;
    uint16_t reserved         : 10;
  } ImageSelection;
  #pragma pack()

  /*!
   * @brief This struct is used to pair img data with camera
   */
  enum class ReceivedImgDesc
  {
    RECV_FRONT_LEFT   = 10,
    RECV_FRONT_RIGHT  = 11,
    RECV_DOWN_BACK    = 0,
    RECV_DOWN_FRONT   = 1,
    RECV_FRONT_DEPTH  = 15,
  };

}

#endif // __COMMON_TYPE_HH__

