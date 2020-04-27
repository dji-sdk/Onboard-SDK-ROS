#ifndef __VEHICLE_WRAPPER_HH__
#define __VEHICLE_WRAPPER_HH__
/**-----------------------------------------------
  * @Copyright (C) 2020 All rights reserved.
  * @date: 2020
  * @file: vehicle_wrapper.hh
  * @version: v0.0.1
  * @author: kevin.hoo@dji.com
  * @create_date: 2020-03-08 18:06:28
  * @last_modified_date: 2020-03-08 20:01:12
  * @brief: TODO
  * @details: TODO
  *-----------------------------------------------*/

// Header include
#include <dji_setup_helpers.hpp>
#include <dji_vehicle.hpp>
#include <dji_osdk_ros/common_type.h>
#include <string>
#include <cmath>
#include <vector>
#ifdef ADVANCED_SENSING
#include <dji_camera_image.hpp>
#endif
#include <mutex>

const double C_EARTH = 6378137.0;
const double DEG2RAD = 0.01745329252;
// Declaration
namespace dji_osdk_ros
{
  using namespace DJI::OSDK;
  using namespace Telemetry;
  class VehicleWrapper : private Setup
  {
    public:
      VehicleWrapper(int app_id,
                     const std::string& env_key,
                     const std::string& dev_acm_name,
                     const std::string& dev_name,
                     unsigned int baud_rate,
                     bool enableAdvancedSensing = false);
      virtual ~VehicleWrapper() = default;
    
    public:
      void setupEnvironment(bool enable_advanced_sensing);
      bool initVehicle();

    public:
      /*! Parts of Camera */
      bool setEV(const PayloadIndex& payloadIndex, const ExposureCompensation& exposureCompensation);
      bool setExposureMode(const PayloadIndex& index, const ExposureMode& dataTarget);
      bool setISO(const PayloadIndex& payloadIndex, const ISO& ios);
      bool setShutterSpeed(const PayloadIndex& payloadIndex, const ShutterSpeed& shutterSpeed);
      bool setAperture(const PayloadIndex& payloadIndex, const Aperture& aperture);
      bool setFocusPoint(const PayloadIndex& payloadIndex,const float& x, const float& y);
      bool setTapZoomPoint(const PayloadIndex& payloadIndex,const uint8_t& multiplier, const float& x,const float& y);
      bool startZoom(const PayloadIndex& payloadIndex,const uint8_t& direction, const uint8_t& speed);
      bool stopZoom(const PayloadIndex& payloadIndex);
      bool startShootSinglePhoto(const PayloadIndex& payloadIndex);
      bool startShootBurstPhoto(const PayloadIndex& payloadIndex, const PhotoBurstCount& photoBurstCount);
      bool startShootAEBPhoto(const PayloadIndex& index,  const PhotoAEBCount& photoAebCount);
      bool startShootIntervalPhoto(const PayloadIndex& payloadIndex, const PhotoIntervalData& photoIntervalData);
      bool shootPhotoStop(const PayloadIndex& payloadIndex);
      bool startRecordVideo(const PayloadIndex& payloadIndex);
      bool stopRecordVideo(const PayloadIndex& payloadIndex);
      /*! Parts of Gimbal */
      GimbalSingleData getGimbalData(const PayloadIndex& payloadIndex);
      bool rotateGimbal(const PayloadIndex& PayloadIndex, const GimbalRotationData& rotationData);
      bool resetGimbal(const PayloadIndex& payloadIndex);

    /* Parts of Flight Control*/
      bool setNewHomeLocation(int timeout = 1);
      bool setHomeAltitude(uint16_t altitude, int timeout = 1);
      bool goHome(int timeout);
      bool setAvoid(bool enable);

      bool monitoredTakeoff(ACK::ErrorCode& ack, int timeout);
      bool monitoredLanding(ACK::ErrorCode& ack, int timeout);
      bool moveByPositionOffset(ACK::ErrorCode& ack, int timeout, MoveOffset& p_offset);

      uint8_t outputMFIO(uint8_t mode, uint8_t channel, uint32_t init_on_time_us, uint16_t freq, bool block, uint8_t gpio_value);
      uint32_t inputMFIO(uint8_t mode, uint8_t channel, bool block);
      uint8_t stopMFIO(uint8_t mode, uint8_t channel);

#ifdef ADVANCED_SENSING
      bool startStream(bool is_h264, uint8_t request_view);
      bool stopStream(bool is_h264, uint8_t request_view);

      CameraRGBImage& getCameraImage();
      std::vector<uint8_t>& getCameraRawData();
      void setCameraRawData(uint8_t* rawData, int bufLen);
      void setCameraImage(const CameraRGBImage& img);
      void setAcmDevicePath(const std::string& acm_path);
#endif

  protected:
      bool startGlobalPositionBroadcast();
      Telemetry::Vector3f toEulerAngle(void* quaternionData);
      void localOffsetFromGpsOffset(Telemetry::Vector3f& deltaNed, void* target, void* origin);
    
    public:
      Vehicle::ActivateData *getActivateData()
      {
        return &this->activate_data_;
      }
    
      Vehicle *getVehicle()
      {
        return vehicle;
      }
    
      Linker *getLinker()
      {
        return linker;
      }
    
    private:
      uint32_t timeout_;
      Vehicle::ActivateData activate_data_;
      int          app_id_;
      std::string  enc_key_;
      std::string  device_acm_;
      std::string  device_;
      unsigned int baudrate_;
      std::string  sample_case_;
      const static unsigned int default_acm_baudrate = 230400;

#ifdef ADVANCED_SENSING
      CameraRGBImage image_from_camera_;
      std::vector<uint8_t> raw_data_from_camera_;
      std::mutex camera_data_mutex_;
#endif
};
}

#endif // __VEHICLE_WRAPPER_HH__

