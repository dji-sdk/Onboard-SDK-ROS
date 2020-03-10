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
#include <djiosdk/dji_setup_helpers.hpp>
#include <djiosdk/dji_vehicle.hpp>
#include <dji_osdk_ros/common_type.hh>
#include <string>
#include <cmath>

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
                     const std::string& dev_name,
                     unsigned int baud_rate,
                     bool enableAdvancedSensing = false);
      virtual ~VehicleWrapper() = default;
    
    public:
      void setupEnvironment(bool enable_advanced_sensing);
      bool initVehicle();

    public:
      bool takePicture();
      bool startCaptureVideo();
      bool stopCaptureVideo();
      bool setGimbalAngle(const GimbalContainer& gimbal);

      bool goHome(int timeout);
      bool monitoredTakeoff(ACK::ErrorCode& ack, int timeout);
      bool monitoredLanding(ACK::ErrorCode& ack, int timeout);
      bool moveByPositionOffset(ACK::ErrorCode& ack, int timeout, MoveOffset& p_offset);
      bool getCurrentGimbal(RotationAngle& initial_angle);

      uint8_t outputMFIO(uint8_t mode, uint8_t channel, uint32_t init_on_time_us, uint16_t freq, bool block, uint8_t gpio_value);
      uint32_t inputMFIO(uint8_t mode, uint8_t channel, bool block);
      uint8_t stopMFIO(uint8_t mode, uint8_t channel);

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
      const static unsigned int default_acm_baudrate = 921600;
  };
}

#endif // __VEHICLE_WRAPPER_HH__

