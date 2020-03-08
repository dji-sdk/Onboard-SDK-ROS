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
#include <string>

// Declaration
namespace dji_osdk_ros
{
  using namespace DJI::OSDK;
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

