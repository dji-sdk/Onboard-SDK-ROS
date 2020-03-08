/**
  * @Copyright (C) 2020 All rights reserved.
  * @date: 2020
  * @file: vehicle_wrapper.cc
  * @version: v0.0.1
  * @author: kevin.hoo@dji.com
  * @create_date: 2020-03-08 18:11:38
  * @last_modified_date: 2020-03-08 20:01:21
  * @brief: TODO
  * @details: TODO
  */

//INCLUDE
#include <dji_osdk_ros/vehicle_wrapper.hh>
#include <djiosdk/dji_platform.hpp>
#include <dji_osdk_ros/osdkhal_linux.h>
#include <dji_osdk_ros/osdkosal_linux.h>
#include <iostream>

//CODE
namespace dji_osdk_ros
{
  static E_OsdkStat OsdkUser_Console(const uint8_t *data, uint16_t dataLen)
{
  printf("%s", data);
  return OSDK_STAT_OK;
}

static T_OsdkLoggerConsole printConsole = {
    .consoleLevel = OSDK_LOGGER_CONSOLE_LOG_LEVEL_INFO,
    .func = OsdkUser_Console,
};

static T_OsdkHalUartHandler halUartHandler = {
    .UartInit = OsdkLinux_UartInit,
    .UartWriteData = OsdkLinux_UartSendData,
    .UartReadData = OsdkLinux_UartReadData,
};

#ifdef ADVANCED_SENSING
static T_OsdkHalUSBBulkHandler halUSBBulkHandler = {
    .USBBulkInit = OsdkLinux_USBBulkInit,
    .USBBulkWriteData = OsdkLinux_USBBulkSendData,
    .USBBulkReadData = OsdkLinux_USBBulkReadData,
};
#endif

static T_OsdkOsalHandler osalHandler = {
    .TaskCreate = OsdkLinux_TaskCreate,
    .TaskDestroy = OsdkLinux_TaskDestroy,
    .TaskSleepMs = OsdkLinux_TaskSleepMs,
    .MutexCreate = OsdkLinux_MutexCreate,
    .MutexDestroy = OsdkLinux_MutexDestroy,
    .MutexLock = OsdkLinux_MutexLock,
    .MutexUnlock = OsdkLinux_MutexUnlock,
    .SemaphoreCreate = OsdkLinux_SemaphoreCreate,
    .SemaphoreDestroy = OsdkLinux_SemaphoreDestroy,
    .SemaphoreWait = OsdkLinux_SemaphoreWait,
    .SemaphoreTimedWait = OsdkLinux_SemaphoreTimedWait,
    .SemaphorePost = OsdkLinux_SemaphorePost,
    .GetTimeMs = OsdkLinux_GetTimeMs,
#ifdef OS_DEBUG
    .GetTimeUs = OsdkLinux_GetTimeUs,
#endif
    .Malloc = OsdkLinux_Malloc,
    .Free = OsdkLinux_Free,
  };

  VehicleWrapper::VehicleWrapper(
                     int app_id,
                     const std::string& enc_key,
                     const std::string& dev_name,
                     unsigned int baud_rate,
                     bool enableAdvancedSensing)
    : Setup(enableAdvancedSensing),
      app_id_(app_id),
      enc_key_(enc_key),
      device_acm_(""),
      device_(dev_name),
      baudrate_(baud_rate)
  {
    timeout_ = 1;
    setupEnvironment(enableAdvancedSensing);
    if(initVehicle() == false)
    {
      std::cout << "Init Vehicle Failed, TERMINATE!" << std::endl;
      exit(1);
    };
  }

  void VehicleWrapper::setupEnvironment(bool enable_advanced_sensing)
  {
    if(DJI_REG_LOGGER_CONSOLE(&printConsole) != true)
    {
      throw std::runtime_error("logger console register fail");
    }

    if(DJI_REG_UART_HANDLER(&halUartHandler) != true)
    {
      throw std::runtime_error("Uart handler register fail");
    }

#ifdef ADVANCED_SENSING
    if(DJI_REG_USB_BULK_HANDLER(&halUSBBulkHandler) != true)
    {
      throw std::runtime_error("USB Bulk handler register fail");
    }
#endif

  if(DJI_REG_OSAL_HANDLER(&osalHandler) != true)
  {
    throw std::runtime_error("Osal handler register fail");
  }

  }

  bool VehicleWrapper::initVehicle()
  {
    ACK::ErrorCode ack;
    /*! Linker initialization */
    if (!initLinker())
    {
      DERROR("Failed to initialize Linker");
      return false;
    }

    /*! Linker add uart channel */
    if (!addUartChannel(device_.c_str(),
                        baudrate_,
                        FC_UART_CHANNEL_ID))
    {
      DERROR("Failed to initialize Linker channel");
      return false;
    }

    /*! Linker add USB acm channel */
    if (!addUartChannel(device_acm_.c_str(),
                        default_acm_baudrate,
                        USB_ACM_CHANNEL_ID))
    {
      DERROR("Failed to initialize ACM Linker channel!");
    }

    /*! Vehicle initialization */
    if (!linker)
    {
      DERROR("Linker get failed.");
      return false;
    }

    vehicle = new Vehicle(linker);
    if (!vehicle)
    {
      DERROR("Vehicle create failed.");
      return false;
    }

    // Activate
    activate_data_.ID = app_id_;
    char app_key[65];
    activate_data_.encKey = app_key;
    strcpy(activate_data_.encKey, enc_key_.c_str());
    activate_data_.version = vehicle->getFwVersion();

    ack = vehicle->activate(&activate_data_, timeout_);
    if (ACK::getError(ack))
    {
      ACK::getErrorCodeMessage(ack, __func__);
      return false;
    }

    return true;
  }
}
