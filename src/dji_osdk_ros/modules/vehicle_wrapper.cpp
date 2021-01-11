/** @file vehicle_wrapper.cpp
 *  @version 4.0
 *  @date May 2020
 *
 *  @brief layer of modules of osdk ros 4.0.Encapsulate the interface of osdk.
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
#include <dji_platform.hpp>
#include <dji_vehicle_callback.hpp>

#include <dji_osdk_ros/vehicle_wrapper.h>
#include <osdkhal_linux.h>
#include <osdkosal_linux.h>

#include <iostream>
//CODE
namespace dji_osdk_ros
{
  static E_OsdkStat OsdkUser_Console(const uint8_t *data, uint16_t dataLen)
{
//  printf("%s", data);
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
    .UartClose = OsdkLinux_UartClose,
};

#ifdef ADVANCED_SENSING
static T_OsdkHalUSBBulkHandler halUSBBulkHandler = {
    .USBBulkInit = OsdkLinux_USBBulkInit,
    .USBBulkWriteData = OsdkLinux_USBBulkSendData,
    .USBBulkReadData = OsdkLinux_USBBulkReadData,
    .USBBulkClose = OsdkLinux_USBBulkClose,
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
                     const std::string& device_acm_name,
                     const std::string& dev_name,
                     unsigned int baud_rate,
                     bool enableAdvancedSensing)
    : Setup(enableAdvancedSensing),
      app_id_(app_id),
      enc_key_(enc_key),
      device_acm_(device_acm_name),
      device_(dev_name),
      baudrate_(baud_rate)
  {
    std::cout << "EnableAd: " << enableAdvancedSensing << std::endl;
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
    if (!addFCUartChannel(device_.c_str(),
                          baudrate_))
    {
      DERROR("Failed to initialize Linker channel");
      return false;
    }

    /*! Linker add USB acm channel */
    if (!addUSBACMChannel(device_acm_.c_str(),
                          default_acm_baudrate))
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

  bool VehicleWrapper::setEV(const PayloadIndex& payloadIndex, const ExposureCompensation& exposureCompensation)
  {
    if (!vehicle || !vehicle->cameraManager) {
      DERROR("vehicle or cameraManager is a null value.");
      return false;
    }
    ErrorCode::ErrorCodeType retCode;
    CameraManager *pm = vehicle->cameraManager;
    CameraModule::ExposureCompensation evGet;
    PayloadIndexType index = static_cast<PayloadIndexType>(payloadIndex);
    CameraModule::ExposureCompensation dataTarget = static_cast<CameraModule::ExposureCompensation>(exposureCompensation);
    retCode = pm->getExposureCompensationSync(index, evGet, 1);
    if (retCode == ErrorCode::SysCommonErr::Success) {
      DSTATUS("Get ev = %d", evGet);
      if (dataTarget == evGet) {
        DSTATUS("The ev value is already %d.", dataTarget);
        return true;
      }
    }

    DSTATUS("Set evTarget = %d", dataTarget);
    retCode = pm->setExposureCompensationSync(index, dataTarget, 1);
    if (retCode == ErrorCode::SysCommonErr::Success) {
      DSTATUS("Set ev value successfully.");
    } else {
      DERROR("Set ev parameter error. Error code : 0x%lX", retCode);
      ErrorCode::printErrorCodeMsg(retCode);
      DERROR(
        "In order to use this function, the camera exposure mode should be "
        "set to be PROGRAM_AUTO, SHUTTER_PRIORITY or APERTURE_PRIORITY "
        "first");
      return false;
    }

    return true;
  }

  bool VehicleWrapper::setExposureMode(const PayloadIndex& payloadIndex, const ExposureMode& exposureMode)
  {
    if (!vehicle || !vehicle->cameraManager) {
      DERROR("vehicle or cameraManager is a null value.");
      return false;
    }
    ErrorCode::ErrorCodeType retCode;
    CameraManager *pm = vehicle->cameraManager;
    CameraModule::ExposureMode exposureModeGet;

    PayloadIndexType index = static_cast<PayloadIndexType>(payloadIndex);
    CameraModule::ExposureMode dataTarget = static_cast<CameraModule::ExposureMode>(exposureMode);

    retCode = pm->getExposureModeSync(index, exposureModeGet, 1);
    if (retCode == ErrorCode::SysCommonErr::Success) {
      DSTATUS("Get exposure mode = %d", exposureModeGet);
      if (dataTarget == exposureModeGet)  {
        DSTATUS("The exposure mode is already %d.", dataTarget);
        return true;
      }
    }

    DSTATUS("Set exposure mode = %d", dataTarget);
    retCode = pm->setExposureModeSync(index, dataTarget, 1);
    if (retCode == ErrorCode::SysCommonErr::Success) {
      DSTATUS("Set exposure mode successfully.");
    } else {
      DERROR("Set exposure mode error. Error code : 0x%lX", retCode);
      ErrorCode::printErrorCodeMsg(retCode);
      return false;
    }

    return true;
  }

  bool VehicleWrapper::setISO(const PayloadIndex& payloadIndex, const ISO& ios)
  {
    if (!vehicle || !vehicle->cameraManager) {
      DERROR("vehicle or cameraManager is a null value.");
      return false;
    }
    ErrorCode::ErrorCodeType retCode;
    CameraManager *pm = vehicle->cameraManager;
    CameraModule::ISO isoGet;

    PayloadIndexType index       = static_cast<PayloadIndexType>(payloadIndex);
    CameraModule::ISO dataTarget = static_cast<CameraModule::ISO>(ios);

    retCode = pm->getISOSync(index, isoGet, 1);
    if (retCode == ErrorCode::SysCommonErr::Success) {
      DSTATUS("Get iso = %d", isoGet);
      if (dataTarget == isoGet) {
        DSTATUS("The iso parameter is already %d.", dataTarget);
        return true;
      }
    }

    DSTATUS("Set iso = %d", dataTarget);
    retCode = pm->setISOSync(index, dataTarget, 1);
    if (retCode == ErrorCode::SysCommonErr::Success) {
      DSTATUS("Set iso successfully");
    } else {
      DERROR("Set ISO parameter error. Error code : 0x%lX", retCode);
      ErrorCode::printErrorCodeMsg(retCode);
      DERROR(
        "For the X5, X5R, X4S and X5S, the ISO value can be set for all "
        "modes. For the other cameras, the ISO value can only be set when "
        "the camera exposure mode is in Manual mode.");
      return false;
    }

    return true;
  }

  bool VehicleWrapper::setShutterSpeed(const PayloadIndex& payloadIndex, const ShutterSpeed& shutterSpeed)
  {
    if (!vehicle || !vehicle->cameraManager) {
      DERROR("vehicle or cameraManager is a null value.");
      return false;
    }
    ErrorCode::ErrorCodeType retCode;
    CameraManager *pm = vehicle->cameraManager;
    CameraModule::ShutterSpeed shutterSpeedGet;

    PayloadIndexType index = static_cast<PayloadIndexType>(payloadIndex);
    CameraModule::ShutterSpeed dataTarget = static_cast<CameraModule::ShutterSpeed>(shutterSpeed);

    retCode = pm->getShutterSpeedSync(index, shutterSpeedGet, 1);
    if (retCode == ErrorCode::SysCommonErr::Success) {
      DSTATUS("Get shutterSpeed = %d", shutterSpeedGet);
      if (dataTarget == shutterSpeedGet) {
        DSTATUS("The shutterSpeed is already %d.", dataTarget);
        return true;
      }
    }

    DSTATUS("Set shutterSpeed = %d", dataTarget);
    retCode = pm->setShutterSpeedSync(index, dataTarget, 1);
    if (retCode == ErrorCode::SysCommonErr::Success) {
      DSTATUS("Set iso successfully");
    } else {
      DERROR("Set shutterSpeed parameter error. Error code : 0x%lX", retCode);
      ErrorCode::printErrorCodeMsg(retCode);
      DERROR(
        "The shutter speed can be set only when the camera exposure mode "
        "is Shutter mode or Manual mode. The shutter speed should not be "
        "set slower than the video frame rate when the camera's mode is "
        "RECORD_VIDEO.");
      return false;
    }

    return true;
  }

  bool VehicleWrapper::setAperture(const PayloadIndex& payloadIndex, const Aperture& aperture)
  {
    if (!vehicle || !vehicle->cameraManager) {
      DERROR("vehicle or cameraManager is a null value.");
      return false;
    }
    ErrorCode::ErrorCodeType retCode;
    CameraManager *pm = vehicle->cameraManager;
    CameraModule::Aperture apertureGet;

    PayloadIndexType index = static_cast<PayloadIndexType>(payloadIndex);
    CameraModule::Aperture dataTarget = static_cast<CameraModule::Aperture>(aperture);
    retCode = pm->getApertureSync(index, apertureGet, 1);
    if (retCode == ErrorCode::SysCommonErr::Success) {
      DSTATUS("Get aperture = %d", apertureGet);
      if (dataTarget == apertureGet) {
        DSTATUS("The aperture is already %d.", dataTarget);
        return true;
      }
    }

    DSTATUS("Set aperture = %d", dataTarget);
    retCode = pm->setApertureSync(index, dataTarget, 1);
    if (retCode == ErrorCode::SysCommonErr::Success) {
      DSTATUS("Set aperture successfully");
    } else {
      DERROR("Set aperture parameter error. Error code : 0x%lX", retCode);
      ErrorCode::printErrorCodeMsg(retCode);
      DERROR(
        "In order to use this function, the exposure mode ExposureMode "
        "must be in MANUAL or APERTURE_PRIORITY. Supported only by the X5, "
        "X5R, X4S, X5S camera.");
        return false;
    }

    return true;
  }

  bool VehicleWrapper::setFocusPoint(const PayloadIndex& payloadIndex,const float& x,const float& y)
  {
    if (!vehicle || !vehicle->cameraManager) {
      DERROR("vehicle or cameraManager is a null value.");
      return false;
    }
    ErrorCode::ErrorCodeType retCode;
    CameraManager *pm = vehicle->cameraManager;

    PayloadIndexType index = static_cast<PayloadIndexType>(payloadIndex);

    /*!< set camera focus mode to be CameraModule::FocusMode::AUTO */
    DSTATUS("Set focus mode = %d", CameraModule::FocusMode::AUTO);
    retCode = pm->setFocusModeSync(index, CameraModule::FocusMode::AUTO, 1);
    if (retCode == ErrorCode::SysCommonErr::Success) {
      /*!< set camera focus point */
      DSTATUS("Set focus point = (%f,%f)", x, y);
      retCode = pm->setFocusTargetSync(index, {x, y}, 1);
      if (retCode == ErrorCode::SysCommonErr::Success) {
        DSTATUS("Set focus point successfully");
      } else {
        DERROR("Set focus point error. Error code : 0x%lX", retCode);
        ErrorCode::printErrorCodeMsg(retCode);
        DERROR(
            "When the focus mode is auto, the target point is the focal point. "
            "When the focus mode is manual, the target point is the zoom out area "
            "if the focus assistant is enabled for the manual mode. Supported only "
            "by the X5, X5R, Z3 cameras, Mavic Pro camera, Phantom 4 Pro camera, "
            "Mavic 2 Pro, Mavic 2 Zoom Camera, Mavic 2 Enterprise Camera, X5S. "
            "It's should be attention that X4S will keep focus point as (0.5,0.5) "
            "all the time, the setting of focus point to X4S will quickly replaced "
            "by (0.5, 0.5).");
        return false;
      }
    } else {
      DERROR("Set focus mode parameter error. Error code : 0x%lX", retCode);
      ErrorCode::printErrorCodeMsg(retCode);
      DERROR(
          "When the focus mode is auto, the target point is the focal point. "
          "When the focus mode is manual, the target point is the zoom out area "
          "if the focus assistant is enabled for the manual mode. Supported only "
          "by the X5, X5R, Z3 cameras, Mavic Pro camera, Phantom 4 Pro camera, "
          "Mavic 2 Pro, Mavic 2 Zoom Camera, Mavic 2 Enterprise Camera, X5S. "
          "It's should be attention that X4S will keep focus point as (0.5,0.5) "
          "all the time, the setting of focus point to X4S will quickly replaced "
          "by (0.5, 0.5).");
      return false;
    }

    return true;
  }

  bool VehicleWrapper::setTapZoomPoint(const PayloadIndex& payloadIndex,const uint8_t& multiplier,const float& x,const float& y)
  {
    if (!vehicle || !vehicle->cameraManager) {
      DERROR("vehicle or cameraManager is a null value.");
      return false;
    }
    ErrorCode::ErrorCodeType retCode;
    CameraManager *pm = vehicle->cameraManager;

    PayloadIndexType index = static_cast<PayloadIndexType>(payloadIndex);

    /*!< set camera tap zoom enable parameter to be enable */
    DSTATUS("Set tap zoom enable  = %d", true);
    retCode = pm->setTapZoomEnabledSync(index, true, 1);
    if (retCode != ErrorCode::SysCommonErr::Success) {
      DERROR("Set tap zoom enable fail. Error code : 0x%lX", retCode);
      ErrorCode::printErrorCodeMsg(retCode);
      DERROR("It is only supported Z30 camera.");
      return false;
    }

    /*!< set camera tap zoom multiplier parameter */
    DSTATUS("Set tap zoom multiplier = %d", multiplier);
    retCode = pm->setTapZoomMultiplierSync(index, multiplier, 1);
    if (retCode != ErrorCode::SysCommonErr::Success) {
      DERROR("Set tap zoom multiplier fail. Error code : 0x%lX", retCode);
      ErrorCode::printErrorCodeMsg(retCode);
      DERROR("It is only supported Z30 camera or H20/H20T zoom mode.");
      return false;
    }

    /*!< set camera tap zoom multiplier target point */
    DSTATUS("Set tap zoom target point : (%f,%f)", x, y);
    retCode = pm->tapZoomAtTargetSync(index, {x, y}, 1);
    if (retCode != ErrorCode::SysCommonErr::Success) {
      DERROR("Set tap zoom target fail. Error code : 0x%lX", retCode);
      ErrorCode::printErrorCodeMsg(retCode);
      DERROR("It is only supported Z30 camera or H20/H20T zoom mode.");
      return false;
    } else {
      DSTATUS(
          "tap zoom at target (%0.2f, %0.2f) successfully, need several seconds "
          "to zoom.",
          x, y);
    }

    return true;
  }

  bool VehicleWrapper::setZoom(const PayloadIndex& payloadIndex,  float factor)
  {
    if (!vehicle || !vehicle->cameraManager) {
    DERROR("vehicle or cameraManager is a null value.");
    return ErrorCode::SysCommonErr::InstInitParamInvalid;
    }
    ErrorCode::ErrorCodeType retCode;
    CameraManager *pm = vehicle->cameraManager;

    DSTATUS("Attention : It is only supported by X5, X5R and X5S camera on Osmo with"
            "lens Olympus M.Zuiko ED 14-42mm f/3.5-5.6 EZ, Z3 camera, Z30 camera.");

    float curFactor = 0;
    PayloadIndexType index = static_cast<PayloadIndexType>(payloadIndex);
    retCode = pm->getOpticalZoomFactorSync(index, curFactor, 1);
    if (retCode != ErrorCode::SysCommonErr::Success) {
      DERROR("Get zoom parameter fail. Error code : 0x%lX", retCode);
      ErrorCode::printErrorCodeMsg(retCode);
      DSTATUS(
          "Attention : It is only supported by X5, X5R and X5S camera on Osmo with"
          "lens Olympus M.Zuiko ED 14-42mm f/3.5-5.6 EZ, Z3 camera, Z30 camera.");
      return false;
    }
    DSTATUS("Got the current optical zoom factor : %0.1f", curFactor);
    if (curFactor != factor) {
      DSTATUS("Set the current optical zoom factor as %0.1f", factor);
      retCode = pm->setOpticalZoomFactorSync(index, factor, 1);

      if (retCode != ErrorCode::SysCommonErr::Success) {
        DERROR("Set zoom parameter fail. Error code : 0x%lX", retCode);
        ErrorCode::printErrorCodeMsg(retCode);
        return false;
      }
      return true;
    } else {
      DSTATUS("The current zoom factor is already : %0.2f", factor);
      return true;
    }

    return true;
  }

  bool VehicleWrapper::startZoom(const PayloadIndex& payloadIndex,const uint8_t& direction, const uint8_t& speed)
  {
    if (!vehicle || !vehicle->cameraManager) {
      DERROR("vehicle or cameraManager is a null value.");
      return false;
    }
    ErrorCode::ErrorCodeType retCode;
    CameraManager *pm = vehicle->cameraManager;
    PayloadIndexType index = static_cast<PayloadIndexType>(payloadIndex);

    DSTATUS(
        "Attention : It is only supported by X5, X5R and X5S camera on Osmo with"
        "lens Olympus M.Zuiko ED 14-42mm f/3.5-5.6 EZ, Z3 camera, Z30 camera.");

    DSTATUS("Start continuous optical zoom parameters : direction=%d, speed=%d",
            direction, speed);
    retCode = pm->startContinuousOpticalZoomSync(index, direction, speed, 1);
    if (retCode != ErrorCode::SysCommonErr::Success) {
      DERROR("Start continuous zoom fail. Error code : 0x%lX", retCode);
      ErrorCode::printErrorCodeMsg(retCode);
      return false;
    }
    return true;
  }

  bool VehicleWrapper::stopZoom(const PayloadIndex& payloadIndex)
  {
    if (!vehicle || !vehicle->cameraManager) {
      DERROR("vehicle or cameraManager is a null value.");
      return false;
    }
    ErrorCode::ErrorCodeType retCode;
    CameraManager *pm = vehicle->cameraManager;

    PayloadIndexType index = static_cast<PayloadIndexType>(payloadIndex);

    DSTATUS("Stop continuous optical zoom.");
    retCode = pm->stopContinuousOpticalZoomSync(index, 1);
    if (retCode != ErrorCode::SysCommonErr::Success) {
      DERROR("Stop continuous zoom fail. Error code : 0x%lX", retCode);
      ErrorCode::printErrorCodeMsg(retCode);
      return false;
    }
    return true;
  }

  bool VehicleWrapper::startShootSinglePhoto(const PayloadIndex& payloadIndex)
  {
    if (!vehicle || !vehicle->cameraManager) {
      DERROR("vehicle or cameraManager is a null value.");
      return false;
    }
    ErrorCode::ErrorCodeType retCode;
    CameraManager *pm = vehicle->cameraManager;
    PayloadIndexType index = static_cast<PayloadIndexType>(payloadIndex);

    /*!< set camera work mode as shoot photo */
    DSTATUS("set camera work mode as SHOOT_PHOTO");
    retCode = pm->setModeSync(index, CameraModule::WorkMode::SHOOT_PHOTO, 3);
    if (retCode != ErrorCode::SysCommonErr::Success) {
      DERROR("Camera take photo fail. Error code : 0x%lX", retCode);
      ErrorCode::printErrorCodeMsg(retCode);
      return false;
    }

  /*! @TODO XT* and Z30 don't support set shoot-photo mode. To fix it in the
    * future */
    /*!< set shoot-photo mode */
    DSTATUS("set shoot-photo mode as SINGLE");
    retCode = pm->setShootPhotoModeSync(index, CameraModule::ShootPhotoMode::SINGLE, 1);
    if (retCode != ErrorCode::SysCommonErr::Success) {
      DERROR("Set shoot-photo mode as SINGLE fail. Error code : 0x%lX", retCode);
      ErrorCode::printErrorCodeMsg(retCode);
      DERROR("If the camera is XT, XT2, or XTS, set shoot-photo mode interface is"
            "temporarily not supported.");
      return false;
    }

    /*! wait the APP change the shoot-photo mode display */
    Platform::instance().taskSleepMs(500);

    /*!< start to shoot single photo */
    DSTATUS("start to shoot SINGLE photo");
    retCode =
        pm->startShootPhotoSync(index, CameraModule::ShootPhotoMode::SINGLE, 2);
    if (retCode != ErrorCode::SysCommonErr::Success) {
      DERROR("Take SINGLE photo fail. Error code : 0x%lX", retCode);
      ErrorCode::printErrorCodeMsg(retCode);
      return false;
    }

    return true;
  }

  bool VehicleWrapper::startShootBurstPhoto(const PayloadIndex& payloadIndex, const PhotoBurstCount& photoBurstCount)
  {
    if (!vehicle || !vehicle->cameraManager) {
      DERROR("vehicle or cameraManager is a null value.");
      return false;
    }
    ErrorCode::ErrorCodeType retCode;
    CameraManager *pm = vehicle->cameraManager;
    PayloadIndexType index = static_cast<PayloadIndexType>(payloadIndex);
    CameraModule::PhotoBurstCount count = static_cast<CameraModule::PhotoBurstCount>(photoBurstCount);

    /*!< set camera work mode as SHOOT_PHOTO */
    DSTATUS("set camera work mode as SHOOT_PHOTO");
    retCode = pm->setModeSync(index, CameraModule::WorkMode::SHOOT_PHOTO, 3);
    if (retCode != ErrorCode::SysCommonErr::Success) {
      DERROR("Set camera as SHOOT_PHOTO fail. Error code : 0x%lX", retCode);
      ErrorCode::printErrorCodeMsg(retCode);
      return false;
    }

    /*!< set shoot-photo mode */
    DSTATUS("set shoot-photo mode as BURST");
    retCode =
        pm->setShootPhotoModeSync(index, CameraModule::ShootPhotoMode::BURST, 1);
    if (retCode != ErrorCode::SysCommonErr::Success) {
      DERROR("Set shoot-photo mode as BURST fail. Error code : 0x%lX", retCode);
      ErrorCode::printErrorCodeMsg(retCode);
      return false;
    }

    /*! wait the APP change the shoot-photo mode display */
    Platform::instance().taskSleepMs(500);

    /*!< set shoot-photo mode parameter */
    DSTATUS("set count = %d", count);
    retCode = pm->setPhotoBurstCountSync(index, count, 1);
    if (retCode != ErrorCode::SysCommonErr::Success) {
      DERROR("Set the parameter of BURST mode fail. Error code : 0x%lX", retCode);
      ErrorCode::printErrorCodeMsg(retCode);
      return false;
    }

    /*!< start to shoot BURST photo */
    DSTATUS("start to shoot BURST photo");
    retCode =
        pm->startShootPhotoSync(index, CameraModule::ShootPhotoMode::BURST, 2);
    if (retCode != ErrorCode::SysCommonErr::Success) {
      DERROR("Take BURST photo fail. Error code : 0x%lX", retCode);
      ErrorCode::printErrorCodeMsg(retCode);
      return false;
    }

    return true;
  }

  bool VehicleWrapper::startShootAEBPhoto(const PayloadIndex& payloadIndex, const PhotoAEBCount& photoAebCount)
  {
    if (!vehicle || !vehicle->cameraManager) {
      DERROR("vehicle or cameraManager is a null value.");
      return false;
    }
    ErrorCode::ErrorCodeType retCode;
    CameraManager *pm = vehicle->cameraManager;
    PayloadIndexType index = static_cast<PayloadIndexType>(payloadIndex);
    CameraModule::PhotoAEBCount photoNum = static_cast<CameraModule::PhotoAEBCount>(photoAebCount);

    /*!< set camera work mode as SHOOT_PHOTO */
    DSTATUS("set camera work mode as SHOOT_PHOTO");
    retCode = pm->setModeSync(index, CameraModule::WorkMode::SHOOT_PHOTO, 3);
    if (retCode != ErrorCode::SysCommonErr::Success) {
      DERROR("Set camera as SHOOT_PHOTO fail. Error code : 0x%lX", retCode);
      ErrorCode::printErrorCodeMsg(retCode);
      return false;
    }

    /*!< set shoot-photo mode */
    DSTATUS("set shoot-photo mode as AEB");
    retCode =
        pm->setShootPhotoModeSync(index, CameraModule::ShootPhotoMode::AEB, 1);
    if (retCode != ErrorCode::SysCommonErr::Success) {
      DERROR("Set shoot-photo mode as AEB fail. Error code : 0x%lX", retCode);
      ErrorCode::printErrorCodeMsg(retCode);
      return false;
    }

    /*! wait the APP change the shoot-photo mode display */
    Platform::instance().taskSleepMs(500);

    /*!< set shoot-photo mode parameter */
    DSTATUS("set AEB photo number = %d", photoNum);
    retCode = pm->setPhotoAEBCountSync(index, photoNum, 1);
    if (retCode != ErrorCode::SysCommonErr::Success) {
      DERROR("Set the parameter of AEB mode fail. Error code : 0x%lX", retCode);
      ErrorCode::printErrorCodeMsg(retCode);
      return false;
    }

    /*!< start to shoot AEB photo */
    DSTATUS("start to shoot AEB photo");
    retCode =
        pm->startShootPhotoSync(index, CameraModule::ShootPhotoMode::AEB, 2);
    if (retCode != ErrorCode::SysCommonErr::Success) {
      DERROR("Take AEB photo fail. Error code : 0x%lX", retCode);
      ErrorCode::printErrorCodeMsg(retCode);
      return false;
    }

    return true;
  }

  bool VehicleWrapper::startShootIntervalPhoto(const PayloadIndex& payloadIndex, const PhotoIntervalData& photoIntervalData)
  {
    if (!vehicle || !vehicle->cameraManager) {
      DERROR("vehicle or cameraManager is a null value.");
      return false;
    }
    ErrorCode::ErrorCodeType retCode;
    CameraManager *pm = vehicle->cameraManager;
    PayloadIndexType index = static_cast<PayloadIndexType>(payloadIndex);
    CameraModule::PhotoIntervalData intervalData;
    intervalData.photoNumConticap = photoIntervalData.photoNumConticap;
    intervalData.timeInterval     = photoIntervalData.timeInterval;

    /*!< set camera work mode as SHOOT_PHOTO */
    DSTATUS("set camera work mode as SHOOT_PHOTO");
    retCode = pm->setModeSync(index, CameraModule::WorkMode::SHOOT_PHOTO, 3);
    if (retCode != ErrorCode::SysCommonErr::Success) {
      DERROR("Set camera as SHOOT_PHOTO fail. Error code : 0x%lX", retCode);
      ErrorCode::printErrorCodeMsg(retCode);
      return false;
    }

    /*!< set shoot-photo mode */
    DSTATUS("set shoot-photo mode as INTERVAL");
    retCode = pm->setShootPhotoModeSync(
        index, CameraModule::ShootPhotoMode::INTERVAL, 1);
    if (retCode != ErrorCode::SysCommonErr::Success) {
      DERROR("Set shoot-photo mode as INTERVAL fail. Error code : 0x%lX",
             retCode);
      ErrorCode::printErrorCodeMsg(retCode);
      return false;
    }

    /*! wait the APP change the shoot-photo mode display */
    Platform::instance().taskSleepMs(500);

    /*!< set shoot-photo mode parameter */
    DSTATUS("set intervalData : photoNumConticap = %d ,timeInterval = %d",
            intervalData.photoNumConticap, intervalData.timeInterval);
    retCode = pm->setPhotoTimeIntervalSettingsSync(index, intervalData, 1);
    if (retCode != ErrorCode::SysCommonErr::Success) {
      DERROR("Set the parameter of INTERVAL mode fail. Error code : 0x%lX",
             retCode);
      ErrorCode::printErrorCodeMsg(retCode);
      return false;
    }

    /*!< start to shoot INTERVAL photo */
    DSTATUS("start to shoot INTERVAL photo");
    retCode =
        pm->startShootPhotoSync(index, CameraModule::ShootPhotoMode::INTERVAL, 2);
    if (retCode != ErrorCode::SysCommonErr::Success) {
      DERROR("Take INTERVAL photo fail. Error code : 0x%lX", retCode);
      ErrorCode::printErrorCodeMsg(retCode);
      return false;
    }

    return true;
  }

  bool VehicleWrapper::shootPhotoStop(const PayloadIndex& payloadIndex)
  {
    if (!vehicle || !vehicle->cameraManager) {
      DERROR("vehicle or cameraManager is a null value.");
      return false;
    }
    ErrorCode::ErrorCodeType retCode;
    CameraManager *pm = vehicle->cameraManager;
    PayloadIndexType index = static_cast<PayloadIndexType>(payloadIndex);

    /*!< stop camera shooting photo */
    DSTATUS("Stop to shoot photo");
    retCode = pm->stopShootPhotoSync(index, 1);
    if (retCode != ErrorCode::SysCommonErr::Success) {
      DERROR("stop camera shooting photo fail. Error code : 0x%lX", retCode);
      ErrorCode::printErrorCodeMsg(retCode);
      return false;
    }

    return true;
  }

  bool VehicleWrapper::startRecordVideo(const PayloadIndex& payloadIndex)
  {
    if (!vehicle || !vehicle->cameraManager) {
      DERROR("vehicle or cameraManager is a null value.");
      return false;
    }
    ErrorCode::ErrorCodeType retCode;
    CameraManager *pm = vehicle->cameraManager;
    PayloadIndexType index = static_cast<PayloadIndexType>(payloadIndex);

    /*!< set camera work mode as RECORD_VIDEO */
    DSTATUS("Set camera mode to RECORD_VIDEO");
    retCode = pm->setModeSync(index, CameraModule::WorkMode::RECORD_VIDEO, 3);
    if (retCode != ErrorCode::SysCommonErr::Success) {
      DERROR("Set camera as RECORD_VIDEO mode fail. Error code : 0x%lX", retCode);
      ErrorCode::printErrorCodeMsg(retCode);
      return false;
    }

    /*!< start to take video */
    DSTATUS("Start to RECORD_VIDEO");
    retCode = pm->startRecordVideoSync(index, 1);
    if (retCode != ErrorCode::SysCommonErr::Success) {
      DERROR("Start to record video fail. Error code : 0x%lX", retCode);
      ErrorCode::printErrorCodeMsg(retCode);
      return false;
    }

    return true;
  }

  bool VehicleWrapper::stopRecordVideo(const PayloadIndex& payloadIndex)
  {
    if (!vehicle || !vehicle->cameraManager) {
      DERROR("vehicle or cameraManager is a null value.");
      return false;
    }
    ErrorCode::ErrorCodeType retCode;
    CameraManager *pm = vehicle->cameraManager;
    PayloadIndexType index = static_cast<PayloadIndexType>(payloadIndex);

    /*!< stop to take video */
    retCode = pm->stopRecordVideoSync(index, 1);
    DSTATUS("Stop RECORD_VIDEO");
    if (retCode != ErrorCode::SysCommonErr::Success) {
      DERROR("Stop to record video fail. Error code : 0x%lX", retCode);
      ErrorCode::printErrorCodeMsg(retCode);
      return false;
    }

    return true;
  }

  bool VehicleWrapper::motorStartedCheck()
  {
    if (!vehicle)
    {
      std::cout << "Vehicle is a null value!" << std::endl;
      return false;
    }

    int motorsNotStarted = 0;
    int timeoutCycles = 20;
    while (vehicle->subscribe->getValue<TOPIC_STATUS_FLIGHT>() !=
              VehicleStatus::FlightStatus::ON_GROUND &&
          vehicle->subscribe->getValue<TOPIC_STATUS_DISPLAYMODE>() !=
              VehicleStatus::DisplayMode::MODE_ENGINE_START &&
          motorsNotStarted < timeoutCycles) {
      motorsNotStarted++;
      usleep(100000);
    }
    return motorsNotStarted != timeoutCycles ? true : false;
  }

  bool VehicleWrapper::takeOffInAirCheck()
  {
    if (!vehicle)
    {
      std::cout << "Vehicle is a null value!" << std::endl;
      return false;
    }

    int stillOnGround = 0;
    int timeoutCycles = 110;
    while (vehicle->subscribe->getValue<TOPIC_STATUS_FLIGHT>() !=
              VehicleStatus::FlightStatus::IN_AIR &&
          (vehicle->subscribe->getValue<TOPIC_STATUS_DISPLAYMODE>() !=
                VehicleStatus::DisplayMode::MODE_ASSISTED_TAKEOFF ||
            vehicle->subscribe->getValue<TOPIC_STATUS_DISPLAYMODE>() !=
                VehicleStatus::DisplayMode::MODE_AUTO_TAKEOFF) &&
          stillOnGround < timeoutCycles) {
      stillOnGround++;
      usleep(100000);
    }

    return stillOnGround != timeoutCycles ? true : false;
  }

  bool VehicleWrapper::takeoffFinishedCheck()
  {
    if (!vehicle)
    {
      std::cout << "Vehicle is a null value!" << std::endl;
      return false;
    }

    while (vehicle->subscribe->getValue<TOPIC_STATUS_DISPLAYMODE>() ==
              VehicleStatus::DisplayMode::MODE_ASSISTED_TAKEOFF ||
          vehicle->subscribe->getValue<TOPIC_STATUS_DISPLAYMODE>() ==
              VehicleStatus::DisplayMode::MODE_AUTO_TAKEOFF) {
      sleep(1);
    }
    return ((vehicle->subscribe->getValue<TOPIC_STATUS_DISPLAYMODE>() ==
            VehicleStatus::DisplayMode::MODE_P_GPS) ||
            (vehicle->subscribe->getValue<TOPIC_STATUS_DISPLAYMODE>() ==
            VehicleStatus::DisplayMode::MODE_ATTITUDE))
              ? true
              : false;
  }

  bool VehicleWrapper::landFinishedCheck()
  {
    if (!vehicle)
    {
      std::cout << "Vehicle is a null value!" << std::endl;
      return false;
    }

    while(vehicle->subscribe->getValue<TOPIC_STATUS_DISPLAYMODE>() ==
                VehicleStatus::DisplayMode::MODE_AUTO_LANDING &&
            vehicle->subscribe->getValue<TOPIC_STATUS_FLIGHT>() ==
                VehicleStatus::FlightStatus::IN_AIR)
    {
        Platform::instance().taskSleepMs(1000);
    }

    return ((vehicle->subscribe->getValue<TOPIC_STATUS_DISPLAYMODE>() !=
            VehicleStatus::DisplayMode::MODE_P_GPS ||
            vehicle->subscribe->getValue<TOPIC_STATUS_DISPLAYMODE>() !=
            VehicleStatus::DisplayMode::MODE_ATTITUDE)) ? true:false;
  }

  bool VehicleWrapper::monitoredTakeoff(int timeout)
  {
    //! Start takeoff
    //ErrorCode::ErrorCodeType takeoffStatus =
        vehicle->flightController->startTakeoffSync(timeout);

    //! Motors start check
    if (!motorStartedCheck()) {
      std::cout << "Takeoff failed. Motors are not spinning." << std::endl;
      return false;
    } else {
      std::cout << "Motors spinning...\n";
    }
    //! In air check
    if (!takeOffInAirCheck()) {
      std::cout << "Takeoff failed. Aircraft is still on the ground, but the "
                  "motors are spinning."
                << std::endl;
      return false;
    } else {
      std::cout << "Ascending...\n";
    }

    //! Finished takeoff check
    if (takeoffFinishedCheck()) {
      std::cout << "Successful takeoff!\n";
    } else {
      std::cout << "Takeoff finished, but the aircraft is in an unexpected mode. "
                  "Please connect DJI GO.\n";
      return false;
    }
    return true;
  }


  bool VehicleWrapper::monitoredLanding(int timeout)
  {
    if (!vehicle)
    {
      std::cout << "Vehicle is a null value!" << std::endl;
      return false;
    }

    /*! Step 1: Start landing */
    DSTATUS("Start landing action");
    ErrorCode::ErrorCodeType landingErrCode = vehicle->flightController->startLandingSync(timeout);
    if (landingErrCode != ErrorCode::SysCommonErr::Success)
    {
      DERROR( "Fail to execute landing action! Error code: "
              "%llx\n ",landingErrCode);
      return false;
    }

    /*! Step 2: check Landing start*/
    if (!checkActionStarted(VehicleStatus::DisplayMode::MODE_AUTO_LANDING))
    {
      DERROR("Fail to execute Landing action!");
      return false;
    } 
    else
    {
      /*! Step 3: check Landing finished*/
      if (this->landFinishedCheck())
      {
        DSTATUS("Successful landing!");
      }
      else
      {
        DERROR("Landing finished, but the aircraft is in an unexpected mode. "
              "Please connect DJI Assistant.");
        return false;
      }
    }

    return true;
  }

  bool VehicleWrapper::startForceLanding(int timeout)
  {
    if (!vehicle)
    {
      std::cout << "Vehicle is a null value!" << std::endl;
      return false;
    }
    ErrorCode::ErrorCodeType errCode = vehicle->flightController->startForceLandingSync(timeout);
    if (errCode != ErrorCode::SysCommonErr::Success)
    {
      DERROR( "Fail to execute force landing action! Error code: "
              "%llx\n ",errCode);
      return false;
    }

    return true;
  }

  bool VehicleWrapper::startConfirmLanding(int timeout)
  {
    if (!vehicle)
    {
      std::cout << "Vehicle is a null value!" << std::endl;
      return false;
    }
    ErrorCode::ErrorCodeType errCode = vehicle->flightController->startConfirmLandingSync(timeout);
    if (errCode != ErrorCode::SysCommonErr::Success)
    {
      DERROR( "Fail to execute confirm landing action! Error code: "
              "%llx\n ",errCode);
      return false;
    }

    return true;
  }

  bool VehicleWrapper::cancelLanding(int timeout)
  {
    if (!vehicle)
    {
      std::cout << "Vehicle is a null value!" << std::endl;
      return false;
    }
    ErrorCode::ErrorCodeType errCode = vehicle->flightController->cancelLandingSync(timeout);
    if (errCode != ErrorCode::SysCommonErr::Success)
    {
      DERROR( "Fail to execute cancel landing action! Error code: "
              "%llx\n ",errCode);
      return false;
    }

    return true;
  }

  bool VehicleWrapper::goHome( int timeout)
  {
    if (!vehicle)
    {
      std::cout << "Vehicle is a null value!" << std::endl;
      return false;
    }
    ErrorCode::ErrorCodeType goHomeErrCode = vehicle->flightController->startGoHomeSync(timeout);
    if (goHomeErrCode != ErrorCode::SysCommonErr::Success)
    {
      DERROR( "Fail to execute gohome action! Error code: "
              "%llx\n ",goHomeErrCode);
      return false;
    }

    return true;
  }

  bool VehicleWrapper::cancelGoHome( int timeout)
  {
    if (!vehicle)
    {
      std::cout << "Vehicle is a null value!" << std::endl;
      return false;
    }
    ErrorCode::ErrorCodeType goHomeErrCode = vehicle->flightController->cancelGoHomeSync(timeout);
    if (goHomeErrCode != ErrorCode::SysCommonErr::Success)
    {
      DERROR( "Fail to execute cancel go home action! Error code: "
              "%llx\n ",goHomeErrCode);
      return false;
    }

    return true;
  }

  bool VehicleWrapper::setUpSubscription(int pkgIndex, int freq, TopicName* topicList,
                                         uint8_t topicSize, int timeout)
  {
    if (!vehicle)
    {
      std::cout << "Vehicle is a null value!" << std::endl;
      return false;
    }
    if (vehicle) {
      /*! Telemetry: Verify the subscription*/
      ACK::ErrorCode subscribeStatus;
      subscribeStatus = vehicle->subscribe->verify(timeout);
      if (ACK::getError(subscribeStatus) != ACK::SUCCESS) {
        ACK::getErrorCodeMessage(subscribeStatus, __func__);
        return false;
      }

      bool enableTimestamp = false;
      bool pkgStatus = vehicle->subscribe->initPackageFromTopicList(
          pkgIndex, topicSize, topicList, enableTimestamp, freq);
      if (!(pkgStatus)) {
        return pkgStatus;
      }

      /*! Start listening to the telemetry data */
      subscribeStatus = vehicle->subscribe->startPackage(pkgIndex, timeout);
      if (ACK::getError(subscribeStatus) != ACK::SUCCESS) {
        ACK::getErrorCodeMessage(subscribeStatus, __func__);
        /*! Cleanup*/
        ACK::ErrorCode ack = vehicle->subscribe->removePackage(pkgIndex, timeout);
        if (ACK::getError(ack)) {
          DERROR(
              "Error unsubscription; please restart the drone/FC to get "
              "back to a clean state");
        }
        return false;
      }
      return true;
    } else {
      DERROR("vehicle haven't been initialized", __func__);
      return false;
    }
  }

  bool VehicleWrapper::teardownSubscription(const int pkgIndex, int timeout)
  {
    if (!vehicle)
    {
      std::cout << "Vehicle is a null value!" << std::endl;
      return false;
    }
    ACK::ErrorCode ack = vehicle->subscribe->removePackage(pkgIndex, timeout);
    if (ACK::getError(ack)) {
      DERROR(
          "Error unsubscription; please restart the drone/FC to get back "
          "to a clean state.");
      return false;
    }
    return true;
  }

  bool VehicleWrapper::checkActionStarted(uint8_t mode)
  {
    if (!vehicle)
    {
      std::cout << "Vehicle is a null value!" << std::endl;
      return false;
    }
    int actionNotStarted = 0;
    int timeoutCycles = 20;
    while (vehicle->subscribe->getValue<TOPIC_STATUS_DISPLAYMODE>() != mode &&
           actionNotStarted < timeoutCycles) {
      actionNotStarted++;
      Platform::instance().taskSleepMs(100);
    }
    if (actionNotStarted == timeoutCycles) {
      DERROR("Start actions mode %d failed, current DISPLAYMODE is: %d ...", mode,
             vehicle->subscribe->getValue<TOPIC_STATUS_DISPLAYMODE>());
      return false;
    } else {
      DSTATUS("DISPLAYMODE: %d ...", mode);
      return true;
    }
  }

  bool VehicleWrapper::goHomeAndConfirmLanding(int timeout)
  {
    if (!vehicle)
    {
      std::cout << "Vehicle is a null value!" << std::endl;
      return false;
    }
    /*! Step 1: Verify and setup the subscription */
    const int pkgIndex = static_cast<int>(SubscribePackgeIndex::TEMP_SUB_PACKAGE_INDEX);
    int freq = 10;
//    TopicName topicList[] = {TOPIC_STATUS_FLIGHT, TOPIC_STATUS_DISPLAYMODE,
//                             TOPIC_AVOID_DATA, TOPIC_VELOCITY};
    TopicName topicList[] = { TOPIC_AVOID_DATA };
    int topicSize = sizeof(topicList) / sizeof(topicList[0]);
    setUpSubscription(pkgIndex, freq, topicList, topicSize, timeout);

/*! Step 2: Start go home */
    DSTATUS("Start go home action");
    ErrorCode::ErrorCodeType goHomeAck =
        vehicle->flightController->startGoHomeSync(timeout);
    if (goHomeAck != ErrorCode::SysCommonErr::Success) {
      DERROR("Fail to execute go home action!  Error code: %llx\n", goHomeAck);
      return false;
    }
    if (!checkActionStarted(VehicleStatus::DisplayMode::MODE_NAVI_GO_HOME)) {
      return false;
    } else {
      while (vehicle->subscribe->getValue<TOPIC_STATUS_DISPLAYMODE>() ==
             VehicleStatus::DisplayMode::MODE_NAVI_GO_HOME &&
             vehicle->subscribe->getValue<TOPIC_STATUS_FLIGHT>() ==
             VehicleStatus::FlightStatus::IN_AIR) {
        Platform::instance().taskSleepMs(
            1000);  // waiting for this action finished
      }
    }
    DSTATUS("Finished go home action");

/*! Step 3: Start landing */
    DSTATUS("Start landing action");
    if (!checkActionStarted(VehicleStatus::DisplayMode::MODE_AUTO_LANDING)) {
      DERROR("Fail to execute Landing action!");
      return false;
    } else {
      while (vehicle->subscribe->getValue<TOPIC_STATUS_DISPLAYMODE>() ==
             VehicleStatus::DisplayMode::MODE_AUTO_LANDING &&
             vehicle->subscribe->getValue<TOPIC_STATUS_FLIGHT>() ==
             VehicleStatus::FlightStatus::IN_AIR) {
        Telemetry::TypeMap<TOPIC_AVOID_DATA>::type avoidData =
            vehicle->subscribe->getValue<TOPIC_AVOID_DATA>();
        Platform::instance().taskSleepMs(1000);
        if ((0.65 < avoidData.down && avoidData.down < 0.75) &&
            (avoidData.downHealth == 1)) {
          break;
        }
      }
    }
    DSTATUS("Finished landing action");

/*! Step 4: Confirm Landing */
    DSTATUS("Start confirm Landing and avoid ground action");
    ErrorCode::ErrorCodeType forceLandingAvoidGroundAck =
        vehicle->flightController->startConfirmLandingSync(timeout);
    if (forceLandingAvoidGroundAck != ErrorCode::SysCommonErr::Success) {
      DERROR(
          "Fail to execute confirm landing avoid ground action! Error code: "
          "%llx\n ",
          forceLandingAvoidGroundAck);
      return false;
    }
    if (!checkActionStarted(VehicleStatus::DisplayMode::MODE_AUTO_LANDING)) {
      return false;
    } else {
      while (vehicle->subscribe->getValue<TOPIC_STATUS_DISPLAYMODE>() ==
             VehicleStatus::DisplayMode::MODE_AUTO_LANDING &&
             vehicle->subscribe->getValue<TOPIC_STATUS_FLIGHT>() ==
             VehicleStatus::FlightStatus::IN_AIR) {
        Platform::instance().taskSleepMs(1000);
      }
    }
    DSTATUS("Finished force Landing and avoid ground action");

/*! Step 5: Landing finished check*/
    if (vehicle->subscribe->getValue<TOPIC_STATUS_DISPLAYMODE>() !=
        VehicleStatus::DisplayMode::MODE_P_GPS ||
        vehicle->subscribe->getValue<TOPIC_STATUS_DISPLAYMODE>() !=
        VehicleStatus::DisplayMode::MODE_ATTITUDE) {
      DSTATUS("Successful landing!");
    } else {
      DERROR(
          "Landing finished, but the aircraft is in an unexpected mode. "
          "Please connect DJI Assistant.");
      teardownSubscription(pkgIndex, timeout);
      return false;
    }
/*! Step 6: Cleanup */
    teardownSubscription(pkgIndex, timeout);
    return true;
  }

  bool VehicleWrapper::setCurrentAircraftLocAsHomePoint(int timeout)
  {
    if (!vehicle)
    {
      std::cout << "Vehicle is a null value!" << std::endl;
      return false;
    }
    HomeLocationSetStatus homeLocationSetStatus;
    HomeLocationData originHomeLocation;
    ErrorCode::ErrorCodeType ret = ErrorCode::FlightControllerErr::SetHomeLocationErr::Fail;
    auto fun_get_home_location = [&](HomeLocationSetStatus& homeLocationSetStatus,
                                    HomeLocationData& homeLocationInfo,
                                    int responseTimeout) {
      ACK::ErrorCode subscribeStatus;
      subscribeStatus = vehicle->subscribe->verify(responseTimeout);
      if (ACK::getError(subscribeStatus) != ACK::SUCCESS)
      {
        ACK::getErrorCodeMessage(subscribeStatus, __func__);
        return false;
      }
      int pkgIndex = static_cast<int>(SubscribePackgeIndex::TEMP_SUB_PACKAGE_INDEX);
      int freq = 1;
      TopicName topicList[] = {TOPIC_HOME_POINT_SET_STATUS, TOPIC_HOME_POINT_INFO};

      int numTopic = sizeof(topicList) / sizeof(topicList[0]);
      bool enableTimestamp = false;

      bool pkgStatus = vehicle->subscribe->initPackageFromTopicList(
        pkgIndex, numTopic, topicList, enableTimestamp, freq);

      if (!(pkgStatus))
      {
        return pkgStatus;
      }
      subscribeStatus = vehicle->subscribe->startPackage(pkgIndex, responseTimeout);
      if (ACK::getError(subscribeStatus) != ACK::SUCCESS)
      {
        ACK::getErrorCodeMessage(subscribeStatus, __func__);
        /*! Cleanup before return */
        vehicle->subscribe->removePackage(pkgIndex, responseTimeout);
        return false;
      }
      /*! Wait for the data to start coming in.*/
      Platform::instance().taskSleepMs(2000);
      homeLocationSetStatus =
        vehicle->subscribe->getValue<TOPIC_HOME_POINT_SET_STATUS>();
      homeLocationInfo = vehicle->subscribe->getValue<TOPIC_HOME_POINT_INFO>();
      ACK::ErrorCode ack =
        vehicle->subscribe->removePackage(pkgIndex, responseTimeout);
      if (ACK::getError(ack))
      {
        DERROR("Error unsubscription; please restart the drone/FC to get back "
          "to a clean state.");
      }
      return true;
    };
    bool retCode = fun_get_home_location(homeLocationSetStatus, originHomeLocation, timeout);
    if (retCode && (homeLocationSetStatus.status == 1))
    {
      ret = vehicle->flightController->setHomeLocationUsingCurrentAircraftLocationSync(timeout);
      if (ret != ErrorCode::SysCommonErr::Success)
      {
        DSTATUS("Set new home location failed, ErrorCode is:%8x", ret);
      }
      else
      {
        DSTATUS("Set new home location successfully");
        return true;
      }
    }
    return ret;
  }

  bool VehicleWrapper::setHomeAltitude(uint16_t altitude, int timeout)
  {
    if (!vehicle)
    {
      std::cout << "Vehicle is a null value!" << std::endl;
      return false;
    }
    ErrorCode::ErrorCodeType ret = vehicle->flightController->setGoHomeAltitudeSync(altitude, timeout);
    if (ret != ErrorCode::SysCommonErr::Success)
    {
      DSTATUS("Set go home altitude failed, ErrorCode is:%8x", ret);
      return false;
    }
    else
    {
      DSTATUS("Set go home altitude successfully,altitude is: %d", altitude);
      return true;
    }
  }

  bool VehicleWrapper::getHomeAltitude(uint16_t& altitude, int timeout)
  {
    if (!vehicle)
    {
      std::cout << "Vehicle is a null value!" << std::endl;
      return false;
    }
    ErrorCode::ErrorCodeType ret = vehicle->flightController->getGoHomeAltitudeSync(altitude, timeout);
    if (ret != ErrorCode::SysCommonErr::Success)
    {
      DSTATUS("Get go home altitude failed, ErrorCode is:%8x", ret);
      return false;
    }
    else
    {
      DSTATUS("Get go home altitude successfully,altitude is: %d", altitude);
      return true;
    }
  }

  bool VehicleWrapper::setHomePoint(float64_t latitude, float64_t longitude, int timeout)
  {
    if (!vehicle)
    {
      std::cout << "Vehicle is a null value!" << std::endl;
      return false;
    }
    DJI::OSDK::FlightController::HomeLocation homeLocation;
    homeLocation.latitude = latitude;
    homeLocation.longitude = longitude;
    ErrorCode::ErrorCodeType ret = vehicle->flightController->setHomeLocationSync(homeLocation, timeout);
    if (ret != ErrorCode::SysCommonErr::Success)
    {
      DSTATUS("Set home point failed, ErrorCode is:%8x", ret);
      return false;
    }
    else
    {
      DSTATUS("Set home point successfully,latitude: %f rad, longitude:%f rad", latitude, longitude);
      return true;
    }
  }

  bool VehicleWrapper::setCollisionAvoidance(bool enable)
  {
    if (!vehicle)
    {
      std::cout << "Vehicle is a null value!" << std::endl;
      return false;
    }
    auto enum_enable = enable ? FlightController::AvoidEnable::AVOID_ENABLE : FlightController::AvoidEnable::AVOID_DISABLE;
    ErrorCode::ErrorCodeType ack = vehicle->flightController->setCollisionAvoidanceEnabledSync(enum_enable, 1);
    if (ack == ErrorCode::SysCommonErr::Success)
    {
      return true;
    }
    return false;
  }

  bool VehicleWrapper::getCollisionAvoidance(uint8_t& enable)
  {
    if (!vehicle)
    {
      std::cout << "Vehicle is a null value!" << std::endl;
      return false;
    }
    FlightController::AvoidEnable enum_enable;
    ErrorCode::ErrorCodeType ack = vehicle->flightController->getCollisionAvoidanceEnabledSync(enum_enable, 1);
    enable = static_cast<uint8_t>(enum_enable);
    if (ack == ErrorCode::SysCommonErr::Success)
    {
      return true;
    }

    enable = 0xF;
    return false;
  }

  bool VehicleWrapper::setUpwardsAvoidance(bool enable)
  {
    if (!vehicle)
    {
      std::cout << "Vehicle is a null value!" << std::endl;
      return false;
    }
    auto enum_enable = enable ? FlightController::UpwardsAvoidEnable::UPWARDS_AVOID_ENABLE : FlightController::UpwardsAvoidEnable::UPWARDS_AVOID_DISABLE;
    ErrorCode::ErrorCodeType ack = vehicle->flightController->setUpwardsAvoidanceEnabledSync(enum_enable, 1);
    if (ack == ErrorCode::SysCommonErr::Success)
    {
      return true;
    }
    return false;
  }

  bool VehicleWrapper::getUpwardsAvoidance(uint8_t& enable)
  {
    if (!vehicle)
    {
      std::cout << "Vehicle is a null value!" << std::endl;
      return false;
    }
    FlightController::UpwardsAvoidEnable enum_enable;
    ErrorCode::ErrorCodeType ack = vehicle->flightController->getUpwardsAvoidanceEnabledSync(enum_enable, 1);
    enable = static_cast<uint8_t>(enum_enable);
    if (ack == ErrorCode::SysCommonErr::Success)
    {
      return true;
    }

    enable = 0xF;
    return false;
  }

  bool VehicleWrapper::setJoystickMode(const JoystickMode &joystickMode)
  {
    if (!vehicle)
    {
      std::cout << "Vehicle is a null value!" << std::endl;
      return false;
    }
    DJI::OSDK::FlightController::JoystickMode interJoystickMode;
    interJoystickMode.horizontalLogic = static_cast<DJI::OSDK::FlightController::HorizontalLogic>(joystickMode.horizontalLogic);
    interJoystickMode.verticalLogic   = static_cast<DJI::OSDK::FlightController::VerticalLogic>(joystickMode.verticalLogic);
    interJoystickMode.yawLogic = static_cast<DJI::OSDK::FlightController::YawLogic>(joystickMode.yawLogic);
    interJoystickMode.horizontalCoordinate = static_cast<DJI::OSDK::FlightController::HorizontalCoordinate>(joystickMode.horizontalCoordinate);
    interJoystickMode.stableMode = static_cast<DJI::OSDK::FlightController::StableMode>(joystickMode.stableMode);

    vehicle->flightController->setJoystickMode(interJoystickMode);

    return true;
  }

  bool VehicleWrapper::JoystickAction(const JoystickCommand &JoystickCommand)
  {
    if (!vehicle)
    {
      std::cout << "Vehicle is a null value!" << std::endl;
      return false;
    }
    DJI::OSDK::FlightController::JoystickCommand interJoystickCommand;
    memcpy(&interJoystickCommand, &JoystickCommand, sizeof(interJoystickCommand));
    vehicle->flightController->setJoystickCommand(interJoystickCommand);
    vehicle->flightController->joystickAction();

    return true;
  }

  Vector3f VehicleWrapper::quaternionToEulerAngle(const Telemetry::Quaternion& quat)
  {
    Telemetry::Vector3f eulerAngle;
    double q2sqr = quat.q2 * quat.q2;
    double t0 = -2.0 * (q2sqr + quat.q3 * quat.q3) + 1.0;
    double t1 = 2.0 * (quat.q1 * quat.q2 + quat.q0 * quat.q3);
    double t2 = -2.0 * (quat.q1 * quat.q3 - quat.q0 * quat.q2);
    double t3 = 2.0 * (quat.q2 * quat.q3 + quat.q0 * quat.q1);
    double t4 = -2.0 * (quat.q1 * quat.q1 + q2sqr) + 1.0;
    t2 = (t2 > 1.0) ? 1.0 : t2;
    t2 = (t2 < -1.0) ? -1.0 : t2;
    eulerAngle.x = asin(t2);
    eulerAngle.y = atan2(t3, t4);
    eulerAngle.z = atan2(t1, t0);
    return eulerAngle;
  }

  Vector3f VehicleWrapper::localOffsetFromGpsAndFusedHeightOffset(
    const Telemetry::GPSFused& target, const Telemetry::GPSFused& origin,
    const float32_t& targetHeight, const float32_t& originHeight)
  {
    Telemetry::Vector3f deltaNed;
    double deltaLon = target.longitude - origin.longitude;
    double deltaLat = target.latitude - origin.latitude;
    deltaNed.x = deltaLat * C_EARTH;
    deltaNed.y = deltaLon * C_EARTH * cos(target.latitude);
    deltaNed.z = targetHeight - originHeight;
    return deltaNed;
  }

  Vector3f VehicleWrapper::vector3FSub(const Vector3f& vectorA,
                                     const Vector3f& vectorB)
  {
    Telemetry::Vector3f result;
    result.x = vectorA.x - vectorB.x;
    result.y = vectorA.y - vectorB.y;
    result.z = vectorA.z - vectorB.z;
    return result;
  }

  template <typename Type>
  int VehicleWrapper::signOfData(Type type)
  {
    return type < 0 ? -1 : 1;
  }

  void VehicleWrapper::horizCommandLimit(float speedFactor, float& commandX,
                                         float& commandY)
  {
    if (fabs(commandX) > speedFactor)
      commandX = signOfData<float>(commandX) * speedFactor;
    if (fabs(commandY) > speedFactor)
      commandY = signOfData<float>(commandY) * speedFactor;
  }

  float32_t VehicleWrapper::vectorNorm(Vector3f v)
  {
    return sqrt(pow(v.x, 2) + pow(v.y, 2) + pow(v.z, 2));
  }

  bool VehicleWrapper::moveByPositionOffset(const JoystickCommand &JoystickCommand,int timeout,
                                            float posThresholdInM,float yawThresholdInDeg)
  {
    if (!vehicle)
    {
      std::cout << "Vehicle is a null value!" << std::endl;
      return false;
    }

    using namespace Telemetry;
    Vector3f offsetDesired;
    offsetDesired.x = JoystickCommand.x;
    offsetDesired.y = JoystickCommand.y;
    offsetDesired.z = JoystickCommand.z;
    float yawDesiredInDeg = JoystickCommand.yaw;

    int responseTimeout = 1;
    int timeoutInMilSec = 40000;
    int controlFreqInHz = 50;  // Hz
    int cycleTimeInMs = 1000 / controlFreqInHz;
    int outOfControlBoundsTimeLimit = 10 * cycleTimeInMs;    // 10 cycles
    int withinControlBoundsTimeReqmt = 100 * cycleTimeInMs;  // 100 cycles
    int elapsedTimeInMs = 0;
    int withinBoundsCounter = 0;
    int outOfBounds = 0;
    int brakeCounter = 0;
    int speedFactor = 2;

    /* now we need position-height broadcast to obtain the real-time altitude of the aircraft, 
    * which is consistent with the altitude closed-loop data of flight control internal position control
    * TO DO:the data will be replaced by new data subscription.
    */
    if (!startGlobalPositionBroadcast())
    {
      return false;
    }
    sleep(1);

    //! get origin position and relative height(from home point)of aircraft.
    Telemetry::TypeMap<TOPIC_GPS_FUSED>::type originGPSPosition =
        vehicle->subscribe->getValue<TOPIC_GPS_FUSED>();
    Telemetry::GlobalPosition currentBroadcastGP = vehicle->broadcast->getGlobalPosition();
    float32_t originHeightBaseHomepoint = currentBroadcastGP.height;
    FlightController::JoystickMode joystickMode = {
      FlightController::HorizontalLogic::HORIZONTAL_POSITION,
      FlightController::VerticalLogic::VERTICAL_POSITION,
      FlightController::YawLogic::YAW_ANGLE,
      FlightController::HorizontalCoordinate::HORIZONTAL_GROUND,
      FlightController::StableMode::STABLE_ENABLE,
    };
    vehicle->flightController->setJoystickMode(joystickMode);

    while (elapsedTimeInMs < timeoutInMilSec) {
      Telemetry::TypeMap<TOPIC_GPS_FUSED>::type currentGPSPosition =
          vehicle->subscribe->getValue<TOPIC_GPS_FUSED>();
      Telemetry::TypeMap<TOPIC_QUATERNION>::type currentQuaternion =
          vehicle->subscribe->getValue<TOPIC_QUATERNION>();
      currentBroadcastGP = vehicle->broadcast->getGlobalPosition();
      float yawInRad = quaternionToEulerAngle(currentQuaternion).z;
      //! get the vector between aircraft and origin point.

      Vector3f localOffset = localOffsetFromGpsAndFusedHeightOffset(currentGPSPosition, originGPSPosition,
                                                                    currentBroadcastGP.height, originHeightBaseHomepoint);
      //! get the vector between aircraft and target point.
      Vector3f offsetRemaining = vector3FSub(offsetDesired, localOffset);

      Vector3f positionCommand = offsetRemaining;
      horizCommandLimit(speedFactor, positionCommand.x, positionCommand.y);

      FlightController::JoystickCommand joystickCommand = {
          positionCommand.x, positionCommand.y,
          offsetDesired.z + originHeightBaseHomepoint, yawDesiredInDeg};

      vehicle->flightController->setJoystickCommand(joystickCommand);

      vehicle->flightController->joystickAction();

      if (vectorNorm(offsetRemaining) < posThresholdInM &&
          std::fabs(yawInRad / DEG2RAD - yawDesiredInDeg) < yawThresholdInDeg) {
        //! 1. We are within bounds; start incrementing our in-bound counter
        withinBoundsCounter += cycleTimeInMs;
      } else {
        if (withinBoundsCounter != 0) {
          //! 2. Start incrementing an out-of-bounds counter
          outOfBounds += cycleTimeInMs;
        }
      }
      //! 3. Reset withinBoundsCounter if necessary
      if (outOfBounds > outOfControlBoundsTimeLimit) {
        withinBoundsCounter = 0;
        outOfBounds = 0;
      }
      //! 4. If within bounds, set flag and break
      if (withinBoundsCounter >= withinControlBoundsTimeReqmt) {
        break;
      }
      usleep(cycleTimeInMs * 1000);
      elapsedTimeInMs += cycleTimeInMs;
    }

    while (brakeCounter < withinControlBoundsTimeReqmt) {
      //! TODO: remove emergencyBrake
      vehicle->flightController->emergencyBrakeAction();
      usleep(cycleTimeInMs * 1000);
      brakeCounter += cycleTimeInMs;
    }

    if (elapsedTimeInMs >= timeoutInMilSec) {
      std::cout << "Task timeout!\n";
      return false;
    }
    return true;
  }

  void VehicleWrapper::velocityAndYawRateCtrl(const JoystickCommand &JoystickCommand, int timeMs)
  {

    Vector3f offsetDesired;
    offsetDesired.x = JoystickCommand.x;
    offsetDesired.y = JoystickCommand.y;
    offsetDesired.z = JoystickCommand.z;
    float yawRate = JoystickCommand.yaw;

    uint32_t originTime  = 0;
    uint32_t currentTime = 0;
    uint32_t elapsedTimeInMs = 0;
    OsdkOsal_GetTimeMs(&originTime);
    OsdkOsal_GetTimeMs(&currentTime);
    elapsedTimeInMs = currentTime - originTime;

    FlightController::JoystickMode joystickMode = {
      FlightController::HorizontalLogic::HORIZONTAL_VELOCITY,
      FlightController::VerticalLogic::VERTICAL_VELOCITY,
      FlightController::YawLogic::YAW_RATE,
      FlightController::HorizontalCoordinate::HORIZONTAL_GROUND,
      FlightController::StableMode::STABLE_ENABLE,
    };

    vehicle->flightController->setJoystickMode(joystickMode);
    FlightController::JoystickCommand joystickCommand = {offsetDesired.x, offsetDesired.y, offsetDesired.z,yawRate};
    vehicle->flightController->setJoystickCommand(joystickCommand);

    while(elapsedTimeInMs <= timeMs)
    {
      vehicle->flightController->joystickAction();
      usleep(20000);
      OsdkOsal_GetTimeMs(&currentTime);
      elapsedTimeInMs = currentTime - originTime;
    }
  }

  bool VehicleWrapper::obtainReleaseCtrlAuthority(bool enableObtain, int timeout)
  {
    if (!vehicle)
    {
      std::cout << "Vehicle is a null value!" << std::endl;
      return false;
    }

    ACK::ErrorCode initAck;

    if (enableObtain)
    {
      initAck = vehicle->control->obtainCtrlAuthority(timeout);
    }
    else
    {
      initAck = vehicle->control->releaseCtrlAuthority(timeout);
    }

    if (ACK::getError(initAck))
    {
      ACK::getErrorCodeMessage(initAck, __func__);
      return false;
    }

    return true;
  }

  bool VehicleWrapper::turnOnOffMotors(bool OnOff,int timeOut)
  {
    if (!vehicle)
    {
      std::cout << "Vehicle is a null value!" << std::endl;
      return false;
    }

    if (OnOff)
    {
      ErrorCode::ErrorCodeType ack = vehicle->flightController->turnOnMotorsSync(timeOut);
      if (ack == ErrorCode::SysCommonErr::Success)
      {
          return true;
      }
    }
    else
    {
      ErrorCode::ErrorCodeType ack = vehicle->flightController->turnOffMotorsSync(timeOut);
      if (ack == ErrorCode::SysCommonErr::Success)
      {
          return true;
      }
    }
    
    return false;
  }

  bool VehicleWrapper::killSwitch(bool enable, char msg[10], int timeOut)
  {
    if (!vehicle)
    {
      std::cout << "Vehicle is a null value!" << std::endl;
      return false;
    }

    ErrorCode::ErrorCodeType ack;
    if (enable)
    {
      ack = vehicle->flightController->killSwitch(DJI::OSDK::FlightActions::KillSwitch::ENABLE,
                                                  timeOut, msg);
    }
    else
    {
      ack = vehicle->flightController->killSwitch(DJI::OSDK::FlightActions::KillSwitch::DISABLE,
                                                  timeOut, msg);
    }

    if (ack == ErrorCode::SysCommonErr::Success)
    {
      return true;
    }

    return false;
  }

  bool VehicleWrapper::emergencyBrake()
  {
    if (!vehicle)
    {
      std::cout << "Vehicle is a null value!" << std::endl;
      return false;
    }

    vehicle->flightController->emergencyBrakeAction();

    return true;
  }

  bool VehicleWrapper::startGlobalPositionBroadcast()
  {
    uint8_t freq[16];
    /* Channels definition for A3/N3/M600
     * 0 - Timestamp
     * 1 - Attitude Quaternions
     * 2 - Acceleration
     * 3 - Velocity (Ground Frame)
     * 4 - Angular Velocity (Body Frame)
     * 5 - Position
     * 6 - GPS Detailed Information
     * 7 - RTK Detailed Information
     * 8 - Magnetometer
     * 9 - RC Channels Data
     * 10 - Gimbal Data
     * 11 - Flight Status
     * 12 - Battery Level
     * 13 - Control Information
     */
    freq[0]  = DataBroadcast::FREQ_HOLD;
    freq[1]  = DataBroadcast::FREQ_HOLD;
    freq[2]  = DataBroadcast::FREQ_HOLD;
    freq[3]  = DataBroadcast::FREQ_HOLD;
    freq[4]  = DataBroadcast::FREQ_HOLD;
    freq[5]  = DataBroadcast::FREQ_50HZ; // This is the only one we want to change
    freq[6]  = DataBroadcast::FREQ_HOLD;
    freq[7]  = DataBroadcast::FREQ_HOLD;
    freq[8]  = DataBroadcast::FREQ_HOLD;
    freq[9]  = DataBroadcast::FREQ_HOLD;
    freq[10] = DataBroadcast::FREQ_HOLD;
    freq[11] = DataBroadcast::FREQ_HOLD;
    freq[12] = DataBroadcast::FREQ_HOLD;
    freq[13] = DataBroadcast::FREQ_HOLD;

    ACK::ErrorCode ack = vehicle->broadcast->setBroadcastFreq(freq, 1);
    if (ACK::getError(ack))
    {
      ACK::getErrorCodeMessage(ack, __func__);
      return false;
    }
    else
    {
      return true;
    }
  }

  GimbalSingleData VehicleWrapper::getGimbalData(const PayloadIndex& payloadIndex)
  {
    GimbalSingleData gimbalSingleData;
    PayloadIndexType index = static_cast<PayloadIndexType>(payloadIndex);
    if (vehicle->isM300() && index <= PAYLOAD_INDEX_2 && index >= PAYLOAD_INDEX_0)
    {
      gimbalSingleData.pitch  = vehicle->subscribe->getValue<TOPIC_THREE_GIMBAL_DATA>().gbData[index].pitch;
      gimbalSingleData.roll   = vehicle->subscribe->getValue<TOPIC_THREE_GIMBAL_DATA>().gbData[index].roll;
      gimbalSingleData.yaw    = vehicle->subscribe->getValue<TOPIC_THREE_GIMBAL_DATA>().gbData[index].yaw;
      gimbalSingleData.status = vehicle->subscribe->getValue<TOPIC_THREE_GIMBAL_DATA>().gbData[index].status;
      gimbalSingleData.mode   = vehicle->subscribe->getValue<TOPIC_THREE_GIMBAL_DATA>().gbData[index].mode;
      return gimbalSingleData;
    }
    else if (index <= PAYLOAD_INDEX_1 && index >= PAYLOAD_INDEX_0)
    {
      gimbalSingleData.pitch  = vehicle->subscribe->getValue<TOPIC_DUAL_GIMBAL_DATA>().gbData[index].pitch;
      gimbalSingleData.roll   = vehicle->subscribe->getValue<TOPIC_DUAL_GIMBAL_DATA>().gbData[index].roll;
      gimbalSingleData.yaw    = vehicle->subscribe->getValue<TOPIC_DUAL_GIMBAL_DATA>().gbData[index].yaw;
      gimbalSingleData.status = vehicle->subscribe->getValue<TOPIC_DUAL_GIMBAL_DATA>().gbData[index].status;
      gimbalSingleData.mode   = vehicle->subscribe->getValue<TOPIC_DUAL_GIMBAL_DATA>().gbData[index].mode;
      return gimbalSingleData;
    }
    else {
      GimbalSingleData data = {0};
      std::cout << "Invalid payload index : %d" << index << std ::endl;
      return data;
    }
  }

  bool VehicleWrapper::rotateGimbal(const PayloadIndex& payloadIndex, const GimbalRotationData& rotationData)
  {
    if (!vehicle || !vehicle->gimbalManager)
    {
      DERROR("vehicle or gimbalManager is a null value.");
      return false;
    }

    ErrorCode::ErrorCodeType retCode;
    PayloadIndexType index = static_cast<PayloadIndexType>(payloadIndex);
    GimbalModule::Rotation rotation;
    rotation.rotationMode = rotationData.rotationMode;
    rotation.pitch        = rotationData.pitch;
    rotation.roll         = rotationData.roll;
    rotation.yaw          = rotationData.yaw;
    rotation.time         = rotationData.time;
    GimbalManager *p = vehicle->gimbalManager;

    DSTATUS("Start to rotate the gimbal %d, (p,r,y) = (%0.2f,%0.2f,%0.2f) mode:%d"
            " time:%0.2fs", index, rotation.pitch, rotation.roll, rotation.yaw,
            rotation.rotationMode, rotation.time);
    retCode = p->rotateSync(index, rotation, 1);
    if (retCode != ErrorCode::SysCommonErr::Success) {
      DERROR("Rotation gimbal fail. Error code : 0x%lX", retCode);
      ErrorCode::printErrorCodeMsg(retCode);
      return false;
    }

    return true;
  }

  bool VehicleWrapper::resetGimbal(const PayloadIndex& payloadIndex)
  {
    if (!vehicle || !vehicle->gimbalManager) {
      DERROR("vehicle or gimbalManager is a null value.");
      return false;
    }

    ErrorCode::ErrorCodeType retCode;
    PayloadIndexType index = static_cast<PayloadIndexType>(payloadIndex);
    GimbalManager *p = vehicle->gimbalManager;

    DSTATUS("Start to rotate the gimbal %d", index);
    retCode = p->resetSync(index, 1);
    if (retCode != ErrorCode::SysCommonErr::Success) {
      std::cout << "Reset gimbal fail. Error code : 0x" << hex << retCode << std::endl;
      ErrorCode::printErrorCodeMsg(retCode);
      return false;
    }

    return true;
  }

  bool VehicleWrapper::getBatteryWholeInfo(DJI::OSDK::BatteryWholeInfo& batteryWholeInfo)
  {
    if (!vehicle) {
      DERROR("vehicle is a null value");
      return false;
    }
    bool result = true;

    result &= vehicle->djiBattery->subscribeBatteryWholeInfo(true);
    OsdkOsal_TaskSleepMs(500);
    result &= vehicle->djiBattery->getBatteryWholeInfo(batteryWholeInfo);

    return result;
  }

  bool VehicleWrapper::getSingleBatteryDynamicInfo(const DJI::OSDK::DJIBattery::RequestSmartBatteryIndex batteryIndex,
                                                   DJI::OSDK::SmartBatteryDynamicInfo& batteryDynamicInfo)
  {
    if (!vehicle) {
      DERROR("vehicle is a null value");
      return false;
    }

    return vehicle->djiBattery->getSingleBatteryDynamicInfo(batteryIndex, batteryDynamicInfo);
  }

  uint8_t VehicleWrapper::outputMFIO(uint8_t mode, uint8_t channel, uint32_t init_on_time_us, uint16_t freq, bool block, uint8_t gpio_set_value)
  {
    int responseTimeout = 1;
    uint32_t initOnTimeUs = init_on_time_us; // us
    uint16_t io_freq = freq;
    switch (MFIO::MODE(mode))
    {
      case MFIO::MODE::MODE_PWM_OUT:
      {
        break;
      }
      case MFIO::MODE::MODE_GPIO_OUT:
      {
        initOnTimeUs = gpio_set_value;
        io_freq = 0;
        break;
      }
      default:
        break;
    }
    std::cout << "Configuring channel\n";
    std::cout << "Channel: " << channel << " configured to output " << freq << "Hz PWM with " << double(initOnTimeUs)/20000 << " duty cycle.\n";
    vehicle->mfio->config(MFIO::MODE(mode), MFIO::CHANNEL(channel), initOnTimeUs, io_freq, responseTimeout);

    if(block == true)
    {
      vehicle->mfio->setValue(MFIO::CHANNEL(channel), initOnTimeUs, responseTimeout);
    }
    else
    {
      vehicle->mfio->setValue(MFIO::CHANNEL(channel), initOnTimeUs);
    }

    return 0;
  }

  uint8_t VehicleWrapper::stopMFIO(uint8_t mode, uint8_t channel)
  {
    int responseTimeout = 1;
    std::cout << "Turning off the PWM signal\n";
    uint32_t digitalValue = 0;
    uint16_t digitalFreq  = 0;
    vehicle->mfio->config(MFIO::MODE(mode), MFIO::CHANNEL(channel), digitalValue, digitalFreq, responseTimeout);
    return 0;
  }

  uint32_t VehicleWrapper::inputMFIO(uint8_t mode, uint8_t channel, bool block)
  {
    int responseTimeout = 1;
    // Set SDK5 to ADC input
    uint32_t initOnTimeUs = 0; // us
    uint16_t pwmFreq      = 0; // Hz

    std::cout << "Configuring channel\n";
    vehicle->mfio->config(MFIO::MODE(mode), MFIO::CHANNEL(channel), initOnTimeUs, pwmFreq, responseTimeout);
    std::cout << "Channel " << channel <<" configured to ADC input.\n";

    ACK::MFIOGet ack;
    ack = vehicle->mfio->getValue(MFIO::CHANNEL(channel), responseTimeout);

    std::cout << "ADC status:" << ack.ack.data << std::endl;
    std::cout << "ADC value:" << ack.value << std::endl;
    return ack.value;
  }

  void VehicleWrapper::sendDataToMSDK(uint8_t* data, uint8_t len)
  {
      if (!vehicle)
      {
        std::cout << "vehicle is a null value!" << std::endl;
      }

      vehicle->mobileDevice->sendDataToMSDK(data, len);
  }

  void VehicleWrapper::sendDataToPSDK(uint8_t* data, uint8_t len)
  {
    if (!vehicle)
    {
      std::cout << "Vehicle is a null value!" << std::endl;
    }

    vehicle->payloadDevice->sendDataToPSDK(data, len);
  }

  bool VehicleWrapper::initWaypointV2(DJI::OSDK::WayPointV2InitSettings *info, int timeout)
  {
    if (!vehicle)
    {
      std::cout << "Vehicle is a null value!" << std::endl;
    }

    ErrorCode::ErrorCodeType ret = vehicle->waypointV2Mission->init(info,timeout);
    if(ret != ErrorCode::SysCommonErr::Success)
    {
       DERROR("Init mission setting ErrorCode:0x%lX\n", ret);
       ErrorCode::printErrorCodeMsg(ret);
       return false;
    }
    else
    {
      DSTATUS("Init mission setting successfully!\n");
    }
    return true;
  }

  bool VehicleWrapper::uploadWaypointV2(int timeout)
  {
    if (!vehicle)
    {
      std::cout << "Vehicle is a null value!" << std::endl;
    }

    ErrorCode::ErrorCodeType ret = vehicle->waypointV2Mission->uploadMission(timeout);
    if(ret != ErrorCode::SysCommonErr::Success)
    {
      DERROR("Upload waypoint v2 mission ErrorCode:0x%lX\n", ret);
      ErrorCode::printErrorCodeMsg(ret);
      return false;
    }
    else
    {
      DSTATUS("Upload waypoint v2 mission successfully!\n");
    }
    return true;

  }

  bool VehicleWrapper::downloadWaypointV2(std::vector<DJI::OSDK::WaypointV2> &mission, int timeout)
  {
    if (!vehicle)
    {
      std::cout << "Vehicle is a null value!" << std::endl;
    }

    ErrorCode::ErrorCodeType ret = vehicle->waypointV2Mission->downloadMission(mission, timeout);
    if(ret != ErrorCode::SysCommonErr::Success)
    {
      DERROR("Download waypoint v2 mission ErrorCode:0x%lX\n", ret);
      ErrorCode::printErrorCodeMsg(ret);
      return false;
    }
    else
    {
      DSTATUS("Download waypoint v2 mission successfully!\n");
    }
    return true;
  }

  bool VehicleWrapper::uploadWaypointV2Actions(std::vector<DJIWaypointV2Action> &actions, int timeout)
  {
    if (!vehicle)
    {
      std::cout << "Vehicle is a null value!" << std::endl;
    }

    ErrorCode::ErrorCodeType ret = vehicle->waypointV2Mission->uploadAction(actions,timeout);
    if(ret != ErrorCode::SysCommonErr::Success)
    {
      DERROR("Upload waypoint v2 actions ErrorCode:0x%lX\n", ret);
      ErrorCode::printErrorCodeMsg(ret);
      return false;
    }
    else
    {
      DSTATUS("Upload waypoint v2 actions successfully!\n");
    }
    return true;
  }

  bool VehicleWrapper::startWaypointV2Mission(int timeout)
  {
    if (!vehicle)
    {
      std::cout << "Vehicle is a null value!" << std::endl;
    }

    ErrorCode::ErrorCodeType ret = vehicle->waypointV2Mission->start(timeout);
    if(ret != ErrorCode::SysCommonErr::Success)
    {
      DERROR("Start waypoint v2 mission ErrorCode:0x%lX\n", ret);
      ErrorCode::printErrorCodeMsg(ret);
      return false;
    }
    else
    {
      DSTATUS("Start waypoint v2 mission successfully!\n");
    }
    return true;

  }

  bool VehicleWrapper::stopWaypointV2Mission(int timeout)
  {
    if (!vehicle)
    {
      std::cout << "Vehicle is a null value!" << std::endl;
    }

    ErrorCode::ErrorCodeType ret = vehicle->waypointV2Mission->stop(timeout);
    if(ret != ErrorCode::SysCommonErr::Success)
    {
      DERROR("stop waypoint v2 mission ErrorCode:0x%lX\n", ret);
      ErrorCode::printErrorCodeMsg(ret);
      return false;
    }
    else
    {
      DSTATUS("stop waypoint v2 mission successfully!\n");
    }
    return true;
  }

  bool VehicleWrapper::pauseWaypointV2Mission(int timeout)
  {
    if (!vehicle)
    {
      std::cout << "Vehicle is a null value!" << std::endl;
    }

    ErrorCode::ErrorCodeType ret = vehicle->waypointV2Mission->pause(timeout);
    if(ret != ErrorCode::SysCommonErr::Success)
    {
      DERROR("pause waypoint v2 mission ErrorCode:0x%lX\n", ret);
      ErrorCode::printErrorCodeMsg(ret);
      return false;
    }
    else
    {
      DSTATUS("pause waypoint v2 mission successfully!\n");
    }
    return true;
  }

bool VehicleWrapper::resumeWaypointV2Mission(int timeout)
{
  if (!vehicle)
  {
    std::cout << "Vehicle is a null value!" << std::endl;
  }

  ErrorCode::ErrorCodeType ret = vehicle->waypointV2Mission->resume(timeout);
  if(ret != ErrorCode::SysCommonErr::Success)
  {
    DERROR("resume waypoint v2 mission ErrorCode:0x%lX\n", ret);
    ErrorCode::printErrorCodeMsg(ret);
    return false;
  }
  else
  {
    DSTATUS("resume waypoint v2 mission successfully!\n");
  }
  return true;
}

bool VehicleWrapper::setGlobalCruiseSpeed(const float32_t &cruiseSpeed, int timeout)
{
  if (!vehicle)
  {
    std::cout << "Vehicle is a null value!" << std::endl;
    return false;
  }

  ErrorCode::ErrorCodeType ret = vehicle->waypointV2Mission->setGlobalCruiseSpeed(cruiseSpeed, timeout);
  if(ret !=  ErrorCode::SysCommonErr::Success)
  {
    DERROR("Set glogal cruise speed %f m/s failed ErrorCode:0x%lX\n", cruiseSpeed, ret);
    ErrorCode::printErrorCodeMsg(ret);
    return false;
  }
  DSTATUS("Current cruise speed is: %f m/s\n", cruiseSpeed);
  return true;
}

bool VehicleWrapper::getGlobalCruiseSpeed(float32_t &cruiseSpeed, int timeout)
{
  if (!vehicle)
  {
    std::cout << "Vehicle is a null value!" << std::endl;
    return false;
  }

  ErrorCode::ErrorCodeType ret = vehicle->waypointV2Mission->getGlobalCruiseSpeed(cruiseSpeed, timeout);
  if(ret !=  ErrorCode::SysCommonErr::Success)
  {
    DERROR("Get glogal cruise speed failed ErrorCode:0x%lX\n", ret);
    ErrorCode::printErrorCodeMsg(ret);
    return false;
  }
  DSTATUS("Current cruise speed is: %f m/s\n",cruiseSpeed);
  return true;
}
bool VehicleWrapper::RegisterMissionEventCallback(void *userData, PushCallback cb)
{
  if (!vehicle)
  {
    std::cout << "Vehicle is a null value!" << std::endl;
    return false;
  }
  vehicle->waypointV2Mission->RegisterMissionEventCallback(userData, cb);

  return true;
}
bool VehicleWrapper::RegisterMissionStateCallback(void *userData, PushCallback cb)
{
  if (!vehicle)
  {
    std::cout << "Vehicle is a null value!" << std::endl;
    return false;
  }
  vehicle->waypointV2Mission->RegisterMissionStateCallback(userData, cb);

  return true;
}

bool VehicleWrapper::enableSubscribeHMSInfo(bool enable, uint32_t timeOutMs)
{
  if (!vehicle)
  {
    std::cout << "Vehicle is a null value!" << std::endl;
    return false;
  }

  return vehicle->djiHms->subscribeHMSInf(enable, timeOutMs);
}

bool VehicleWrapper::getHMSListInfo(HMSPushPacket& hmsPushPacket)
{
  if (!vehicle)
  {
    std::cout << "Vehicle is a null value!" << std::endl;
    return false;
  }

  DJI::OSDK::HMSPushPacket tempHMSPushPacket;
  tempHMSPushPacket = vehicle->djiHms->getHMSPushPacket();

  hmsPushPacket.timeStamp = tempHMSPushPacket.timeStamp;
  hmsPushPacket.hmsPushData.msgVersion = tempHMSPushPacket.hmsPushData.msgVersion;
  hmsPushPacket.hmsPushData.msgIndex = tempHMSPushPacket.hmsPushData.msgIndex;
  hmsPushPacket.hmsPushData.msgEnd = tempHMSPushPacket.hmsPushData.msgEnd;
  hmsPushPacket.hmsPushData.globalIndex = tempHMSPushPacket.hmsPushData.globalIndex;
  if (tempHMSPushPacket.hmsPushData.errList.size())
  {
    hmsPushPacket.hmsPushData.errList.clear();
    hmsPushPacket.hmsPushData.errList.resize(tempHMSPushPacket.hmsPushData.errList.size());

    for (int i = 0; i < hmsPushPacket.hmsPushData.errList.size(); i++)
    {
      hmsPushPacket.hmsPushData.errList[i].alarmID     = tempHMSPushPacket.hmsPushData.errList[i].alarmID;
      hmsPushPacket.hmsPushData.errList[i].reportLevel = tempHMSPushPacket.hmsPushData.errList[i].reportLevel;
      hmsPushPacket.hmsPushData.errList[i].sensorIndex = tempHMSPushPacket.hmsPushData.errList[i].sensorIndex;
      //DSTATUS("0x%08x,%d,%d", hmsPushPacket.hmsPushData.errList[i].alarmID, hmsPushPacket.hmsPushData.errList[i].sensorIndex,
      //hmsPushPacket.hmsPushData.errList[i].reportLevel);
    }
  }
  return true;
}

bool VehicleWrapper::getHMSDeviceIndex(uint8_t& deviceIndex)
{
  if (!vehicle)
  {
    std::cout << "Vehicle is a null value!" << std::endl;
    return false;
  }

  deviceIndex = vehicle->djiHms->getDeviceIndex();
  return true;
}

#ifdef ADVANCED_SENSING
  bool VehicleWrapper::startFPVCameraStream(CameraImageCallback cb, void * cbParam)
  {
    if (!vehicle->advancedSensing)
    {
      return false;
    }
    else
    {
      return vehicle->advancedSensing->startFPVCameraStream(cb, cbParam);
    }
  }

  bool VehicleWrapper::startMainCameraStream(CameraImageCallback cb, void * cbParam)
  {
    if (!vehicle->advancedSensing)
    {
      return false;
    }
    else
    {
      return vehicle->advancedSensing->startMainCameraStream(cb, cbParam);
    }
  }

  bool VehicleWrapper::stopFPVCameraStream()
  {
    if (!vehicle->advancedSensing)
    {
      return false;
    }
    else
    {
      vehicle->advancedSensing->stopFPVCameraStream();
      return true;
    }
  }

  bool VehicleWrapper::stopMainCameraStream()
  {
    if (!vehicle->advancedSensing)
    {
      return false;
    }
    else
    {
      vehicle->advancedSensing->stopMainCameraStream();
      return true;
    }
  }

  bool VehicleWrapper::startH264Stream(LiveView::LiveViewCameraPosition pos, H264Callback cb, void *userData)
  {
    if (!vehicle->advancedSensing)
    {
      return false;
    }
    else
    {
      vehicle->advancedSensing->startH264Stream(pos, cb, userData);
      return true;
    }
  }

  bool VehicleWrapper::stopH264Stream(LiveView::LiveViewCameraPosition pos)
  {
    if (!vehicle->advancedSensing)
    {
      return false;
    }
    else
    {
      vehicle->advancedSensing->stopH264Stream(pos);
      return true;
    }
  }

  void VehicleWrapper::subscribeStereoImages(const dji_osdk_ros::ImageSelection *select, VehicleCallBack callback, UserData userData)
  {
      DJI::OSDK::AdvancedSensing::ImageSelection select_;
      memcpy(&select_, select, sizeof(DJI::OSDK::AdvancedSensing::ImageSelection));
      vehicle->advancedSensing->subscribeStereoImages(&select_, callback, userData);
  }

  void VehicleWrapper::subscribeFrontStereoVGA(const uint8_t freq, VehicleCallBack callback, UserData userData)
  {
      vehicle->advancedSensing->subscribeFrontStereoVGA(freq, callback, userData);
  }

  void VehicleWrapper::subscribeFrontStereoDisparity(VehicleCallBack callback, UserData userData)
  {
      vehicle->advancedSensing->subscribeFrontStereoDisparity(callback, userData);
  }

  void VehicleWrapper::unsubscribeStereoImages()
  {
      vehicle->advancedSensing->unsubscribeStereoImages();
  }
  
  void VehicleWrapper::unsubscribeVGAImages()
  {
      vehicle->advancedSensing->unsubscribeVGAImages();
  }

  void VehicleWrapper::setAcmDevicePath(const std::string& acm_path)
  {
      if (vehicle->advancedSensing)
      {
        vehicle->advancedSensing->setAcmDevicePath(acm_path.c_str());
      }
  }
#endif

  bool VehicleWrapper::isM100()
  {
    return(vehicle->isM100());
  }

  bool VehicleWrapper::isM200V2()
  {
    return(vehicle->isM210V2());
  }

  bool VehicleWrapper::isM300()
  {
    return(vehicle->isM300());
  }

  bool VehicleWrapper::isM600()
  {
    return(vehicle->isLegacyM600());
  }

  void VehicleWrapper::setUpM100DefaultFreq(uint8_t freq[16])
  {
    /* Channels definition for M100
     * 0 - Timestamp
     * 1 - Attitude Quaterniouns
     * 2 - Acceleration
     * 3 - Velocity (Ground Frame)
     * 4 - Angular Velocity (Body Frame)
     * 5 - Position
     * 6 - Magnetometer
     * 7 - RC Channels Data
     * 8 - Gimbal Data
     * 9 - Flight Status
     * 10 - Battery Level
     * 11 - Control Information
     */
    freq[0]  = DataBroadcast::FREQ_1HZ;
    freq[1]  = DataBroadcast::FREQ_10HZ;
    freq[2]  = DataBroadcast::FREQ_50HZ;
    freq[3]  = DataBroadcast::FREQ_100HZ;
    freq[4]  = DataBroadcast::FREQ_50HZ;
    freq[5]  = DataBroadcast::FREQ_10HZ;
    freq[6]  = DataBroadcast::FREQ_1HZ;
    freq[7]  = DataBroadcast::FREQ_10HZ;
    freq[8]  = DataBroadcast::FREQ_50HZ;
    freq[9]  = DataBroadcast::FREQ_100HZ;
    freq[10] = DataBroadcast::FREQ_50HZ;
    freq[11] = DataBroadcast::FREQ_10HZ;
  }

  void VehicleWrapper::setUpA3N3DefaultFreq(uint8_t freq[16])
  {
    /* Channels definition for A3/N3/M600
     * 0 - Timestamp
     * 1 - Attitude Quaterniouns
     * 2 - Acceleration
     * 3 - Velocity (Ground Frame)
     * 4 - Angular Velocity (Body Frame)
     * 5 - Position
     * 6 - GPS Detailed Information
     * 7 - RTK Detailed Information
     * 8 - Magnetometer
     * 9 - RC Channels Data
     * 10 - Gimbal Data
     * 11 - Flight Statusack
     * 12 - Battery Level
     * 13 - Control Information
     * 14 - Compass Data
     */
    freq[0]  = DataBroadcast::FREQ_50HZ;
    freq[1]  = DataBroadcast::FREQ_50HZ;
    freq[2]  = DataBroadcast::FREQ_50HZ;
    freq[3]  = DataBroadcast::FREQ_50HZ;
    freq[4]  = DataBroadcast::FREQ_50HZ;
    freq[5]  = DataBroadcast::FREQ_50HZ;
    freq[6]  = DataBroadcast::FREQ_0HZ; // Don't send GPS details
    freq[7]  = DataBroadcast::FREQ_0HZ; // Don't send RTK
    freq[8]  = DataBroadcast::FREQ_0HZ; // Don't send Mag
    freq[9]  = DataBroadcast::FREQ_50HZ;
    freq[10] = DataBroadcast::FREQ_50HZ;
    freq[11] = DataBroadcast::FREQ_10HZ;
    freq[12] = DataBroadcast::FREQ_1HZ;
    freq[13] = DataBroadcast::FREQ_1HZ;
    freq[14] = DataBroadcast::FREQ_1HZ;
  }

  ACK::ErrorCode VehicleWrapper::setBroadcastFreq(uint8_t* dataLenIs16, int timeout)
  {
      return vehicle->broadcast->setBroadcastFreq(dataLenIs16, timeout);
  }

  void VehicleWrapper::setUserBroadcastCallback(VehicleCallBack callback,
                                                UserData        userData)
  {
      vehicle->broadcast->setUserBroadcastCallback(callback, userData);
  }

  uint16_t VehicleWrapper::getPassFlag()
  {
      return vehicle->broadcast->getPassFlag();
  }

  Telemetry::RC VehicleWrapper::getRC()
  {
      return vehicle->broadcast->getRC();
  }

  Telemetry::Quaternion VehicleWrapper::getQuaternion()
  {
      return vehicle->broadcast->getQuaternion();
  }

  Telemetry::Vector3f VehicleWrapper::getAcceleration()
  {
      return vehicle->broadcast->getAcceleration();
  }

  Telemetry::Vector3f VehicleWrapper::getAngularRate()
  {
      return vehicle->broadcast->getAngularRate();
  }

  Telemetry::GlobalPosition VehicleWrapper::getGlobalPosition()
  {
     return vehicle->broadcast->getGlobalPosition();
  }

  Telemetry::Vector3f VehicleWrapper::getVelocity()
  {
     return vehicle->broadcast->getVelocity();
  }

  Telemetry::Battery VehicleWrapper::getBatteryInfo()
  {
     return vehicle->broadcast->getBatteryInfo();
  }

  Telemetry::Status VehicleWrapper::getStatus()
  {
      return vehicle->broadcast->getStatus();
  }

  Telemetry::Gimbal VehicleWrapper::getGimbal()
  {
     return vehicle->broadcast->getGimbal();
  }

  void VehicleWrapper::setFromMSDKCallback(VehicleCallBack callback, UserData userData)
  {
     vehicle->mobileDevice->setFromMSDKCallback(callback, userData);
  }

  void VehicleWrapper::setFromPSDKCallback(VehicleCallBack callback, UserData userData)
  {
     vehicle->payloadDevice->setFromPSDKCallback(callback, userData);
  }

  void VehicleWrapper::subscribeNMEAMsgs(VehicleCallBack cb, void *userData)
  {
     vehicle->hardSync->subscribeNMEAMsgs(cb, userData);
  }

  void VehicleWrapper::subscribeUTCTime(VehicleCallBack cb, void *userData)
  {
     vehicle->hardSync->subscribeUTCTime(cb, userData);
  }

  void VehicleWrapper::subscribeFCTimeInUTCRef(VehicleCallBack cb, void *userData)
  {
      vehicle->hardSync->subscribeFCTimeInUTCRef(cb, userData);
  }

  void VehicleWrapper::subscribePPSSource(VehicleCallBack cb, void *userData)
  {
      vehicle->hardSync->subscribePPSSource(cb, userData);
  }

  void VehicleWrapper::unsubscribeNMEAMsgs()
  {
      vehicle->hardSync->unsubscribeNMEAMsgs();
  }

  void VehicleWrapper::unsubscribeUTCTime()
  {
      vehicle->hardSync->unsubscribeUTCTime();
  }

  void VehicleWrapper::unsubscribeFCTimeInUTCRef()
  {
      vehicle->hardSync->unsubscribeFCTimeInUTCRef();
  }

  void VehicleWrapper::unsubscribePPSSource()
  {
      vehicle->hardSync->unsubscribePPSSource();
  }

  ACK::ErrorCode VehicleWrapper::verify(int timeout)
  {
      return vehicle->subscribe->verify(timeout);
  }

  /*!
   * @details Setup members of package[packageID]
   *          Do basic gate keeping. No api->send call involved
   */
  bool VehicleWrapper::initPackageFromTopicList(int packageID, int numberOfTopics,
                                                TopicName* topicList,
                                                bool sendTimeStamp, uint16_t freq)
  {
      return vehicle->subscribe->initPackageFromTopicList(packageID, numberOfTopics,
                                                 topicList, sendTimeStamp, freq);
  }

  ACK::ErrorCode VehicleWrapper::startPackage(int packageID, int timeout)
  {
      return vehicle->subscribe->startPackage(packageID, timeout);
  }

  /*!
   * @brief Non-blocking call for start package
   * @param packageID
   * @param timeout
   * @return
   */
  ACK::ErrorCode VehicleWrapper::removePackage(int packageID, int timeout)
  {
      return vehicle->subscribe->removePackage(packageID, timeout);
  }

  /*!
   * @brief Register a callback function after package[packageID] is received
   * @param packageID
   * @param userFunctionAfterPackageExtraction
   */
  void VehicleWrapper::registerUserPackageUnpackCallback(int packageID, VehicleCallBack userFunctionAfterPackageExtraction,
                                                         UserData userData)
  {
      vehicle->subscribe->registerUserPackageUnpackCallback(packageID, userFunctionAfterPackageExtraction, userData);
  }

  Version::FirmWare VehicleWrapper::getFwVersion() const
  {
      return vehicle->getFwVersion();
  }

  char* VehicleWrapper::getHwVersion() const
  {
      return vehicle->getHwVersion();
  }

  ErrorCode::ErrorCodeType VehicleWrapper::initGimbalModule(dji_osdk_ros::PayloadIndex index,
                                                            const char* name)
  {
      return vehicle->gimbalManager->initGimbalModule(static_cast<DJI::OSDK::PayloadIndexType>(index), name);
  }

  ErrorCode::ErrorCodeType VehicleWrapper::initCameraModule(dji_osdk_ros::PayloadIndex index,
                                                            const char* name)
  {
      return vehicle->cameraManager->initCameraModule(static_cast<DJI::OSDK::PayloadIndexType>(index), name);
  }
}
