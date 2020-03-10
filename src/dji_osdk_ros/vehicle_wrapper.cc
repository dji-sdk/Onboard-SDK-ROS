/**
  * @Copyright (C) 2020 All rights reserved.
  * @date: 2020
  * @file: vehicle_wrapper.cc
  * @version: v0.0.1
  * @author: kevin.hoo@dji.com
  * @create_date: 2020-03-08 18:11:38
  * @last_modified_date: 2020-03-10 13:39:17
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

  bool VehicleWrapper::takePicture()
  {
    // Take picture
    std::cout << "Ensure SD card is present.\n";
    std::cout << "Taking picture..\n";
    vehicle->camera->shootPhoto();
    std::cout << "Check DJI GO App or SD card for a new picture.\n";

    std::cout << "Setting new Gimbal rotation angle to [0,-50, 0] using absolute "
                 "control:\n";
    return true;
  }

  bool VehicleWrapper::startCaptureVideo()
  {
    std::cout << "Ensure SD card is present.\n";
    std::cout << "Starting video..\n";
    vehicle->camera->videoStart();
    return true;
  }

  bool VehicleWrapper::stopCaptureVideo()
  {
    // Stop the video
    std::cout << "Stopping video...\n";
    vehicle->camera->videoStop();
    std::cout << "Check DJI GO App or SD card for a new video.\n";
    return true;
  }

  bool VehicleWrapper::monitoredTakeoff(ACK::ErrorCode& ack, int timeout)
  {
    using namespace Telemetry;
    //@todo: remove this once the getErrorCode function signature changes
    char func[50];
    int  pkgIndex;

    if (vehicle->isM100() && vehicle->isLegacyM600())
    {
      // Telemetry: Verify the subscription
      ack = vehicle->subscribe->verify(timeout);
      if (ACK::getError(ack) != ACK::SUCCESS)
      {
        ACK::getErrorCodeMessage(ack, func);
        return false;
      }

      // Telemetry: Subscribe to flight status and mode at freq 10 Hz
      pkgIndex                  = 0;
      int       freq            = 10;
      TopicName topicList10Hz[] = { TOPIC_STATUS_FLIGHT,
                                    TOPIC_STATUS_DISPLAYMODE };
      int  numTopic        = sizeof(topicList10Hz) / sizeof(topicList10Hz[0]);
      bool enableTimestamp = false;

      bool pkgStatus = vehicle->subscribe->initPackageFromTopicList(
        pkgIndex, numTopic, topicList10Hz, enableTimestamp, freq);
      if (!(pkgStatus))
      {
        return pkgStatus;
      }
      ack = vehicle->subscribe->startPackage(pkgIndex, timeout);
      if (ACK::getError(ack) != ACK::SUCCESS)
      {
        ACK::getErrorCodeMessage(ack, func);
        // Cleanup before return
        vehicle->subscribe->removePackage(pkgIndex, timeout);
        return false;
      }
    }

    // Start takeoff
    ack = vehicle->control->takeoff(timeout);
    if (ACK::getError(ack) != ACK::SUCCESS)
    {
      ACK::getErrorCodeMessage(ack, func);
      return false;
    }

    // First check: Motors started
    int motorsNotStarted = 0;
    int timeoutCycles    = 20;

    if (vehicle->isM100() && vehicle->isLegacyM600())
    {
      while (vehicle->subscribe->getValue<TOPIC_STATUS_FLIGHT>() !=
             VehicleStatus::FlightStatus::ON_GROUND &&
             vehicle->subscribe->getValue<TOPIC_STATUS_DISPLAYMODE>() !=
             VehicleStatus::DisplayMode::MODE_ENGINE_START &&
             motorsNotStarted < timeoutCycles)
      {
        motorsNotStarted++;
        usleep(100000);
      }

      if (motorsNotStarted == timeoutCycles)
      {
        std::cout << "Takeoff failed. Motors are not spinning." << std::endl;
        // Cleanup
        if (vehicle->isM100() && vehicle->isLegacyM600())
        {
          vehicle->subscribe->removePackage(0, timeout);
        }
        return false;
      }
      else
      {
        std::cout << "Motors spinning...\n";
      }
    }
    else if (vehicle->isLegacyM600())
    {
      while ((vehicle->broadcast->getStatus().flight <
              DJI::OSDK::VehicleStatus::FlightStatus::ON_GROUND) &&
             motorsNotStarted < timeoutCycles)
      {
        motorsNotStarted++;
        usleep(100000);
      }

      if (motorsNotStarted < timeoutCycles)
      {
        std::cout << "Successful TakeOff!" << std::endl;
      }
    }
    else // M100
    {
      while ((vehicle->broadcast->getStatus().flight <
              DJI::OSDK::VehicleStatus::M100FlightStatus::TAKEOFF) &&
             motorsNotStarted < timeoutCycles)
      {
        motorsNotStarted++;
        usleep(100000);
      }

      if (motorsNotStarted < timeoutCycles)
      {
        std::cout << "Successful TakeOff!" << std::endl;
      }
    }

    // Second check: In air
    int stillOnGround = 0;
    timeoutCycles     = 110;

    if (vehicle->isM100() && vehicle->isLegacyM600())
    {
      while (vehicle->subscribe->getValue<TOPIC_STATUS_FLIGHT>() !=
             VehicleStatus::FlightStatus::IN_AIR &&
             (vehicle->subscribe->getValue<TOPIC_STATUS_DISPLAYMODE>() !=
              VehicleStatus::DisplayMode::MODE_ASSISTED_TAKEOFF ||
              vehicle->subscribe->getValue<TOPIC_STATUS_DISPLAYMODE>() !=
              VehicleStatus::DisplayMode::MODE_AUTO_TAKEOFF) &&
             stillOnGround < timeoutCycles)
      {
        stillOnGround++;
        usleep(100000);
      }

      if (stillOnGround == timeoutCycles)
      {
        std::cout << "Takeoff failed. Aircraft is still on the ground, but the "
                     "motors are spinning."
                  << std::endl;
        // Cleanup
        if (vehicle->isM100() && vehicle->isLegacyM600())
        {
          vehicle->subscribe->removePackage(0, timeout);
        }
        return false;
      }
      else
      {
        std::cout << "Ascending...\n";
      }
    }
    else if (vehicle->isLegacyM600())
    {
      while ((vehicle->broadcast->getStatus().flight <
              DJI::OSDK::VehicleStatus::FlightStatus::IN_AIR) &&
             stillOnGround < timeoutCycles)
      {
        stillOnGround++;
        usleep(100000);
      }

      if (stillOnGround < timeoutCycles)
      {
        std::cout << "Aircraft in air!" << std::endl;
      }
    }
    else // M100
    {
      while ((vehicle->broadcast->getStatus().flight !=
              DJI::OSDK::VehicleStatus::M100FlightStatus::IN_AIR_STANDBY) &&
             stillOnGround < timeoutCycles)
      {
        stillOnGround++;
        usleep(100000);
      }

      if (stillOnGround < timeoutCycles)
      {
        std::cout << "Aircraft in air!" << std::endl;
      }
    }

    // Final check: Finished takeoff
    if (vehicle->isM100() && vehicle->isLegacyM600())
    {
      while (vehicle->subscribe->getValue<TOPIC_STATUS_DISPLAYMODE>() ==
             VehicleStatus::DisplayMode::MODE_ASSISTED_TAKEOFF ||
             vehicle->subscribe->getValue<TOPIC_STATUS_DISPLAYMODE>() ==
             VehicleStatus::DisplayMode::MODE_AUTO_TAKEOFF)
      {
        sleep(1);
      }

      if (vehicle->isM100() && vehicle->isLegacyM600())
      {
        if (vehicle->subscribe->getValue<TOPIC_STATUS_DISPLAYMODE>() !=
            VehicleStatus::DisplayMode::MODE_P_GPS ||
            vehicle->subscribe->getValue<TOPIC_STATUS_DISPLAYMODE>() !=
            VehicleStatus::DisplayMode::MODE_ATTITUDE)
        {
          std::cout << "Successful takeoff!\n";
        }
        else
        {
          std::cout
            << "Takeoff finished, but the aircraft is in an unexpected mode. "
               "Please connect DJI GO.\n";
          vehicle->subscribe->removePackage(0, timeout);
          return false;
        }
      }
    }
    else
    {
      float32_t                 delta;
      Telemetry::GlobalPosition currentHeight;
      Telemetry::GlobalPosition deltaHeight =
        vehicle->broadcast->getGlobalPosition();

      do
      {
        sleep(4);
        currentHeight = vehicle->broadcast->getGlobalPosition();
        delta         = std::fabs(currentHeight.altitude - deltaHeight.altitude);
        deltaHeight.altitude = currentHeight.altitude;
      } while (delta >= 0.009);

      std::cout << "Aircraft hovering at " << currentHeight.altitude << "m!\n";
    }

    // Cleanup
    if (vehicle->isM100() && vehicle->isLegacyM600())
    {
      ack = vehicle->subscribe->removePackage(pkgIndex, timeout);
      if (ACK::getError(ack))
      {
        std::cout
          << "Error unsubscribing; please restart the drone/FC to get back "
             "to a clean state.\n";
      }
    }

    return true;
  }

  bool VehicleWrapper::monitoredLanding(ACK::ErrorCode& ack, int timeout)
  {
    using namespace Telemetry;
    //@todo: remove this once the getErrorCode function signature changes
    char func[50];
    int  pkgIndex;

    if (vehicle->isM100() && vehicle->isLegacyM600())
    {
      // Telemetry: Verify the subscription
      ack = vehicle->subscribe->verify(timeout);
      if (ACK::getError(ack) != ACK::SUCCESS)
      {
        ACK::getErrorCodeMessage(ack, func);
        return false;
      }

      // Telemetry: Subscribe to flight status and mode at freq 10 Hz
      pkgIndex                  = 0;
      int       freq            = 10;
      TopicName topicList10Hz[] = { TOPIC_STATUS_FLIGHT,
                                    TOPIC_STATUS_DISPLAYMODE };
      int  numTopic        = sizeof(topicList10Hz) / sizeof(topicList10Hz[0]);
      bool enableTimestamp = false;

      bool pkgStatus = vehicle->subscribe->initPackageFromTopicList(
        pkgIndex, numTopic, topicList10Hz, enableTimestamp, freq);
      if (!(pkgStatus))
      {
        return pkgStatus;
      }
      ack = vehicle->subscribe->startPackage(pkgIndex, timeout);
      if (ACK::getError(ack) != ACK::SUCCESS)
      {
        ACK::getErrorCodeMessage(ack, func);
        // Cleanup before return
        vehicle->subscribe->removePackage(pkgIndex, timeout);
        return false;
      }
    }

    // Start landing
    ack = vehicle->control->land(timeout);
    if (ACK::getError(ack) != ACK::SUCCESS)
    {
      ACK::getErrorCodeMessage(ack, func);
      return false;
    }

    // First check: Landing started
    int landingNotStarted = 0;
    int timeoutCycles     = 20;

    if (vehicle->isM100() && vehicle->isLegacyM600())
    {
      while (vehicle->subscribe->getValue<TOPIC_STATUS_DISPLAYMODE>() !=
             VehicleStatus::DisplayMode::MODE_AUTO_LANDING &&
             landingNotStarted < timeoutCycles)
      {
        landingNotStarted++;
        usleep(100000);
      }
    }
    else if (vehicle->isM100())
    {
      while (vehicle->broadcast->getStatus().flight !=
             DJI::OSDK::VehicleStatus::M100FlightStatus::LANDING &&
             landingNotStarted < timeoutCycles)
      {
        landingNotStarted++;
        usleep(100000);
      }
    }

    if (landingNotStarted == timeoutCycles)
    {
      std::cout << "Landing failed. Aircraft is still in the air." << std::endl;
      if (vehicle->isM100() && vehicle->isLegacyM600())
      {
        // Cleanup before return
        ack = vehicle->subscribe->removePackage(pkgIndex, timeout);
        if (ACK::getError(ack)) {
          std::cout << "Error unsubscribing; please restart the drone/FC to get "
                       "back to a clean state.\n";
        }
      }
      return false;
    }
    else
    {
      std::cout << "Landing...\n";
    }

    // Second check: Finished landing
    if (vehicle->isM100() && vehicle->isLegacyM600())
    {
      while (vehicle->subscribe->getValue<TOPIC_STATUS_DISPLAYMODE>() ==
             VehicleStatus::DisplayMode::MODE_AUTO_LANDING &&
             vehicle->subscribe->getValue<TOPIC_STATUS_FLIGHT>() ==
             VehicleStatus::FlightStatus::IN_AIR)
      {
        sleep(1);
      }

      if (vehicle->subscribe->getValue<TOPIC_STATUS_DISPLAYMODE>() !=
          VehicleStatus::DisplayMode::MODE_P_GPS ||
          vehicle->subscribe->getValue<TOPIC_STATUS_DISPLAYMODE>() !=
          VehicleStatus::DisplayMode::MODE_ATTITUDE)
      {
        std::cout << "Successful landing!\n";
      }
      else
      {
        std::cout
          << "Landing finished, but the aircraft is in an unexpected mode. "
             "Please connect DJI GO.\n";
        ack = vehicle->subscribe->removePackage(pkgIndex, timeout);
        if (ACK::getError(ack))
        {
          std::cout << "Error unsubscribing; please restart the drone/FC to get "
                       "back to a clean state.\n";
        }
        return false;
      }
    }
    else if (vehicle->isLegacyM600())
    {
      while (vehicle->broadcast->getStatus().flight >
             DJI::OSDK::VehicleStatus::FlightStatus::STOPED)
      {
        sleep(1);
      }

      Telemetry::GlobalPosition gp;
      do
      {
        sleep(2);
        gp = vehicle->broadcast->getGlobalPosition();
      } while (gp.altitude != 0);

      if (gp.altitude != 0)
      {
        std::cout
          << "Landing finished, but the aircraft is in an unexpected mode. "
             "Please connect DJI GO.\n";
        return false;
      }
      else
      {
        std::cout << "Successful landing!\n";
      }
    }
    else // M100
    {
      while (vehicle->broadcast->getStatus().flight ==
             DJI::OSDK::VehicleStatus::M100FlightStatus::FINISHING_LANDING)
      {
        sleep(1);
      }

      Telemetry::GlobalPosition gp;
      do
      {
        sleep(2);
        gp = vehicle->broadcast->getGlobalPosition();
      } while (gp.altitude != 0);

      if (gp.altitude != 0)
      {
        std::cout
          << "Landing finished, but the aircraft is in an unexpected mode. "
             "Please connect DJI GO.\n";
        return false;
      }
      else
      {
        std::cout << "Successful landing!\n";
      }
    }

    // Cleanup
    if (vehicle->isM100() && vehicle->isLegacyM600())
    {
      ack = vehicle->subscribe->removePackage(pkgIndex, timeout);
      if (ACK::getError(ack))
      {
        std::cout
          << "Error unsubscribing; please restart the drone/FC to get back "
             "to a clean state.\n";
      }
    }

    return true;
  }

  bool VehicleWrapper::goHome(int timeout)
  {
    auto ack = vehicle->control->goHome(timeout);
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


  bool VehicleWrapper::moveByPositionOffset(ACK::ErrorCode& ack, int timeout, MoveOffset& p_offset)
  {
    // Set timeout: this timeout is the time you allow the drone to take to finish
    // the
    // mission
    using namespace Telemetry;
    auto xOffsetDesired = p_offset.x;
    auto yOffsetDesired = p_offset.y;
    auto zOffsetDesired = p_offset.z;
    auto yawDesired = p_offset.yaw;
    auto posThresholdInM = p_offset.pos_threshold;
    auto yawThresholdInDeg = p_offset.yaw_threshold;

    int responseTimeout              = 1;
    int timeoutInMilSec              = 40000;
    int controlFreqInHz              = 50; // Hz
    int cycleTimeInMs                = 1000 / controlFreqInHz;
    int outOfControlBoundsTimeLimit  = 10 * cycleTimeInMs; // 10 cycles
    int withinControlBoundsTimeReqmt = 50 * cycleTimeInMs; // 50 cycles
    int pkgIndex;

    //@todo: remove this once the getErrorCode function signature changes
    char func[50];

    if (vehicle->isM100() && vehicle->isLegacyM600())
    {
      // Telemetry: Verify the subscription
      ack = vehicle->subscribe->verify(responseTimeout);
      if (ACK::getError(ack) != ACK::SUCCESS)
      {
        ACK::getErrorCodeMessage(ack, func);
        return false;
      }

      // Telemetry: Subscribe to quaternion, fused lat/lon and altitude at freq 50
      // Hz
      pkgIndex                  = 0;
      int       freq            = 50;
      TopicName topicList50Hz[] = { TOPIC_QUATERNION, TOPIC_GPS_FUSED };
      int       numTopic = sizeof(topicList50Hz) / sizeof(topicList50Hz[0]);
      bool      enableTimestamp = false;

      bool pkgStatus = vehicle->subscribe->initPackageFromTopicList(
        pkgIndex, numTopic, topicList50Hz, enableTimestamp, freq);
      if (!(pkgStatus))
      {
        return pkgStatus;
      }
      ack =
        vehicle->subscribe->startPackage(pkgIndex, responseTimeout);
      if (ACK::getError(ack) != ACK::SUCCESS)
      {
        ACK::getErrorCodeMessage(ack, func);
        // Cleanup before return
        vehicle->subscribe->removePackage(pkgIndex, responseTimeout);
        return false;
      }

      // Also, since we don't have a source for relative height through subscription,
      // start using broadcast height
      if (!startGlobalPositionBroadcast())
      {
        // Cleanup before return
        vehicle->subscribe->removePackage(pkgIndex, responseTimeout);
        return false;
      }
    }

    // Wait for data to come in
    sleep(1);

    // Get data

    // Global position retrieved via subscription
    Telemetry::TypeMap<TOPIC_GPS_FUSED>::type currentSubscriptionGPS;
    Telemetry::TypeMap<TOPIC_GPS_FUSED>::type originSubscriptionGPS;
    // Global position retrieved via broadcast
    Telemetry::GlobalPosition currentBroadcastGP;
    Telemetry::GlobalPosition originBroadcastGP;

    // Convert position offset from first position to local coordinates
    Telemetry::Vector3f localOffset;

    if (vehicle->isM100() && vehicle->isLegacyM600())
    {
      currentSubscriptionGPS = vehicle->subscribe->getValue<TOPIC_GPS_FUSED>();
      originSubscriptionGPS  = currentSubscriptionGPS;
      localOffsetFromGpsOffset(localOffset,
                               static_cast<void*>(&currentSubscriptionGPS),
                               static_cast<void*>(&originSubscriptionGPS));

      // Get the broadcast GP since we need the height for zCmd
      currentBroadcastGP = vehicle->broadcast->getGlobalPosition();
    }
    else
    {
      currentBroadcastGP = vehicle->broadcast->getGlobalPosition();
      originBroadcastGP  = currentBroadcastGP;
      localOffsetFromGpsOffset(localOffset,
                               static_cast<void*>(&currentBroadcastGP),
                               static_cast<void*>(&originBroadcastGP));
    }

    // Get initial offset. We will update this in a loop later.
    double xOffsetRemaining = xOffsetDesired - localOffset.x;
    double yOffsetRemaining = yOffsetDesired - localOffset.y;
    double zOffsetRemaining = zOffsetDesired - localOffset.z;

    // Conversions
    double yawDesiredRad     = DEG2RAD * yawDesired;
    double yawThresholdInRad = DEG2RAD * yawThresholdInDeg;

    //! Get Euler angle

    // Quaternion retrieved via subscription
    Telemetry::TypeMap<TOPIC_QUATERNION>::type subscriptionQ;
    // Quaternion retrieved via broadcast
    Telemetry::Quaternion broadcastQ;

    double yawInRad;
    if (vehicle->isM100() && vehicle->isLegacyM600())
    {
      subscriptionQ = vehicle->subscribe->getValue<TOPIC_QUATERNION>();
      yawInRad = toEulerAngle((static_cast<void*>(&subscriptionQ))).z / DEG2RAD;
    }
    else
    {
      broadcastQ = vehicle->broadcast->getQuaternion();
      yawInRad   = toEulerAngle((static_cast<void*>(&broadcastQ))).z / DEG2RAD;
    }

    int   elapsedTimeInMs     = 0;
    int   withinBoundsCounter = 0;
    int   outOfBounds         = 0;
    int   brakeCounter        = 0;
    int   speedFactor         = 2;
    float xCmd, yCmd, zCmd;

    /*! Calculate the inputs to send the position controller. We implement basic
     *  receding setpoint position control and the setpoint is always 1 m away
     *  from the current position - until we get within a threshold of the goal.
     *  From that point on, we send the remaining distance as the setpoint.
     */
    if (xOffsetDesired > 0)
      xCmd = (xOffsetDesired < speedFactor) ? xOffsetDesired : speedFactor;
    else if (xOffsetDesired < 0)
      xCmd =
        (xOffsetDesired > -1 * speedFactor) ? xOffsetDesired : -1 * speedFactor;
    else
      xCmd = 0;

    if (yOffsetDesired > 0)
      yCmd = (yOffsetDesired < speedFactor) ? yOffsetDesired : speedFactor;
    else if (yOffsetDesired < 0)
      yCmd =
        (yOffsetDesired > -1 * speedFactor) ? yOffsetDesired : -1 * speedFactor;
    else
      yCmd = 0;

    if (vehicle->isM100() && vehicle->isLegacyM600())
    {
      zCmd = currentBroadcastGP.height + zOffsetDesired; //Since subscription cannot give us a relative height, use broadcast.
    }
    else
    {
      zCmd = currentBroadcastGP.height + zOffsetDesired;
    }

    //! Main closed-loop receding setpoint position control
    while (elapsedTimeInMs < timeoutInMilSec)
    {
      vehicle->control->positionAndYawCtrl(xCmd, yCmd, zCmd,
                                                              yawDesiredRad / DEG2RAD);

      usleep(cycleTimeInMs * 1000);
      elapsedTimeInMs += cycleTimeInMs;

      //! Get current position in required coordinates and units
      if (vehicle->isM100() && vehicle->isLegacyM600())
      {
        subscriptionQ = vehicle->subscribe->getValue<TOPIC_QUATERNION>();
        yawInRad      = toEulerAngle((static_cast<void*>(&subscriptionQ))).z;
        currentSubscriptionGPS = vehicle->subscribe->getValue<TOPIC_GPS_FUSED>();
        localOffsetFromGpsOffset(localOffset,
                                 static_cast<void*>(&currentSubscriptionGPS),
                                 static_cast<void*>(&originSubscriptionGPS));

        // Get the broadcast GP since we need the height for zCmd
        currentBroadcastGP = vehicle->broadcast->getGlobalPosition();
      }
      else
      {
        broadcastQ         = vehicle->broadcast->getQuaternion();
        yawInRad           = toEulerAngle((static_cast<void*>(&broadcastQ))).z;
        currentBroadcastGP = vehicle->broadcast->getGlobalPosition();
        localOffsetFromGpsOffset(localOffset,
                                 static_cast<void*>(&currentBroadcastGP),
                                 static_cast<void*>(&originBroadcastGP));
      }

      //! See how much farther we have to go
      xOffsetRemaining = xOffsetDesired - localOffset.x;
      yOffsetRemaining = yOffsetDesired - localOffset.y;
      zOffsetRemaining = zOffsetDesired - localOffset.z;

      //! See if we need to modify the setpoint
      if (std::abs(xOffsetRemaining) < speedFactor)
      {
        xCmd = xOffsetRemaining;
      }
      if (std::abs(yOffsetRemaining) < speedFactor)
      {
        yCmd = yOffsetRemaining;
      }

      if (vehicle->isM100() && std::abs(xOffsetRemaining) < posThresholdInM &&
          std::abs(yOffsetRemaining) < posThresholdInM &&
          std::abs(yawInRad - yawDesiredRad) < yawThresholdInRad)
      {
        //! 1. We are within bounds; start incrementing our in-bound counter
        withinBoundsCounter += cycleTimeInMs;
      }
      else if (std::abs(xOffsetRemaining) < posThresholdInM &&
               std::abs(yOffsetRemaining) < posThresholdInM &&
               std::abs(zOffsetRemaining) < posThresholdInM &&
               std::abs(yawInRad - yawDesiredRad) < yawThresholdInRad)
      {
        //! 1. We are within bounds; start incrementing our in-bound counter
        withinBoundsCounter += cycleTimeInMs;
      }
      else
      {
        if (withinBoundsCounter != 0)
        {
          //! 2. Start incrementing an out-of-bounds counter
          outOfBounds += cycleTimeInMs;
        }
      }
      //! 3. Reset withinBoundsCounter if necessary
      if (outOfBounds > outOfControlBoundsTimeLimit)
      {
        withinBoundsCounter = 0;
        outOfBounds         = 0;
      }
      //! 4. If within bounds, set flag and break
      if (withinBoundsCounter >= withinControlBoundsTimeReqmt)
      {
        break;
      }
    }

    //! Set velocity to zero, to prevent any residual velocity from position
    //! command
    if (vehicle->isM100() && vehicle->isLegacyM600())
    {
      while (brakeCounter < withinControlBoundsTimeReqmt)
      {
        vehicle->control->emergencyBrake();
        usleep(cycleTimeInMs * 10);
        brakeCounter += cycleTimeInMs;
      }
    }

    if (elapsedTimeInMs >= timeoutInMilSec)
    {
      std::cout << "Task timeout!\n";
      if (vehicle->isM100() && vehicle->isLegacyM600())
      {
        ack =
          vehicle->subscribe->removePackage(pkgIndex, responseTimeout);
        if (ACK::getError(ack))
        {
          std::cout << "Error unsubscribing; please restart the drone/FC to get "
                       "back to a clean state.\n";
        }
      }
      return ACK::FAIL;
    }

    if (vehicle->isM100() && vehicle->isLegacyM600())
    {
      ack =
        vehicle->subscribe->removePackage(pkgIndex, responseTimeout);
      if (ACK::getError(ack))
      {
        std::cout
          << "Error unsubscribing; please restart the drone/FC to get back "
             "to a clean state.\n";
      }
    }

    return ACK::SUCCESS;
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

  void VehicleWrapper::localOffsetFromGpsOffset(Telemetry::Vector3f& deltaNed, void* target, void* origin)
  {
    Telemetry::GPSFused*       subscriptionTarget;
    Telemetry::GPSFused*       subscriptionOrigin;
    Telemetry::GlobalPosition* broadcastTarget;
    Telemetry::GlobalPosition* broadcastOrigin;
    double                     deltaLon;
    double                     deltaLat;

    if (!vehicle->isM100() && !vehicle->isLegacyM600())
    {
      subscriptionTarget = (Telemetry::GPSFused*)target;
      subscriptionOrigin = (Telemetry::GPSFused*)origin;
      deltaLon   = subscriptionTarget->longitude - subscriptionOrigin->longitude;
      deltaLat   = subscriptionTarget->latitude - subscriptionOrigin->latitude;
      deltaNed.x = deltaLat * C_EARTH;
      deltaNed.y = deltaLon * C_EARTH * std::cos(subscriptionTarget->latitude);
      deltaNed.z = subscriptionTarget->altitude - subscriptionOrigin->altitude;
    }
    else
    {
      broadcastTarget = (Telemetry::GlobalPosition*)target;
      broadcastOrigin = (Telemetry::GlobalPosition*)origin;
      deltaLon        = broadcastTarget->longitude - broadcastOrigin->longitude;
      deltaLat        = broadcastTarget->latitude - broadcastOrigin->latitude;
      deltaNed.x      = deltaLat * C_EARTH;
      deltaNed.y      = deltaLon * C_EARTH * std::cos(broadcastTarget->latitude);
      deltaNed.z      = broadcastTarget->altitude - broadcastOrigin->altitude;
    }
  }

  Telemetry::Vector3f VehicleWrapper::toEulerAngle(void* quaternionData)
  {
    Telemetry::Vector3f    ans;
    Telemetry::Quaternion* quaternion = (Telemetry::Quaternion*)quaternionData;

    double q2sqr = quaternion->q2 * quaternion->q2;
    double t0    = -2.0 * (q2sqr + quaternion->q3 * quaternion->q3) + 1.0;
    double t1 =
      +2.0 * (quaternion->q1 * quaternion->q2 + quaternion->q0 * quaternion->q3);
    double t2 =
      -2.0 * (quaternion->q1 * quaternion->q3 - quaternion->q0 * quaternion->q2);
    double t3 =
      +2.0 * (quaternion->q2 * quaternion->q3 + quaternion->q0 * quaternion->q1);
    double t4 = -2.0 * (quaternion->q1 * quaternion->q1 + q2sqr) + 1.0;

    t2 = (t2 > 1.0) ? 1.0 : t2;
    t2 = (t2 < -1.0) ? -1.0 : t2;

    ans.x = asin(t2);
    ans.y = atan2(t3, t4);
    ans.z = atan2(t1, t0);

    return ans;
  }

  bool VehicleWrapper::getCurrentGimbal(RotationAngle& initial_angle)
  {
    // Get Gimbal initial values
    if (!vehicle->isM100() && !vehicle->isLegacyM600())
    {
      initial_angle.roll  = vehicle->subscribe->getValue<TOPIC_GIMBAL_ANGLES>().y;
      initial_angle.pitch = vehicle->subscribe->getValue<TOPIC_GIMBAL_ANGLES>().x;
      initial_angle.yaw   = vehicle->subscribe->getValue<TOPIC_GIMBAL_ANGLES>().z;
    }
    else
    {
      initial_angle.roll  = vehicle->broadcast->getGimbal().roll;
      initial_angle.pitch = vehicle->broadcast->getGimbal().pitch;
      initial_angle.yaw   = vehicle->broadcast->getGimbal().yaw;
    }
    return true;
  }

  bool VehicleWrapper::setGimbalAngle(const GimbalContainer& gimbal)
  {
    DJI::OSDK::Gimbal::AngleData gimbalAngle = {};
    gimbalAngle.roll     = gimbal.roll;
    gimbalAngle.pitch    = gimbal.pitch;
    gimbalAngle.yaw      = gimbal.yaw;
    gimbalAngle.duration = gimbal.duration;
    gimbalAngle.mode |= 0;
    gimbalAngle.mode |= gimbal.isAbsolute;
    gimbalAngle.mode |= gimbal.yaw_cmd_ignore << 1;
    gimbalAngle.mode |= gimbal.roll_cmd_ignore << 2;
    gimbalAngle.mode |= gimbal.pitch_cmd_ignore << 3;

    vehicle->gimbal->setAngle(&gimbalAngle);
    // Give time for gimbal to sync
    sleep(4);
    return true;
  }
}
