#ifndef __DJI_SDK_HH__
#define __DJI_SDK_HH__
/**-----------------------------------------------
  * @Copyright (C) 2020 All rights reserved.
  * @date: 2020
  * @file: dji_sdk.hh
  * @version: v0.0.1
  * @author: kevin.hoo@gmail.com
  * @create_date: 2020-02-27 11:08:35
  * @last_modified_date: 2020-02-27 11:57:39
  * @brief: TODO
  * @details: TODO
  *-----------------------------------------------*/

// Header include
#include <dji_control.hpp>
#include <dji_status.hpp>
#include <dji_version.hpp>

// Declaration
namespace DJISDK
{

enum class DroneFirmwareVersion
{
  M100_31 = DJI::OSDK::Version::M100_31
};

enum class AircraftVersion
{
  UNKNOWN,
  M100,
  M600,
  A3,
  N3,
  M210
};

enum class FlightControlFlag
{
    HORIZONTAL_ANGLE         = DJI::OSDK::Control::HORIZONTAL_ANGLE,
    HORIZONTAL_VELOCITY      = DJI::OSDK::Control::HORIZONTAL_VELOCITY,
    HORIZONTAL_POSITION      = DJI::OSDK::Control::HORIZONTAL_POSITION,
    // Horizontal angular rate is supported only by A3/N3 based platform
    // and is NOT supported by M100
    HORIZONTAL_ANGULAR_RATE  = DJI::OSDK::Control::HORIZONTAL_ANGULAR_RATE,

    VERTICAL_VELOCITY = DJI::OSDK::Control::VERTICAL_VELOCITY,
    VERTICAL_POSITION = DJI::OSDK::Control::VERTICAL_POSITION,
    VERTICAL_THRUST   = DJI::OSDK::Control::VERTICAL_THRUST,

    YAW_ANGLE = DJI::OSDK::Control::YAW_ANGLE,
    YAW_RATE  = DJI::OSDK::Control::YAW_RATE,

    HORIZONTAL_GROUND = DJI::OSDK::Control::HORIZONTAL_GROUND,
    HORIZONTAL_BODY   = DJI::OSDK::Control::HORIZONTAL_BODY,

    STABLE_DISABLE = DJI::OSDK::Control::STABLE_DISABLE,
    STABLE_ENABLE  = DJI::OSDK::Control::STABLE_ENABLE
};

/*!
 * Refer to demo_flight_control.cpp in dji_sdk_demo for how to
 * use the display mode.
 */
enum class DisplayMode
{
    /*! This mode requires the user to manually
     * control the aircraft to remain stable in air. */
    MODE_MANUAL_CTRL=DJI::OSDK::VehicleStatus::DisplayMode::MODE_MANUAL_CTRL,
    /*! In this mode, the aircraft can keep
     * attitude stabilization and only use the
     * barometer for positioning to control the altitude. <br>
     * The aircraft can not autonomously locate and hover stably.*/
    MODE_ATTITUDE=DJI::OSDK::VehicleStatus::DisplayMode::MODE_ATTITUDE,

    /*! The aircraft is in normal GPS mode. <br>
     * In normal GPS mode, the aircraft can
     * autonomously locate and hover stably. <br>
     *  The sensitivity of the aircraft to the
     *  command response is moderate.
     */
    MODE_P_GPS=DJI::OSDK::VehicleStatus::DisplayMode::MODE_P_GPS,
    /*! In hotpoint mode */
    MODE_HOTPOINT_MODE=DJI::OSDK::VehicleStatus::DisplayMode::MODE_HOTPOINT_MODE,
    /*! In this mode, user can push the throttle
     * stick to complete stable take-off. */
    MODE_ASSISTED_TAKEOFF=DJI::OSDK::VehicleStatus::DisplayMode::MODE_ASSISTED_TAKEOFF,
    /*! In this mode, the aircraft will autonomously
     * start motor, ascend and finally hover. */
    MODE_AUTO_TAKEOFF=DJI::OSDK::VehicleStatus::DisplayMode::MODE_AUTO_TAKEOFF,
    /*! In this mode, the aircraft can land autonomously. */
    MODE_AUTO_LANDING=DJI::OSDK::VehicleStatus::DisplayMode::MODE_AUTO_LANDING,
    /*! In this mode, the aircraft can antonomously return the
     * last recorded Home Point. <br>
     * There are three types of this mode: Smart RTH(Return-to-Home),
     * Low Batterry RTH, and Failsafe RTTH.  */
    MODE_NAVI_GO_HOME=DJI::OSDK::VehicleStatus::DisplayMode::MODE_NAVI_GO_HOME,
    /*! In this mode, the aircraft is controled by SDK API. <br>
     * User can directly define the control mode of horizon
     * and vertical directions and send control datas to aircraft. */
    MODE_NAVI_SDK_CTRL=DJI::OSDK::VehicleStatus::DisplayMode::MODE_NAVI_SDK_CTRL,

    /*! drone is forced to land, might due to low battery */
    MODE_FORCE_AUTO_LANDING=DJI::OSDK::VehicleStatus::DisplayMode::MODE_FORCE_AUTO_LANDING,
    /*! drone will search for the last position where the rc is not lost */
    MODE_SEARCH_MODE =DJI::OSDK::VehicleStatus::DisplayMode::MODE_SEARCH_MODE,
    /*! Mode for motor starting. <br>
     * Every time user unlock the motor, this will be the first mode. */
    MODE_ENGINE_START = DJI::OSDK::VehicleStatus::DisplayMode::MODE_ENGINE_START
};

/*!
 * Note that the flight status for M100 and A3/N3 are different.
 *
 * Refer to demo_flight_control.cpp in dji_sdk_demo for how to
 * use the flight status.
 *
 */
enum class FlightStatus
{
    STATUS_STOPPED   = DJI::OSDK::VehicleStatus::FlightStatus::STOPED,
    STATUS_ON_GROUND = DJI::OSDK::VehicleStatus::FlightStatus::ON_GROUND,
    STATUS_IN_AIR    = DJI::OSDK::VehicleStatus::FlightStatus::IN_AIR
};

enum class M100FlightStatus
{
    M100_STATUS_ON_GROUND        = DJI::OSDK::VehicleStatus::M100FlightStatus::ON_GROUND_STANDBY,
    M100_STATUS_TAKINGOFF        = DJI::OSDK::VehicleStatus::M100FlightStatus::TAKEOFF,
    M100_STATUS_IN_AIR           = DJI::OSDK::VehicleStatus::M100FlightStatus::IN_AIR_STANDBY,
    M100_STATUS_LANDING          = DJI::OSDK::VehicleStatus::M100FlightStatus::LANDING,
    M100_STATUS_FINISHED_LANDING = DJI::OSDK::VehicleStatus::M100FlightStatus::FINISHING_LANDING
};

}

#endif // __DJI_SDK_HH__

