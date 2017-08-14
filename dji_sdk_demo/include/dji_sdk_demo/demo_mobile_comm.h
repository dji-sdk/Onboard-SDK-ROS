/** @file demo_mobile_comm.h
 *  @version 3.3
 *  @date May, 2017
 *
 *  @brief
 *  demo sample of how to use mobile communication APIs
 *
 *  @copyright 2017 DJI. All rights reserved.
 *
 */

#ifndef DEMO_MOBILE_COMM_H
#define DEMO_MOBILE_COMM_H

// msgs
#include <dji_sdk/MobileData.h>
#include <sensor_msgs/Joy.h>
// DJI SDK includes
#include <dji_sdk/Activation.h>
#include <dji_sdk/DroneTaskControl.h>
#include <dji_sdk/SDKControlAuthority.h>
#include <dji_sdk/MissionWpAction.h>
#include <dji_sdk/MissionHpAction.h>
#include <dji_sdk/MissionWpUpload.h>
#include <dji_sdk/MissionHpUpload.h>
#include <dji_sdk/MissionHpUpdateRadius.h>
#include <dji_sdk/MissionHpUpdateYawRate.h>
#include <dji_sdk/DroneArmControl.h>
#include <dji_sdk/DroneTaskControl.h>
#include <dji_sdk/SendMobileData.h>
#include <dji_sdk/QueryDroneVersion.h>

// ROS includes
#include <ros/ros.h>
#include <sensor_msgs/NavSatFix.h>
#include <geometry_msgs/QuaternionStamped.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <std_msgs/UInt8.h>
#include <tf/tf.h>
// SDK library
#include <djiosdk/dji_vehicle.hpp>

#define C_EARTH (double)6378137.0
#define C_PI (double)3.141592653589793
#define DEG2RAD(DEG) ((DEG) * ((C_PI) / (180.0)))

#pragma pack(1)
typedef struct AckReturnToMobile{
  uint16_t cmdID;
  uint16_t ack;
} AckReturnToMobile;
#pragma pack()

typedef struct ServiceAck{
  bool result;
  int cmd_set;
  int cmd_id;
  unsigned int ack_data;
  ServiceAck(bool res, int set, int id, unsigned int ack):
      result(res), cmd_set(set), cmd_id(id), ack_data(ack) {};
  ServiceAck() {};
}ServiceAck;

typedef float  float32_t;
typedef double float64_t;

class Mission
{
public:
  // The basic state transition flow is:
  // 0---> 1 ---> 2 ---> ... ---> N ---> 0
  // where state 0 means the mission is note started
  // and each state i is for the process of moving to a target point.
  int state;
  int inbound_counter;
  int outbound_counter;
  int break_counter;

  float target_offset_x;
  float target_offset_y;
  float target_offset_z;
  float target_yaw;
  sensor_msgs::NavSatFix start_gps_location;

  bool finished;

  Mission() : state(0), inbound_counter(0), outbound_counter(0), break_counter(0),
              target_offset_x(0.0), target_offset_y(0.0), target_offset_z(0.0),
              finished(false)
  {
  }

  void step();

  void setTarget(float x, float y, float z, float yaw)
  {
    target_offset_x = x;
    target_offset_y = y;
    target_offset_z = z;
    target_yaw      = yaw;
  }

  void reset()
  {
    inbound_counter = 0;
    outbound_counter = 0;
    break_counter = 0;
    finished = false;
  }

};

static void Display_Main_Menu(void);

bool sendToMobile(AckReturnToMobile returnAckMobile);

void fromMobileDataSubscriberCallback(const dji_sdk::MobileData::ConstPtr& from_mobile_data);

ServiceAck armMotors();

ServiceAck disArmMotors();

ServiceAck obtainCtrlAuthority();

ServiceAck releaseCtrlAuthority();

ServiceAck takeoff();

ServiceAck goHome();

ServiceAck land();

ServiceAck activate();

ServiceAck hotpointUpdateRadius(float radius);

ServiceAck hotpointUpdateYawRate(float yawRate, int direction);

ServiceAck initWaypointMission(dji_sdk::MissionWaypointTask &waypointTask);

ServiceAck initHotpointMission(dji_sdk::MissionHotpointTask &hotpointTask);

ServiceAck missionAction(DJI::OSDK::DJI_MISSION_TYPE type,
                         DJI::OSDK::MISSION_ACTION action);

void gpsPosCallback(const sensor_msgs::NavSatFix::ConstPtr& msg);

void localOffsetFromGpsOffset(geometry_msgs::Vector3&  deltaNed,
                         sensor_msgs::NavSatFix& target,
                         sensor_msgs::NavSatFix& origin);

geometry_msgs::Vector3 toEulerAngle(geometry_msgs::Quaternion quat);

void display_mode_callback(const std_msgs::UInt8::ConstPtr& msg);

void flight_status_callback(const std_msgs::UInt8::ConstPtr& msg);

void gps_callback(const sensor_msgs::NavSatFix::ConstPtr& msg);

void attitude_callback(const geometry_msgs::QuaternionStamped::ConstPtr& msg);

bool takeoff_land(int task);

bool obtain_control();

bool is_M100();
bool monitoredTakeoff();
bool M100monitoredTakeoff();

bool runWaypointMission(uint8_t numWaypoints,
                        int responseTimeout);

void setWaypointDefaults(DJI::OSDK::WayPointSettings* wp);

void setWaypointInitDefaults(dji_sdk::MissionWaypointTask &waypointTask);

std::vector<DJI::OSDK::WayPointSettings> createWaypoints(int numWaypoints,
                                                         DJI::OSDK::float64_t distanceIncrement, DJI::OSDK::float32_t start_alt);

std::vector<DJI::OSDK::WayPointSettings> generateWaypointsPolygon(
  DJI::OSDK::WayPointSettings* start_data,
  DJI::OSDK::float64_t increment,
  int num_wp);

void uploadWaypoints(std::vector<DJI::OSDK::WayPointSettings>& wp_list,
                     int responseTimeout,
                     dji_sdk::MissionWaypointTask &waypointTask);

bool runHotpointMission(int initialRadius,
                        int responseTimeout);

void setHotpointInitDefault(dji_sdk::MissionHotpointTask &hotpointTask);



#endif //DEMO_MOBILE_COMM_H
