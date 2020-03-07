/** @file flight_controller.hh
 *  @version 4.0
 *  @date Mar, 2020
 *
 *  @brief
 *  FlightControllerROS:
 *
 *  @copyright 2020 DJI. All rights reserved.
 *
 */
#ifndef ONBOARD_SDK_ROS_FLIGHT_CONTROLLER_HH
#define ONBOARD_SDK_ROS_FLIGHT_CONTROLLER_HH

#include <ros/ros.h>
#include <tf/tf.h>

#include <geometry_msgs/QuaternionStamped.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <sensor_msgs/NavSatFix.h>
#include <std_msgs/UInt8.h>
#include <mutex>
#include <thread>

//
//class Mission
//{
//public:
//  // The basic state transition flow is:
//  // 0---> 1 ---> 2 ---> ... ---> N ---> 0
//  // where state 0 means the mission is note started
//  // and each state i is for the process of moving to a target point.
//  int state;
//
//  int inbound_counter;
//  int outbound_counter;
//  int break_counter;
//
//  float target_offset_x;
//  float target_offset_y;
//  float target_offset_z;
//  float target_yaw;
//  sensor_msgs::NavSatFix start_gps_location;
//  geometry_msgs::Point start_local_position;
//
//  bool finished;
//
//  Mission()
//    : state(0),
//      inbound_counter(0),
//      outbound_counter(0),
//      break_counter(0),
//      target_offset_x(0.0),
//      target_offset_y(0.0),
//      target_offset_z(0.0),
//      finished(false)
//  {
//  }
//  void localOffsetFromGpsOffset(geometry_msgs::Vector3&  deltaNed,
//                           sensor_msgs::NavSatFix& target,
//                           sensor_msgs::NavSatFix& origin);
//
//  void step(sensor_msgs::NavSatFix current_gps, geometry_msgs::Quaternion current_atti);
//
//  void setTarget(float x, float y, float z, float yaw)
//  {
//    target_offset_x = x;
//    target_offset_y = y;
//    target_offset_z = z;
//    target_yaw      = yaw;
//  }
//
//  void reset()
//  {
//    inbound_counter = 0;
//    outbound_counter = 0;
//    break_counter = 0;
//    finished = false;
//  }
//
//};

class FlightControllerROS
{
public:
  FlightControllerROS();

  void init();
  bool takeoff();
  bool land();
  void deinit(){ callback_thread_.join();}

protected:
  bool gainControl();
  bool enableLocalPosition();
  bool callTakeoffTask(int task);
  uint32_t getVersion();

  void listenCallback();

  void attitudeCallback(const geometry_msgs::QuaternionStamped::ConstPtr& msg);
  void gpsCallback(const sensor_msgs::NavSatFix::ConstPtr& msg);
  void flightStatusCallback(const std_msgs::UInt8::ConstPtr& msg);
  void displayModeCallback(const std_msgs::UInt8::ConstPtr& msg);
  void localPositionCallback(const geometry_msgs::PointStamped::ConstPtr& msg);



private:
  ros::NodeHandle node_handler_;

  ros::Subscriber attitude_suber_;
  ros::Subscriber gps_pos_suber_;
  ros::Subscriber flight_status_suber_;
  ros::Subscriber display_mode_suber_;
  ros::Subscriber local_pos_suber_;

  ros::Publisher yaw_control_puber_;
  ros::Publisher break_control_puber_;

  ros::ServiceClient query_version_client_;
  ros::ServiceClient dispatch_task_client_;
  ros::ServiceClient ctrl_authority_client_;
  ros::ServiceClient enable_local_pos_client_;

  std::thread callback_thread_;
//  Mission square_mission_;

  geometry_msgs::Quaternion attitude_;
  geometry_msgs::Point latest_local_pos_;
  sensor_msgs::NavSatFix latest_gps_pos_;
  uint32_t drone_version_;
  uint8_t flight_status_;
  uint8_t display_mode_;

  std::mutex attitude_mutex_;
  std::mutex version_mutex_;
  std::mutex fs_mutex_;
  std::mutex dm_mutex_;
  std::mutex llp_mutex_;
  std::mutex gps_mutex_;
};



#endif //ONBOARD_SDK_ROS_FLIGHT_CONTROLLER_HH
