/** @file dji_sdk_node_publisher.cpp
 *  @version 3.7
 *  @date July, 2018
 *
 *  @brief
 *  Implementation of the publishers of DJISDKNode
 *
 *  @copyright 2018 DJI. All rights reserved.
 *
 */

#include <dji_sdk/dji_sdk_node.h>
#include <tf/tf.h>
#include <sensor_msgs/Joy.h>
#include <dji_telemetry.hpp>

#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>


#define _TICK2ROSTIME(tick) (ros::Duration((double)(tick) / 1000.0))


void
DJISDKNode::SDKBroadcastCallback(Vehicle* vehicle, RecvContainer recvFrame,
                                 DJI::OSDK::UserData userData)
{
  ((DJISDKNode*)userData)->dataBroadcastCallback();
}

void
DJISDKNode::dataBroadcastCallback()
{
  using namespace DJI::OSDK;

  ros::Time now_time = ros::Time::now();

  uint16_t data_enable_flag = vehicle->broadcast->getPassFlag();

  uint16_t flag_has_rc = isM100() ? (data_enable_flag & DataBroadcast::DATA_ENABLE_FLAG::HAS_RC) :
                         (data_enable_flag & DataBroadcast::DATA_ENABLE_FLAG::A3_HAS_RC);
  if (flag_has_rc)
  {
    sensor_msgs::Joy rc_joy;
    rc_joy.header.stamp    = now_time;
    rc_joy.header.frame_id = "rc";

    rc_joy.axes.reserve(6);
    rc_joy.axes.push_back(static_cast<float>(vehicle->broadcast->getRC().roll     / 10000.0));
    rc_joy.axes.push_back(static_cast<float>(vehicle->broadcast->getRC().pitch    / 10000.0));
    rc_joy.axes.push_back(static_cast<float>(vehicle->broadcast->getRC().yaw      / 10000.0));
    rc_joy.axes.push_back(static_cast<float>(vehicle->broadcast->getRC().throttle / 10000.0));

    rc_joy.axes.push_back(static_cast<float>(vehicle->broadcast->getRC().mode));
    rc_joy.axes.push_back(static_cast<float>(vehicle->broadcast->getRC().gear));
    rc_publisher.publish(rc_joy);
  }

  tf::Matrix3x3 R_FRD2NED;
  tf::Quaternion q_FLU2ENU;

  if (data_enable_flag & DataBroadcast::DATA_ENABLE_FLAG::HAS_Q)
  {
    R_FRD2NED.setRotation(tf::Quaternion(vehicle->broadcast->getQuaternion().q1,
                                         vehicle->broadcast->getQuaternion().q2,
                                         vehicle->broadcast->getQuaternion().q3,
                                         vehicle->broadcast->getQuaternion().q0));
    tf::Matrix3x3 R_FLU2ENU = R_ENU2NED.transpose() * R_FRD2NED * R_FLU2FRD;
    R_FLU2ENU.getRotation(q_FLU2ENU);

    geometry_msgs::QuaternionStamped q;
    q.header.stamp = now_time;
    q.header.frame_id = "body_FLU";

    q.quaternion.w = q_FLU2ENU.getW();
    q.quaternion.x = q_FLU2ENU.getX();
    q.quaternion.y = q_FLU2ENU.getY();
    q.quaternion.z = q_FLU2ENU.getZ();

    attitude_publisher.publish(q);
  }

  if ( (data_enable_flag & DataBroadcast::DATA_ENABLE_FLAG::HAS_Q) &&
       (data_enable_flag & DataBroadcast::DATA_ENABLE_FLAG::HAS_W) &&
       (data_enable_flag & DataBroadcast::DATA_ENABLE_FLAG::HAS_A))
  {
    sensor_msgs::Imu imu;

    imu.header.frame_id = "body_FLU";
    imu.header.stamp    = now_time;

    imu.linear_acceleration.x =  vehicle->broadcast->getAcceleration().x * gravity_const;
    imu.linear_acceleration.y = -vehicle->broadcast->getAcceleration().y * gravity_const;
    imu.linear_acceleration.z = -vehicle->broadcast->getAcceleration().z * gravity_const;

    imu.angular_velocity.x    =  vehicle->broadcast->getAngularRate().x;
    imu.angular_velocity.y    = -vehicle->broadcast->getAngularRate().y;
    imu.angular_velocity.z    = -vehicle->broadcast->getAngularRate().z;

    // Since the orientation is duplicated from attitude
    // at this point, q_FLU2ENU has already been updated
    imu.orientation.w = q_FLU2ENU.getW();
    imu.orientation.x = q_FLU2ENU.getX();
    imu.orientation.y = q_FLU2ENU.getY();
    imu.orientation.z = q_FLU2ENU.getZ();

    imu_publisher.publish(imu);
  }

  if (data_enable_flag & DataBroadcast::DATA_ENABLE_FLAG::HAS_POS)
  {
    DJI::OSDK::Telemetry::GlobalPosition global_pos =
      vehicle->broadcast->getGlobalPosition();
    std_msgs::UInt8 gps_health;
    gps_health.data = global_pos.health;
    gps_health_publisher.publish(gps_health);

    sensor_msgs::NavSatFix gps_pos;
    gps_pos.header.stamp    = now_time;
    gps_pos.header.frame_id = "gps";
    gps_pos.latitude        = global_pos.latitude * 180 / C_PI;
    gps_pos.longitude       = global_pos.longitude * 180 / C_PI;
    gps_pos.altitude        = global_pos.altitude;
    this->current_gps_latitude = gps_pos.latitude;
    this->current_gps_longitude = gps_pos.longitude;
    this->current_gps_altitude = gps_pos.altitude;
    this->current_gps_health = global_pos.health;
    gps_position_publisher.publish(gps_pos);

    if(local_pos_ref_set)
    {
      geometry_msgs::PointStamped local_pos;
      local_pos.header.frame_id = "/local";
      local_pos.header.stamp = now_time;
      gpsConvertENU(local_pos.point.x, local_pos.point.y, gps_pos.longitude,
                       gps_pos.latitude, this->local_pos_ref_longitude, this->local_pos_ref_latitude);
      local_pos.point.z = gps_pos.altitude - this->local_pos_ref_altitude;
      /*!
      * note: We are now following REP 103 to use ENU for
      *       short-range Cartesian representations. Local position is published
      *       in ENU Frame
      */

      this->local_position_publisher.publish(local_pos);
    }

    std_msgs::Float32 agl_height;
    agl_height.data = global_pos.height;
    height_publisher.publish(agl_height);
  }

  if (data_enable_flag & DataBroadcast::DATA_ENABLE_FLAG::HAS_V)
  {
    geometry_msgs::Vector3Stamped velocity;
    velocity.header.stamp    = now_time;
    velocity.header.frame_id = "ground_ENU";

    velocity.vector.x = vehicle->broadcast->getVelocity().y;
    velocity.vector.y = vehicle->broadcast->getVelocity().x;
    velocity.vector.z = vehicle->broadcast->getVelocity().z;
    velocity_publisher.publish(velocity);
  }

  uint16_t flag_has_battery =
          isM100() ? (data_enable_flag & DataBroadcast::DATA_ENABLE_FLAG::HAS_BATTERY) :
                     (data_enable_flag & DataBroadcast::DATA_ENABLE_FLAG::A3_HAS_BATTERY);

  if ( flag_has_battery )
  {
    sensor_msgs::BatteryState msg_battery_state;
    msg_battery_state.header.stamp = now_time;
    msg_battery_state.capacity = vehicle->broadcast->getBatteryInfo().capacity;
    msg_battery_state.voltage  = vehicle->broadcast->getBatteryInfo().voltage;
    msg_battery_state.current  = vehicle->broadcast->getBatteryInfo().current;
    msg_battery_state.percentage = vehicle->broadcast->getBatteryInfo().percentage;
    msg_battery_state.charge   = NAN;
    msg_battery_state.design_capacity = NAN;
    msg_battery_state.power_supply_health = msg_battery_state.POWER_SUPPLY_HEALTH_UNKNOWN;
    msg_battery_state.power_supply_status = msg_battery_state.POWER_SUPPLY_STATUS_UNKNOWN;
    msg_battery_state.power_supply_technology = msg_battery_state.POWER_SUPPLY_TECHNOLOGY_UNKNOWN;
    msg_battery_state.present = (vehicle->broadcast->getBatteryInfo().voltage!=0);
    battery_state_publisher.publish(msg_battery_state);
  }

  uint16_t flag_has_status =
          isM100() ? (data_enable_flag & DataBroadcast::DATA_ENABLE_FLAG::HAS_STATUS) :
          (data_enable_flag & DataBroadcast::DATA_ENABLE_FLAG::A3_HAS_STATUS);

  if ( flag_has_status)
  {
    Telemetry::TypeMap<Telemetry::TOPIC_STATUS_FLIGHT>::type fs =
      vehicle->broadcast->getStatus().flight;

    std_msgs::UInt8 flight_status;
    flight_status.data = fs;
    flight_status_publisher.publish(flight_status);
  }

  uint16_t flag_has_gimbal = 
          isM100() ? (data_enable_flag & DataBroadcast::DATA_ENABLE_FLAG::HAS_GIMBAL) :
          (data_enable_flag & DataBroadcast::DATA_ENABLE_FLAG::A3_HAS_GIMBAL);
  if (flag_has_gimbal)
  {
    Telemetry::Gimbal gimbal_reading;

    
    Telemetry::Gimbal gimbal_angle = vehicle->broadcast->getGimbal();

    geometry_msgs::Vector3Stamped gimbal_angle_vec3;

    gimbal_angle_vec3.header.stamp = now_time;
    gimbal_angle_vec3.header.frame_id = "ground_ENU";
    gimbal_angle_vec3.vector.x     = gimbal_angle.roll;
    gimbal_angle_vec3.vector.y     = gimbal_angle.pitch;
    gimbal_angle_vec3.vector.z     = gimbal_angle.yaw;
    gimbal_angle_publisher.publish(gimbal_angle_vec3);
  }
}

void
DJISDKNode::publish5HzData(Vehicle *vehicle, RecvContainer recvFrame,
                            DJI::OSDK::UserData userData)
{
  DJISDKNode *p = (DJISDKNode *)userData;

  uint8_t* data = recvFrame.recvData.raw_ack_array;
  ROS_ASSERT(DJISDKNode::PACKAGE_ID_5HZ == *data );

  data++;
  Telemetry::TimeStamp packageTimeStamp = * (reinterpret_cast<Telemetry::TimeStamp *>(data));

  ros::Time msg_time = ros::Time::now();

  if(p->align_time_with_FC)
  {
    if(p->curr_align_state == ALIGNED)
    {
      msg_time = p->base_time + _TICK2ROSTIME(packageTimeStamp.time_ms);
    }
    else
    {
      return;
    }
  }

  //TODO: publish gps detail data if needed
  Telemetry::TypeMap<Telemetry::TOPIC_BATTERY_INFO>::type battery_info=
    vehicle->subscribe->getValue<Telemetry::TOPIC_BATTERY_INFO>();
  sensor_msgs::BatteryState msg_battery_state;
  msg_battery_state.header.stamp = msg_time;
  msg_battery_state.capacity = battery_info.capacity;
  msg_battery_state.voltage  = battery_info.voltage;
  msg_battery_state.current  = battery_info.current;
  msg_battery_state.percentage = battery_info.percentage;
  msg_battery_state.charge   = NAN;
  msg_battery_state.design_capacity = NAN;
  msg_battery_state.power_supply_health = msg_battery_state.POWER_SUPPLY_HEALTH_UNKNOWN;
  msg_battery_state.power_supply_status = msg_battery_state.POWER_SUPPLY_STATUS_UNKNOWN;
  msg_battery_state.power_supply_technology = msg_battery_state.POWER_SUPPLY_TECHNOLOGY_UNKNOWN;
  msg_battery_state.present = (battery_info.voltage!=0);
  p->battery_state_publisher.publish(msg_battery_state);

  if(p->rtkSupport)
  {
    Telemetry::TypeMap<Telemetry::TOPIC_RTK_POSITION>::type rtk_telemetry_position=
        vehicle->subscribe->getValue<Telemetry::TOPIC_RTK_POSITION>();

    Telemetry::TypeMap<Telemetry::TOPIC_RTK_VELOCITY>::type rtk_telemetry_velocity=
        vehicle->subscribe->getValue<Telemetry::TOPIC_RTK_VELOCITY>();

    Telemetry::TypeMap<Telemetry::TOPIC_RTK_YAW>::type rtk_telemetry_yaw=
        vehicle->subscribe->getValue<Telemetry::TOPIC_RTK_YAW>();

    Telemetry::TypeMap<Telemetry::TOPIC_RTK_YAW_INFO>::type rtk_telemetry_yaw_info=
        vehicle->subscribe->getValue<Telemetry::TOPIC_RTK_YAW_INFO>();

    Telemetry::TypeMap<Telemetry::TOPIC_RTK_POSITION_INFO>::type rtk_telemetry_position_info=
        vehicle->subscribe->getValue<Telemetry::TOPIC_RTK_POSITION_INFO>();

    Telemetry::TypeMap<Telemetry::TOPIC_RTK_CONNECT_STATUS>::type rtk_telemetry_connect_status=
            vehicle->subscribe->getValue<Telemetry::TOPIC_RTK_CONNECT_STATUS>();

    sensor_msgs::NavSatFix rtk_position;
    rtk_position.header.stamp = msg_time;
    rtk_position.latitude = rtk_telemetry_position.latitude;
    rtk_position.longitude = rtk_telemetry_position.longitude;
    rtk_position.altitude = rtk_telemetry_position.HFSL;
    p->rtk_position_publisher.publish(rtk_position);

    //! Velocity converted to m/s to conform to REP103.
    geometry_msgs::Vector3Stamped rtk_velocity;
    rtk_velocity.header.stamp = msg_time;
    rtk_velocity.vector.x = (rtk_telemetry_velocity.x)/100;
    rtk_velocity.vector.y = (rtk_telemetry_velocity.y)/100;
    rtk_velocity.vector.z = (rtk_telemetry_velocity.z)/100;
    p->rtk_velocity_publisher.publish(rtk_velocity);

    std_msgs::Int16 rtk_yaw;
    rtk_yaw.data = rtk_telemetry_yaw;
    p->rtk_yaw_publisher.publish(rtk_yaw);

    std_msgs::UInt8 rtk_yaw_info;
    rtk_yaw_info.data = (int)rtk_telemetry_yaw_info;
    p->rtk_yaw_info_publisher.publish(rtk_yaw_info);

    std_msgs::UInt8 rtk_position_info;
    rtk_position_info.data = (int)rtk_telemetry_position_info;
    p->rtk_position_info_publisher.publish(rtk_position_info);

    std_msgs::UInt8 rtk_connection_status;
    rtk_connection_status.data = (rtk_telemetry_connect_status.rtkConnected == 1) ? 1 : 0;
    p->rtk_connection_status_publisher.publish(rtk_connection_status);
  }

  return;
}

void
DJISDKNode::publish50HzData(Vehicle* vehicle, RecvContainer recvFrame,
                            DJI::OSDK::UserData userData)
{
  DJISDKNode* p = (DJISDKNode*)userData;

  uint8_t* data = recvFrame.recvData.raw_ack_array;
  ROS_ASSERT(DJISDKNode::PACKAGE_ID_50HZ == *data );
  data++;
  Telemetry::TimeStamp packageTimeStamp = * (reinterpret_cast<Telemetry::TimeStamp *>(data));

  ros::Time msg_time = ros::Time::now();

  if(p->align_time_with_FC)
  {
    if(p->curr_align_state == ALIGNED)
    {
      msg_time = p->base_time + _TICK2ROSTIME(packageTimeStamp.time_ms);
    }
    else
    {
      return;
    }
  }

  Telemetry::TypeMap<Telemetry::TOPIC_GPS_FUSED>::type fused_gps =
    vehicle->subscribe->getValue<Telemetry::TOPIC_GPS_FUSED>();
  Telemetry::TypeMap<Telemetry::TOPIC_ALTITUDE_FUSIONED>::type fused_altitude =
    vehicle->subscribe->getValue<Telemetry::TOPIC_ALTITUDE_FUSIONED>();


  sensor_msgs::NavSatFix gps_pos;
  gps_pos.header.frame_id = "/gps";
  gps_pos.header.stamp    = msg_time;
  gps_pos.latitude        = fused_gps.latitude * 180.0 / C_PI;   //degree
  gps_pos.longitude       = fused_gps.longitude * 180.0 / C_PI;  //degree
  gps_pos.altitude        = fused_altitude;                      //meter
  p->current_gps_latitude = gps_pos.latitude;
  p->current_gps_longitude = gps_pos.longitude;
  p->current_gps_altitude = fused_altitude;
  p->gps_position_publisher.publish(gps_pos);

  if(p->local_pos_ref_set)
  {
    geometry_msgs::PointStamped local_pos;
    local_pos.header.frame_id = "/local";
    local_pos.header.stamp = gps_pos.header.stamp;
    p->gpsConvertENU(local_pos.point.x, local_pos.point.y, gps_pos.longitude,
        gps_pos.latitude, p->local_pos_ref_longitude, p->local_pos_ref_latitude);
    local_pos.point.z = gps_pos.altitude - p->local_pos_ref_altitude;
   /*!
   * note: We are now following REP 103 to use ENU for
   *       short-range Cartesian representations. Local position is published
   *       in ENU Frame
   */
    p->local_position_publisher.publish(local_pos);
  }

  // Telemetry::RelativePosition relative_position;
  // relative_position = vehicle->broadcast->getRelativePosition();
  Telemetry::TypeMap<Telemetry::TOPIC_AVOID_DATA>::type relative_position =
    vehicle->subscribe->getValue<Telemetry::TOPIC_AVOID_DATA>();

  dji_osdk_ros::RelPosition rel_pos_msg;
  rel_pos_msg.header.frame_id = "/rel_pos";
  rel_pos_msg.header.stamp = msg_time;
  rel_pos_msg.down  = relative_position.down;
  rel_pos_msg.front = relative_position.front;
  rel_pos_msg.right = relative_position.right;
  rel_pos_msg.back  = relative_position.back;
  rel_pos_msg.left  = relative_position.left;
  rel_pos_msg.up    = relative_position.up;
  rel_pos_msg.downHealth  = relative_position.downHealth;
  rel_pos_msg.frontHealth = relative_position.frontHealth;
  rel_pos_msg.rightHealth = relative_position.rightHealth;
  rel_pos_msg.backHealth  = relative_position.backHealth;
  rel_pos_msg.leftHealth  = relative_position.leftHealth;
  rel_pos_msg.upHealth    = relative_position.upHealth;
  p->relative_position_publisher.publish(rel_pos_msg);

  Telemetry::TypeMap<Telemetry::TOPIC_HEIGHT_FUSION>::type fused_height =
    vehicle->subscribe->getValue<Telemetry::TOPIC_HEIGHT_FUSION>();
  std_msgs::Float32 height;
  height.data = fused_height;
  p->height_publisher.publish(height);
  Telemetry::TypeMap<Telemetry::TOPIC_STATUS_FLIGHT>::type fs =
    vehicle->subscribe->getValue<Telemetry::TOPIC_STATUS_FLIGHT>();

  std_msgs::UInt8 flight_status;
  flight_status.data = fs;
  p->flight_status_publisher.publish(flight_status);

  Telemetry::TypeMap<Telemetry::TOPIC_VELOCITY>::type v_FC =
    vehicle->subscribe->getValue<Telemetry::TOPIC_VELOCITY>();
  geometry_msgs::Vector3Stamped v;
  // v_FC has 2 fields, data and info. The latter contains the health


  /*!
   * note: We are now following REP 103 to use ENU for
   *       short-range Cartesian representations
   */
  v.header.frame_id = "ground_ENU";
  v.header.stamp = msg_time;
  v.vector.x = v_FC.data.y;  //x, y are swapped from NE to EN
  v.vector.y = v_FC.data.x;
  v.vector.z = v_FC.data.z; //z sign is already U
  p->velocity_publisher.publish(v);

  Telemetry::TypeMap<Telemetry::TOPIC_GPS_CONTROL_LEVEL>::type gps_ctrl_level=
    vehicle->subscribe->getValue<Telemetry::TOPIC_GPS_CONTROL_LEVEL>();
  std_msgs::UInt8 msg_gps_ctrl_level;
  msg_gps_ctrl_level.data = gps_ctrl_level;
  p->current_gps_health = gps_ctrl_level;
  p->gps_health_publisher.publish(msg_gps_ctrl_level);

  Telemetry::TypeMap<Telemetry::TOPIC_GIMBAL_ANGLES>::type gimbal_angle =
    vehicle->subscribe->getValue<Telemetry::TOPIC_GIMBAL_ANGLES>();

  geometry_msgs::Vector3Stamped gimbal_angle_vec3;

  gimbal_angle_vec3.header.stamp = ros::Time::now();
  gimbal_angle_vec3.vector.x     = gimbal_angle.x;
  gimbal_angle_vec3.vector.y     = gimbal_angle.y;
  gimbal_angle_vec3.vector.z     = gimbal_angle.z;
  p->gimbal_angle_publisher.publish(gimbal_angle_vec3);

  // See dji_sdk.h for details about display_mode

  Telemetry::TypeMap<Telemetry::TOPIC_STATUS_DISPLAYMODE>::type dm =
    vehicle->subscribe->getValue<Telemetry::TOPIC_STATUS_DISPLAYMODE>();

  std_msgs::UInt8 status_dm;
  status_dm.data = dm;
  p->displaymode_publisher.publish(status_dm);

  /*!
   * note: Since FW version 3.3.0 and SDK version 3.7, we expose all the button on the LB2 RC
   *       as well as the RC connection status via different topics.
   */
  if(vehicle->getFwVersion() > versionBase33)
  {
    Telemetry::TypeMap<Telemetry::TOPIC_POSITION_VO>::type vo_position =
          vehicle->subscribe->getValue<Telemetry::TOPIC_POSITION_VO>();

    dji_osdk_ros::VOPosition vo_pos;
    // This name does not follow the convention because we are not sure it is real NED.
    vo_pos.header.frame_id = "/ground_nav";
    vo_pos.header.stamp = msg_time;
    vo_pos.x  = vo_position.x;
    vo_pos.y       = vo_position.y;
    vo_pos.z        = vo_position.z;
    vo_pos.xHealth = vo_position.xHealth;
    vo_pos.yHealth = vo_position.yHealth;
    vo_pos.zHealth = vo_position.zHealth;
    p->vo_position_publisher.publish(vo_pos);
  
    Telemetry::TypeMap<Telemetry::TOPIC_RC_WITH_FLAG_DATA>::type rc_with_flag =
            vehicle->subscribe->getValue<Telemetry::TOPIC_RC_WITH_FLAG_DATA>();

    sensor_msgs::Joy rc_joy;
    rc_joy.header.stamp    = msg_time;
    rc_joy.header.frame_id = "rc";

    rc_joy.axes.reserve(12);
    rc_joy.axes.push_back(static_cast<float>(rc_with_flag.roll));
    rc_joy.axes.push_back(static_cast<float>(rc_with_flag.pitch));
    rc_joy.axes.push_back(static_cast<float>(rc_with_flag.yaw));
    rc_joy.axes.push_back(static_cast<float>(rc_with_flag.throttle));

    // A3 and N3 has access to more buttons on RC
    std::string hardwareVersion(vehicle->getHwVersion());
    if( (hardwareVersion == std::string(Version::N3)) || hardwareVersion == std::string(Version::A3))
    {
      Telemetry::TypeMap<Telemetry::TOPIC_RC_FULL_RAW_DATA>::type rc_full_raw =
        vehicle->subscribe->getValue<Telemetry::TOPIC_RC_FULL_RAW_DATA>();
      rc_joy.axes.push_back(static_cast<float>(-(rc_full_raw.lb2.mode - 1024)    / 660));
      rc_joy.axes.push_back(static_cast<float>(-(rc_full_raw.lb2.gear - 1519)    / 165));
      rc_joy.axes.push_back(static_cast<float>((rc_full_raw.lb2.camera -364)   / 1320));
      rc_joy.axes.push_back(static_cast<float>((rc_full_raw.lb2.video - 364)    / 1320));
      rc_joy.axes.push_back(static_cast<float>((rc_full_raw.lb2.videoPause-364) / 1320));
      rc_joy.axes.push_back(static_cast<float>((rc_full_raw.lb2.goHome-364) /1320));
      rc_joy.axes.push_back(static_cast<float>((rc_full_raw.lb2.leftWheel-1024.0)  / 660.0));
      rc_joy.axes.push_back(static_cast<float>((rc_full_raw.lb2.rightWheelButton - 364)/ 1320));
    }
    else
    {
      Telemetry::TypeMap<Telemetry::TOPIC_RC>::type rc =
        vehicle->subscribe->getValue<Telemetry::TOPIC_RC>();

      rc_joy.axes.push_back(static_cast<float>(rc.mode*1.0));
      rc_joy.axes.push_back(static_cast<float>(rc.gear*1.0));
    }

    p->rc_publisher.publish(rc_joy);

    bool temp;
    temp = rc_with_flag.flag.skyConnected && rc_with_flag.flag.groundConnected;

    std_msgs::UInt8 rc_connected;
    rc_connected.data = temp ? 1 : 0;
    p->rc_connection_status_publisher.publish(rc_connected);

    // Publish flight anomaly if FC is supported
    Telemetry::TypeMap<Telemetry::TOPIC_FLIGHT_ANOMALY>::type flight_anomaly_data =
            vehicle->subscribe->getValue<Telemetry::TOPIC_FLIGHT_ANOMALY>();

    dji_osdk_ros::FlightAnomaly flight_anomaly_msg;
    flight_anomaly_msg.data = *(reinterpret_cast<uint32_t*>(&flight_anomaly_data));
    p->flight_anomaly_publisher.publish(flight_anomaly_msg);
  }
  else
  {
    /********* RC Map (A3) *********
    *
    *       -10000  <--->  0      <---> 10000
    * MODE: API(F)  <---> ATTI(A) <--->  POS (P)
    *
    *        CH3 +10000                     CH1 +10000
    *               ^                              ^
    *               |                              |                   / -5000
    *    CH2        |                   CH0        |                  /
    *  -10000 <-----------> +10000    -10000 <-----------> +10000    H
    *               |                              |                  \
    *               |                              |                   \ -10000
    *               V                              V
    *            -10000                         -10000
    *
    *   In this code, before publishing, we normalize RC
    *****************************/

    Telemetry::TypeMap<Telemetry::TOPIC_RC>::type rc =
            vehicle->subscribe->getValue<Telemetry::TOPIC_RC>();

    sensor_msgs::Joy rc_joy;
    rc_joy.header.stamp    = msg_time;
    rc_joy.header.frame_id = "rc";

    rc_joy.axes.reserve(6);

    rc_joy.axes.push_back(static_cast<float>(rc.roll     / 10000.0));
    rc_joy.axes.push_back(static_cast<float>(rc.pitch    / 10000.0));
    rc_joy.axes.push_back(static_cast<float>(rc.yaw      / 10000.0));
    rc_joy.axes.push_back(static_cast<float>(rc.throttle / 10000.0));
    rc_joy.axes.push_back(static_cast<float>(rc.mode*1.0));
    rc_joy.axes.push_back(static_cast<float>(rc.gear*1.0));
    p->rc_publisher.publish(rc_joy);
  }

  uint16_t data_enable_flag = vehicle->broadcast->getPassFlag();
  
  
  //update device control info
  // if (data_enable_flag & DataBroadcast::DATA_ENABLE_FLAG::A3_HAS_DEVICE)
  // {
    Telemetry::TypeMap<Telemetry::TOPIC_CONTROL_DEVICE>::type sdk_info =
      vehicle->subscribe->getValue<Telemetry::TOPIC_CONTROL_DEVICE>();
    std_msgs::UInt8 status_device;
    // status_device.data = vehicle->broadcast->getSDKInfo().deviceStatus;
    status_device.data = sdk_info.deviceStatus;
    // TODO The underlying status changed, but we look for a value of 2. Fix this.
    //https://github.com/dji-sdk/Onboard-SDK/issues/528
    //Jeff's farewell Mary Poppins
    if (status_device.data == 4){
      status_device.data = 2;
      ROS_INFO_THROTTLE(1,"**UNDER AM CONTROL**");
    }
    p->device_status_publisher.publish(status_device);
  // }
}


void
DJISDKNode::publish100HzData(Vehicle *vehicle, RecvContainer recvFrame,
                                  DJI::OSDK::UserData userData)
{
  DJISDKNode *p = (DJISDKNode *)userData;

  uint8_t* data = recvFrame.recvData.raw_ack_array;
  ROS_ASSERT(DJISDKNode::PACKAGE_ID_100HZ == *data );
  data++;
  Telemetry::TimeStamp packageTimeStamp = * (reinterpret_cast<Telemetry::TimeStamp *>(data));

  ros::Time msg_time = ros::Time::now();

  if(p->align_time_with_FC)
  {
    if(p->curr_align_state == ALIGNED)
    {
      msg_time = p->base_time + _TICK2ROSTIME(packageTimeStamp.time_ms);
    }
    else
    {
      return;
    }
  }

  Telemetry::TypeMap<Telemetry::TOPIC_QUATERNION>::type quat =
          vehicle->subscribe->getValue<Telemetry::TOPIC_QUATERNION>();
  geometry_msgs::QuaternionStamped q;

  /*!
   * note: We are now following REP 103 to use FLU for
   *       body frame. The quaternion is the rotation from
   *       body_FLU to ground_ENU
   */
  q.header.frame_id = "body_FLU";
  q.header.stamp    = msg_time;

  tf::Matrix3x3 R_FRD2NED(tf::Quaternion(quat.q1, quat.q2, quat.q3, quat.q0));
  tf::Matrix3x3 R_FLU2ENU = p->R_ENU2NED.transpose() * R_FRD2NED * p->R_FLU2FRD;
  tf::Quaternion q_FLU2ENU;
  R_FLU2ENU.getRotation(q_FLU2ENU);
  // @note this mapping is tested
  q.quaternion.w = q_FLU2ENU.getW();
  q.quaternion.x = q_FLU2ENU.getX();
  q.quaternion.y = q_FLU2ENU.getY();
  q.quaternion.z = q_FLU2ENU.getZ();
  p->attitude_publisher.publish(q);

  Telemetry::TypeMap<Telemetry::TOPIC_ANGULAR_RATE_FUSIONED>::type w_FC =
    vehicle->subscribe->getValue<Telemetry::TOPIC_ANGULAR_RATE_FUSIONED>();

  geometry_msgs::Vector3Stamped angular_rate;

  /*!
   * note: We are now following REP 103 to use FLU for
   *       body frame
   */
  angular_rate.header.frame_id = "body_FLU";
  angular_rate.header.stamp    = msg_time;

  angular_rate.vector.x        =  w_FC.x;
  angular_rate.vector.y        = -w_FC.y; //y,z sign are flipped from RD to LU
  angular_rate.vector.z        = -w_FC.z;
  p->angularRate_publisher.publish(angular_rate);

  Telemetry::TypeMap<Telemetry::TOPIC_ACCELERATION_GROUND>::type a_FC =
    vehicle->subscribe->getValue<Telemetry::TOPIC_ACCELERATION_GROUND>();
  geometry_msgs::Vector3Stamped acceleration;

  /*!
   * note: 1. We are now following REP 103 to use ENU for
   *       short-range Cartesian representations
   *
   *       2. TODO: This accel is in ground frame, which may
   *       cause confusion with the body-frame accel in imu message
   */

  acceleration.header.frame_id = "ground_ENU";
  acceleration.header.stamp    = msg_time;

  acceleration.vector.x        = a_FC.y;  //x, y are swapped from NE to EN
  acceleration.vector.y        = a_FC.x;
  acceleration.vector.z        = a_FC.z;  //z sign is already U
  p->acceleration_publisher.publish(acceleration);
}

void
DJISDKNode::publish400HzData(Vehicle *vehicle, RecvContainer recvFrame,
                                  DJI::OSDK::UserData userData)
{
  DJISDKNode *p = (DJISDKNode *) userData;

  uint8_t* data = recvFrame.recvData.raw_ack_array;
  ROS_ASSERT(DJISDKNode::PACKAGE_ID_400HZ == *data );

  data++;
  Telemetry::TimeStamp packageTimeStamp = * (reinterpret_cast<Telemetry::TimeStamp *>(data));

  Telemetry::TypeMap<Telemetry::TOPIC_HARD_SYNC>::type hardSync_FC =
    vehicle->subscribe->getValue<Telemetry::TOPIC_HARD_SYNC>();

  ros::Time now_time = ros::Time::now();
  ros::Time msg_time = now_time;

  if(p->align_time_with_FC)
  {
    p->alignRosTimeWithFlightController(now_time, packageTimeStamp.time_ms);
    if(p->curr_align_state == ALIGNED)
    {
      msg_time = p->base_time + _TICK2ROSTIME(packageTimeStamp.time_ms);
    }
    else
    {
      return;
    }
  }

  sensor_msgs::Imu synced_imu;

  synced_imu.header.frame_id = "body_FLU";
  synced_imu.header.stamp    = msg_time;

  //y, z signs are flipped from RD to LU for rate and accel
  synced_imu.angular_velocity.x    =   hardSync_FC.w.x;
  synced_imu.angular_velocity.y    =  -hardSync_FC.w.y;
  synced_imu.angular_velocity.z    =  -hardSync_FC.w.z;

  synced_imu.linear_acceleration.x =   hardSync_FC.a.x * p->gravity_const;
  synced_imu.linear_acceleration.y =  -hardSync_FC.a.y * p->gravity_const;
  synced_imu.linear_acceleration.z =  -hardSync_FC.a.z * p->gravity_const;

  /*!
   * The quaternion is the rotation from body_FLU to ground_ENU.
   * Refer to:
   *   https://github.com/mavlink/mavros/blob/master/mavros/src/plugins/imu_pub.cpp
   */
  tf::Matrix3x3 R_FRD2NED(tf::Quaternion(hardSync_FC.q.q1, hardSync_FC.q.q2,
                                         hardSync_FC.q.q3, hardSync_FC.q.q0));
  tf::Matrix3x3 R_FLU2ENU = p->R_ENU2NED.transpose() * R_FRD2NED * p->R_FLU2FRD;
  tf::Quaternion q_FLU2ENU;
  R_FLU2ENU.getRotation(q_FLU2ENU);

  synced_imu.orientation.w = q_FLU2ENU.getW();
  synced_imu.orientation.x = q_FLU2ENU.getX();
  synced_imu.orientation.y = q_FLU2ENU.getY();
  synced_imu.orientation.z = q_FLU2ENU.getZ();

  p->imu_publisher.publish(synced_imu);

  if (hardSync_FC.ts.flag == 1)
  {
    sensor_msgs::TimeReference trigTime;
    trigTime.header.stamp = msg_time;
    trigTime.time_ref     = now_time;
    trigTime.source       = "FC";

    p->trigger_publisher.publish(trigTime);
  }
}

/*!
 * @brief: The purpose of time alignment is to use the time received from flight
 *         controller to stamp all published ros messages. The reason is that the
 *         flight controller is running a real time system, while the ros time can
 *         be affected by OS scheduling depending on system load.
 */

void DJISDKNode::alignRosTimeWithFlightController(ros::Time now_time, uint32_t tick)
{
  if (curr_align_state == UNALIGNED)
  {
    base_time = now_time - _TICK2ROSTIME(tick);
    curr_align_state = ALIGNING;
    ROS_INFO("[dji_osdk_ros] Start time alignment ...");
    return;
  }

  if (curr_align_state == ALIGNING)
  {
    static int aligned_count = 0;
    static int retry_count = 0;
    ROS_INFO_THROTTLE(1.0, "[dji_osdk_ros] Aliging time...");

    double dt = std::fabs((now_time - (base_time + _TICK2ROSTIME(tick))).toSec());

    if(dt < TIME_DIFF_CHECK )
    {
      aligned_count++;
    }
    else if(aligned_count > 0)
    {
      base_time = now_time - _TICK2ROSTIME(tick);
      ROS_INFO("[dji_osdk_ros] ***** Time difference out of bound after %d samples, retried %d times, dt=%.3f... *****",
               aligned_count, retry_count, dt);
      aligned_count = 0;
      retry_count++;
    }

    if(aligned_count > STABLE_ALIGNMENT_COUNT)
    {
      ROS_INFO("[dji_osdk_ros] ***** Time alignment successful! *****");
      curr_align_state = ALIGNED;
    }

    return;
  }
}

#ifdef ADVANCED_SENSING

sensor_msgs::CameraInfo DJISDKNode::getCameraInfo(int camera_select, bool isLeftRequired)
{
	sensor_msgs::CameraInfo cam_info;
	
	cam_info.width = 320;
	cam_info.height = 240;
	
	cam_info.distortion_model = "plumb_bob";
	
	cam_info.D = std::vector<double>(5,0.0);
	
	switch(camera_select)
	{
		//front camera
		case 1:
		{
			cam_info.K = {407.1327819824219, 0, 321.8077392578125, 0.0, 407.1327819824219, 236.9904937744141, 0.0, 0.0, 1.0};
			cam_info.R = {1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0};
			cam_info.P = {407.1327819824219, 0, 321.8077392578125, -116.0410842895508, 0.0, 407.1327819824219, 236.9904937744141, 0.0, 0.0, 0.0, 1.0, 0.0};
			break;
		}
		//left camera
		case 2:
		{
			if(isLeftRequired)
			{
				cam_info.K = {405.863209, 0.000000, 321.034642,0.000000, 406.405344, 234.984922,0.000000, 0.000000, 1.000000};
				cam_info.D = {-0.004661, 0.003370, 0.000506, 0.000643, 0.000000};
				cam_info.R = {0.999999, 0.000188, 0.001306,-0.000189, 1.000000, 0.000714,-0.001306, -0.000715, 0.999999};
				cam_info.P = {406.427434, 0.000000, 320.900703, 0.000000,0.000000, 406.427434, 234.718628, 0.000000,0.000000, 0.000000, 1.000000, 0.000000};
				break;
			}
			
				cam_info.K = {405.958187, 0.000000, 322.083430,0.000000, 406.341539, 234.588475, 0.000000, 0.000000, 1.000000};
				cam_info.D = {-0.005493, 0.003726, -0.000700, 0.000489, 0.000000};
				cam_info.R = {0.999997, 0.000116, 0.002235,-0.000115, 1.000000, -0.000715,-0.002235, 0.000714, 0.999997};
				cam_info.P = {406.427434, 0.000000, 320.900703, -48.321484,0.000000, 406.427434, 234.718628, 0.000000,0.000000, 0.000000, 1.000000, 0.000000};
			break;
		}
		//right camera
		case 3:
		{
			cam_info.K = {488.5599365234375, 0.0, 319.80328369140625, 0.0, 488.5599365234375, 239.97528076171875, 0.0, 0.0, 1.0};
			cam_info.R = {1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0};
			cam_info.P = {488.5599365234375, 0.0, 319.80328369140625, -99.43592071533203, 0.0, 488.5599365234375, 239.97528076171875, 0.0, 0.0, 0.0, 1.0, 0.0};
			break;
		}
		//rear camera
		case 4:
		{
			cam_info.K = {486.2147216796875, 0.0, 318.2126159667969, 0.0, 486.2147216796875, 238.1256408691406, 0.0, 0.0, 1.0};
			cam_info.R = {1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0};
			cam_info.P = {486.2147216796875, 0.0, 318.2126159667969, -99.34449005126953, 0.0, 486.2147216796875, 238.1256408691406, 0.0, 0.0, 0.0, 1.0, 0.0};
			break;
		}
		//down camera
		case 5:
		{
			cam_info.K = {480.2206726074219, 0.0, 318.1107177734375, 0.0, 480.2206726074219, 242.7613830566406, 0.0, 0.0, 1.0};
			cam_info.R = {1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0};
			cam_info.P = {480.2206726074219, 0.0, 318.1107177734375, 0.0,  0.0, 480.2206726074219, 242.7613830566406, 79.36713409423828, 0.0, 0.0, 1.0, 0.0};
			break;
		}
		
		//up camera
		case 6:
		{
			cam_info.width = 240;
			cam_info.height = 320;
			if(isLeftRequired)
			{
				cam_info.K = {201.990533, 0.000000, 118.106840,0.000000, 202.493255, 158.157213, 0.000000, 0.000000, 1.000000};
				cam_info.D = {0.001765, -0.002758, -0.000692, -0.001906, 0.000000};
				cam_info.R = {0.999994, -0.001909, -0.002898,0.001904, 0.999997, -0.001712,0.002901, 0.001707, 0.999994};
				cam_info.P = {204.589978, 0.000000, 118.121675, 0.000000,0.000000, 204.589978, 158.783770, 0.000000,0.000000, 0.000000, 1.000000, 0.000000};
				break;
			}
			
				cam_info.K = {202.192738, 0.000000, 118.737948,0.000000, 202.403765, 159.274456, 0.000000, 0.000000, 1.000000};
				cam_info.D = {0.003813, 0.001792, 0.001038, -0.000216, 0.000000};
				cam_info.R = {0.999995, -0.001825, 0.002471,0.001820, 0.999997, 0.001712,-0.002474, -0.001707, 0.999995};
				cam_info.P = {204.589978, 0.000000, 118.121675, 20.188123,0.000000, 204.589978, 158.783770, 0.000000,0.000000, 0.000000, 1.000000, 0.000000};
			break;
		}
		default:
			break;
	}
	
	
	return cam_info;
}

void DJISDKNode::publishCameraInfo(const std_msgs::Header &header)
{	
	if(latest_camera_ < 1)
	{
		ROS_WARN_THROTTLE(5.0,"selected camera id is not correct");
		return;
	}
	
	sensor_msgs::CameraInfo left_camera_info = getCameraInfo(latest_camera_, true);
	sensor_msgs::CameraInfo right_camera_info = getCameraInfo(latest_camera_, false);
	
	left_camera_info.distortion_model = right_camera_info.distortion_model = "plumb_bob";
			
	left_camera_info.header = right_camera_info.header = header;
	left_camera_info.header.frame_id = "left_camera";
	right_camera_info.header.frame_id = "right_camera";
	left_camera_info_pub_.publish(left_camera_info);
	right_camera_info_pub_.publish(right_camera_info);

}

void DJISDKNode::publish240pStereoImage(Vehicle*            vehicle,
                                        RecvContainer       recvFrame,
                                        DJI::OSDK::UserData userData)
{
  DJISDKNode *node_ptr = (DJISDKNode *)userData;

  node_ptr->stereo_subscription_success = true;

  sensor_msgs::Image img;
  img.height = 240;
  img.width = 320;
  img.data.resize(img.height*img.width);
  img.encoding = "mono8";
  img.step = 320;
  uint8_t img_idx = 0;

  for (int pair_idx = 0; pair_idx < CAMERA_PAIR_NUM; ++pair_idx) {
    for (int dir_idx = 0; dir_idx < IMAGE_TYPE_NUM; ++dir_idx) {

      uint8_t bit_location = pair_idx * IMAGE_TYPE_NUM + dir_idx;
      uint8_t bit_val = (recvFrame.recvData.stereoImgData->img_desc >> bit_location) & 1;

      if (bit_val) {
        img.header.seq = recvFrame.recvData.stereoImgData->frame_index;
        img.header.stamp = ros::Time::now(); // @todo
        img.header.frame_id = recvFrame.recvData.stereoImgData->img_vec[img_idx].name;
        memcpy((char*)(&img.data[0]), recvFrame.recvData.stereoImgData->img_vec[img_idx++].image, 240*320);

        if (bit_location == AdvancedSensing::RECV_FRONT_LEFT)
          node_ptr->stereo_240p_front_left_publisher.publish(img);
        if (bit_location == AdvancedSensing::RECV_FRONT_RIGHT)
          node_ptr->stereo_240p_front_right_publisher.publish(img);
        if (bit_location == AdvancedSensing::RECV_DOWN_BACK)
          node_ptr->stereo_240p_down_back_publisher.publish(img);
        if (bit_location == AdvancedSensing::RECV_DOWN_FRONT)
          node_ptr->stereo_240p_down_front_publisher.publish(img);
        if (bit_location == AdvancedSensing::RECV_FRONT_DEPTH)
          node_ptr->stereo_240p_front_depth_publisher.publish(img);
      }
    }
  }
}

void DJISDKNode::publishVGAStereoImage(Vehicle*            vehicle,
                                       RecvContainer       recvFrame,
                                       DJI::OSDK::UserData userData)
{
  DJISDKNode *node_ptr = (DJISDKNode *)userData;

  node_ptr->stereo_vga_subscription_success = true;
  sensor_msgs::Image img;
  img.height = 480;
  img.width = 640;
  img.step = 640;
  img.encoding = "mono8";
  img.data.resize(img.height*img.width);
	memcpy((char*)(&img.data[0]), recvFrame.recvData.stereoVGAImgData->img_vec[0], 480*640);
	//processRosImage(img, node_ptr->latest_camera_);
  img.header.seq = recvFrame.recvData.stereoVGAImgData->frame_index;
  img.header.stamp = ros::Time::now(); // @todo
  img.header.frame_id = "vga_left";
	node_ptr->stereo_vga_front_left_publisher.publish(img);
	
	
	
	img.height = 480;
  img.width = 640;
  img.step = 640;
  img.encoding = "mono8";
  img.data.resize(img.height*img.width);
	memcpy((char*)(&img.data[0]), recvFrame.recvData.stereoVGAImgData->img_vec[1], 480*640);
	//processRosImage(img, node_ptr->latest_camera_);
  img.header.frame_id = "vga_right";
	node_ptr->stereo_vga_front_right_publisher.publish(img);
	
 
  //img.header.frame_id = "vga_right";
	//unsigned char * img_data_ptr2 = (unsigned char*) &recvFrame.recvData.stereoVGAImgData->img_vec[1];
  //cv::Mat mat2(img.height, img.width, CV_8U, img_data_ptr, img.step);
  //perform operations on the cv image
 // cv::resize(mat2, mat2, cv::Size(240, 320),0,0,cv::INTER_LINEAR);
  //memcpy((char*)(&img.data[0]), recvFrame.recvData.stereoVGAImgData->img_vec[1], 480*640);
  //img_bridge = cv_bridge::CvImage(img.header, sensor_msgs::image_encodings::MONO8, mat2);
	//img_bridge.toImageMsg(img);
  //node_ptr->stereo_vga_front_right_publisher.publish(img);
  
  node_ptr->publishCameraInfo(img.header);
}

void DJISDKNode::processRosImage(sensor_msgs::Image &img, int camera_select)
{
	
	//get cv image
	cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(img, img.encoding);
	cv::Mat mat = cv_ptr->image;
  //perform operations on the cv image
  cv::resize(mat, mat, cv::Size(img.width/2, img.height/2),0,0,cv::INTER_LINEAR);
  
  
  //convert to ros image
	img.height = 240;
  img.width = 320;
  img.step = 320;
  img.encoding = "mono8";
  img.data.resize(img.height*img.width);
  
  if(camera_select == 6)
  {
  	double angle = 90.0;
  	// get rotation matrix for rotating the image around its center in pixel coordinates
    cv::Point2f center((mat.cols-1)/2.0, (mat.rows-1)/2.0);
    cv::Mat rot = cv::getRotationMatrix2D(center, angle, 1.0);
    // determine bounding rectangle, center not relevant
    cv::Rect2f bbox = cv::RotatedRect(cv::Point2f(), mat.size(), angle).boundingRect2f();
    // adjust transformation matrix
    rot.at<double>(0,2) += bbox.width/2.0 - mat.cols/2.0;
    rot.at<double>(1,2) += bbox.height/2.0 - mat.rows/2.0;

    cv::warpAffine(mat, mat, rot, bbox.size());
    
    img.height = 320;
    img.width = 240;
    img.step = 240;
  }
  
  
	
	
	cv_bridge::CvImage img_bridge = cv_bridge::CvImage(img.header, sensor_msgs::image_encodings::MONO8, mat);
	img_bridge.toImageMsg(img);
}

void DJISDKNode::publishFPVCameraImage(CameraRGBImage rgbImg, void* userData)
{
  DJISDKNode *node_ptr = (DJISDKNode *)userData;

  sensor_msgs::Image img;
  img.height = rgbImg.height;
  img.width = rgbImg.width;
  img.step = rgbImg.width*3;
  img.encoding = "rgb8";
  img.data = rgbImg.rawData;

  img.header.stamp = ros::Time::now();
  img.header.frame_id = "FPV_CAMERA";
  node_ptr->fpv_camera_stream_publisher.publish(img);
}

void DJISDKNode::publishMainCameraImage(CameraRGBImage rgbImg, void* userData)
{
  DJISDKNode *node_ptr = (DJISDKNode *)userData;

  sensor_msgs::Image img;
  img.height = rgbImg.height;
  img.width = rgbImg.width;
  img.step = rgbImg.width*3;
  img.encoding = "rgb8";
  img.data = rgbImg.rawData;

  img.header.stamp = ros::Time::now();
  img.header.frame_id = "MAIN_CAMERA";
  node_ptr->main_camera_stream_publisher.publish(img);
}
#endif // ADVANCED_SENSING
