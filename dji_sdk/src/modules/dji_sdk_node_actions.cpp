/** @file dji_sdk_node_actions.cpp
 *  @version 3.1.8
 *  @date July 29th, 2016
 *
 *  @brief
 *  All the action callbacks are implemented here.
 *
 *  @copyright 2016 DJI. All rights reserved.
 *
 */


#include <dji_sdk/dji_sdk_node.h>
#include <algorithm>

bool DJISDKNode::process_waypoint(dji_sdk::Waypoint new_waypoint) 
{
    double dst_latitude = new_waypoint.latitude;
    double dst_longitude = new_waypoint.longitude;
    float dst_altitude = new_waypoint.altitude;

    double org_latitude = global_position.latitude;
    double org_longitude = global_position.longitude;
    float org_altitude = global_position.altitude;

    double dis_x, dis_y;
    float dis_z;

    dis_x = dst_latitude - org_latitude;
    dis_y = dst_longitude - org_longitude;
    dis_z = dst_altitude - org_altitude;

    double det_x,det_y;
    float det_z;


    DJI::onboardSDK::FlightData flight_ctrl_data;
    flight_ctrl_data.flag = 0x90;
    flight_ctrl_data.z = dst_altitude;
    flight_ctrl_data.yaw = new_waypoint.heading;


    int latitude_progress = 0; 
    int longitude_progress = 0; 
    int altitude_progress = 0; 

    while (latitude_progress < 100 || longitude_progress < 100 || altitude_progress <100) {
        if(waypoint_navigation_action_server->isPreemptRequested()) {
            return false;
        }

        double d_lon = dst_longitude - global_position.longitude;
        double d_lat = dst_latitude - global_position.latitude;

        flight_ctrl_data.x = ((d_lat) *C_PI/180) * C_EARTH;
        flight_ctrl_data.y = ((d_lon) * C_PI/180) * C_EARTH * cos((dst_latitude)*C_PI/180);
        rosAdapter->flight->setFlight(&flight_ctrl_data);

        det_x = (100 * (dst_latitude - global_position.latitude))/dis_x;
        det_y = (100 * (dst_longitude - global_position.longitude))/dis_y;
        det_z = (100 * (dst_altitude - global_position.altitude))/dis_z;

        latitude_progress = 100 - std::abs((int) det_x);
        longitude_progress = 100 - std::abs((int) det_y);
        altitude_progress = 100 - std::abs((int) det_z);

        //lazy evaluation
        //need to find a better way
        if (std::abs(dst_latitude - global_position.latitude) < 0.00001) latitude_progress = 100;
        if (std::abs(dst_longitude - global_position.longitude) < 0.00001) longitude_progress = 100;
        if (std::abs(dst_altitude - global_position.altitude) < 0.12) altitude_progress = 100;

        waypoint_navigation_feedback.latitude_progress = latitude_progress;
        waypoint_navigation_feedback.longitude_progress = longitude_progress;
        waypoint_navigation_feedback.altitude_progress = altitude_progress;
        waypoint_navigation_action_server->publishFeedback(waypoint_navigation_feedback);

        usleep(20000);

    }
    ros::Duration(new_waypoint.staytime).sleep();
    return true;
}


bool DJISDKNode::drone_task_action_callback(const dji_sdk::DroneTaskGoalConstPtr& goal)
{
  uint8_t request_action = goal->task;

  if (request_action == 1)
  {
    //takeoff
    rosAdapter->flight->task(DJI::onboardSDK::Flight::TASK::TASK_TAKEOFF);
  }
  else if (request_action == 2)
  {
    //landing
    rosAdapter->flight->task(DJI::onboardSDK::Flight::TASK::TASK_LANDING);
  }
  else if (request_action == 3)
  {
    //gohome
    rosAdapter->flight->task(DJI::onboardSDK::Flight::TASK::TASK_GOHOME);
  }

  drone_task_feedback.progress = 1;
  drone_task_action_server->publishFeedback(drone_task_feedback);
  drone_task_action_server->setSucceeded();
  
  return true;
}


bool DJISDKNode::local_position_navigation_action_callback(const dji_sdk::LocalPositionNavigationGoalConstPtr& goal)
{
  /*IMPORTANT*/
  /*
     There has been declared a pointer `local_navigation_action` as the function parameter,
     However, it is the `local_navigation_action_server` that we should use.
     If `local_navigation_action` is used instead, there will be a runtime sengmentation fault.

     so interesting
  */

  float dst_x = goal->x;
  float dst_y = goal->y;
  float dst_z = goal->z;

  float org_x = local_position.x;
  float org_y = local_position.y;
  float org_z = local_position.z;

  float dis_x = dst_x - org_x;
  float dis_y = dst_y - org_y;
  float dis_z = dst_z - org_z; 

  float det_x, det_y, det_z;

  DJI::onboardSDK::FlightData flight_ctrl_data;
  flight_ctrl_data.flag = 0x90;
  flight_ctrl_data.z = dst_z;
  flight_ctrl_data.yaw = 0;

  int x_progress = 0; 
  int y_progress = 0; 
  int z_progress = 0; 
  while (x_progress < 100 || y_progress < 100 || z_progress <100) {

     flight_ctrl_data.x = dst_x - local_position.x;
     flight_ctrl_data.y = dst_y - local_position.y;
     rosAdapter->flight->setFlight(&flight_ctrl_data);

     det_x = (100 * (dst_x - local_position.x)) / dis_x;
     det_y = (100 * (dst_y - local_position.y)) / dis_y;
     det_z = (100 * (dst_z - local_position.z)) / dis_z;

     x_progress = 100 - (int)det_x;
     y_progress = 100 - (int)det_y;
     z_progress = 100 - (int)det_z;

     //lazy evaluation
     if (std::abs(dst_x - local_position.x) < 0.1) x_progress = 100;
     if (std::abs(dst_y - local_position.y) < 0.1) y_progress = 100;
     if (std::abs(dst_z - local_position.z) < 0.1) z_progress = 100;

     local_position_navigation_feedback.x_progress = x_progress;
     local_position_navigation_feedback.y_progress = y_progress;
     local_position_navigation_feedback.z_progress = z_progress;
     local_position_navigation_action_server->publishFeedback(local_position_navigation_feedback);

     usleep(20000);
  }

  local_position_navigation_result.result = true;
  local_position_navigation_action_server->setSucceeded(local_position_navigation_result);

  return true;
}


bool DJISDKNode::global_position_navigation_action_callback(const dji_sdk::GlobalPositionNavigationGoalConstPtr& goal)
{
    double dst_latitude = goal->latitude;
    double dst_longitude = goal->longitude;
    float dst_altitude = goal->altitude;

    double org_latitude = global_position.latitude;
    double org_longitude = global_position.longitude;
    float org_altitude = global_position.altitude;

    double dis_x, dis_y;
    float dis_z;

    dis_x = dst_latitude - org_latitude;
    dis_y = dst_longitude - org_longitude;
    dis_z = dst_altitude - org_altitude;

    double det_x, det_y;
    float det_z;

    DJI::onboardSDK::FlightData flight_ctrl_data;
    flight_ctrl_data.flag = 0x90;
    flight_ctrl_data.z = dst_altitude;
    flight_ctrl_data.yaw = 0;


    int latitude_progress = 0; 
    int longitude_progress = 0; 
    int altitude_progress = 0; 

    while (latitude_progress < 100 || longitude_progress < 100 || altitude_progress < 100) {

        double d_lon = dst_longitude - global_position.longitude;
        double d_lat = dst_latitude - global_position.latitude;

        flight_ctrl_data.x = ((d_lat) *C_PI/180) * C_EARTH;
        flight_ctrl_data.y = ((d_lon) * C_PI/180) * C_EARTH * cos((dst_latitude)*C_PI/180);
        rosAdapter->flight->setFlight(&flight_ctrl_data);

        det_x = (100* (dst_latitude - global_position.latitude))/dis_x;
        det_y = (100* (dst_longitude - global_position.longitude))/dis_y;
        det_z = (100* (dst_altitude - global_position.altitude))/dis_z;


        latitude_progress = 100 - (int)det_x;
        longitude_progress = 100 - (int)det_y;
        altitude_progress = 100 - (int)det_z;

        //lazy evaluation
        if (std::abs(dst_latitude - global_position.latitude) < 0.00001) latitude_progress = 100;
        if (std::abs(dst_longitude - global_position.longitude) < 0.00001) longitude_progress = 100;
        if (std::abs(dst_altitude - global_position.altitude) < 0.12) altitude_progress = 100;


        global_position_navigation_feedback.latitude_progress = latitude_progress;
        global_position_navigation_feedback.longitude_progress = longitude_progress;
        global_position_navigation_feedback.altitude_progress = altitude_progress;
        global_position_navigation_action_server->publishFeedback(global_position_navigation_feedback);

        usleep(20000);

    }

    global_position_navigation_result.result = true;
    global_position_navigation_action_server->setSucceeded(global_position_navigation_result);

    return true;
}


bool DJISDKNode::waypoint_navigation_action_callback(const dji_sdk::WaypointNavigationGoalConstPtr& goal)
{
    dji_sdk::WaypointList new_waypoint_list;
    new_waypoint_list = goal->waypoint_list;

    bool isSucceeded;
    for (int i = 0; i < new_waypoint_list.waypoint_list.size(); i++) {
        const dji_sdk::Waypoint new_waypoint = new_waypoint_list.waypoint_list[i];
        waypoint_navigation_feedback.index_progress = i;
        isSucceeded = process_waypoint(new_waypoint);
        if(!isSucceeded) {
            waypoint_navigation_result.result = false;
            waypoint_navigation_action_server->setPreempted(waypoint_navigation_result);
            return false;
        }
    }

    waypoint_navigation_result.result = true;
    waypoint_navigation_action_server->setSucceeded(waypoint_navigation_result);

    return true;
}

