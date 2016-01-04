#ifndef __DJI_SDK_NODE_H__
#define __DJI_SDK_NODE_H__

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/UInt8.h>
#include <boost/bind.hpp>
#include <dji_sdk/dji_sdk.h>
#include <actionlib/server/simple_action_server.h>

#define C_EARTH (double) 6378137.0
#define C_PI (double) 3.141592653589793

class DJISDKNode
{
private:
//Drone state variables:
    dji_sdk::Acceleration acceleration;
    dji_sdk::AttitudeQuaternion attitude_quaternion;
    dji_sdk::Compass compass;
    dji_sdk::FlightControlInfo flight_control_info;
    uint8_t flight_status;
    dji_sdk::Gimbal gimbal;
    dji_sdk::GlobalPosition global_position;
    dji_sdk::GlobalPosition global_position_ref;
    dji_sdk::LocalPosition local_position;
    dji_sdk::LocalPosition local_position_ref;
    dji_sdk::PowerStatus power_status;
    dji_sdk::RCChannels rc_channels;
    dji_sdk::Velocity velocity;
    nav_msgs::Odometry odometry;

    bool sdk_permission_opened = false;
    bool activated = false;
    bool localposbase_use_height = true;

    int global_position_ref_seted = 0;

//internal variables
    char app_key[65];
    activate_data_t user_act_data;

//Publishers:
    ros::Publisher acceleration_publisher;
    ros::Publisher attitude_quaternion_publisher;
    ros::Publisher compass_publisher;
    ros::Publisher flight_control_info_publisher;
    ros::Publisher flight_status_publisher; 
    ros::Publisher gimbal_publisher;
    ros::Publisher global_position_publisher;
    ros::Publisher local_position_publisher;
    ros::Publisher power_status_publisher;
    ros::Publisher rc_channels_publisher;
    ros::Publisher velocity_publisher;
    ros::Publisher activation_publisher;
    ros::Publisher odometry_publisher;
    ros::Publisher sdk_permission_publisher;

    void init_publishers(ros::NodeHandle& nh)
    {
        // start ros publisher
        acceleration_publisher = nh.advertise<dji_sdk::Acceleration>("dji_sdk/acceleration", 10);
        attitude_quaternion_publisher = nh.advertise<dji_sdk::AttitudeQuaternion>("dji_sdk/attitude_quaternion", 10);
        compass_publisher = nh.advertise<dji_sdk::Compass>("dji_sdk/compass", 10);
        flight_control_info_publisher = nh.advertise<dji_sdk::FlightControlInfo>("dji_sdk/flight_control_info", 10);
        flight_status_publisher = nh.advertise<std_msgs::UInt8>("dji_sdk/flight_status", 10);
        gimbal_publisher = nh.advertise<dji_sdk::Gimbal>("dji_sdk/gimbal", 10);
        global_position_publisher = nh.advertise<dji_sdk::GlobalPosition>("dji_sdk/global_position", 10);
        local_position_publisher = nh.advertise<dji_sdk::LocalPosition>("dji_sdk/local_position", 10);
        power_status_publisher = nh.advertise<dji_sdk::PowerStatus>("dji_sdk/power_status", 10);
        rc_channels_publisher = nh.advertise<dji_sdk::RCChannels>("dji_sdk/rc_channels", 10);
        velocity_publisher = nh.advertise<dji_sdk::Velocity>("dji_sdk/velocity", 10);
        activation_publisher = nh.advertise<std_msgs::UInt8>("dji_sdk/activation", 10);
        odometry_publisher = nh.advertise<nav_msgs::Odometry>("dji_sdk/odometry",10);
        sdk_permission_publisher = nh.advertise<std_msgs::UInt8>("dji_sdk/sdk_permission", 10);
    }

//Services:
    ros::ServiceServer attitude_control_service;
    ros::ServiceServer camera_action_control_service;
    ros::ServiceServer drone_task_control_service;
    ros::ServiceServer gimbal_angle_control_service;
    ros::ServiceServer gimbal_speed_control_service;
    ros::ServiceServer global_position_control_service;
    ros::ServiceServer local_position_control_service;
    ros::ServiceServer sdk_permission_control_service;
    ros::ServiceServer velocity_control_service;

    bool attitude_control_callback(dji_sdk::AttitudeControl::Request& request, dji_sdk::AttitudeControl::Response& response);
    bool camera_action_control_callback(dji_sdk::CameraActionControl::Request& request, dji_sdk::CameraActionControl::Response& response);
    bool drone_task_control_callback(dji_sdk::DroneTaskControl::Request& request, dji_sdk::DroneTaskControl::Response& response);
    bool gimbal_angle_control_callback(dji_sdk::GimbalAngleControl::Request& request, dji_sdk::GimbalAngleControl::Response& response);
    bool gimbal_speed_control_callback(dji_sdk::GimbalSpeedControl::Request& request, dji_sdk::GimbalSpeedControl::Response& response);
    bool global_position_control_callback(dji_sdk::GlobalPositionControl::Request& request, dji_sdk::GlobalPositionControl::Response& response);
    bool local_position_control_callback(dji_sdk::LocalPositionControl::Request& request, dji_sdk::LocalPositionControl::Response& response);
    bool sdk_permission_control_callback(dji_sdk::SDKPermissionControl::Request& request, dji_sdk::SDKPermissionControl::Response& response);
    bool velocity_control_callback(dji_sdk::VelocityControl::Request& request, dji_sdk::VelocityControl::Response& response);

    void init_services(ros::NodeHandle& nh)
    {
        attitude_control_service = nh.advertiseService("dji_sdk/attitude_control", &DJISDKNode::attitude_control_callback, this);
        camera_action_control_service = nh.advertiseService("dji_sdk/camera_action_control",&DJISDKNode::camera_action_control_callback, this);
        drone_task_control_service = nh.advertiseService("dji_sdk/drone_task_control", &DJISDKNode::drone_task_control_callback, this);
        gimbal_angle_control_service = nh.advertiseService("dji_sdk/gimbal_angle_control", &DJISDKNode::gimbal_angle_control_callback, this);
        gimbal_speed_control_service = nh.advertiseService("dji_sdk/gimbal_speed_control", &DJISDKNode::gimbal_speed_control_callback, this);
        global_position_control_service = nh.advertiseService("dji_sdk/global_position_control", &DJISDKNode::global_position_control_callback, this);
        local_position_control_service = nh.advertiseService("dji_sdk/local_position_control", &DJISDKNode::local_position_control_callback, this);
        sdk_permission_control_service = nh.advertiseService("dji_sdk/sdk_permission_control", &DJISDKNode::sdk_permission_control_callback, this);
        velocity_control_service = nh.advertiseService("dji_sdk/velocity_control", &DJISDKNode::velocity_control_callback, this);
    }

//Actions:
    typedef actionlib::SimpleActionServer<dji_sdk::DroneTaskAction> DroneTaskActionServer;
    typedef actionlib::SimpleActionServer<dji_sdk::LocalPositionNavigationAction> LocalPositionNavigationActionServer;
    typedef actionlib::SimpleActionServer<dji_sdk::GlobalPositionNavigationAction> GlobalPositionNavigationActionServer;
    typedef actionlib::SimpleActionServer<dji_sdk::WaypointNavigationAction> WaypointNavigationActionServer;

    DroneTaskActionServer* drone_task_action_server;
    LocalPositionNavigationActionServer* local_position_navigation_action_server;
    GlobalPositionNavigationActionServer* global_position_navigation_action_server;
    WaypointNavigationActionServer* waypoint_navigation_action_server;

    dji_sdk::DroneTaskFeedback drone_task_feedback;
    dji_sdk::DroneTaskResult drone_task_result; 
    dji_sdk::LocalPositionNavigationFeedback local_position_navigation_feedback; 
    dji_sdk::LocalPositionNavigationResult local_position_navigation_result; 
    dji_sdk::GlobalPositionNavigationFeedback global_position_navigation_feedback;
    dji_sdk::GlobalPositionNavigationResult global_position_navigation_result; 
    dji_sdk::WaypointNavigationFeedback waypoint_navigation_feedback;
    dji_sdk::WaypointNavigationResult waypoint_navigation_result;

    bool drone_task_action_callback(const dji_sdk::DroneTaskGoalConstPtr& goal);
    bool local_position_navigation_action_callback(const dji_sdk::LocalPositionNavigationGoalConstPtr& goal);
    bool global_position_navigation_action_callback(const dji_sdk::GlobalPositionNavigationGoalConstPtr& goal);
    bool waypoint_navigation_action_callback(const dji_sdk::WaypointNavigationGoalConstPtr& goal);

    void init_actions(ros::NodeHandle& nh)
    {
        drone_task_action_server = new DroneTaskActionServer(nh, 
            "dji_sdk/drone_task_action", 
            boost::bind(&DJISDKNode::drone_task_action_callback, this, _1), false);
        drone_task_action_server->start();

        local_position_navigation_action_server = new LocalPositionNavigationActionServer(nh, 
            "dji_sdk/local_position_navigation_action", 
            boost::bind(&DJISDKNode::local_position_navigation_action_callback, this, _1), false);
        local_position_navigation_action_server->start();

        global_position_navigation_action_server = new GlobalPositionNavigationActionServer(nh, 
            "dji_sdk/global_position_navigation_action", 
            boost::bind(&DJISDKNode::global_position_navigation_action_callback, this, _1), false );
        global_position_navigation_action_server->start();

        waypoint_navigation_action_server = new WaypointNavigationActionServer(nh, 
            "dji_sdk/waypoint_navigation_action", 
            boost::bind(&DJISDKNode::waypoint_navigation_action_callback, this, _1), false);
        waypoint_navigation_action_server->start();
    }

public:
    DJISDKNode(ros::NodeHandle& nh, ros::NodeHandle& nh_private);

private:
    int init_parameters_and_activate(ros::NodeHandle& nh_private);
    void broadcast_callback();

    bool process_waypoint(dji_sdk::Waypoint new_waypoint);

    void gps_convert_ned(float &ned_x, float &ned_y,
            double gps_t_lon, double gps_t_lat,
            double gps_r_lon, double gps_r_lat);

    dji_sdk::LocalPosition gps_convert_ned(dji_sdk::GlobalPosition loc);
};

#endif
