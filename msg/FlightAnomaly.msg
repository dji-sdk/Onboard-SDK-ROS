# Here is an example how to use FlightAnomaly msg:
#
# void flightAnomalyCallback(const dji_osdk_ros::FlightAnomaly::ConstPtr & msg)
# {
#   uint32_t flightAnomalyData = msg->data;
#   if(flightAnomalyData)
#   {
#     ROS_INFO("Flight Anomaly Reported by Flight Controller. Here are details:");
#     if(flightAnomalyData && dji_osdk_ros::FlightAnomaly::COMPASS_INSTALLATION_ERROR)
#     {
#         ROS_INFO("COMPASS_INSTALLATION_ERROR");
#     }
#
#     if(flightAnomalyData && dji_osdk_ros::FlightAnomaly::IMU_INSTALLATION_ERROR)
#     {
#         ROS_INFO("IMU_INSTALLATION_ERROR");
#     }
#
#     // etc...
#   }
# }
#
#

# constants for anomaly details
uint32 IMPACT_IN_AIR              = 1     #
uint32 RANDOM_FLY                 = 2     #
uint32 VERTICAL_CONTROL_FAIL      = 4     #
uint32 HORIZONTAL_CONTROL_FAIL    = 8     #
uint32 YAW_CONTROL_FAIL           = 16    #
uint32 AIRCRAFT_IS_FALLING        = 32    #
uint32 STRONG_WIND_LEVEL1         = 64    #
uint32 STRONG_WIND_LEVEL2         = 128   #
uint32 COMPASS_INSTALLATION_ERROR = 256   #
uint32 IMU_INSTALLATION_ERROR     = 512   #
uint32 ESC_TEMPERATURE_HIGH       = 1024  #
uint32 ESC_DISCONNECTED           = 2048  #
uint32 GPS_YAW_ERROR              = 4096  #

#
uint32 data

