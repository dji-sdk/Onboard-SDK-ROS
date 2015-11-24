/*****************************************************************************
 * @Brief     ROS-depended and dji2mav-depended. An adapter for waypoint
 * @Version   0.2.1
 * @Author    Chris Liu
 * @Created   2015/11/24
 * @Modified  2015/11/24
 *****************************************************************************/

#ifndef _DJI2MAV_WAYPOINTTYPE_H_
#define _DJI2MAV_WAYPOINTTYPE_H_


enum WaypointCmd {
    none = 0,
    waypoint = 16,
    gohome = 20,
    land = 21,
    takeoff = 22
};

struct WaypointType {
    WaypointCmd cmd;
    // limited by the mavlink. Only float32 is sent by the protocol
    float lat;
    float lon;
    float alt;
    // extended by the mavlink. It is set to be param1 and param4
    float heading;
    float staytime;
};


#endif
