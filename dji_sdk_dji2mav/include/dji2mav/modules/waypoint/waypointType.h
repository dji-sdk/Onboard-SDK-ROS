/*****************************************************************************
 * @Brief     An struct with enum type for a waypoint inside the list
 * @Version   0.2.1
 * @Author    Chris Liu
 * @Created   2015/11/23
 * @Modified  2015/11/23
 *****************************************************************************/

#ifndef _DJI2MAV_WAYPOINTTYPE_H_
#define _DJI2MAV_WAYPOINTTYPE_H_


enum WaypointCmd;

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

enum WaypointCmd {
    none,
    takeoff,
    land,
    gohome,
    waypoint
};


#endif

