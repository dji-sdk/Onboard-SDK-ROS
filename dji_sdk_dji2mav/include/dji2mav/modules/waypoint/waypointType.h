/*****************************************************************************
 * @Brief     An struct with enum type for a waypoint inside the list
 * @Version   1.1
 * @Author    Chris Liu
 * @Created   2015/11/23
 * @Modified  2015/11/23
 *****************************************************************************/

#ifndef _DJI2MAV_WAYPOINTTYPE_H_
#define _DJI2MAV_WAYPOINTTYPE_H_


enum WaypointCmd;

struct WaypointType {
    WaypointCmd cmd;
    // extended by the mavlink. It is set to be param1 and param4
    float staytime;
    float heading;
    // limited by the mavlink. Only float32 is sent by the protocol
    float x;
    float y;
    float z;
};

enum WaypointCmd {
    takeoff,
    land,
    gohome,
    waypoint
};


#endif

