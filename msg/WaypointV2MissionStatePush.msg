int8   commonDataVersion
uint16 commonDataLen
uint16 curWaypointIndex
uint8  state  #0x0:ground station not start. 0x1:mission prepared. 0x2:enter mission.
               #0x3:execute flying route mission.
               #0x4:pause state. 0x5:enter mission after ending pause.
               #0x6:exit mission.
uint16 velocity #uint:0.01m/s