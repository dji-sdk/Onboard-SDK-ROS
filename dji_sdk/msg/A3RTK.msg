uint32 date
uint32 time
float64 longitude_RTK
float64 latitude_RTK
float32 height_above_sea_RTK
int32 longitude_single
int32 latitude_single
int32 height_above_sea_single
float32 velocity_north
float32 velocity_east
float32 velocity_ground
int16 yaw #between baseline and south
uint8 satellite_used_RTK
uint8 satellite_used_single
float32 horizontal_dop
float32 position_dop
uint8[6] position_flag #0 sigle point, 1 RTK, 2 fixed direction, 3 reserve
uint16 gps_state
uint16 rtk_updated_flag
