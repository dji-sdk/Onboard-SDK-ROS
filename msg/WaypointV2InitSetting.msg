# @brief Waypoint V2 Mission Initialization settings
# @details This is one of the few structs in the OSDK codebase that
# is used in both a sending and a receiving API.

#constant for finishedAction
uint8 DJIWaypointV2MissionFinishedNoAction = 0  # No further action will be taken.
                                                # The aircraft can be controlled by the remote controller.
uint8 DJIWaypointV2MissionFinishedGoHome = 1    # Goes home when the mission is finished. The aircraft will
                                                # land directly if it is within 20 meters away from the home point.
uint8 DJIWaypointV2MissionFinishedAutoLanding = 2  # The aircraft will land automatically at the last waypoint.
uint8 DJIWaypointV2MissionFinishedGoToFirstWaypoint = 3  # The aircraft will go back to the first waypoint and hover.
uint8 DJIWaypointV2MissionFinishedContinueUntilStop = 4  #  When the aircraft reaches its final waypoint, it will hover without ending the
                                                         # mission. The joystick  can still be used to pull the aircraft back along its
                                                         # previous waypoints. The only way this mission  can end is if stopMission is
                                                         # called

#constant for gotoFirstWaypintMode
uint8 DJIWaypointV2MissionGotoFirstWaypointModeSafely = 0 # Go to the waypoint safely. The aircraft will rise to the same altitude of the
                                                          # waypoint if the current  altitude is lower then the waypoint altitude. It then
                                                          # goes to the waypoint coordinate from the current  altitude, and proceeds to the
                                                          # altitude of the waypoint.
uint8 DJIWaypointV2MissionGotoFirstWaypointModePointToPoint = 1 # Go to the waypoint from the current aircraft point to the waypoint directly.


uint32 missionID    # The Mission ID. Use to distinguish　different mission
uint16 missTotalLen # The Mission waypoint total length, could not exceed 65535
uint8 repeatTimes   # Mission execution can be repeated more than once. A value of 0 means the mission
                    # only executes once, and does not repeat. A value of 1 means the mission will
                    # execute a total of two times.
uint8 finishedAction # Action the aircraft will take when the waypoint mission is complete.

# While the aircraft is travelling between waypoints, you can offset its speed by
# using the throttle joystick  on the remote controller. `maxFlightSpeed` is this
# offset when the joystick is pushed  to maximum deflection. For example, If
# maxFlightSpeed is 10 m/s, then pushing the throttle joystick all the  way up
# will add 10 m/s to the aircraft speed, while pushing down will subtract 10 m/s
# from the aircraft  speed. If the remote controller stick is not at maximum
# deflection, then the offset speed will be  interpolated between [0,
# `maxFlightSpeed`] with a resolution of 1000 steps. If the  offset speed is
# negative, then the aircraft will fly backwards to previous waypoints. When it
# reaches the  first waypoint, it will then hover in place until a positive speed
# is  applied. `maxFlightSpeed` has a range of [2,15] m/s.
# unit: m/s
float32 maxFlightSpeed

# The base automatic speed of the aircraft as it moves between waypoints with
# range [-15, 15] m/s. The  aircraft's actual speed is a combination of the base
# automatic speed, and the speed control given by  the throttle joystick on the
# remote controller. If `autoFlightSpeed` >0: Actual  speed is `autoFlightSpeed` +
# Joystick Speed (with combined max  of `maxFlightSpeed`) If `autoFlightSpeed` =0:
# Actual speed is  controlled only by the remote controller joystick. If
# `autoFlightSpeed` <0 and the  aircraft is at the first waypoint, the aircraft
# will hover in place until the speed is made positive by  the remote controller
# joystick.
# unit: m/s
float32 autoFlightSpeed

uint8 exitMissionOnRCSignalLost    # Determines whether the mission should stop when connection between the  aircraft
                                   # and remote controller is lost. Default is `NO`.

uint8 gotoFirstWaypointMode        # Defines how the aircraft will go to the first waypoint from its current
                                   # position. Default  is ``DJIWaypointV2MissionV2_DJIWaypointV2MissionGotoWaypointMode_Safely``.

WaypointV2[] mission