# gimbal actuator will be performed when a waypoint mission is executed.
#constant for DJIWaypointV2ActionActuatorGimbalOperationType
# Rotates the gimbal. Only valid when the trigger type is
# ``DJIWaypointV2MissionV2_DJIWaypointV2TriggerAssociatedTimingType_ReachPoint``.
uint8 DJIWaypointV2ActionActuatorGimbalOperationTypeRotateGimbal = 1
# Unknown
uint8 DJIWaypointV2ActionActuatorGimbalOperationTypeUnknown = 255

uint8 DJIWaypointV2ActionActuatorGimbalOperationType
uint16 actuatorIndex   # The index of actuator. It is valid when the diagnostics is related
                      # to camera or gimbal and the connected product has multiple gimbals and cameras.
WaypointV2GimbalActuatorRotationParam waypointV2GimbalActuatorRotationParam # The operation type of gimbal actuator.