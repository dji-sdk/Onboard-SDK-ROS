# Parameters for aircraft control actuator. It is valid only when the
# ``DJIWaypointV2Action_DJIWaypointV2Actuator_type`` is
# ``DJIWaypointV2MissionV2_DJIWaypointV2ActionActuatorType_AircraftControl``.
#contant for DJIWaypointV2ActionActuatorAircraftControlOperationType
# Rotates the aircraft's yaw.
uint8 DJIWaypointV2ActionActuatorAircraftControlOperationTypeRotateYaw = 1
# Keeps the aircraft stop flying or start flying.
uint8 DJIWaypointV2ActionActuatorAircraftControlOperationTypeFlyingControl = 2
# Unknown
uint8 DJIWaypointV2ActionActuatorAircraftControlOperationTypeUnknown = 255

uint8 actuatorIndex   # The index of actuator. It is valid when the diagnostics is related
                      # to camera or gimbal and the connected product has multiple gimbals and cameras.
uint16 DJIWaypointV2ActionActuatorAircraftControlOperationType 
WaypointV2AircraftControlActuatorFlying waypointV2AircraftControlActuatorFlying
WaypointV2AircraftControlActuatorRotateHeading waypointV2AircraftControlActuatorRotateHeading