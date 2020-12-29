# This class represents an action for ``DJIWaypointV2Mission``. It
# determines how action is performed when a waypoint mission is executed.

# The action will be triggered when action associated executes.
# The parameters should be defined by ``DJIWaypointV2Action_DJIWaypointV2AssociateTriggerParam``.
uint8 DJIWaypointV2ActionTriggerTypeActionAssociated = 2

# The action will be triggered when the aircraft flies from one waypoint to the next.
# The parameters should be defined by ``DJIWaypointV2Action_DJIWaypointV2TrajectoryTriggerParam``.
uint8 DJIWaypointV2ActionTriggerTypeTrajectory = 3

# The action will be triggered when the aircraft flies between two waypoints
# The parameters should be defined by ``DJIWaypointV2Action_DJIWaypointV2IntervalTriggerParam``.
uint8 DJIWaypointV2ActionTriggerTypeInterval = 4
uint8 DJIWaypointV2ActionTriggerTypeSampleReachPoint = 5

# Unknown
uint8 DJIWaypointV2ActionTriggerTypeUnknown = 255

#contant for waypointV2ActuatorTriggerType
# The action will be executed by the camera.
# The parameters should be defined by ``DJIWaypointV2Action_DJIWaypointV2CameraActuatorParam``.
uint8 DJIWaypointV2ActionActuatorTypeCamera = 1

# The action will be executed by the gimbal.
# The parameters should be defined by ``DJIWaypointV2Action_DJIWaypointV2GimbalActuatorParam``.
uint8 DJIWaypointV2ActionActuatorTypeGimbal = 2

# The action will executes by control aircraft.
# The parameters should be setting by ``DJIWaypointV2Action_DJIWaypointV2CameraActuatorParam``.
uint8 DJIWaypointV2ActionActuatorTypeAircraftControl = 4

# Unknown actuator type.
uint8 DJIWaypointV2ActionActuatorTypeUnknown = 255

uint16 actionId # The ID of Action.

uint8 waypointV2ActionTriggerType
uint8 waypointV2ACtionActuatorType

# The trigger of action.You can only choose one to config
WaypointV2AssociateTrigger waypointV2AssociateTrigger
WaypointV2IntervalTrigger waypointV2IntervalTrigger
WaypointV2TrajectoryTrigger waypointV2TrajectoryTrigger
WaypointV2SampleReachPointTrigger waypointV2SampleReachPointTrigger

 # The actuator of action.You can only choose one to config
WaypointV2CameraActuator waypointV2CameraActuator
WaypointV2GimbalActuator waypointV2GimbalActuator
WaypointV2AircraftControlActuator waypointV2AircraftControlActuator