# This determines how the camera will be performed when a waypoint mission is executing.
#constant for DJIWaypointV2ActionActuatorCameraOperationType
# Starts to shoot a photo.
uint16 DJIWaypointV2ActionActuatorCameraOperationTypeTakePhoto = 1
# Starts to record a video.
uint16 DJIWaypointV2ActionActuatorCameraOperationTypeStartRecordVideo = 2
# Stops to record a video.
uint16 DJIWaypointV2ActionActuatorCameraOperationTypeStopRecordVideo = 3
# Starts focus.
uint16 DJIWaypointV2ActionActuatorCameraOperationTypeFocus = 4
# Starts focal lenth. Only support those support flocal lenth cameras.
uint16 DJIWaypointV2ActionActuatorCameraOperationTypeFocalLength = 5

uint16 actuatorIndex   # The index of actuator. It is valid when the diagnostics is related
                      # to camera or gimbal and the connected product has multiple gimbals and cameras.

uint16 DJIWaypointV2ActionActuatorCameraOperationType
# you can only choose one to config.
WaypointV2CameraActuatorFocusParam focusParam  # The parameters for camera focus operation. It is valid only when
                                       # ``DJIWaypointV2Action_DJIWaypointV2CameraActuatorParam_operationType`` is
                                       # ``DJIWaypointV2MissionV2_DJIWaypointV2ActionActuatorCameraOperationType_Focus``
WaypointV2CameraActuatorFocalLengthParam zoomParam # The parameters for camera focus length operation. It is valid only when
                                           # ``DJIWaypointV2Action_DJIWaypointV2CameraActuatorParam_operationType`` is
                                           # ``DJIWaypointV2MissionV2_DJIWaypointV2ActionActuatorCameraOperationType_FocalLength``