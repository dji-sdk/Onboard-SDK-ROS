# This class defines a camera focus operation for ``DJIWaypointV2Action_DJIWaypointV2CameraActuatorParam``.
# The lens focus target point. When the focus mode is auto, the target point
# is the focal point. When the focus mode is manual, the target point is the zoom
# out area if the focus assistant is enabled for the manual mode.
#  The range for x and y is from 0.0 to 1.0. The point [0.0, 0.0] represents the top-left angle of the screen.
  
  uint16 x # x axis focus point value.range: [0,10000]
  uint16 y # y axis focus point value.range: [0,10000]
  uint8 retryTimes = 1
  uint8 focusDelayTime = 0