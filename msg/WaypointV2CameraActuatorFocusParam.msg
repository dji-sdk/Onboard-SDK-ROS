# This class defines a camera focus operation for ``DJIWaypointV2Action_DJIWaypointV2CameraActuatorParam``.
# The lens focus target point. When the focus mode is auto, the target point
# is the focal point. When the focus mode is manual, the target point is the zoom
# out area if the focus assistant is enabled for the manual mode.
#  The range for x and y is from 0.0 to 1.0. The point [0.0, 0.0] represents the top-left angle of the screen.
  
  float32 x # x axis focus point value.range: [0,1]
  float32 y # y axis focus point value.range: [0,1]
  uint8 regionType #focus type:0:point focus,1:rectangle focus
  float32 width #Normalized focus area width(0,1)
  float32 height # Normalized focus area height(0,1)
  uint8 retryTimes = 1