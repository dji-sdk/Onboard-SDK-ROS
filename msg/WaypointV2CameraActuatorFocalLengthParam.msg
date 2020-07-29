# This class defines a camera focal length operation for  ``DJIWaypointV2Action_DJIWaypointV2CameraActuatorParam``.
# Focal length of zoom lens. Valid range is [``DJICamera_DJICameraOpticalZoomSpec_minFocalLength``,
# ``DJICamera_DJICameraOpticalZoomSpec_minFocalLength``] and must be a multiple of
# ``DJICamera_DJICameraOpticalZoomSpec_focalLengthStep``.
#  Only support by those camera ``DJICamera_CameraSettings_isOpticalZoomSupported`` return ``TRUE``.
  uint16 focalLength

  uint8 retryTimes = 1