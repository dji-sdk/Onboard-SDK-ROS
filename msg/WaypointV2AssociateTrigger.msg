#contant for actionAssociatedType
uint8 DJIWaypointV2TriggerAssociatedTimingTypeSimultaneously = 1  # The trigger starts simultaneously with the trigger that is associated.
uint8 DJIWaypointV2TriggerAssociatedTimingTypeAfterFinised = 2    # The trigger starts after the trigger associated has finished.
uint8 DJIWaypointV2TriggerAssociatedTimingTypeUnknown = 255       # Unkown timing type.

uint8 actionAssociatedType  # The type of assciate trigger.
uint8 waitingTime           # Waiting time in seconds after ActionTrigger starts.
uint16 actionIdAssociated # Associated action ID.