uint8 event
uint32 FCTimestamp
#ID:0x01
uint8 interruptReason #0x00:rc triggered interrupt

#ID:0x02
uint8 recoverProcess #0x00:finished pause recover

#ID:0x03
uint8 finishReason #0x00:finished normally; 0x10:External user trigger ended successfully

#ID:0x10
uint16 waypointIndex

#ID:0x11
uint8 currentMissionExecNum
uint8 finishedAllExecNum  #0:not finished; 1:finished all exec num