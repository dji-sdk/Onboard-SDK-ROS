uint8 cellBreak                  # 0:normal;other:Undervoltage core index(0x01-0x1F)
uint8 selfCheckError             # enum-type: DJISmartBatterySelfCheck
uint8 batteryClosedReason        # enum-type: DJI_BETTERY_CLOSED_REASON
uint8 batSOHState                # enum-type: DJISmartBatterySohState*/
uint8 maxCycleLimit              # APP:cycle_limit*10*/
uint8 batteryCommunicationAbnormal
uint8 hasCellBreak
uint8 heatState                  # enum-type: DJISmartBatteryHeatState