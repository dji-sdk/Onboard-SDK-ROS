uint8 isFakeSingleBatteryMode
uint8 isSingleBatteryMode
uint8 batteryNotReady  # The battery is not ready (the battery has not communicated just after being powered on,
                       # or has not passed the first battery certification).
uint8 voltageNotSafety # Generally caused by low temperature, the battery has electricity,
                       # but the battery voltage is too low.
uint8 veryLowVoltageAlarm
uint8 LowVoltageAlarm
uint8 seriousLowCapacityAlarm
uint8 LowCapacityAlarm