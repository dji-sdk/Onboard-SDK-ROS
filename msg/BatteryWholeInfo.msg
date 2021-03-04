uint16  remainFlyTime             # remain fly time(s)
uint16  goHomeNeedTime            # Time required for the gohome flight (s)
uint16  landNeedTime              # Time required for the land flight (s).max value:100*/
uint16  goHomeNeedCapacity        # Capacity required for the gohome flight (%).max value:100*/
uint16  landNeedCapacity          # Capacity required for the land flight (%).max value:100*/
float32 safeFlyRadius             # Safe flight area radius (m)*/
float32 capacityConsumeSpeed      # (mAh/sec)*/
BatteryState batteryState
uint8   goHomeCountDownState      # Countdown status of smart battery gohome
                                   # 0/2:not in gohome state
                                   # 1  :in gohome state
                                   #
uint8   gohomeCountDownvalue      # uint:s.max value:10
uint16  voltage                   # mv
uint8   batteryCapacityPercentage # uint:%.max value:100
uint8   lowBatteryAlarmThreshold
uint8   lowBatteryAlarmEnable
uint8   seriousLowBatteryAlarmThreshold
uint8   seriousLowBatteryAlarmEnable  