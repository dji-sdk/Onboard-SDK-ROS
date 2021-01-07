/** @file battery_node.cpp
 *  @version 4.1
 *  @date Dec 2020
 *
 *  @brief sample node of battery.
 *
 *  @Copyright (c) 2020 DJI
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 *
 */

#include <ros/ros.h>
#include <dji_osdk_ros/dji_vehicle_node.h>

const uint8_t MAX_TIME_COUNT = 50;
int main(int argc, char** argv)
{
    ros::init(argc, argv, "battery_node");
    ros::NodeHandle nh;
    uint8_t time_count = 0;

    auto get_whole_battery_info = nh.serviceClient<dji_osdk_ros::GetWholeBatteryInfo>("get_whole_battery_info");
    auto get_single_battery_dynamic_info = nh.serviceClient<dji_osdk_ros::GetSingleBatteryDynamicInfo>("get_single_battery_dynamic_info");

    dji_osdk_ros::GetWholeBatteryInfo getWholeBatteryInfo;
    dji_osdk_ros::GetSingleBatteryDynamicInfo firstBatteryDynamicInfo;
    dji_osdk_ros::GetSingleBatteryDynamicInfo secondBatteryDynamicInfo;

    while (time_count < MAX_TIME_COUNT)
    {
      get_whole_battery_info.call(getWholeBatteryInfo);
      ROS_INFO("(It's valid only for M210V2)Whole battery Info:");
      ROS_INFO("(It's valid only for M210V2)batteryCapacityPercentage is %d(%)",getWholeBatteryInfo.response.battery_whole_info.batteryCapacityPercentage);
      ROS_INFO("(It's valid only for M210V2)remainFlyTime is %d(s)",getWholeBatteryInfo.response.battery_whole_info.remainFlyTime);
      ROS_INFO("(It's valid only for M210V2)goHomeNeedTime is %d(s)",getWholeBatteryInfo.response.battery_whole_info.goHomeNeedTime);
      ROS_INFO("(It's valid only for M210V2)landNeedTime is %d(s)",getWholeBatteryInfo.response.battery_whole_info.landNeedTime);
      ROS_INFO("(It's valid only for M210V2)goHomeNeedCapacity is %d(%)",getWholeBatteryInfo.response.battery_whole_info.goHomeNeedCapacity);
      ROS_INFO("(It's valid only for M210V2)landNeedCapacity is %d(%)",getWholeBatteryInfo.response.battery_whole_info.landNeedCapacity);
      ROS_INFO("(It's valid only for M210V2)safeFlyRadius is %f(m)",getWholeBatteryInfo.response.battery_whole_info.safeFlyRadius);
      ROS_INFO("(It's valid only for M210V2)capacityConsumeSpeed is %f(mAh/sec)",getWholeBatteryInfo.response.battery_whole_info.capacityConsumeSpeed);
      ROS_INFO("(It's valid only for M210V2)goHomeCountDownState is %d",getWholeBatteryInfo.response.battery_whole_info.goHomeCountDownState);
      ROS_INFO("(It's valid only for M210V2)gohomeCountDownvalue is %d",getWholeBatteryInfo.response.battery_whole_info.gohomeCountDownvalue);
      ROS_INFO("(It's valid only for M210V2)voltage is %d(mv)",getWholeBatteryInfo.response.battery_whole_info.voltage);
      ROS_INFO("(It's valid only for M210V2)lowBatteryAlarmThreshold is %d(%)",getWholeBatteryInfo.response.battery_whole_info.lowBatteryAlarmThreshold);
      ROS_INFO("(It's valid only for M210V2)lowBatteryAlarmEnable is %d",getWholeBatteryInfo.response.battery_whole_info.lowBatteryAlarmEnable);
      ROS_INFO("(It's valid only for M210V2)seriousLowBatteryAlarmThreshold is %d(%)",getWholeBatteryInfo.response.battery_whole_info.seriousLowBatteryAlarmThreshold);
      ROS_INFO("(It's valid only for M210V2)seriousLowBatteryAlarmEnable is %d",getWholeBatteryInfo.response.battery_whole_info.seriousLowBatteryAlarmEnable);
      ROS_INFO("(It's valid only for M210V2)voltageNotSafety is %d",getWholeBatteryInfo.response.battery_whole_info.batteryState.voltageNotSafety);
      ROS_INFO("(It's valid only for M210V2)veryLowVoltageAlarm is %d",getWholeBatteryInfo.response.battery_whole_info.batteryState.veryLowVoltageAlarm);
      ROS_INFO("(It's valid only for M210V2)LowVoltageAlarm is %d",getWholeBatteryInfo.response.battery_whole_info.batteryState.LowVoltageAlarm);
      ROS_INFO("(It's valid only for M210V2)seriousLowCapacityAlarm is %d",getWholeBatteryInfo.response.battery_whole_info.batteryState.seriousLowCapacityAlarm);
      ROS_INFO("(It's valid only for M210V2)LowCapacityAlarm is %d",getWholeBatteryInfo.response.battery_whole_info.batteryState.LowCapacityAlarm);
      ROS_INFO("---------------------------------------------------");
      ros::Duration(0.2).sleep();
      ROS_INFO("Single Battery Info:");
      ROS_INFO("The First Battery Info:");
      firstBatteryDynamicInfo.request.batteryIndex = dji_osdk_ros::GetSingleBatteryDynamicInfo::Request::first_smart_battery;
      get_single_battery_dynamic_info.call(firstBatteryDynamicInfo);
      ROS_INFO("battery index %d batteryCapacityPercent is %d(%)",firstBatteryDynamicInfo.response.smartBatteryDynamicInfo.batteryIndex,
               firstBatteryDynamicInfo.response.smartBatteryDynamicInfo.batteryCapacityPercent);
      ROS_INFO("battery index %d currentVoltage is %d(mV)",firstBatteryDynamicInfo.response.smartBatteryDynamicInfo.batteryIndex,
               firstBatteryDynamicInfo.response.smartBatteryDynamicInfo.currentVoltage);
      ROS_INFO("battery index %d batteryTemperature is %d",firstBatteryDynamicInfo.response.smartBatteryDynamicInfo.batteryIndex,
              firstBatteryDynamicInfo.response.smartBatteryDynamicInfo.batteryTemperature/10);
      ROS_INFO("The Second Battery Info:");
      secondBatteryDynamicInfo.request.batteryIndex = dji_osdk_ros::GetSingleBatteryDynamicInfo::Request::second_smart_battery;
      get_single_battery_dynamic_info.call(secondBatteryDynamicInfo);
      ROS_INFO("battery index %d batteryCapacityPercent is %d(%)",secondBatteryDynamicInfo.response.smartBatteryDynamicInfo.batteryIndex,
               secondBatteryDynamicInfo.response.smartBatteryDynamicInfo.batteryCapacityPercent);
      ROS_INFO("battery index %d currentVoltage is %d(mV)",secondBatteryDynamicInfo.response.smartBatteryDynamicInfo.batteryIndex,
               secondBatteryDynamicInfo.response.smartBatteryDynamicInfo.currentVoltage);
      ROS_INFO("battery index %d batteryTemperature is %d",secondBatteryDynamicInfo.response.smartBatteryDynamicInfo.batteryIndex,
              secondBatteryDynamicInfo.response.smartBatteryDynamicInfo.batteryTemperature/10);
      ROS_INFO("---------------------------------------------------");

      time_count++;
    }

    ROS_INFO_STREAM("Finished. Press CTRL-C to terminate the node");

    ros::spin();
    return 0;
}
