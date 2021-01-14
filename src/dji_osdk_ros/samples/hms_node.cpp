/** @file hms_node.cpp
 *  @version 4.1
 *  @date Dec 2020
 *
 *  @brief sample node of hms.
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
#include <dji_osdk_ros/GetHMSData.h>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "hms_node");
    ros::NodeHandle nh;

    ros::Duration(1).sleep();
    ros::AsyncSpinner spinner(1);
    spinner.start();

    auto get_hms_data_client = nh.serviceClient<dji_osdk_ros::GetHMSData>("get_hms_data");
    dji_osdk_ros::GetHMSData get_hms_data;
    get_hms_data.request.enable = true;
    ros::Rate rate(5);

    while(ros::ok())
    {
        get_hms_data_client.call(get_hms_data);
        // ROS_INFO("%d", get_hms_data.response.result);

        for (int i = 0; i < get_hms_data.response.errList.size(); i++)
        {
            ROS_INFO("hmsErrListNum: %d, timeStamp:%ld, alarm_id: 0x%08x, sensorIndex: %d, level: %d",
                      i+1, get_hms_data.response.timeStamp,
                         get_hms_data.response.errList[i].alarmID,
                         get_hms_data.response.errList[i].sensorIndex,
                         get_hms_data.response.errList[i].reportLevel);
            if (get_hms_data.response.deviceIndex != 0xff)
            {
                ROS_INFO("%d Camera/Gimbal has trouble!", get_hms_data.response.deviceIndex);
            }
        }

        rate.sleep();
    }

    ROS_INFO_STREAM("Finished. Press CTRL-C to terminate the node");

    ros::waitForShutdown();
    return 0;
}


