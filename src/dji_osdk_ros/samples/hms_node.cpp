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
#include <dji_osdk_ros/SubscribeHMSInf.h>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "hms_node");
    ros::NodeHandle nh;

    auto subscribe_hms_info_client = nh.serviceClient<dji_osdk_ros::SubscribeHMSInf>("/subscribe_hms_info");
    dji_osdk_ros::SubscribeHMSInf subscribe_hms_info;
    subscribe_hms_info.request.enable = true;

    ros::Duration(1).sleep();
    ros::AsyncSpinner spinner(1);
    spinner.start();

    ros::Rate rate(10);
    while(ros::ok())
    {
        subscribe_hms_info_client.call(subscribe_hms_info);

        for (int i = 0; i < subscribe_hms_info.response.errList.size(); i++)
        {
            ROS_INFO("hmsErrListNum: %d, alarm_id: 0x%08x, sensorIndex: %d, level: %d",
                      i, subscribe_hms_info.response.errList[i].alarmID,
                         subscribe_hms_info.response.errList[i].sensorIndex,
                         subscribe_hms_info.response.errList[i].reportLevel);
            if (subscribe_hms_info.response.deviceIndex != 0xff)
            {
                ROS_INFO("%d Camera/Gimbal has trouble!", subscribe_hms_info.response.deviceIndex);
            }
        }

        rate.sleep();
    }

    ROS_INFO_STREAM("Finished. Press CTRL-C to terminate the node");

    ros::waitForShutdown();
    return 0;
}


