# DJI Onboard SDK ROS 3.8.1

## Latest Update

OSDK-ROS 3.8.1 was released on 4 June 2019. This release adds support of Onboard-Payload SDK communication and time sync function. Additionally, the dependency of djiosdk-core is auto-update in the release. Please see the [release notes](https://developer.dji.com/onboard-sdk/documentation/appendix/releaseNotes.html) and [ROS sample setup](https://developer.dji.com/onboard-sdk/documentation/development-workflow/sample-setup.html#ros-onboard-computer) for more information.

## Quick Start Guide 

This repository contains the Onboard SDK ROS wrapper and demos. The ROS package requires Onboard SDK (djiosdk-core) to be installed to your system prior to running it. For detailed setup instructions, please follow the documentation [here](https://developer.dji.com/onboard-sdk/documentation/development-workflow/sample-setup.html#ros-onboard-computer). 

We encourage you to take a look at the documentation for full details. 

ROS Wiki can be found [here](http://wiki.ros.org/dji_sdk). Please be sure to read the [release notes](https://developer.dji.com/onboard-sdk/documentation/appendix/releaseNotes.html).

## Firmware Compatibility

This chart shows the latest firmware that were available and are supported at the time of 3.7 release.

| Aircraft/FC           | Firmware Package Version | Flight Controller Version | OSDK Branch            | Notes                                                                 |
|-----------------------|--------------------------|---------------------------|------------------------|-----------------------------------------------------------------------|
| **M210/M210 RTK V2**  | **1.0.0450**             | **3.4.3.31**              | **OSDK 3.8.1**         |                                                                       |
|                       |                          |                           |                        |                                                                       |
| **M210/M210 RTK**     | **1.2.0440**             | **3.3.10.12**             | **OSDK 3.8.1**         |                                                                       |
|                       |                          |                           |                        |                                                                       |
| **M600/M600 Pro**     | **1.0.1.66**             | **3.2.41.13**             | **OSDK 3.8.1**         |                                                                       |
|                       |                          |                           |                        |                                                                       |
| **A3/A3 Pro**         | **1.7.6.0**              | **3.3.8.39**              | **OSDK 3.8.1**         |                                                                       |
|                       |                          |                           |                        |                                                                       |
| **N3**                | **1.7.6.0**              | **3.3.8.39**              | **OSDK 3.8.1**         |                                                                       |
|                       |                          |                           |                        |                                                                       |
| **M100**              | 1.3.1.82                 | **3.1.10.0**              | **OSDK 3.8.1**         |                                                                       |

## Notes on differences between M100 and A3/N3/M600/M210 setup

Onboard SDK ROS is backward compatible with M100. However, due to the limitations of the flight controller of M100, some new features such as hardware sync, MFIO, on demand telemetry data (subscription) are only supported by A3/N3, and some settings for M100 are different from those for A3/N3.

1. The DJI Assistant 2 for M100 and for A3/N3 are slighly **different**. Please download DJI Assistant 2 from the corresponding product webpage.

2. The DJI SDK ROS package requires **different baud rate** for M100 and A3/N3. For M100, set the baud rate to 230400 in DJI Assistant 2's "SDK" tab, and the sdk.launch file; while for **A3/N3/M600/M210, use 921600**.

3. For M100, on the right side of the "SDK setting" tab of DJI Assistant 2, set the Data Type of ACC and GYRO to "Raw Data", and ALTI to "Data Fusion". The reason is that the raw data of acc and gyro are part of the `/dji_sdk/imu` message.

4. The `flight_status` enums for M100 and A3/N3 are different. See `dji_sdk.h` for details and `demo_flight_control` for examples.

5. Some topics  are only available on A3/N3: `display_mode`, `angular_velocity_fused`, `acceleration_ground_fused`, `trigger_time`. 

6. The imu topic is published at 400Hz on A3/N3, and at 100Hz on M100.

7. Some services are only available on A3/N3: `mfio_config`, `mfio_set_value`, `set_hardsyc`

## Support

You can get support from DJI and the community with the following methods:

- **Email to dev@dji.com**
- Report issue on github



