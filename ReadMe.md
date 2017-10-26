# DJI Onboard SDK ROS 3.3

## Latest Update

The latest update on Sep 8 2017 brought back the local position topic in Cartesian frame (ENU) and addressed several issues reported on github. The `dji_sdk/SetLocalPosRef` service and the `dji_sdk/local_position` topic were added.

## Quick Start Guide 

This repository contains the Onboard SDK ROS wrapper and demos. The ROS package requires Onboard SDK (djiosdk-core) to be installed to your system prior to running it. For detailed setup instructions, please follow the documentation [here](http://developer.dji.com/onboard-sdk/documentation/sample-doc/sample-setup.html#ros-oes). 

The ROS package is a full rewrite of DJI Onboard SDK ROS, with changes in APIs, reference frames, communication mechanisms etc. We encourage you to take a look at the documentation for full details. 

ROS Wiki can be found [here](http://wiki.ros.org/dji_sdk). Please be sure to read the [release notes](https://developer.dji.com/onboard-sdk/documentation/appendix/releaseNotes.html).

DJI Onboard SDK ROS 3.3 was developed to support N3/A3 FW 1.7.5 and above, and is backword compatible with M100 FW 1.3.1.

## Firmware Compatibility

| Aircraft/FC       | Firmware Package Version | Flight Controller Version | OSDK-ROS Branch            | Notes                                                           |
|-------------------|--------------------------|---------------------------|----------------------------|-----------------------------------------------------------------|
| **A3/A3 Pro**     | **1.7.1.5**              | **3.2.36.8**              | **OSDK-ROS 3.3 (Current)** |                                                                 |
|                   | 1.7.0.5                  | 3.2.15.50                 | OSDK-ROS 3.2               |                                                                 |
|                   | 1.7.0.0                  | 3.2.15.37                 | OSDK-ROS 3.2               |                                                                 |
|                   |                          |                           |                            |                                                                 |
| **N3**            | **1.7.1.5**              | **3.2.36.8**              | **OSDK-ROS 3.3 (Current)** |                                                                 |
|                   | 1.7.0.0                  | 3.2.15.37                 | OSDK-ROS 3.2               |                                                                 |
|                   |                          |                           |                            |                                                                 |
| **M600/M600 Pro** | **1.0.1.65**             | **3.2.41.9**              | **OSDK 3.3 (Current)**     | Released 10/24/17                                               |
|                   | *1.0.1.60*               | *3.2.41.5*                | *Not Supported*            | Firmware pulled due to instability  				  |
|                   | **1.0.1.20**             | **3.2.15.62**             | **OSDK-ROS 3.3 (Current)** | 3.3 backward compatiblity released 10/18/17                     |
|                   | 1.0.0.80                 | 3.2.15.00                 | OSDK-ROS 3.2               |                                                                 |
|                   |                          |                           |                            |                                                                 |
| **M100**          | 1.3.1.0                  | 3.1.10.0                  | **OSDK-ROS 3.3 (Current)** |                                                                 |
|                   |                          |                           | and OSDK-ROS 3.2           |                                                                 |



## Notes on differences between M100 and A3/N3 setup

Onboard SDK ROS is backward compatible with M100. However, due to the limitations of the flight controller of M100, some new features such as hardware sync, MFIO, on demand telemetry data (subscription) are only supported by A3/N3, and some settings for M100 are different from those for A3/N3.

1. The DJI Assistant 2 for M100 and for A3/N3 are slighly **different**. Please download DJI Assistant 2 from the corresponding product webpage.

2. The DJI SDK ROS package requires **different baud rate** for M100 and A3/N3. For M100, set the baud rate to 230400 in DJI Assistant 2's "SDK" tab, and the sdk.launch file; while for A3/N3, use 921600.

3. For M100, on the right side of the "SDK setting" tab of DJI Assistant 2, set the Data Type of ACC and GYRO to "Raw Data", and ALTI to "Data Fusion". The reason is that the raw data of acc and gyro are part of the `/dji_sdk/imu` message.

4. The `flight_status` enums for M100 and A3/N3 are different. See `dji_sdk.h` for details and `demo_flight_control` for examples.

5. Some topics  are only available on A3/N3: `display_mode`, `angular_velocity_fused`, `acceleration_ground_fused`, `trigger_time`. 

6. The imu topic is published at 400Hz on A3/N3, and at 100Hz on M100.

7. Some services are only available on A3/N3: `mfio_config`, `mfio_set_value`, `set_hardsyc`

## Support

You can get support from DJI and the community with the following methods:

- Report issue on github
- Email to dev@dji.com
- [**DJI Forum**](http://forum.dev.dji.com/en)




