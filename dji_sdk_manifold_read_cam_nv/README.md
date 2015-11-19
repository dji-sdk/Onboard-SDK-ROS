##DJI Onboard SDK ROS Package for Video Decoding on Manifold

This package is a specified video decoding package for Manifold.

###How to use
1. Install the necessary library: refer [here](https://github.com/dji-sdk/manifold_cam)
2. Delete the `CATKIN_IGNORE` file inside package and `catkin_make`.
3. `rosrun dji_sdk_manifold_read_cam_nv dji_sdk_manifold_read_cam_nv`
4. The image will be published into topic `/dji_sdk/image_raw`


###Note:
1. This package is specially designed for Manifold.
2. The RC controller must be connected to Matrice 100 in order to get the video stream.
3. The DJI Go has a power-saving strategy, users should either enter the camera view or do not run DJI GO at first in order to get the video stream. The video will be freezed if uses stay in the main screen of DJI GO. 
3. This package does not provice video transparent transmission. You cannot see the video on DJI Go while running this package.
4. This package uses hardware decoding method, while the other one uses FFmpeg.
5. The image format is in Grayscale, while the other one publishes RGB image.

![image](../dji_sdk_doc/readcam.png)