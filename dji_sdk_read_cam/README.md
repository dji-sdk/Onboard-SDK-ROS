##DJI Onboard SDK ROS Package for Video Decoding on Manifold

This video-decoding package is a specifically desiged package for Manifold to read the X3 video stream.

###How to use
1. Install the necessary library: refer [here](https://github.com/dji-sdk/manifold_cam)
2. Delete the `CATKIN_IGNORE` file inside package and `catkin_make`.
2. Modify the `manifold_cam.launch` in `launch` folder, set `gray_or_rgb` to 0 if the gray video stream is needed, or to 1 if you prefer the RGB format.
3. `sudo -s` first, then `roslaunch dji_sdk_read_cam manifold_cam.launch`
4. The image will be published into topic `/dji_sdk/image_raw`


###Note:
1. This package is specially designed for Manifold.
2. The RC controller must be connected to Matrice 100 in order to get the video stream.
3. The DJI Go has a power-saving strategy, users should either enter the camera view or do not run DJI GO at first in order to get the video stream. The video will be freezed if uses stay in the main screen of DJI GO. 
3. This package does not provice video transparent transmission. You cannot see the video on DJI Go while running this package.


The default video stream is in RGB format.
![image](../dji_sdk_doc/readcam_nv.png)

Comment the `#define RGB` in `nv_cam.cpp` if you prefer the grayscale.
![image](../dji_sdk_doc/readcam.png)


	Note: frame size is 640x480, no matter RGB or Grayscale
