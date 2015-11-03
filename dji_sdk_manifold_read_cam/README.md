##DJI Onboard SDK ROS Package for Video Decoding on Manifold

This package is a specified video decoding package for Manifold.

###How to use
1. Install the necessary library: it should be already there, try to run the project in the `demo` folder of your Manifold home directory. If not, please run the `install_lib.sh`
2. Delete the `CATKIN_IGNORE` file inside package and `catkin_make`.
3. `rosrun dji_sdk_manifold_read_cam dji_sdk_manifold_read_cam`
4. The image will be published into topic `/dji_sdk/image_raw`


###Note:
1. This package is specially designed for Manifold.
2. The RC controller must be connected to Matrice 100 in order to get the video stream.
3. This package does not provice video transparent transmission. You cannot see the video on DJI Go while running this package.
4. This package uses FFMPENG for video decoding. while the _nv one uses hardware decoding method.
5. The image format is RGB, while the _nv one publishes Grayscale image.

![image](../dji_sdk_doc/readcam_nv.png)