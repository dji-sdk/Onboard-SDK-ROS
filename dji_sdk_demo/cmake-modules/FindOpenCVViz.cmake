# Once done this will define
#
#  FOUND_OPENCV_VIZ - system has advanced-sensing

if (NOT FOUND_OPENCV_VIZ)

    find_path(OPENCV_VIZ
            NAMES
            viz3d.hpp
            PATHS
            /usr/include/opencv2/viz
            /usr/local/include/opencv2/viz
            /opt/local/include/opencv2/viz
            /sw/include/opencv2/viz
            PATH_SUFFIXES
            opencv2
            )

    if(NOT OPENCV_VIZ STREQUAL OPENCV_VIZ-NOTFOUND)
        message(STATUS "Found viz3d in OpenCV, will use it to visualize point cloud")
        set(FOUND_OPENCV_VIZ TRUE)
    else()
        message(STATUS "Did not find viz3d in OpenCV")
    endif()

endif (NOT FOUND_OPENCV_VIZ)