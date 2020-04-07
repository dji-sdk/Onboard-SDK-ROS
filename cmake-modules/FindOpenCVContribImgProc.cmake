# Once done this will define
#
#  FOUND_OPENCV_CONTRIB_IMG_PROC - system has opencv contrib img proc module
#  This boolean is currently not used in the system.
#  It's nice to have and not required.

if (NOT FOUND_OPENCV_CONTRIB_IMG_PROC)

    find_path(OPENCV_CONTRIB_IMG_PROC
            NAMES
            disparity_filter.hpp
            PATHS
            /usr/include/opencv2/ximgproc
            /usr/local/include/opencv2/ximgproc
            /opt/local/include/opencv2/ximgproc
            /sw/include/opencv2/ximgproc
            PATH_SUFFIXES
            ximgproc
            )

    if(NOT OPENCV_CONTRIB_IMG_PROC STREQUAL OPENCV_CONTRIB_IMG_PROC-NOTFOUND)
        message(STATUS "Found ximgproc module in OpenCV, will use it to filter disparity map in depth perception sample")
        set(FOUND_OPENCV_CONTRIB_IMG_PROC TRUE)
        add_definitions(-DUSE_OPEN_CV_CONTRIB)
    endif()

endif (NOT FOUND_OPENCV_CONTRIB_IMG_PROC)