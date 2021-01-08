FILE(GLOB_RECURSE HEADERS
     ../../include/*/*.h*
     ../../include/*.h*
     )

find_path(OPENCV_CONTRIB_IMG_PROC
        NAMES
        disparity_filter.hpp
        PATHS
        /usr/include/opencv2/ximgproc
        /usr/local/include/opencv2/ximgproc
        /opt/local/include/opencv2/ximgproc
        /sw/include/opencv2/ximgproc
        PATH_SUFFIXES
        opencv2/ximgproc
        )

find_path(OPENCV_VIZ
        NAMES
        viz3d.hpp
        PATHS
        /usr/include/opencv2/viz
        /usr/local/include/opencv2/viz
        /opt/local/include/opencv2/viz
        /sw/include/opencv2/viz
        PATH_SUFFIXES
        opencv2/viz
        )


# Try to see if user has OpenCV installed
# If there's version 3.3.0+, depth perception samples will be compiled
# If it's not version 3.3.0+, the sample will not be compiled
find_package( OpenCV 3.3.0 QUIET )
if (OpenCV_FOUND)
    message( "\n${PROJECT_NAME}...")
    message( STATUS "Found OpenCV ${OpenCV_VERSION} installed in the system, will use it for depth perception sample")
    message( STATUS " - Includes: ${OpenCV_INCLUDE_DIRS}")
    message( STATUS " - Libraries: ${OpenCV_LIBRARIES}")
    add_definitions(-DOPEN_CV_INSTALLED)
    set(OPEN_CV_3_3_0_INSTALLED 1 CACHE INTERNAL "OPEN_CV_3_3_0_INSTALLED")

    if(NOT OPENCV_CONTRIB_IMG_PROC STREQUAL OPENCV_CONTRIB_IMG_PROC-NOTFOUND)
        message(STATUS "Found ximgproc module in OpenCV, will use it to filter disparity map in depth perception sample")
        add_definitions(-DUSE_OPEN_CV_CONTRIB)
        set(ENABLE_OPEN_CV_CONTRIB 1 CACHE INTERNAL "ENABLE_OPEN_CV_CONTRIB")
    else()
        message(STATUS "Did not find ximgproc in OpenCV")
    endif()

    if(NOT OPENCV_VIZ STREQUAL OPENCV_VIZ-NOTFOUND)
        message(STATUS "Found viz3d in OpenCV, will use it to visualize point cloud")
        set(FOUND_OPENCV_VIZ TRUE CACHE INTERNAL "FOUND_OPENCV_VIZ")
    else()
        message(STATUS "Did not find viz3d in OpenCV")
    endif()
else()
    find_package( OpenCV QUIET )
    if (OpenCV_FOUND)
        message(STATUS "Found OpenCV ${OpenCV_VERSION} installed in the system, will use it to display image in AdvancedSensing APIs")
        message(STATUS " - Includes: ${OpenCV_INCLUDE_DIRS}")
        message( STATUS " - Libraries: ${OpenCV_LIBRARIES}")
        add_definitions(-DOPEN_CV_INSTALLED)
        set(ENABLE_OPEN_CV_INSTALLED 1 CACHE INTERNAL "ENABLE_OPEN_CV_INSTALLED")
    else()
        message(STATUS "Did not find OpenCV in the system, image data is inside RecvContainer as raw data")
    endif()
endif()

find_package(CUDA QUIET)
if(CUDA_FOUND)
    message(STATUS "Found ${CUDA_VERSION} CUDA installed in the system, will use it for depth perception sample")
    message(STATUS " - Includes: ${CUDA_INCLUDE_DIRS}")
    add_definitions(-DUSE_GPU)
    set(ENABLE_GPU 1 CACHE INTERNAL "ENABLE_GPU")
elseif()
    message(STATUS "Did not find CUDA in the system")
endif()

find_package(darknet_ros QUIET)
if(darknet_ros_FOUND)
    message(STATUS "Found darknet_ros package, will use it for object depth perception sample.")
    add_definitions(-DUSE_DARKNET_ROS)
    set(ENABLE_DARKNET_ROS 1 CACHE INTERNAL "ENABLE_DARKNET_ROS")
endif()

if (OPEN_CV_3_3_0_INSTALLED AND FOUND_OPENCV_VIZ)
    FILE(GLOB SOURCES *.cc *.cpp *.c *.cxx stereo_utility/*cpp)
    include_directories(../../../include/dji_osdk_ros/stereo_utility)
    else()
    FILE(GLOB SOURCES *.cc *.cpp *.c *.cxx)
endif()

ADD_LIBRARY(${PROJECT_NAME} SHARED ${HEADERS} ${SOURCES})
MESSAGE(STATUS "PROJECT_NAME: ${PROJECT_NAME}")
MESSAGE(STATUS "DJIOSDK_LIB: ${DJIOSDK_LIBRARIES}")
TARGET_LINK_LIBRARIES(${PROJECT_NAME}
                      ${catkin_LIBRARIES}
		      dji-linker.a
		      djiosdk-core.a
                      )
SET(ADVANCED_SENSING)
SET(WAYPT2_CORE)
	TARGET_LINK_LIBRARIES(${PROJECT_NAME}
                      ${catkin_LIBRARIES}
		      advanced-sensing.a
		      ${LIBUSB_1_LIBRARIES}
		      ${FFMPEG_LIBRARIES}
                      )

add_dependencies(${PROJECT_NAME} dji_osdk_ros_generate_messages_cpp)

target_include_directories(${PROJECT_NAME} PRIVATE ${OpenCV_INCLUDE_DIRS})

if (OpenCV_FOUND)
	target_link_libraries(${PROJECT_NAME} ${OpenCV_LIBS})
endif ()