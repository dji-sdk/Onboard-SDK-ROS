FILE(GLOB EXAMPLES *.cc *.c *.cpp)

# include_directories(stereo_utility)

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
    set(OPEN_CV_3_3_0_INSTALLED 1)

    if(NOT OPENCV_CONTRIB_IMG_PROC STREQUAL OPENCV_CONTRIB_IMG_PROC-NOTFOUND)
        message(STATUS "Found ximgproc module in OpenCV, will use it to filter disparity map in depth perception sample")
        add_definitions(-DUSE_OPEN_CV_CONTRIB)
    else()
        message(STATUS "Did not find ximgproc in OpenCV")
    endif()

    if(NOT OPENCV_VIZ STREQUAL OPENCV_VIZ-NOTFOUND)
        message(STATUS "Found viz3d in OpenCV, will use it to visualize point cloud")
        set(FOUND_OPENCV_VIZ TRUE)
    else()
        message(STATUS "Did not find viz3d in OpenCV")
    endif()
else()
    find_package( OpenCV QUIET )
    if (OpenCV_FOUND)
        message(STATUS "Found OpenCV ${OpenCV_VERSION} installed in the system, this sample requires 3.3.0+ for better experience")
        message(STATUS " - Includes: ${OpenCV_INCLUDE_DIRS}")
        add_definitions(-DOPEN_CV_INSTALLED)
    else()
        message(STATUS "Did not find OpenCV in the system, image data is inside RecvContainer as raw data")
    endif()
endif()

find_package(CUDA QUIET)
if(CUDA_FOUND)
    message(STATUS "Found ${CUDA_VERSION} CUDA installed in the system, will use it for depth perception sample")
    message(STATUS " - Includes: ${CUDA_INCLUDE_DIRS}")
    add_definitions(-DUSE_GPU)
elseif()
    message(STATUS "Did not find CUDA in the system")
endif()

find_package(darknet_ros QUIET)
if(darknet_ros_FOUND)
    message(STATUS "Found darknet_ros package, will use it for object depth perception demo.")
    add_definitions(-DUSE_DARKNET_ROS)
endif()

add_executable(flight_control_node
               flight_control_node.cpp)
target_link_libraries(flight_control_node
        ${PROJECT_NAME}
        )
add_dependencies(flight_control_node dji_osdk_ros_generate_messages_cpp)

add_executable(gimbal_camera_control_node
               gimbal_camera_control_node.cpp )
target_link_libraries(gimbal_camera_control_node
        ${PROJECT_NAME}
        )
add_dependencies(gimbal_camera_control_node dji_osdk_ros_generate_messages_cpp)

add_executable(mobile_device_node
               mobile_device_node.cpp)
target_link_libraries(mobile_device_node
        ${PROJECT_NAME}
        )
add_dependencies(mobile_device_node dji_osdk_ros_generate_messages_cpp)

add_executable(payload_device_node
               payload_device_node.cpp)

target_link_libraries(payload_device_node
        ${PROJECT_NAME}
        )
add_dependencies(payload_device_node dji_osdk_ros_generate_messages_cpp)

add_executable(telemetry_node telemetry_node.cpp)

target_link_libraries(telemetry_node
        ${PROJECT_NAME}
        )
add_dependencies(telemetry_node dji_osdk_ros_generate_messages_cpp)

add_executable(time_sync_node time_sync_node.cpp)

target_link_libraries(time_sync_node
        ${PROJECT_NAME}
        )
add_dependencies(time_sync_node dji_osdk_ros_generate_messages_cpp)

add_executable(camera_stream_node camera_stream_node.cpp)
target_link_libraries(camera_stream_node
        ${PROJECT_NAME}
        )
add_dependencies(camera_stream_node dji_osdk_ros_generate_messages_cpp)

add_executable(camera_h264_node camera_h264_node.cpp)
target_link_libraries(camera_h264_node
        ${PROJECT_NAME}
        )
add_dependencies(camera_h264_node dji_osdk_ros_generate_messages_cpp)

if (OPEN_CV_3_3_0_INSTALLED AND FOUND_OPENCV_VIZ)
    message(STATUS "Found OpenCV ${OpenCV_VERSION}, Viz3d, and advanced sensing module, depth percpetion demo will be compiled")

    add_executable(stereo_vision_depth_perception_node
                   stereo_vision_depth_perception_node.cpp)
    target_include_directories(${PROJECT_NAME} PRIVATE ${OpenCV_INCLUDE_DIRS})

    if (OpenCV_FOUND)
        target_link_libraries(stereo_vision_depth_perception_node
                ${catkin_LIBRARIES}  
                ${PROJECT_NAME}
                ${OpenCV_LIBRARIES}
                )
    else()
    target_link_libraries(stereo_vision_depth_perception_node
            ${catkin_LIBRARIES}        
            ${PROJECT_NAME}
            )
    endif()
    add_dependencies(stereo_vision_depth_perception_node dji_osdk_ros_generate_messages_cpp)
else()
   message(STATUS "Did not find required libraries, stereo vision depth perception sample will not be compiled.")
endif ()

if (OPEN_CV_3_3_0_INSTALLED AND FOUND_OPENCV_VIZ AND darknet_ros_FOUND)
    message(STATUS "Found OpenCV ${OpenCV_VERSION}, Viz3d, darknet_ros, depth percpetion demo will be compiled")

    add_executable(stereo_vision_object_depth_perception_node
                   stereo_vision_object_depth_perception_node.cpp
                  )
    target_link_libraries(stereo_vision_object_depth_perception_node
            ${catkin_LIBRARIES}
            ${PROJECT_NAME}
            ${OpenCV_LIBS}
            )
    add_dependencies(stereo_vision_object_depth_perception_node dji_sdk_generate_messages_cpp)
else()
    message(STATUS "Did not find required libraries, stereo vision object depth perception sample will not be compiled.")
 endif ()

