FILE(GLOB EXAMPLES *.cc *.c *.cpp)

include_directories(stereo_utility)

if(ENABLE_OPEN_CV_CONTRIB)
add_definitions(-DUSE_OPEN_CV_CONTRIB)
endif()

if(ENABLE_OPEN_CV_INSTALLED OR OPEN_CV_3_3_0_INSTALLED)
add_definitions(-DOPEN_CV_INSTALLED)
endif()

if(ENABLE_GPU)
add_definitions(-DUSE_GPU)
endif()

if(ENABLE_DARKNET_ROS)
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

find_package(SDL2 REQUIRED)
if (SDL2_FOUND)
    message(STATUS "Found SDL2:")
    message(STATUS " - Includes: ${SDL2_INCLUDE_DIRS}")
    message(STATUS " - Libraries: ${SDL2_LIBRARIES}")
    add_definitions(-DSDL2_INSTALLED)
    set(SDL2_LIB_DEPEND_FLAG -lSDL2)
else ()
message(STATUS "--Recommendation : It is found that using \"cv::imshow\" on some platforms will cause more
                CPU resources and more processing time in camera_stream_node. Using SDL to display images can improve this
                situation. \n--Install SDL2 library in shell : \"sudo apt-get install libsdl2-dev\".")
endif()
if(ENABLE_OPEN_CV_INSTALLED OR OPEN_CV_3_3_0_INSTALLED)
message(STATUS "Find OPENCV, camera stream node will be compiled.")
add_executable(camera_stream_node camera_stream_node.cpp)
target_link_libraries(camera_stream_node
        ${PROJECT_NAME}
        ${SDL2_LIB_DEPEND_FLAG}
        )
add_dependencies(camera_stream_node dji_osdk_ros_generate_messages_cpp)
else()
message(STATUS "Did not find OPENCV, camera stream node will not be compiled.")
endif()

add_executable(camera_h264_node camera_h264_node.cpp)
target_link_libraries(camera_h264_node
        ${PROJECT_NAME}
        )
add_dependencies(camera_h264_node dji_osdk_ros_generate_messages_cpp)

add_executable(mission_node mission_node.cpp)
target_link_libraries(mission_node
        ${PROJECT_NAME}
        )
add_dependencies(mission_node dji_osdk_ros_generate_messages_cpp)

add_executable(waypointV2_node waypointV2_node.cpp)
target_link_libraries(waypointV2_node
        ${PROJECT_NAME}
        )
add_dependencies(waypointV2_node dji_osdk_ros_generate_messages_cpp)

add_executable(battery_node battery_node.cpp)
target_link_libraries(battery_node
        ${PROJECT_NAME}
        )
add_dependencies(battery_node dji_osdk_ros_generate_messages_cpp)

add_executable(hms_node hms_node.cpp)
target_link_libraries(hms_node
        ${PROJECT_NAME}
        )
add_dependencies(hms_node dji_osdk_ros_generate_messages_cpp)

if (OPEN_CV_3_3_0_INSTALLED AND FOUND_OPENCV_VIZ)

    message(STATUS "Found OpenCV ${OpenCV_VERSION}, Viz3d, and advanced sensing module, stereo vision depth perception node will be compiled")
    add_definitions(-DOPEN_CV_INSTALLED)
    add_executable(stereo_vision_depth_perception_node
                   stereo_vision_depth_perception_node.cpp)
    target_include_directories(${PROJECT_NAME} PRIVATE ${OpenCV_INCLUDE_DIRS})

    if (OPEN_CV_3_3_0_INSTALLED)
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
   message(STATUS "Did not find required libraries, stereo vision depth perception node will not be compiled.")
endif ()

