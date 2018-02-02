cmake_minimum_required(VERSION 2.8.3)
project(dji_sdk_demo)

# Add module path for <name>.cmake conf files
set(CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake-modules)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
        roscpp
        geometry_msgs
        dji_sdk
        image_transport # for message_filter
        )

find_package(DJIOSDK REQUIRED)

find_package(darknet_ros QUIET)
if(darknet_ros_FOUND)
    message(STATUS "Found darknet_ros package, will use it for object depth perception demo.")
    add_definitions(-DUSE_DARKNET_ROS)
endif()

# Try to see if user has OpenCV installed
# If there's version 3.3.0+, depth perception samples will be compiled
# If it's not version 3.3.0+, the sample will not be compiled
find_package(OpenCV 3.3.0 QUIET)
if (OpenCV_FOUND)
    message(STATUS "Found OpenCV ${OpenCV_VERSION} installed in the system, will use it for depth perception sample")
    message(STATUS " - Includes: ${OpenCV_INCLUDE_DIRS}")
    add_definitions(-DOPEN_CV_INSTALLED)
    set(OPEN_CV_3_3_0_INSTALLED TRUE)

    find_package(OpenCVContribImgProc QUIET)

    find_package(OpenCVViz QUIET)
else()
    find_package(OpenCV QUIET )
    if (OpenCV_FOUND)
        message(STATUS "Found OpenCV ${OpenCV_VERSION} installed in the system, depth percpetion demo requires 3.3.0+ for better experience")
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

find_package(AdvancedSensing QUIET)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)

set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")

## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################

## To declare and build messages, services or actions from within this
## package, follow these steps:
## * Let MSG_DEP_SET be the set of packages whose message types you use in
##   your messages/services/actions (e.g. std_msgs, actionlib_msgs, ...).
## * In the file package.xml:
##   * add a build_depend tag for "message_generation"
##   * add a build_depend and a run_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependency has been pulled in
##     but can be declared for certainty nonetheless:
##     * add a run_depend tag for "message_runtime"
## * In this file (CMakeLists.txt):
##   * add "message_generation" and every package in MSG_DEP_SET to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * add "message_runtime" and every package in MSG_DEP_SET to
##     catkin_package(CATKIN_DEPENDS ...)
##   * uncomment the add_*_files sections below as needed
##     and list every .msg/.srv/.action file to be processed
##   * uncomment the generate_messages entry below
##   * add every package in MSG_DEP_SET to generate_messages(DEPENDENCIES ...)

## Generate messages in the 'msg' folder
# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
# generate_messages(
#   DEPENDENCIES
#   std_msgs  # Or other packages containing msgs
# )

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

## To declare and build dynamic reconfigure parameters within this
## package, follow these steps:
## * In the file package.xml:
##   * add a build_depend and a run_depend tag for "dynamic_reconfigure"
## * In this file (CMakeLists.txt):
##   * add "dynamic_reconfigure" to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * uncomment the "generate_dynamic_reconfigure_options" section below
##     and list every .cfg file to be processed

## Generate dynamic reconfigure parameters in the 'cfg' folder
# generate_dynamic_reconfigure_options(
#   cfg/DynReconf1.cfg
#   cfg/DynReconf2.cfg
# )

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
        #  INCLUDE_DIRS include
        #  LIBRARIES dji_sdk_demo
        #  CATKIN_DEPENDS dji_sdk roscpp
        #  DEPENDS dji_sdk
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
        include
        ${catkin_INCLUDE_DIRS}
)

## Declare a C++ library
# add_library(dji_sdk_demo
#   src/${PROJECT_NAME}/dji_sdk_demo.cpp
# )

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(dji_sdk_demo ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable
# add_executable(dji_sdk_demo_node src/dji_sdk_demo_node.cpp)

## Add cmake target dependencies of the executable
## same as for the library above
# add_dependencies(dji_sdk_demo_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
# target_link_libraries(dji_sdk_demo_node
#   ${catkin_LIBRARIES}
# )

add_executable(demo_mission
        src/demo_mission.cpp)
target_link_libraries(demo_mission
        ${catkin_LIBRARIES}
        ${DJIOSDK_LIBRARIES}
        )
add_dependencies(demo_mission dji_sdk_generate_messages_cpp)

add_executable(demo_camera_gimbal
        src/demo_camera_gimbal.cpp )
target_link_libraries(demo_camera_gimbal
        ${catkin_LIBRARIES}
        ${DJIOSDK_LIBRARIES}
        )
add_dependencies(demo_camera_gimbal dji_sdk_generate_messages_cpp)

add_executable(demo_mfio
        src/demo_mfio.cpp)
target_link_libraries(demo_mfio
        ${catkin_LIBRARIES}
        ${DJIOSDK_LIBRARIES}
        )
add_dependencies(demo_mfio dji_sdk_generate_messages_cpp)

add_executable(demo_flight_control
        src/demo_flight_control.cpp)

target_link_libraries(demo_flight_control
        ${catkin_LIBRARIES}
        ${DJIOSDK_LIBRARIES}
        )
add_dependencies(demo_flight_control dji_sdk_generate_messages_cpp)

add_executable(demo_mobile_comm
        src/demo_mobile_comm.cpp)

target_link_libraries(demo_mobile_comm
        ${catkin_LIBRARIES}
        ${DJIOSDK_LIBRARIES}
        )
add_dependencies(demo_mobile_comm dji_sdk_generate_messages_cpp)

add_executable(demo_local_position_control
        src/demo_local_position_control.cpp)
target_link_libraries(demo_local_position_control
        ${catkin_LIBRARIES}
        ${DJIOSDK_LIBRARIES}
        )
add_dependencies(demo_local_position_control dji_sdk_generate_messages_cpp)


if (OPEN_CV_3_3_0_INSTALLED AND FOUND_OPENCV_VIZ AND FOUND_ADVANCED_SENSING)
    message(STATUS "Found OpenCV ${OpenCV_VERSION}, Viz3d, and advanced sensing module, depth percpetion demo will be compiled")

    FILE(GLOB STEREO_UTILITY_SRC_FILE
            src/stereo_utility/camera_param.cpp
            src/stereo_utility/stereo_frame.cpp
            src/stereo_utility/config.cpp
            src/stereo_utility/point_cloud_viewer.cpp)

    add_executable(demo_stereo_depth_perception
            src/demo_stereo_depth_perception.cpp
            ${STEREO_UTILITY_SRC_FILE} )
    target_link_libraries(demo_stereo_depth_perception
            ${catkin_LIBRARIES}
            ${DJIOSDK_LIBRARIES}
            ${OpenCV_LIBS}
            )
    add_dependencies(demo_stereo_depth_perception dji_sdk_generate_messages_cpp)
endif()

if (OPEN_CV_3_3_0_INSTALLED AND FOUND_OPENCV_VIZ AND darknet_ros_FOUND AND FOUND_ADVANCED_SENSING)
    message(STATUS "Found OpenCV ${OpenCV_VERSION}, Viz3d, darknet_ros, and advanced sensing module, depth percpetion demo will be compiled")

    add_executable(demo_stereo_object_depth_perception
            src/demo_stereo_object_depth_perception.cpp
            ${STEREO_UTILITY_SRC_FILE} )
    target_link_libraries(demo_stereo_object_depth_perception
            ${catkin_LIBRARIES}
            ${DJIOSDK_LIBRARIES}
            ${OpenCV_LIBS}
            )
    add_dependencies(demo_stereo_object_depth_perception dji_sdk_generate_messages_cpp)
endif()

#############
## Install ##
#############


# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
#catkin_install_python(PROGRAMS
#   src/client.py
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
#)

## Mark cpp header files for installation

install(DIRECTORY launch
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
        )
## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_dji_sdk_demo.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
