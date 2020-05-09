FILE(GLOB_RECURSE HEADERS
     ../../include/*/*.h*
     ../../include/*.h*
     )
FILE(GLOB_RECURSE SOURCES *.cc *.cpp *.c *.cxx)


ADD_LIBRARY(${PROJECT_NAME} SHARED ${HEADERS} ${SOURCES})
MESSAGE(STATUS "PROJECT_NAME: ${PROJECT_NAME}")
MESSAGE(STATUS "DJIOSDK_LIB: ${DJIOSDK_LIBRARIES}")
TARGET_LINK_LIBRARIES(${PROJECT_NAME}
                      ${catkin_LIBRARIES}
                      #${DJIOSDK_LIBRARIES}
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

# Try to see if user has OpenCV installed
# if yes, default callback will display the image
find_package( OpenCV QUIET )
if (OpenCV_FOUND)
	message( "\n${PROJECT_NAME}...")
	message( STATUS "Found OpenCV installed in the system, will use it to display image in AdvancedSensing APIs")
	message( STATUS " - Includes: ${OpenCV_INCLUDE_DIRS}")
	message( STATUS " - Libraries: ${OpenCV_LIBRARIES}")
	add_definitions(-DOPEN_CV_INSTALLED)
else()
	message( STATUS "Did not find OpenCV in the system, image data is inside RecvContainer as raw data")
endif ()

target_include_directories(${PROJECT_NAME} PRIVATE ${OpenCV_INCLUDE_DIRS})

if (OpenCV_FOUND)
	target_link_libraries(${PROJECT_NAME} ${OpenCV_LIBS})
endif ()

