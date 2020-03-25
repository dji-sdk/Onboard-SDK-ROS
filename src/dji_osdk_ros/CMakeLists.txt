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
		      djiosdk-core.a
                      )
SET(ADVANCED_SENSING)
SET(WAYPT2_CORE)
#if(ADVANCED_SENSING)
	TARGET_LINK_LIBRARIES(${PROJECT_NAME}
                      ${catkin_LIBRARIES}
		      advanced-sensing.a
		      ${LIBUSB_1_LIBRARIES}
		      ${FFMPEG_LIBRARIES}
                      )
#endif()

#if(WAYPT2_CORE)
	TARGET_LINK_LIBRARIES(${PROJECT_NAME}
                      ${catkin_LIBRARIES}
		      waypointv2-interface.a
		      waypointv2-core.a
		      DJIProtobuf.a
		      djisdk-common.a
                      )
#endif()

