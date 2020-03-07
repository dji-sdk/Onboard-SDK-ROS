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
                      ${DJIOSDK_LIBRARIES}
                      )

