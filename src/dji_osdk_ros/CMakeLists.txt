add_subdirectory(modules)

FILE(GLOB EXAMPLES *.cc *.c *.cpp)
find_package( OpenCV QUIET )
if (OpenCV_FOUND)
  add_definitions(-DOPEN_CV_INSTALLED)
endif ()

FOREACH(EXAMPLE ${EXAMPLES})
  GET_FILENAME_COMPONENT(MY_TARGET "${EXAMPLE}" NAME_WE)
  ADD_EXECUTABLE(${MY_TARGET} ${EXAMPLE})

  ## Add cmake target dependencies of the executable/library
  ## as an example, message headers may need to be generated before nodes
  add_dependencies(${MY_TARGET}
          dji_osdk_ros_generate_messages_cpp)

  TARGET_LINK_LIBRARIES(${MY_TARGET}
                        ${PROJECT_NAME}
                        )

  INSTALL(TARGETS ${MY_TARGET}
          ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
          LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
          RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
          )
ENDFOREACH()
