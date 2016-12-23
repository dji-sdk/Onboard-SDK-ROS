include(ExternalProject)
message( "External project - GoogleTest" )

if(GTEST)
  if (CMAKE_SYSTEM_NAME MATCHES Linux)
    find_package(Threads)
    ExternalProject_Add(
      googletest
      # Disable update step
      UPDATE_COMMAND ""
      GIT_REPOSITORY https://github.com/google/googletest.git
      CMAKE_ARGS -DCMAKE_ARCHIVE_OUTPUT_DIRECTORY_DEBUG:PATH=DebugLibs
        -DCMAKE_ARCHIVE_OUTPUT_DIRECTORY_RELEASE:PATH=ReleaseLibs
        -DCMAKE_CXX_FLAGS=${MSVC_COMPILER_DEFS}
        -Dgtest_force_shared_crt=${GTEST_FORCE_SHARED_CRT}
        -Dgtest_disable_pthreads=${GTEST_DISABLE_PTHREADS}
        -DCMAKE_INSTALL_PREFIX=${CMAKE_CURRENT_BINARY_DIR}/../../../devel)
  endif ()
endif ()

