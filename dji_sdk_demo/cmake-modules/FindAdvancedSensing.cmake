# Once done this will define
#
#  FOUND_ADVANCED_SENSING - system has advanced-sensing

if (NOT FOUND_ADVANCED_SENSING)

    find_path(ADVANCED_SENSING_LIB
            NAMES
            libadvanced-sensing.a
            PATHS
            /usr/lib
            /usr/local/lib
            /opt/local/lib
            /sw/lib
            PATH_SUFFIXES
            lib
            )

    if(NOT ADVANCED_SENSING_LIB STREQUAL ADVANCED_SENSING_LIB-NOTFOUND)
        message(STATUS "Found Advanced Sensing in the system")
        set(FOUND_ADVANCED_SENSING TRUE)
    endif()

endif (NOT FOUND_ADVANCED_SENSING)