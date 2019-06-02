
# Download,compile and install djiosdk-core 
include(ExternalProject)
# TODO VERSION may be changed
set(VERSION "3.8.1")
set(EXTERNAL_SOURCE_NAME Onboard-SDK)
set(EXTERNAL_SOURCE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/../${EXTERNAL_SOURCE_NAME}-${VERSION})
set(EXTERNAL_BUILD_PATH  ${EXTERNAL_SOURCE_PATH}/../${EXTERNAL_SOURCE_NAME}-${VERSION}-build)

message(STATUS "${EXTERNAL_SOURCE_NAME} ${VERSION} path is ${EXTERNAL_SOURCE_PATH}")
# Set appropriate branch name
set(BRANCH_NAME ${VERSION})
ExternalProject_Add (
  ${EXTERNAL_SOURCE_NAME}
  GIT_REPOSITORY https://github.com/dji-sdk/Onboard-SDK.git
  GIT_TAG ${BRANCH_NAME}
  SOURCE_DIR  ${EXTERNAL_SOURCE_PATH}
  BINARY_DIR  ${EXTERNAL_BUILD_PATH}
  CMAKE_COMMAND cd ${EXTERNAL_BUILD_PATH} && cmake ${EXTERNAL_SOURCE_PATH} -DADVANCED_SENSING=ON
  BUILD_COMMAND   cd ${EXTERNAL_BUILD_PATH} &&  make djiosdk-core
  INSTALL_COMMAND cd ${EXTERNAL_BUILD_PATH} && sudo make install djiosdk-core
  )
ExternalProject_Get_Property(${EXTERNAL_SOURCE_NAME} source_dir)

