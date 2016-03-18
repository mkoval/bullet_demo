cmake_minimum_required(VERSION 2.8.11)
project(bullet_demo)

list(INSERT CMAKE_MODULE_PATH 0 "${PROJECT_SOURCE_DIR}/cmake") 
list(APPEND CMAKE_CXX_FLAGS "-std=c++11")

set(INCLUDE_INSTALL_DIR include)
set(LIBRARY_INSTALL_DIR lib)
set(CONFIG_INSTALL_DIR "${LIBRARY_INSTALL_DIR}/${PROJECT_NAME}/cmake")

# Bullet
find_package(Bullet REQUIRED CONFIG)
include_directories(SYSTEM "${BULLET_ROOT_DIR}/${BULLET_INCLUDE_DIR}")
link_directories("${BULLET_ROOT_DIR}/${BULLET_LIBRARY_DIRS}")
add_definitions(${BULLET_DEFINITIONS})

if ("${BULLET_VERSION_STRING}" VERSION_LESS "2.83")
  message(FATAL_ERROR "Bullet version 2.83.7 or great required; found"
                      " ${BULLET_VERSION_STRING}.")
endif ()

# OpenCL, install nvidia-opencl-dev on Ubuntu 14.04 with an NVidia GPU.
find_package(OpenCL REQUIRED)
include_directories(${OpenCL_INCLUDE_DIRS})

message(STATUS "OpenCL_INCLUDE_DIRS = ${OpenCL_INCLUDE_DIRS}")

add_executable("${PROJECT_NAME}"
  src/bullet_demo.cpp
)
target_link_libraries("${PROJECT_NAME}"
  ${BULLET_LIBRARIES}
  ${OpenCL_LIBRARIES}
)

install(TARGETS "${PROJECT_NAME}"
  DESTINATION bin
)
