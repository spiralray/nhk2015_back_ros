cmake_minimum_required(VERSION 2.8.3)
project(wiimote_orientation)

option(USE_OPENCL_REGISTRATION "Use OpenCL implementation of the depth registration" ON)
option(USE_OPENCV_OPENCL       "Use OpenCV OpenCL functions for rectification"       OFF)
set(MIN_WORKER_THREADS 2 CACHE STRING "Minimum number of worker threads")

set(CMAKE_BUILD_TYPE Release)
#set(CMAKE_BUILD_TYPE RelWithDebInfo)
#set(CMAKE_BUILD_TYPE Debug)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x -std=c++11 -DBT_USE_DOUBLE_PRECISION -Wall")
# Unused warnings
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wuninitialized -Winit-self -Wunused-function -Wunused-label -Wunused-variable -Wunused-but-set-variable -Wunused-but-set-parameter")
# Additional warnings
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Warray-bounds -Wtype-limits -Wreturn-type -Wsequence-point -Wparentheses -Wmissing-braces -Wchar-subscripts -Wswitch -Wwrite-strings -Wenum-compare -Wempty-body -Wlogical-op")

find_package(catkin REQUIRED COMPONENTS roscpp rostime std_msgs sensor_msgs )
find_package(cmake_modules QUIET)

## System dependencies are found with CMake's conventions
find_package(OpenCV REQUIRED)
find_package(OpenMP)

if(OPENMP_FOUND)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
endif()

################################################
## Declare ROS messages, services and actions ##
################################################

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
  INCLUDE_DIRS include
)

###########
## Build ##
###########

# Display additional files in qtcreator
execute_process(COMMAND find ${PROJECT_SOURCE_DIR} -type f OUTPUT_VARIABLE FILES_STRING)
string(REPLACE "\n" ";" FILES_LIST ${FILES_STRING})
add_custom_target(additional_files_${PROJECT_NAME}
  SOURCES
  ${FILES_LIST}
)

include_directories(include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIR}
)

if(USE_OPENCL_REGISTRATION)
  add_definitions(-DUSE_OPENCL_REGISTRATION)
endif()
if(USE_OPENCV_OPENCL)
  add_definitions(-USE_OPENCV_OPENCL)
endif()
add_definitions(-DMIN_WORKER_THREADS=${MIN_WORKER_THREADS})
add_definitions(-DK2_CALIB_PATH="${PROJECT_SOURCE_DIR}/data/")

add_executable(wiimote_orientation src/main.cpp)
target_link_libraries(wiimote_orientation
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
)
