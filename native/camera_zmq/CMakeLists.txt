cmake_minimum_required(VERSION 3.16)
project(ammr_camera_zmq LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(OpenCV REQUIRED COMPONENTS core imgcodecs imgproc)
find_package(realsense2 REQUIRED)
find_package(PkgConfig REQUIRED)
pkg_check_modules(ZMQ REQUIRED libzmq)

find_path(ORBBEC_INCLUDE_DIR libobsensor/ObSensor.hpp
  HINTS
    $ENV{ORBBEC_SDK_ROOT}/include
    /home/lin/agilex_ws/install/orbbec_camera/include
    /usr/local/include)
find_library(ORBBEC_LIBRARY NAMES OrbbecSDK
  HINTS
    $ENV{ORBBEC_SDK_ROOT}/lib
    /home/lin/agilex_ws/install/orbbec_camera/lib
    /usr/local/lib)

if(NOT ORBBEC_INCLUDE_DIR OR NOT ORBBEC_LIBRARY)
  message(FATAL_ERROR "OrbbecSDK not found; set ORBBEC_SDK_ROOT")
endif()

add_executable(ammr_camera_zmq_publisher camera_zmq_publisher.cpp)
target_include_directories(ammr_camera_zmq_publisher PRIVATE
  ${ORBBEC_INCLUDE_DIR} ${OpenCV_INCLUDE_DIRS} ${ZMQ_INCLUDE_DIRS})
target_link_libraries(ammr_camera_zmq_publisher PRIVATE
  ${ORBBEC_LIBRARY} realsense2::realsense2 ${OpenCV_LIBS} ${ZMQ_LIBRARIES})
target_compile_options(ammr_camera_zmq_publisher PRIVATE -Wall -Wextra -Wpedantic)

install(TARGETS ammr_camera_zmq_publisher RUNTIME DESTINATION bin)

