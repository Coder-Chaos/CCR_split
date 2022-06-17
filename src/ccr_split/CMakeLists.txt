cmake_minimum_required(VERSION 3.21)

if(COMMAND cmake_policy)
cmake_policy(SET CMP0003 NEW)
endif(COMMAND cmake_policy)

if (UNIX)
    find_package(PkgConfig QUIET)
    pkg_check_modules(GSTREAMER QUIET gstreamer-1.0)
    MESSAGE(STATUS "gstreamer dirs:" ${GSTREAMER_INCLUDE_DIRS})
    MESSAGE(STATUS "gstreamer lib:" ${GSTREAMER_LIBRARIES})
    include_directories(${GSTREAMER_INCLUDE_DIRS})
    link_directories(${GSTREAMER_LIBRARY_DIRS})
    #target_link_libraries(main ${GSTREAMER_LIBRARIES})
endif ()

list( APPEND CMAKE_MODULE_PATH /usr/share/cmake-3.10/Modules ) 
#set (CMAKE_PREFIX_PATH "D:\\Qt\\Qt5.10.0\\5.10.0\\msvc2015\\")
#find_package(GStreamer REQUIRED)
#find_package(Do3Think REQUIRED)
#find_package(Common REQUIRED)
#find_package(Utility REQUIRED)
#find_package(FreeModbus REQUIRED)
#include(OpenCV)
find_package(OpenCV REQUIRED)

SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -pthread")

set(COMMON_INCLUDE_DIRS ${PROJECT_SOURCE_DIR}/src/common
                        ${PROJECT_SOURCE_DIR}/src/common/log
                        ${PROJECT_SOURCE_DIR}/src/common/net
                        ${PROJECT_SOURCE_DIR}/src/common/image
                        ${PROJECT_SOURCE_DIR}/src/common/udmabuf
                        ${PROJECT_SOURCE_DIR}/src/common/matrix
                        ${PROJECT_SOURCE_DIR}/src/common/mathlib
                        #${PROJECT_SOURCE_DIR}/src/common/controllib
    )
set(UTILITY_INCLUDE_DIRS ${PROJECT_SOURCE_DIR}/src/utility)

# freemodbus sources
set(FREEMODBUS_DIRS ${PROJECT_SOURCE_DIR}/lib/freemodbus)
set(FREEMODBUS_SOURCES  ${FREEMODBUS_DIRS}/LINUXTCP/port/portevent.c
                        ${FREEMODBUS_DIRS}/LINUXTCP/port/portother.c
                        ${FREEMODBUS_DIRS}/LINUXTCP/port/porttcp.c
                        ${FREEMODBUS_DIRS}/modbus/mb.c
                        ${FREEMODBUS_DIRS}/modbus/tcp/mbtcp.c
                        ${FREEMODBUS_DIRS}/modbus/functions/mbfunccoils.c
                        ${FREEMODBUS_DIRS}/modbus/functions/mbfuncdiag.c
                        ${FREEMODBUS_DIRS}/modbus/functions/mbfuncdisc.c
                        ${FREEMODBUS_DIRS}/modbus/functions/mbfuncholding.c
                        ${FREEMODBUS_DIRS}/modbus/functions/mbfuncinput.c
                        ${FREEMODBUS_DIRS}/modbus/functions/mbfuncother.c
                        ${FREEMODBUS_DIRS}/modbus/functions/mbutils.c
)
# add sources to lib
add_library(FREEMODBUS_LIB ${FREEMODBUS_SOURCES})
# add include
set(FREEMODBUS_INCLUDE_DIRS ${FREEMODBUS_DIRS}/modbus/include
                            ${FREEMODBUS_DIRS}/modbus/tcp
                            ${FREEMODBUS_DIRS}/LINUXTCP/port    
    )

# Do3Think sources
set(Do3Think_DIRS ${PROJECT_SOURCE_DIR}/lib/dvp)
set(Do3Think_SOURCES    ${Do3Think_DIRS}/Demo/Demo.cpp)
# add sources to lib
add_library(DO3THINK_LIBRARIES ${Do3Think_SOURCES})
set(DO3THINK_LIBRARIES  ${Do3Think_DIRS}/libdvp.so
                        ${Do3Think_DIRS}/libhzd.so)
# add include
set(DO3THINK_INCLUDE_DIRS ${Do3Think_DIRS}/Demo)

set(I2CSMBUS_LIB /usr/lib/aarch64-linux-gnu/libi2c.so)

# set header file
include_directories(
    ${CMAKE_CURRENT_SOURCE_DIR}/robot/vision_detection/do3think
    ${CMAKE_CURRENT_SOURCE_DIR}/robot/vision_detection/gstreamer
    ${COMMON_INCLUDE_DIRS}
    ${UTILITY_INCLUDE_DIRS}
    ${FREEMODBUS_INCLUDE_DIRS}
    ${GSTREAMER_INCLUDE_DIRS}
    ${DO3THINK_INCLUDE_DIRS}
    #${PROJECT_SOURCE_DIR}/src/common
    #${PROJECT_SOURCE_DIR}/utility
    ${PROJECT_SOURCE_DIR}/src/can
    ${PROJECT_SOURCE_DIR}/src/tfminip
    ${PROJECT_SOURCE_DIR}/src/shared_mem
    ${PROJECT_SOURCE_DIR}/src/uvc
    ${PROJECT_SOURCE_DIR}/src/udp
    ${PROJECT_SOURCE_DIR}/src/tcp_can
    ${PROJECT_SOURCE_DIR}/src/tcp_pos
    ${PROJECT_SOURCE_DIR}/src/cuavpm
    #${CMAKE_CURRENT_SOURCE_DIR}/robot/motion/tcp_can
    ${PROJECT_SOURCE_DIR}/src/freemodbus_tcp
    ${CMAKE_CURRENT_SOURCE_DIR}/robot/motion/tmotor
    ${CMAKE_CURRENT_SOURCE_DIR}/robot/motion/maxon
    ${CMAKE_CURRENT_SOURCE_DIR}/robot/motion
    ${CMAKE_CURRENT_SOURCE_DIR}/robot
    ${CMAKE_CURRENT_SOURCE_DIR}
    )

# vision
set(vision_detection_src robot/vision_detection/vision_detection.cc 
                        robot/vision_detection/gstreamer/gst_appsrc.cc
                        robot/vision_detection/do3think/camera.cc
                        robot/vision_detection/do3think/do3think.cc
                        ${COMMON_INCLUDE_DIRS}
                        ${PROJECT_SOURCE_DIR}/src/common/common.cc
                        ${PROJECT_SOURCE_DIR}/src/common/log/log.c
                        ${PROJECT_SOURCE_DIR}/src/common/net/net.cc
                        ${PROJECT_SOURCE_DIR}/src/common/udmabuf/udmabuf.cc
                        ${PROJECT_SOURCE_DIR}/src/utility/utility.cc
                        ${UTILITY_INCLUDE_DIRS}
                        ${FREEMODBUS_INCLUDE_DIRS}
                        ${GSTREAMER_INCLUDE_DIRS}
                        ${PROJECT_SOURCE_DIR}/src/shared_mem/shared_mem.c
                        #${PROJECT_SOURCE_DIR}/hardware/hls/vitis_2020.2/stream_preprocess/ip/stream_preprocess/drivers/stream_preprocess_v1_0/src/xstream_preprocess.c
                        #${PROJECT_SOURCE_DIR}/hardware/hls/vitis_2020.2/stream_preprocess/ip/stream_preprocess/drivers/stream_preprocess_v1_0/src/xstream_preprocess_sinit.c
                        #${PROJECT_SOURCE_DIR}/hardware/hls/vitis_2020.2/stream_preprocess/ip/stream_preprocess/drivers/stream_preprocess_v1_0/src/xstream_preprocess_linux.c
)

add_executable(vision_detection ${vision_detection_src})

target_include_directories(vision_detection PUBLIC 
                                                    ${GSTREAMER_INCLUDE_DIRS}  
                                                    ${RESIZE_ACCEL_INCLUDE_DIRS}
                                                    ${PROJECT_SOURCE_DIR}/hardware/hls/vitis_2020.2/stream_preprocess/ip/stream_preprocess/drivers/stream_preprocess_v1_0/src)
target_link_libraries(vision_detection PUBLIC ${OpenCV_LIBS} ${GSTREAMER_LIBRARIES} ${DO3THINK_LIBRARIES} -lpthread -latomic -lrt)

# core
set(ccr_core_src ccr_core.cc
                        ${PROJECT_SOURCE_DIR}/src/uvc/uvc.cc
                        ${PROJECT_SOURCE_DIR}/src/shared_mem/shared_mem.c
                        ${PROJECT_SOURCE_DIR}/src/udp/udp_send.cc
                        ${PROJECT_SOURCE_DIR}/src/can/can.cc
                        ${PROJECT_SOURCE_DIR}/src/tcp_can/tcp_client.cc
                        ${PROJECT_SOURCE_DIR}/src/tcp_pos/tcp_pos.cc
                        ${PROJECT_SOURCE_DIR}/src/tcp_pos/sensors/gps.cc
                        ${PROJECT_SOURCE_DIR}/src/cuavpm/cuavpm.cc
                        #${CMAKE_CURRENT_SOURCE_DIR}/robot/motion/tcp_can/tcp_client.cc
                        ${PROJECT_SOURCE_DIR}/src/tfminip/tfminip.cc
                        ${PROJECT_SOURCE_DIR}/src/freemodbus_tcp/freemodbus_tcp.c
                        ${FREEMODBUS_INCLUDE_DIRS}
                        ${FREEMODBUS_SOURCES}    
                        ${PROJECT_SOURCE_DIR}/src/common/common.cc
                        ${PROJECT_SOURCE_DIR}/src/common/log/log.c
                        ${PROJECT_SOURCE_DIR}/src/common/net/net.cc
                        ${PROJECT_SOURCE_DIR}/src/utility/utility.cc
                        ${CMAKE_CURRENT_SOURCE_DIR}/robot/motion/tmotor/tmotor.cc
                        ${CMAKE_CURRENT_SOURCE_DIR}/robot/motion/maxon/maxon.cc
                        ${CMAKE_CURRENT_SOURCE_DIR}/robot/motion/motion.cc
                        ${CMAKE_CURRENT_SOURCE_DIR}/robot/robot.cc
                        ${PROJECT_SOURCE_DIR}/src/common/mathlib/math/filter/LowPassFilter2p.cpp
)

add_executable(ccr_core ${ccr_core_src})
# The POSIX IPC functions are part of the POSIX real-time extensions, and so you need to link with librt
target_link_libraries(ccr_core PUBLIC ${I2CSMBUS_LIB} -lrt)